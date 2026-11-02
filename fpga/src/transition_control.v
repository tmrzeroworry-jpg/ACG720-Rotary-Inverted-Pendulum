// TR05 offline candidate. Energy swing-up + actively damped return, not S0.
module transition_control #(
 parameter integer UP_PWM=150,DOWN_PWM=150,SLEW=3,TIMEOUT_MS=8000
)(input wire clk,rst_n,tick,fast_start,return_start,cancel,fresh,seam_wait,seam_bridge,
 input wire[7:0] sensor_fault,input wire[3:0] ctrl_state,
 input wire[15:0] angle,up,default_down,
 input wire signed[31:0] arm_position,arm_velocity,input wire signed[9:0] seed_command,
 output wire busy,return_busy,output reg takeover,capture,parked,abort,
 output reg[7:0] phase,result,turns,output reg mode,
 output reg[15:0] ident,elapsed,down_ref,output reg down_valid,
 output reg signed[9:0] command,
 output wire signed[31:0] rate_diag,goal_diag,energy_diag);
 //0 idle,1kick up,2energy,3catch/verify,4return prepare,5leave upright,
 //6active descent damping,7bottom feedback,8release verify,9up done,10down done,11failed.
 assign busy=phase>=1 && phase<=8;assign return_busy=busy && mode;
 reg signed[31:0] previous,position,rate,goal,kinetic,potential;
 reg signed[15:0]cos_up,cos_down;
 reg signed[31:0] h0,h1,h2,h3,h4,h5,h6,h7,origin,anchor,turn_origin;
 reg direction,desc_negative;reg[3:0] recover;reg[15:0] stable,fast_ms,phase_ms;
 reg[3:0] calc;reg signed[31:0] square,error_energy;
 reg signed[9:0]pump;reg signed[25:0]product;
 reg[9:0] rest_n;reg[31:0] rest_sum;reg[15:0] rest_min,rest_max;
 reg signed[31:0] du,dd,delta,newpos,idx,ud,vd,tmp,target,limit;
 assign rate_diag=rate;assign goal_diag=goal;assign energy_diag=error_energy;
 function signed[31:0] abs32;input signed[31:0]x;begin abs32=x<0?-x:x;end endfunction
 function signed[31:0] wrap;input signed[31:0]x;begin if(x>13200)wrap=x-26400;else if(x< -13200)wrap=x+26400;else wrap=x;end endfunction
 function signed[15:0] cosine;input[5:0]i;begin case(i)
    6'd0:cosine=16'sd32767;
    6'd1:cosine=16'sd32609;
    6'd2:cosine=16'sd32137;
    6'd3:cosine=16'sd31356;
    6'd4:cosine=16'sd30273;
    6'd5:cosine=16'sd28898;
    6'd6:cosine=16'sd27245;
    6'd7:cosine=16'sd25329;
    6'd8:cosine=16'sd23170;
    6'd9:cosine=16'sd20787;
    6'd10:cosine=16'sd18204;
    6'd11:cosine=16'sd15446;
    6'd12:cosine=16'sd12539;
    6'd13:cosine=16'sd9512;
    6'd14:cosine=16'sd6393;
    6'd15:cosine=16'sd3212;
    6'd16:cosine=16'sd0;
    6'd17:cosine=-16'sd3212;
    6'd18:cosine=-16'sd6393;
    6'd19:cosine=-16'sd9512;
    6'd20:cosine=-16'sd12539;
    6'd21:cosine=-16'sd15446;
    6'd22:cosine=-16'sd18204;
    6'd23:cosine=-16'sd20787;
    6'd24:cosine=-16'sd23170;
    6'd25:cosine=-16'sd25329;
    6'd26:cosine=-16'sd27245;
    6'd27:cosine=-16'sd28898;
    6'd28:cosine=-16'sd30273;
    6'd29:cosine=-16'sd31356;
    6'd30:cosine=-16'sd32137;
    6'd31:cosine=-16'sd32609;
    6'd32:cosine=-16'sd32767;
    default:cosine=-16'sd32767;
 endcase end endfunction

 // Decode wrapped errors in a separate 50 MHz register stage. This removes
 // sensor selection + wrap + multiplier/feedback from one long clock path.
 // One fabric-clock latency (20 ns), no extra 1 ms control-sample delay.
 reg signed[16:0] up_error_q,down_error_q;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin up_error_q<=0;down_error_q<=0;end
  else begin
   up_error_q<=wrap($signed({1'b0,angle})-$signed({1'b0,up}));
   down_error_q<=wrap($signed({1'b0,angle})-$signed({1'b0,down_ref}));
  end
 end
 task fail;input[7:0]why;begin phase<=11;result<=why;abort<=1;goal<=0;calc<=0;end endtask
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   phase<=0;result<=0;turns<=0;mode<=0;ident<=0;elapsed<=0;down_ref<=default_down;down_valid<=0;
   command<=0;goal<=0;rate<=0;position<=0;previous<=0;origin<=0;anchor<=0;turn_origin<=0;direction<=1;desc_negative<=0;
   h0<=0;h1<=0;h2<=0;h3<=0;h4<=0;h5<=0;h6<=0;h7<=0;recover<=0;
   stable<=0;fast_ms<=0;phase_ms<=0;takeover<=0;capture<=0;parked<=0;abort<=0;
   calc<=0;cos_up<=0;cos_down<=0;square<=0;pump<=0;product<=0;kinetic<=0;potential<=0;error_energy<=0;
   rest_n<=0;rest_sum<=0;rest_min<=65535;rest_max<=0;
  end else begin
   takeover<=0;capture<=0;parked<=0;abort<=0;
   // Learn only from a quiet hanging observation in READY. Never alter original control references.
   if(tick && !busy && ctrl_state==1)begin
    if(fresh && abs32(wrap($signed({1'b0,angle})-$signed({1'b0,default_down})))<1500 && abs32(arm_velocity)<40)begin
     if(angle<rest_min)rest_min<=angle;if(angle>rest_max)rest_max<=angle;
     if(rest_max>rest_min && rest_max-rest_min>120)begin rest_n<=0;rest_sum<=0;rest_min<=angle;rest_max<=angle;end
     else if(rest_n==1023)begin down_ref<=(rest_sum+angle)>>10;down_valid<=1;rest_n<=0;rest_sum<=0;rest_min<=angle;rest_max<=angle;end
     else begin rest_n<=rest_n+1'b1;rest_sum<=rest_sum+angle;end
    end else begin rest_n<=0;rest_sum<=0;rest_min<=65535;rest_max<=0;end
   end
   if((fast_start || return_start) && !busy)begin
    du=up_error_q;dd=down_error_q;
    if(fresh && down_valid && ((fast_start && ctrl_state==1 && abs32(dd)<2200 && abs32(arm_velocity)<100) ||
       (return_start && ctrl_state==4 && abs32(du)<1000 && abs32(arm_velocity)<600)))begin
     phase<=return_start ? 4:1;mode<=return_start;ident<=ident+1'b1;elapsed<=0;phase_ms<=0;result<=0;
     previous<=angle;position<=0;rate<=0;goal<=seed_command;command<=seed_command;
     origin<=arm_position;anchor<=arm_position;turn_origin<=0;direction<=1;desc_negative<=du<0;turns<=0;recover<=8;
     h0<=0;h1<=0;h2<=0;h3<=0;h4<=0;h5<=0;h6<=0;h7<=0;stable<=0;fast_ms<=0;calc<=0;
     if(fast_start)takeover<=1;
    end else begin result<=1;if(fast_start)begin phase<=11;abort<=1;end end
   end
   if(busy)begin
    case(calc)
     1:begin square<=($signed(rate)>>>4)*($signed(rate)>>>4);calc<=2;end
     2:begin kinetic<=($signed({1'b0,square})*64'sd33)>>>16;potential<=((32768-cos_down)*6600)>>>15;calc<=3;end
     3:begin
      du=abs32(up_error_q);
      target=13200+((du*23831)>>>16);error_energy<=target-kinetic-potential;calc<=4;
     end
     4:begin
      if(phase==2)begin
       tmp=(error_energy*5)>>>6;if(tmp>UP_PWM)tmp=UP_PWM;if(tmp< -UP_PWM)tmp=-UP_PWM;
       pump<=direction ? tmp:-tmp;
      end else if(phase==6)begin
       tmp=rate>>>7;if(tmp>DOWN_PWM)tmp=DOWN_PWM;if(tmp< -DOWN_PWM)tmp=-DOWN_PWM;pump<=tmp;
      end
      else pump<=0;
      calc<=5;
     end
     5:begin product<=pump*cos_up;calc<=6;end
     6:begin goal<=$signed(product)>>>15;calc<=0;end
    endcase
    if(tick)begin
     elapsed<=elapsed+1'b1;phase_ms<=phase_ms+1'b1;
     du=up_error_q;dd=down_error_q;
     delta=wrap($signed({1'b0,angle})-previous);previous<=angle;newpos=position+delta;position<=newpos;
     if(seam_wait || seam_bridge)recover<=8;else if(recover!=0)recover<=recover-1'b1;
     else rate<=((rate*3)+(newpos-h7)*125)>>>2;
     h7<=h6;h6<=h5;h5<=h4;h4<=h3;h3<=h2;h2<=h1;h1<=h0;h0<=newpos;
     idx=(abs32(dd)*81345)>>>25;if(idx>32)idx=32;
     cos_down<=$signed(cosine(idx[5:0]));cos_up<=-$signed(cosine(idx[5:0]));
     if(abs32(arm_velocity)>1800)fast_ms<=fast_ms+1'b1;else fast_ms<=0;
     if((rate>2500 && direction) || (rate< -2500 && !direction))begin
      if(abs32(newpos-turn_origin)>500)begin direction<=rate<0;turns<=turns+1'b1;turn_origin<=newpos;end
     end
     case(phase)
      1:begin
       tmp=phase_ms<80 ? phase_ms:80;goal<=-tmp;
       if(phase_ms>=120)begin phase<=2;phase_ms<=0;end
      end
      2:begin
       if(!seam_wait && recover==0)calc<=1;
       if(abs32(du)<2200 && abs32(rate)<20000 && phase_ms>120)begin capture<=1;phase<=3;phase_ms<=0;calc<=0;stable<=0;end
       if(turns>=6)fail(7);
      end
      3:begin
       if(ctrl_state==4 && abs32(du)<800)begin if(stable>=499)begin phase<=9;result<=0;end else stable<=stable+1'b1;end
       else if(ctrl_state==3 || ctrl_state==9)stable<=0;else fail(8);
      end
      4:begin
       if(ctrl_state!=4)fail(8);
       if(abs32(arm_velocity)<100 && abs32(du)<600)begin
        if(stable>=99)begin phase<=5;phase_ms<=0;takeover<=1;command<=seed_command;goal<=seed_command;stable<=0;end
        else stable<=stable+1'b1;
       end else stable<=0;
       if(phase_ms>=2000)fail(9);
      end
      5:begin
       tmp=phase_ms<90 ? phase_ms/2:45;goal<=desc_negative ? tmp:-tmp;
       if(abs32(du)>330 || phase_ms>=220)begin phase<=6;phase_ms<=0;end
      end
      6:begin
       if(!seam_wait && recover==0)calc<=1;
       if(abs32(dd)<5000 && abs32(rate)<92000)begin phase<=7;phase_ms<=0;anchor<=arm_position;calc<=0;end
      end
      7:begin
       // LQR-seeded four-state bottom damping; pole derivative gain increased after model checks.
       goal<=((($signed(arm_position)-anchor)*3)>>>7)+(($signed(arm_velocity)*11)>>>8)-((dd*3)>>>7)-((rate*10)>>>10);
       if(abs32(dd)<100 && abs32(rate)<1200 && abs32(arm_velocity)<30)begin
        if(stable>=399)begin phase<=8;phase_ms<=0;goal<=0;end else stable<=stable+1'b1;
       end else stable<=0;
      end
      8:begin
       goal<=0;
       if(abs32(dd)>150 || abs32(rate)>1800 || abs32(arm_velocity)>50)begin phase<=7;phase_ms<=0;stable<=0;end
       else if(phase_ms>=199)begin phase<=10;result<=0;parked<=1;end
      end
     endcase
     if(seam_wait)begin goal<=0;calc<=0;end
     limit=mode ? DOWN_PWM:UP_PWM;
     tmp=goal;if(tmp>limit)tmp=limit;if(tmp< -limit)tmp=-limit;
     if(tmp>$signed(command)+SLEW)command<=command+SLEW;
     else if(tmp<$signed(command)-SLEW)command<=command-SLEW;else command<=tmp[9:0];
     // Keep the specific primary cause even when the core has already cut drive.
     if(sensor_fault!=0)fail(4);
     else if(!fresh)fail(3);
     else if(abs32(delta)>500 && !seam_bridge)fail(4);
     else if(abs32(arm_position-origin)>(mode ? 1040:1800) || fast_ms>=100 || abs32(rate)>150000)fail(5);
     else if(mode && phase>=5 && ((desc_negative ? -newpos:newpos)< -1500 || (desc_negative ? -newpos:newpos)>20000))fail(6);
     else if(ctrl_state==5 || ctrl_state==6 || ctrl_state==7)fail(10);
     else if(elapsed>=TIMEOUT_MS-1)fail(2);
    end
    if(cancel)fail(11);
   end
  end
 end
endmodule
