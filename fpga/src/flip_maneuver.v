// TR02: single, direction-locked pendulum revolution. Not an arbitrary PWM API.
// Progress is unwrapped from fresh canonical ADC samples; no synthetic angle progression.
// E=6600*(1+cos(beta))+kinetic, kinetic derives from passive omega0^2~95/s^2.
// Fixed-point approximation: kinetic=((v_counts_s>>4)^2*33)>>16.
module flip_maneuver #(
 parameter integer FULL=26400,MAX_PWM=100,SLEW=2,TIMEOUT_MS=6000
)(input wire clk,rst_n,tick,start,cancel,negative,late,fresh,seam_wait,seam_bridge,input wire[7:0] sensor_fault,
 input wire[15:0] angle,up,input wire[3:0] ctrl_state,
 input wire signed[31:0] arm_position,arm_velocity,input wire signed[9:0] seed_command,
 output wire busy,output reg takeover,capture,abort,
 output reg[7:0] phase,result,output reg[15:0] ident,elapsed,
 output reg signed[31:0] progress,output reg signed[9:0] command,
 output wire signed[31:0] rate_diag,energy_diag,goal_diag);
 // 0 idle,1launch,2turn,3capture,4verify,5done,6failed.
 // result:1start gate,2timeout,3reverse,4ADC discontinuity,5overspeed/travel,
 // 6missed top,7capture failed,8cancelled,9sensor,10core protection.
 assign busy=phase>=1 && phase<=4;

 reg dir,late_latched;reg signed[31:0] cos_product;
 wire signed[31:0] brake_at=late_latched ? 19800:18700;reg[15:0] previous;reg signed[31:0] origin,peak,rate,h0,h1,h2,h3;
 reg[15:0] reverse_ms,stable_ms,arm_fast_ms;reg[3:0] rate_recover;
 reg[2:0] calc_phase;reg signed[31:0] goal,cos_latch,kinetic,potential;
 reg signed[31:0] square,energy_error,pump,tmp;
 reg signed[15:0] v16;
 reg signed[31:0] delta,next_p,inst_rate,beta,abeta,idx,e_target;
  assign rate_diag=rate;assign energy_diag=energy_error;assign goal_diag=goal;
 function signed[31:0] abs32;input signed[31:0]v;begin abs32=v<0?-v:v;end endfunction
 function signed[31:0] wrapped;input signed[31:0]v;
  begin if(v>FULL/2)wrapped=v-FULL;else if(v< -FULL/2)wrapped=v+FULL;else wrapped=v;end
 endfunction
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
 task fail;input[7:0]why;begin phase<=6;result<=why;abort<=1;calc_phase<=0;end endtask
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   phase<=0;result<=0;ident<=0;elapsed<=0;progress<=0;command<=0;
   takeover<=0;capture<=0;abort<=0;dir<=0;late_latched<=0;cos_product<=0;previous<=0;origin<=0;peak<=0;rate<=0;
   h0<=0;h1<=0;h2<=0;h3<=0;reverse_ms<=0;stable_ms<=0;arm_fast_ms<=0;
   rate_recover<=0;energy_error<=0;calc_phase<=0;goal<=0;cos_latch<=0;kinetic<=0;potential<=0;square<=0;
  end else begin
   takeover<=0;capture<=0;abort<=0;
   if(start && !busy)begin
    if(ctrl_state==4 && fresh && abs32(wrapped($signed({1'b0,angle})-$signed({1'b0,up})))<600 &&
       abs32(arm_velocity)<100 && abs32($signed(seed_command))<=80)begin
     phase<=1;result<=0;energy_error<=0;ident<=ident+1'b1;elapsed<=0;progress<=0;peak<=0;rate<=0;
     rate_recover<=0;previous<=angle;origin<=arm_position;dir<=negative;late_latched<=late;command<=seed_command;goal<=0;
     h0<=0;h1<=0;h2<=0;h3<=0;reverse_ms<=0;stable_ms<=0;arm_fast_ms<=0;calc_phase<=0;takeover<=1;
    end else result<=1;
   end
   if(busy)begin
    // Energy chain is pipelined: no square->gain->slew chain in one50MHz cycle.
    case(calc_phase)
     1:begin v16=$signed(rate)>>>4;square<=v16*v16;calc_phase<=2;end
     2:begin kinetic<=(($signed({1'b0,square})*64'sd33)>>>16);potential<=((cos_latch+32'sd32768)*32'sd6600)>>>15;calc_phase<=3;end
     3:begin
      e_target=progress<brake_at ? 18000 : progress<FULL ?
       13200+(((FULL-progress)*(late_latched ? 32'sd47662:32'sd40855))>>>16) :13200;
      energy_error<=e_target-kinetic-potential;calc_phase<=4;
     end
     4:begin pump<=(energy_error*32'sd5)>>>6;calc_phase<=5;end
     5:begin
      // Preserve downhill momentum before scheduled recovery. Do not brake just
      // because the approximate energy proxy momentarily exceeds its target.
      if(progress<brake_at && pump<0)pump<=0;
      else if(pump>MAX_PWM)pump<=MAX_PWM;
      else if(pump< -MAX_PWM)pump<=-MAX_PWM;
      calc_phase<=6;
     end
     6:begin cos_product<=pump*cos_latch;calc_phase<=7;end
     7:begin
      // Continuous cos weighting, no sign(cos) step at horizontal positions.
      tmp=$signed(cos_product)>>>15;
      goal<=dir ? tmp:-tmp;calc_phase<=0;
     end
    endcase
    if(tick)begin
     elapsed<=elapsed+1'b1;
     delta=wrapped($signed({1'b0,angle})-$signed({1'b0,previous}));previous<=angle;
     if(dir)delta=-delta;
     next_p=progress+delta;progress<=next_p;
     if(next_p>peak)peak<=next_p;
     // A bridged endpoint displacement is measured across an observation gap,
     // not a one-ms velocity. Keep the last rate until 4 fresh history ticks exist.
     if(seam_wait || seam_bridge)rate_recover<=4;
     else if(rate_recover!=0)rate_recover<=rate_recover-1'b1;
     else begin inst_rate=(next_p-h3)*32'sd250;rate<=((rate*3)+inst_rate)>>>2;end
     h3<=h2;h2<=h1;h1<=h0;h0<=next_p;
     beta=wrapped($signed({1'b0,angle})-$signed({1'b0,up}));abeta=abs32(beta);
     idx=(abeta*32'sd81345)>>>25;if(idx>32)idx=32;
     cos_latch<=$signed(cosine(idx[5:0]));
     if(abs32(arm_velocity)>1600)arm_fast_ms<=arm_fast_ms+1'b1;else arm_fast_ms<=0;
     if(phase<=2)begin
      if((peak-next_p>220 && rate< -1000) || next_p< -220)reverse_ms<=reverse_ms+1'b1;else reverse_ms<=0;
      if(phase==1)begin
       tmp=elapsed<120 ? elapsed/2 :60;if(tmp>60)tmp=60;
       goal<=dir ? tmp : -tmp;
       if(next_p>=1100 || elapsed>=300)begin phase<=2;calc_phase<=1;end
      end else if(!seam_wait && rate_recover==0)calc_phase<=1;
      if(seam_wait)begin goal<=0;calc_phase<=0;end
      if(goal>$signed(command)+SLEW)command<=command+SLEW;
      else if(goal<$signed(command)-SLEW)command<=command-SLEW;
      else command<=goal[9:0];
      // Catch only after measured unwrapped travel reaches a full turn.
      if(phase==2 && next_p>=FULL && abeta<1100 && rate>=0 && rate<18000)begin phase<=3;capture<=1;calc_phase<=0;end
      else if(next_p>FULL+1400)fail(6);
      if(reverse_ms>=40)fail(3);
     end else if(phase==3)begin
      if(ctrl_state==4)begin phase<=4;stable_ms<=0;end
      else if(ctrl_state!=8 && ctrl_state!=3)fail(7);
     end else if(phase==4)begin
      if(ctrl_state==4 && abeta<800)begin
       if(stable_ms>=499)begin phase<=5;result<=0;end else stable_ms<=stable_ms+1'b1;
      end else if(ctrl_state!=4)fail(7);else stable_ms<=0;
     end
     if(elapsed>=TIMEOUT_MS-1)fail(2);
     if(abs32(delta)>500 && !seam_bridge)fail(4);
     if(sensor_fault!=0)fail(sensor_fault==1 ? 12:13);
     if(abs32(arm_position-origin)>1040 || arm_fast_ms>=100 || abs32(rate)>150000)fail(5);
     if(!fresh && sensor_fault==0)fail(9);
     if(ctrl_state==5 || ctrl_state==6 || ctrl_state==7 || ctrl_state==1)fail(10);
    end
    if(cancel)fail(8);
   end
  end
 end
endmodule
