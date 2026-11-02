// Registered friction compensation, PD/LQR blend, clamp and slew.
// For bounded numerator0..8000, floor(n*5243/2^20) is EXACT floor(n/200).
// Exhaustive software and RTL vectors check this identity. No combinational divider.
module lqr_output_pipeline(
 input wire clk,rst_n,launch,cancel,is_catch,limit_slew,
 input wire[1:0]profile_id,
 input wire signed[31:0] linear,arm_rate,pd_sum,
 input wire[8:0] blend,
 input wire signed[9:0] current_cmd,
 output reg busy,done,
 output reg signed[31:0] compensated,
 output reg signed[9:0] target,command
);
 reg[3:0]phase;
 reg signed[31:0]lin,old;
 reg[8:0]mix;
 reg catch_mode,slew_mode,quiet,negative,motion_negative;
 reg smooth_comp;
 reg comp_enabled;reg signed[31:0]slew_step;
 reg[9:0]weight;
 reg[5:0]static_mag;
 reg[14:0]moving_num,static_num;
 reg[29:0]moving_product,static_product;
 reg signed[9:0]pd_clamped,lqr_clamped;
 reg signed[31:0]pd_comp;
 reg signed[25:0]mix_product;
 wire[31:0]amag=arm_rate<0 ? -arm_rate :arm_rate;
 wire[31:0]umag=linear<0 ? -linear :linear;
 wire signed[31:0]moving_value=motion_negative ? $signed({26'd0,(smooth_comp ? moving_product[27:22] :moving_product[25:20])}) :
                                                       -$signed({26'd0,(smooth_comp ? moving_product[27:22] :moving_product[25:20])});
 wire signed[31:0]static_value=negative ? -$signed({26'd0,(smooth_comp ? static_product[27:22] :static_product[25:20])}) :
                                                       $signed({26'd0,(smooth_comp ? static_product[27:22] :static_product[25:20])});
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   busy<=0;done<=0;phase<=0;compensated<=0;target<=0;command<=0;
   lin<=0;old<=0;mix<=0;catch_mode<=0;slew_mode<=0;quiet<=0;negative<=0;motion_negative<=0;
   smooth_comp<=0;comp_enabled<=1;slew_step<=12;
   weight<=0;static_mag<=0;moving_num<=0;static_num<=0;moving_product<=0;static_product<=0;
   pd_clamped<=0;lqr_clamped<=0;pd_comp<=0;mix_product<=0;
  end else if(cancel)begin busy<=0;done<=0;phase<=0;end
  else begin
   done<=0;
   if(launch && !busy)begin
    busy<=1;phase<=1;lin<=linear;old<={{22{current_cmd[9]}},current_cmd};
    mix<=blend;catch_mode<=is_catch;slew_mode<=limit_slew;
    // LQ03 isolates estimator dynamics: common compensated12-code/ms output.
    comp_enabled<=1'b1;
    slew_step<=32'sd12;
    quiet<=umag<3 && amag<50;negative<=linear<0;motion_negative<=arm_rate<0;
    smooth_comp<=!is_catch && profile_id[0];
    weight<=(!is_catch && profile_id[0]) ? (amag>800 ? 10'd800 :amag[9:0]) : (amag>200 ? 10'd200 :{2'd0,amag[7:0]});
    // Smooth family: local static increment slope1/4, maximum24codes.
    static_mag<=(!is_catch && profile_id[0]) ? (umag>=96 ? 6'd24 :umag[6:2]) : (umag>=32 ? 6'd40 :((umag[4:0]*11'd40)>>5));
    pd_comp<=pd_sum>0 ? pd_sum+32'sd40 : pd_sum<0 ? pd_sum-32'sd40 :32'sd0;
   end else if(busy)begin
    case(phase)
     1:begin
      moving_num<=weight*15'd28;
      static_num<=static_mag*((smooth_comp ? 15'd800 :15'd200)-{5'd0,weight});
      pd_clamped<=pd_comp>255 ? 10'sd255 :pd_comp < -255 ? -10'sd255 :pd_comp[9:0];
      phase<=2;
     end
     2:begin moving_product<=moving_num*30'd5243;static_product<=static_num*30'd5243;phase<=3;end
     3:begin compensated<=quiet ? 32'sd0 : comp_enabled ? lin+moving_value+static_value : lin;phase<=4;end
     4:begin lqr_clamped<=compensated>180 ? 10'sd180 : compensated < -180 ? -10'sd180 :compensated[9:0];phase<=5;end
     5:begin
      mix_product<=$signed(pd_clamped)*$signed({1'b0,(9'd256-mix)})
                 +$signed(lqr_clamped)*$signed({1'b0,mix});phase<=6;
     end
     6:begin target<=catch_mode ? (mix_product>>>8) :lqr_clamped;phase<=7;end
     7:begin
      if(slew_mode && $signed(target)>old+slew_step)command<=old+slew_step;
      else if(slew_mode && $signed(target)<old-slew_step)command<=old-slew_step;
      else command<=target;
      done<=1;busy<=0;phase<=0;
     end
     default:begin busy<=0;phase<=0;end
    endcase
   end
  end
 end
endmodule
