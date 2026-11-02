// Generated from config/lqr_gains.json. Q16 coefficients, signed64 products.
// Native state order: arm count, ADC error, arm counts/s, ADC codes/s.
module lqr_feedback(input wire clk,rst_n,launch,soft_profile,low_dynamics,
 input wire signed [31:0] arm_error,pole_error,arm_rate,pole_rate,
 output reg done,
 output reg signed [31:0] arm_p,pole_p,arm_d,pole_d);
 reg pending;
 localparam signed [31:0] LOW_ARM_P=-13872,LOW_POLE_P=10603,LOW_ARM_D=-9281,LOW_POLE_D=945;
 reg signed [63:0] ap,pp,ad,pd;
 localparam signed [31:0] ARM_P=-16763, SOFT_ARM_P=-13824;
 localparam signed [31:0] POLE_P=21067, SOFT_POLE_P=13190;
 localparam signed [31:0] ARM_D=-17765, SOFT_ARM_D=-10675;
 localparam signed [31:0] POLE_D=1761, SOFT_POLE_D=1127;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin pending<=0;done<=0;ap<=0;pp<=0;ad<=0;pd<=0;arm_p<=0;pole_p<=0;arm_d<=0;pole_d<=0;end
  else begin
   done<=pending;pending<=launch;
   if(launch)begin
    ap<=$signed(arm_error)*$signed(low_dynamics?LOW_ARM_P:soft_profile?SOFT_ARM_P:ARM_P);
    pp<=$signed(pole_error)*$signed(low_dynamics?LOW_POLE_P:soft_profile?SOFT_POLE_P:POLE_P);
    ad<=$signed(arm_rate)*$signed(low_dynamics?LOW_ARM_D:soft_profile?SOFT_ARM_D:ARM_D);
    pd<=$signed(pole_rate)*$signed(low_dynamics?LOW_POLE_D:soft_profile?SOFT_POLE_D:POLE_D);
   end
   if(pending)begin arm_p<=ap>>>16;pole_p<=pp>>>16;arm_d<=ad>>>16;pole_d<=pd>>>16;end
  end
 end
endmodule
