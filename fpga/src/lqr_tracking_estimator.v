// LQ05 constant-velocity alpha/beta tracker, alpha=1/2, beta=1/8.
// Relative-error Q16 formulation avoids multiplying absolute encoder position.
// Every1ms tick uses the held ADC code: this is NOT a conversion-ready timestamp
// observer or Kalman filter. ADS1115860SPS repeats are modeled in qualification.
// Tracker runs continuously; CATCH uses the old estimator. No handoff reset.
module lqr_tracking_estimator(
 input wire clk,rst_n,tick,recalibrate,accepted,rejected,fresh,
 input wire[15:0]angle_code,
 input wire signed[31:0]arm_position,
 output reg signed[31:0]pole_rate,arm_rate,
 output wire ready,busy
);
 reg[2:0]phase;
 reg[4:0]warm;reg seen;
 reg[15:0]prev_p;reg signed[31:0]prev_e;
 reg signed[47:0]po,eo,pv,ev,dpq,deq,pr,er,pnext,enext;
 reg signed[57:0]pprod,eprod;
 wire signed[17:0]draw=$signed({1'b0,angle_code})-$signed({1'b0,prev_p});
 wire signed[17:0]dp=draw>13200 ? draw-18'sd26400 :draw < -13200 ? draw+18'sd26400 :draw;
 wire signed[31:0]de=arm_position-prev_e;
 wire bad=dp>1600 || dp < -1600 || de>128 || de < -128;
 assign ready=seen && warm>=16 && fresh;
 assign busy=phase!=0;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   phase<=0;warm<=0;seen<=0;prev_p<=0;prev_e<=0;po<=0;eo<=0;pv<=0;ev<=0;
   dpq<=0;deq<=0;pr<=0;er<=0;pnext<=0;enext<=0;pprod<=0;eprod<=0;pole_rate<=0;arm_rate<=0;
  end else if(recalibrate || rejected || !fresh)begin
   phase<=0;warm<=0;seen<=0;prev_p<=angle_code;prev_e<=arm_position;
   po<=0;eo<=0;pv<=0;ev<=0;pole_rate<=0;arm_rate<=0;
  end else begin
   if(accepted)seen<=1;
   if(tick)begin
    prev_p<=angle_code;prev_e<=arm_position;
    if(!seen || warm==0 || bad)begin
     phase<=0;po<=0;eo<=0;pv<=0;ev<=0;pole_rate<=0;arm_rate<=0;
     warm<=bad ? 0 :1;
    end else begin
     dpq<=$signed({{30{dp[17]}},dp})<<<16;
     deq<=$signed({{16{de[31]}},de})<<<16;
     phase<=1;
    end
   end else case(phase)
    1:begin pr<=dpq-po-pv;er<=deq-eo-ev;phase<=2;end
    2:begin
     po<=(pr>>>1)-pr;eo<=(er>>>1)-er;
     pnext<=pv+(pr>>>3);enext<=ev+(er>>>3);phase<=3;
    end
    3:begin
     pv<=pnext;ev<=enext;
     pprod<=pnext*58'sd1000;eprod<=enext*58'sd1000;phase<=4;
    end
    4:begin
     pole_rate<=pprod>>>16;arm_rate<=eprod>>>16;
     if(warm<31)warm<=warm+1'b1;
     phase<=0;
    end
    default:phase<=0;
   endcase
  end
 end
endmodule
