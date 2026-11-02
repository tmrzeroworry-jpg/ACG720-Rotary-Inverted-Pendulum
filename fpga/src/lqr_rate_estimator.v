// LQ03: true counts/s; selectable 4/2 tick difference and alpha1/4 or1/2.
// mode0 baseline; bit0 short difference, bit1 faster IIR. Q2 state is
// retained across mode changes, including CATCH->BALANCE; never zeroed there.
// No legacy delta*500 scale. Invalid input re-primes rather than creating a kick.
module lqr_rate_estimator(
 input wire clk,rst_n,tick,recalibrate,accepted,rejected,fresh,
 input wire [1:0] mode,
 input wire [15:0] angle_code,
 input wire signed [31:0] arm_position,
 output wire signed [31:0] pole_rate,arm_rate,
 output wire ready
);
 reg [15:0] ph0,ph1,ph2,ph3;
 reg signed [31:0] eh0,eh1,eh2,eh3;
 reg signed [35:0] pa,ea;
 reg [4:0] warm;
 reg seen;
 wire short_window=mode[0];
 wire fast_filter=mode[1];
 wire [15:0] angle_previous=short_window ? ph1 : ph3;
 wire signed [31:0] arm_previous=short_window ? eh1 : eh3;
 wire signed [17:0] raw_delta=$signed({1'b0,angle_code})-$signed({1'b0,angle_previous});
 wire signed [17:0] pd=raw_delta>13200 ? raw_delta-18'sd26400 :
                               raw_delta < -13200 ? raw_delta+18'sd26400 :raw_delta;
 wire signed [31:0] ed=arm_position-arm_previous;
 wire signed [35:0] pole_scaled=$signed(pd)*36'sd250;
 wire signed [35:0] arm_scaled=$signed(ed)*36'sd250;
 wire signed [35:0] pole_inst=short_window ? pole_scaled<<<1 : pole_scaled;
 wire signed [35:0] arm_inst=short_window ? arm_scaled<<<1 : arm_scaled;
 // Same physical discontinuity threshold for both windows.
 wire discontinuity=short_window ? (pd>800 || pd < -800 || ed>64 || ed < -64) :
                                  (pd>1600 || pd < -1600 || ed>128 || ed < -128);
 assign pole_rate=pa>>>2;
 assign arm_rate=ea>>>2;
 assign ready=seen && warm>=16 && fresh;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin ph0<=0;ph1<=0;ph2<=0;ph3<=0;eh0<=0;eh1<=0;eh2<=0;eh3<=0;
   pa<=0;ea<=0;warm<=0;seen<=0;
  end else if(recalibrate || rejected || !fresh)begin
   ph0<=angle_code;ph1<=angle_code;ph2<=angle_code;ph3<=angle_code;
   eh0<=arm_position;eh1<=arm_position;eh2<=arm_position;eh3<=arm_position;
   pa<=0;ea<=0;warm<=0;seen<=0;
  end else begin
   if(accepted)seen<=1;
   if(tick)begin
    if(!seen || warm<4)begin
     pa<=0;ea<=0;
    end else if(discontinuity)begin
     // Sensor/encoder discontinuity; prime history before accepting control.
     pa<=0;ea<=0;warm<=0;
    end else begin
     if(fast_filter)begin
      // pa/ea always have two fractional bits: alpha1/2 requires 2*v_inst.
      pa<=pa-(pa>>>1)+(pole_inst<<<1);
      ea<=ea-(ea>>>1)+(arm_inst<<<1);
     end else begin
      pa<=pa-(pa>>>2)+pole_inst;
      ea<=ea-(ea>>>2)+arm_inst;
     end
    end
    ph3<=ph2;ph2<=ph1;ph1<=ph0;ph0<=angle_code;
    eh3<=eh2;eh2<=eh1;eh1<=eh0;eh0<=arm_position;
    if(seen && warm<31 && !discontinuity)warm<=warm+1'b1;
   end
  end
 end
endmodule
