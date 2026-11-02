// Profile3 only: six-sample least-squares derivative at a known1.30ms cadence.
// Return in legacy units (0.495 * real counts/s), so BAL_KD_ANGLE remains96.
// Numerator=5*d0+8*d1+9*d2+8*d3+5*d4, coefficient495000/(35*1300).
// The actual valid-to-valid interval is checked; transition/missing samples re-prime.
module balance_rate_estimator #(
    parameter integer CLK_FREQ_HZ=50_000_000,
    parameter integer FULL_SCALE=26400,
    parameter integer EXPECTED_US=1300
)(
    input wire clk, rst_n,
    input wire active,
    input wire [15:0] angle_raw,
    input wire angle_valid,
    input wire signed [31:0] fallback_velocity,
    output reg signed [31:0] velocity,
    output reg ready
);
    localparam integer EXPECTED_CLKS=(64'd1*CLK_FREQ_HZ*EXPECTED_US)/1_000_000;
    reg [31:0] interval_clks;
    reg [15:0] previous_angle;
    reg seeded;
    reg [2:0] deltas;
    reg signed [17:0] d1,d2,d3,d4;
    reg [2:0] pending;
    reg signed [31:0] numerator_r;
    reg signed [63:0] product_r;
    reg signed [17:0] raw_delta, dx_r;
    reg sample_bad, sample_in_range;
    reg signed [31:0] numerator;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            interval_clks<=0; previous_angle<=0; seeded<=0; deltas<=0;
            d1<=0; d2<=0; d3<=0; d4<=0; velocity<=0; ready<=0;
            pending<=0; numerator_r<=0; product_r<=0;
            raw_delta<=0;dx_r<=0;sample_bad<=0;sample_in_range<=0;numerator=0;
        end else begin
            if(interval_clks<32'hffffffff) interval_clks<=interval_clks+1'b1;
            if(!active) begin
                seeded<=0; deltas<=0; ready<=0; velocity<=fallback_velocity;
                interval_clks<=0; pending<=0;
            end else if(angle_valid) begin
                interval_clks<=0;
                raw_delta<=$signed({2'b00,angle_raw})-$signed({2'b00,previous_angle});
                sample_in_range<=angle_raw<=FULL_SCALE;
                sample_bad<=!seeded || angle_raw>FULL_SCALE ||
                   interval_clks<EXPECTED_CLKS-4 || interval_clks>EXPECTED_CLKS+4;
                previous_angle<=angle_raw;
                pending<=1;
            end else if(pending==1) begin
                if(raw_delta>FULL_SCALE/2) dx_r<=raw_delta-FULL_SCALE;
                else if(raw_delta< -FULL_SCALE/2) dx_r<=raw_delta+FULL_SCALE;
                else dx_r<=raw_delta;
                pending<=2;
            end else if(pending==2) begin
                seeded<=sample_in_range;
                if(sample_bad || dx_r>300 || dx_r< -300) begin
                    deltas<=0; d1<=0; d2<=0; d3<=0; d4<=0; ready<=0; pending<=0;
                    velocity<=fallback_velocity;
                end else begin
                    d4<=d3; d3<=d2; d2<=d1; d1<=dx_r;
                    if(deltas<5) deltas<=deltas+1'b1;
                    if(deltas>=4) begin
                        numerator=5*$signed(dx_r)+8*$signed(d1)+9*$signed(d2)+8*$signed(d3)+5*$signed(d4);
                        numerator_r<=numerator; pending<=3;
                    end else pending<=0;
                end
            end else if(pending==3) begin
                product_r<=$signed(numerator_r)*64'sd712974; pending<=4;
            end else if(pending==4) begin
                velocity<=product_r[47:16]; ready<=1; pending<=0;
            end
        end
    end
endmodule
