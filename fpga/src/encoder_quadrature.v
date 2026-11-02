// -----------------------------------------------------------------------------
// 13 线霍尔编码器 A/B 相四倍频解码
// 位置计数：有符号32位；明确保留test54/test60的计数方向。
// 原版1'sd1表示-1；本版显式32位加减，避免重构时无意翻转坐标。
// 速度输出：counts/s，每 1ms 根据位置增量刷新一次，并做一阶低通
// REVERSE=1 可整体反转编码器计数方向
// -----------------------------------------------------------------------------
module encoder_quadrature #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter REVERSE = 0
)(
    input  wire clk,
    input  wire rst_n,
    input  wire enc_a,
    input  wire enc_b,
    output reg  signed [31:0] position,
    output reg  signed [31:0] velocity
);

    localparam integer SPEED_DIV = CLK_FREQ_HZ / 1000;

    reg [1:0] sync_a;
    reg [1:0] sync_b;
    reg a_d;
    reg b_d;

    reg [31:0] speed_cnt;
    reg signed [31:0] prev_position;
    reg signed [31:0] inst_velocity;

    wire a = sync_a[1];
    wire b = sync_b[1];
    wire [3:0] transition = {a_d, b_d, a, b};

    wire inc = (transition == 4'b0001) ||
               (transition == 4'b0111) ||
               (transition == 4'b1110) ||
               (transition == 4'b1000);

    wire dec = (transition == 4'b0010) ||
               (transition == 4'b1011) ||
               (transition == 4'b1101) ||
               (transition == 4'b0100);

    wire signed [31:0] delta_position = position - prev_position;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync_a        <= 2'b00;
            sync_b        <= 2'b00;
            a_d           <= 1'b0;
            b_d           <= 1'b0;
            position      <= 32'sd0;
            prev_position <= 32'sd0;
            speed_cnt     <= 32'd0;
            velocity      <= 32'sd0;
            inst_velocity <= 32'sd0;
        end else begin
            sync_a <= {sync_a[0], enc_a};
            sync_b <= {sync_b[0], enc_b};
            a_d <= a;
            b_d <= b;

            if (inc && !dec) begin
                if (REVERSE) position <= position + 32'sd1;
                else         position <= position - 32'sd1;
            end else if (dec && !inc) begin
                if (REVERSE) position <= position - 32'sd1;
                else         position <= position + 32'sd1;
            end

            if (speed_cnt >= SPEED_DIV - 1) begin
                speed_cnt <= 32'd0;
                // 1ms 内增量乘 1000，转换为 counts/s。
                inst_velocity <= delta_position * 32'sd1000;
                // v[n] = 7/8*v[n-1] + 1/8*vin，抑制四倍频计数量化噪声。
                velocity <= ((velocity <<< 3) - velocity + inst_velocity) >>> 3;
                prev_position <= position;
            end else begin
                speed_cnt <= speed_cnt + 1'b1;
            end
        end
    end

endmodule
