// -----------------------------------------------------------------------------
// 按键消抖
// ACG720 用户按键：按下为低电平，松开为高电平。
// key_pulse 在按键稳定按下后输出一个 50MHz 时钟周期宽的脉冲。
// -----------------------------------------------------------------------------
module key_filter #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer DEBOUNCE_MS = 20
)(
    input  wire clk,
    input  wire rst_n,
    input  wire key_n,
    output reg  key_level_n,
    output reg  key_pulse
);

    localparam integer CNT_MAX = CLK_FREQ_HZ / 1000 * DEBOUNCE_MS;

    reg [1:0] key_sync;
    reg [31:0] cnt;
    reg stable_n;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            key_sync    <= 2'b11;
            cnt         <= 32'd0;
            stable_n    <= 1'b1;
            key_level_n <= 1'b1;
            key_pulse   <= 1'b0;
        end else begin
            key_pulse <= 1'b0;
            key_sync  <= {key_sync[0], key_n};

            if (key_sync[1] != stable_n) begin
                if (cnt >= CNT_MAX - 1) begin
                    cnt      <= 32'd0;
                    stable_n <= key_sync[1];
                    key_level_n <= key_sync[1];
                    if (key_sync[1] == 1'b0)
                        key_pulse <= 1'b1;
                end else begin
                    cnt <= cnt + 1'b1;
                end
            end else begin
                cnt <= 32'd0;
            end
        end
    end

endmodule
