// -----------------------------------------------------------------------------
// 通用 UART 字节发送器：8N1
// 默认 115200bps，tx_start 拉高一个时钟周期启动发送
// -----------------------------------------------------------------------------
module uart_tx_byte #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer BAUD_RATE = 115200
)(
    input  wire clk,
    input  wire rst_n,
    input  wire [7:0] tx_data,
    input  wire tx_start,
    output reg  uart_tx,
    output reg  busy,
    output reg  done
);

    localparam integer BAUD_DIV = CLK_FREQ_HZ / BAUD_RATE;

    reg [15:0] baud_cnt;
    reg [3:0] bit_index;
    reg [9:0] shift_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            uart_tx <= 1'b1;
            busy <= 1'b0;
            done <= 1'b0;
            baud_cnt <= 16'd0;
            bit_index <= 4'd0;
            shift_reg <= 10'h3ff;
        end else begin
            done <= 1'b0;

            if (!busy) begin
                uart_tx <= 1'b1;
                if (tx_start) begin
                    busy <= 1'b1;
                    baud_cnt <= 16'd0;
                    bit_index <= 4'd0;
                    shift_reg <= {1'b1, tx_data, 1'b0};
                    uart_tx <= 1'b0;
                end
            end else begin
                if (baud_cnt >= BAUD_DIV - 1) begin
                    baud_cnt <= 16'd0;
                    if (bit_index == 4'd9) begin
                        busy <= 1'b0;
                        uart_tx <= 1'b1;
                        done <= 1'b1;
                    end else begin
                        bit_index <= bit_index + 1'b1;
                        shift_reg <= {1'b1, shift_reg[9:1]};
                        uart_tx <= shift_reg[1];
                    end
                end else begin
                    baud_cnt <= baud_cnt + 1'b1;
                end
            end
        end
    end

endmodule
