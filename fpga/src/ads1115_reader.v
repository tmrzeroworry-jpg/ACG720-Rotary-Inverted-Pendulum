// -----------------------------------------------------------------------------
// ADS1115 I2C 读取模块
// 硬件配置：ADDR=GND -> 7 位地址 0x48；电位器 POT 接 AIN0
// 配置字：0xC2E3
//   单端 AIN0，PGA=±4.096V，连续转换，860SPS，禁用比较器
// 流程：写 Config(0x01) 和 Pointer(0x00) 后，周期读取 Conversion 两字节
// 默认 I2C 约 100kHz；FPGA 内部上拉与 ADS1115 模块外部上拉可并联使用
// -----------------------------------------------------------------------------
module ads1115_reader #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer I2C_FREQ_HZ = 100_000,
    parameter [6:0] I2C_ADDR = 7'h48,
    parameter [15:0] CONFIG_WORD = 16'hC2E3
)(
    input  wire clk,
    input  wire rst_n,
    input  wire slow_sample, // profile3 BALANCE only; read period1.30ms

    inout  wire i2c_sda,
    inout  wire i2c_scl,

    output reg  [15:0] adc_raw,
    output reg         adc_valid,
    output reg         adc_online,
    output reg         adc_configured,
    output reg         ack_error
);

    localparam integer I2C_DIV = CLK_FREQ_HZ / (I2C_FREQ_HZ * 4);
    localparam integer INIT_TICKS = 10000; // tick2.5us at50MHz/100kHz:25ms
    wire [15:0] read_ticks = slow_sample ? 16'd404 : 16'd280;
    // Read transaction+idle:990us baseline,1300us profile3 BALANCE.

    localparam [3:0]
        ST_IDLE   = 4'd0,
        ST_START  = 4'd1,
        ST_TX     = 4'd2,
        ST_TX_ACK = 4'd3,
        ST_RX     = 4'd4,
        ST_RX_ACK = 4'd5,
        ST_STOP   = 4'd6;

    localparam [1:0]
        CMD_CONFIG  = 2'd0,
        CMD_POINTER = 2'd1,
        CMD_READ    = 2'd2;

    reg [3:0] state;
    reg [1:0] active_cmd;
    reg [1:0] phase;
    reg [2:0] bit_index;
    reg [2:0] tx_index;
    reg [1:0] rx_index;
    reg [2:0] tx_len;
    reg [1:0] rx_len;
    reg [7:0] rx_shift;
    reg [7:0] rx_data_msb;

    reg [15:0] div_cnt;
    reg [15:0] wait_ticks;
    reg pointer_set;

    reg sda_oe;
    reg scl_oe;
    reg [1:0] sda_sync;
    reg [1:0] scl_sync;

    assign i2c_sda = sda_oe ? 1'b0 : 1'bz;
    assign i2c_scl = scl_oe ? 1'b0 : 1'bz;

    wire sda_in = sda_sync[1];
    wire scl_in = scl_sync[1];
    wire i2c_tick = (div_cnt == I2C_DIV - 1);

    function [7:0] current_tx_byte;
        input [1:0] cmd;
        input [2:0] index;
        begin
            case (cmd)
                CMD_CONFIG: begin
                    case (index)
                        3'd0: current_tx_byte = {I2C_ADDR, 1'b0};
                        3'd1: current_tx_byte = 8'h01;
                        3'd2: current_tx_byte = CONFIG_WORD[15:8];
                        default: current_tx_byte = CONFIG_WORD[7:0];
                    endcase
                end
                CMD_POINTER: begin
                    if (index == 3'd0) current_tx_byte = {I2C_ADDR, 1'b0};
                    else               current_tx_byte = 8'h00;
                end
                default: current_tx_byte = {I2C_ADDR, 1'b1};
            endcase
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_cnt <= 16'd0;
        end else if (div_cnt >= I2C_DIV - 1) begin
            div_cnt <= 16'd0;
        end else begin
            div_cnt <= div_cnt + 1'b1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sda_sync <= 2'b11;
            scl_sync <= 2'b11;
        end else begin
            sda_sync <= {sda_sync[0], i2c_sda};
            scl_sync <= {scl_sync[0], i2c_scl};
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= ST_IDLE;
            active_cmd     <= CMD_CONFIG;
            phase          <= 2'd0;
            bit_index      <= 3'd7;
            tx_index       <= 3'd0;
            rx_index       <= 2'd0;
            tx_len         <= 3'd0;
            rx_len         <= 2'd0;
            rx_shift       <= 8'd0;
            rx_data_msb    <= 8'd0;
            wait_ticks     <= 16'd0;
            pointer_set    <= 1'b0;
            sda_oe         <= 1'b0;
            scl_oe         <= 1'b0;
            adc_raw        <= 16'd0;
            adc_valid      <= 1'b0;
            adc_online     <= 1'b0;
            adc_configured <= 1'b0;
            ack_error      <= 1'b0;
        end else begin
            adc_valid <= 1'b0;

            if (i2c_tick) begin
                case (state)
                    ST_IDLE: begin
                        sda_oe <= 1'b0;
                        scl_oe <= 1'b0;

                        if (wait_ticks < INIT_TICKS) begin
                            wait_ticks <= wait_ticks + 1'b1;
                        end else if (wait_ticks >= read_ticks + INIT_TICKS) begin
                            wait_ticks <= INIT_TICKS;
                            ack_error <= 1'b0;

                            if (!adc_configured) begin
                                active_cmd <= CMD_CONFIG;
                                tx_len <= 3'd4;
                                rx_len <= 2'd0;
                            end else if (!pointer_set) begin
                                active_cmd <= CMD_POINTER;
                                tx_len <= 3'd2;
                                rx_len <= 2'd0;
                            end else begin
                                // Pointer 已保持为 Conversion(0x00)，后续直接读
                                active_cmd <= CMD_READ;
                                tx_len <= 3'd1;
                                rx_len <= 2'd2;
                            end

                            state <= ST_START;
                            phase <= 2'd0;
                            tx_index <= 3'd0;
                            rx_index <= 2'd0;
                            bit_index <= 3'd7;
                        end else begin
                            wait_ticks <= wait_ticks + 1'b1;
                        end
                    end

                    ST_START: begin
                        case (phase)
                            2'd0: begin
                                sda_oe <= 1'b0;
                                scl_oe <= 1'b0;
                                phase <= 2'd1;
                            end
                            2'd1: begin
                                sda_oe <= 1'b1;
                                scl_oe <= 1'b0;
                                phase <= 2'd2;
                            end
                            2'd2: begin
                                sda_oe <= 1'b1;
                                scl_oe <= 1'b1;
                                phase <= 2'd0;
                                state <= ST_TX;
                                bit_index <= 3'd7;
                            end
                            default: phase <= 2'd0;
                        endcase
                    end

                    ST_TX: begin
                        case (phase)
                            2'd0: begin
                                scl_oe <= 1'b1;
                                // 开漏：数据位为 0 时拉低 SDA，为 1 时释放（上拉为高）
                                sda_oe <= ((current_tx_byte(active_cmd, tx_index) >> bit_index) & 8'd1) == 8'd0;
                                phase <= 2'd1;
                            end
                            2'd1: begin
                                scl_oe <= 1'b0;
                                phase <= 2'd2;
                            end
                            2'd2: begin
                                phase <= 2'd3;
                            end
                            default: begin
                                scl_oe <= 1'b1;
                                phase <= 2'd0;
                                if (bit_index == 3'd0) begin
                                    state <= ST_TX_ACK;
                                    bit_index <= 3'd7;
                                end else begin
                                    bit_index <= bit_index - 1'b1;
                                end
                            end
                        endcase
                    end

                    ST_TX_ACK: begin
                        case (phase)
                            2'd0: begin
                                scl_oe <= 1'b1;
                                sda_oe <= 1'b0;
                                phase <= 2'd1;
                            end
                            2'd1: begin
                                scl_oe <= 1'b0;
                                phase <= 2'd2;
                            end
                            2'd2: begin
                                if (sda_in)
                                    ack_error <= 1'b1;
                                phase <= 2'd3;
                            end
                            default: begin
                                scl_oe <= 1'b1;
                                phase <= 2'd0;
                                if (tx_index == tx_len - 1'b1) begin
                                    if (rx_len != 2'd0) begin
                                        state <= ST_RX;
                                        bit_index <= 3'd7;
                                        rx_shift <= 8'd0;
                                    end else begin
                                        state <= ST_STOP;
                                    end
                                end else begin
                                    tx_index <= tx_index + 1'b1;
                                    state <= ST_TX;
                                    bit_index <= 3'd7;
                                end
                            end
                        endcase
                    end

                    ST_RX: begin
                        case (phase)
                            2'd0: begin
                                scl_oe <= 1'b1;
                                sda_oe <= 1'b0;
                                phase <= 2'd1;
                            end
                            2'd1: begin
                                scl_oe <= 1'b0;
                                phase <= 2'd2;
                            end
                            2'd2: begin
                                rx_shift <= {rx_shift[6:0], sda_in};
                                phase <= 2'd3;
                            end
                            default: begin
                                scl_oe <= 1'b1;
                                phase <= 2'd0;
                                if (bit_index == 3'd0) begin
                                    if (rx_index == 2'd0)
                                        rx_data_msb <= rx_shift;
                                    else
                                        adc_raw <= {rx_data_msb, rx_shift};
                                    state <= ST_RX_ACK;
                                    bit_index <= 3'd7;
                                end else begin
                                    bit_index <= bit_index - 1'b1;
                                end
                            end
                        endcase
                    end

                    ST_RX_ACK: begin
                        case (phase)
                            2'd0: begin
                                scl_oe <= 1'b1;
                                sda_oe <= (rx_index == rx_len - 1'b1) ? 1'b0 : 1'b1;
                                phase <= 2'd1;
                            end
                            2'd1: begin
                                scl_oe <= 1'b0;
                                phase <= 2'd2;
                            end
                            2'd2: begin
                                phase <= 2'd3;
                            end
                            default: begin
                                scl_oe <= 1'b1;
                                phase <= 2'd0;
                                if (rx_index == rx_len - 1'b1) begin
                                    state <= ST_STOP;
                                end else begin
                                    rx_index <= rx_index + 1'b1;
                                    rx_shift <= 8'd0;
                                    state <= ST_RX;
                                    bit_index <= 3'd7;
                                end
                            end
                        endcase
                    end

                    ST_STOP: begin
                        case (phase)
                            2'd0: begin
                                scl_oe <= 1'b1;
                                sda_oe <= 1'b1;
                                phase <= 2'd1;
                            end
                            2'd1: begin
                                scl_oe <= 1'b0;
                                sda_oe <= 1'b1;
                                phase <= 2'd2;
                            end
                            2'd2: begin
                                scl_oe <= 1'b0;
                                sda_oe <= 1'b0;
                                phase <= 2'd3;
                            end
                            default: begin
                                state <= ST_IDLE;
                                phase <= 2'd0;

                                if (!ack_error) begin
                                    adc_online <= 1'b1;
                                    if (active_cmd == CMD_CONFIG) begin
                                        adc_configured <= 1'b1;
                                        pointer_set <= 1'b0;
                                    end
                                    if (active_cmd == CMD_POINTER)
                                        pointer_set <= 1'b1;
                                    if (active_cmd == CMD_READ) begin
                                        adc_valid <= 1'b1;
                                    end
                                end else begin
                                    adc_online <= 1'b0;
                                    adc_configured <= 1'b0;
                                    pointer_set <= 1'b0;
                                end
                            end
                        endcase
                    end

                    default: begin
                        state <= ST_IDLE;
                        sda_oe <= 1'b0;
                        scl_oe <= 1'b0;
                    end
                endcase
            end
        end
    end

endmodule
