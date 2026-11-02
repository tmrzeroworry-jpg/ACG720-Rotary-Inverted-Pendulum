// -----------------------------------------------------------------------------
// TB6612FNG 电机驱动接口
// motor_cmd：-255 .. +255；正/负分别对应一组 H 桥方向
// enable=0 时 STBY=0，驱动器关断；brake=1 时执行 TB6612 短路刹车
// PWM 默认 10kHz，保留与 J280 官方接口测试例程一致的速度范围
// -----------------------------------------------------------------------------
module motor_tb6612 #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer PWM_FREQ_HZ = 10_000,
    parameter REVERSE = 0
)(
    input  wire clk,
    input  wire rst_n,
    input  wire candidate_pwm, // profile4 BALANCE:25kHz and atomic command latch
    input  wire enable,
    input  wire brake,
    input  wire signed [9:0] motor_cmd,

    output reg motor_pwm,
    output reg motor_ain1,
    output reg motor_ain2,
    output reg motor_stby
);

    localparam integer PWM_PERIOD = CLK_FREQ_HZ / PWM_FREQ_HZ;
    localparam integer CANDIDATE_PERIOD = CLK_FREQ_HZ / 25_000;
    reg candidate_latched;
    wire [31:0] current_period = candidate_latched ? CANDIDATE_PERIOD : PWM_PERIOD;

    reg [15:0] pwm_cnt;
    reg [8:0] duty_cnt;
    reg [8:0] duty_latched;
    reg dir_latched;

    wire signed [9:0] cmd_signed = REVERSE ? -motor_cmd : motor_cmd;
    wire cmd_negative = (cmd_signed < 0);
    // 先在有符号 10 位域内取反，再截为 9 位幅值，避免低位截断后再取反出错。
    wire [8:0] cmd_abs = cmd_negative ? -cmd_signed : cmd_signed;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_cnt      <= 16'd0;
            duty_cnt     <= 9'd0;
            duty_latched <= 9'd0;
            dir_latched  <= 1'b0;
            candidate_latched<=1'b0;
        end else begin
            if (pwm_cnt >= current_period - 1)
                pwm_cnt <= 16'd0;
            else
                pwm_cnt <= pwm_cnt + 1'b1;

            if (pwm_cnt == 16'd0) begin
                candidate_latched<=candidate_pwm;
                if (cmd_abs > 9'd255)
                    duty_cnt <= 9'd255;
                else
                    duty_cnt <= cmd_abs;
                dir_latched <= cmd_negative;
            end

            if(pwm_cnt==0 && candidate_pwm)
                duty_latched <= (cmd_abs>255) ? 9'd255 : cmd_abs;
            else duty_latched <= duty_cnt;
        end
    end

    always @(*) begin
        motor_pwm  = 1'b0;
        motor_ain1 = 1'b0;
        motor_ain2 = 1'b0;
        motor_stby = 1'b0;

        if (enable) begin
            motor_stby = 1'b1;
            if (brake) begin
                // TB6612 短路刹车：AIN1=1, AIN2=1, PWM=1
                motor_ain1 = 1'b1;
                motor_ain2 = 1'b1;
                motor_pwm  = 1'b1;
            end else begin
                if (duty_latched == 9'd0) begin
                    motor_ain1 = 1'b0;
                    motor_ain2 = 1'b0;
                    motor_pwm  = 1'b0;
                end else if (!dir_latched) begin
                    motor_ain1 = 1'b1;
                    motor_ain2 = 1'b0;
                    motor_pwm  = (pwm_cnt < ((duty_latched * current_period) >>> 8));
                end else begin
                    motor_ain1 = 1'b0;
                    motor_ain2 = 1'b1;
                    motor_pwm  = (pwm_cnt < ((duty_latched * current_period) >>> 8));
                end
            end
        end
    end

endmodule
