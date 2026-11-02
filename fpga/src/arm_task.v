// -----------------------------------------------------------------------------
// 拓展任务发生器 V5（WEXF）—— 8 拨码全功能版
//   cmd 1 位置移动：SW0~SW2 选 8 个位置（码 n → n×45° = n×130 计数），
//          以稳摆零点为参考，按"一圈内最短路径"取模：
//          不论横杆已经转过多少圈，都只在半圈内走过去
//   cmd 2 恒速旋转：SW6 选方向（0 正 / 1 负）、SW7 选速率（0 慢 0.25 圈/s、1 快 0.5 圈/s），
//          速度约 1 秒线性过渡到位
//   cmd 3 正弦运动：SW3~SW5 三位组合查表（速度幅值恒为 A=520 counts/s）
//          000 一般正弦 0.25Hz    100 一般正弦 0.5Hz
//          010 单向正向 0.25Hz    110 单向正向 0.5Hz   001 单向正向 0.75Hz
//          101 单向负向 0.25Hz    011 单向负向 0.5Hz   111 单向负向 0.75Hz
//          一般正弦 v = A·sin(2πft)；单向正弦 v = A·|sin(2πft)| 只朝一个方向
// 任何新命令立即接管（含旋转中按 S2 回位置即停转）；退出稳摆自动复位。
// 编码器：13 线霍尔 × 4 倍频 × 20 减速比 = 1040 counts/圈
// -----------------------------------------------------------------------------
module arm_task #(
    parameter integer CLK_FREQ_HZ    = 50_000_000,
    parameter integer COUNTS_PER_REV = 1040
)(
    input  wire clk,
    input  wire rst_n,
    input  wire tick,                    // 1kHz 控制节拍
    input  wire active,                  // 处于 BALANCE
    input  wire cmd_pulse,               // 稳摆中按键命令（单拍）
    input  wire [1:0] cmd_mode,          // 1 位置 / 2 恒速 / 3 单向正弦
    // cmd_cfg：[2:0]=位置码 0..7  [3]=速率档  [4]=方向(0 正 1 负)  [6:5]=频率档
    input wire stream_enable,
    input wire signed [31:0] stream_velocity_q16,
    input  wire [7:0] cmd_cfg,
    output reg  signed [31:0] offset,    // 叠加到横杆零点上的目标偏移（counts）
    output reg  moving,                  // 正在运动（遥测诊断）
    output wire idle                     // 无任务（尚未下发任何 S2/S3/S4 命令）
);
    localparam integer REV      = COUNTS_PER_REV;       // 1040
    localparam integer HALF_REV = COUNTS_PER_REV / 2;   // 520
    localparam integer SEG      = COUNTS_PER_REV / 8;   // 130（45°）
    localparam signed [47:0] REV_Q16 = COUNTS_PER_REV * 65536;

    // 恒速两档（Q16 counts/ms）
    localparam signed [31:0] SPIN_SLOW_Q16 = 32'sd17039; // 260 counts/s = 0.25 圈/s
    localparam signed [31:0] SPIN_FAST_Q16 = 32'sd34079; // 520 counts/s = 0.50 圈/s
    localparam signed [31:0] RAMP_SLOW     = 32'sd17;
    localparam signed [31:0] RAMP_FAST     = 32'sd34;

    // 正弦相位旋转常数（Q20），每 1ms 转一步；模长误差经计算，60s 幅度漂移 < 4%
    localparam signed [20:0] CA = 21'sd1048575, SA = 21'sd1647;  // 0.25 Hz
    localparam signed [20:0] CB = 21'sd1048571, SB = 21'sd3294;  // 0.5  Hz
    localparam signed [20:0] CC = 21'sd1048564, SC = 21'sd4942;  // 0.75 Hz

    // 位置移动：比例步进 ≈ 差值/250（Q16，时间常数~250ms），限幅到最快档，
    // 近目标减速、MIN_STEP 兜底并精确咬合（1/8 圈 130 计数约 0.25s，半圈约 1s，零过冲）
    localparam integer MOVE_MUL = 32'd2097, MOVE_SH = 19;
    localparam signed [31:0] MOVE_MIN_STEP = 32'sd3277; // 0.05 counts/ms 收尾速度

    reg [1:0] mode;
    reg holding; // UI01 explicit hold command only; original task paths unchanged.
    reg signed [47:0] acc_q16;          // 位置偏移累加器（Q16，可为多圈）
    reg signed [47:0] move_final_q16;   // 位置移动目标（精确咬合用）
    reg signed [47:0] rev_q16;          // 同一偏移在一圈内回绕，仅用于取模
    reg signed [31:0] rate_q16;         // 恒速当前速度（Q16 counts/ms）
    reg signed [31:0] spin_step_q16;    // 本任务的目标速度
    reg signed [31:0] ramp_inc;         // 过渡增量
    reg dir;                            // 0 正向 / 1 负向
    reg uni;                            // 1 单向（取绝对值） / 0 一般双向正弦
    reg signed [20:0] phasor_x, phasor_y, ph_c, ph_s;
    reg signed [63:0] rot_t0, rot_t1, rate_t;
    reg signed [63:0] diff_t, prop_t, rev_t;
    reg signed [47:0] diff_r;           // 位置误差锁存（第一拍）
    reg               pos_pend;         // 位置步进第二拍待执行
    reg signed [47:0] step_t, step_r;
    reg               step_v;           // 步进已锁存，下一拍再做相位回绕

    assign idle = (mode == 2'd0) && !stream_enable;       // 供 swing_controller 判断"零点归位"能否进行

    // ---------------------------------------------------------------------
    // 一圈内相位累加器：单独一拍完成，避免与"步进计算（乘法+限幅）"串在
    // 同一条组合路径上（否则时序会差 6ns 以上）
    // ---------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rev_q16 <= 48'sd0;
        else if (!active) rev_q16 <= 48'sd0;
        else if (step_v) begin
            rev_t = rev_q16 + step_r;
            if (rev_t >= REV_Q16) rev_t = rev_t - REV_Q16;
            else if (rev_t < 0) rev_t = rev_t + REV_Q16;
            rev_q16 <= rev_t[47:0];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mode <= 2'd0; holding <= 1'b0;
            acc_q16 <= 48'sd0; move_final_q16 <= 48'sd0;
            rate_q16 <= 32'sd0;
            spin_step_q16 <= SPIN_FAST_Q16; ramp_inc <= RAMP_FAST; dir <= 1'b0;
            phasor_x <= 21'sd1048576; phasor_y <= 21'sd0; ph_c <= CA; ph_s <= SA; uni <= 1'b1;
            offset <= 32'sd0; moving <= 1'b0;
            step_t <= 48'sd0; step_r <= 48'sd0; step_v <= 1'b0;
            diff_r <= 48'sd0; pos_pend <= 1'b0;
        end else if (!active) begin
            mode <= 2'd0; holding <= 1'b0;
            acc_q16 <= 48'sd0; move_final_q16 <= 48'sd0;
            rate_q16 <= 32'sd0;
            spin_step_q16 <= SPIN_FAST_Q16; ramp_inc <= RAMP_FAST; dir <= 1'b0;
            phasor_x <= 21'sd1048576; phasor_y <= 21'sd0; ph_c <= CA; ph_s <= SA; uni <= 1'b1;
            offset <= 32'sd0; moving <= 1'b0;
            step_v <= 1'b0;
            pos_pend <= 1'b0;
        end else begin
            step_v <= 1'b0;
            if (cmd_pulse) begin
                mode <= cmd_mode; holding <= 1'b0;
                pos_pend <= 1'b0;   // 新命令接管：丢弃未执行的位置步进
                if (cmd_mode == 2'd1 && cmd_cfg[7]) begin
                    holding <= 1'b1; rate_q16 <= step_r[31:0];
                    move_final_q16 <= acc_q16;
                end else if (cmd_mode == 2'd1) begin
                    // 目标 = 当前偏移 + 一圈内最短的差值（取模）
                    // 45°×8 正好整除 1040，无取整误差
                    diff_t = ($signed({29'd0, cmd_cfg[2:0]}) * SEG)           // 期望相位 0..910
                             - $signed({{16{rev_q16[47]}}, rev_q16[47:16]});  // 当前相位 0..1039
                    if (diff_t > HALF_REV) diff_t = diff_t - REV;
                    else if (diff_t < -HALF_REV) diff_t = diff_t + REV;
                    move_final_q16 <= acc_q16 + (diff_t <<< 16);
                end else if (cmd_mode == 2'd2) begin
                    spin_step_q16 <= cmd_cfg[3] ? SPIN_FAST_Q16 : SPIN_SLOW_Q16;
                    ramp_inc      <= cmd_cfg[3] ? RAMP_FAST : RAMP_SLOW;
                    dir           <= cmd_cfg[4];
                end else if (cmd_mode == 2'd3) begin
                    // S4 正弦：SW3~SW5 三位组合查表
                    //   000 一般正弦 0.25Hz   100 一般正弦 0.5Hz
                    //   010 单向正向 0.25Hz   110 单向正向 0.5Hz   001 单向正向 0.75Hz
                    //   101 单向负向 0.25Hz   011 单向负向 0.5Hz   111 单向负向 0.75Hz
                    case (cmd_cfg[6:4])
                        3'b000: begin ph_c <= CA; ph_s <= SA; uni <= 1'b0; dir <= 1'b0; end
                        3'b100: begin ph_c <= CB; ph_s <= SB; uni <= 1'b0; dir <= 1'b0; end
                        3'b010: begin ph_c <= CA; ph_s <= SA; uni <= 1'b1; dir <= 1'b0; end
                        3'b110: begin ph_c <= CB; ph_s <= SB; uni <= 1'b1; dir <= 1'b0; end
                        3'b001: begin ph_c <= CC; ph_s <= SC; uni <= 1'b1; dir <= 1'b0; end
                        3'b101: begin ph_c <= CA; ph_s <= SA; uni <= 1'b1; dir <= 1'b1; end
                        3'b011: begin ph_c <= CB; ph_s <= SB; uni <= 1'b1; dir <= 1'b1; end
                        default:begin ph_c <= CC; ph_s <= SC; uni <= 1'b1; dir <= 1'b1; end
                    endcase
                    spin_step_q16 <= SPIN_FAST_Q16; // 正弦峰值速度固定 520 counts/s
                    phasor_x <= 21'sd1048576; phasor_y <= 21'sd0; // v 从 0 平滑起步
                end
            end
            if (tick && !cmd_pulse) begin
                if(stream_enable)begin
                    // Existing task accumulator: no reference jump on entry/exit.
                    mode<=0;rate_q16<=0;pos_pend<=0;
                    step_t={{16{stream_velocity_q16[31]}},stream_velocity_q16};
                    acc_q16<=acc_q16+step_t;step_r<=step_t;step_v<=1;
                    offset<=$signed((acc_q16+step_t)>>>16);moving<=1;
                end else if(holding)begin
                    if(rate_q16>RAMP_FAST)rate_q16<=rate_q16-RAMP_FAST;
                    else if(rate_q16 < -RAMP_FAST)rate_q16<=rate_q16+RAMP_FAST;
                    else rate_q16<=0;
                    step_t={{16{rate_q16[31]}},rate_q16};
                    acc_q16<=acc_q16+step_t;step_r<=step_t;step_v<=1;
                    offset<=$signed((acc_q16+step_t)>>>16);moving<=rate_q16!=0;
                    if(rate_q16==0)begin holding<=0;move_final_q16<=acc_q16;end
                end else case (mode)
                    2'd1: begin // 位置移动：本拍只锁存误差，下一拍再算步进（流水一拍换时序）
                        diff_r  <= move_final_q16 - acc_q16;
                        pos_pend <= 1'b1;
                    end
                    2'd2: begin // 恒速旋转：速度线性过渡到目标
                        if (dir) begin
                            if (rate_q16 > -spin_step_q16) rate_q16 <= rate_q16 - ramp_inc;
                        end else begin
                            if (rate_q16 < spin_step_q16) rate_q16 <= rate_q16 + ramp_inc;
                        end
                        step_t = {{16{rate_q16[31]}}, rate_q16};
                        acc_q16 <= acc_q16 + step_t;
                        moving <= 1'b1;
                        step_r <= step_t;
                        step_v <= 1'b1;
                        offset <= $signed((acc_q16 + step_t) >>> 16);
                    end
                    2'd3: begin // 单向正弦速度：相位定模旋转，v = A·|sin|
                        rot_t0 = (ph_c * phasor_x) - (ph_s * phasor_y);
                        rot_t1 = (ph_s * phasor_x) + (ph_c * phasor_y);
                        phasor_x <= rot_t0[41:20] + rot_t0[19]; // 就近取整
                        phasor_y <= rot_t1[41:20] + rot_t1[19];
                        rate_t = $signed({{43{phasor_y[20]}}, phasor_y}); // Q20 的 sin
                        if (uni && rate_t < 0) rate_t = -rate_t;          // 单向才取绝对值
                        rate_t = (rate_t * spin_step_q16) >>> 20;
                        if (dir) rate_t = -rate_t;
                        step_t = rate_t[47:0];
                        acc_q16 <= acc_q16 + step_t;
                        moving <= 1'b1;
                        step_r <= step_t;
                        step_v <= 1'b1;
                        offset <= $signed((acc_q16 + step_t) >>> 16);
                    end
                    default: begin
                        step_t = 48'sd0;
                        moving <= 1'b0;   // 无任务：保持当前偏移
                        step_r <= 48'sd0;
                        step_v <= 1'b1;
                    end
                endcase
            end else if (pos_pend) begin // 位置移动第二拍：乘限幅 + 咬合 + 累加
                pos_pend <= 1'b0;
                diff_t = diff_r;
                prop_t = ($signed(diff_r[31:0]) * MOVE_MUL) >>> MOVE_SH;
                if (prop_t > SPIN_FAST_Q16) prop_t = SPIN_FAST_Q16;
                else if (prop_t < -SPIN_FAST_Q16) prop_t = -SPIN_FAST_Q16;
                if (prop_t >= 0 && prop_t < MOVE_MIN_STEP) prop_t = MOVE_MIN_STEP;
                else if (prop_t < 0 && prop_t > -MOVE_MIN_STEP) prop_t = -MOVE_MIN_STEP;
                if ((diff_t >= 0 && diff_t <= prop_t) ||
                    (diff_t < 0 && diff_t >= prop_t)) begin
                    step_t = diff_t;      // 剩余不足一步：精确咬合
                    moving <= 1'b0;
                end else begin
                    step_t = prop_t;
                    moving <= 1'b1;
                end
                acc_q16 <= acc_q16 + step_t;
                step_r <= step_t;
                step_v <= 1'b1;
                offset <= $signed((acc_q16 + step_t) >>> 16);
            end
        end
    end
endmodule
