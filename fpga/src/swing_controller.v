// -----------------------------------------------------------------------------
// IP570 旋转倒立摆：自动起摆、捕获与倒立稳摆控制器
// 控制周期：1kHz
// 起摆：按摆角速度方向施加限幅泵摆 PWM，同时加入摆臂居中修正
// 捕获：接近直立点后提高角速度阻尼
// 稳摆：摆角误差 + 摆角速度 + 摆臂位置 + 摆臂速度 的 PD 组合
//
// 时序说明（v2 修复）：
//   控制计算拆成三级流水线（control_tick 启动，3 个时钟后更新 motor_cmd），
//   1kHz 控制节拍相对 50MHz 时钟有充足余量，延迟 60ns 对控制无影响。
//   起摆中的除以 SWING_VEL_DIV 改为乘倒数 VEL_MUL，避免组合逻辑除法器。
//   摆角速度低通滤波拆成两拍完成，避免 (v<<2)-v+inst 单拍长链。
// -----------------------------------------------------------------------------
module swing_controller #(
    parameter ONE_EURO=0,
    parameter integer CLK_FREQ_HZ = 50_000_000,

    // ADS1115 ±4.096V 量程下，WDD35D4 约 345°/0~3.3V 的理论计数
    parameter integer ANGLE_FULL_SCALE = 26400,
    parameter integer ANGLE_HALF_SCALE = 13200,
    // 竖直零点修正：标定假设"竖直=下垂+严格半圈"，但传感器非线性/标定时
    // 线缆张力使对径关系不成立。calib_check3 手持物理倒立实测 err=-709
    // （stdev 仅 3），即控制器"竖直"偏了 +9.7°——稳摆时持续单向重力矩、
    // 摆臂被迫无限加速，是 test30-32 三次同向慢发散的根因
    parameter integer UPRIGHT_TRIM = 709,
    parameter USE_BOOT_REFERENCE = 0,
    parameter [15:0] BOOT_DOWN = 16'd20160,
    parameter [15:0] BOOT_UP = 16'd6251,
    parameter PEND_DIR_REVERSE = 0,

    // 校准与控制阈值，单位均为 ADC count
    parameter integer CALIB_SAMPLES = 1200,  // 约 1.19s 采样窗（本工程 angle_valid 约 1010Hz），
                                             // 上电/待机长按S0后的共同标定窗
    parameter integer CALIB_WAIT_MS = 3000, // 上电等待 ADS1115 上线的最长时间
    parameter integer CATCH_ANGLE = 2800,   // 约 38°：test19 顶点停滞在 2079（窗 2000 之外 79 计数），
                                            // 放宽以包住阻尼区驯服后的低速顶点
    parameter integer CATCH_RATE  = 20000,  // count/s，约 0.76 圈/s：只接低速到达，
                                            // 15000 过严会放过 err≈-2200/vel≈17000 的可接机会（test24）
    parameter integer CATCH_HOLD_MS = 300,
    // test60：交付速度闸门——CATCH 停留时间到且摆杆速度足够小才进 BALANCE。
    // test58/59 三跑证明：交付 |rate|>3900 必死（65~133ms），温和交付（-3199）
    // 可稳 391ms 以上；原切换只看时间不看速度，等于每次掷骰子
    parameter integer CATCH_EXIT_RATE = 1500, // count/s，约 0.06 圈/s
    parameter integer CATCH_MAX_MS = 1500,    // 等不到温和交付就退回 SWING 重泵
                                              // （20s 起摆总超时兜底，不会死循环）
    parameter integer FAIL_ANGLE  = 12000,  // 约 163°：仅在传感器异常/彻底倒下时兜底刹车。
                                            // 必须远大于 RESTART_ANGLE，否则 150ms 确认窗内
                                            // 就会先被这个分支刹车，自动重起摆永远触发不了。
    // WEXD：大扰动自动重起摆。摆角超阈值并持续确认时间 → 退出 BALANCE 回 SWING 重来；
    // 稳定不倒 RESTART_OK_MS 后清零计数；连续失败 RESTART_MAX 次则刹车 + fault 4。
    parameter integer RESTART_ANGLE = 2200,      // 30.0°
    parameter integer RESTART_CONFIRM_MS = 150,  // 持续超阈确认，滤掉瞬时毛刺
    parameter integer RESTART_MAX = 5,           // 连续重起摆上限
    parameter integer RESTART_OK_MS = 3000,      // 稳摆这么久才算"这一轮成功"
    // WEXG：固定停泊位（零点归位）。每次进入稳摆后，在没有下发 S2/S3/S4 任务时，
    // 以受限速度把平衡零点 lqr_center 缓缓滑向 home_center（本次上电后第一次稳摆的位置）。
    // 只平移零点、不改任何增益，等效于让横杆以 0.19 圈/s 缓慢平移到位。
    parameter integer HOME_SLEW_DIV     = 5,     // 每 5 ms 走 1 计数 = 200 counts/s
    parameter integer BAL_HOME_DELAY_MS = 1000,  // 进稳摆后先稳住 1 秒再开始归位
    parameter integer HOME_GATE_ANGLE   = 800,   // 摆角小于此值才归位（约 11°）
    parameter integer ARM_COUNTS_PER_REV = 1040, // 横杆编码器一圈计数（13 线 ×4 ×20）
    parameter integer ARM_HALF_REV       = 520,  // 半圈：归位取模用
    parameter integer ARM_LIMIT   = 40000,
    parameter ARM_LIMIT_ENABLE = 1, // test50 explicitly bypasses travel stop, timers retained
    parameter integer SWING_TIMEOUT_MS = 30000,
    parameter integer AGITATE_MS = 1500, // 起摆初期方波激励时长，摆杆静止时速度泵无法自启动
    parameter integer AGITATE_DUTY = 70,  // 激励幅值：高于电机死区(约38)即可，满幅会抽鞭子

    // 起摆限幅与增益
    parameter integer SWING_BASE_DUTY = 85,
    parameter integer SWING_VEL_DIV   = 1200,
    parameter integer SWING_MAX_DUTY  = 170,
    parameter integer SWING_MIN_RATE  = 3000, // 低速死区：|vel| 低于此值只保留摆臂居中，
                                              // 防止速度估计噪声导致泵/刹方向高频翻转（异响）
    parameter integer SWING_CENTER_SHIFT = 12,
    parameter integer ENERGY_TARGET = 13150, // 能量目标：配合阻尼区使低速到达（test26 验证，
                                             // 17s 可控 7 次捕获尝试）；14500 太猛会翻顶（test27）
    parameter integer ENERGY_GAIN = 82,    // 能量差比例系数：cmd = err*(82/1024) ≈ err*0.08，平滑渐进
    parameter integer ENERGY_GAIN_SHIFT = 10,
    parameter integer NEAR_TOP_ANGLE = 6000,  // 离顶约 82° 内进入近顶阻尼区（只刹不泵）
    parameter integer NEAR_TOP_DAMP_MUL = 600, // 阻尼系数：cmd = -vel*(600/65536) ≈ -vel*0.009
    parameter integer PUMP_BAND = 9000,        // 只在底部 62° 内泵能，中段自由滑行
    parameter integer SERVO_TARGET = 13000, // 顶部区能量伺服目标：顶点精确停在离顶约 1.5°
    parameter integer SERVO_CAP = 60,       // 顶部区伺服上限：温柔调整不扰动
    parameter integer ROT_RATE = 15000,   // 旋转检测：|vel| 持续超过此值且不换向
    parameter integer ROT_MS = 1000,      // 旋转检测：同号持续时间阈值（约 1s）
    parameter integer CATCH_EXIT_ANGLE = 5000, // 捕获中允许摆角冲到此范围，荡回来继续接

    // 稳摆增益：全部通过整数乘加移位实现，便于上板调参
    parameter integer BAL_KP_ANGLE = 24,  // test30/31：旧值 6 时 7° 倾斜仅输出 ~111，
                                          // 追不上重力矩导致慢发散；24 使 4° 即输出 ~180
    parameter integer BAL_KP_SHIFT = 6,
    parameter integer BAL_KD_ANGLE = 48,  // test30：旧值 8 在 vel=3000 时贡献仅 6，
                                          // 阻尼形同虚设导致欠阻尼慢发散（立 0.45s 后倾倒）；
                                          // 48 使 vel=3000 时贡献 35、坠落时贡献上百
    parameter integer BAL_KD_SHIFT = 12,
    parameter integer BAL_KP_ARM = 1,
    parameter integer BAL_KP_ARM_SHIFT = 9,
    parameter integer BAL_KD_ARM = 1,
    parameter integer BAL_KD_ARM_SHIFT = 11,
    parameter integer BALANCE_REVERSE = 0,
    // 慢弹簧横杆锚定（泄漏积分器）：比例/速度锚定不分快慢，会把摆杆的
    // 高速纠正也抵消（test43-45 摆杆近自由落体下沉而 cmd 存在——输出被
    // 锚定项吃掉）；test34 积分意外证明只有"慢"锚定有效。
    // acc += arm_error - acc>>>LEAK：1kHz 时时间常数约 0.256s，对快速纠正透明，
    // 对持续漂移稳态等效 arm_error/16 弹簧，泄漏防积分过头
    // test48: coupled upright feedback, independent of swing/catch arm centering.
    parameter BAL_ARM_REVERSE = 0, // 1 negates BOTH arm terms in BALANCE only
    parameter ARM_ANCHOR_ENABLE = 1,
    parameter integer BAL_RATE_FILTER_SHIFT = 3, // extra 1kHz LPF, ~7.5ms time constant
    parameter integer BAL_SLEW_PER_MS = 16,
    parameter integer ARM_ANCHOR_LEAK = 8,
    parameter integer ARM_ANCHOR_SHIFT = 12,
    parameter integer TRIM_SERVO_TICKS = 100,  // 0.2s（2ms tick）；test36：0.5s
                                               // 太慢，首刀未落摆臂已加速失控
    parameter integer DEADZONE_COMP = 40, // 电机死区补偿：CATCH 突变式基值
    parameter integer DEADZONE_QUIET = 6, // BALANCE 指令死区：|u|<此值直接输出0，
                                          // 让系统在零点附近安静滑行（治极限环抖动）
    parameter integer BALANCE_TRIAL_MS = 20000,

    // ===== test55 新增：三层自动保护 =====
    parameter integer BAL_GRACE_MS = 500,   // 进 BALANCE 后的保护宽限期（捕获交付瞬态不计）
    // 保护1：失速旋转（仅 BALANCE 武装）。高速旋转禁止直接短路刹车
    // （满速刹车能量全砸在驱动内阻上，是烧 TB6612 的典型工况）→ 先进 ST_DECEL 主动反驱
    parameter integer RUNAWAY_RATE = 2200,  // 横杆速度阈值 counts/s，约 2 圈/秒
    parameter integer RUNAWAY_MS = 300,     // 需持续超阈这么久才触发（防毛刺）
    parameter integer DECEL_DONE_RATE = 600, // 减速到低于此转速转短路刹车
    parameter integer DECEL_TIMEOUT_MS = 2000, // 主动减速超时则强制短路刹车兜底
    // 保护2：高频抖振（仅 BALANCE 武装）。滑动窗内有效换向次数超阈 →
    // 先软压抖（换向斜率收紧到 OSC_SOFT_SLEW），再犯则刹车
    parameter integer OSC_WINDOW_MS = 1000,
    parameter integer OSC_REVERSES = 40,
    parameter integer OSC_MIN_AMP = 20,     // 换向时新指令幅值超过此值才算有效换向
    parameter integer OSC_SOFT_SLEW = 8,
    parameter integer OSC_SOFT_MS = 500,
    // 保护3：热保护 I²t 模型（CATCH/BALANCE/DECEL 武装；SWING 有 20s 超时兜底不武装）。
    // 每 ms 累计 (cmd²>>>8)-56：|cmd|=120 为中性点，180 约 3s 触发，255 约 1s 触发
    parameter integer THERMAL_MAX = 210000,
    parameter integer THERMAL_RESUME = 100000, // 冷却到此值以下才允许再次启动
    parameter integer ANCHOR_GATE_ANGLE = 220, // 慢弹簧门控：|err|<此值(约3°)才积累，
                                               // 摆杆大幅度纠正时锚定冻结（积分分离）

    // 捕获阶段独立增益
    parameter integer CATCH_KP_ANGLE = 20,
    parameter integer CATCH_KP_SHIFT = 6,
    parameter integer CATCH_KD_ANGLE = 40,  // 坠落刹车加倍：26° 坠落需要强阻尼（test21）
    parameter integer CATCH_KD_SHIFT = 11,
    // 捕获阶段独立的横杆弱增益：捕获需要横杆自由（全力接摆杆），
    // 稳摆需要横杆锚定（防加速漂移环）。test42 共用强锚定导致捕获
    // 横杆被绑死，交付角度从 2° 恶化到 8.5°
    parameter integer CATCH_KP_ARM = 8,
    parameter integer CATCH_KP_ARM_SHIFT = 8,
    parameter integer CATCH_KD_ARM = 4,
    parameter integer CATCH_KD_ARM_SHIFT = 10
)(
    input  wire clk,
    input  wire rst_n,
    input  wire [1:0] profile_id,
    input wire[1:0]euro_mode,
    input wire euro_p_quiet,euro_a_quiet,
    input  wire recalibrate_pulse,
    input  wire signed [31:0] candidate_balance_rate,
    input  wire candidate_rate_ready,

    input  wire [15:0] angle_raw,
    input  wire angle_valid,
    input  wire angle_online,

    input  wire signed [31:0] arm_position,
    input  wire signed [31:0] arm_velocity,

    input wire start_fast,return_start,return_cancel,
    output wire life_busy,return_busy,life_fresh,life_mode,life_down_valid,
    output wire[7:0]life_phase,life_result,life_turns,
    output wire[15:0]life_id,life_elapsed,life_down,
    output wire signed[31:0]life_rate,
    input  wire start_pulse,
    input  wire brake_pulse,
    input  wire [1:0] task_mode,   // 稳摆中任务：1位置(S2) 2恒速(S3) 3正弦(S4)
    input  wire task_cmd,          // 任务命令脉冲（仅稳摆中由按键产生）
    input wire flip_start,flip_negative,flip_late,flip_cancel,
    output wire flip_busy,flip_sensor_fresh,output wire[7:0] flip_phase,flip_result,
    output wire[15:0] flip_id,flip_elapsed,output wire signed[31:0] flip_progress,
    input wire stream_enable,
    input wire signed [31:0] stream_velocity_q16,
    output wire task_idle_o,
    input  wire [7:0] task_cfg,    // S2/S3/S4 配置（见 test_selector / arm_task）

    output reg  signed [9:0] motor_cmd,
    output reg  motor_enable,
    output reg  motor_brake,
    output reg  [3:0] state,
    output reg  [7:0] fault_code,

    output reg  [15:0] angle_down,
    output reg  [15:0] angle_up,
    output reg  signed [17:0] upright_error,
    output reg  signed [31:0] pend_velocity,
    output wire signed [31:0] balance_velocity, // LQ01 CATCH/BAL true ADCcodes/s
    output wire signed [15:0] trim_adj_o,
    output wire signed [31:0] diag_p, diag_d, diag_arm_p, diag_arm_d,
    output wire signed [31:0] diag_linear,
    output reg signed [31:0] diag_compensated,
    output reg signed [9:0] diag_target
    ,output wire control_tick_o, adc_fresh_o, lqr_ready_o
    ,output wire signed[31:0] lqr_arm_rate_o
    ,output wire signed[31:0] arm_target_o
    ,output wire signed[31:0] arm_home_o
    ,output wire arm_home_valid_o
);



    // T60R: accepted-only signed ADC guard; UART still transmits untouched raw word.
    wire life_takeover,life_capture,life_park,life_abort,life_wait,life_bridge;
    wire[15:0] life_angle,life_seam_age,life_raw_age,life_good_age;wire[7:0]life_sensor_fault,life_sensor_diag;
    wire signed[9:0]life_command;wire signed[31:0]life_goal,life_energy;
    wire life_fast=start_pulse && start_fast && state==1;
    wire flip_takeover,flip_capture,flip_abort;
    wire signed[9:0] flip_command;wire signed[31:0] flip_rate_diag,flip_energy_diag,flip_goal_diag;
    wire[15:0] flip_angle,flip_seam_age;wire flip_seam_wait,flip_bridge;
    wire[7:0] flip_sensor_fault,flip_sensor_diag;
    wire angle_word_legal,angle_accepted_now,angle_rejected_now,angle_clamped,angle_wrapped;
    reg angle_accepted,angle_rejected;
    wire [15:0] guarded_code;
    reg [15:0] last_good_angle;
    reg angle_good_seen;
    ads1115_angle_guard angle_guard(.adc_word(angle_raw),.adc_valid(angle_valid),
        .word_legal(angle_word_legal),.sample_valid(angle_accepted_now),.sample_rejected(angle_rejected_now),
        .sample_clamped(angle_clamped),.sample_wrapped(angle_wrapped),.angle_code(guarded_code));
    // Only a completed, accepted transaction changes the control measurement.
    // Previous code exposed adc_raw beforethe reader completedACK/STOP.
    wire [15:0] angle_safe=last_good_angle;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n || recalibrate_pulse)begin
            last_good_angle<=0;angle_good_seen<=0;angle_accepted<=0;angle_rejected<=0;
        end else begin
            // Event and cached payload are aligned for consumers at the nextedge.
            angle_accepted<=angle_accepted_now;angle_rejected<=angle_rejected_now;
            if(angle_accepted_now)begin last_good_angle<=guarded_code;angle_good_seen<=1;end
        end
    end
    localparam [3:0]
        ST_CALIBRATE = 4'd0,
        ST_READY     = 4'd1,
        ST_SWING     = 4'd2,
        ST_CATCH     = 4'd3,
        ST_BALANCE   = 4'd4,
        ST_BRAKE     = 4'd5,
        ST_FAULT     = 4'd6,
        ST_FLIP      = 4'd8, ST_TRANSITION = 4'd9,
        ST_DECEL     = 4'd7; // test55：失速保护 L1，主动反驱减速后再短路刹车

    localparam integer CTRL_DIV = CLK_FREQ_HZ / 1000;

    // 起摆速度增益的倒数乘法因子：abs_vel/SWING_VEL_DIV ≈ ((abs_vel>>>6)*VEL_MUL)>>>16
    localparam integer VEL_MUL = (32'd4194304 + SWING_VEL_DIV / 2) / SWING_VEL_DIV;

    reg [15:0] angle_prev;
    reg angle_first;
    reg [23:0] cal_sum;
    reg [10:0] cal_count;
    reg [15:0] cal_average;
    reg [15:0] cal_up;
    reg preserve_reference;
    // 摆动中点标定：静摩擦使静止下垂每次偏真垂点 ±2~5°（每次 S4 抽奖，
    // 是 test33-44 倾斜方向随批次变化的根源）；自由摆动极值关于真垂点
    // 对称，取极值中点消摩擦；不拨摆杆则退化为静止标定
    reg [15:0] cal_seed;
    reg signed [17:0] cal_dmin;
    reg signed [17:0] cal_dmax;
    reg signed [17:0] cal_d;
    reg signed [17:0] cal_mid;
    reg signed [31:0] cal_down32;

    reg [15:0] adc_age_ms;
    reg [31:0] ctrl_cnt;
    reg [15:0] state_timer_ms;
    reg [15:0] fall_ms;        // WEXD：摆角超阈持续时间（确认用）
    reg [2:0]  restart_cnt;    // WEXD：连续自动重起摆次数
    reg signed [31:0] home_center; // WEXG：固定停泊位（本次上电后第一次稳摆的横杆位置）
    reg signed [31:0] home_snap;   // WEXH：与 home_center 同物理位置、离当前位置最近的等价值
    reg        home_valid;         // WEXG：停泊位已确定
    reg        snap_busy;          // WEXH：等价值正在收敛
    reg [5:0]  snap_cnt;           // WEXH：收敛迭代计数（保护用）
    reg [2:0]  home_div;           // WEXG：归位步进分频计数
    wire       task_idle;
    assign task_idle_o=task_idle;          // WEXG：arm_task 当前无任务
    reg [15:0] swing_total_ms; // 起摆总计时：CATCH 反复进出不清零，仅 READY 清零
    reg signed [31:0] arm_center;

    // 摆角速度滤波流水线
    reg vel_pending;
    reg signed [31:0] vel_inst_r;
    reg signed [31:0] vel_part_r;
    reg [7:0] rej_cnt;

    // 控制计算三级流水线
    reg pipe_busy;
    reg [1:0] pipe_phase;
    reg [3:0] pipe_state;
    reg signed [31:0] s1_a, s1_b, s1_c, s1_d;
    reg signed [31:0] s2_sum;

    reg signed [17:0] angle_delta_inst;
    wire lqr_profile=1'b1; // Shared proven swing/CATCH engine; WT01 substitutes source feedback in BAL.
    wire signed [31:0] lqr_pole_rate,lqr_arm_rate;
    wire lqr_ready;
    reg signed [31:0] lqr_center;
    reg [6:0] lqr_settle;
    wire signed [31:0] lqr_arm_error;
    wire [8:0] catch_blend=state_timer_ms<=100 ? 9'd0 :
                          state_timer_ms>=164 ? 9'd256 : ((state_timer_ms-100)<<2);
    wire signed [31:0] lp,ld,lap,lad;
    wire lqr_done;
    wire track_busy;
    wire wt_mode=(state==ST_BALANCE && profile_id<3);
    wire low_dynamics=(state==ST_BALANCE && profile_id==3);
    // 拓展任务：稳摆中按键下发（S2 选位 / S3 恒速 / S4 正弦）。lqr_center
    // 保持为稳摆入口锁存的横杆零点（不再改写），有效目标=零点+任务偏移。
    // 注意：arm_task 实例化必须放在 control_tick 声明之后（见下方），
    // 否则此处引用会生成无驱动的隐式网线，综合器会把整条控制链当死代码扫掉。
    wire signed [31:0] task_offset;
    wire task_moving;
    wire signed [31:0] lqr_center_eff=lqr_center+task_offset;
    assign arm_target_o=lqr_center_eff;
    assign lqr_arm_error=arm_position-lqr_center_eff;
    assign arm_home_o=home_center; // Read-only telemetry: never fed back to control.
    assign arm_home_valid_o=home_valid;
    wire euro_active=ONE_EURO && euro_mode!=0 && state==ST_BALANCE;
    wire euro_pole_active=euro_active && euro_mode[0];
    wire euro_arm_active=euro_active && euro_mode[1];
    wire euro_p_busy,euro_a_busy,euro_p_ready,euro_a_ready;
    wire lqr_launch=pipe_busy && pipe_phase==0 && pipe_state!=ST_SWING && lqr_profile && !wt_mode && (!low_dynamics || !track_busy) && (!euro_active || (!euro_p_busy && (!euro_arm_active || !euro_a_busy)));
    reg signed [31:0] catch_pd_sum;
    reg signed [31:0] weight_motion,comp_motion,comp_static,catch_legacy;
    reg signed [63:0] blended_command;
    reg signed [23:0] trim_sum;   // 自适应零点：误差累加器
    reg [8:0]  trim_cnt;          // 自适应零点：采样计数
    reg signed [15:0] trim_adj;   // WT01四方案共用-73码实验偏置，不是在线零点学习
    reg signed[15:0] lqr_applied_trim;
    reg signed[17:0] lqr_pole_error;
    assign trim_adj_o = state==ST_BALANCE ? lqr_applied_trim :16'sd0;
    reg signed [23:0] arm_anchor; // 慢弹簧横杆锚定（泄漏积分器）
    // ===== test55 保护状态寄存器 =====
    reg [15:0] runaway_cnt;     // 失速持续计时
    reg [7:0]  osc_cnt;         // 滑窗内有效换向计数
    reg [15:0] osc_window;      // 滑窗计时
    reg osc_soft_once;          // 本轮 BALANCE 是否已软压抖过一次
    reg [15:0] osc_soft_timer;  // 软压抖剩余时间
    reg signed [9:0] prev_cmd;  // 换向检测用
    reg [31:0] i2t_acc;         // 热保护 I²t 累加器
    reg [15:0] decel_timer;     // ST_DECEL 超时兜底
    wire thermal_armed = (state == ST_CATCH) || (state == ST_BALANCE) ||
                         (state == ST_DECEL) || (state == ST_FLIP) || (state == ST_TRANSITION);
    wire signed [31:0] cmd_sq = $signed(motor_cmd) * $signed(motor_cmd);
    // 稳摆换向斜率：软压抖期间收紧到 OSC_SOFT_SLEW
    wire signed [31:0] bal_slew_eff = (osc_soft_timer > 16'd0) ? OSC_SOFT_SLEW : BAL_SLEW_PER_MS;
    reg signed [31:0] calc_cmd;
    reg signed [31:0] swing_mag;
    reg [63:0] ek64;
    reg [31:0] energy_now;
    // TR05: exact legacy energy arithmetic, split before clamps/sign selection.
    // Adds one fabric clock only to original SWING; CATCH/BAL unchanged.
    wire signed[31:0] energy_bottom_gap=ENERGY_TARGET-$signed({1'b0,energy_now});
    wire signed[31:0] energy_top_gap=SERVO_TARGET-$signed({1'b0,energy_now});
    reg signed[31:0] energy_bottom_gap_q,energy_top_gap_q;
    reg signed[43:0] pump_bottom_q,pump_top_q;
    reg swing_energy_ready;
    always @(posedge clk or negedge rst_n)begin
      if(!rst_n)begin
        energy_bottom_gap_q<=0;energy_top_gap_q<=0;pump_bottom_q<=0;pump_top_q<=0;
      end else begin
        energy_bottom_gap_q<=energy_bottom_gap;energy_top_gap_q<=energy_top_gap;
        pump_bottom_q<=(energy_bottom_gap*ENERGY_GAIN)>>>ENERGY_GAIN_SHIFT;
        pump_top_q<=(energy_top_gap*ENERGY_GAIN)>>>ENERGY_GAIN_SHIFT;
      end
    end

    reg signed [31:0] energy_err;
    reg signed [43:0] pump64;
    reg signed [31:0] cap_ramp;
    // 旋转检测：能量代理对转圈态失效（转圈过底仍误判能量不足），
    // 速度同号高速持续 ROT_MS 即判定转圈，全程强制阻尼
    reg [15:0] rot_ms;
    reg [1:0]  rot_sgn;
    wire rotating = (rot_ms >= 16'd1000); // ROT_MS=1000，直接写字面量避免参数位选隐患

    wire control_tick = (ctrl_cnt == CTRL_DIV - 1);
    assign control_tick_o=control_tick;
    // 拓展任务发生器（放在 control_tick 声明之后实例化）
    arm_task #(.CLK_FREQ_HZ(CLK_FREQ_HZ)) task_gen(
        .clk(clk),.rst_n(rst_n),.tick(control_tick),
        .active(state==ST_BALANCE && !return_busy),.cmd_pulse(task_cmd),.cmd_mode(task_mode),
        .cmd_cfg(task_cfg),.stream_enable(stream_enable),.stream_velocity_q16(stream_velocity_q16),
        .offset(task_offset),.moving(task_moving),.idle(task_idle));
    assign adc_fresh_o=angle_online && angle_good_seen && angle_word_legal && adc_age_ms<20;
    wire signed [31:0] arm_error = arm_position - arm_center;
    wire signed [17:0] base_upright_error = circular_delta(angle_safe, angle_up);
    wire signed [17:0] safety_upright_error = PEND_DIR_REVERSE ? -base_upright_error :base_upright_error;
    wire signed [17:0] raw_upright_error = base_upright_error;
    wire signed [17:0] oriented_upright_error = PEND_DIR_REVERSE ?
                                                -raw_upright_error : raw_upright_error;
    wire signed [31:0] oriented_pend_velocity = PEND_DIR_REVERSE ?
                                                -pend_velocity : pend_velocity;

    // Capture the feedback-only reference correction before the multiplier stage.
    // Global swing/catch/angle safety/telemetry error retain the fixed base reference.
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin lqr_applied_trim<=0;lqr_pole_error<=0;end
        else if(recalibrate_pulse)begin lqr_applied_trim<=0;lqr_pole_error<=0;end
        else if(control_tick)begin
            lqr_applied_trim<=state==ST_BALANCE ? trim_adj :16'sd0;
            lqr_pole_error<=oriented_upright_error-(state==ST_BALANCE ?
                          (PEND_DIR_REVERSE ? -trim_adj :trim_adj) :16'sd0);
        end
    end

    // Keep fractional state to avoid a 7-count rounding deadband in the LPF.
    reg signed [35:0] balance_rate_acc;
    // Zero shift is a true bypass, without an additional 1ms register delay.
    // test55：S3(profile>=2) 起改用泄漏低通速度（BAL_RATE_FILTER_SHIFT，7.5ms 时间常数，
    // 削 D 项的采样台阶噪声）；S1/S2 保留原 IIR 速度直通。LS 微分器路由停用，
    // ADC 采样率不再随方案切换（避免与滤波升级混杂变量）
    wire signed [31:0] baseline_balance_rate = (profile_id >= 2'd2 && BAL_RATE_FILTER_SHIFT != 0)
        ? (balance_rate_acc >>> BAL_RATE_FILTER_SHIFT) : oriented_pend_velocity;
    wire signed [31:0] legacy_balance_velocity = baseline_balance_rate;
    wire signed[31:0] wt_request,wt_pp,wt_pd,wt_ap,wt_ad,wt_pole_rate,wt_arm_rate;
    assign balance_velocity=wt_mode ? wt_pole_rate :lqr_profile && (state==ST_CATCH || state==ST_BALANCE) ?
                              lqr_pole_rate : legacy_balance_velocity;
    wire signed[63:0]euro_p_q,euro_a_q;
    wire signed[31:0]euro_p_rate,euro_a_rate;
    wire[7:0]euro_p_cutoff,euro_a_cutoff;
    one_euro_filter #(.PERIOD(26400),.MAX_STEP(400)) euro_pole(
       .clk(clk),.rst_n(rst_n),.enable(ONE_EURO!=0),.valid(adc_fresh_o && !angle_rejected && !recalibrate_pulse),
       .sample(control_tick),.variant(euro_p_quiet ? 2'd2:2'd1),.x({{14{oriented_upright_error[17]}},oriented_upright_error}),
       .busy(euro_p_busy),.ready(euro_p_ready),.done(),.jump(),.overflow(),
       .filtered_q16(euro_p_q),.velocity(euro_p_rate),.cutoff_hz(euro_p_cutoff));
    one_euro_filter #(.PERIOD(0),.MAX_STEP(32),.FAST_MIN(20),.QUIET_MIN(10),.FAST_MAX(80),.QUIET_MAX(40),
       .FAST_BETA_Q24(503316),.QUIET_BETA_Q24(251658),.DERIV_ALPHA_Q20(117058)) euro_arm(
       .clk(clk),.rst_n(rst_n),.enable(ONE_EURO!=0),.valid(adc_fresh_o && !angle_rejected && !recalibrate_pulse),
       .sample(control_tick),.variant(euro_a_quiet ? 2'd2:2'd1),.x(arm_position),
       .busy(euro_a_busy),.ready(euro_a_ready),.done(),.jump(),.overflow(),
       .filtered_q16(euro_a_q),.velocity(euro_a_rate),.cutoff_hz(euro_a_cutoff));
    wire signed [31:0] legacy_lqr_pole_rate,legacy_lqr_arm_rate,track_pole_rate,track_arm_rate;
    wire legacy_lqr_ready,track_ready;
    assign lqr_pole_rate=euro_pole_active ? (PEND_DIR_REVERSE ? -euro_p_rate:euro_p_rate) :low_dynamics ? track_pole_rate :legacy_lqr_pole_rate;
    assign lqr_arm_rate=euro_arm_active ? euro_a_rate:low_dynamics ? track_arm_rate :legacy_lqr_arm_rate;
    assign lqr_ready=legacy_lqr_ready && (!low_dynamics || track_ready) && (!euro_active || (euro_p_ready && (!euro_arm_active || euro_a_ready)));
    lqr_tracking_estimator tracking(.clk(clk),.rst_n(rst_n),.tick(control_tick),
        .recalibrate(recalibrate_pulse),.accepted(angle_accepted),.rejected(angle_rejected),
        .fresh(adc_fresh_o),.angle_code(angle_safe),.arm_position(arm_position),
        .pole_rate(track_pole_rate),.arm_rate(track_arm_rate),.ready(track_ready),.busy(track_busy));
    lqr_rate_estimator rates(.clk(clk),.rst_n(rst_n),.tick(control_tick),
        .mode(state==ST_BALANCE ? 2'd2 : 2'd0),
        .recalibrate(recalibrate_pulse),.accepted(angle_accepted),.rejected(angle_rejected),
        .fresh(adc_fresh_o),.angle_code(angle_safe),.arm_position(arm_position),
        .pole_rate(legacy_lqr_pole_rate),.arm_rate(legacy_lqr_arm_rate),.ready(legacy_lqr_ready));
    lqr_feedback feedback(.clk(clk),.rst_n(rst_n),.launch(lqr_launch),.soft_profile(1'b1),.low_dynamics(low_dynamics),
        .arm_error(lqr_arm_error),.pole_error({{14{lqr_pole_error[17]}},lqr_pole_error}),
        .arm_rate(lqr_arm_rate),.pole_rate(lqr_pole_rate),.done(lqr_done),
        .pole_p(lp),.pole_d(ld),.arm_p(lap),.arm_d(lad));
    wire wt_busy,wt_done,wt_overflow;
    wire wt_launch=pipe_busy && pipe_phase==0 && wt_mode && !wt_busy && (!euro_active || (!euro_p_busy && (!euro_arm_active || !euro_a_busy)));
    wire signed[17:0] wt_measure_error=oriented_upright_error-
                           (PEND_DIR_REVERSE ? -trim_adj :trim_adj);
    wheeltec_balance #(.ONE_EURO(ONE_EURO)) vendor_control(.clk(clk),.rst_n(rst_n),.tick(control_tick),
        .euro_mode(euro_mode),.euro_a_quiet(euro_a_quiet),.euro_pole_q16(euro_p_q-($signed(PEND_DIR_REVERSE ? -trim_adj:trim_adj) * 64'sd65536)),.euro_arm_q16(euro_a_q-$signed(lqr_center_eff)*64'sd65536),
        .launch(wt_launch),.active(wt_mode),.use_lqr(profile_id!=0),.fast_lqr(profile_id==2),
        .pole_error(wt_measure_error),.arm_error(lqr_arm_error),
        .arm_soft(task_moving),
        .busy(wt_busy),.done(wt_done),.overflow(wt_overflow),.request(wt_request),
        .pole_p(wt_pp),.pole_d(wt_pd),.arm_p(wt_ap),.arm_d(wt_ad),
        .pole_rate(wt_pole_rate),.arm_rate(wt_arm_rate));
    wire output_busy,output_done;
    wire signed[31:0]output_compensated;
    wire signed[9:0]output_target,output_command;
    reg signed[31:0] shaped_sum;
    wire notch_busy,notch_done,notch_overflow;
    wire signed[31:0] notch_result;
    wire notch_launch=pipe_busy && pipe_phase==2 && pipe_state!=ST_SWING && lqr_profile && !wt_mode && !notch_busy && !notch_done;
    lqr_notch shaping(.clk(clk),.rst_n(rst_n),.launch(notch_launch),
        .cancel(recalibrate_pulse || !(state==ST_CATCH || state==ST_BALANCE)),
        .active(state==ST_BALANCE),.profile_id(2'd3),.linear(s2_sum),
        .busy(notch_busy),.done(notch_done),.overflow(notch_overflow),.shaped(notch_result));
    wire output_launch=pipe_busy && pipe_phase==3 && pipe_state!=ST_SWING && lqr_profile && !output_busy && !output_done;
    lqr_output_pipeline output_stage(.clk(clk),.rst_n(rst_n),.launch(output_launch),
        .profile_id({1'b0,(profile_id==3)}),
        .cancel(!pipe_busy || !(state==ST_CATCH || state==ST_BALANCE)),
        .is_catch(pipe_state==ST_CATCH),.limit_slew(pipe_state==ST_BALANCE || state_timer_ms>100),
        .linear(shaped_sum),.arm_rate(lqr_arm_rate),.pd_sum(catch_pd_sum),.blend(catch_blend),.current_cmd(motor_cmd),
        .busy(output_busy),.done(output_done),.compensated(output_compensated),.target(output_target),.command(output_command));
    assign lqr_ready_o=lqr_ready;
    assign lqr_arm_rate_o=wt_mode ? wt_arm_rate :lqr_arm_rate;
    // Freeze transition diagnostics across exit; UART50Hz must retain the primary failure.
    reg life_trace_valid;reg[7:0]life_sensor_fault_hold,life_core_fault_hold,life_diag_hold;
    reg[3:0]life_core_state_hold;reg[15:0]life_seam_hold,life_raw_hold,life_good_hold;
    wire life_diag_visible=state==ST_TRANSITION || (life_trace_valid && life_phase==11 && state!=ST_BALANCE && state!=ST_CATCH);
    wire[31:0]life_diag_reason={2'b01,10'd0,life_core_state_hold,life_core_fault_hold,life_sensor_fault_hold};
    wire[31:0]life_diag_ages={life_raw_hold>255?8'hff:life_raw_hold[7:0],life_good_hold>255?8'hff:life_good_hold[7:0],life_seam_hold};
    always @(posedge clk or negedge rst_n)begin
     if(!rst_n)begin life_trace_valid<=0;life_sensor_fault_hold<=0;life_core_fault_hold<=0;life_core_state_hold<=0;life_diag_hold<=0;life_seam_hold<=0;life_raw_hold<=0;life_good_hold<=0;end
     else if(life_fast || return_start)begin life_trace_valid<=1;life_sensor_fault_hold<=0;life_core_fault_hold<=0;life_core_state_hold<=0;life_diag_hold<=0;life_seam_hold<=0;life_raw_hold<=0;life_good_hold<=0;end
     else if(start_pulse)life_trace_valid<=0;
     else if(life_busy)begin
      life_diag_hold<=life_sensor_diag;life_seam_hold<=life_seam_age;life_raw_hold<=life_raw_age;life_good_hold<=life_good_age;
      if(life_sensor_fault!=0 && life_sensor_fault_hold==0)life_sensor_fault_hold<=life_sensor_fault;
      if((state==ST_BRAKE || state==ST_FAULT || state==ST_DECEL) && fault_code!=0 && life_core_fault_hold==0)begin life_core_fault_hold<=fault_code;life_core_state_hold<=state;end
     end
    end
    assign diag_p=life_diag_visible ? $signed(life_diag_reason):state==ST_FLIP ? $signed({24'd0,flip_sensor_diag}):s1_a;
    assign diag_d=life_diag_visible ? $signed(life_diag_ages):state==ST_FLIP ? $signed({16'd0,flip_seam_age}):s1_b;
    assign diag_arm_p=life_diag_visible ? $signed({24'd0,life_diag_hold}):state==ST_FLIP ? $signed({24'd0,flip_sensor_fault}):s1_c;
    assign diag_arm_d=life_diag_visible ? life_rate:state==ST_FLIP ? flip_rate_diag:s1_d;
    assign diag_linear=life_diag_visible ? life_energy:state==ST_FLIP ? flip_energy_diag:(pipe_state==ST_SWING ? s2_sum:shaped_sum);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            balance_rate_acc <= 36'sd0;
        else if (state != ST_BALANCE)
            balance_rate_acc <= $signed({{4{oriented_pend_velocity[31]}}, oriented_pend_velocity}) <<< BAL_RATE_FILTER_SHIFT;
        else if (control_tick)
            balance_rate_acc <= balance_rate_acc
                + $signed({{4{oriented_pend_velocity[31]}}, oriented_pend_velocity})
                - (balance_rate_acc >>> BAL_RATE_FILTER_SHIFT);
    end

    function signed [17:0] circular_delta;
        input [15:0] now_angle;
        input [15:0] ref_angle;
        reg [18:0] delta;
        begin
            delta = $signed({1'b0, now_angle}) - $signed({1'b0, ref_angle});
            if ($signed(delta) > ANGLE_HALF_SCALE)
                delta = $signed(delta) - ANGLE_FULL_SCALE;
            else if ($signed(delta) < -ANGLE_HALF_SCALE)
                delta = $signed(delta) + ANGLE_FULL_SCALE;
            circular_delta = delta[17:0];
        end
    endfunction

    transition_angle_guard motion_sensor(.clk(clk),.rst_n(rst_n),.tick(control_tick),.start(life_fast || return_start),
      .active(life_busy),.raw_valid(angle_valid),.online(angle_online),.raw(angle_raw),.seed(angle_safe),
      .angle(life_angle),.fresh(life_fresh),.waiting(life_wait),.bridge(life_bridge),.fault(life_sensor_fault),.seam_age(life_seam_age),.diagnostic(life_sensor_diag),.raw_age_o(life_raw_age),.good_age_o(life_good_age));
    transition_control lifecycle(.clk(clk),.rst_n(rst_n),.tick(control_tick),.fast_start(life_fast),.return_start(return_start),
      .cancel(return_cancel || brake_pulse || task_cmd),.fresh(life_busy ? life_fresh:adc_fresh_o),
      .seam_wait(life_wait),.seam_bridge(life_bridge),.sensor_fault(life_sensor_fault),.ctrl_state(state),
      .angle(life_angle),.up(angle_up),.default_down(BOOT_DOWN),.arm_position(arm_position),.arm_velocity(arm_velocity),.seed_command(motor_cmd),
      .busy(life_busy),.return_busy(return_busy),.takeover(life_takeover),.capture(life_capture),.parked(life_park),.abort(life_abort),
      .phase(life_phase),.result(life_result),.turns(life_turns),.mode(life_mode),.ident(life_id),.elapsed(life_elapsed),
      .down_ref(life_down),.down_valid(life_down_valid),.command(life_command),.rate_diag(life_rate),.goal_diag(life_goal),.energy_diag(life_energy));
    flip_angle_tracker flip_sensor(.clk(clk),.rst_n(rst_n),.tick(control_tick),.start(flip_start && !flip_busy),
      .active(flip_busy),.negative(flip_negative),.raw_valid(angle_valid),.online(angle_online),
      .raw(angle_raw),.seed(angle_safe),.angle(flip_angle),.fresh(flip_sensor_fresh),
      .waiting(flip_seam_wait),.bridge(flip_bridge),.fault(flip_sensor_fault),.seam_age(flip_seam_age),.diagnostic(flip_sensor_diag));
    flip_maneuver flip_seq(.clk(clk),.rst_n(rst_n),.tick(control_tick),.start(flip_start),
      .cancel(flip_cancel || brake_pulse || task_cmd),.negative(flip_negative),.late(flip_late),.fresh(flip_busy ? flip_sensor_fresh:adc_fresh_o),
      .seam_wait(flip_seam_wait),.seam_bridge(flip_bridge),.sensor_fault(flip_sensor_fault),
      .angle(flip_angle),.up(angle_up),.ctrl_state(state),.arm_position(arm_position),
      .arm_velocity(arm_velocity),.seed_command(motor_cmd),.busy(flip_busy),.takeover(flip_takeover),
      .capture(flip_capture),.abort(flip_abort),.phase(flip_phase),.result(flip_result),
      .ident(flip_id),.elapsed(flip_elapsed),.progress(flip_progress),.command(flip_command),
      .rate_diag(flip_rate_diag),.energy_diag(flip_energy_diag),.goal_diag(flip_goal_diag));
    function [31:0] abs32;
        input signed [31:0] value;
        begin
            abs32 = (value < 0) ? -value : value;
        end
    endfunction

    function [9:0] clamp_pwm;
        input signed [31:0] value;
        begin
            if (value > 255)
                clamp_pwm = 10'sd255;
            else if (value < -255)
                clamp_pwm = -10'sd255;
            else
                clamp_pwm = value[9:0];
        end
    endfunction

    // 1kHz 控制节拍
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            ctrl_cnt <= 32'd0;
        else if (ctrl_cnt >= CTRL_DIV - 1)
            ctrl_cnt <= 32'd0;
        else
            ctrl_cnt <= ctrl_cnt + 1'b1;
    end

    // Guard freshness independently of the reader's online latch.
    // 1kHz logical timer; normal .99ms sample cadence unchanged.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) adc_age_ms <= 16'd0;
        else if (angle_accepted) adc_age_ms <= 16'd0;
        else if (control_tick && adc_age_ms < 16'd1000)
            adc_age_ms <= adc_age_ms + 1'b1;
    end

    // 摆杆角度速度估算：当前读取间隔约 0.99ms；保留 legacy500 比例，非真实 count/s
    // 两拍流水线：angle_accepted 当拍锁存瞬时增量与滤波部分和，下一拍更新输出
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || recalibrate_pulse) begin
            angle_prev <= 16'd0;
            angle_first <= 1'b1;
            pend_velocity <= 32'sd0;
            vel_pending <= 1'b0;
            vel_inst_r <= 32'sd0;
            vel_part_r <= 32'sd0;
            rej_cnt <= 8'd0;
            angle_delta_inst <= 18'sd0;
        end else begin
            if(angle_rejected) begin
                angle_first<=1;pend_velocity<=0;vel_pending<=0;rej_cnt<=0;
            end else if (angle_accepted) begin
                if (angle_first) begin
                    angle_first <= 1'b0;
                    angle_prev <= angle_safe;
                    pend_velocity <= 32'sd0;
                    vel_pending <= 1'b0;
                end else begin
                    angle_delta_inst = circular_delta(angle_safe, angle_prev);
                    // 野值剔除：物理上摆杆 2ms 内不可能超过约 200 counts
                    // （ADS1115 转换寄存器读写撕裂会产生近千 counts 的假跳变）
                    if ((angle_delta_inst > 18'sd300) || (angle_delta_inst < -18'sd300)) begin
                        // 丢弃本样本的速度更新，但必须更新参考位置，
                        // 否则后续样本相对过时参考持续超限，速度计永久锁死；
                        // 连续剔除超过约 500ms 则重置速度估计（兜底自愈）
                        angle_prev <= angle_safe;
                        vel_pending <= 1'b0;
                        if (rej_cnt >= 8'd250) begin
                            rej_cnt <= 8'd0;
                            pend_velocity <= 32'sd0;
                            angle_first <= 1'b1;
                        end else begin
                            rej_cnt <= rej_cnt + 1'b1;
                        end
                    end else begin
                        rej_cnt <= 8'd0;
                        // Legacy scale retained so swing/catch and existing gain magnitudes do not change.
                        vel_inst_r <= angle_delta_inst * 32'sd500;
                        vel_part_r <= (pend_velocity <<< 2) - pend_velocity;
                        vel_pending <= 1'b1;
                        angle_prev <= angle_safe;
                    end
                end
            end else if (vel_pending) begin
                pend_velocity <= (vel_part_r + vel_inst_r) >>> 2;
                vel_pending <= 1'b0;
            end
        end
    end

    // 控制计算三级流水线（在主时序块内推进，避免多驱动 motor_cmd）
    // 第 1 拍：各项乘积/中间量；第 2 拍：求和与方向；第 3 拍：限幅输出

    // 主状态机与控制器
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_CALIBRATE;
            fault_code <= 8'd0;
            cal_sum <= 24'd0;
            cal_count <= 11'd0;
            cal_average <= 16'd0;
            cal_seed <= 16'd0;
            cal_dmin <= 18'sd0;
            cal_dmax <= 18'sd0;
            preserve_reference<=USE_BOOT_REFERENCE;
            lqr_center<=0;lqr_settle<=0;catch_pd_sum<=0;
            trim_sum <= 24'sd0;
            trim_cnt <= 9'd0;
            trim_adj <= 16'sd0;
            arm_anchor <= 24'sd0;
            angle_down <= 16'd0;
            angle_up <= 16'd0;
            upright_error <= 18'sd0;
            arm_center <= 32'sd0;
            state_timer_ms <= 16'd0;
            swing_total_ms <= 16'd0;
            fall_ms <= 16'd0;
            restart_cnt <= 3'd0;
            home_center <= 32'sd0;
            home_snap <= 32'sd0;
            home_valid <= 1'b0;
            snap_busy <= 1'b0;
            snap_cnt <= 6'd0;
            home_div <= 3'd0;
            rot_ms <= 16'd0;
            rot_sgn <= 2'd0;
            motor_cmd <= 10'sd0;
            motor_enable <= 1'b0;
            motor_brake <= 1'b0;
            pipe_busy  <= 1'b0;
            pipe_phase <= 2'd0;
            pipe_state <= ST_READY; swing_energy_ready<=0;
            s1_a <= 32'sd0;
            s1_b <= 32'sd0;
            s1_c <= 32'sd0;
            s1_d <= 32'sd0;
            s2_sum <= 32'sd0; shaped_sum<=0;
            diag_compensated<=0; diag_target<=0;
            runaway_cnt <= 16'd0;
            osc_cnt <= 8'd0;
            osc_window <= 16'd0;
            osc_soft_once <= 1'b0;
            osc_soft_timer <= 16'd0;
            prev_cmd <= 10'sd0;
            i2t_acc <= 32'd0;
            decel_timer <= 16'd0;
        end else begin
            // WT01 common candidate reference; never learns from mean motion.
            // Offset resets outsideBAL, and ramps1ADCcode/ms to -73 only forS2/S4.
            if(state!=ST_BALANCE)trim_adj<=0;
            else if(control_tick)begin
                if(trim_adj > -16'sd73)trim_adj<=trim_adj-16'sd1;
            end
            upright_error <= oriented_upright_error;

            if (recalibrate_pulse) begin
                state<=ST_CALIBRATE; fault_code<=0;
                cal_count<=0; cal_sum<=0; cal_seed<=0; cal_dmin<=0; cal_dmax<=0;
                lqr_center<=0;lqr_settle<=0;catch_pd_sum<=0;
                preserve_reference<=0;
                trim_adj<=0; trim_sum<=0; trim_cnt<=0; arm_anchor<=0;
                state_timer_ms<=0; swing_total_ms<=0; rot_ms<=0; rot_sgn<=0;
                motor_enable<=0; motor_brake<=0; motor_cmd<=0;
                pipe_busy<=0; pipe_phase<=0; shaped_sum<=0; diag_compensated<=0; diag_target<=0;
                runaway_cnt<=0; osc_cnt<=0; osc_window<=0; osc_soft_once<=0;
                osc_soft_timer<=0; prev_cmd<=0; decel_timer<=0;
            end else if (brake_pulse) begin
                state <= ST_BRAKE;
                fault_code <= 8'd0;
                state_timer_ms <= 16'd0;
                motor_cmd <= 10'sd0;
                pipe_busy  <= 1'b0;
                pipe_phase <= 2'd0;
            end else if((state==ST_CATCH || state==ST_BALANCE) && (notch_overflow || (wt_mode && wt_overflow)))begin
                state<=ST_FAULT;fault_code<=10;motor_enable<=0;motor_brake<=0;motor_cmd<=0;pipe_busy<=0;pipe_phase<=0;
            end else if(lqr_profile && (state==ST_BALANCE || (state==ST_CATCH && state_timer_ms>=100)) && !lqr_ready)begin
                state<=ST_FAULT;fault_code<=1;motor_enable<=0;motor_brake<=0;motor_cmd<=0;pipe_busy<=0;pipe_phase<=0;
            end else if ((state == ST_SWING || state == ST_CATCH || state == ST_BALANCE ||
                          state == ST_DECEL || state == ST_FLIP || state == ST_TRANSITION)
                         && (state==ST_FLIP ? !flip_sensor_fresh : state==ST_TRANSITION ? !life_fresh:(!angle_online || adc_age_ms >= 16'd20))) begin
                state <= ST_FAULT;
                fault_code <= 8'd1;
                motor_enable <= 1'b0;
                motor_brake <= 1'b0;
                motor_cmd <= 10'sd0;
                pipe_busy <= 1'b0;
                pipe_phase <= 2'd0;
            end else if (control_tick && thermal_armed && (i2t_acc > THERMAL_MAX)) begin
                // 全局热保护跳闸（CATCH/BALANCE/DECEL 武装；SWING 由 20s 超时兜底）
                state <= ST_BRAKE;
                fault_code <= 8'd8;
                state_timer_ms <= 16'd0;
                pipe_busy <= 1'b0;
                pipe_phase <= 2'd0;
            end else if(life_abort)begin
                state<=ST_BRAKE;if(fault_code==0 || (state!=ST_FAULT && state!=ST_BRAKE && state!=ST_DECEL))fault_code<=12;state_timer_ms<=0;motor_cmd<=0;pipe_busy<=0;pipe_phase<=0;
            end else if(life_park)begin
                state<=ST_READY;fault_code<=0;state_timer_ms<=0;motor_cmd<=0;motor_enable<=0;motor_brake<=0;pipe_busy<=0;pipe_phase<=0;
            end else if(return_start && state==ST_BALANCE)begin
                lqr_center<=arm_position;arm_center<=arm_position;pipe_busy<=0;pipe_phase<=0;arm_anchor<=0;
            end else if(life_takeover)begin
                state<=ST_TRANSITION;arm_center<=arm_position;state_timer_ms<=0;pipe_busy<=0;pipe_phase<=0;
            end else if(life_capture && state==ST_TRANSITION)begin
                state<=ST_CATCH;state_timer_ms<=0;swing_total_ms<=0;lqr_center<=arm_position;lqr_settle<=0;
                arm_anchor<=0;trim_sum<=0;trim_cnt<=0;pipe_busy<=0;pipe_phase<=0;
            end else if(flip_abort)begin
                state<=ST_BRAKE;fault_code<=8'd11;state_timer_ms<=0;
                pipe_busy<=0;pipe_phase<=0;motor_cmd<=0;
            end else if(flip_takeover && state==ST_BALANCE)begin
                state<=ST_FLIP;state_timer_ms<=0;arm_center<=arm_position;
                pipe_busy<=0;pipe_phase<=0;fall_ms<=0;
            end else if(flip_capture && state==ST_FLIP)begin
                state<=ST_CATCH;state_timer_ms<=0;swing_total_ms<=0;
                lqr_center<=arm_position;lqr_settle<=0;pipe_busy<=0;pipe_phase<=0;
                arm_anchor<=0;trim_sum<=0;trim_cnt<=0;
            end else begin
                // 旋转检测计数：速度同号高速持续 ROT_MS 判定转圈；
                // 换向/低速时每次只减 4 而不是清零——单个采样毛刺（test19 t=5.5
                // 曾因一个毛刺导致计数器清零、刹车中断重新泵能）只损失 4ms 积累
                if (control_tick) begin
                    if (oriented_pend_velocity > ROT_RATE) begin
                        if (rot_sgn == 2'd1) begin
                            if (rot_ms < 16'd60000)
                                rot_ms <= rot_ms + 1'b1;
                        end else if (rot_ms > 16'd4) begin
                            rot_ms <= rot_ms - 3'd4;
                        end else begin
                            rot_sgn <= 2'd1;
                            rot_ms  <= 16'd0;
                        end
                    end else if (oriented_pend_velocity < -ROT_RATE) begin
                        if (rot_sgn == 2'd2) begin
                            if (rot_ms < 16'd60000)
                                rot_ms <= rot_ms + 1'b1;
                        end else if (rot_ms > 16'd4) begin
                            rot_ms <= rot_ms - 3'd4;
                        end else begin
                            rot_sgn <= 2'd2;
                            rot_ms  <= 16'd0;
                        end
                    end else begin
                        // 低速（顶点换向附近）：缓慢消减
                        if (rot_ms > 16'd4)
                            rot_ms <= rot_ms - 3'd4;
                        else
                            rot_ms <= 16'd0;
                    end
                end
                if (state == ST_READY) begin
                    rot_ms  <= 16'd0;
                    rot_sgn <= 2'd0;
                end

                // ===== test55 三层保护的后勤计数（每 control_tick）=====
                if (control_tick) begin
                    // I²t 热模型：通电状态累计发热，其余状态自然冷却（每 ms 净衰减 56）；
                    // |cmd|=120 为中性点，180 约 3s 跳闸，255 约 1s 跳闸
                    if (thermal_armed && ((cmd_sq >>> 8) > 32'd56)) begin
                        if (i2t_acc < 32'd300000)
                            i2t_acc <= i2t_acc + ((cmd_sq >>> 8) - 32'd56);
                    end else if (i2t_acc > 32'd56) begin
                        i2t_acc <= i2t_acc - 32'd56;
                    end else begin
                        i2t_acc <= 32'd0;
                    end
                    // 失速持续计时：仅 BALANCE 宽限期后武装
                    if ((state == ST_BALANCE) && (state_timer_ms > BAL_GRACE_MS)) begin
                        if (abs32(arm_velocity) > RUNAWAY_RATE) begin
                            if (runaway_cnt < RUNAWAY_MS)
                                runaway_cnt <= runaway_cnt + 1'b1;
                        end else begin
                            runaway_cnt <= 16'd0;
                        end
                    end else begin
                        runaway_cnt <= 16'd0;
                    end
                    // 抖振滑窗：统计有效换向（换号且新幅值>OSC_MIN_AMP），
                    // 仅 BALANCE 宽限期后武装
                    if ((state == ST_BALANCE) && (state_timer_ms > BAL_GRACE_MS)) begin
                        if (osc_window >= OSC_WINDOW_MS-1) begin
                            osc_window <= 16'd0;
                            osc_cnt <= 8'd0;
                        end else begin
                            osc_window <= osc_window + 1'b1;
                            if ((motor_cmd[9] != prev_cmd[9]) && (prev_cmd != 10'sd0) &&
                                ((motor_cmd > (lqr_profile?80:OSC_MIN_AMP)) || (motor_cmd < -(lqr_profile?80:OSC_MIN_AMP)))) begin
                                if (osc_cnt < 8'd255)
                                    osc_cnt <= osc_cnt + 1'b1;
                            end
                        end
                    end else begin
                        osc_window <= 16'd0;
                        osc_cnt <= 8'd0;
                    end
                    if (osc_soft_timer > 16'd0)
                        osc_soft_timer <= osc_soft_timer - 1'b1;
                    prev_cmd <= motor_cmd;
                end

                // 流水线推进：control_tick 启动后数拍完成 motor_cmd 更新
                // CATCH/BALANCE 3 拍；SWING 能量计算链长，拆成 4 拍
                if (pipe_busy) begin
                    case (pipe_phase)
                        2'd0: begin
                            swing_energy_ready<=0;
                            if (pipe_state == ST_SWING) begin
                                // 能量控制（Åström-Furuta 思路，比例式）：
                                //   势能代理 G = 13200-|err|（底部 0，直立 13200）
                                //   动能代理 E_kin = vel²/400000 ≈ ((|vel|>>>5)² × 2684) >>> 20
                                //   （系数按实测重新校准：旧刻度高速低估约 30%，
                                //     转圈时误判能量不足继续泵能，是过顶加速的根因）
                                //   恰好到达直立点所需总能量 = 13200
                                // cmd = sign(vel) × clamp((TARGET−E)×0.08)，缺口大满幅泵、
                                // 接近目标自动收小、超额自动反向刹车，收敛而非极限环振荡
                                // 本拍只算能量并寄存，泵幅下一拍再算（时序）
                                ek64 = ((abs32(oriented_pend_velocity) >>> 5) *
                                        (abs32(oriented_pend_velocity) >>> 5) * 32'd2684) >>> 20;
                                energy_now <= (32'd13200 - abs32(oriented_upright_error))
                                              + ek64[31:0];

                                // 摆臂居中：旋转刹车期间适度加强（>>>3），
                                // 防止持续单向刹车把摆臂推到限位（test24/25 撞限位）；
                                // 注意 >>>2 会与刹车方向拉锯导致摆臂高频抽动异响；
                                // 常规起摆用 SWING_CENTER_SHIFT
                                if (rotating)
                                    s1_b <= +(arm_error >>> 3);
                                else
                                    s1_b <= +(arm_error >>> SWING_CENTER_SHIFT);
                                s1_c <= +(arm_error >>> 10);
                                // 低速死区只在底部带生效：顶部伺服区（|err|<PUMP_BAND）
                                // 正是伺服轻推摆杆爬最后一段的发力处，低速掐断会让
                                // 摆杆一致停在捕获窗外沿（test29：8 次捕获全在 2475~2809）
                                if (abs32(oriented_pend_velocity) > SWING_MIN_RATE)
                                    s1_d <= 32'sd0;
                                else if (abs32(oriented_upright_error) < PUMP_BAND)
                                    s1_d <= 32'sd0;
                                else
                                    s1_d <= 32'sd1;
                            end else if (pipe_state == ST_CATCH) begin
                                s1_a <= (CATCH_KP_ANGLE * oriented_upright_error) >>> CATCH_KP_SHIFT;
                                s1_b <= (CATCH_KD_ANGLE * oriented_pend_velocity) >>> CATCH_KD_SHIFT;
                                // 捕获期横杆用弱增益（自由），与稳摆强锚定分开
                                s1_c <= +(CATCH_KP_ARM * arm_error) >>> CATCH_KP_ARM_SHIFT;
                                s1_d <= +(CATCH_KD_ARM * arm_velocity) >>> CATCH_KD_ARM_SHIFT;
                            end else begin
                                // ST_BALANCE
                                s1_a <= (BAL_KP_ANGLE * oriented_upright_error) >>> BAL_KP_SHIFT;
                                s1_b <= (BAL_KD_ANGLE * balance_velocity) >>> BAL_KD_SHIFT;
                                if (BAL_ARM_REVERSE) begin
                                    s1_c <= -((BAL_KP_ARM * arm_error) >>> BAL_KP_ARM_SHIFT);
                                    s1_d <= -((BAL_KD_ARM * arm_velocity) >>> BAL_KD_ARM_SHIFT);
                                end else begin
                                    s1_c <= (BAL_KP_ARM * arm_error) >>> BAL_KP_ARM_SHIFT;
                                    s1_d <= (BAL_KD_ARM * arm_velocity) >>> BAL_KD_ARM_SHIFT;
                                end
                            end
                            if((!low_dynamics || !track_busy || pipe_state==ST_SWING) &&
                               (!euro_active || (!euro_p_busy && (!euro_arm_active || !euro_a_busy))))pipe_phase <= 2'd1;
                        end
                        2'd1: begin
                          if(pipe_state==ST_SWING && !swing_energy_ready)begin swing_energy_ready<=1;end else begin
                            if (pipe_state == ST_SWING) begin
                                // 第 2 拍：转圈 → 满功率强刹；
                                // 底部带（|err|>9000）→ 能量泵摆（斜坡上限，粗调）；
                                // 顶部区（|err|<9000）→ 能量伺服（SERVO_CAP 小上限精调），
                                // 把能量精确伺候到 SERVO_TARGET，顶点停在离顶约 1.5°，
                                // 让捕获发生在近垂直处（test28：窗沿 35° 停滞物理上接不住）
                                if (rotating) begin
                                    // 转圈强刹：放宽到满功率 255（物理上旋转动能只能
                                    // 通过转轴耦合缓慢消耗，此为兜底，关键仍是预防）
                                    pump64 = -((oriented_pend_velocity * NEAR_TOP_DAMP_MUL) >>> 16);
                                    if (pump64 > 255)
                                        pump64 = 255;
                                    else if (pump64 < -255)
                                        pump64 = -255;
                                    s1_a <= pump64[31:0];
                                end else begin
                                    if (abs32(oriented_upright_error) > PUMP_BAND) begin
                                        // 底部带泵能：上限随时间渐进（起步 70 每秒 +8），
                                        // 精细段缺口 <1500 软限 45（test24 一炮打过顶）
                                        energy_err = energy_bottom_gap_q;
                                        pump64 = pump_bottom_q;
                                        if ((energy_err > 0) && (energy_err < 1500)) begin
                                            if (pump64 > 45)
                                                pump64 = 45;
                                        end
                                        cap_ramp = 32'sd70 + {16'd0, state_timer_ms[15:7]};
                                        if (cap_ramp > SWING_MAX_DUTY)
                                            cap_ramp = SWING_MAX_DUTY;
                                    end else begin
                                        // 顶部区能量伺服：差则沿速度轻推，多则轻刹
                                        energy_err = energy_top_gap_q;
                                        pump64 = pump_top_q;
                                        cap_ramp = SERVO_CAP;
                                    end
                                    if (pump64 > cap_ramp)
                                        pump64 = cap_ramp;
                                    else if (pump64 < -cap_ramp)
                                        pump64 = -cap_ramp;
                                    if (oriented_pend_velocity >= 0)
                                        s1_a <= pump64[31:0];
                                    else
                                        s1_a <= -pump64[31:0];
                                end
                            end else if(wt_mode)begin
                                if(wt_done)begin
                                    s1_a<=wt_pp;s1_b<=wt_pd;s1_c<=wt_ap;s1_d<=wt_ad;
                                    s2_sum<=wt_request;shaped_sum<=wt_request;pipe_phase<=2'd2;
                                end
                            end else if(lqr_profile)begin
                                if(lqr_done)begin
                                    catch_pd_sum<=s1_a+s1_b+s1_c+s1_d;
                                    s1_a<=lp;s1_b<=ld;s1_c<=lap;s1_d<=lad;
                                    s2_sum<=lp+ld+lap+lad;
                                    pipe_phase<=2'd2;
                                end
                            end else begin
                                // CATCH/BALANCE：角度项取正号（负反馈）。
                                // test28/29 的 100Hz 数据铁证：捕获时摆杆近乎静止，
                                // cmd=+255 满功率输出 60ms，摆臂以约 4g 移动，摆杆
                                // 却以 52 rev/s² 坠落——38° 自由落体仅 9.2 rev/s²。
                                // 坠落达重力 5.6 倍的唯一解释：摆臂在把转轴从摆杆下
                                // "抽走"（正反馈）。此前 test07"取负=负反馈"的校准结论
                                // 是错的，当时发散实因增益过高振荡。此错误符号正是
                                // test20/21/26/28/29 全部捕获失败的根因。
                                // 摆臂居中/阻尼项 s1_c/s1_d 保持正号不动（独立验证过：
                                // cmd>0 → 编码器 pos 减小）。
                                calc_cmd = (s1_a + s1_b) + (s1_c + s1_d);
                                // 慢弹簧锚定项：只对持续漂移发力，等效 arm_error/16
                                if (pipe_state == ST_BALANCE && ARM_ANCHOR_ENABLE
                                    && (profile_id >= 2'd1)) begin
                                    if (BAL_ARM_REVERSE)
                                        calc_cmd = calc_cmd - (arm_anchor >>> ARM_ANCHOR_SHIFT);
                                    else
                                        calc_cmd = calc_cmd + (arm_anchor >>> ARM_ANCHOR_SHIFT);
                                end
                                if (BALANCE_REVERSE)
                                    calc_cmd = -calc_cmd;
                                s2_sum <= calc_cmd;
                            end
                            if(pipe_state==ST_SWING || !lqr_profile)pipe_phase <= 2'd2;
                        end
                        end
                        2'd2: begin
                            if (pipe_state == ST_SWING) begin
                                // SWING 第 3 拍：合成；低速时只保留摆臂居中
                                if (s1_d != 32'sd0)
                                    s2_sum <= s1_c;
                                else
                                    s2_sum <= s1_a + s1_b;
                                pipe_phase <= 2'd3;
                            end else begin
                                // CATCH/BALANCE 第 3 拍：死区补偿后输出并结束。
                                // 电机占空比低于约 38 不出力：小角度时 PD 输出仅十几，
                                // 等效无反馈（test20 摆杆已在 2.8° 立住却坠落）。
                                // 非零命令叠加 ±DEADZONE_COMP 基值，使其真正推动电机
                                if(wt_mode)begin
                                    // Vendor steady control: no inherited friction compensation or notch.
                                    // Final clamp is applied AFTER forming the new request.
                                    diag_compensated<=s2_sum;
                                    calc_cmd=s2_sum>180 ? 180 :s2_sum < -180 ? -180 :s2_sum;
                                    diag_target<=calc_cmd[9:0];
                                    if(calc_cmd>$signed(motor_cmd)+32'sd12)motor_cmd<=motor_cmd+10'sd12;
                                    else if(calc_cmd<$signed(motor_cmd)-32'sd12)motor_cmd<=motor_cmd-10'sd12;
                                    else motor_cmd<=calc_cmd[9:0];
                                    pipe_busy<=0;pipe_phase<=0;
                                end else if(lqr_profile)begin
                                    if(notch_done)begin
                                        shaped_sum<=notch_result;
                                        pipe_phase<=2'd3;
                                    end
                                end else begin
                                    calc_cmd=s2_sum;
                                if (pipe_state == ST_BALANCE) begin
                                    // test55 新死区策略（四方案共享）：
                                    // |u|<DEADZONE_QUIET 直接输出 0——零点附近安静滑行，
                                    // 消除 ±40 突变补偿的继电器型极限环（高频抖动主源）；
                                    // 跨过门槛后斜坡补偿，|u|=64 处补满 ±40
                                    if ((calc_cmd > 0) && (calc_cmd < DEADZONE_QUIET))
                                        calc_cmd = 0;
                                    else if ((calc_cmd < 0) && (calc_cmd > -DEADZONE_QUIET))
                                        calc_cmd = 0;
                                    else if (calc_cmd > 0 && calc_cmd < 64)
                                        calc_cmd = calc_cmd + ((calc_cmd * DEADZONE_COMP) >>> 6);
                                    else if (calc_cmd < 0 && calc_cmd > -64)
                                        calc_cmd = calc_cmd - (((-calc_cmd) * DEADZONE_COMP) >>> 6);
                                    else if (calc_cmd >= 64)
                                        calc_cmd = calc_cmd + DEADZONE_COMP;
                                    else if (calc_cmd <= -64)
                                        calc_cmd = calc_cmd - DEADZONE_COMP;
                                end else if (calc_cmd > 0)
                                    calc_cmd = calc_cmd + DEADZONE_COMP;
                                else if (calc_cmd < 0)
                                    calc_cmd = calc_cmd - DEADZONE_COMP;
                                // BALANCE: bound reversals AFTER deadzone compensation and
                                // saturation. Stop/fault paths bypass this limiter immediately.
                                diag_compensated<=calc_cmd;
                                calc_cmd = $signed(clamp_pwm(calc_cmd));
                                diag_target<=calc_cmd[9:0];
                                if (pipe_state == ST_BALANCE && !lqr_profile &&
                                    calc_cmd > ($signed(motor_cmd) + bal_slew_eff))
                                    motor_cmd <= motor_cmd + bal_slew_eff[9:0];
                                else if (pipe_state == ST_BALANCE && !lqr_profile &&
                                    calc_cmd < ($signed(motor_cmd) - bal_slew_eff))
                                    motor_cmd <= motor_cmd - bal_slew_eff[9:0];
                                else
                                    motor_cmd <= calc_cmd[9:0];
                                pipe_busy  <= 1'b0;
                                pipe_phase <= 2'd0;
                                end
                            end
                        end
                        default: begin
                            if(pipe_state!=ST_SWING && lqr_profile)begin
                                if(output_done)begin
                                    diag_compensated<=output_compensated;
                                    diag_target<=output_target;
                                    motor_cmd<=output_command;
                                    pipe_busy<=0;pipe_phase<=0;
                                end
                            end else begin
                            // SWING 第 4 拍：换向斜率限制后输出并结束。
                            // 本输出级每个 control_tick 只执行一次，±16/拍 ≈ 16000/s，
                            // 全反向约 32ms——抑制 ±160 瞬时换向的减速箱撞击声，
                            // 对 kHz 级控制带宽几乎无影响
                            calc_cmd = s2_sum;
                            if (calc_cmd > 32'sd255)
                                calc_cmd = 32'sd255;
                            else if (calc_cmd < -32'sd255)
                                calc_cmd = -32'sd255;
                            if (calc_cmd > ($signed(motor_cmd) + 32'sd16))
                                motor_cmd <= motor_cmd + 10'sd16;
                            else if (calc_cmd < ($signed(motor_cmd) - 32'sd16))
                                motor_cmd <= motor_cmd - 10'sd16;
                            else
                                motor_cmd <= calc_cmd[9:0];
                            pipe_busy  <= 1'b0;
                            pipe_phase <= 2'd0;
                            end
                        end
                    endcase
                end

                case (state)
                    ST_CALIBRATE: begin
                        motor_enable <= 1'b0;
                        motor_brake <= 1'b0;
                        motor_cmd <= 10'sd0;

                        if (!angle_online || adc_age_ms>=20) begin
                            // ADS1115 上电初始化需要约 50ms，等待其上线；
                            // 超过 CALIB_WAIT_MS 仍无应答才判定为故障
                            if (control_tick) begin
                                if (state_timer_ms >= CALIB_WAIT_MS - 1) begin
                                    state <= ST_FAULT;
                                    fault_code <= 8'd1; // ADS1115 未应答或 I2C 异常
                                end else begin
                                    state_timer_ms <= state_timer_ms + 1'b1;
                                end
                            end
                        end else begin
                            state_timer_ms <= 16'd0;
                            if (angle_accepted) begin
                                if (cal_count == 11'd0) begin
                                    // 首个样本作摆动参考种子（此时摆杆近似下垂）
                                    cal_seed <= angle_safe;
                                    cal_dmin <= 18'sd0;
                                    cal_dmax <= 18'sd0;
                                end else begin
                                    cal_d = circular_delta(angle_safe, cal_seed);
                                    // 只统计 ±62° 内样本，防缠绕过界/野值污染极值
                                    if ((cal_d < 18'sd9000) && (cal_d > -18'sd9000)) begin
                                        if (cal_d < cal_dmin)
                                            cal_dmin <= cal_d;
                                        if (cal_d > cal_dmax)
                                            cal_dmax <= cal_d;
                                    end
                                end
                                cal_count <= cal_count + 1'b1;

                                if (cal_count == CALIB_SAMPLES - 1) begin
                                    // 窗口极值中点；四方案均保留原标定算法
                                    cal_mid = (cal_dmin + cal_dmax) >>> 1;
                                    cal_down32 = $signed({16'd0, cal_seed}) + cal_mid;
                                    if (cal_down32 >= ANGLE_FULL_SCALE)
                                        cal_down32 = cal_down32 - ANGLE_FULL_SCALE;
                                    else if (cal_down32 < 0)
                                        cal_down32 = cal_down32 + ANGLE_FULL_SCALE;
                                    if(!preserve_reference && (cal_down32<18000 || cal_down32>23000))begin
                                        state<=ST_FAULT;fault_code<=9;motor_enable<=0;motor_brake<=0;motor_cmd<=0;
                                    end else begin
                                    angle_down <= cal_down32[15:0];
                                    // 竖直零点 = 下垂对径位置 - UPRIGHT_TRIM（实测修正，
                                    // 手持物理倒立时 err 应为 0；见 UPRIGHT_TRIM 注释）
                                    cal_up = cal_down32[15:0] + ANGLE_HALF_SCALE[15:0];
                                    if (cal_up >= ANGLE_FULL_SCALE[15:0])
                                        cal_up = cal_up - ANGLE_FULL_SCALE[15:0];
                                    if (cal_up >= UPRIGHT_TRIM[15:0])
                                        angle_up <= cal_up - UPRIGHT_TRIM[15:0];
                                    else
                                        angle_up <= cal_up + ANGLE_FULL_SCALE[15:0]
                                                    - UPRIGHT_TRIM[15:0];
                                    if(preserve_reference)begin
                                        // Preserve the measured LQ04 reference for all four comparisons.
                                        // Explicit idle long-S0 recalibration restores the original formula.
                                        angle_down<=BOOT_DOWN;angle_up<=BOOT_UP;
                                    end
                                    // No online trim learning in this LQR branch.
                                    trim_adj <= 16'sd0;
                                    trim_sum <= 24'sd0;
                                    trim_cnt <= 9'd0;
                                    state <= ST_READY;
                                    end
                                end
                            end
                        end
                    end

                    ST_READY: begin
                        motor_enable <= 1'b0;
                        motor_brake <= 1'b0;
                        motor_cmd <= 10'sd0;
                        state_timer_ms <= 16'd0;
                        swing_total_ms <= 16'd0;

                        // 热保护冷却未完成时禁止启动；手转横杆可自检失速检测（显示 fault=7）
                        if (start_pulse && angle_online && adc_age_ms < 16'd20 &&
                            (i2t_acc < THERMAL_RESUME)) begin
                            arm_center <= arm_position;
                            fault_code<=0;
                            restart_cnt <= 3'd0;   // WEXD：手动起摆重新计数
                            fall_ms <= 16'd0;
                            state <= start_fast ? ST_TRANSITION:ST_SWING;
                        end else if (abs32(arm_velocity) > RUNAWAY_RATE) begin
                            fault_code <= 8'd7; // 自检指示：不上电机电也能验证失速检测活着
                        end
                    end

                    ST_SWING: begin
                        motor_enable <= 1'b1;
                        motor_brake <= 1'b0;

                        if (control_tick) begin
                            state_timer_ms <= state_timer_ms + 1'b1;
                            swing_total_ms <= swing_total_ms + 1'b1;

                            if (!angle_online) begin
                                state <= ST_FAULT;
                                fault_code <= 8'd1;
                            end else if (ARM_LIMIT_ENABLE && (abs32(arm_error) > ARM_LIMIT)) begin
                                state <= ST_BRAKE;
                                state_timer_ms <= 16'd0;
                                pipe_busy <= 1'b0;
                                fault_code <= 8'd2; // 摆臂行程超限
                            end else if ((abs32(oriented_upright_error) < CATCH_ANGLE) &&
                                         (abs32(oriented_pend_velocity) < CATCH_RATE)) begin
                                state <= ST_CATCH;
                                state_timer_ms <= 16'd0;
                                lqr_center<=arm_position;lqr_settle<=0;
                                // WEXG：上电后第一次稳定捕获的位置，即为固定停泊位
                                // WEXH：home_snap 从停泊位出发，后续按整圈收敛到离当前位置最近的等价值
                                if (!home_valid) begin
                                    home_center <= arm_position;
                                    home_snap  <= arm_position;  // 首次：停泊位就是这里，无需移动
                                    home_valid <= 1'b1;
                                end else begin
                                    home_snap  <= home_center;    // 之后：从停泊位出发取模
                                end
                                snap_busy <= 1'b1;
                                snap_cnt <= 6'd0;
                            end else if (swing_total_ms >= SWING_TIMEOUT_MS) begin
                                // 用总计时器判定：CATCH 反复进出不会重置它
                                state <= ST_BRAKE;
                                state_timer_ms <= 16'd0;
                                pipe_busy <= 1'b0;
                                fault_code <= 8'd3; // 起摆超时
                            end else if ((state_timer_ms < AGITATE_MS) &&
                                         (abs32(oriented_pend_velocity) <= SWING_MIN_RATE)) begin
                                // 起摆初期激励：摆杆接近静止时速度泵存在死区，
                                // 用固定幅值方波（每 256ms 换向，接近摆杆固有频率约 1.5Hz）
                                // 把摆杆从静止晃起来，同时带轻度摆臂居中修正；
                                // 幅值用 AGITATE_DUTY=70 而非满幅——test17/18 中 ±160 的
                                // 第一下就把摆杆抽过顶（1 秒冲顶），必须柔起
                                if (state_timer_ms[8])
                                    calc_cmd = -AGITATE_DUTY;
                                else
                                    calc_cmd = AGITATE_DUTY;
                                // 实测极性：cmd>0 使编码器 pos 减小，居中修正取正号
                                calc_cmd = calc_cmd + (arm_error >>> 7);
                                motor_cmd <= clamp_pwm(calc_cmd);
                            end else begin
                                // 摆杆已动起来：沿速度方向泵能，三级流水线计算
                                pipe_busy  <= 1'b1;
                                pipe_phase <= 2'd0;
                                pipe_state <= ST_SWING;
                            end
                        end
                    end

                    ST_CATCH: begin
                        motor_enable <= 1'b1;
                        motor_brake <= 1'b0;
                        if(control_tick)begin
                            swing_total_ms<=swing_total_ms+1'b1;
                            if(lqr_ready && state_timer_ms>=164 && abs32(oriented_upright_error)<733
                                && abs32(lqr_pole_rate)<10000 && abs32(lqr_arm_rate)<800)begin
                                if(lqr_settle<40)lqr_settle<=lqr_settle+1'b1;
                            end else lqr_settle<=0;
                        end

                        if (control_tick) begin
                            state_timer_ms <= state_timer_ms + 1'b1;
                            swing_total_ms <= swing_total_ms + 1'b1;

                            if (!angle_online) begin
                                state <= ST_FAULT;
                                fault_code <= 8'd1;
                            end else if (ARM_LIMIT_ENABLE && (abs32(arm_error) > ARM_LIMIT)) begin
                                state <= ST_BRAKE;
                                state_timer_ms <= 16'd0;
                                pipe_busy <= 1'b0;
                                fault_code <= 8'd2;
                            end else if (swing_total_ms >= SWING_TIMEOUT_MS) begin
                                // 起摆总超时在捕获态同样生效，防 SWING/CATCH 循环兜底
                                state <= ST_BRAKE;
                                state_timer_ms <= 16'd0;
                                pipe_busy <= 1'b0;
                                fault_code <= 8'd3;
                            end else if(swing_total_ms>=SWING_TIMEOUT_MS)begin
                                state<=ST_BRAKE;state_timer_ms<=0;pipe_busy<=0;fault_code<=3;
                            end else if (abs32(oriented_upright_error) > CATCH_EXIT_ANGLE) begin
                                state <= ST_SWING;
                                state_timer_ms <= 16'd0;
                            end else if (state_timer_ms >= CATCH_MAX_MS) begin
                                // test60：等不到温和交付，退回 SWING 重新泵一轮
                                state <= ST_SWING;
                                state_timer_ms <= 16'd0;
                            end else if ((lqr_profile && lqr_settle>=40) ||
                                         (!lqr_profile && state_timer_ms>=CATCH_HOLD_MS)) begin
                                state <= ST_BALANCE;
                                // Same physical hold target for source algorithms and local references.
                                lqr_center<=arm_position;
                                // WEXG：上电后第一次捕获的位置即为固定停泊位；之后不再改，
                                // 后续每次重起摆都靠 BALANCE 内的慢速归位滑回来
                                // WEXH：home_snap 从停泊位出发并按整圈收敛（归位路线≤半圈）
                                if (!home_valid) begin
                                    home_center <= arm_position;
                                    home_snap  <= arm_position;
                                    home_valid <= 1'b1;
                                end else begin
                                    home_snap  <= home_center;
                                end
                                snap_busy <= 1'b1;
                                snap_cnt <= 6'd0;
                                state_timer_ms <= 16'd0;
                            end else begin
                                pipe_busy  <= 1'b1;
                                pipe_phase <= 2'd0;
                                pipe_state <= ST_CATCH;
                                // 捕获期间清零累加器（保留 trim_adj 学习成果），
                                // 进 BALANCE 重新统计
                                trim_sum <= 24'sd0;
                                trim_cnt <= 9'd0;
                                arm_anchor <= 24'sd0; // 慢弹簧每次进稳摆重新积累
                                osc_soft_once <= 1'b0; // 每轮捕获给一次软压抖自愈机会
                            end
                        end
                    end

                    ST_BALANCE: begin
                        motor_enable <= 1'b1;
                        motor_brake <= 1'b0;

                        if (control_tick) begin
                            // 饱和计数：拓展任务不限时（BALANCE_TRIAL_MS 顶层调到
                            // 600000，32 位比较永不成立），防 16 位回绕解除保护武装
                            if (state_timer_ms < 16'hFFFF)
                                state_timer_ms <= state_timer_ms + 1'b1;
                            if (state_timer_ms >= BALANCE_TRIAL_MS) begin
                                state <= ST_BRAKE;
                                state_timer_ms <= 16'd0;
                                fault_code <= 8'd5; // finite trial complete, not angle failure
                                pipe_busy <= 1'b0;
                            end else if (!angle_online) begin
                                state <= ST_FAULT;
                                fault_code <= 8'd1;
                            end else if (ARM_LIMIT_ENABLE && (abs32(arm_position - lqr_center_eff) > ARM_LIMIT)) begin
                                // 拓展任务中横杆按目标移动，行程保护跟踪"目标跟随误差"
                                // 而非固定零点；目标本身走多远都不误判，跟不上才刹车
                                state <= ST_BRAKE;
                                state_timer_ms <= 16'd0;
                                pipe_busy <= 1'b0;
                                fault_code <= 8'd2;
                            end else if (abs32(safety_upright_error) > RESTART_ANGLE
                                         && (state_timer_ms > BAL_GRACE_MS)) begin
                                // WEXD 大扰动自救：摆角连续超阈 150ms → 判为倒下，
                                // 退出 BALANCE 回 SWING 重新起摆（不再是刹车结束）。
                                if (fall_ms >= RESTART_CONFIRM_MS) begin
                                    if (restart_cnt >= RESTART_MAX) begin
                                        state <= ST_BRAKE;      // 连续失败太多次：兜底刹车
                                        state_timer_ms <= 16'd0;
                                        pipe_busy <= 1'b0;
                                        fault_code <= 8'd4;
                                    end else begin
                                        restart_cnt <= restart_cnt + 1'b1;
                                        arm_center <= arm_position; // 摆臂居中参考跟随当前位置
                                        state <= ST_SWING;
                                        state_timer_ms <= 16'd0;
                                        swing_total_ms <= 16'd0;    // 重新给满起摆超时
                                        fall_ms <= 16'd0;
                                        pipe_busy <= 1'b0;
                                        fault_code <= 8'd0;
                                    end
                                end else begin
                                    fall_ms <= fall_ms + 1'b1;
                                end
                            end else if (abs32(safety_upright_error) > FAIL_ANGLE) begin
                                state <= ST_BRAKE;
                                state_timer_ms <= 16'd0;
                                pipe_busy <= 1'b0;
                                fault_code <= 8'd4; // 稳摆角度超限
                            end else if (runaway_cnt >= RUNAWAY_MS) begin
                                // 失速保护 L1：高速旋转禁止直接短路刹车（满速刹车能量
                                // 全砸驱动内阻，烧 TB6612 的典型工况）→ 先主动反驱减速
                                state <= ST_DECEL;
                                state_timer_ms <= 16'd0;
                                decel_timer <= 16'd0;
                                pipe_busy <= 1'b0;
                            end else if (osc_cnt > OSC_REVERSES) begin
                                if ((osc_soft_timer == 16'd0) && !osc_soft_once) begin
                                    // 抖振保护 L1：软压抖——换向斜率收紧，给一次自愈机会
                                    osc_soft_timer <= OSC_SOFT_MS;
                                    osc_soft_once <= 1'b1;
                                    osc_cnt <= 8'd0;
                                    osc_window <= 16'd0;
                                end else begin
                                    // 软压抖后仍抖：刹车保护
                                    state <= ST_BRAKE;
                                    state_timer_ms <= 16'd0;
                                    pipe_busy <= 1'b0;
                                    fault_code <= 8'd6; // 高频抖振保护
                                end
                            end else begin
                                fall_ms <= 16'd0;   // 回到阈值内：确认计时清零
                                if (state_timer_ms > RESTART_OK_MS)
                                    restart_cnt <= 3'd0;

                                // WEXG/WEXH 零点归位（已取模）：
                                //   home_snap = 与 home_center 物理同一位置、但离当前位置最近
                                //               的等价值（差一圈就整圈平移），因此归位路线永远
                                //               不超过半圈，不会绕好几圈。
                                //   先收敛 home_snap（每毫秒整圈平移一次，最多 63 ms）；
                                //   再以每 HOME_SLEW_DIV 毫秒 1 计数的速度把零点滑过去。
                                if (snap_busy && !return_busy) begin
                                    if (home_snap - arm_position > ARM_HALF_REV) begin
                                        home_snap <= home_snap - ARM_COUNTS_PER_REV;
                                        snap_cnt  <= snap_cnt + 1'b1;
                                    end else if (home_snap - arm_position < -ARM_HALF_REV) begin
                                        home_snap <= home_snap + ARM_COUNTS_PER_REV;
                                        snap_cnt  <= snap_cnt + 1'b1;
                                    end else begin
                                        snap_busy <= 1'b0;   // 已在半圈以内
                                    end
                                    if (snap_cnt >= 6'd63) snap_busy <= 1'b0; // 保护
                                end else if (home_valid && task_idle && !snap_busy && !return_busy &&
                                    (state_timer_ms > BAL_HOME_DELAY_MS) &&
                                    (abs32(safety_upright_error) < HOME_GATE_ANGLE) &&
                                    (lqr_center != home_snap)) begin
                                    if (home_div == (HOME_SLEW_DIV-1)) begin
                                        home_div <= 3'd0;
                                        if (lqr_center < home_snap)
                                            lqr_center <= lqr_center + 1'b1;
                                        else
                                            lqr_center <= lqr_center - 1'b1;
                                    end else begin
                                        home_div <= home_div + 1'b1;
                                    end
                                end else begin
                                    home_div <= 3'd0;
                                end

                                pipe_busy  <= 1'b1;
                                pipe_phase <= 2'd0;
                                pipe_state <= ST_BALANCE;
                                // 慢弹簧锚定（S2 起启用，且加门控）：只跟踪半秒以上的
                                // 横杆持续漂移，对摆杆高速纠正透明；|err| 超门限时冻结
                                // ——积分分离：纠正动作期间回中完全闭嘴（避坑指南策略1）
                                if (ARM_ANCHOR_ENABLE && (profile_id >= 2'd1)) begin
                                    if (abs32(oriented_upright_error) < ANCHOR_GATE_ANGLE)
                                        arm_anchor <= arm_anchor + arm_error
                                                      - (arm_anchor >>> ARM_ANCHOR_LEAK);
                                end else begin
                                    arm_anchor <= 24'sd0;
                                end
                            end
                        end
                    end

                    ST_TRANSITION:begin
                        motor_enable<=1;motor_brake<=0;pipe_busy<=0;
                        if(control_tick)begin motor_cmd<=life_command;diag_compensated<=life_goal;end
                    end
                    ST_FLIP:begin
                        motor_enable<=1;motor_brake<=0;pipe_busy<=0;
                        if(control_tick)begin
                            motor_cmd<=flip_command;
                            
                            
                            diag_compensated<=flip_goal_diag;
                        end
                    end

                    ST_DECEL: begin
                        // 失速保护 L1：主动反驱减速（斜率 ±16/ms 逼近目标，避免电流冲击），
                        // 转速降到安全区后转短路刹车；超时压不下来同样强制短路刹车兜底
                        motor_enable <= 1'b1;
                        motor_brake <= 1'b0;

                        if (control_tick) begin
                            decel_timer <= decel_timer + 1'b1;
                            if ((abs32(arm_velocity) < DECEL_DONE_RATE) ||
                                (decel_timer >= DECEL_TIMEOUT_MS)) begin
                                state <= ST_BRAKE;
                                state_timer_ms <= 16'd0;
                                fault_code <= 8'd7; // 失速旋转保护
                            end else begin
                                // T60R: positive command decreases measured encoder position.
                                // Therefore braking torque uses the SAME measured velocity sign.
                                calc_cmd = (arm_velocity >>> 4);
                                if (calc_cmd > 200)
                                    calc_cmd = 200;
                                else if (calc_cmd < -200)
                                    calc_cmd = -200;
                                if (calc_cmd > ($signed(motor_cmd) + 16))
                                    motor_cmd <= motor_cmd + 10'sd16;
                                else if (calc_cmd < ($signed(motor_cmd) - 16))
                                    motor_cmd <= motor_cmd - 10'sd16;
                                else
                                    motor_cmd <= calc_cmd[9:0];
                            end
                        end
                    end

                    ST_BRAKE: begin
                        // T60R: all exits, including angle/thermal/timeout, avoid
                        // short-braking above the existing DECEL_DONE_RATE threshold.
                        // High speed -> STBY=0/coast; no forced full-speed brake on timeout.
                        motor_enable <= (abs32(arm_velocity) < DECEL_DONE_RATE);
                        motor_brake <= (abs32(arm_velocity) < DECEL_DONE_RATE);
                        motor_cmd <= 10'sd0;

                        if (control_tick) begin
                            if (state_timer_ms >= 16'd250) begin
                                state <= ST_READY;
                                state_timer_ms <= 16'd0;
                            end else begin
                                state_timer_ms <= state_timer_ms + 1'b1;
                            end
                        end
                    end

                    default: begin
                        motor_enable <= 1'b0;
                        motor_brake <= 1'b0;
                        motor_cmd <= 10'sd0;

                        // 兼容内部启动接口；外部故障恢复由长按S0的recalibrate_pulse处理
                        if (start_pulse) begin
                            state <= ST_CALIBRATE;
                            fault_code <= 8'd0;
                            cal_sum <= 24'd0;
                            cal_count <= 11'd0;
                            state_timer_ms <= 16'd0;
                        end
                    end
                endcase
            end
        end
    end

endmodule
