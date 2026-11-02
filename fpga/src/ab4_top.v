// 拓展版 V2（选题一拓展要求）：S0停止；S1起摆稳摆不限时；稳摆中 S2 按 SW0/SW1
// 选位（¼/½/¾/回零）、S3 恒速旋转、S4 正弦速度旋转；非稳摆时 S2~S4 无效。
// 控制配置固定为九组合实测最优的第8组（摆杆快速1€＋横臂更平滑1€，WT01原S1）。
module ab4_top #(
    parameter integer CLK_FREQ_HZ=50_000_000,
    parameter integer CALIB_SAMPLES=1200,
    parameter integer CATCH_HOLD_MS=300,
    parameter integer BALANCE_TRIAL_MS=600000, // 拓展演示不限时；计时器饱和，
                                               // 实际由 S0/失稳/热保护收尾
    parameter USE_BOOT_REFERENCE=1,
    parameter ENCODER_REVERSE=0,
    parameter MOTOR_REVERSE=0,
    parameter PEND_DIR_REVERSE=0,
    parameter BALANCE_REVERSE=0
)(
    input wire clk_50m,
    input wire key_stop_n,
    input wire [3:0] key_test_n,
    input wire enc_a,enc_b,
    input wire [7:0] sw_n,
    inout wire i2c_sda,i2c_scl,
    output wire motor_pwm,motor_ain1,motor_ain2,motor_stby,
    input wire uart_rx,
    output wire uart_tx,
    output wire [3:0] led
);
    reg [31:0] por_count=0;
    reg por_done=0;
    always @(posedge clk_50m) begin
        if(!por_done) begin
            if(por_count>=CLK_FREQ_HZ/50-1) por_done<=1;
            else por_count<=por_count+1'b1;
        end
    end
    wire sys_rst_n=por_done;
    reg [25:0] heartbeat_count;
    always @(posedge clk_50m or negedge sys_rst_n)
        if(!sys_rst_n) heartbeat_count<=0;
        else heartbeat_count<=heartbeat_count+1'b1;

    wire [15:0] angle_raw,angle_down,angle_up;
    wire angle_valid,angle_online,angle_configured,ack_error;
    wire signed [31:0] arm_position,arm_velocity;
    wire [3:0] state;
    wire [7:0] fault;
    wire signed [17:0] error;
    wire signed [31:0] pend_velocity,balance_velocity,candidate_rate;
    wire candidate_ready,lqr_rate_ready;
    wire signed[31:0] lqr_arm_velocity;
    wire signed [15:0] trim;
    wire signed [9:0] command,target;
    wire controller_enable,controller_brake,control_tick,adc_fresh;
    wire signed [31:0] p,d,arm_p,arm_d,linear,compensated;
    wire [1:0] profile_id;
    wire signed[31:0] arm_target_reference;
    wire signed [31:0] arm_home_reference;
    wire arm_home_valid;
    wire [31:0] trial_id;
    wire start_pulse,stop_trip,recalibrate,stop_latched,trial_active,drive_allowed,keys_released;
    // Legacy PD candidate flags disabled; all profiles use the same860SPS ADC and10kHz PWM.
    wire profile3_balance=1'b0;
    wire profile4_balance=1'b0;
    wire combined_stop_n=key_stop_n;
    wire remote_owned,remote_start,remote_fast,remote_stop,remote_task,remote_ready;
    wire[1:0]remote_mode;wire[7:0]remote_cfg,remote_op,remote_result,remote_flags;
    wire[31:0]remote_session;wire[15:0]remote_seq,remote_age;
    wire[1:0]filter_mode;
    wire pole_quiet,arm_quiet;
    wire [1:0] task_mode;
    wire task_cmd;
    wire [7:0] task_cfg;
    wire start_fast,return_start,return_abort,life_busy,return_busy,life_fresh,life_mode,life_down_valid;
    wire[7:0]life_phase,life_result,life_turns;wire[15:0]life_id,life_elapsed,life_down;wire signed[31:0]life_rate;
    wire flip_start,flip_negative,flip_late,flip_abort,flip_busy,flip_sensor_fresh;
    wire[7:0]flip_phase,flip_result;wire[15:0]flip_id,flip_elapsed;wire signed[31:0]flip_progress;
    wire [7:0] rx_byte;wire rx_valid,rx_framing_error;
    wire stream_enable,task_idle;wire signed[31:0] stream_velocity,stream_desired;
    wire [31:0] stream_session;wire [15:0] stream_seq,stream_age;wire[7:0] stream_flags,stream_result;

    wire[1:0]pole_code=filter_mode[0] ? (pole_quiet ? 2'd2:2'd1):2'd0;
    wire[1:0]arm_code=filter_mode[1] ? (arm_quiet ? 2'd2:2'd1):2'd0;
    assign recalibrate=1'b0;
    test_selector #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.REMOTE_ENABLE(1)) selector(
      .remote_owned(remote_owned),.remote_start(remote_start),.remote_fast(remote_fast),.remote_stop(remote_stop),
      .remote_task(remote_task),.remote_mode(remote_mode),.remote_cfg(remote_cfg),.remote_ready(remote_ready),
      .clk(clk_50m),.rst_n(sys_rst_n),.sw_n(sw_n),.stop_n(key_stop_n),.recal_n(1'b1),
      .test_n(key_test_n),.ctrl_state(state),.sensor_ready(adc_fresh && angle_configured && !ack_error),
      .profile_id(profile_id),.filter_mode(filter_mode),.pole_quiet(pole_quiet),.arm_quiet(arm_quiet),
      .task_mode(task_mode),.task_cmd(task_cmd),.task_cfg(task_cfg),.trial_id(trial_id),.start_pulse(start_pulse),.start_fast(start_fast),
      .stop_trip(stop_trip),.recalibrate_pulse(),.stop_latched(stop_latched),
      .trial_active(trial_active),.drive_allowed(drive_allowed),.keys_released(keys_released));
    encoder_quadrature #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.REVERSE(ENCODER_REVERSE)) encoder(
        .clk(clk_50m),.rst_n(sys_rst_n),.enc_a(enc_a),.enc_b(enc_b),
        .position(arm_position),.velocity(arm_velocity));
    ads1115_reader #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.I2C_FREQ_HZ(100000),.I2C_ADDR(7'h48),.CONFIG_WORD(16'hC2E3)) adc(
        .clk(clk_50m),.rst_n(sys_rst_n),.slow_sample(profile3_balance),
        .i2c_sda(i2c_sda),.i2c_scl(i2c_scl),.adc_raw(angle_raw),.adc_valid(angle_valid),
        .adc_online(angle_online),.adc_configured(angle_configured),.ack_error(ack_error));
    balance_rate_estimator #(.CLK_FREQ_HZ(CLK_FREQ_HZ)) rate_estimator(
        .clk(clk_50m),.rst_n(sys_rst_n),.active(profile3_balance),.angle_raw(angle_raw),
        .angle_valid(angle_valid),.fallback_velocity(pend_velocity),
        .velocity(candidate_rate),.ready(candidate_ready));
    // SWING/early PD catch preserve the proven source algorithm. LQR
    // gains are generated in lqr_feedback.v; no old PD tuning is called LQR.
    swing_controller #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ),.CALIB_SAMPLES(CALIB_SAMPLES),.CATCH_HOLD_MS(CATCH_HOLD_MS),
        .ONE_EURO(1),.BALANCE_TRIAL_MS(BALANCE_TRIAL_MS),.USE_BOOT_REFERENCE(USE_BOOT_REFERENCE),.PEND_DIR_REVERSE(PEND_DIR_REVERSE),
        .BALANCE_REVERSE(BALANCE_REVERSE),.SWING_BASE_DUTY(60),.SWING_MAX_DUTY(160),
        .SWING_TIMEOUT_MS(20000),.ARM_LIMIT(3000),.ARM_LIMIT_ENABLE(1),
        .BAL_KP_ANGLE(48),.BAL_KD_ANGLE(96),.BAL_KP_ARM(8),.BAL_KP_ARM_SHIFT(8),
        .BAL_KD_ARM(64),.BAL_KD_ARM_SHIFT(10),.BAL_ARM_REVERSE(1),
        .ARM_ANCHOR_ENABLE(0),.BAL_RATE_FILTER_SHIFT(0),.BAL_SLEW_PER_MS(64),
        .DEADZONE_QUIET(6),.ANCHOR_GATE_ANGLE(220),
        .SWING_CENTER_SHIFT(4)
    ) controller(
        .clk(clk_50m),.rst_n(sys_rst_n),.euro_mode(filter_mode),.euro_p_quiet(pole_quiet),.euro_a_quiet(arm_quiet),.profile_id(profile_id),.recalibrate_pulse(recalibrate),
        .task_mode(task_mode),.task_cmd(task_cmd),.task_cfg(task_cfg),
        .stream_enable(stream_enable),.stream_velocity_q16(stream_velocity),.task_idle_o(task_idle),
        .start_fast(start_fast),.return_start(return_start),.return_cancel(return_abort),
        .life_busy(life_busy),.return_busy(return_busy),.life_fresh(life_fresh),.life_mode(life_mode),.life_down_valid(life_down_valid),
        .life_phase(life_phase),.life_result(life_result),.life_turns(life_turns),.life_id(life_id),.life_elapsed(life_elapsed),.life_down(life_down),.life_rate(life_rate),
        .flip_start(flip_start),.flip_negative(flip_negative),.flip_late(flip_late),.flip_cancel(flip_abort),.flip_busy(flip_busy),
        .flip_sensor_fresh(flip_sensor_fresh),.flip_phase(flip_phase),.flip_result(flip_result),.flip_id(flip_id),.flip_elapsed(flip_elapsed),.flip_progress(flip_progress),
        .candidate_balance_rate(candidate_rate),.candidate_rate_ready(candidate_ready),
        .angle_raw(angle_raw),.angle_valid(angle_valid),.angle_online(angle_online),
        .arm_position(arm_position),.arm_velocity(arm_velocity),
        .start_pulse(start_pulse),.brake_pulse(stop_trip),
        .motor_cmd(command),.motor_enable(controller_enable),.motor_brake(controller_brake),
        .state(state),.fault_code(fault),.angle_down(angle_down),.angle_up(angle_up),
        .upright_error(error),.pend_velocity(pend_velocity),.balance_velocity(balance_velocity),
        .trim_adj_o(trim),.diag_p(p),.diag_d(d),.diag_arm_p(arm_p),.diag_arm_d(arm_d),
        .diag_linear(linear),.diag_compensated(compensated),.diag_target(target),
        .control_tick_o(control_tick),.adc_fresh_o(adc_fresh),.lqr_ready_o(lqr_rate_ready),.lqr_arm_rate_o(lqr_arm_velocity),.arm_target_o(arm_target_reference),.arm_home_o(arm_home_reference),.arm_home_valid_o(arm_home_valid));
    wire stream_permit=key_stop_n && drive_allowed && adc_fresh && angle_configured && !ack_error &&
                       (error < 18'sd800 && error > -18'sd800);
    wire start_quiet=(arm_velocity < 32'sd100 && arm_velocity > -32'sd100);
    uart_rx_byte #(.CLK_FREQ_HZ(CLK_FREQ_HZ)) receiver(.clk(clk_50m),.rst_n(sys_rst_n),.uart_rx(uart_rx),
       .data(rx_byte),.valid(rx_valid),.framing_error(rx_framing_error));
    remote_control remote(.clk(clk_50m),.rst_n(sys_rst_n),.tick(control_tick),.rx_data(rx_byte),.rx_valid(rx_valid),
      .stop_n(key_stop_n),.sensor_ready(adc_fresh && angle_configured && !ack_error),.start_ready(remote_ready),
      .keys_released(keys_released),.ctrl_state(state),.life_busy(life_busy),.flip_busy(flip_busy),.stream_enabled(stream_enable && (stream_velocity!=0 || stream_desired!=0)),
      .owned(remote_owned),.start_pulse(remote_start),.start_fast(remote_fast),.task_pulse(remote_task),.stop_pulse(remote_stop),
      .task_mode(remote_mode),.task_cfg(remote_cfg),.session(remote_session),.ack_seq(remote_seq),.age_ms(remote_age),
      .ack_op(remote_op),.result(remote_result),.flags(remote_flags));
    velocity_stream #(.MAX_V_Q16(68157),.MAX_A_STEP(136)) stream(.clk(clk_50m),.rst_n(sys_rst_n),.tick(control_tick),.rx_data(rx_byte),.rx_valid(rx_valid),
       .life_busy(life_busy),.return_busy(return_busy),.return_safe(key_stop_n && drive_allowed && life_fresh && !ack_error),
       .return_start(return_start),.return_abort(return_abort),.flip_busy(flip_busy),.flip_safe(key_stop_n && drive_allowed && (flip_busy ? flip_sensor_fresh:adc_fresh) && !ack_error),
       .flip_start(flip_start),.flip_negative(flip_negative),.flip_late(flip_late),.flip_abort(flip_abort),
       .balance(state==4),.permit(stream_permit),.local_cancel(task_cmd),.task_idle(task_idle || (remote_owned && remote_mode==1 && remote_cfg==128)),.start_quiet(start_quiet),
       .enabled(stream_enable),.velocity_q16(stream_velocity),.desired_q16(stream_desired),
       .session(stream_session),.ack_seq(stream_seq),.age_ms(stream_age),.flags(stream_flags),.result(stream_result));
    wire signed[9:0] motion_command,flip_motion_command;
    flip_output_guard smooth_flip(.clk(clk_50m),.rst_n(sys_rst_n),.tick(control_tick),.busy(flip_busy),.phase(flip_phase),
      .permit(key_stop_n && drive_allowed && state!=5 && state!=6),.request(command),.command(flip_motion_command));
    flip_output_guard #(.MAIN_SLEW(3)) smooth_transition(.clk(clk_50m),.rst_n(sys_rst_n),.tick(control_tick),.busy(life_busy),
      .phase((life_phase==3 || life_phase==4) ? 8'd3:8'd2),.permit(key_stop_n && drive_allowed && state!=5 && state!=6),
      .request(flip_motion_command),.command(motion_command));
    wire pwm_i,ain1_i,ain2_i,stby_i;
    motor_tb6612 #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.PWM_FREQ_HZ(10000),.REVERSE(MOTOR_REVERSE)) motor(
        .clk(clk_50m),.rst_n(sys_rst_n),.candidate_pwm(profile4_balance),
        .enable(drive_allowed && controller_enable),.brake(controller_brake),.motor_cmd(motion_command),
        .motor_pwm(pwm_i),.motor_ain1(ain1_i),.motor_ain2(ain2_i),.motor_stby(stby_i));
    // Raw S0 press gates every output independently of debounce/controller state.
    assign motor_pwm=combined_stop_n && drive_allowed ? pwm_i : 1'b0;
    assign motor_ain1=combined_stop_n && drive_allowed ? ain1_i : 1'b0;
    assign motor_ain2=combined_stop_n && drive_allowed ? ain2_i : 1'b0;
    assign motor_stby=combined_stop_n && drive_allowed ? stby_i : 1'b0;
    wire signed [9:0] applied_command=drive_allowed && controller_enable ? motion_command : 10'sd0;
    wire [15:0] balance_ms,saturation_ms,reversals,error_peak,command_peak;
    wire [63:0] error_sq_sum,command_sq_sum;
    trial_statistics #(.SATURATION_LIMIT(180)) statistics(
        .clk(clk_50m),.rst_n(sys_rst_n),.start_pulse(start_pulse),.control_tick(control_tick),
        .state(state),.error(error),.command(applied_command),.balance_ms(balance_ms),
        .saturation_ms(saturation_ms),.reversals(reversals),.error_sq_sum(error_sq_sum),
        .command_sq_sum(command_sq_sum),.error_peak(error_peak),.command_peak(command_peak));
    wire [7:0] flags={1'b0,lqr_rate_ready,ack_error,angle_configured,angle_online,
                      state==1,stop_latched,trial_active};
    telemetry_ab4 #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.BUILD_ID(32'h55493032),.INCLUDE_REMOTE(1),.INCLUDE_LIFE(1),.INCLUDE_FLIP(1),.REPORT_HZ(50),.INCLUDE_STREAM(1),.INCLUDE_ARM_TARGET(1),.INCLUDE_HOME_REFERENCE(1)) telemetry(
        .clk(clk_50m),.rst_n(sys_rst_n),
        .life_phase(life_phase),.life_result(life_result),.life_id(life_id),.life_elapsed(life_elapsed),.life_mode(life_mode),.life_turns(life_turns),
        .remote_session(remote_session),.remote_seq(remote_seq),.remote_op(remote_op),.remote_result(remote_result),
        .remote_flags(remote_flags),.remote_mode({6'd0,task_mode}),.remote_cfg(task_cfg),.remote_age(remote_age),
        .life_down(life_down),.life_down_valid(life_down_valid),.life_rate(life_rate),.stream_session(stream_session),.stream_seq(stream_seq),
        .flip_phase(flip_phase),.flip_result(flip_result),.flip_id(flip_id),.flip_progress(flip_progress),.flip_elapsed(flip_elapsed),
        .stream_flags(stream_flags),.stream_result(stream_result),.stream_velocity(stream_velocity),.stream_desired(stream_desired),.stream_age(stream_age),
        .trial_id(trial_id),.profile({2'd0,profile_id,pole_code,arm_code}+8'd1),
        .arm_target_reference(arm_target_reference),.arm_home_reference(arm_home_reference),.arm_home_valid(arm_home_valid),.state({4'd0,state}),.fault(fault),.flags(flags),.raw(angle_raw),.error(error),
        .rate((state==3 || state==4) ? balance_velocity : pend_velocity),.arm_position(arm_position),
        .arm_rate((state==3 || state==4) ? lqr_arm_velocity : arm_velocity),.command(applied_command),.down(angle_down),.up(angle_up),.trim(trim),
        .p(p),.d(d),.arm_p(arm_p),.arm_d(arm_d),.linear(linear),.compensated(compensated),
        .target(target),.balance_ms(balance_ms),.saturation_ms(saturation_ms),.reversals(reversals),
        .error_sq_sum(error_sq_sum),.command_sq_sum(command_sq_sum),
        .error_peak(error_peak),.command_peak(command_peak),
        .cfg_event(1'b0),.cfg_payload(96'd0),
        .ir_event(1'b0),.ir_address(16'd0),.ir_command(8'd0),
        .ir_repeat(1'b0),.ir_learning(1'b0),.ir_ready(1'b0),
        .ir_stage(3'd0),.ir_action(8'd0),.ir_errors(16'd0),.uart_tx(uart_tx));
    assign led[0]=heartbeat_count[24];
    assign led[1]=task_mode[0];
    assign led[2]=task_mode[1];
    assign led[3]=trial_active;
endmodule
