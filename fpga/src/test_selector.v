// S0 stops drive immediately; a stop stays latched until a fresh accepted test.
// 拓展版按键语义 V2（选题一拓展要求）：
//   READY 中按 S1：起摆稳摆（配置固定第8组：profile0+摆杆快速1€+横臂更平滑1€）
//   稳摆(BALANCE)中按 S2：横杆转到 SW0/SW1 选定位置（00→¼圈 10→½圈 01→¾圈 11→回零）
//   稳摆中按 S3：横杆恒速旋转；稳摆中按 S4：横杆正弦速度曲线旋转
//   非稳摆状态 S2/S3/S4 一律无效；S0 任何时刻立即停车。
module test_selector #(
    parameter integer REMOTE_ENABLE=0,
    parameter integer CLK_FREQ_HZ=50_000_000,
    parameter integer DEBOUNCE_MS=20,
    parameter integer RECAL_HOLD_MS=2000
)(
    input wire clk, rst_n,
    input wire remote_owned,remote_start,remote_fast,remote_stop,remote_task,
    input wire[1:0]remote_mode,input wire[7:0]remote_cfg,output wire remote_ready,
    input wire stop_n,recal_n,
    input wire [3:0] test_n,
    input wire[7:0]sw_n,
    output reg[1:0]filter_mode,
    output reg pole_quiet,arm_quiet,
    output reg[1:0]task_mode,
    output reg task_cmd,                 // 稳摆中任务命令（单拍脉冲）
    output reg[7:0] task_cfg,            // S2/S3/S4 配置（按下瞬间锁存）
                                         // [2:0]=位置码 [3]=速率档 [4]=方向 [6:5]=频率档
    input wire [3:0] ctrl_state,
    input wire sensor_ready,
    output reg [1:0] profile_id,
    output reg [31:0] trial_id,
    output reg start_pulse,output reg start_fast,
    output wire stop_trip,
    output reg recalibrate_pulse,
    output reg stop_latched,
    output reg trial_active,
    output wire drive_allowed,
    output wire keys_released
);
    wire web_owned=REMOTE_ENABLE && remote_owned;
    wire web_stop=REMOTE_ENABLE && remote_stop;
    wire [3:0] levels, pulses;
    genvar k;
    generate for(k=0;k<4;k=k+1) begin: buttons
        key_filter #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.DEBOUNCE_MS(DEBOUNCE_MS)) f(
            .clk(clk),.rst_n(rst_n),.key_n(test_n[k]),
            .key_level_n(levels[k]),.key_pulse(pulses[k]));
    end endgenerate
    reg[7:0]sw_meta,sw_sync,sw_candidate,sw_stable;
    reg[31:0]stable_count;reg sw_valid;
    wire switches_ready=sw_valid && sw_n==sw_sync && sw_sync==sw_candidate && sw_candidate==sw_stable;
    always @(posedge clk or negedge rst_n)begin
     if(!rst_n)begin sw_meta<=8'hFF;sw_sync<=8'hFF;sw_candidate<=8'hFF;sw_stable<=8'hFF;stable_count<=0;sw_valid<=0;end
     else begin
      sw_meta<=sw_n;sw_sync<=sw_meta;
      if(sw_sync!=sw_candidate)begin sw_candidate<=sw_sync;stable_count<=0;sw_valid<=0;end
      else if(stable_count<CLK_FREQ_HZ/1000*DEBOUNCE_MS)stable_count<=stable_count+1'b1;
      else begin sw_stable<=sw_candidate;sw_valid<=1;end
     end
    end
    reg [1:0] stop_sync,recal_sync;
    reg stop_previous;
    reg armed, seen_running, recal_done;
    reg [31:0] release_count, hold_count;
    localparam integer RELEASE_CLKS=CLK_FREQ_HZ/1000*DEBOUNCE_MS;
    localparam integer RECAL_CLKS=CLK_FREQ_HZ/1000*RECAL_HOLD_MS;
    localparam signed [31:0] MOVE_DIR_C=32'sd1; // S2 位置方向（反了改 -1）
    assign keys_released=(&test_n) && (&levels) && stop_n && stop_sync[1];
    assign remote_ready=armed && keys_released && sensor_ready;
    assign drive_allowed=rst_n && stop_n && !web_stop && !stop_latched && trial_active;
    // Independent synchronized falling edge: the raw gate can latch before
    // the two synchronizer stages, so using stop_latched here would lose the trip.
    assign stop_trip=(!stop_sync[1] && stop_previous) || web_stop;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            stop_sync<=2'b11;recal_sync<=2'b11; stop_previous<=1; profile_id<=0;filter_mode<=2'b01;pole_quiet<=0;arm_quiet<=1;task_mode<=0;task_cmd<=0;task_cfg<=8'd0; trial_id<=0;
            start_pulse<=0; start_fast<=1;recalibrate_pulse<=0; stop_latched<=1;
            trial_active<=0; seen_running<=0; armed<=0;
            release_count<=0; hold_count<=0; recal_done<=0;
        end else begin
            stop_sync<={stop_sync[0],stop_n};recal_sync<={recal_sync[0],recal_n};
            stop_previous<=stop_sync[1];
            start_pulse<=0; recalibrate_pulse<=0; task_cmd<=0;
            if((ctrl_state>=2 && ctrl_state<=4) || ctrl_state==9) seen_running<=1;
            if(seen_running && (ctrl_state==1 || ctrl_state==6)) begin
                trial_active<=0; seen_running<=0;
            end
            if(keys_released) begin
                if(release_count<RELEASE_CLKS) release_count<=release_count+1'b1;
                else armed<=1;
            end else release_count<=0;

            // Holding S0 never starts a trial. Recalibration is one-shot and idle only.
            if(!recal_sync[1] && (ctrl_state==1 || ctrl_state==6 || ctrl_state==0)) begin
                if(!recal_done) begin
                    if(hold_count>=RECAL_CLKS-1) begin
                        recalibrate_pulse<=1; recal_done<=1; hold_count<=0;
                    end else hold_count<=hold_count+1'b1;
                end
            end else begin
                hold_count<=0;
                if(recal_sync[1]) recal_done<=0;
            end

            if(!stop_n || !stop_sync[1] || web_stop) begin
                stop_latched<=1; trial_active<=0; armed<=0; release_count<=0;
            end else if(web_owned)begin
                if(remote_start && remote_ready && ctrl_state==1)begin
                    armed<=0;task_mode<=0;task_cfg<=0;
                    profile_id<=0;filter_mode<=2'b01;pole_quiet<=0;arm_quiet<=1;
                    start_pulse<=1;start_fast<=remote_fast;trial_id<=trial_id+1'b1;
                    trial_active<=1;stop_latched<=0;seen_running<=0;
                end else if(remote_task && ctrl_state==4 && sensor_ready && keys_released)begin
                    task_cmd<=1;task_mode<=remote_mode;task_cfg<=remote_cfg;
                end
            end else if(|pulses) begin
                armed<=0; // ignored/busy presses are never queued for later.
                if(armed && ctrl_state==1 && sensor_ready && switches_ready && test_n==levels) begin
                    // READY 中仅 S1 有效：起摆稳摆，固定第8组配置，任务清空
                    if((~levels)==4'b0001) begin
                        task_mode<=2'd0;
                        profile_id<=2'd0;filter_mode<=2'b01;pole_quiet<=1'b0;arm_quiet<=1'b1;
                        start_pulse<=1;start_fast<=sw_stable[7];
                        trial_id<=trial_id+1'b1; trial_active<=1;
                        stop_latched<=0; seen_running<=0;
                    end
                end else if(armed && ctrl_state==4 && test_n==levels) begin
                    // 稳摆中 S2/S3/S4 下发任务，拨码在按下瞬间读取（ON=1）
                    // S2：SW0~SW2 = 位置码 000→零点 001→45° … 111→315°
                    // S3：SW6 = 方向(0 正/1 负)，SW7 = 速率(0 慢/1 快)
                    // S4：SW3~SW4 = 频率档，SW5 = 方向(0 正/1 负)
                    case(~levels)
                        4'b0010: begin // S2 指定位置
                            task_mode<=2'd1; task_cmd<=1;
                            task_cfg[2:0]<=~{sw_stable[0],sw_stable[1],sw_stable[2]};
                        end
                        4'b0100: begin // S3 恒速旋转
                            task_mode<=2'd2; task_cmd<=1;
                            task_cfg[3]<=~sw_stable[7];
                            task_cfg[4]<=~sw_stable[6];
                        end
                        4'b1000: begin // S4 正弦：SW3~SW5 三位组合查表（见 arm_task）
                            task_mode<=2'd3; task_cmd<=1;
                            task_cfg[6:4]<=~{sw_stable[3],sw_stable[4],sw_stable[5]};
                        end
                        default: begin end
                    endcase
                end
            end
        end
    end
endmodule
