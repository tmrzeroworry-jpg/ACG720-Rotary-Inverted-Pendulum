// V4:113bytes adds signed home reference and validity, same 100Hz schedule.
// V2:104bytes; optional V3:108bytes with exact arm target. CRC over all header/payload bytes, CRLF.
// Independent10ms schedule at115200bps; missed slots create sequence gaps.
module telemetry_ab4 #(
    parameter integer CLK_FREQ_HZ=50_000_000,
    parameter integer BAUD_RATE=115200,
    parameter integer INCLUDE_REMOTE=0,
    parameter integer REPORT_HZ=100,INCLUDE_STREAM=0,INCLUDE_FLIP=0,INCLUDE_LIFE=0,
    parameter [31:0] BUILD_ID=32'h57543031,
    parameter integer INCLUDE_ARM_TARGET=0,
    parameter integer INCLUDE_HOME_REFERENCE=0,
    parameter ENABLE_CONFIG_EVENTS=0,
    parameter integer CONFIG_BYTES=12
)(
    input wire clk, rst_n,
    input wire[31:0]remote_session,input wire[15:0]remote_seq,remote_age,
    input wire[7:0]remote_op,remote_result,remote_flags,remote_mode,remote_cfg,
    input wire[7:0]life_phase,life_result,life_turns,input wire[15:0]life_id,life_elapsed,life_down,
    input wire life_mode,life_down_valid,input wire signed[31:0]life_rate,
    input wire [31:0] trial_id,
    input wire[7:0]flip_phase,flip_result,input wire[15:0]flip_id,flip_elapsed,
    input wire signed[31:0]flip_progress,
    input wire [31:0] stream_session,
    input wire [15:0] stream_seq,stream_age,
    input wire [7:0] stream_flags,stream_result,
    input wire signed [31:0] stream_velocity,stream_desired,
    input wire signed[31:0] arm_target_reference,
    input wire signed[31:0] arm_home_reference,
    input wire arm_home_valid,
    input wire [7:0] profile, state, fault, flags,
    input wire [15:0] raw,
    input wire signed [17:0] error,
    input wire signed [31:0] rate, arm_position, arm_rate,
    input wire signed [9:0] command,
    input wire [15:0] down, up,
    input wire signed [15:0] trim,
    input wire signed [31:0] p,d,arm_p,arm_d,linear,compensated,
    input wire signed [9:0] target,
    input wire [15:0] balance_ms,saturation_ms,reversals,
    input wire [63:0] error_sq_sum,command_sq_sum,
    input wire [15:0] error_peak,command_peak,
    input wire cfg_event,input wire[CONFIG_BYTES*8-1:0]cfg_payload,
    input wire ir_event,ir_repeat,ir_learning,ir_ready,
    input wire[15:0]ir_address,ir_errors,
    input wire[7:0]ir_command,ir_action,
    input wire[2:0]ir_stage,
    output wire uart_tx
);
    localparam integer FRAME_CLKS=CLK_FREQ_HZ/REPORT_HZ;
    localparam integer MS_CLKS=CLK_FREQ_HZ/1000;
    reg [31:0] schedule_count, ms_count, uptime_ms, frame_sequence;
    localparam integer CRC_START=INCLUDE_REMOTE?168:INCLUDE_LIFE?152:INCLUDE_FLIP?137:INCLUDE_STREAM?127:INCLUDE_HOME_REFERENCE?109:INCLUDE_ARM_TARGET?104:100;
    localparam integer LAST_BYTE=CRC_START+3;
    reg [(CRC_START-4)*8-1:0] snapshot;
    reg cfg_pending,cfg_mode;
    reg[CONFIG_BYTES*8-1:0]cfg_buffer,cfg_snapshot;
    reg ir_pending,ir_mode;
    reg[63:0]ir_payload,ir_snapshot;
    reg [1:0] tx_state;
    reg [7:0] index;
    reg [15:0] crc;
    reg [7:0] tx_data;
    reg tx_start, missed_slot;
    wire busy,done;
    localparam IDLE=0, LOAD=1, WAIT_BUSY=2, WAIT_DONE=3;
    uart_tx_byte #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.BAUD_RATE(BAUD_RATE)) uart(
        .clk(clk),.rst_n(rst_n),.tx_data(tx_data),.tx_start(tx_start),
        .uart_tx(uart_tx),.busy(busy),.done(done));
    function [7:0] byte_at;
        input [7:0] n;
        begin
            if(cfg_mode)begin
                case(n)
                 0:byte_at=8'haa;1:byte_at=8'h58;2:byte_at=CONFIG_BYTES==12?1:2;3:byte_at=CONFIG_BYTES+8;
                 CONFIG_BYTES+4:byte_at=crc[15:8];CONFIG_BYTES+5:byte_at=crc[7:0];
                 CONFIG_BYTES+6:byte_at=13;CONFIG_BYTES+7:byte_at=10;
                 default:byte_at=cfg_snapshot[(CONFIG_BYTES+3-n)*8+:8];
                endcase
            end else if(ir_mode)begin
                case(n)
                 0:byte_at=8'haa;1:byte_at=8'h57;2:byte_at=1;3:byte_at=16;
                 12:byte_at=crc[15:8];13:byte_at=crc[7:0];14:byte_at=13;15:byte_at=10;
                 default:byte_at=ir_snapshot[(11-n)*8+:8];
                endcase
            end else case(n)
                0:byte_at=8'haa; 1:byte_at=8'h56; 2:byte_at=INCLUDE_REMOTE?8:INCLUDE_LIFE?7:INCLUDE_FLIP?6:INCLUDE_STREAM?5:INCLUDE_HOME_REFERENCE?4:INCLUDE_ARM_TARGET?3:2; 3:byte_at=CRC_START+4;
                CRC_START:byte_at=crc[15:8]; CRC_START+1:byte_at=crc[7:0];
                CRC_START+2:byte_at=8'h0d; CRC_START+3:byte_at=8'h0a;
                default:byte_at=snapshot[(CRC_START-1-n)*8+:8];
            endcase
        end
    endfunction
    function [15:0] crc_next;
        input [15:0] old;
        input [7:0] data;
        reg [15:0] c;
        integer b;
        begin
            c=old^{data,8'd0};
            for(b=0;b<8;b=b+1) c=c[15] ? (c<<1)^16'h1021 : c<<1;
            crc_next=c;
        end
    endfunction
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            schedule_count<=0; ms_count<=0; uptime_ms<=0; frame_sequence<=0;
            cfg_pending<=0;cfg_mode<=0;cfg_buffer<=0;cfg_snapshot<=0;
            snapshot<=0; tx_state<=IDLE; index<=0; crc<=16'hffff;
            tx_data<=0; tx_start<=0; missed_slot<=0;ir_pending<=0;ir_mode<=0;ir_payload<=0;ir_snapshot<=0;
        end else begin
            tx_start<=0;
            if(ENABLE_CONFIG_EVENTS && cfg_event)begin cfg_pending<=1;cfg_buffer<=cfg_payload;end
            if(ir_event)begin
                ir_pending<=1;
                ir_payload<={ir_address,ir_command,{5'd0,ir_learning,ir_ready,ir_repeat},
                             {5'd0,ir_stage},ir_action,ir_errors};
            end
            if(ms_count>=MS_CLKS-1) begin ms_count<=0; uptime_ms<=uptime_ms+1'b1; end
            else ms_count<=ms_count+1'b1;
            if(schedule_count>=FRAME_CLKS-1) begin
                schedule_count<=0; frame_sequence<=frame_sequence+1'b1;
                if(tx_state==IDLE && ENABLE_CONFIG_EVENTS && cfg_pending)begin
                    cfg_mode<=1;ir_mode<=0;cfg_snapshot<=cfg_buffer;cfg_pending<=cfg_event;
                    index<=0;crc<=16'hffff;tx_state<=LOAD;missed_slot<=1;
                end else if(tx_state==IDLE) begin
                    cfg_mode<=0;
                    snapshot<=INCLUDE_REMOTE ? {BUILD_ID,frame_sequence,uptime_ms,trial_id,
                        profile,state,fault,{missed_slot,flags[6:0]},raw,error[15:0],
                        rate,arm_position,arm_rate,{{6{command[9]}},command},down,up,trim,
                        p,d,arm_p,arm_d,linear,compensated,{{6{target[9]}},target},
                        balance_ms,saturation_ms,reversals,error_sq_sum,command_sq_sum,error_peak,command_peak,
                        arm_target_reference,arm_home_reference,{7'd0,arm_home_valid},
                        stream_session,stream_seq,stream_flags,stream_result,stream_velocity,stream_desired,stream_age,flip_phase,flip_result,flip_id,flip_progress,flip_elapsed,life_phase,life_result,life_id,life_elapsed,{7'd0,life_mode},life_turns,life_down,{7'd0,life_down_valid},life_rate,remote_session,remote_seq,remote_op,remote_result,remote_flags,remote_mode,remote_cfg,remote_age,24'd0} : INCLUDE_LIFE ? {BUILD_ID,frame_sequence,uptime_ms,trial_id,
                        profile,state,fault,{missed_slot,flags[6:0]},raw,error[15:0],
                        rate,arm_position,arm_rate,{{6{command[9]}},command},down,up,trim,
                        p,d,arm_p,arm_d,linear,compensated,{{6{target[9]}},target},
                        balance_ms,saturation_ms,reversals,error_sq_sum,command_sq_sum,error_peak,command_peak,
                        arm_target_reference,arm_home_reference,{7'd0,arm_home_valid},
                        stream_session,stream_seq,stream_flags,stream_result,stream_velocity,stream_desired,stream_age,flip_phase,flip_result,flip_id,flip_progress,flip_elapsed,life_phase,life_result,life_id,life_elapsed,{7'd0,life_mode},life_turns,life_down,{7'd0,life_down_valid},life_rate} : INCLUDE_FLIP ? {BUILD_ID,frame_sequence,uptime_ms,trial_id,
                        profile,state,fault,{missed_slot,flags[6:0]},raw,error[15:0],
                        rate,arm_position,arm_rate,{{6{command[9]}},command},down,up,trim,
                        p,d,arm_p,arm_d,linear,compensated,{{6{target[9]}},target},
                        balance_ms,saturation_ms,reversals,error_sq_sum,command_sq_sum,error_peak,command_peak,
                        arm_target_reference,arm_home_reference,{7'd0,arm_home_valid},
                        stream_session,stream_seq,stream_flags,stream_result,stream_velocity,stream_desired,stream_age,flip_phase,flip_result,flip_id,flip_progress,flip_elapsed} : INCLUDE_STREAM ? {BUILD_ID,frame_sequence,uptime_ms,trial_id,
                        profile,state,fault,{missed_slot,flags[6:0]},raw,error[15:0],
                        rate,arm_position,arm_rate,{{6{command[9]}},command},down,up,trim,
                        p,d,arm_p,arm_d,linear,compensated,{{6{target[9]}},target},
                        balance_ms,saturation_ms,reversals,error_sq_sum,command_sq_sum,error_peak,command_peak,
                        arm_target_reference,arm_home_reference,{7'd0,arm_home_valid},
                        stream_session,stream_seq,stream_flags,stream_result,stream_velocity,stream_desired,stream_age} : INCLUDE_HOME_REFERENCE ? {BUILD_ID,frame_sequence,uptime_ms,trial_id,
                        profile,state,fault,{missed_slot,flags[6:0]},raw,error[15:0],
                        rate,arm_position,arm_rate,{{6{command[9]}},command},down,up,trim,
                        p,d,arm_p,arm_d,linear,compensated,{{6{target[9]}},target},
                        balance_ms,saturation_ms,reversals,error_sq_sum,command_sq_sum,error_peak,command_peak,
                        arm_target_reference,arm_home_reference,{7'd0,arm_home_valid}} : INCLUDE_ARM_TARGET ? {BUILD_ID,frame_sequence,uptime_ms,trial_id,
                        profile,state,fault,{missed_slot,flags[6:0]},raw,error[15:0],
                        rate,arm_position,arm_rate,{{6{command[9]}},command},down,up,trim,
                        p,d,arm_p,arm_d,linear,compensated,{{6{target[9]}},target},
                        balance_ms,saturation_ms,reversals,error_sq_sum,command_sq_sum,error_peak,command_peak,arm_target_reference} :
                        {BUILD_ID,frame_sequence,uptime_ms,trial_id,
                        profile,state,fault,{missed_slot,flags[6:0]},raw,error[15:0],
                        rate,arm_position,arm_rate,{{6{command[9]}},command},down,up,trim,
                        p,d,arm_p,arm_d,linear,compensated,{{6{target[9]}},target},
                        balance_ms,saturation_ms,reversals,error_sq_sum,command_sq_sum,error_peak,command_peak};
                    index<=0; crc<=16'hffff; tx_state<=LOAD; missed_slot<=0;ir_mode<=0;
                end else missed_slot<=1;
            end else schedule_count<=schedule_count+1'b1;
            case(tx_state)
                IDLE:if(ir_pending && schedule_count!=FRAME_CLKS-1)begin
                    cfg_mode<=0;ir_mode<=1;ir_snapshot<=ir_payload;ir_pending<=ir_event;
                    index<=0;crc<=16'hffff;tx_state<=LOAD;
                end
                LOAD:begin
                    tx_data<=byte_at(index); tx_start<=1;
                    if(index<(cfg_mode?CONFIG_BYTES+4:ir_mode?12:CRC_START)) crc<=crc_next(crc,byte_at(index));
                    tx_state<=WAIT_BUSY;
                end
                WAIT_BUSY:if(busy) tx_state<=WAIT_DONE;
                WAIT_DONE:if(done || !busy) begin
                    if(index==(cfg_mode?CONFIG_BYTES+7:ir_mode?15:LAST_BYTE)) tx_state<=IDLE;
                    else begin index<=index+1'b1; tx_state<=LOAD; end
                end
                default:begin end
            endcase
        end
    end
endmodule
