// Counts the command held at each1ms BALANCE tick, including zero crossings.
// Sums are retained after completion and reset only for a newly accepted trial.
module trial_statistics #(parameter integer SATURATION_LIMIT=255)(
    input wire clk, rst_n, start_pulse, control_tick,
    input wire [3:0] state,
    input wire signed [17:0] error,
    input wire signed [9:0] command,
    output reg [15:0] balance_ms, saturation_ms, reversals,
    output reg [63:0] error_sq_sum, command_sq_sum,
    output reg [15:0] error_peak, command_peak
);
    reg previous_nonzero_negative, have_nonzero;
    wire signed [35:0] error_squared=$signed(error)*$signed(error);
    wire signed [19:0] command_squared=$signed(command)*$signed(command);
    wire [17:0] error_abs=error<0 ? -error : error;
    wire [9:0] command_abs=command<0 ? -command : command;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            balance_ms<=0; saturation_ms<=0; reversals<=0;
            error_sq_sum<=0; command_sq_sum<=0;
            error_peak<=0; command_peak<=0;
            previous_nonzero_negative<=0; have_nonzero<=0;
        end else if(start_pulse) begin
            balance_ms<=0; saturation_ms<=0; reversals<=0;
            error_sq_sum<=0; command_sq_sum<=0;
            error_peak<=0; command_peak<=0; have_nonzero<=0;
        end else if(control_tick && state==4) begin
            balance_ms<=balance_ms+1'b1;
            if(command_abs>=SATURATION_LIMIT) saturation_ms<=saturation_ms+1'b1;
            error_sq_sum<=error_sq_sum+{{28{1'b0}},error_squared};
            command_sq_sum<=command_sq_sum+{{44{1'b0}},command_squared};
            if(error_abs>error_peak) error_peak<=error_abs>65535 ? 16'hffff : error_abs[15:0];
            if(command_abs>command_peak) command_peak<=command_abs;
            if(command!=0) begin
                if(have_nonzero && previous_nonzero_negative!=command[9]) reversals<=reversals+1'b1;
                previous_nonzero_negative<=command[9]; have_nonzero<=1;
            end
        end
    end
endmodule
