// Protocol fixture only. Compiles original serializer; no FPGA download.
`timescale 1ns/1ps
module tb_wexh_uart;
 reg clk=0;always #10 clk=~clk;
 reg rst=0;reg[15:0]bal=0;
 wire tx;
 integer file,n=0;
 telemetry_ab4 #(.BUILD_ID(32'h57455848),.INCLUDE_ARM_TARGET(1)) dut(
  .clk(clk),.rst_n(rst),.trial_id(32'd1),.profile(8'd5),
  .state(8'd4),.fault(8'd0),.flags(8'h59),.raw(16'd6223),.error(-18'sd77),
  .rate(-32'sd100),.arm_position(-32'sd1040),.arm_rate(32'sd520),
  .command(-10'sd85),.down(16'd20100),.up(16'd6300),.trim(-16'sd73),
  .p(32'sd1),.d(-32'sd2),.arm_p(32'sd3),.arm_d(-32'sd4),
  .linear(-32'sd2),.compensated(-32'sd2),.target(-10'sd80),
  .balance_ms(bal),.saturation_ms(16'd2),.reversals(16'd3),
  .error_sq_sum(64'd123),.command_sq_sum(64'd456),.error_peak(16'd77),.command_peak(16'd85),
  .arm_target_reference(-32'sd910),.cfg_event(1'b0),.cfg_payload(96'd0),
  .ir_event(1'b0),.ir_address(16'd0),.ir_command(8'd0),.ir_repeat(1'b0),
  .ir_learning(1'b0),.ir_ready(1'b0),.ir_stage(3'd0),.ir_action(8'd0),.ir_errors(16'd0),.uart_tx(tx));
 always @(negedge clk) bal=16'd65530+dut.frame_sequence*16'd10;
 always @(posedge clk)if(rst && dut.tx_start)begin
  $fwrite(file,"%02x\n",dut.tx_data);n=n+1;
  if(n==648)begin $fclose(file);$display("PASS 6 original WEXH UART frames");$finish;end
 end
 initial begin
  file=$fopen("wexh_uart_vectors.hex","w");
  #100;rst=1;
  #100000000;$fatal(1,"UART fixture timeout");
 end
endmodule
