`timescale 1ns/1ps
module tb_stream_reference;
 reg clk=0;always #5 clk=~clk;reg rst=0,tick=0,active=1,cmd=0,enabled=0;reg signed[31:0]v=0;
 wire signed[31:0]offset;wire moving,idle;integer i;
 arm_task dut(.clk(clk),.rst_n(rst),.tick(tick),.active(active),.cmd_pulse(cmd),.cmd_mode(2'd1),.cmd_cfg(8'd1),
 .stream_enable(enabled),.stream_velocity_q16(v),.offset(offset),.moving(moving),.idle(idle));
 task ms;begin @(negedge clk);tick=1;@(negedge clk);tick=0;repeat(4)@(negedge clk);end endtask
 initial begin repeat(3)@(negedge clk);rst=1;enabled=1;v=65536/4;
  repeat(1000)ms();if(offset!=250 || idle)$fatal(1,"stream integration count=%0d",offset);
  v=0;repeat(50)ms();if(offset!=250)$fatal(1,"stop reference jumped");
  // Physical task command exactly coincident with control tick must win.
  @(negedge clk);cmd=1;tick=1;@(negedge clk);cmd=0;tick=0;enabled=0;repeat(4)@(negedge clk);
  if(dut.mode!=1)$fatal(1,"physical task lost on same-tick takeover");
  active=0;repeat(4)@(negedge clk);if(offset!=0)$fatal(1,"loss BAL reset");
  $display("PASS reference integration, zero hold, synchronous task takeover, BAL loss");$finish;
 end
endmodule
