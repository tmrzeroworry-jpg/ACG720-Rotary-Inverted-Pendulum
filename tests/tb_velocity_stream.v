`timescale 1ns/1ps
module tb_velocity_stream;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,tick=0;reg[7:0] data=0;reg valid=0;reg balance=1,permit=1,cancel=0,idle=1,quiet=1;
 wire enabled;wire signed[31:0] velocity,desired;wire[31:0]session;wire[15:0]seq,age;wire[7:0]flags,result;
 velocity_stream dut(.life_busy(1'b0),.return_busy(1'b0),.return_safe(1'b1),.return_start(),.return_abort(),.clk(clk),.rst_n(rst),.tick(tick),.rx_data(data),.rx_valid(valid),.balance(balance),.permit(permit),
 .local_cancel(cancel),.flip_busy(1'b0),.flip_safe(1'b1),.task_idle(idle),.start_quiet(quiet),.enabled(enabled),.velocity_q16(velocity),.desired_q16(desired),
 .session(session),.ack_seq(seq),.age_ms(age),.flags(flags),.result(result));
 reg[7:0] bytes[0:167];integer i;reg signed[31:0] prev;
 task send;input integer f;begin
  for(i=f*24;i<(f+1)*24;i=i+1)begin @(negedge clk);data=bytes[i];valid=1;@(negedge clk);valid=0;end
  repeat(4)@(negedge clk);
 end endtask
 task ms;begin
  prev=velocity;@(negedge clk);tick=1;@(negedge clk);tick=0;
  if(velocity-prev>11 || prev-velocity>11)$fatal(1,"acceleration bound violated");
  if(velocity>17039 || velocity< -17039)$fatal(1,"velocity bound violated");
 end endtask
 initial begin
  $readmemh("tests/commands.hex",bytes);repeat(3)@(negedge clk);rst=1;
  send(0);if(session!=32'h12345678 || !flags[5])$fatal(1,"HELLO");
  send(1);if(!enabled || desired<=0)$fatal(1,"START");
  repeat(200)ms();if(velocity!=2200)$fatal(1,"positive ramp");
  send(2);repeat(200)ms();if(velocity!=0)$fatal(1,"reversal ramp");
  send(3);if(desired>=0 || seq!=3)$fatal(1,"duplicate was accepted");
  send(4);if(result!=2 || desired>=0)$fatal(1,"invalid acceleration accepted");
  send(6);if(seq!=4)$fatal(1,"corrupt CRC accepted");
  repeat(120)ms();if(!flags[2] || desired!=0)$fatal(1,"watchdog failed");
  send(2);if(desired!=0)$fatal(1,"timeout auto resumed");
  cancel=1;@(negedge clk);cancel=0;repeat(3)@(negedge clk);
  if(enabled || flags[5] || velocity!=0)$fatal(1,"physical takeover failed");
  send(0);send(1);permit=0;repeat(3)@(negedge clk);if(enabled)$fatal(1,"S0 gate");
  $display("PASS: handshake, bounds, reverse, duplicate/CRC, range, timeout, no-resume, local/S0 priority");$finish;
 end
endmodule
