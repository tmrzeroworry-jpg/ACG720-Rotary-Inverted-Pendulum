`timescale 1ns/1ps
module tb_flip_link;
 reg clk=0;always #5 clk=~clk;reg rst=0,tick=0,valid=0,balance=1,permit=1,flipbusy=0;reg[7:0]data;
 wire en,start,negative,abort;wire signed[31:0]v,d;wire[31:0]session;wire[15:0]seq,age;wire[7:0]flags,result;integer starts=0,aborts=0,i;
 reg[7:0]bytes[0:143];
 velocity_stream dut(.life_busy(1'b0),.return_busy(1'b0),.return_safe(1'b1),.return_start(),.return_abort(),.clk(clk),.rst_n(rst),.tick(tick),.rx_data(data),.rx_valid(valid),.balance(balance),.permit(permit),.local_cancel(1'b0),.task_idle(1'b1),.start_quiet(1'b1),
  .flip_busy(flipbusy),.flip_safe(1'b1),.flip_start(start),.flip_negative(negative),.flip_abort(abort),
  .enabled(en),.velocity_q16(v),.desired_q16(d),.session(session),.ack_seq(seq),.age_ms(age),.flags(flags),.result(result));
 always @(posedge clk)begin if(start)starts=starts+1;if(abort)aborts=aborts+1;end
 task send;input integer f;begin for(i=f*24;i<(f+1)*24;i=i+1)begin @(negedge clk);data=bytes[i];valid=1;@(negedge clk);valid=0;end repeat(5)@(negedge clk);end endtask
 task ms;begin @(negedge clk);tick=1;@(negedge clk);tick=0;repeat(2)@(negedge clk);end endtask
 initial begin
  $readmemh("tests/flip_commands.hex",bytes);repeat(3)@(negedge clk);rst=1;send(0);send(1);
  if(starts!=1||!negative||en)$fatal(1,"flip start routing");
  flipbusy=1;balance=0;permit=0;repeat(100)ms();send(2);if(age!=0||seq!=3||!flags[5])$fatal(1,"heartbeat outside BAL");
  send(3);if(starts!=1)$fatal(1,"duplicate busy start queued");
  repeat(200)ms();send(4);if(age<199 || seq!=4)$fatal(1,"wrong session renewed watchdog");
  repeat(105)ms();if(aborts==0)$fatal(1,"flip watchdog");
  send(5);if(aborts<2)$fatal(1,"explicit cancel");
  $display("PASS flip command single-start, signed direction, nonBAL lease/heartbeat, busy rejection, wrong-session exclusion, watchdog/cancel");$finish;
 end
endmodule
