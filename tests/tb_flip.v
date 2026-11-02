`timescale 1ns/1ps
module tb_flip;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,tick=0,start=0,cancel=0,negative=0,fresh=1;reg[15:0]angle=6251;
 reg[3:0]state=4;wire busy,takeover,capture,abort;wire[7:0]phase,result;wire[15:0]ident,elapsed;wire signed[31:0]progress;wire signed[9:0]command;
 flip_maneuver dut(.clk(clk),.rst_n(rst),.tick(tick),.start(start),.cancel(cancel),.negative(negative),.late(1'b0),.fresh(fresh),
  .seam_wait(1'b0),.seam_bridge(1'b0),.sensor_fault(8'd0),.angle(angle),.up(16'd6251),.ctrl_state(state),.arm_position(32'sd0),.arm_velocity(32'sd0),.seed_command(10'sd0),
  .busy(busy),.takeover(takeover),.capture(capture),.abort(abort),.phase(phase),.result(result),.ident(ident),.elapsed(elapsed),.progress(progress),.command(command));
 integer i,p,sgn,last,scenario=0;reg[15:0]oldid;
 task reset;begin rst=0;tick=0;start=0;cancel=0;fresh=1;state=4;angle=6251;repeat(4)@(negedge clk);rst=1;repeat(2)@(negedge clk);end endtask
 task begin_flip;begin start=1;@(negedge clk);start=0;repeat(3)@(negedge clk);if(!busy||!ident)$fatal(1,"start rejected");state=8;end endtask
 task step;input integer travel;begin
   p=6251+(negative?-travel:travel);p=p%26400;if(p<0)p=p+26400;angle=p;
   last=command;@(negedge clk);tick=1;@(negedge clk);tick=0;repeat(20)@(negedge clk);
   if(phase<=2 && busy && (command-last>2 || last-command>2))$fatal(1,"PWM slew");
   if(command>100||command< -100)$fatal(1,"PWM cap");
 end endtask
 initial begin
  for(sgn=0;sgn<2;sgn=sgn+1)begin
   reset;negative=sgn;begin_flip;oldid=ident;
   // Duplicate START while busy must never schedule another revolution.
   start=1;@(negedge clk);start=0;repeat(3)@(negedge clk);if(ident!=oldid)$fatal(1,"duplicate start");
   for(i=1;i<=1000;i=i+1)begin
    p=$rtoi(13200.0*(1.0-$cos(3.141592653589793*i/1000.0)));step(p);
    if(i<1000 && phase==3)$fatal(1,"caught before full revolution");
   end
   if(phase!=3 || progress<26400)$fatal(1,"full turn not captured phase%0d prog%0d",phase,progress);
   state=3;repeat(250)step(26400);state=4;repeat(505)step(26400);
   if(phase!=5||result!=0||busy)$fatal(1,"stable confirmation");
  end
  reset;negative=0;begin_flip;
  for(i=1;i<=300;i=i+1)step(i*44);
  for(i=299;i>=0 && busy;i=i-1)step(i*44);
  if(phase!=6||result!=3)$fatal(1,"half swing falsely accepted %d",result);
  reset;begin_flip;step(1000);if(result!=4||phase!=6)$fatal(1,"ADC jump");
  reset;begin_flip;fresh=0;step(0);if(result!=9)$fatal(1,"sensor gate");
  reset;begin_flip;cancel=1;step(0);cancel=0;if(result!=8)$fatal(1,"cancel gate");
  reset;begin_flip;for(i=0;i<6010&&busy;i=i+1)step(0);if(result!=2)$fatal(1,"timeout");
  reset;state=1;start=1;@(negedge clk);start=0;repeat(3)@(negedge clk);if(busy||ident!=0||result!=1)$fatal(1,"READY start accepted");
  $display("PASS both directions, real wrap, full-turn-only catch, hold, no-repeat, reverse, ADC, sensor, cancel, timeout, start gates, slew/cap");$finish;
 end
endmodule
