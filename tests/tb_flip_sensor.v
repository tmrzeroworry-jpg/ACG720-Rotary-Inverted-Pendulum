`timescale 1ns/1ps
module tb_flip_sensor;
 reg clk=0;always #5 clk=~clk;reg rst=0,tick=0,start=0,active=0,neg=0,valid=0,online=1;
 reg[15:0]raw=6251,seed=6251;wire[15:0]angle,age;wire fresh,waiting,bridge;wire[7:0]fault,diag;
 flip_angle_tracker dut(.clk(clk),.rst_n(rst),.tick(tick),.start(start),.active(active),.negative(neg),.raw_valid(valid),.online(online),.raw(raw),.seed(seed),.angle(angle),.fresh(fresh),.waiting(waiting),.bridge(bridge),.fault(fault),.seam_age(age),.diagnostic(diag));
 integer i;
 task reset;input integer s,n;begin rst=0;active=0;start=0;tick=0;valid=0;online=1;neg=n;seed=s;repeat(4)@(negedge clk);rst=1;repeat(3)@(negedge clk);start=1;active=1;@(negedge clk);start=0;end endtask
 task sample;input integer value;begin raw=value;valid=1;@(negedge clk);valid=0;repeat(2)@(negedge clk);tick=1;@(negedge clk);tick=0;repeat(3)@(negedge clk);end endtask
 initial begin
  reset(25800,0);sample(25900);if(!waiting)$fatal(1,"positive seam not armed");
  sample(26900);sample(-250);sample(18000);sample(100);sample(450);sample(500);sample(550);
  if(waiting || !bridge || fault || angle!=550)$fatal(1,"positive seam failed %d %d",angle,fault);
  sample(650);if(fault)$fatal(1,"positive continuation");
  reset(800,1);sample(550);if(!waiting)$fatal(1,"negative seam not armed");
  sample(-200);sample(7000);sample(26300);sample(25900);sample(25800);sample(25700);
  if(waiting || !bridge || fault || angle!=25700)$fatal(1,"negative seam failed");
  sample(25600);if(fault)$fatal(1,"negative continuation");
  reset(6200,0);sample(-200);if(fault!=3)$fatal(1,"nonseam bad word accepted");
  reset(6200,0);sample(10000);if(fault!=2)$fatal(1,"nonseam jump accepted");
  reset(25800,0);sample(25900);for(i=0;i<81;i=i+1)sample(-200);
  if(fault!=1 || fresh)$fatal(1,"seam not bounded");
  reset(25800,0);sample(25900);online=0;#1;if(fresh)$fatal(1,"offline allowed");
  $display("PASS endpoint crossing both ways, signed offsets/spikes, three real exit samples, no fake progress, bounded wait, outside-seam and I2C guards");$finish;
 end
endmodule
