`timescale 1ns/1ps
module tb_transition_seam;
 reg clk=0;always #5 clk=~clk;reg rst=0,tick=0,start=0,active=0,valid=0,online=1;reg[15:0]raw=0,seed=25800;
 wire[15:0]angle,age;wire fresh,waiting,bridge;wire[7:0]fault,diag;integer i,c=0,tmp;
 transition_angle_guard dut(.clk(clk),.rst_n(rst),.tick(tick),.start(start),.active(active),.raw_valid(valid),.online(online),.raw(raw),.seed(seed),.angle(angle),.fresh(fresh),.waiting(waiting),.bridge(bridge),.fault(fault),.seam_age(age),.diagnostic(diag));
 task reset;begin rst=0;active=0;start=0;tick=0;valid=0;online=1;seed=25800;repeat(4)@(negedge clk);rst=1;repeat(3)@(negedge clk);start=1;active=1;@(negedge clk);start=0;end endtask
 task sample;input integer v;begin raw=v;valid=1;@(negedge clk);valid=0;repeat(2)@(negedge clk);tick=1;@(negedge clk);tick=0;repeat(3)@(negedge clk);end endtask
 initial begin
  tmp=$value$plusargs("CASE=%d",c);reset();sample(25900);if(!waiting)$fatal(1,"not armed");
  if(c==0)begin
   for(i=0;i<100;i=i+1)sample(-4);
   if(fault || !fresh)$fatal(1,"100ms genuine transport incorrectly stopped, sensor reason=%d",fault);
   sample(450);sample(500);if(!waiting)$fatal(1,"less than three observations");sample(550);
   if(waiting || !bridge || fault || angle!=550)$fatal(1,"slow genuine exit");
  end else if(c==1)begin
   sample(450);sample(500);sample(550);sample(600);sample(650);sample(620);sample(590);sample(550);
   if(!waiting)$fatal(1,"reversal inside1200-code zone not rearmed");
   sample(-250);sample(26300);sample(25900);sample(25800);sample(25700);
   if(waiting || fault || !bridge || angle!=25700)$fatal(1,"reverse crossing failed");
  end else if(c==2)begin
   sample(-200);sample(100);sample(26000);sample(25300);sample(25200);sample(25100);
   if(waiting || fault || !bridge || angle!=25100)$fatal(1,"same-edge retreat not observed");
  end else if(c==3)begin
   for(i=0;i<161;i=i+1)sample(-250);if(fault!=1 || fresh)$fatal(1,"unbounded analog wait");
  end else if(c==4)begin
   sample(450);sample(500);sample(25300);if(!waiting)$fatal(1,"mixed exit votes accepted");
   sample(550);sample(600);if(!waiting)$fatal(1,"old votes reused");sample(650);if(waiting || fault)$fatal(1,"votes did not restart");
  end else if(c==5)begin
   valid=0;repeat(22)begin @(negedge clk);tick=1;@(negedge clk);tick=0;repeat(3)@(negedge clk);end
   if(fresh)$fatal(1,"stale ADC accepted during seam");
   online=0;#1;if(fresh)$fatal(1,"offline accepted");
  end
  $display("PASS seam case %d, real-sample exits and guards",c);$finish;
 end
endmodule
