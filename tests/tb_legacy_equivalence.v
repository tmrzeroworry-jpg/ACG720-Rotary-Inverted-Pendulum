`timescale 1ns/1ps
module tb_legacy_equivalence;
 reg clk=0,rst=0;always #5 clk=~clk;
 reg[31:0]energy=0;reg signed[31:0]ang=10000,vel=2000;reg rot=0;reg[15:0]timer=0;integer i;
 swing_controller n(.clk(clk),.rst_n(rst));
 swing_controller_gold o(.clk(clk),.rst_n(rst));
 initial begin
  force n.state=1;force o.state=1;force n.control_tick=0;force o.control_tick=0;
  force n.lqr_profile=0;force o.lqr_profile=0;force n.low_dynamics=0;force o.low_dynamics=0;
  force n.euro_active=0;force o.euro_active=0;force n.wt_mode=0;force o.wt_mode=0;
  force n.energy_now=energy;force o.energy_now=energy;
  force n.oriented_upright_error=ang;force o.oriented_upright_error=ang;
  force n.oriented_pend_velocity=vel;force o.oriented_pend_velocity=vel;
  force n.rotating=rot;force o.rotating=rot;force n.state_timer_ms=timer;force o.state_timer_ms=timer;
  repeat(4)@(negedge clk);rst=1;repeat(4)@(negedge clk);
  force n.pipe_busy=1;force o.pipe_busy=1;force n.pipe_state=2;force o.pipe_state=2;force n.pipe_phase=1;force o.pipe_phase=1;
  for(i=0;i<1200;i=i+1)begin
   energy=(i*47591)%60000;ang=(i*977)%26401-13200;vel=(i%2 ? -1:1)*(i*61);rot=(i%5==0);timer=i*15;
   force n.swing_energy_ready=0;@(negedge clk);release n.swing_energy_ready;
   repeat(5)@(negedge clk);
   if(n.s1_a!==o.s1_a)$fatal(1,"legacy pump differs case %d new %d old %d",i,n.s1_a,o.s1_a);
  end
  $display("PASS 1200 exact legacy energy/clamp/sign cases against unmodified TR04, with one-clock retiming");$finish;
 end
endmodule
