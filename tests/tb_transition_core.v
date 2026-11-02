`timescale 1ns/1ps
module tb_transition_core;
 reg clk=0;always #5 clk=~clk;reg rst=0,valid=0,start=0,online=1;reg[15:0]raw=20000;
 wire tick,dvalid;wire[3:0]state;wire[7:0]fault,phase,result;wire signed[31:0]dp,dd;integer i,failseam=0,tmp;
 swing_controller #(.CLK_FREQ_HZ(100000),.CALIB_SAMPLES(8),.USE_BOOT_REFERENCE(1)) dut(
 .clk(clk),.rst_n(rst),.profile_id(2'd0),.euro_mode(2'd1),.euro_p_quiet(1'b0),.euro_a_quiet(1'b1),.recalibrate_pulse(1'b0),.candidate_balance_rate(32'sd0),.candidate_rate_ready(1'b1),
 .angle_raw(raw),.angle_valid(valid),.angle_online(online),.arm_position(32'sd0),.arm_velocity(32'sd0),.start_fast(1'b1),.return_start(1'b0),.return_cancel(1'b0),.start_pulse(start),.brake_pulse(1'b0),
 .task_mode(2'd0),.task_cmd(1'b0),.task_cfg(8'd0),.flip_start(1'b0),.flip_negative(1'b0),.flip_late(1'b0),.flip_cancel(1'b0),.stream_enable(1'b0),.stream_velocity_q16(32'sd0),
 .state(state),.fault_code(fault),.life_down_valid(dvalid),.life_phase(phase),.life_result(result),.control_tick_o(tick),.diag_p(dp),.diag_d(dd));
 task sample;input integer v;begin raw=v;valid=1;@(negedge clk);valid=0;while(!tick)@(negedge clk);@(negedge clk);repeat(5)@(negedge clk);end endtask
 initial begin
  tmp=$value$plusargs("FAILSEAM=%d",failseam);repeat(4)@(negedge clk);rst=1;
  for(i=0;i<1100;i=i+1)sample(20000);if(state!=1 || !dvalid)$fatal(1,"core boot/rest %d %d",state,dvalid);
  start=1;@(negedge clk);start=0;for(i=0;i<140;i=i+1)sample(20000);
  for(i=1;i<=74;i=i+1)sample(20000+80*i);
  if(!dut.life_wait || state!=9)$fatal(1,"seam not entered %d %d",state,phase);
  for(i=0;i<100;i=i+1)sample(-4);
  if(state!=9 || result!=0)$fatal(1,"integrated core early exit, state=%d fault=%d life=%d",state,fault,result);
  if(failseam)begin
   for(i=0;i<70;i=i+1)sample(-4);
   if(fault!=1 || result!=4 || dp[7:0]!=1 || dp[15:8]!=1)$fatal(1,"primary timeout overwritten fault=%d life=%d diag=%h",fault,result,dp);
  end else begin
   sample(450);sample(500);sample(550);sample(600);sample(650);sample(700);sample(750);sample(800);sample(850);sample(900);
   if(state!=9 || result!=0)$fatal(1,"bridge core exit %d %d",fault,result);
   online=0;repeat(5)sample(900);
   if(fault!=1 || result!=3 || dp[15:8]!=1)$fatal(1,"primary offline overwritten fault=%d life=%d diag=%h",fault,result,dp);
  end
  $display("PASS full-core seam/exit integration and frozen primary fault, scenario %d",failseam);$finish;
 end
endmodule
