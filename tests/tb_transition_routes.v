`timescale 1ns/1ps
module tb_transition_routes;
 reg clk=0;always #5 clk=~clk;reg rst=0,stop=1;reg[3:0]keys=15,state=1;reg[7:0]sw=255;wire start,fast,allowed,trip;integer n,starts=0;
 test_selector #(.CLK_FREQ_HZ(1000),.DEBOUNCE_MS(2)) dut(.clk(clk),.rst_n(rst),.stop_n(stop),.recal_n(stop),.test_n(keys),.sw_n(sw),.ctrl_state(state),.sensor_ready(1'b1),.start_pulse(start),.start_fast(fast),.drive_allowed(allowed),.stop_trip(trip));
 always @(posedge clk)if(start)starts=starts+1;
 wire pwm,ain1,ain2,stby;wire scl,sda;
 ab4_top #(.CLK_FREQ_HZ(50000000)) top(.clk_50m(clk),.key_stop_n(stop),.key_test_n(keys),.sw_n(sw),.enc_a(1'b0),.enc_b(1'b0),.uart_rx(1'b1),.i2c_scl(scl),.i2c_sda(sda),.motor_pwm(pwm),.motor_ain1(ain1),.motor_ain2(ain2),.motor_stby(stby));
 initial begin
  repeat(4)@(negedge clk);rst=1;repeat(30)@(negedge clk);keys=14;repeat(12)@(negedge clk);keys=15;repeat(15)@(negedge clk);
  if(starts!=1 || !fast || !allowed)$fatal(1,"default fast S1");
  state=9;repeat(4)@(negedge clk);keys=14;repeat(12)@(negedge clk);keys=15;repeat(15)@(negedge clk);if(starts!=1)$fatal(1,"busy S1 queued");
  state=1;sw=127;repeat(30)@(negedge clk);keys=14;repeat(12)@(negedge clk);keys=15;repeat(15)@(negedge clk);if(starts!=2 || fast)$fatal(1,"SW7 fallback");
  force top.drive_allowed=1;force top.pwm_i=1;force top.ain1_i=1;force top.ain2_i=1;force top.stby_i=1;
  #1;if({pwm,ain1,ain2,stby}!=4'b1111)$fatal(1,"emergency fixture not armed");stop=0;#1;if({pwm,ain1,ain2,stby}!=0)$fatal(1,"S0 not asynchronous to full top");
  repeat(5)@(negedge clk);if(allowed)$fatal(1,"S0 latch");stop=1;repeat(20)@(negedge clk);if(allowed)$fatal(1,"automatic restart after S0");
  $display("PASS SW7 default/fallback selection, busy no queue, S0 asynchronous four-pin gating and latched no restart");$finish;
 end
endmodule
