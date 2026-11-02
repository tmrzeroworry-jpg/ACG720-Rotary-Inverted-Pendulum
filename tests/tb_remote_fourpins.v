`timescale 1ns/1ps
module tb_remote_fourpins;
 reg clk=0;always #5 clk=~clk;reg stop=1;wire pwm,a1,a2,stby;wire scl,sda;
 ab4_top t(.clk_50m(clk),.key_stop_n(stop),.key_test_n(4'b1111),.sw_n(8'hff),.enc_a(1'b0),.enc_b(1'b0),.uart_rx(1'b1),.i2c_scl(scl),.i2c_sda(sda),.motor_pwm(pwm),.motor_ain1(a1),.motor_ain2(a2),.motor_stby(stby));
 task arm_fixture;begin
 force t.selector.trial_active=1;force t.selector.stop_latched=0;
 force t.controller_enable=1;force t.pwm_i=1;force t.ain1_i=1;force t.ain2_i=1;force t.stby_i=1;
 #1;if({pwm,a1,a2,stby}!=15)$fatal(1,"fixture");release t.selector.trial_active;release t.selector.stop_latched;
 end endtask
 initial begin
 force t.sys_rst_n=0;repeat(4)@(negedge clk);force t.sys_rst_n=1;force t.state=4;repeat(6)@(negedge clk);
 arm_fixture();force t.remote_stop=1;#1;if({pwm,a1,a2,stby}!=0)$fatal(1,"web-stop did not gate all four pins");repeat(4)@(negedge clk);release t.remote_stop;repeat(5)@(negedge clk);
 if({pwm,a1,a2,stby}!=0)$fatal(1,"web-stop did not latch");arm_fixture();stop=0;#1;if({pwm,a1,a2,stby}!=0)$fatal(1,"S0 is not raw asynchronous");repeat(5)@(negedge clk);stop=1;repeat(5)@(negedge clk);if({pwm,a1,a2,stby}!=0)$fatal(1,"S0 auto resumed");
 $display("PASS full-top web-stop four-pin gate+latch and independent raw S0 gate+latch");$finish;
 end
endmodule
