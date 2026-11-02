`timescale 1ns/1ps
module tb_remote_hold;
 reg clk=0;always #5 clk=~clk;reg rst=0,tick=0,cmd=0;reg[1:0]mode=0;reg[7:0]cfg=0;
 wire signed[31:0]offset;wire moving,idle;reg signed[31:0]prev;integer n;reg signed[31:0]holdpos;
 arm_task dut(.clk(clk),.rst_n(rst),.tick(tick),.active(1'b1),.cmd_pulse(cmd),.cmd_mode(mode),.cmd_cfg(cfg),.stream_enable(1'b0),.stream_velocity_q16(32'sd0),.offset(offset),.moving(moving),.idle(idle));
 task ms;begin @(negedge clk);tick=1;@(negedge clk);tick=0;repeat(8)@(negedge clk);end endtask
 task send;input[1:0]m;input[7:0]c;begin @(negedge clk);mode=m;cfg=c;cmd=1;@(negedge clk);cmd=0;repeat(5)@(negedge clk);end endtask
 initial begin
 repeat(4)@(negedge clk);rst=1;repeat(5)@(negedge clk);
 send(2,8);repeat(1200)ms();if(!moving||idle||offset<300)$fatal(1,"spin fixture");
 send(1,128);prev=dut.rate_q16;
 for(n=0;n<1100;n=n+1)begin ms();if(dut.rate_q16>prev||prev-dut.rate_q16>34)$fatal(1,"positive hold slew");prev=dut.rate_q16;end
 holdpos=offset;repeat(100)ms();if(offset!=holdpos||moving||idle)$fatal(1,"hold drifts/home enabled");
 send(2,24);repeat(1200)ms();if(dut.rate_q16>=0)$fatal(1,"negative fixture");send(1,128);prev=dut.rate_q16;
 for(n=0;n<1100;n=n+1)begin ms();if(dut.rate_q16<prev||dut.rate_q16-prev>34)$fatal(1,"negative hold slew");prev=dut.rate_q16;end
 holdpos=offset;repeat(100)ms();if(offset!=holdpos||moving||idle)$fatal(1,"negative hold");
 $display("PASS positive/negative reference deceleration bounded34 Q16/ms, no offset jump, hold disables auto-home");$finish;
 end
endmodule
