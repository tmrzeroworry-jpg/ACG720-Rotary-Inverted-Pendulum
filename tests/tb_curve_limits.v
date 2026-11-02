`timescale 1ns/1ps
module tb_curve_limits;
 reg clk=0;always #5 clk=~clk;reg rst=0,tick=0,valid=0,permit=1;reg[7:0]data=0;
 wire enabled;wire signed[31:0]v,desired;wire[7:0]flags,result;wire[15:0]ack,age;wire[31:0]session;
 velocity_stream #(.MAX_V_Q16(68157),.MAX_A_STEP(136)) dut(.clk(clk),.rst_n(rst),.tick(tick),.rx_data(data),.rx_valid(valid),.balance(1'b1),.permit(permit),.local_cancel(1'b0),.task_idle(1'b1),.start_quiet(1'b1),.life_busy(1'b0),.return_busy(1'b0),.return_safe(1'b1),.flip_busy(1'b0),.flip_safe(1'b1),.enabled(enabled),.velocity_q16(v),.desired_q16(desired),.session(session),.ack_seq(ack),.age_ms(age),.flags(flags),.result(result));
 integer i,k,n,sn=0;reg[7:0]b[0:23];reg[15:0]crc;reg signed[31:0]prev;
 function[15:0]nextcrc;input[15:0]old;input[7:0]x;reg[15:0]c;integer j;begin c=old^{x,8'd0};for(j=0;j<8;j=j+1)c=c[15]?(c<<1)^16'h1021:c<<1;nextcrc=c;end endfunction
 task send;input[7:0]op;input signed[31:0]vel;input[15:0]a;
 begin sn=sn+1;{b[0],b[1],b[2],b[3],b[4],b[5],b[6],b[7],b[8],b[9],b[10],b[11],b[12],b[13],b[14],b[15],b[16],b[17],b[18],b[19]}={32'haac30118,op,8'd0,sn[15:0],32'h12345678,vel,a,16'd300};crc=16'hffff;for(i=0;i<20;i=i+1)crc=nextcrc(crc,b[i]);b[20]=crc[15:8];b[21]=crc[7:0];b[22]=13;b[23]=10;
 for(i=0;i<24;i=i+1)begin @(negedge clk);data=b[i];valid=1;@(negedge clk);valid=0;end repeat(4)@(negedge clk);end endtask
 task ms;begin prev=v;@(negedge clk);tick=1;@(negedge clk);tick=0;if(v-prev>136||prev-v>136||v>68157||v< -68157)$fatal(1,"expanded speed/slew bound");end endtask
 initial begin
 repeat(4)@(negedge clk);rst=1;send(0,0,136);send(1,0,136);if(!enabled)$fatal(1,"start");
 for(k=0;k<6;k=k+1)begin send(2,68157,136);repeat(100)ms();end
 if(v!=68157)$fatal(1,"cannot reach360 cap");send(2,68158,136);if(result!=2||desired!=68157)$fatal(1,"speed out-of-range accepted");send(2,0,137);if(result!=2||desired!=68157)$fatal(1,"acceleration out-of-range accepted");
 for(k=0;k<11;k=k+1)begin send(2,-68157,136);repeat(100)ms();end
 if(v!= -68157)$fatal(1,"reverse ramp");send(3,0,136);repeat(600)ms();if(v!=0||desired!=0)$fatal(1,"soft stop at high speed");
 send(0,0,136);send(1,68157,136);repeat(310)ms();if(!flags[2]||desired!=0)$fatal(1,"expanded watchdog");repeat(600)ms();if(v!=0)$fatal(1,"watchdog failed to decelerate");
 permit=0;repeat(4)@(negedge clk);if(enabled||v!=0)$fatal(1,"S0/permit");
 $display("PASS +/-68157 Q16,136 Q16/ms, over-limit rejection, full-speed reversal/soft-stop, watchdog and S0");$finish;
 end
endmodule
