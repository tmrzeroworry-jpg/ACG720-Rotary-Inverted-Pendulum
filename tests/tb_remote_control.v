`timescale 1ns/1ps
module tb_remote_control;
 reg clk=0;always #5 clk=~clk;reg rst=0,tick=0,valid=0,stop=1,ready=1,life=0,flip=0,stream=0;
 reg[7:0]data=0,sw=0;reg[3:0]state=1,keys=15;
 wire owned,start,fast,taskp,stopp,allowed,start_out,fast_out,task_out,trip,canstart;
 wire[1:0]mode,modeout;wire[7:0]cfg,cfgout,op,result,flags;
 wire[31:0]session;wire[15:0]seq,age;integer starts=0,tasks=0,i,n;reg[15:0]c;reg[7:0]b[0:23];
 remote_control r(.clk(clk),.rst_n(rst),.tick(tick),.rx_valid(valid),.rx_data(data),.stop_n(stop),.sensor_ready(ready),
 .start_ready(canstart),.keys_released(&keys),.ctrl_state(state),.life_busy(life),.flip_busy(flip),.stream_enabled(stream),
 .owned(owned),.start_pulse(start),.start_fast(fast),.task_pulse(taskp),.stop_pulse(stopp),.task_mode(mode),.task_cfg(cfg),
 .session(session),.ack_seq(seq),.age_ms(age),.ack_op(op),.result(result),.flags(flags));
 test_selector #(.REMOTE_ENABLE(1),.CLK_FREQ_HZ(1000),.DEBOUNCE_MS(2)) s(.clk(clk),.rst_n(rst),.stop_n(stop),.recal_n(1'b1),
 .test_n(keys),.sw_n(sw),.ctrl_state(state),.sensor_ready(ready),.remote_owned(owned),.remote_start(start),.remote_fast(fast),
 .remote_stop(stopp),.remote_task(taskp),.remote_mode(mode),.remote_cfg(cfg),.remote_ready(canstart),.start_pulse(start_out),
 .start_fast(fast_out),.task_cmd(task_out),.task_mode(modeout),.task_cfg(cfgout),.drive_allowed(allowed),.stop_trip(trip));
 always @(posedge clk)begin if(start_out)starts=starts+1;if(task_out)tasks=tasks+1;end
 function[15:0]crc;input[15:0]old;input[7:0]v;reg[15:0]x;integer k;begin x=old^{v,8'd0};for(k=0;k<8;k=k+1)x=x[15]?(x<<1)^16'h1021:x<<1;crc=x;end endfunction
 task send;input[7:0]cmd;input[15:0]sn;input[31:0]sid,arg;input bad;
 begin
 {b[0],b[1],b[2],b[3],b[4],b[5],b[6],b[7],b[8],b[9],b[10],b[11],b[12],b[13],b[14],b[15],b[16],b[17],b[18],b[19]}={32'haac40118,cmd,8'd0,sn,sid,arg,32'h000001f4};
 c=16'hffff;for(i=0;i<20;i=i+1)c=crc(c,b[i]);b[20]=c[15:8];b[21]=c[7:0]^bad;b[22]=13;b[23]=10;
 for(i=0;i<24;i=i+1)begin @(negedge clk);data=b[i];valid=1;@(negedge clk);valid=0;end
 repeat(5)@(negedge clk);
 end endtask
 task ms;begin @(negedge clk);tick=1;@(negedge clk);tick=0;repeat(3)@(negedge clk);end endtask
 initial begin
 repeat(4)@(negedge clk);rst=1;repeat(30)@(negedge clk);
 send(2,1,1,0,0);if(starts!=0)$fatal(1,"start before claim");
 send(0,1,32'h12345678,0,0);if(!owned)$fatal(1,"claim");
 send(2,2,32'h12345678,0,0);if(starts!=1||!fast_out||!allowed)$fatal(1,"fast start ignores physical SW");
 send(2,2,32'h12345678,0,0);if(starts!=1)$fatal(1,"duplicate start");
 state=9;send(2,3,32'h12345678,1,0);if(starts!=1||result!=3)$fatal(1,"busy start queued");
 state=4;repeat(10)@(negedge clk);
 for(n=0;n<8;n=n+1)begin send(3,4+n,32'h12345678,n,0);if(modeout!=1||cfgout!=n)$fatal(1,"position mapping");end
 for(n=0;n<4;n=n+1)begin send(4,12+n,32'h12345678,n,0);if(modeout!=2||cfgout!={3'd0,n[1],n[0],3'd0})$fatal(1,"spin mapping");end
 for(n=0;n<8;n=n+1)begin send(5,16+n,32'h12345678,n,0);if(modeout!=3||cfgout!={1'd0,n[2:0],4'd0})$fatal(1,"sine mapping");end
 send(6,24,32'h12345678,0,0);if(modeout!=1||cfgout!=128)$fatal(1,"hold mapping");
 n=tasks;keys=13;repeat(20)@(negedge clk);keys=15;repeat(20)@(negedge clk);if(tasks!=n)$fatal(1,"physical task conflicts with remote");
 life=1;send(3,25,32'h12345678,0,0);if(tasks!=n||result!=3)$fatal(1,"life conflict");life=0;
 flip=1;send(5,26,32'h12345678,0,0);if(tasks!=n||result!=3)$fatal(1,"flip conflict");flip=0;
 stream=1;send(4,27,32'h12345678,0,0);if(tasks!=n||result!=3)$fatal(1,"stream conflict");stream=0;
 send(3,28,32'h12345678,8,0);if(tasks!=n||result!=2)$fatal(1,"range");
 send(3,29,32'h12345678,0,1);if(seq!=28)$fatal(1,"CRC");
 send(3,30,32'h87654321,0,0);if(seq!=28||session!=32'h12345678)$fatal(1,"wrong session");
 send(0,31,32'h87654321,0,0);if(session!=32'h12345678)$fatal(1,"ownership stolen");
 send(1,29,32'h12345678,0,0);repeat(450)ms();send(1,29,32'h12345678,0,0);repeat(50)ms();
 if(owned||allowed||result!=5)$fatal(1,"watchdog/duplicate heartbeat");
 state=1;repeat(20)@(negedge clk);send(2,30,32'h12345678,0,0);if(starts!=1)$fatal(1,"restart after watchdog");
 send(0,40,3,0,0);send(2,41,3,1,0);if(starts!=2||fast_out)$fatal(1,"legacy start");
 stop=0;#1;if(allowed)$fatal(1,"S0 gate");repeat(6)@(negedge clk);if(owned)$fatal(1,"S0 ownership");stop=1;repeat(20)@(negedge clk);if(allowed)$fatal(1,"S0 auto resume");
 send(0,50,5,0,0);send(7,1,9,0,0);if(owned||allowed||op!=7||result!=0)$fatal(1,"independent web STOP");
 $display("PASS remote commands, 8 positions/8 sine/4 spin, atomic SW-independent mapping, mutual exclusion, CRC, session, duplicate, watchdog, S0 and web-stop");$finish;
 end
endmodule
