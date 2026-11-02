`timescale 1ns/1ps
module tb_curve_handoff;
 reg clk=0;always #5 clk=~clk;reg rst=0,tick=0,valid=0;reg[7:0]data=0;
 wire owned,taskp,enabled;wire signed[31:0]v,desired,offset;wire[1:0]mode;wire[7:0]cfg,result;wire[15:0]seq,age;integer i,k,c4seq=0,c3seq=0,count=0;reg[7:0]b[0:23];reg[15:0]crc;reg signed[31:0]prior_offset;
 remote_control c(.clk(clk),.rst_n(rst),.tick(tick),.rx_valid(valid),.rx_data(data),.stop_n(1'b1),.sensor_ready(1'b1),.start_ready(1'b1),.keys_released(1'b1),.ctrl_state(4'd4),.life_busy(1'b0),.flip_busy(1'b0),.stream_enabled(enabled&&(v!=0||desired!=0)),.owned(owned),.task_pulse(taskp),.task_mode(mode),.task_cfg(cfg),.result(result));
 velocity_stream #(.MAX_V_Q16(68157),.MAX_A_STEP(136)) s(.clk(clk),.rst_n(rst),.tick(tick),.rx_valid(valid),.rx_data(data),.balance(1'b1),.permit(1'b1),.local_cancel(taskp),.task_idle(1'b1),.start_quiet(1'b1),.life_busy(1'b0),.return_busy(1'b0),.return_safe(1'b1),.flip_busy(1'b0),.flip_safe(1'b1),.enabled(enabled),.velocity_q16(v),.desired_q16(desired));
 arm_task a(.clk(clk),.rst_n(rst),.tick(tick),.active(1'b1),.cmd_pulse(taskp),.cmd_mode(mode),.cmd_cfg(cfg),.stream_enable(enabled),.stream_velocity_q16(v),.offset(offset));
 always @(posedge clk)if(taskp)count=count+1;
 function[15:0]nextcrc;input[15:0]old;input[7:0]x;reg[15:0]c;integer j;begin c=old^{x,8'd0};for(j=0;j<8;j=j+1)c=c[15]?(c<<1)^16'h1021:c<<1;nextcrc=c;end endfunction
 task send;input web;input[7:0]op;input signed[31:0]arg;reg[15:0]num,acc,ttl;reg[31:0]sid;
 begin
 if(web)begin c4seq=c4seq+1;num=c4seq;sid=32'h33333333;acc=0;ttl=500;end else begin c3seq=c3seq+1;num=c3seq;sid=32'h44444444;acc=136;ttl=300;end
 {b[0],b[1],b[2],b[3],b[4],b[5],b[6],b[7],b[8],b[9],b[10],b[11],b[12],b[13],b[14],b[15],b[16],b[17],b[18],b[19]}={8'haa,(web?8'hc4:8'hc3),8'd1,8'd24,op,8'd0,num,sid,arg,acc,ttl};crc=16'hffff;for(i=0;i<20;i=i+1)crc=nextcrc(crc,b[i]);b[20]=crc[15:8];b[21]=crc[7:0];b[22]=13;b[23]=10;
 for(i=0;i<24;i=i+1)begin @(negedge clk);data=b[i];valid=1;@(negedge clk);valid=0;end repeat(8)@(negedge clk);end endtask
 task ms;begin @(negedge clk);tick=1;@(negedge clk);tick=0;repeat(8)@(negedge clk);end endtask
 initial begin repeat(4)@(negedge clk);rst=1;
 send(1,0,0);send(0,0,0);send(0,1,17039);repeat(100)ms();send(1,1,0);send(1,3,4);if(count!=0||result!=3)$fatal(1,"moving stream task conflict");
 send(0,3,0);for(k=0;k<4;k=k+1)begin send(1,1,0);repeat(100)ms();end
 if(v!=0||desired!=0||!enabled)$fatal(1,"stationary stream fixture");prior_offset=offset;send(1,3,4);
 if(count!=1||enabled||mode!=1||cfg!=4||offset!=prior_offset)$fatal(1,"no continuous handoff from stopped curve");
 repeat(20)ms();if(offset==prior_offset)$fatal(1,"position task did not take over");
 $display("PASS moving curve rejects task; stopped curve transfers without reference jump and disables old stream");$finish;end
endmodule
