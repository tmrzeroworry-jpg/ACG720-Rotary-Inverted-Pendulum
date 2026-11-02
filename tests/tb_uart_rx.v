`timescale 1ns/1ps
module tb_uart_rx;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,rx=1;wire[7:0]data;wire valid,err;integer n=0,e=0;reg[7:0]got[0:3];
 uart_rx_byte #(.CLK_FREQ_HZ(921600),.BAUD_RATE(115200)) dut(.clk(clk),.rst_n(rst),.uart_rx(rx),.data(data),.valid(valid),.framing_error(err));
 always @(posedge clk)begin if(valid)begin got[n]=data;n=n+1;end if(err)e=e+1;end
 task bitcell;input v;begin rx=v;repeat(8)@(negedge clk);end endtask
 task send;input[7:0]b;input stopbit;integer k;begin bitcell(0);for(k=0;k<8;k=k+1)bitcell(b[k]);bitcell(stopbit);bitcell(1);end endtask
 initial begin repeat(4)@(negedge clk);rst=1;repeat(4)@(negedge clk);
  send(8'haa,1);send(8'hc3,1);send(8'h00,1);send(8'hff,0);repeat(20)@(negedge clk);
  if(n!=3 || got[0]!=8'haa || got[1]!=8'hc3 || got[2]!=0 || e!=1)$fatal(1,"UART bit/framing mismatch n=%0d err=%0d",n,e);
  $display("PASS UART synchronizer, LSB ordering, stop-bit rejection");$finish;
 end
endmodule
