`timescale 1ns/1ps
module tb_flip_variants;
 reg clk=0;always #5 clk=~clk;reg rst=0,valid=0;reg[7:0]data;
 wire start,negative,late;reg[7:0]bytes[0:191];integer c,i,starts=0;
 velocity_stream dut(.life_busy(1'b0),.return_busy(1'b0),.return_safe(1'b1),.return_start(),.return_abort(),.clk(clk),.rst_n(rst),.tick(1'b0),.rx_data(data),.rx_valid(valid),.balance(1'b1),.permit(1'b1),.local_cancel(1'b0),.task_idle(1'b1),.start_quiet(1'b1),.flip_busy(1'b0),.flip_safe(1'b1),.flip_start(start),.flip_negative(negative),.flip_late(late),.flip_abort(),.enabled(),.velocity_q16(),.desired_q16(),.session(),.ack_seq(),.age_ms(),.flags(),.result());
 always @(posedge clk)if(start)starts=starts+1;
 initial begin
  $readmemh("tests/flip_variants.hex",bytes);
  for(c=0;c<4;c=c+1)begin
   rst=0;repeat(3)@(negedge clk);rst=1;
   for(i=c*48;i<(c+1)*48;i=i+1)begin data=bytes[i];valid=1;@(negedge clk);valid=0;@(negedge clk);end
   repeat(5)@(negedge clk);
   if(starts!=c+1 || negative!=(c>=2) || late!=(c%2))$fatal(1,"variant routing %d",c);
  end
  $display("PASS real host frames decode A/B and both directions, single start each");$finish;
 end
endmodule
