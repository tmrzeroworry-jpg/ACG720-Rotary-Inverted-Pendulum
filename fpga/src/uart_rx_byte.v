// 115200 8N1, two-stage synchronizer, midpoint samples, stop-bit check.
module uart_rx_byte #(parameter integer CLK_FREQ_HZ=50000000, BAUD_RATE=115200)(
 input wire clk,rst_n,uart_rx, output reg [7:0] data, output reg valid, output reg framing_error);
 localparam integer DIV=CLK_FREQ_HZ/BAUD_RATE;
 reg [1:0] sync; reg [1:0] state; reg [31:0] count; reg [2:0] bit_no; reg [7:0] shift;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin sync<=3;state<=0;count<=0;bit_no<=0;shift<=0;data<=0;valid<=0;framing_error<=0;end
  else begin
   sync<={sync[0],uart_rx};valid<=0;framing_error<=0;
   case(state)
    0:if(!sync[1])begin state<=1;count<=DIV/2-1;end
    1:if(count!=0)count<=count-1;else if(sync[1])state<=0;else begin state<=2;count<=DIV-1;bit_no<=0;end
    2:if(count!=0)count<=count-1;else begin
       shift[bit_no]<=sync[1];count<=DIV-1;
       if(bit_no==7)state<=3;else bit_no<=bit_no+1'b1;
      end
    3:if(count!=0)count<=count-1;else begin
       state<=0;if(sync[1])begin data<=shift;valid<=1;end else framing_error<=1;
      end
   endcase
  end
 end
endmodule
