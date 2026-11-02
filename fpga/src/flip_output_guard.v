// Flip/CATCH transfer only; progressively restore original control bandwidth.
module flip_output_guard #(parameter integer MAIN_SLEW=2)(input wire clk,rst_n,tick,busy,permit,input wire[7:0]phase,
 input wire signed[9:0] request,output wire signed[9:0] command);
 reg active;reg signed[9:0] value;reg[7:0]tail,capture_ms;reg signed[31:0] step;
 assign command=active ? value:request;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin active<=0;value<=0;tail<=0;capture_ms<=0;end
  else if(!permit)begin active<=0;value<=0;tail<=0;capture_ms<=0;end
  else if(!active)begin value<=request;if(busy && phase<=3)begin active<=1;tail<=0;capture_ms<=0;end end
  else if(tick)begin
   if(phase<=2 && busy)begin step=MAIN_SLEW;capture_ms<=0;end
   else begin step=capture_ms<60?4+capture_ms:64;if(capture_ms<100)capture_ms<=capture_ms+1'b1;end
   if(request>$signed(value)+step)value<=value+step;
   else if(request<$signed(value)-step)value<=value-step;else value<=request;
   if(busy && phase<=3)tail<=0;else begin
    if(tail<200)tail<=tail+1'b1;
    if(tail>=100 && request-value<=16 && value-request<=16)active<=0;
   end
  end
 end
endmodule
