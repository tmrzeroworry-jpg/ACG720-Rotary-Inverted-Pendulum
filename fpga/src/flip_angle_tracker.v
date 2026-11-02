// TR03: endpoint-aware observations for the new flip only. Baseline ADC guard untouched.
// Hold measured angle across the known potentiometer seam; resume only after
// three real samples on the expected opposite edge. No synthetic progress.
module flip_angle_tracker #(
 parameter integer FULL=26400,EDGE=600,EXIT=400,EXIT_MAX=4000,SEAM_MS=80
)(input wire clk,rst_n,tick,start,active,negative,raw_valid,online,
 input wire[15:0] raw,seed,
 output reg[15:0] angle,output wire fresh,output reg waiting,bridge,
 output reg[7:0] fault,output reg[15:0] seam_age,
 output wire[7:0] diagnostic);
 reg[15:0] latest,raw_age,good_age,last;reg pending,used;reg[2:0]votes;
 reg signed[31:0] value,delta;reg legal,far_edge,near_edge;
 assign fresh=online && raw_age<20 && fault==0 && (waiting || good_age<20);
 assign diagnostic={4'd0,used,waiting,votes[1:0]};
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin latest<=0;pending<=0;raw_age<=65535;good_age<=65535;
   last<=0;angle<=0;waiting<=0;bridge<=0;used<=0;votes<=0;fault<=0;seam_age<=0;end
  else begin
   if(tick)begin
    bridge<=0;
    if(raw_age<65535)raw_age<=raw_age+1'b1;
    if(good_age<65535)good_age<=good_age+1'b1;
    if(!active)begin waiting<=0;used<=0;votes<=0;fault<=0;seam_age<=0;end
    else if(waiting)begin
     seam_age<=seam_age+1'b1;
     if(seam_age>=SEAM_MS-1)fault<=1;
    end
    if(pending)begin
     pending<=0;
     value=$signed({latest[15],latest});
     legal=(value>=-64 && value<=FULL+512);
     if(value<0)value=0;else if(value>FULL)value=FULL;
     near_edge=negative ? value<EDGE : value>FULL-EDGE;
     far_edge=negative ? value>FULL-EXIT_MAX && value<FULL-EXIT : value>EXIT && value<EXIT_MAX;
     if(active && fault==0)begin
      if(waiting)begin
       // Ignore seam spikes rather than converting them into a rate impulse.
       if(legal && far_edge)begin
        if(votes>=2)begin
         delta=value-$signed({1'b0,last});
         if(delta>FULL/2)delta=delta-FULL;else if(delta< -FULL/2)delta=delta+FULL;
         if(negative)delta=-delta;
         if(delta>=0 && delta<EXIT_MAX+EDGE+512)begin
          last<=value;angle<=value==FULL ? 0:value;waiting<=0;used<=1;
          bridge<=1;good_age<=0;votes<=0;
         end else fault<=2;
        end else votes<=votes+1'b1;
       end else votes<=0;
      end else if(legal && !used && near_edge)begin
       // Enter before a negative offset or an endpoint wrap can invalidate ADC freshness.
       last<=value;angle<=value==FULL ? 0:value;good_age<=0;
       waiting<=1;seam_age<=0;votes<=0;
      end else if(legal)begin
       delta=value-$signed({1'b0,last});
       if(delta>FULL/2)delta=delta-FULL;else if(delta< -FULL/2)delta=delta+FULL;
       if(delta>500 || delta< -500)fault<=2;
       else begin last<=value;angle<=value==FULL ? 0:value;good_age<=0;end
      end else fault<=3;
     end
    end
   end
   // A genuine I2C conversion refreshes transport even if analog seam code is invalid.
   if(raw_valid)begin latest<=raw;pending<=1;raw_age<=0;end
   if(!active)begin angle<=seed;last<=seed;end
   if(start)begin angle<=seed;last<=seed;good_age<=0;waiting<=0;bridge<=0;used<=0;votes<=0;fault<=0;seam_age<=0;end
  end
 end
endmodule
