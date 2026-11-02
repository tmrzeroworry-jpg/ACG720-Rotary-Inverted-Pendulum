// TR06 bounded bidirectional seam observations; three real exit samples, no predicted angle.
module transition_angle_guard #(
 parameter integer FULL=26400,EDGE=600,EXIT=400,EXIT_MAX=4000,SEAM_MS=160,REVERSE_DISTANCE=64
)(input wire clk,rst_n,tick,start,active,raw_valid,online,
 input wire[15:0] raw,seed,
 output reg[15:0] angle,output wire fresh,output reg waiting,bridge,
 output reg[7:0] fault,output reg[15:0] seam_age,
 output wire[7:0] diagnostic,output wire[15:0]raw_age_o,good_age_o);
 reg[15:0]latest,raw_age,good_age,last,edge_extreme;
 reg pending,used,negative,vote_retreat;reg[2:0]votes;
 reg signed[31:0]value,delta;reg legal,near_edge,far_edge,retreat_edge,reenter,retreating;
 assign fresh=online && raw_age<20 && fault==0 && (waiting || good_age<20);
 assign diagnostic={2'b0,vote_retreat,negative,used,waiting,votes[1:0]};
 assign raw_age_o=raw_age;assign good_age_o=good_age;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   latest<=0;pending<=0;raw_age<=65535;good_age<=65535;last<=0;angle<=0;
   negative<=0;waiting<=0;bridge<=0;used<=0;votes<=0;vote_retreat<=0;
   fault<=0;seam_age<=0;edge_extreme<=0;
  end else begin
   if(tick)begin
    bridge<=0;
    if(raw_age<65535)raw_age<=raw_age+1'b1;
    if(good_age<65535)good_age<=good_age+1'b1;
    if(!active)begin waiting<=0;used<=0;votes<=0;fault<=0;seam_age<=0;end
    else if(waiting)begin seam_age<=seam_age+1'b1;if(seam_age>=SEAM_MS-1)fault<=1;end
    if(pending)begin
     pending<=0;value=$signed({latest[15],latest});
     legal=(value>=-64 && value<=FULL+512);
     if(value<0)value=0;else if(value>FULL)value=FULL;
     delta=value-$signed({1'b0,last});
     if(delta>FULL/2)delta=delta-FULL;else if(delta< -FULL/2)delta=delta+FULL;
     near_edge=(value<EDGE && delta<0) || (value>FULL-EDGE && delta>0);
     far_edge=negative ? value>FULL-EXIT_MAX && value<FULL-EXIT : value>EXIT && value<EXIT_MAX;
     retreat_edge=negative ? value>EDGE+EXIT && value<EXIT_MAX : value<FULL-EDGE-EXIT && value>FULL-EXIT_MAX;
     reenter=used && ((negative && value>FULL-EDGE && value>$signed({1'b0,edge_extreme})+REVERSE_DISTANCE) ||
                     (!negative && value<EDGE && value+REVERSE_DISTANCE<$signed({1'b0,edge_extreme})));
     if(active && fault==0)begin
      if(waiting)begin
       if(legal && (far_edge || retreat_edge))begin
        retreating=retreat_edge;
        if(votes>=2 && vote_retreat==retreating)begin
         delta=value-$signed({1'b0,last});
         if(delta>FULL/2)delta=delta-FULL;else if(delta< -FULL/2)delta=delta+FULL;
         if(negative ^ retreating)delta=-delta;
         if(delta>=0 && delta<EXIT_MAX+EDGE+512)begin
          last<=value;angle<=value==FULL ? 0:value;waiting<=0;used<=1;edge_extreme<=value;
          bridge<=1;good_age<=0;votes<=0;
         end else fault<=2;
        end else begin
         votes<=votes!=0 && vote_retreat==retreating ? votes+1'b1:3'd1;
         vote_retreat<=retreating;
        end
       end else votes<=0;
      end else if(legal && near_edge && (!used || reenter))begin
       negative<=value<EDGE;last<=value;angle<=value==FULL ? 0:value;good_age<=0;
       waiting<=1;seam_age<=0;votes<=0;vote_retreat<=0;
      end else if(legal)begin
       if(value>1200 && value<FULL-1200)used<=0;
       if(used)begin
        if(negative && value<edge_extreme)edge_extreme<=value;
        if(!negative && value>edge_extreme)edge_extreme<=value;
       end
       if(delta>500 || delta< -500)fault<=2;
       else begin last<=value;angle<=value==FULL ? 0:value;good_age<=0;end
      end else fault<=3;
     end
    end
   end
   if(raw_valid)begin latest<=raw;pending<=1;raw_age<=0;end
   if(!active)begin angle<=seed;last<=seed;end
   if(start)begin
    angle<=seed;last<=seed;good_age<=0;waiting<=0;bridge<=0;used<=0;
    votes<=0;vote_retreat<=0;fault<=0;seam_age<=0;edge_extreme<=seed;
   end
  end
 end
endmodule
