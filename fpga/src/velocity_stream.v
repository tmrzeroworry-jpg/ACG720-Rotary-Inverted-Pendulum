// TR01: reference velocity only. Never accepts PWM or starts swing-up.
// Frame (24B, big endian): AA C3 01 18 op 00 seq16 session32 vQ16s32 aStep16 ttl16 CRC16 CR LF.
// op: 0 HELLO (idle only), 1 START, 2 SET, 3 SOFT_STOP. CRC-CCITT over bytes0..19.
// v unit=Q16 encoder counts/ms; acceleration step is v increment per1ms.
module velocity_stream #(
 parameter integer MAX_V_Q16=17039, MAX_A_STEP=17, MAX_TTL_MS=300,
 parameter integer FRAME_TIMEOUT_CLKS=1500000
)(input wire clk,rst_n,tick, input wire [7:0] rx_data,input wire rx_valid,
 input wire balance,permit,local_cancel,task_idle,start_quiet,
 input wire life_busy,return_busy,return_safe,output reg return_start,return_abort,
 input wire flip_busy,flip_safe,output reg flip_start,flip_negative,flip_late,flip_abort,
 output reg enabled,output reg signed[31:0] velocity_q16,desired_q16,
 output reg [31:0] session,output reg[15:0] ack_seq,age_ms,
 output wire[7:0] flags,output reg[7:0] result);
 reg [191:0] packet;reg[4:0] index;reg[15:0] crc,ttl,acc_step;
 reg [31:0] gap;reg leased,have_seq,timed_out,rejected,check_packet;
 reg signed[31:0] goal;
 wire[7:0] op=packet[159:152];
 wire[15:0] pseq=packet[143:128];
 wire[31:0] psession=packet[127:96];
 wire signed[31:0] pv=packet[95:64];
 wire[15:0] pa=packet[63:48],pttl=packet[47:32];
 wire[15:0] seq_delta=pseq-ack_seq;
 assign flags={1'b0,flip_busy,leased,!permit,rejected,timed_out,enabled,1'b1};
 function[15:0] crc_next;
  input[15:0] old;input[7:0] b;reg[15:0] c;integer k;
  begin c=old^{b,8'd0};for(k=0;k<8;k=k+1)c=c[15]?(c<<1)^16'h1021:c<<1;crc_next=c;end
 endfunction
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   packet<=0;index<=0;crc<=16'hffff;gap<=0;check_packet<=0;return_start<=0;return_abort<=0;
   enabled<=0;velocity_q16<=0;desired_q16<=0;session<=0;ack_seq<=0;age_ms<=16'hffff;
   flip_start<=0;flip_negative<=0;flip_late<=0;flip_abort<=0;ttl<=MAX_TTL_MS;acc_step<=MAX_A_STEP;leased<=0;have_seq<=0;timed_out<=0;rejected<=0;result<=0;
  end else begin
   check_packet<=0;flip_start<=0;flip_abort<=0;return_start<=0;return_abort<=0;
   if(index!=0)begin
    if(gap>=FRAME_TIMEOUT_CLKS-1)begin index<=0;crc<=16'hffff;gap<=0;end else gap<=gap+1;
   end
   if(rx_valid)begin
    gap<=0;
    if(index==0)begin
     if(rx_data==8'haa)begin packet<={184'd0,rx_data};index<=1;crc<=crc_next(16'hffff,rx_data);end
    end else if((index==1 && rx_data!=8'hc3)||(index==2 && rx_data!=1)||(index==3 && rx_data!=24))begin
     index<=0;crc<=16'hffff;
    end else begin
     packet<={packet[183:0],rx_data};
     if(index<20)crc<=crc_next(crc,rx_data);
     if(index==23)begin index<=0;check_packet<=1;end else index<=index+1'b1;
    end
   end
   if(tick && flip_busy && age_ms>=MAX_TTL_MS-1)flip_abort<=1;
   if(tick && return_busy && age_ms>=MAX_TTL_MS-1)return_abort<=1;
   if(tick)begin
    if(age_ms!=16'hffff)age_ms<=age_ms+1'b1;
    if(enabled)begin
     goal=desired_q16;
     if(age_ms>=ttl-1)begin goal=0;desired_q16<=0;timed_out<=1;result<=3;end
     if(velocity_q16<goal)begin
      if(goal-velocity_q16>$signed({16'd0,acc_step}))velocity_q16<=velocity_q16+$signed({16'd0,acc_step});
      else velocity_q16<=goal;
     end else if(velocity_q16>goal)begin
      if(velocity_q16-goal>$signed({16'd0,acc_step}))velocity_q16<=velocity_q16-$signed({16'd0,acc_step});
      else velocity_q16<=goal;
     end
    end
   end
   if(check_packet && packet[191:160]==32'haac30118 && packet[151:144]==0 &&
      packet[31:16]==crc && packet[15:0]==16'h0d0a)begin
    if(op==0 && !life_busy && !flip_busy && (!enabled || velocity_q16==0) && psession!=0 && permit)begin
     desired_q16<=0;session<=psession;ack_seq<=pseq;have_seq<=1;leased<=1;result<=0;rejected<=0;timed_out<=0;
    end else if(psession==session && leased && have_seq && seq_delta!=0 && !seq_delta[15])begin
     ack_seq<=pseq;rejected<=0;
     if(op==4 && !life_busy && !flip_busy && balance && permit && start_quiet &&
        ((!enabled && task_idle)||(enabled && velocity_q16==0)) && (pv==1 || pv==-1 || pv==2 || pv==-2))begin
      flip_start<=1;flip_negative<=pv<0;flip_late<=(pv==2 || pv==-2);enabled<=0;velocity_q16<=0;desired_q16<=0;age_ms<=0;result<=0;
     end else if(op==7 && !life_busy && !flip_busy && balance && permit)begin
      return_start<=1;enabled<=0;velocity_q16<=0;desired_q16<=0;age_ms<=0;result<=0;
     end else if(op==8 && return_busy && return_safe)begin age_ms<=0;result<=0;end
     else if(op==9 && return_busy)begin return_abort<=1;age_ms<=0;result<=0;end
     else if(op==5 && flip_busy && flip_safe)begin age_ms<=0;result<=0;end
     else if(op==6 && flip_busy)begin flip_abort<=1;age_ms<=0;result<=0;end
     else if(op==3)begin desired_q16<=0;age_ms<=0;result<=0;end
     else if((op==1 || op==2) && balance && permit && !local_cancel &&
        !life_busy && ((op==1 && ((!enabled && task_idle) || (enabled && velocity_q16==0)) && start_quiet) || (op==2 && enabled && !timed_out)))begin
      if(pv>MAX_V_Q16 || pv< -MAX_V_Q16 || pa==0 || pa>MAX_A_STEP || pttl<100 || pttl>MAX_TTL_MS)begin
       rejected<=1;result<=2; // range error, no watchdog refresh
      end else begin
       desired_q16<=pv;acc_step<=pa;ttl<=pttl;age_ms<=0;
       if(op==1)begin enabled<=1;velocity_q16<=0;timed_out<=0;end
       result<=0;
      end
     end else begin rejected<=1;result<=1;end
    end
   end
   // Never resume after S0, loss of BAL, physical task takeover, or angle gate.
   if(!balance || !permit || local_cancel)begin
    enabled<=0;velocity_q16<=0;desired_q16<=0;if((!flip_busy && !flip_start && !return_busy && !return_start) || (flip_busy && !flip_safe) || (return_busy && !return_safe) || local_cancel)begin leased<=0;have_seq<=0;end
    if(local_cancel)result<=4;else if(!permit)result<=5;
   end
  end
 end
endmodule
