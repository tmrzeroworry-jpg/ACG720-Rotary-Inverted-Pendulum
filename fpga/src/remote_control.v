// UI01. Separate AAC4 channel: discrete high-level commands, never PWM.
// 24 bytes: AAC4 01 18 op 00 seq16 session32 arg32 0000 01F4 CRC16 CRLF.
// 0 claim, 1 heartbeat, 2 start(0 fast/1 original), 3 position(0..7),
// 4 spin(bit0 fast/bit1 negative), 5 sine(0..7), 6 hold, 7 stop, 8 release.
module remote_control #(
 parameter integer LEASE_MS=500,FRAME_TIMEOUT_CLKS=1500000
)(input wire clk,rst_n,tick,rx_valid,input wire[7:0]rx_data,
 input wire stop_n,sensor_ready,start_ready,keys_released,
 input wire[3:0]ctrl_state,input wire life_busy,flip_busy,stream_enabled,
 output reg owned,start_pulse,start_fast,task_pulse,stop_pulse,
 output reg[1:0] task_mode,output reg[7:0]task_cfg,
 output reg[31:0]session,output reg[15:0]ack_seq,age_ms,
 output reg[7:0]ack_op,result,output wire[7:0]flags);
 reg[191:0]packet;reg[4:0]index;reg[31:0]gap;reg[15:0]crc;reg check_packet,have_seq;
 wire[7:0]op=packet[159:152];wire[15:0]seq=packet[143:128];
 wire[31:0]sid=packet[127:96],arg=packet[95:64];wire[15:0]delta=seq-ack_seq;
 wire idle_actions=!life_busy&&!flip_busy&&!stream_enabled;
 assign flags={4'd0,stop_n,sensor_ready,(ctrl_state==4),owned};
 function[15:0]crc_next;input[15:0]old;input[7:0]b;reg[15:0]c;integer k;
 begin c=old^{b,8'd0};for(k=0;k<8;k=k+1)c=c[15]?(c<<1)^16'h1021:c<<1;crc_next=c;end endfunction
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   packet<=0;index<=0;gap<=0;crc<=16'hffff;check_packet<=0;have_seq<=0;
   owned<=0;start_pulse<=0;start_fast<=1;task_pulse<=0;stop_pulse<=0;
   task_mode<=0;task_cfg<=0;session<=0;ack_seq<=0;age_ms<=65535;ack_op<=0;result<=0;
  end else begin
   check_packet<=0;start_pulse<=0;task_pulse<=0;stop_pulse<=0;
   if(index!=0)begin
    if(gap>=FRAME_TIMEOUT_CLKS-1)begin index<=0;crc<=16'hffff;gap<=0;end else gap<=gap+1'b1;
   end
   if(rx_valid)begin
    gap<=0;
    if(index==0)begin
     if(rx_data==8'haa)begin packet<={184'd0,rx_data};index<=1;crc<=crc_next(16'hffff,rx_data);end
    end else if((index==1&&rx_data!=8'hc4)||(index==2&&rx_data!=1)||(index==3&&rx_data!=24))begin index<=0;crc<=16'hffff;end
    else begin
     packet<={packet[183:0],rx_data};if(index<20)crc<=crc_next(crc,rx_data);
     if(index==23)begin index<=0;check_packet<=1;end else index<=index+1'b1;
    end
   end
   if(tick)begin
    if(age_ms!=65535)age_ms<=age_ms+1'b1;
    if(owned&&age_ms>=LEASE_MS-1)begin owned<=0;have_seq<=0;stop_pulse<=1;result<=5;end
   end
   if(check_packet&&packet[191:160]==32'haac40118&&packet[151:144]==0&&
      packet[63:32]==32'h000001f4&&packet[31:16]==crc&&packet[15:0]==16'h0d0a&&sid!=0)begin
    // STOP is always available, including lost ownership or a different page.
    if(op==7&&arg==0)begin
     owned<=0;have_seq<=0;stop_pulse<=1;session<=sid;ack_seq<=seq;ack_op<=7;result<=0;age_ms<=0;
    end else if(op==0&&arg==0&&!owned&&stop_n&&sensor_ready&&keys_released&&
                (ctrl_state==1||ctrl_state==4)&&idle_actions)begin
     owned<=1;have_seq<=1;session<=sid;ack_seq<=seq;ack_op<=0;result<=0;age_ms<=0;
    end else if(owned&&have_seq&&sid==session&&delta!=0&&!delta[15])begin
     ack_seq<=seq;ack_op<=op;result<=0;
     if(op==1&&arg==0)age_ms<=0;
     else if(op==8&&arg==0)begin owned<=0;have_seq<=0;stop_pulse<=1;age_ms<=0;end
     else if(op==2&&arg<=1&&ctrl_state==1&&sensor_ready&&start_ready&&idle_actions)begin
      start_pulse<=1;start_fast<=arg==0;task_mode<=0;task_cfg<=0;age_ms<=0;
     end else if((op==3||op==4||op==5||op==6)&&ctrl_state==4&&sensor_ready&&keys_released&&idle_actions)begin
      if((op==3&&arg<=7)||(op==4&&arg<=3)||(op==5&&arg<=7)||(op==6&&arg==0))begin
       task_pulse<=1;age_ms<=0;
       case(op)
        3:begin task_mode<=1;task_cfg<={5'd0,arg[2:0]};end
        4:begin task_mode<=2;task_cfg<={3'd0,arg[1],arg[0],3'd0};end
        5:begin task_mode<=3;task_cfg<={1'd0,arg[2:0],4'd0};end
        default:begin task_mode<=1;task_cfg<=128;end
       endcase
      end else result<=2;
     end else result<=3;
    end
   end
   // Physical S0 overrides every accepted packet and cannot be cleared by UART.
   if(!stop_n)begin
    if(owned)stop_pulse<=1;
    owned<=0;have_seq<=0;start_pulse<=0;task_pulse<=0;result<=4;
   end
  end
 end
endmodule
