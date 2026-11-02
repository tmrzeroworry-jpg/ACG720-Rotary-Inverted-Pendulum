// WHEELTEC steady-control algorithm port; original author: 平衡小车之家/WHEELTEC.
// Source trace: provenance/vendor_utf8/{1,4}-control.c, not the STM32 binary.
// Native units: ADC codes, encoder counts, PWM /256. Q16 states,Q20 gains.
// PD5ms/position25ms; vendor LQR20ms. Held between scheduled updates.
// Motor saturation/slew and all hardware protection remain in the parent.
module wheeltec_balance #(parameter ONE_EURO=0)(
 input wire clk,rst_n,tick,launch,active,use_lqr,fast_lqr,
 input wire[1:0]euro_mode,
 input wire euro_a_quiet,
 input wire signed[63:0]euro_pole_q16,euro_arm_q16,
 input wire signed[17:0] pole_error,
 input wire signed[31:0] arm_error,
 input wire arm_soft,
 output reg busy,done,overflow,
 output reg signed[31:0] request,pole_p,pole_d,arm_p,arm_d,pole_rate,arm_rate
);
 localparam signed[31:0] PD_PP=32'sd231379,PD_PD=32'sd2313787;
 localparam signed[31:0] PD_AP=-32'sd219890,PD_AD=-32'sd7329681;
 // WEX8 分态增益：arm_soft=1（S2移动中 / S3、S4旋转中）用原厂软档；
 // arm_soft=0（静止 / S2到位后）用硬档。WEX10 起硬档 ×2.25（×2 实测良好但
 // 有残余静差；×4 已在 WEX5 否决——捕获瞬态拖垮摆环；×2.25 在二者间取保守值）。
 // 回退 WEX4：swing_controller 中把 .arm_soft 改接 1'b1 即可。
 localparam signed[31:0] PD_AP_X2=-32'sd494752,PD_AD_X2=-32'sd16491782;
 wire signed[31:0] PD_AP_S=arm_soft?PD_AP:PD_AP_X2;
 wire signed[31:0] PD_AD_S=arm_soft?PD_AD:PD_AD_X2;
 localparam signed[31:0] LQR_PP=32'sd142697,LQR_PD=32'sd630477;
 localparam signed[31:0] LQR_AP=-32'sd217192,LQR_AD=-32'sd11665988;
 localparam signed[31:0] LQR5_PD=32'sd2521908,LQR5_AD=-32'sd46663953;
 reg history_ready,update_due,lqr_r,fast_r,position_due;
 reg[4:0] sample_count;
 reg[2:0] position_count,phase;
 reg signed[17:0] h1,h2,h3;
 reg signed[49:0] mean_q,previous_mean,previous_mean2,previous_arm,position_history;
 reg signed[49:0] x_p,x_d,x_a,x_ad,position_delta,position_next;
 reg signed[75:0] position_product;
 reg signed[81:0] pp,pd,ap,ad;
 reg signed[63:0] rate_p,rate_a;
 reg signed[31:0] pp_i,ap_i,pole_i,arm_i,total_i;
 reg arithmetic_overflow;
 wire signed[19:0] error_sum=$signed({{2{pole_error[17]}},pole_error})+
       $signed({{2{h1[17]}},h1})+$signed({{2{h2[17]}},h2})+$signed({{2{h3[17]}},h3});
 wire signed[49:0] current_q=$signed({{32{pole_error[17]}},pole_error})<<<16;
 wire signed[49:0] arm_q=$signed({{18{arm_error[31]}},arm_error})<<<16;
 wire signed[83:0] pole_sum=$signed({{2{pp[81]}},pp})+$signed({{2{pd[81]}},pd});
 wire signed[83:0] arm_sum=$signed({{2{ap[81]}},ap})+$signed({{2{ad[81]}},ad});
 wire signed[83:0] all_sum=pole_sum+arm_sum;
 wire use_euro=ONE_EURO && euro_mode[0];
 wire arm_euro=ONE_EURO && euro_mode[1];
 wire signed[63:0]arm_filtered_q;wire arm_filter_busy;
 wire arm_sample=launch && !busy && update_due && !use_lqr && position_count==4;
 one_euro_filter #(.SAMPLE_HZ(40),.PERIOD(0),.MAX_STEP(1000),.FAST_MIN(2),.QUIET_MIN(1),
  .FAST_MAX(12),.QUIET_MAX(8),.FAST_BETA_Q24(503316),.QUIET_BETA_Q24(251658),.DERIV_ALPHA_Q20(335860)) position_euro(
  .clk(clk),.rst_n(rst_n),.enable(active && arm_euro && !use_lqr),.valid(1'b1),.sample(arm_sample),
  .variant(euro_a_quiet ? 2'd2:2'd1),.x(arm_error),.busy(arm_filter_busy),.done(),.ready(),.jump(),.overflow(),
  .filtered_q16(arm_filtered_q),.velocity(),.cutoff_hz());
 wire signed[49:0]pole_input_q=use_euro ? euro_pole_q16[49:0]:mean_q;
 wire signed[49:0]arm_input_q=(arm_euro && use_lqr) ? euro_arm_q16[49:0]:arm_q;
 function signed[83:0] integer_pwm;
  input signed[83:0] q;
  begin integer_pwm=q<0 ? -((-q)>>>36) :q>>>36;end
 endfunction
 function signed[31:0] clipped_pwm;
  input signed[83:0] q;
  reg signed[83:0] v;
  begin v=integer_pwm(q);clipped_pwm=v>84'sd2147483647 ?32'sh7fffffff :
       v < -84'sd2147483648 ?32'sh80000000 :v[31:0];end
 endfunction
 function outside_pwm;
  input signed[83:0] q;
  reg signed[83:0] v;
  begin v=integer_pwm(q);outside_pwm=v>84'sd2147483647 || v < -84'sd2147483648;end
 endfunction
 wire signed[31:0] pole_integer=clipped_pwm(pole_sum),arm_integer=clipped_pwm(arm_sum);
 wire signed[32:0] pd_integer=$signed({pole_i[31],pole_i})+
       (position_due ? $signed({arm_i[31],arm_i}) :
        ($signed({arm_p[31],arm_p})+$signed({arm_d[31],arm_d})));
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   busy<=0;done<=0;overflow<=0;history_ready<=0;update_due<=0;lqr_r<=0;fast_r<=0;position_due<=0;
   sample_count<=0;position_count<=0;phase<=0;h1<=0;h2<=0;h3<=0;
   mean_q<=0;previous_mean<=0;previous_mean2<=0;previous_arm<=0;position_history<=0;
   x_p<=0;x_d<=0;x_a<=0;x_ad<=0;position_delta<=0;position_next<=0;position_product<=0;
   pp<=0;pd<=0;ap<=0;ad<=0;rate_p<=0;rate_a<=0;pp_i<=0;ap_i<=0;pole_i<=0;arm_i<=0;total_i<=0;arithmetic_overflow<=0;
   request<=0;pole_p<=0;pole_d<=0;arm_p<=0;arm_d<=0;pole_rate<=0;arm_rate<=0;
  end else if(!active)begin
   busy<=0;done<=0;overflow<=0;history_ready<=0;update_due<=0;phase<=0;
   sample_count<=0;position_count<=0;position_history<=0;
   request<=0;pole_p<=0;pole_d<=0;arm_p<=0;arm_d<=0;pole_rate<=0;arm_rate<=0;
  end else begin
   done<=0;
   if(tick)begin
    h3<=h2;h2<=h1;h1<=pole_error;
    if(!history_ready)begin
     h1<=pole_error;h2<=pole_error;h3<=pole_error;mean_q<=current_q;
     previous_mean<=current_q;previous_mean2<=current_q;previous_arm<=arm_q;history_ready<=1;
    end else mean_q<=$signed({{30{error_sum[19]}},error_sum})<<<14;
    if(sample_count==(use_lqr && !fast_lqr?19:4))begin sample_count<=0;update_due<=1;end
    else sample_count<=sample_count+1'b1;
   end
   if(launch && !busy)begin
    if(!update_due)done<=1;
    else begin
     update_due<=0;busy<=1;phase<=1;lqr_r<=use_lqr;fast_r<=fast_lqr;
     x_p<=pole_input_q;x_d<=(pole_input_q-previous_mean2)>>>1;x_a<=arm_input_q;
     x_ad<=arm_input_q-previous_arm;previous_mean2<=previous_mean;previous_mean<=pole_input_q;previous_arm<=arm_input_q;
     position_due<=!use_lqr && position_count==4;
     if(!use_lqr)begin
      if(position_count==4)position_count<=0;else position_count<=position_count+1'b1;
     end
     position_delta<=arm_q-position_history;
    end
   end else if(busy)begin
    case(phase)
     // WEXC: pole speed window 5ms -> 10ms (x_d>>1 keeps same D gain), halves pole_D noise.
     // WEXB: 横杆位置反馈平滑 alpha 0.2 -> 0.35（3355443/2^24=0.2；tau 112ms -> 57ms），
     // 换回 0.75Hz 模态处约 14 度相位；量化噪声折算 <0.5 PWM，平滑收益不抵相位代价。
     1:begin position_product<=position_delta*26'sd5872026;phase<=2;end
     2:if(!arm_filter_busy)begin position_next<=arm_euro&&!lqr_r ? arm_filtered_q[49:0]:position_history+(position_product>>>24);phase<=3;end
     3:begin
      pp<=x_p*$signed(lqr_r?LQR_PP:PD_PP);
      pd<=x_d*$signed(lqr_r?(fast_r?LQR5_PD:LQR_PD):PD_PD);
      ap<=(lqr_r?x_a:position_next)*$signed(lqr_r?LQR_AP:PD_AP_S);
      ad<=(lqr_r?x_ad:(position_next-position_history))*$signed(lqr_r?(fast_r?LQR5_AD:LQR_AD):PD_AD_S);
      rate_p<=x_d*$signed(lqr_r && !fast_r?32'sd100:32'sd400);
      rate_a<=!lqr_r && arm_euro ? (position_next-position_history)*64'sd40:x_ad*$signed(lqr_r && !fast_r?32'sd50:32'sd200);
      phase<=4;
     end
     4:begin
      pp_i<=clipped_pwm({{2{pp[81]}},pp});ap_i<=clipped_pwm({{2{ap[81]}},ap});
      pole_i<=pole_integer;arm_i<=arm_integer;total_i<=clipped_pwm(all_sum);
      arithmetic_overflow<=outside_pwm(pole_sum)||(lqr_r?outside_pwm(all_sum):(position_due && outside_pwm(arm_sum)));
      phase<=5;
     end
     5:begin
      pole_p<=pp_i;
      pole_d<=pole_i-pp_i;
      pole_rate<=rate_p>>>16;
      if(lqr_r || !arm_euro || position_due)arm_rate<=rate_a>>>16;
      if(lqr_r)begin
       request<=total_i;arm_p<=ap_i;
       arm_d<=total_i-pole_i-ap_i;
       overflow<=arithmetic_overflow;
      end else begin
       request<=pd_integer>33'sd2147483647 ?32'sh7fffffff :
                   pd_integer < -33'sd2147483648 ?32'sh80000000 :pd_integer[31:0];
       if(position_due)begin
        position_history<=position_next;arm_p<=ap_i;
        arm_d<=arm_i-ap_i;
       end
       overflow<=arithmetic_overflow||
                 pd_integer>33'sd2147483647 || pd_integer < -33'sd2147483648;
      end
      done<=1;busy<=0;phase<=0;
     end
     default:begin busy<=0;done<=1;overflow<=1;phase<=0;end
    endcase
   end
  end
 end
endmodule
