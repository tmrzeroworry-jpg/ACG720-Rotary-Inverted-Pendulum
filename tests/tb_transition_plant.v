// Reduced identified-fit exploration; return feedback is real RTL.
// Fast startup verifies arrival request only (actual upright CATCH remains original).
`timescale 1ns/1ps
module tb_transition_plant;
 reg clk=0;always #5 clk=~clk;reg rst=0,tick=0,fast_start=0,return_start=0,raw_valid=0;reg[15:0]angle=20000;reg[3:0]state=1;
 wire busy,rb,fresh,waiting,bridge,takeover,capture,parked,abort,dvalid,mode;
 wire[7:0]phase,result,turns,sfault;wire[15:0]ident,elapsed,down,seam_age,seen_angle;
 wire signed[9:0]command,applied;reg signed[9:0]core_command=0;
 always @(posedge clk)if(tick)core_command<=state==9 ? command:10'sd0;wire signed[31:0]rate;
 real beta=3.141592653589793,w=0,arm=0,v=0,a,ba,pu,pi=3.141592653589793,g=95.8,b=0.316,damping=2,fr=9,centr=0.198,grav=13.76,kappa=1.39;
 integer fd;integer model=0,action=0,side=1,n,j,tmp,code,arrived=0,finished=0;real phase_angle,rawphase,peak_rate=0;
 transition_angle_guard sensor(.clk(clk),.rst_n(rst),.tick(tick),.start(fast_start||return_start),.active(busy),.raw_valid(raw_valid),.online(1'b1),.raw(angle),.seed(angle),.angle(seen_angle),.fresh(fresh),.waiting(waiting),.bridge(bridge),.fault(sfault),.seam_age(seam_age),.diagnostic());
 transition_control dut(.clk(clk),.rst_n(rst),.tick(tick),.fast_start(fast_start),.return_start(return_start),.cancel(1'b0),.fresh(busy?fresh:1'b1),.seam_wait(waiting),.seam_bridge(bridge),.sensor_fault(sfault),.ctrl_state(state),.angle(seen_angle),.up(16'd6251),.default_down(16'd20160),.arm_position($rtoi(arm*1040/(2*pi))),.arm_velocity($rtoi(v*1040/(2*pi))),.seed_command(10'sd0),.busy(busy),.return_busy(rb),.takeover(takeover),.capture(capture),.parked(parked),.abort(abort),.phase(phase),.result(result),.turns(turns),.mode(mode),.ident(ident),.elapsed(elapsed),.down_ref(down),.down_valid(dvalid),.command(command),.rate_diag(rate),.goal_diag(),.energy_diag());
 flip_output_guard #(.MAIN_SLEW(3)) guard(.clk(clk),.rst_n(rst),.tick(tick),.busy(busy),.phase(phase==3 || phase==4 ? 8'd3:8'd2),.permit(1'b1),.request(core_command),.command(applied));
 always @(posedge clk)begin if(takeover)state<=9;if(capture)arrived=1;if(parked)begin state<=1;finished=1;end end
 function real ss;input real z;real ex;begin if(z>10)ss=1;else if(z< -10)ss=-1;else begin ex=$exp(2*z);ss=(ex-1)/(ex+1);end end endfunction
 task sample;begin raw_valid=1;@(negedge clk);raw_valid=0;repeat(3)@(negedge clk);tick=1;@(negedge clk);tick=0;repeat(20)@(negedge clk);end endtask
 initial begin
  tmp=$value$plusargs("MODEL=%d",model);tmp=$value$plusargs("ACTION=%d",action);tmp=$value$plusargs("SIDE=%d",side);
  if(model==1)begin g=98.2;b=0.248;damping=1.226;fr=3.18;centr=0.208;grav=9.49;kappa=1.387;end
  fd=$fopen($sformatf("tests/trans_trace_%0d_%0d_%0d.csv",model,action,side),"w");$fwrite(fd,"time,beta,w,arm,v,u,phase,rate,goal,down_error,cos\n");
  repeat(4)@(negedge clk);rst=1;repeat(4)@(negedge clk);
  for(n=0;n<1100;n=n+1)sample();if(!dvalid || down!=20000)$fatal(1,"rest reference not learned");
  if(action==1)begin beta=side*0.01;angle=6251+side*40;state=4;repeat(4)@(negedge clk);return_start=1;end else fast_start=1;
  @(negedge clk);fast_start=0;return_start=0;repeat(4)@(negedge clk);
  for(n=0;n<8000 && phase!=11 && !arrived && !finished;n=n+1)begin
   phase_angle=beta;while(phase_angle<0)phase_angle=phase_angle+2*pi;while(phase_angle>=2*pi)phase_angle=phase_angle-2*pi;
   if(phase_angle<=pi)rawphase=6251+phase_angle*13749/pi;else rawphase=20000+(phase_angle-pi)*12651/pi;
   code=$rtoi(rawphase)%26400;angle=code;
   if(n%4!=0 && (code<100 || code>26300))angle=16'hff38;
   sample();pu=$signed(applied);
   for(j=0;j<10;j=j+1)begin
    a=-b*pu-damping*v-fr*ss(v/0.2)-centr*$sin(beta)*w*w+grav*$sin(2*beta);
    ba=g*$sin(beta)+kappa*a*$cos(beta)-0.15*w-0.586*ss(w/0.1);
    v=v+a*0.0001;arm=arm+v*0.0001;w=w+ba*0.0001;beta=beta+w*0.0001;
   end
   if(n%10==0)$fwrite(fd,"%f,%f,%f,%f,%f,%d,%d,%d,%d,%d,%d\n",n*0.001,beta,w,arm,v,$signed(applied),phase,$signed(rate),$signed(dut.goal),$signed(dut.dd),$signed(dut.cos_up));
   if(w>peak_rate)peak_rate=w;
  end
  if((action==0 && !arrived)||(action==1 && !finished))$fatal(1,"failed model%d action%d side%d phase%d reason%d beta%f rate%f arm%f turns%d",model,action,side,phase,result,beta*180/pi,w,arm/(2*pi),turns);
  $display("PASS RTL model%d action%d side%d t%f turns%d beta%f rate%f armv%f peak%f",model,action,side,n*0.001,turns,beta*180/pi,w,v,peak_rate);$finish;
 end
endmodule
