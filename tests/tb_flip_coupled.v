// Exploratory50Hz-fit model; verifies arrival requests, not physical capture.
`timescale 1ns/1ps
module tb_flip_coupled;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,tick=0,start=0,neg=0,late=0,raw_valid=0;reg[15:0]angle=6251;
 wire busy,takeover,capture,abort;wire[7:0]phase,result;wire[15:0]ident,elapsed;wire signed[31:0]progress;wire signed[9:0]u;
 wire[15:0]seen_angle,seam_age;wire fresh,wait_seam,bridge;wire[7:0]sensor_fault;wire signed[9:0]applied;
 real beta=0,w=0,arm=0,v=0,a,ba,pu,pi=3.141592653589793,g=95.8,b=0.316,damping=2,fr=9,centr=0.198,grav=13.76,kappa=1.39;
 flip_angle_tracker sensor(.clk(clk),.rst_n(rst),.tick(tick),.start(start),.active(busy),.negative(neg),.raw_valid(raw_valid),.online(1'b1),.raw(angle),.seed(16'd6251),.angle(seen_angle),.fresh(fresh),.waiting(wait_seam),.bridge(bridge),.fault(sensor_fault),.seam_age(seam_age),.diagnostic());
 flip_maneuver dut(.clk(clk),.rst_n(rst),.tick(tick),.start(start),.cancel(1'b0),.negative(neg),.late(late),.fresh(busy ? fresh:1'b1),
 .seam_wait(wait_seam),.seam_bridge(bridge),.sensor_fault(sensor_fault),.angle(seen_angle),.up(16'd6251),.ctrl_state(start ? 4'd4:4'd8),
 .arm_position($rtoi(arm*1040/(2*pi))),.arm_velocity($rtoi(v*1040/(2*pi))),.seed_command(10'sd0),
 .busy(busy),.takeover(takeover),.capture(capture),.abort(abort),.phase(phase),.result(result),.ident(ident),.elapsed(elapsed),.progress(progress),.command(u),.rate_diag(),.energy_diag(),.goal_diag());
 flip_output_guard guard(.clk(clk),.rst_n(rst),.tick(tick),.busy(busy),.phase(phase),.permit(1'b1),.request(u),.command(applied));
 integer n,j,code,variant=0,direction=1,model=0,tmp,seen=0;
 function real smoothsign;input real z;real ex;begin if(z>10)smoothsign=1;else if(z< -10)smoothsign=-1;else begin ex=$exp(2*z);smoothsign=(ex-1)/(ex+1);end end endfunction
 always @(posedge clk)if(capture)seen=1;
 initial begin
  tmp=$value$plusargs("DIR=%d",direction);tmp=$value$plusargs("VAR=%d",variant);tmp=$value$plusargs("MODEL=%d",model);neg=direction<0;late=variant;
  if(model==1)begin g=98.2;b=0.248;damping=1.226;fr=3.18;centr=0.208;grav=9.49;kappa=1.387;end
  repeat(4)@(negedge clk);rst=1;repeat(4)@(negedge clk);start=1;@(negedge clk);start=0;repeat(4)@(negedge clk);
  for(n=0;n<1900 && phase!=6 && !seen;n=n+1)begin
   code=6251+$rtoi(beta*26400/(2*pi));code=code%26400;if(code<0)code=code+26400;angle=code;
   if(n%4!=0 && (code<160 || code>26240))angle=16'hff38;
   if(n%13==0 && wait_seam)angle=15000;
   raw_valid=1;@(negedge clk);raw_valid=0;repeat(3)@(negedge clk);
   tick=1;@(negedge clk);tick=0;repeat(20)@(negedge clk);
   pu=$signed(applied);
   for(j=0;j<10;j=j+1)begin
    a=-b*pu-damping*v-fr*smoothsign(v/0.2)-centr*$sin(beta)*w*w+grav*$sin(2*beta);
    ba=g*$sin(beta)+kappa*a*$cos(beta)-0.15*w-0.586*smoothsign(w/0.1);
    v=v+a*0.0001;arm=arm+v*0.0001;w=w+ba*0.0001;beta=beta+w*0.0001;
   end
  end
  if(!seen)$fatal(1,"no arrival: model%d var%d dir%d phase%d reason%d progress%d beta%f",model,variant,direction,phase,result,progress,beta*180/pi);
  if(progress<26400)$fatal(1,"premature capture");
  $display("PASS coupled arrival only model%d variant%d dir%d time%f pole_velocity%f arm_turn%f",model,variant,direction,n*0.001,w,arm/(2*pi));$finish;
 end
endmodule
