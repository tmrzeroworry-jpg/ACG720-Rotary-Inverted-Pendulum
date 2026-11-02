`timescale 1ns/1ps
module tb_transition_unit;
 reg clk=0;always #5 clk=~clk;reg rst=0,tick=0,fs=0,rs=0,cancel=0,fresh=1;reg[15:0]angle=20000;reg[3:0]state=1;
 wire busy,rb,to,cap,park,abort,valid,mode;wire[7:0]phase,result,turns;wire[15:0]id,ms,down;wire signed[9:0]u;
 transition_control dut(.clk(clk),.rst_n(rst),.tick(tick),.fast_start(fs),.return_start(rs),.cancel(cancel),.fresh(fresh),.seam_wait(1'b0),.seam_bridge(1'b0),.sensor_fault(8'd0),.ctrl_state(state),.angle(angle),.up(16'd6251),.default_down(16'd20160),.arm_position(32'sd0),.arm_velocity(32'sd0),.seed_command(10'sd0),.busy(busy),.return_busy(rb),.takeover(to),.capture(cap),.parked(park),.abort(abort),.phase(phase),.result(result),.turns(turns),.mode(mode),.ident(id),.elapsed(ms),.down_ref(down),.down_valid(valid),.command(u),.rate_diag(),.goal_diag(),.energy_diag());
 integer n,seen=0,last=0;reg[15:0]old;
 always @(posedge clk)if(cap)seen=1;
 task ms_tick;begin @(negedge clk);tick=1;@(negedge clk);tick=0;repeat(20)@(negedge clk);end endtask
 task reset;begin rst=0;tick=0;fs=0;rs=0;cancel=0;fresh=1;state=1;angle=20000;repeat(4)@(negedge clk);rst=1;repeat(4)@(negedge clk);end endtask
 task learn;begin for(n=0;n<1030;n=n+1)ms_tick();if(!valid || down!=20000 || busy)$fatal(1,"rest learning/no-autostart");end endtask
 initial begin
  reset();fs=1;@(negedge clk);fs=0;if(result!=1 || busy)$fatal(1,"unlearned startup accepted");
  reset();learn();fs=1;@(negedge clk);fs=0;state=9;old=id;
  fs=1;@(negedge clk);fs=0;if(id!=old)$fatal(1,"busy restart queued");
  for(n=0;n<140;n=n+1)begin ms_tick();if($signed(u)>150 || $signed(u)< -150)$fatal(1,"up cap");end
  for(n=0;n<135;n=n+1)begin angle=20000-n*100;ms_tick();end
  angle=6251;for(n=0;n<40;n=n+1)ms_tick();if(!seen)$fatal(1,"no capture request");
  state=3;for(n=0;n<200;n=n+1)ms_tick();state=4;for(n=0;n<510;n=n+1)ms_tick();if(phase!=9)$fatal(1,"no up verification");
  rs=1;@(negedge clk);rs=0;for(n=0;n<105;n=n+1)ms_tick();state=9;
  for(n=0;n<140;n=n+1)ms_tick();
  for(n=0;n<139;n=n+1)begin angle=6251+n*100;ms_tick();end
  angle=20000;for(n=0;n<450;n=n+1)ms_tick();
  if(phase!=8)$fatal(1,"release before quiet verification %d",phase);
  angle=20220;ms_tick();if(phase!=7)$fatal(1,"did not resume damping after release perturbation");
  angle=20000;for(n=0;n<660;n=n+1)ms_tick();if(phase!=10 || busy || u!=0)$fatal(1,"parked without zero/settle");
  state=4;angle=6251;repeat(4)@(negedge clk);rs=1;@(negedge clk);rs=0;fresh=0;ms_tick();if(phase!=11 || result!=3)$fatal(1,"sensor exit");
  reset();learn();fs=1;@(negedge clk);fs=0;state=9;cancel=1;@(negedge clk);cancel=0;if(result!=11)$fatal(1,"cancel");
  $display("PASS reference learning, no autostart, gated/busy starts, up handoff, bottom settle/release/re-entry, caps, sensor, cancel");$finish;
 end
endmodule
