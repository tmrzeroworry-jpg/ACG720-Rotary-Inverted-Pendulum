// LQ04 finite-depth notch after raw four-state feedback, before compensation.
// 1kHz launches; Q26 coefficients, Q16 history; one shared48x32 multiplier.
// Coefficients generated in config/experiment_LQ04.json, exact fixed-point DC1.
module lqr_notch(
 input wire clk,rst_n,launch,cancel,active,
 input wire [1:0] profile_id,
 input wire signed [31:0] linear,
 output reg busy,done,overflow,
 output reg signed [31:0] shaped
);
 reg [3:0] phase;
 reg primed;
 reg [1:0] last_profile;
 reg signed [47:0] x0,x1,x2,y1,y2,operand;
 reg signed [31:0] b0,b1,b2,a1,a2,coefficient;
 reg signed [79:0] product;
 reg signed [83:0] acc;
 wire signed [47:0] input_q16=$signed({{16{linear[31]}},linear})<<<16;
 wire signed [79:0] multiplication=operand*coefficient;
 wire signed [83:0] final_q16=(acc-$signed({{4{product[79]}},product}))>>>26;
 wire range_error=final_q16>84'sd140737488355327 || final_q16 < -84'sd140737488355328;
 function signed [31:0] rounded_output;
  input signed [47:0] q16;
  reg signed [48:0] rounded;
  begin
   rounded=($signed({q16[47],q16})+49'sd32768)>>>16;
   rounded_output=rounded>49'sd2147483647 ? 32'sh7fffffff :
                  rounded < -49'sd2147483648 ? 32'sh80000000 : rounded[31:0];
  end
 endfunction
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   busy<=0;done<=0;overflow<=0;phase<=0;primed<=0;last_profile<=0;shaped<=0;
   x0<=0;x1<=0;x2<=0;y1<=0;y2<=0;operand<=0;coefficient<=0;product<=0;acc<=0;
   b0<=0;b1<=0;b2<=0;a1<=0;a2<=0;
  end else if(cancel)begin
   busy<=0;done<=0;overflow<=0;phase<=0;primed<=0;shaped<=0;
  end else begin
   done<=0;
   if(!active || profile_id==0)primed<=0;
   if(launch && !busy)begin
    overflow<=0;x0<=input_q16;last_profile<=profile_id;
    // Generated coefficient assignments inserted below.
    case(profile_id)
     2'd1:begin b0<=32'sd66538877;b1<=-32'sd131622779;b2<=32'sd65398903;a1<=-32'sd131622779;a2<=32'sd64828916;end
     2'd2:begin b0<=32'sd66253883;b1<=-32'sd131622779;b2<=32'sd65683897;a1<=-32'sd131622779;a2<=32'sd64828916;end
     2'd3:begin b0<=32'sd65987608;b1<=-32'sd129422966;b2<=32'sd63745094;a1<=-32'sd129422966;a2<=32'sd62623838;end
     default:begin b0<=32'sd67108864;b1<=0;b2<=0;a1<=0;a2<=0;end
    endcase
    if(!active || profile_id==0)begin
     shaped<=linear;done<=1;busy<=0;phase<=0;
    end else if(!primed || profile_id!=last_profile)begin
     // Steady-state initialization has no kick on CATCH->BAL or mode change.
     x1<=input_q16;x2<=input_q16;y1<=input_q16;y2<=input_q16;
     shaped<=linear;primed<=1;done<=1;busy<=0;phase<=0;
    end else begin busy<=1;phase<=1;end
   end else if(busy)begin
    case(phase)
     1:begin operand<=x0;coefficient<=b0;acc<=0;phase<=2;end
     2:begin product<=multiplication;phase<=3;end
     3:begin acc<={{4{product[79]}},product};operand<=x1;coefficient<=b1;phase<=4;end
     4:begin product<=multiplication;phase<=5;end
     5:begin acc<=acc+{{4{product[79]}},product};operand<=x2;coefficient<=b2;phase<=6;end
     6:begin product<=multiplication;phase<=7;end
     7:begin acc<=acc+{{4{product[79]}},product};operand<=y1;coefficient<=a1;phase<=8;end
     8:begin product<=multiplication;phase<=9;end
     9:begin acc<=acc-{{4{product[79]}},product};operand<=y2;coefficient<=a2;phase<=10;end
     10:begin product<=multiplication;phase<=11;end
     11:begin
      overflow<=range_error;
      shaped<=range_error ? 32'sd0 : rounded_output(final_q16[47:0]);
      if(!range_error)begin x2<=x1;x1<=x0;y2<=y1;y1<=final_q16[47:0];end
      else primed<=0;
      done<=1;busy<=0;phase<=0;
     end
     default:begin busy<=0;phase<=0;primed<=0;overflow<=1;end
    endcase
   end
  end
 end
endmodule
