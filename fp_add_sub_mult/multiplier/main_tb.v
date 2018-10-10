`include"main.v"
module maintb();


reg [22:0]ma,mb;
reg [7:0]ea,eb;
reg clk;
reg sa,sb;

wire [23:0]ma_out;
wire signbit;
wire [7:0]exponent;
main mooo (sa,sb,ea,eb,ma,mb,ma_out,signbit,exponent,clk);



initial 
clk=1'b0;

initial
begin

ea=8'b10000001;
eb=8'b10000010;
sa=1'b0;
sb=1'b1;
ma=23'b01000000000000000000000;
mb=23'b01000000000000000000000;

#40 $finish;
end

always
#2 clk=~clk;

initial
$monitor($time,"clk=%b,sa=%b,sb=%b,ea=%b,eb=%b,ma=%b,mb=%b,ma_out=%b,signbit=%b,exponent=%b",clk,sa,sb,ea,eb,ma,mb,ma_out,signbit,exponent);

endmodule
