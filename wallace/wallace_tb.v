//`include "wallace.v"
`timescale 1ns / 1ps
module top;
reg [31:0]a,b;
reg clk;
wire cout;
wire [63:0]s;

wallace out(a,b,s,cout,clk);

initial
clk=1'b0;

initial
begin


#2
a=32'b00000000000000000000000000000011;
b=32'b00000000000000000000000000000011;


/*#4
a=16'b0000000000000111;
b=16'b0000000000010011;
cin=1'b1;*/


#34 $finish;
end

always
#2 clk=~clk;

initial
$monitor($time,"clock=%b,a=%b,b=%b,s=%b,cout=%b",clk,a,b,s,cout);
endmodule
