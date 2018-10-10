`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/17/2017 02:37:04 PM
// Design Name: 
// Module Name: carry_pipe_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module carry_pipe_tb ;
reg [31:0]a,b;
reg cin,clk;
wire cout;
wire [31:0]s;

carry_pipe out(cout,s,a,b,cin,clk);

initial
clk=1'b0;

initial
begin


#4
a=32'b00000000000000000000000000000011;
b=32'b00000000000000000000000000000011;
cin=1'b1;

#4
a=32'b00000000000000000000000000000111;
b=32'b00000000000000000000000000000111;
cin=1'b1;


#8 $finish;
end

always
#2 clk=~clk;

//$monitor($time,"clock=%b,a=%b,b=%b,s=%b,cout=%b",clk,a,b,s,cout);
endmodule
