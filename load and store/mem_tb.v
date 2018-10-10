//`include "imem.v"

module top;
//imem(mem_rd,data,wr_data,op1,mem_rd1,data1,wr_data1,op2,cout,clk);
reg mem_rd,mem_rd1;//inputs
	reg  [7:0]data,data1; //inputs
	reg [31:0]wr_data,wr_data1; //inputs
	reg clk;
	wire cout;
	wire [31:0]op1,op2;


imem mmmm(mem_rd,data,wr_data,op1,mem_rd1,data1,wr_data1,op2,cout,clk);

initial
clk=1'b0;
initial
begin

data=8'd0;

//d_in=10'b1010001100;
mem_rd=1'b1;

data1=8'd1;

//d_in=10'b1010001100;
mem_rd1=1'b1;


#4

mem_rd=1'b0;
data=8'd0;
wr_data=8'd1;

mem_rd1=1'b0;
data1=8'd1;
wr_data1=8'd5;
#4

data=8'd0;

mem_rd=1'b1;

data1=8'd1;

mem_rd1=1'b1;
//dmemwr=1'b1;

#4
mem_rd1=1'b1;

data1=8'd128;

#20 $finish;
end

always
#2 clk = ~clk;
//nitial
//$monitor($time,"data=%b,data1=%b",data,data);

//end
endmodule
