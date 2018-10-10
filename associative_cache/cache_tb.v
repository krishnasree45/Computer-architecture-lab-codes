//`include "cache.v"
module mem_tb();
reg clk;
reg cache_read;
reg [7:0]data;
reg [31:0]wr_data; //inputs
wire [31:0]op;
wire [31:0]eql1;
wire cache_miss;
wire [4:0]out;
cache m0(cache_read,cache_miss,wr_data,data,op,eql1,c,out,clk);

initial 
clk=1'b0;


initial 
begin 

cache_read=1'b1;
data=8'd1;
 
#12
cache_read=1'b1;
data=8'd128;

#12

cache_read=1'b0;
data=8'd128;
wr_data=8'd2;
#12
cache_read=1'b1;
data=8'd128;

//cache_write_data=32'd2;

/*#12

cache_read=1'b1;
data=8'd11;*/

/*data=8'd1;

#8
 data=8'd2;
 
#8
 data=8'd16;
 

#8

 data=8'd10;
*/


 #12$finish;
 end
 
 always
#2 clk=~clk;

//initial
//
endmodule
