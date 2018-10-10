`include "proc_dup.v"
module mem_tb();
reg clk;
 reg reset;
 reg [7:0]pc;
 wire [31:0]inst,check_wb;
 wire [31:0]data_a;
 wire [31:0]data_b;
 wire [7:0]insop,write_back;
 wire [7:0]insa;
 wire [7:0]insb;
 wire [31:0]exec_out;
 
  main m0(clk,reset,pc,inst,insop,insa,insb,data_a,data_b,exec_out,write_back,check_wb);

initial 
clk=1'b0;


initial 
begin 

pc=8'b00000000;

#20
pc=8'b00000001;
#40 
pc=8'b00000010;

 #30$finish;
 end
 
 always
#2 clk=~clk;

initial

$monitor ($time,"clock=%b,inst=%b,opcode=%d,insa=%d,insb=%d,data_a=%b,data_b=%b,exec_out=%b,write_back=%b,check_wb=%b\n",clk,inst,insop,insa,insb,data_a,data_b,exec_out,write_back,check_wb);

endmodule
