//`include "ins_cache.v"
//`include "cla32.v"
//`include "wallace.v"
//`include "fresh_pipe.v"
//`include "fp_mult.v"
//`include "left_barrel.v"
//`include "barrelshift32.v"
//`include "ff_proc.v"

module main(clk,reset,pc,inst,opcode,insa,insb,data_a,data_b,exec_out,write_back,check_wb);
 input clk,reset;
 input [7:0]pc;
 output [31:0]inst;
 output [31:0]data_op;
 output [31:0]data_a;
 output [31:0]data_b;
 output [7:0]insa;
 output [7:0]insb;
 output [7:0]opcode;
 output [31:0] exec_out,check_wb;
 wire [31:0] execout_ip;
 wire [7:0]opcode1;
 wire [7:0]ea1,eb1;
 output [7:0]write_back;
 wire [31:0] inst_ff;
 wire [7:0]insa_ff,insb_ff,write_back_ff;
 wire [31:0] data_aff,data_bff;
 wire [31:0] check_wb_ff;
 reg [31:0]reg_file[31:0];
 initial 
 begin
 	reg_file[0]=32'b00000000000000000000000000000000;
	reg_file[1]=32'b00000000000000000000000000000001;
	reg_file[2]=32'b00000000000000000000000000000010;
	reg_file[3]=32'b00000000000000000000000000000011;
	reg_file[4]=32'b00000000000000000000000000000100;
	reg_file[5]=32'b00000000000000000000000000000101;
	reg_file[6]=32'b00000000000000000000000000000110;
	reg_file[7]=32'b01000000000000000000000000000100;
	reg_file[8]=32'b00000000000000000000000000000100;
	reg_file[9]=32'b00000000000000000000000000001001;
	reg_file[10]=32'b00000000000000000000000000001010;
	reg_file[11]=32'b00000000000000000000000000001011;
	reg_file[12]=32'b00000000000000000000000000001100;
	reg_file[13]=32'b00000000000000000000000000001101;
	reg_file[14]=32'b00000000000000000000000000001110;
	reg_file[15]=32'b00000000000000000000000000001111;
 
 end


cache inf(1,0,pc,inst_ff,cache_miss,clk);//fetch
ff_fetch f00000( inst_ff, clk, inst);

assign opcode=inst[7:0];
 assign write_back=inst[31:24];
 


assign insa=inst[15:8];
  assign insb=inst[23:16];
 
  assign data_aff=reg_file[insa];
  assign data_bff=reg_file[insb];
ff_decode dec00( insa_ff,insb_ff,data_aff,data_bff,write_back_ff, clk, insa,insb,data_a,data_b,write_back);

ins_execute exec(opcode,data_a,data_b,execout_ip,clk);
ff_execute e000( execout_ip, clk, exec_out);


always@(*)
 begin
 reg_file[write_back]<=exec_out;
 end
assign  check_wb=reg_file[write_back];
ff_writeback  ffwb(check_wb_ff,clk,check_wb);

endmodule


 
module ins_execute(opcode,data_a,data_b,exec_out,clk);
  input [7:0]opcode;
  input clk;
  input [31:0]data_a,data_b,data_a1,data_b1;
  output reg [31:0] exec_out;
  wire [31:0]cla_out,wallace_out1,fp_add_out,fp_sub_out,fp_mult_out,a,out,shift_left_out,shift_right_out;
  wire [63:0]wallace_out;
  wire cout1,carry,s0,s1,s2,s3,s4;
  wire s00,s11,s22,s33,s44;
  wire [31:0] abb,outtt;
  wire [7:0]ea1,eb1;
  
  
 
  //multiplication variables
  wire sa_mult,sb_mult,sign_mult,totalcarry_mult,signbit_mult;//
  wire [22:0]ma_mult,mb_mult;//
   wire [7:0]ea_mult,eb_mult,exponent_mult;//
   wire [23:0] ma_out_mult;//
   
   //addition variables
    wire sa_add,sb_add,sign_add,totalcarry_add,signbit_add;//
    wire [22:0]ma_add,mb_add;//
    wire [7:0]ea_add,eb_add,exponent_add;//
    wire [23:0] ma_out_add;//
    wire [23:0]out1_add,out2_add,sum_add;//
    wire [7:0]x_temp1_add;
    
    
    //subtraction 
    wire sa_sub,sb_sub,sign_sub,totalcarry_sub,signbit_sub;//
    wire [22:0]ma_sub,mb_sub;//
    wire [7:0]ea_sub,eb_sub,exponent_sub;//
    wire [23:0] ma_out_sub;//
    wire [23:0]out1_sub,out2_sub,sum_sub;//
    wire [7:0]x_temp1_sub;
   
   
  assign data_a1=data_a;
  assign data_b1=data_b;
  
  cla16 c0(cout1,cla_out,data_a1,data_b1,0,clk);
  
  wallacetreemult  w0(data_a1,data_b1,wallace_out,carry,clk);
  
  assign wallace_out1[31:0]=wallace_out[31:0];
  
  
  assign sa_mult=data_a[0];
  assign sb_mult=data_b[0];
  assign ea_mult=data_a[8:1];
  assign eb_mult=data_b[8:1];
  assign ma_mult=data_a[31:9];
  assign mb_mult=data_b[31:9];
  
  fp_mult gggggg(sa_mult,sb_mult,ea_mult,eb_mult,ma_mult,mb_mult,ma_out_mult,signbit_mult,exponent_mult,clk);
  
  assign fp_mult_out[0]=signbit_mult;
  assign fp_mult_out[8:1]=exponent_mult;
  assign fp_mult_out[31:9]=ma_out_mult[22:0];
  
  
  
  assign sa_add=data_a[0];
  assign sb_add=data_b[0];
  assign ea_add=data_a[8:1];
  assign eb_add=data_b[8:1];
  assign ma_add=data_a[31:9];
  assign mb_add=data_b[31:9];
  
  fp_add fooq(sa_add,sb_add,0,ma_add,mb_add,ea_add,eb_add,ma_out_add,signbit_add,exponent_add,totalcarry_add,out1_add,out2_add,x_temp1_add,sum_add,clk);
  
  assign fp_add_out[0]=signbit_add;
  assign fp_add_out[8:1]=exponent_add;
  assign fp_add_out[31:9]=ma_out_add[22:0];
  
  
  
  assign sa_sub=data_a[0];
  assign sb_sub=data_b[0];
  assign ea_sub=data_a[8:1];
  assign eb_sub=data_b[8:1];
  assign ma_sub=data_a[31:9];
  assign mb_sub=data_b[31:9];
  
  fp_add fooo(sa_sub,sb_sub,1,ma_sub,mb_sub,ea_sub,eb_sub,ma_out_sub,signbit_sub,exponent_sub,totalcarry_sub,out1_sub,out2_sub,x_temp1_sub,sum_sub,clk);
  
  assign fp_sub_out[0]=signbit_sub;
  assign fp_sub_out[8:1]=exponent_sub;
  assign fp_sub_out[9]=1'b0;
  assign fp_sub_out[31:10]=ma_out_sub[21:0];
  
  assign a=data_a;
  assign s0=data_b[0];
  assign s1=data_b[1];
  assign s2=data_b[2];
  assign s3=data_b[3];
  assign s4=data_b[0];
  leftbarrelshift32 fffffffff(a,s0,s1,s2,s3,s4,out);
  assign shift_left_out=out;
  
   assign abb=data_a;
  assign s00=data_b[0];
  assign s11=data_b[1];
  assign s22=data_b[2];
  assign s33=data_b[3];
  assign s44=data_b[0];
  barrelshift32  gfbfds(abb,s00,s11,s22,s33,s44,outtt);
  assign shift_right_out=outtt;
  
  always @(*)
  begin
	  if(opcode==00000000)
	  begin
	  	
	  #0.5	exec_out <= cla_out;
	  end
	  else if(opcode==00000001)
	  begin
	  #2.5	exec_out <= wallace_out1;
	  end
	  else if(opcode==00000010)
	  begin
	  #3	exec_out <= fp_add_out;
	  end
	  else if(opcode==00000011)
	  begin
	  #4	exec_out <= fp_sub_out;
	  end
	  else if(opcode==00000100)
	  begin
	  #5	exec_out <= fp_mult_out;
	  end
	  else if(opcode==00000101)
	  begin
	  #16	exec_out <= shift_left_out;
	  end
	   else if(opcode==00000110)
	  begin
	  #7	exec_out <= shift_right_out;
	  end
	  else if(opcode==00000111)
	  begin
	  #8	exec_out <= data_a & data_b;
	  end
	  else if(opcode==00001000)
	  begin
	  #9	exec_out <= data_a | data_b;
	  end
	  else if(opcode==00001001)
	  begin
	  #10	exec_out <= data_a ^ data_b;
	  end
  end
endmodule
 
 
 ////ins cache
 
 module cache(cache_read,wr_data,data,op,cache_miss,clk);//module for accessing memory 16*8 
	
	input [7:0]data;
	input cache_read; //inputs
	input clk;
	input [31:0]wr_data;
	output reg[31:0]op;//out put
	output reg cache_miss;
	output [31:0]e;
	output [4:0]out;
	reg [31:0]cache_data[31:0];
	reg [7:0]cache_tag[31:0];
	reg mem_rd1;
wire [31:0]op1,op2;
wire mem_rd,cout;
output c;



	initial
	begin
	
	cache_tag[0]=8'd0;
	cache_tag[1]=8'd1;
	cache_tag[2]=8'd2;
	cache_tag[3]=8'd3;
	cache_tag[4]=8'd4;
	cache_tag[5]=8'd5;
	cache_tag[6]=8'd6;
	cache_tag[7]=8'd7;
	cache_tag[8]=8'd8;
	cache_tag[9]=8'd9;
	cache_tag[10]=8'd10;
	cache_tag[11]=8'd11;
	cache_tag[12]=8'd12;
	cache_tag[13]=8'd13;
	cache_tag[14]=8'd14;
	cache_tag[15]=8'd15;

	
	cache_data[0]=32'b00000010000000010000000000000000;
	cache_data[1]=32'b00000011000000100000000100000000;
	cache_data[2]=32'b00000100000000110000001000000000;
	cache_data[3]=32'b00000000000000000000000000000110;
	cache_data[4]=32'b00000000000000000000000000000100;
	cache_data[5]=32'b00000000000000000000000000000101;
	cache_data[6]=32'b00000000000000000000000000000110;
	cache_data[7]=32'b01000000000000000000000000000111;
	cache_data[8]=32'b00000000000000000000000000001000;
	cache_data[9]=32'b00000000000000000000000000001001;
	cache_data[10]=32'b00000000000000000000000000001010;
	cache_data[11]=32'b00000000000000000000000000001011;
	cache_data[12]=32'b00000000000000000000000000001100;
	cache_data[13]=32'b00000000000000000000000000001101;
	cache_data[14]=32'b00000000000000000000000000001110;
	cache_data[15]=32'b00000000000000000000000000001111;
	
	end
	assign mem_rd=cache_read;
	
	imem mmm(mem_rd,data,wr_data,op1,mem_rd,data,wr_data,op2,cout,clk);
	
	assign c=cout;
	comparator0 c1(data,cache_tag[0],e[0],clk);
	comparator1 c2(data,cache_tag[1],e[1],clk);
	comparator2 c3(data,cache_tag[2],e[2],clk);
	comparator3 c4(data,cache_tag[3],e[3],clk);
	comparator4 c5(data,cache_tag[4],e[4],clk);
	comparator5 c6(data,cache_tag[5],e[5],clk);
	comparator6 c7(data,cache_tag[6],e[6],clk);
	comparator7 c8(data,cache_tag[7],e[7],clk);
	comparator8 c9(data,cache_tag[8],e[8],clk);
	comparator9 c10(data,cache_tag[9],e[9],clk);
	comparator10 c11(data,cache_tag[10],e[10],clk);
	comparator11 c12(data,cache_tag[11],e[11],clk);
	comparator12 c13(data,cache_tag[12],e[12],clk);
	comparator13 c14(data,cache_tag[13],e[13],clk);
	comparator14 c15(data,cache_tag[14],e[14],clk);
	
	
assign e[31:15]=0;
pri_enc32 pp(e,out);
//assign op=cache_data[out];
	always @(posedge(clk))
	begin
	  if(cache_read)
	  begin
		if(e!=0 )//if there is no cache miss
		begin
		  op<=cache_data[out];
		  cache_miss<=1'b0;
		end
	  
		else if(e==0 && data[7]==1'b0)//reading data from memory
		begin
		 op<=op1;
		 cache_miss<=1'b1;
		 
		end
		
		else if(e==0 && data[7]==1'b1)//reading data from memory
		begin
		 op<=op2;
		 cache_miss<=1'b1;
		end
		
		else
		begin
		op<=32'hz;
		end
          end
          else if(!cache_read)
             begin
                if(e!=0)
                begin
                 cache_data[out]<=wr_data;
                 op<=32'hz;
                end
             end
	
	end
	
	
	
endmodule


//memory


module imem(mem_rd,data,wr_data,op1,mem_rd1,data1,wr_data1,op2,cout,clk);//module for accessing memory 16*8 
	input mem_rd,mem_rd1;//inputs
	input  [7:0]data,data1; //inputs
	input [31:0]wr_data,wr_data1; //inputs
	input clk;
	output reg cout;
	output [31:0]op1,op2;//out put
	reg [31:0]op1,op2;
	reg [31:0]address[255:0];
	reg [31:0]address1[255:0];
	//initializing address in memory
	initial
	begin
	address[0]=32'b00000000000000000000000000000000;
	address[1]=32'b00000000000000000000000000000001;
	address[2]=32'b00000000000000000000000000000010;
	address[3]=32'b00000000000000000000000000000011;
	address[4]=32'b00000000000000000000000000000100;
	address[5]=32'b00000000000000000000000000000101;
	address[6]=32'b00000000000000000000000000000110;
	address[7]=32'b00000000000000000000000000000111;
	address[8]=32'b00000000000000000000000000001000;
	address[9]=32'b00000000000000000000000000001001;
	address[10]=32'b00000000000000000000000000001010;
	address[11]=32'b00000000000000000000000000001011;
	address[12]=32'b00000000000000000000000000001100;
	address[13]=32'b00000000000000000000000000001101;
	address[14]=32'b00000000000000000000000000001110;
	address[16]=32'b00000000000000000000011101011111;
	
	address1[0]=32'b00000000000000000000000000000000;
	address1[1]=32'b00000000000000000000000000000001;
	address1[2]=32'b00000000000000000000000000000010;
	address1[3]=32'b00000000000000000000000000000011;
	address1[4]=32'b00000000000000000000000000000100;
	address1[5]=32'b00000000000000000000000000000101;
	address1[6]=32'b00000000000000000000000000000110;
	address1[7]=32'b00000000000000000000000000000111;
	address1[8]=32'b00000000000000000000000000001000;
	address1[9]=32'b00000000000000000000000000001001;
	address1[10]=32'b00000000000000000000000000001010;
	address1[11]=32'b00000000000000000000000000001011;
	address1[12]=32'b00000000000000000000000000001100;
	address1[13]=32'b00000000000000000000000000001101;
	address1[14]=32'b00000000000000000000000000001110;
	address1[16]=32'b00000000000000000000000000001111;
	address1[128]=32'b11111111111111111111111111111111;
	
	end
	always @(posedge(clk))
	begin
	  if(data[7]==1'b0)  /// instruction memory 0 to 127
	  begin
		if(!mem_rd)//memory write
		begin
		  address[data]=wr_data;
		  op1<=32'hz;
		end
		else if(mem_rd)//reading data from memory
		begin
		 op1<=address[data];
		 cout<=1'b0;
		end
		else 
		begin
		  op1<=32'hz;
		end
	  end
	
	  if(data1[7]==1'b1)/// data memory 128 to 256
	  begin
		if(!mem_rd1)//memory write
		begin
		  address1[data1]=wr_data1;
		  op2<=32'hz;
		end
		else if(mem_rd1)//reading data from memory
		begin
		 op2<=address1[data1];
		 cout<=1'b1;
		end
		else 
		begin
		  op2<=32'hz;
		end
	  end
	end
endmodule

module pri_enc(d,z,y,x,a);
input [7:0]d;
output x,y,z,a;
wire [5:0]k;

assign k[0]=(~d[7]&d[6]);//a
assign k[1]=(~d[7]&~d[6]&d[5]);//b
assign k[2]=(~d[7]&~d[6]&~d[5]&d[4]);//c
assign k[3]=(~d[7]&~d[6]&~d[5]&~d[4]&d[3]);//d
assign k[4]=(~d[7]&~d[6]&~d[5]&~d[4]&~d[3]&d[2]);//e
assign k[5]=(~d[7]&~d[6]&~d[5]&~d[4]&~d[3]&~d[2]&d[1]);//f

assign z=d[7]|k[0]|k[1]|k[2];
assign y=d[7]|k[0]|k[4]|k[3];
assign x=d[7]|k[1]|k[3]|k[5];
assign a=(~d[7]&~d[6]&~d[5]&~d[4]&~d[3]&~d[2]&~d[1]&~d[0]);


endmodule

module pri_enc32(d,out);

input [31:0]d;
wire Z,Y,X,W,V,A;
output [4:0]out;
wire [3:0]z,y,x,p,a;



pri_enc p0(d[7:0],z[0],y[0],x[0],a[0]);
pri_enc p1(d[15:8],z[1],y[1],x[1],a[1]);
pri_enc p2(d[23:16],z[2],y[2],x[2],a[2]);
pri_enc p3(d[31:24],z[3],y[3],x[3],a[3]);

assign A=a[3]&a[2]&a[1]&a[0];
assign out[4]=~a[3]|~a[2];
assign out[3]=~a[3]|(a[2]&(~a[1]));

assign p[0]=~(out[4]|out[3]);
assign p[1]=out[3]&(~out[4]);
assign p[2]=(~out[3])&out[4];
assign p[3]=out[3]&out[4];

assign out[2]=(z[0]&p[0] | z[1]&p[1] | z[2]&p[2] | z[3]&p[3]) ;
assign out[1]=(y[0]&p[0] | y[1]&p[1] | y[2]&p[2] | y[3]&p[3] );
assign out[0]=(x[0]&p[0] | x[1]&p[1] | x[2]&p[2] | x[3]&p[3]) ;


endmodule


module comparator0(a,b,eql,clk);
input clk;
input [7:0]a,b;
output reg eql;
reg [7:0]exnor,bnot;
always @(posedge(clk))
begin
	 bnot<=~b;
	 exnor<=(a&b)|(~a)&(~b);

	eql=&exnor;
//assign noteql=~(eql&1);
end
endmodule	
module comparator1(a,b,eql,clk);
input clk;
input [7:0]a,b;
output reg eql;
reg [7:0]exnor,bnot;
always @(posedge(clk))
begin
	 bnot<=~b;
	 exnor<=(a&b)|(~a)&(~b);
	eql=&exnor;
end
endmodule	

module comparator2(a,b,eql,clk);
input clk;
input [7:0]a,b;
output reg eql;
reg [7:0]exnor,bnot;
always @(posedge(clk))
begin
	 bnot<=~b;
	 exnor<=(a&b)|(~a)&(~b);
	eql=&exnor;
end
endmodule
module comparator3(a,b,eql,clk);
input clk;
input [7:0]a,b;
output reg eql;
reg [7:0]exnor,bnot;
always @(posedge(clk))
begin
	 bnot<=~b;
	 exnor<=(a&b)|(~a)&(~b);
	eql=&exnor;
end
endmodule
module comparator4(a,b,eql,clk);
input clk;
input [7:0]a,b;
output reg eql;
reg [7:0]exnor,bnot;
always @(posedge(clk))
begin
	 bnot<=~b;
	 exnor<=(a&b)|(~a)&(~b);
	eql=&exnor;
end
endmodule
module comparator5(a,b,eql,clk);
input clk;
input [7:0]a,b;
output reg eql;
reg [7:0]exnor,bnot;
always @(posedge(clk))
begin
	 bnot<=~b;
	 exnor<=(a&b)|(~a)&(~b);
	eql=&exnor;
end
endmodule
module comparator6(a,b,eql,clk);
input clk;
input [7:0]a,b;
output reg eql;
reg [7:0]exnor,bnot;
always @(posedge(clk))
begin
	 bnot<=~b;
	 exnor<=(a&b)|(~a)&(~b);
	eql=&exnor;
end
endmodule
module comparator7(a,b,eql,clk);
input clk;
input [7:0]a,b;
output reg eql;
reg [7:0]exnor,bnot;
always @(posedge(clk))
begin
	 bnot<=~b;
	 exnor<=(a&b)|(~a)&(~b);
	eql=&exnor;
end
endmodule
module comparator8(a,b,eql,clk);
input clk;
input [7:0]a,b;
output reg eql;
reg [7:0]exnor,bnot;
always @(posedge(clk))
begin
	 bnot<=~b;
	 exnor<=(a&b)|(~a)&(~b);
	eql=&exnor;
end
endmodule
module comparator9(a,b,eql,clk);
input clk;
input [7:0]a,b;
output reg eql;
reg [7:0]exnor,bnot;
always @(posedge(clk))
begin
	 bnot<=~b;
	 exnor<=(a&b)|(~a)&(~b);
	eql=&exnor;
end
endmodule
module comparator10(a,b,eql,clk);
input clk;
input [7:0]a,b;
output reg eql;
reg [7:0]exnor,bnot;
always @(posedge(clk))
begin
	 bnot<=~b;
	 exnor<=(a&b)|(~a)&(~b);
	eql=&exnor;
end
endmodule
module comparator11(a,b,eql,clk);
input clk;
input [7:0]a,b;
output reg eql;
reg [7:0]exnor,bnot;
always @(posedge(clk))
begin
	 bnot<=~b;
	 exnor<=(a&b)|(~a)&(~b);
	eql=&exnor;
end
endmodule
module comparator12(a,b,eql,clk);
input clk;
input [7:0]a,b;
output reg eql;
reg [7:0]exnor,bnot;
always @(posedge(clk))
begin
	 bnot<=~b;
	 exnor<=(a&b)|(~a)&(~b);
	eql=&exnor;
end
endmodule
module comparator13(a,b,eql,clk);
input clk;
input [7:0]a,b;
output reg eql;
reg [7:0]exnor,bnot;
always @(posedge(clk))
begin
	 bnot<=~b;
	 exnor<=(a&b)|(~a)&(~b);
	eql=&exnor;
end
endmodule
module comparator14(a,b,eql,clk);
input clk;
input [7:0]a,b;
output reg eql;
reg [7:0]exnor,bnot;
always @(posedge(clk))
begin
	 bnot<=~b;
	 exnor<=(a&b)|(~a)&(~b);
	eql=&exnor;
end
endmodule
module comparator15(a,b,eql,clk);
input clk;
input [7:0]a,b;
output reg eql;
reg [7:0]exnor,bnot;
always @(posedge(clk))
begin
	 bnot<=~b;
	 exnor<=(a&b)|(~a)&(~b);
	eql=&exnor;
end
endmodule
module comparator16(a,b,eql,clk);
input clk;
input [7:0]a,b;
output reg eql;
reg [7:0]exnor,bnot;
always @(posedge(clk))
begin
	 bnot<=~b;
	 exnor<=(a&b)|(~a)&(~b);
	eql=&exnor;
end
endmodule
module comparator17(a,b,eql,clk);
input clk;
input [7:0]a,b;
output reg eql;
reg [7:0]exnor,bnot;
always @(posedge(clk))
begin
	 bnot<=~b;
	 exnor<=(a&b)|(~a)&(~b);
	eql=&exnor;
end
endmodule



module comparator18(a,b,eql,clk);
input clk;
input [7:0]a,b;
output reg eql;
reg [7:0]exnor,bnot;
always @(posedge(clk))
begin
	 bnot<=~b;
	 exnor<=(a&b)|(~a)&(~b);

	eql=&exnor;
//assign noteql=~(eql&1);
end
endmodule	
module comparator19(a,b,eql,clk);
input clk;
input [7:0]a,b;
output reg eql;
reg [7:0]exnor,bnot;
always @(posedge(clk))
begin
	 bnot<=~b;
	 exnor<=(a&b)|(~a)&(~b);
	eql=&exnor;
end
endmodule	

module comparator20(a,b,eql,clk);
input clk;
input [7:0]a,b;
output reg eql;
reg [7:0]exnor,bnot;
always @(posedge(clk))
begin
	 bnot<=~b;
	 exnor<=(a&b)|(~a)&(~b);
	eql=&exnor;
end
endmodule
module comparator21(a,b,eql,clk);
input clk;
input [7:0]a,b;
output reg eql;
reg [7:0]exnor,bnot;
always @(posedge(clk))
begin
	 bnot<=~b;
	 exnor<=(a&b)|(~a)&(~b);
	eql=&exnor;
end
endmodule
module comparator22(a,b,eql,clk);
input clk;
input [7:0]a,b;
output reg eql;
reg [7:0]exnor,bnot;
always @(posedge(clk))
begin
	 bnot<=~b;
	 exnor<=(a&b)|(~a)&(~b);
	eql=&exnor;
end
endmodule
module comparator23(a,b,eql,clk);
input clk;
input [7:0]a,b;
output reg eql;
reg [7:0]exnor,bnot;
always @(posedge(clk))
begin
	 bnot<=~b;
	 exnor<=(a&b)|(~a)&(~b);
	eql=&exnor;
end
endmodule
module comparator24(a,b,eql,clk);
input clk;
input [7:0]a,b;
output reg eql;
reg [7:0]exnor,bnot;
always @(posedge(clk))
begin
	 bnot<=~b;
	 exnor<=(a&b)|(~a)&(~b);
	eql=&exnor;
end
endmodule
module comparator25(a,b,eql,clk);
input clk;
input [7:0]a,b;
output reg eql;
reg [7:0]exnor,bnot;
always @(posedge(clk))
begin
	 bnot<=~b;
	 exnor<=(a&b)|(~a)&(~b);
	eql=&exnor;
end
endmodule
module comparator26(a,b,eql,clk);
input clk;
input [7:0]a,b;
output reg eql;
reg [7:0]exnor,bnot;
always @(posedge(clk))
begin
	 bnot<=~b;
	 exnor<=(a&b)|(~a)&(~b);
	eql=&exnor;
end
endmodule
module comparator27(a,b,eql,clk);
input clk;
input [7:0]a,b;
output reg eql;
reg [7:0]exnor,bnot;
always @(posedge(clk))
begin
	 bnot<=~b;
	 exnor<=(a&b)|(~a)&(~b);
	eql=&exnor;
end
endmodule
module comparator28(a,b,eql,clk);
input clk;
input [7:0]a,b;
output reg eql;
reg [7:0]exnor,bnot;
always @(posedge(clk))
begin
	 bnot<=~b;
	 exnor<=(a&b)|(~a)&(~b);
	eql=&exnor;
end
endmodule
module comparator29(a,b,eql,clk);
input clk;
input [7:0]a,b;
output reg eql;
reg [7:0]exnor,bnot;
always @(posedge(clk))
begin
	 bnot<=~b;
	 exnor<=(a&b)|(~a)&(~b);
	eql=&exnor;
end
endmodule
module comparator30(a,b,eql,clk);
input clk;
input [7:0]a,b;
output reg eql;
reg [7:0]exnor,bnot;
always @(posedge(clk))
begin
	 bnot<=~b;
	 exnor<=(a&b)|(~a)&(~b);
	eql=&exnor;
end
endmodule
module comparator31(a,b,eql,clk);
input clk;
input [7:0]a,b;
output reg eql;
reg [7:0]exnor,bnot;
always @(posedge(clk))
begin
	 bnot<=~b;
	 exnor<=(a&b)|(~a)&(~b);
	eql=&exnor;
end
endmodule

 
 ///cla32.v
 

module cla16(cout1,q3,a,b,cin,clk);
output [31:0]q3;
output cout1;
input [31:0]a,b;
input clk;
input cin;
wire [31:0]g,p,c,s;
wire cin1,cout;
wire [31:0]q,q1,q2,q3;

assign g=a&b;
assign p=a^b;

d_ff faa(a[0],b[0],cin,clk,q[0],q1[0],cin1);
assign c[0]=g[0]|(p[0]&cin);
d_ff f0(a[0],b[0],c[0],clk,q[0],q1[0],q2[0]);


assign c[1]=g[1]|(p[1]&(g[0]|(p[0]&cin)));
d_ff f1(a[1],b[1],c[1],clk,q[1],q1[1],q2[1]);

assign c[2]=g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))));
d_ff f2(a[2],b[2],c[2],clk,q[2],q1[2],q2[2]);

assign c[3]=g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))));
d_ff f3(a[3],b[3],c[3],clk,q[3],q1[3],q2[3]);

assign c[4]=g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))));
d_ff f4(a[4],b[4],c[4],clk,q[4],q1[4],q2[4]);

assign c[5]=g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))));
d_ff f5(a[5],b[5],c[5],clk,q[5],q1[5],q2[5]);

assign c[6]=g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))));
d_ff f6(a[6],b[6],c[6],clk,q[6],q1[6],q2[6]);

assign c[7]=g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))));
d_ff f7(a[7],b[7],c[7],clk,q[7],q1[7],q2[7]);

assign c[8]=g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))));
d_ff f8(a[8],b[8],c[8],clk,q[8],q1[8],q2[8]);

assign c[9]=g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))));
d_ff f9(a[9],b[9],c[9],clk,q[9],q1[9],q2[9]);

assign c[10]=g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))));
d_ff f10(a[10],b[10],c[10],clk,q[10],q1[10],q2[10]);

assign c[11]=g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))));
d_ff f11(a[11],b[11],c[11],clk,q[11],q1[11],q2[11]);

assign c[12]=g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))));
d_ff f12(a[12],b[12],c[12],clk,q[12],q1[12],q2[12]);

assign c[13]=g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))));
d_ff f13(a[13],b[13],c[13],clk,q[13],q1[13],q2[13]);

assign c[14]=g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))))));
d_ff f14(a[14],b[14],c[14],clk,q[14],q1[14],q2[14]);

assign c[15]=g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))))))));
d_ff f15(a[15],b[15],c[15],clk,q[15],q1[15],q2[15]);

assign c[16]=g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))))))))));
d_ff f16(a[16],b[16],c[16],clk,q[16],q1[16],q2[16]);

assign c[17]=g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))))))))))));
d_ff f17(a[17],b[17],c[17],clk,q[17],q1[17],q2[17]);

assign c[18]=g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))))))))))))));
d_ff f18(a[18],b[18],c[18],clk,q[18],q1[18],q2[18]);

assign c[19]=g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))));
d_ff f19(a[19],b[19],c[19],clk,q[19],q1[19],q2[19]);

assign c[20]=g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))));
d_ff f20(a[20],b[20],c[20],clk,q[20],q1[20],q2[20]);

assign c[21]=g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))));
d_ff f21(a[21],b[21],c[21],clk,q[21],q1[21],q2[21]);

assign c[22]=g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))));
d_ff f22(a[22],b[22],c[22],clk,q[22],q1[22],q2[22]);

assign c[23]=g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))));
d_ff f23(a[23],b[23],c[23],clk,q[23],q1[23],q2[23]);

assign c[24]=g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))));
d_ff f24(a[24],b[24],c[24],clk,q[24],q1[24],q2[24]);

assign c[25]=g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))));
d_ff f25(a[25],b[25],c[25],clk,q[25],q1[25],q2[25]);

assign c[26]=g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p
[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))));
d_ff f26(a[26],b[26],c[26],clk,q[26],q1[26],q2[26]);

assign c[27]=g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))));
d_ff f27(a[27],b[27],c[27],clk,q[27],q1[27],q2[27]);

assign c[28]=g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))));
d_ff f28(a[28],b[28],c[28],clk,q[28],q1[28],q2[28]);

assign c[29]=g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))));
d_ff f29(a[29],b[29],c[29],clk,q[29],q1[29],q2[29]);

assign c[30]=g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));
d_ff f30(a[30],b[30],c[30],clk,q[30],q1[30],q2[30]);

assign c[31]=g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));
d_ff f31(a[31],b[31],c[31],clk,q[31],q1[31],q2[31]);







assign s[0]=q[0]^q1[0]^cin1;
assign s[1]=q[1]^q1[1]^q2[0];
assign s[2]=q[2]^q1[2]^q2[1];
assign s[3]=q[3]^q1[3]^q2[2];
assign s[4]=q[4]^q1[4]^q2[3];
assign s[5]=q[5]^q1[5]^q2[4];
assign s[6]=q[6]^q1[6]^q2[5];
assign s[7]=q[7]^q1[7]^q2[6];
assign s[8]=q[8]^q1[8]^q2[7];
assign s[9]=q[9]^q1[9]^q2[8];
assign s[10]=q[10]^q1[10]^q2[9];
assign s[11]=q[11]^q1[11]^q2[10]; 
assign s[12]=q[12]^q1[12]^q2[11];
assign s[13]=q[13]^q1[13]^q2[12];
assign s[14]=q[14]^q1[14]^q2[13];
assign s[15]=q[15]^q1[15]^q2[14];
assign s[16]=q[16]^q1[16]^q2[15];
assign s[17]=q[17]^q1[17]^q2[16];
assign s[18]=q[18]^q1[18]^q2[17];
assign s[19]=q[19]^q1[19]^q2[18];
assign s[20]=q[20]^q1[20]^q2[19];
assign s[21]=q[21]^q1[21]^q2[20];
assign s[22]=q[22]^q1[22]^q2[21];
assign s[23]=q[23]^q1[23]^q2[22];
assign s[24]=q[24]^q1[24]^q2[23];
assign s[25]=q[25]^q1[25]^q2[24];
assign s[26]=q[26]^q1[26]^q2[25];
assign s[27]=q[27]^q1[27]^q2[26];
assign s[28]=q[28]^q1[28]^q2[27];
assign s[29]=q[29]^q1[29]^q2[28];
assign s[30]=q[30]^q1[30]^q2[29];
assign s[31]=q[31]^q1[31]^q2[30];
assign cout=q2[31];

dff1 ob(s[0],cout,clk,q3[0],cout1);
dff1 ob1(s[1],cout,clk,q3[1],cout1);
dff1 ob2(s[2],cout,clk,q3[2],cout1);
dff1 ob3(s[3],cout,clk,q3[3],cout1);
dff1 ob111(s[4],cout,clk,q3[4],cout1);
dff1 ob4(s[5],cout,clk,q3[5],cout1);
dff1 ob5(s[6],cout,clk,q3[6],cout1);
dff1 ob6(s[7],cout,clk,q3[7],cout1);
dff1 ob7(s[8],cout,clk,q3[8],cout1);
dff1 ob8(s[9],cout,clk,q3[9],cout1);
dff1 ob9(s[10],cout,clk,q3[10],cout1);
dff1 ob10(s[11],cout,clk,q3[11],cout1);
dff1 ob11(s[12],cout,clk,q3[12],cout1);
dff1 ob12(s[13],cout,clk,q3[13],cout1);
dff1 ob13(s[14],cout,clk,q3[14],cout1);
dff1 ob14(s[15],cout,clk,q3[15],cout1);
dff1 ob15(s[16],cout,clk,q3[16],cout1);
dff1 ob16(s[17],cout,clk,q3[17],cout1);
dff1 ob17(s[18],cout,clk,q3[18],cout1);
dff1 ob18(s[19],cout,clk,q3[19],cout1);
dff1 ob19(s[20],cout,clk,q3[20],cout1);
dff1 ob20(s[21],cout,clk,q3[21],cout1);
dff1 ob21(s[22],cout,clk,q3[22],cout1);
dff1 ob22(s[23],cout,clk,q3[23],cout1);
dff1 ob23(s[24],cout,clk,q3[24],cout1);
dff1 ob24(s[25],cout,clk,q3[25],cout1);
dff1 ob25(s[26],cout,clk,q3[26],cout1);
dff1 ob26(s[27],cout,clk,q3[27],cout1);
dff1 ob27(s[28],cout,clk,q3[28],cout1);
dff1 ob28(s[29],cout,clk,q3[29],cout1);
dff1 ob29(s[30],cout,clk,q3[30],cout1);
dff1 ob30(s[31],cout,clk,q3[31],cout1);

endmodule


//ff.v


module d_ff ( a,b,c, clk, q,q1,q2);
   input a,b,c,clk;
   output q, q1,q2;
   wire clk;
   reg q, q1,q2;
     	 
   always @ (posedge clk)
   begin
    q <= a;
    q1<=b;
    q2 <= c;
 end

endmodule



///dff1.v

module dff1 (s,cout,clk,q3,cout1);
   input s,clk,cout;
   output q3,cout1;
   wire clk;
   reg q3,cout1;
     	 
   always @ (posedge clk)
   begin
    q3 <= s;
    cout1 <= cout;
   
 end

endmodule


////wallace.v

//`include "csa.v"
//`include "cla64.v"
//`include "ff1.v"
module wallacetreemult(A,B,R,carry,clk);
input [31:0]A,B;
input clk;
wire [63:0]U[63:0];
wire [63:0]V[63:0];
output [63:0]R;
output carry;
wire [63:0]P[63:0];
wire [63:0]q[63:0];
wire [63:0]q1[63:0];
wire [63:0]p1[63:0];


assign P[0]=B[0]?{32'b000,A}:64'h0000;
assign P[1]=B[1]?{32'b000,A}<<1:64'h0000;
assign P[2]=B[2]?{32'b000,A}<<2:64'h0000;
assign P[3]=B[3]?{32'b000,A}<<3:64'h0000;
assign P[4]=B[4]?{32'b000,A}<<4:64'h0000;
assign P[5]=B[5]?{32'b000,A}<<5:64'h0000;
assign P[6]=B[6]?{32'b000,A}<<6:64'h0000;
assign P[7]=B[7]?{32'b000,A}<<7:64'h0000;

assign P[8]=B[8]?{32'b000,A}<<8:64'h0000;
assign P[9]=B[9]?{32'b000,A}<<9:64'h0000;
assign P[10]=B[10]?{32'b000,A}<<10:64'h0000;
assign P[11]=B[11]?{32'b000,A}<<11:64'h0000;
assign P[12]=B[12]?{32'b000,A}<<12:64'h0000;
assign P[13]=B[13]?{32'b000,A}<<13:64'h0000;
assign P[14]=B[14]?{32'b000,A}<<14:64'h0000;
assign P[15]=B[15]?{32'b000,A}<<15:64'h0000;

assign P[16]=B[16]?{32'b000,A}<<16:64'h0000;
assign P[17]=B[17]?{32'b000,A}<<17:64'h0000;
assign P[18]=B[18]?{32'b000,A}<<18:64'h0000;
assign P[19]=B[19]?{32'b000,A}<<19:64'h0000;
assign P[20]=B[20]?{32'b000,A}<<20:64'h0000;
assign P[21]=B[21]?{32'b000,A}<<21:64'h0000;
assign P[22]=B[22]?{32'b000,A}<<22:64'h0000;
assign P[23]=B[23]?{32'b000,A}<<23:64'h0000;

assign P[24]=B[24]?{32'b000,A}<<24:64'h0000;
assign P[25]=B[25]?{32'b000,A}<<25:64'h0000;
assign P[26]=B[26]?{32'b000,A}<<26:64'h0000;
assign P[27]=B[27]?{32'b000,A}<<27:64'h0000;
assign P[28]=B[28]?{32'b000,A}<<28:64'h0000;
assign P[29]=B[29]?{32'b000,A}<<29:64'h0000;
assign P[30]=B[30]?{32'b000,A}<<30:64'h0000;
assign P[31]=B[31]?{32'b000,A}<<31:64'h0000;

//1ssst
csaveadder c1(P[0],P[1],P[2],U[0],V[0]);
ff1 f0 ( U[0],V[0], clk, q[0],q1[0]);

csaveadder c2(P[3],P[4],P[5],U[1],V[1]);
ff1 f1 ( U[1],V[1], clk, q[1],q1[1]);

csaveadder c3(P[6],P[7],P[8],U[2],V[2]);
ff1 f2 ( U[2],V[2], clk, q[2],q1[2]);

csaveadder c4(P[9],P[10],P[11],U[3],V[3]);
ff1 f3 ( U[3],V[3], clk, q[3],q1[3]);

csaveadder c5(P[12],P[13],P[14],U[4],V[4]);
ff1 f4 ( U[4],V[4], clk, q[4],q1[4]);

csaveadder c6(P[15],P[16],P[17],U[5],V[5]);
ff1 f5 ( U[5],V[5], clk, q[5],q1[5]);

csaveadder c7(P[18],P[19],P[20],U[6],V[6]);
ff1 f6 ( U[6],V[6], clk, q[6],q1[6]);

csaveadder c8(P[21],P[22],P[23],U[7],V[7]);
ff1 f7 ( U[7],V[7], clk, q[7],q1[7]);

csaveadder c9(P[24],P[25],P[26],U[8],V[8]);
ff1 f8 ( U[8],V[8], clk, q[8],q1[8]);

csaveadder c10(P[27],P[28],P[29],U[9],V[9]);
ff1 f9 ( U[9],V[9], clk, q[9],q1[9]);

ff1 f10( P[30],P[31],clk,p1[1],p1[2]);
//

//2ndd
csaveadder c11(q[0],q1[0],q[1],U[10],V[10]);
ff1 f11 ( U[10],V[10], clk, q[10],q1[10]);
 
csaveadder c12(q1[1],q[2],q1[2],U[11],V[11]);
ff1 f12 ( U[11],V[11], clk, q[11],q1[11]);

csaveadder c13(q[3],q1[3],q[4],U[12],V[12]);
ff1 f13 ( U[12],V[12], clk, q[12],q1[12]);

csaveadder c14(q1[4],q[5],q1[5],U[13],V[13]);
ff1 f14 ( U[13],V[13], clk, q[13],q1[13]);

csaveadder c15(q[6],q1[6],q[7],U[14],V[14]);
ff1 f15 ( U[14],V[14], clk, q[14],q1[14]);

csaveadder c16(q1[7],q[8],q1[8],U[15],V[15]);
ff1 f16 ( U[15],V[15], clk, q[15],q1[15]);

csaveadder c17(q[9],q1[9],p1[1],U[16],V[16]);
ff1 f17 ( U[16],V[16], clk, q[16],q1[16]);

ff1 f18 ( p1[2],p1[2],clk,p1[3],p1[4]);//p1[3]=p1[4]
//


//3rdd
csaveadder c18(q[10],q1[10],q[11],U[17],V[17]);
ff1 f19 ( U[17],V[17], clk, q[17],q1[17]);

csaveadder c19(q1[11],q[12],q1[12],U[18],V[18]);
ff1 f20 ( U[18],V[18], clk, q[18],q1[18]);

csaveadder c20(q[13],q1[13],q[14],U[19],V[19]);
ff1 f21 ( U[19],V[19], clk, q[19],q1[19]);

csaveadder c21(q1[14],q[15],q1[15],U[20],V[20]);
ff1 f22 ( U[20],V[20], clk, q[20],q1[20]);

csaveadder c22(q[16],q1[16],p1[4],U[21],V[21]);//p1[3]=p[31]
ff1 f23 ( U[21],V[21], clk, q[21],q1[21]);

ff1 f24 ( V[21],V[21], clk, p1[5],p1[6]);
//


//4th
csaveadder c23(q[17],q1[17],q[18],U[22],V[22]);
ff1 f25 ( U[22],V[22], clk, q[22],q1[22]);

csaveadder c24(q1[18],q[19],q1[19],U[23],V[23]);
ff1 f26 ( U[23],V[23], clk, q[23],q1[23]);

csaveadder c25(q[20],q1[20],q[21],U[24],V[24]);
ff1 f27 ( U[24],V[24], clk, q[24],q1[24]);

ff1 f28 ( p1[5],p1[6], clk, p1[7],p1[8]);
//


//5thhh

csaveadder c26(q[22],q1[22],q[23],U[25],V[25]);
ff1 f29 ( U[25],V[25], clk, q[25],q1[25]);

csaveadder c27(q1[23],q[24],q1[24],U[26],V[26]);
ff1 f30 ( U[26],V[26], clk, q[26],q1[26]);

ff1 f31 ( p1[7],p1[8], clk, p1[9],p1[10]);
//


//6thh
csaveadder c28(q[25],q1[25],q[26],U[27],V[27]);
ff1 f32 ( U[27],V[27], clk, q[27],q1[27]);

ff1 f33 ( p1[9],p1[10], clk, p1[11],p1[12]);
//


//7thh
csaveadder c29(q1[26],q[27],q1[27],U[28],V[28]);
ff1 f34 ( U[28],V[28], clk, q[28],q1[28]);

ff1 f35 ( p1[11],p1[12], clk, p1[13],p1[14]);

//


//8thh
csaveadder c30(q[28],q1[28],p1[13],U[29],V[29]);
ff1 f36 ( U[29],V[29], clk, q[29],q1[29]);

//
//rca_64bit q0(q[29],q1[29],0,R,carry);
cla32 c000(carry,R,q[29],q1[29],0);
//cla32(cout,s,a,b,cin);
endmodule


//csa.v

module csaveadder(X,Y,Z,U,V);
input [63:0]X;
input [63:0]Y;
input [63:0]Z;
output [63:0]U;
output [63:0]V;
wire temp;

assign V[0]=0;

fulladder fa0(X[0],Y[0],Z[0],U[0],V[1]);
fulladder fa1(X[1],Y[1],Z[1],U[1],V[2]);
fulladder fa2(X[2],Y[2],Z[2],U[2],V[3]);
fulladder fa3(X[3],Y[3],Z[3],U[3],V[4]);
fulladder fa4(X[4],Y[4],Z[4],U[4],V[5]);
fulladder fa5(X[5],Y[5],Z[5],U[5],V[6]);
fulladder fa6(X[6],Y[6],Z[6],U[6],V[7]);
fulladder fa7(X[7],Y[7],Z[7],U[7],V[8]);
fulladder fa8(X[8],Y[8],Z[8],U[8],V[9]);
fulladder fa9(X[9],Y[9],Z[9],U[9],V[10]);
fulladder fa10(X[10],Y[10],Z[10],U[10],V[11]);
fulladder fa11(X[11],Y[11],Z[11],U[11],V[12]);
fulladder fa12(X[12],Y[12],Z[12],U[12],V[13]);
fulladder fa13(X[13],Y[13],Z[13],U[13],V[14]);
fulladder fa14(X[14],Y[14],Z[14],U[14],V[15]);
fulladder fa15(X[15],Y[15],Z[15],U[15],V[16]);
fulladder fa16(X[16],Y[16],Z[16],U[16],V[17]);
fulladder fa17(X[17],Y[17],Z[17],U[17],V[18]);
fulladder fa18(X[18],Y[18],Z[18],U[18],V[19]);
fulladder fa19(X[19],Y[19],Z[19],U[19],V[20]);
fulladder fa20(X[20],Y[20],Z[20],U[20],V[21]);
fulladder fa21(X[21],Y[21],Z[21],U[21],V[22]);
fulladder fa22(X[22],Y[22],Z[22],U[22],V[23]);
fulladder fa23(X[23],Y[23],Z[23],U[23],V[24]);
fulladder fa24(X[24],Y[24],Z[24],U[24],V[25]);
fulladder fa25(X[25],Y[25],Z[25],U[25],V[26]);
fulladder fa26(X[26],Y[26],Z[26],U[26],V[27]);
fulladder fa27(X[27],Y[27],Z[27],U[27],V[28]);
fulladder fa28(X[28],Y[28],Z[28],U[28],V[29]);
fulladder fa29(X[29],Y[29],Z[29],U[29],V[30]);
fulladder fa30(X[30],Y[30],Z[30],U[30],V[31]);
fulladder fa31(X[31],Y[31],Z[31],U[31],V[32]);
fulladder fa32(X[32],Y[32],Z[32],U[32],V[33]);
fulladder fa33(X[33],Y[33],Z[33],U[33],V[34]);
fulladder fa34(X[34],Y[34],Z[34],U[34],V[35]);
fulladder fa35(X[35],Y[35],Z[35],U[35],V[36]);
fulladder fa36(X[36],Y[36],Z[36],U[36],V[37]);
fulladder fa37(X[37],Y[37],Z[37],U[37],V[38]);
fulladder fa38(X[38],Y[38],Z[38],U[38],V[39]);
fulladder fa39(X[39],Y[39],Z[39],U[39],V[40]);
fulladder fa40(X[40],Y[40],Z[40],U[40],V[41]);
fulladder fa41(X[41],Y[41],Z[41],U[41],V[42]);
fulladder fa42(X[42],Y[42],Z[42],U[42],V[43]);
fulladder fa43(X[43],Y[43],Z[43],U[43],V[44]);
fulladder fa44(X[44],Y[44],Z[44],U[44],V[45]);
fulladder fa45(X[45],Y[45],Z[45],U[45],V[46]);
fulladder fa46(X[46],Y[46],Z[46],U[46],V[47]);
fulladder fa47(X[47],Y[47],Z[47],U[47],V[48]);
fulladder fa48(X[48],Y[48],Z[48],U[48],V[49]);
fulladder fa49(X[49],Y[49],Z[49],U[49],V[50]);
fulladder fa50(X[50],Y[50],Z[50],U[50],V[51]);
fulladder fa51(X[51],Y[51],Z[51],U[51],V[52]);
fulladder fa52(X[52],Y[52],Z[52],U[52],V[53]);
fulladder fa53(X[53],Y[53],Z[53],U[53],V[54]);
fulladder fa54(X[54],Y[54],Z[54],U[54],V[55]);
fulladder fa55(X[55],Y[55],Z[55],U[55],V[56]);
fulladder fa56(X[56],Y[56],Z[56],U[56],V[57]);
fulladder fa57(X[57],Y[57],Z[57],U[57],V[58]);
fulladder fa58(X[58],Y[58],Z[58],U[58],V[59]);
fulladder fa59(X[59],Y[59],Z[59],U[59],V[60]);
fulladder fa60(X[60],Y[60],Z[60],U[60],V[61]);
fulladder fa61(X[61],Y[61],Z[61],U[61],V[62]);
fulladder fa62(X[62],Y[62],Z[62],U[62],V[63]);
fulladder fa63(X[63],Y[63],Z[63],U[63],temp);

endmodule

//fa.v

module fulladder(a,b,cin,s,cout);

input a,b,cin;
output s,cout;

assign s=a^b^cin;
assign cout=(a&b)|(b&cin)|(a&cin);

endmodule


///ff1.v

module ff1 ( a,b, clk, q,q1);
   input [63:0]a;
   input [63:0]b;
   //input [63:0]c;
   input clk;
   output [63:0]q;
   output  [63:0]q1;	
  // output [63:0]q63;
   wire clk;
   reg [63:0]q;
   reg  [63:0]q1;
   //reg [63:0]q63;
   always @ (posedge clk)
   begin
    q <= a;
    q1 <=b;
    //q63 <= c;
 end

endmodule

///cla 64.v


//`include "ff.v"
//`include "dff1.v"
module cla32(cout,s,a,b,cin);
output [63:0]s;
output cout;
input [63:0]a,b;
//input clk;
input cin;
wire [63:0]g,p,c,s;
//wire cin1,cout;
//wire [31:0]q,b,c,q3;

assign g=a&b;
assign p=a^b;

//d_ff faa(a[0],b[0],cin,clk,q[0],b[0],cin1);
assign c[0]=g[0]|(p[0]&cin);
//d_ff f0(a[0],b[0],c[0],clk,q[0],b[0],c[0]);


assign c[1]=g[1]|(p[1]&(g[0]|(p[0]&cin)));
//d_ff f1(a[1],b[1],c[1],clk,q[1],b[1],c[1]);

assign c[2]=g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))));
//d_ff f2(a[2],b[2],c[2],clk,q[2],b[2],c[2]);

assign c[3]=g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))));
//d_ff f3(a[3],b[3],c[3],clk,q[3],b[3],c[3]);

assign c[4]=g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))));
//d_ff f4(a[4],b[4],c[4],clk,q[4],b[4],c[4]);

assign c[5]=g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))));
//d_ff f5(a[5],b[5],c[5],clk,q[5],b[5],c[5]);

assign c[6]=g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))));
//d_ff f6(a[6],b[6],c[6],clk,q[6],b[6],c[6]);

assign c[7]=g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))));
//d_ff f7(a[7],b[7],c[7],clk,q[7],b[7],c[7]);

assign c[8]=g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))));
//d_ff f8(a[8],b[8],c[8],clk,q[8],b[8],c[8]);

assign c[9]=g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))));
//d_ff f9(a[9],b[9],c[9],clk,q[9],b[9],c[9]);

assign c[10]=g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))));
//d_ff f10(a[10],b[10],c[10],clk,q[10],b[10],c[10]);

assign c[11]=g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))));
//d_ff f11(a[11],b[11],c[11],clk,q[11],b[11],c[11]);

assign c[12]=g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))));
//d_ff f12(a[12],b[12],c[12],clk,q[12],b[12],c[12]);

assign c[13]=g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))));
//d_ff f13(a[13],b[13],c[13],clk,q[13],b[13],c[13]);

assign c[14]=g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))))));
//d_ff f14(a[14],b[14],c[14],clk,q[14],b[14],c[14]);

assign c[15]=g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))))))));
//d_ff f15(a[15],b[15],c[15],clk,q[15],b[15],c[15]);

assign c[16]=g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))))))))));
//d_ff f16(a[16],b[16],c[16],clk,q[16],b[16],c[16]);

assign c[17]=g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))))))))))));
//d_ff f17(a[17],b[17],c[17],clk,q[17],b[17],c[17]);

assign c[18]=g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))))))))))))));
//d_ff f18(a[18],b[18],c[18],clk,q[18],b[18],c[18]);

assign c[19]=g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))));
//d_ff f19(a[19],b[19],c[19],clk,q[19],b[19],c[19]);

assign c[20]=g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))));
//d_ff f20(a[20],b[20],c[20],clk,q[20],b[20],c[20]);

assign c[21]=g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))));
//d_ff f21(a[21],b[21],c[21],clk,q[21],b[21],c[21]);

assign c[22]=g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))));
//d_ff f22(a[22],b[22],c[22],clk,q[22],b[22],c[22]);

assign c[23]=g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))));
//d_ff f23(a[23],b[23],c[23],clk,q[23],b[23],c[23]);

assign c[24]=g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))));
//d_ff f24(a[24],b[24],c[24],clk,q[24],b[24],c[24]);

assign c[25]=g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))));
//d_ff f25(a[25],b[25],c[25],clk,q[25],b[25],c[25]);

assign c[26]=g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p
[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))));
//d_ff f26(a[26],b[26],c[26],clk,q[26],b[26],c[26]);

assign c[27]=g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))));
//d_ff f27(a[27],b[27],c[27],clk,q[27],b[27],c[27]);

assign c[28]=g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))));
//d_ff f28(a[28],b[28],c[28],clk,q[28],b[28],c[28]);

assign c[29]=g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))));
//d_ff f29(a[29],b[29],c[29],clk,q[29],b[29],c[29]);

assign c[30]=g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));
//d_ff f30(a[30],b[30],c[30],clk,q[30],b[30],c[30]);

assign c[31]=g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));
//d_ff f31(a[31],b[31],c[31],clk,q[31],b[31],c[31]);

assign c[32]=g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[33]=g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[34]=g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[35]=g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[36]=g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[37]=g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[38]=g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[39]=g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[40]=g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[41]=g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[42]=g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[43]=g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));


assign c[44]=g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));


assign c[45]=g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));


assign c[46]=g[46]|(p[46]&(g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));


assign c[47]=g[47]|(p[47]&(g[46]|(p[46]&(g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));


assign c[48]=g[48]|(p[48]&(g[47]|(p[47]&(g[46]|(p[46]&(g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));


assign c[49]=g[49]|(p[49]&(g[48]|(p[48]&(g[47]|(p[47]&(g[46]|(p[46]&(g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));


assign c[50]=g[50]|(p[50]&(g[49]|(p[49]&(g[48]|(p[48]&(g[47]|(p[47]&(g[46]|(p[46]&(g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));


assign c[51]=g[51]|(p[51]&(g[50]|(p[50]&(g[49]|(p[49]&(g[48]|(p[48]&(g[47]|(p[47]&(g[46]|(p[46]&(g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));


assign c[52]=g[52]|(p[51]&(g[51]|(p[51]&(g[50]|(p[50]&(g[49]|(p[49]&(g[48]|(p[48]&(g[47]|(p[47]&(g[46]|(p[46]&(g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));


assign c[53]=g[53]|(p[53]&(g[52]|(p[51]&(g[51]|(p[51]&(g[50]|(p[50]&(g[49]|(p[49]&(g[48]|(p[48]&(g[47]|(p[47]&(g[46]|(p[46]&(g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));


assign c[54]=g[54]|(p[54]&(g[53]|(p[53]&(g[52]|(p[51]&(g[51]|(p[51]&(g[50]|(p[50]&(g[49]|(p[49]&(g[48]|(p[48]&(g[47]|(p[47]&(g[46]|(p[46]&(g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));


assign c[55]=g[55]|(p[55]&(g[54]|(p[54]&(g[53]|(p[53]&(g[52]|(p[51]&(g[51]|(p[51]&(g[50]|(p[50]&(g[49]|(p[49]&(g[48]|(p[48]&(g[47]|(p[47]&(g[46]|(p[46]&(g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));


assign c[56]=g[56]|(p[56]&(g[55]|(p[55]&(g[54]|(p[54]&(g[53]|(p[53]&(g[52]|(p[51]&(g[51]|(p[51]&(g[50]|(p[50]&(g[49]|(p[49]&(g[48]|(p[48]&(g[47]|(p[47]&(g[46]|(p[46]&(g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[57]=g[57]|(p[57]&(g[56]|(p[56]&(g[55]|(p[55]&(g[54]|(p[54]&(g[53]|(p[53]&(g[52]|(p[51]&(g[51]|(p[51]&(g[50]|(p[50]&(g[49]|(p[49]&(g[48]|(p[48]&(g[47]|(p[47]&(g[46]|(p[46]&(g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[58]=g[58]|(p[58]&(g[57]|(p[57]&(g[56]|(p[56]&(g[55]|(p[55]&(g[54]|(p[54]&(g[53]|(p[53]&(g[52]|(p[51]&(g[51]|(p[51]&(g[50]|(p[50]&(g[49]|(p[49]&(g[48]|(p[48]&(g[47]|(p[47]&(g[46]|(p[46]&(g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[59]=g[59]|(p[59]&(g[58]|(p[58]&(g[57]|(p[57]&(g[56]|(p[56]&(g[55]|(p[55]&(g[54]|(p[54]&(g[53]|(p[53]&(g[52]|(p[51]&(g[51]|(p[51]&(g[50]|(p[50]&(g[49]|(p[49]&(g[48]|(p[48]&(g[47]|(p[47]&(g[46]|(p[46]&(g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[60]=g[60]|(p[60]&(g[59]|(p[59]&(g[58]|(p[58]&(g[57]|(p[57]&(g[56]|(p[56]&(g[55]|(p[55]&(g[54]|(p[54]&(g[53]|(p[53]&(g[52]|(p[51]&(g[51]|(p[51]&(g[50]|(p[50]&(g[49]|(p[49]&(g[48]|(p[48]&(g[47]|(p[47]&(g[46]|(p[46]&(g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[61]=g[61]|(p[61]&(g[60]|(p[60]&(g[59]|(p[59]&(g[58]|(p[58]&(g[57]|(p[57]&(g[56]|(p[56]&(g[55]|(p[55]&(g[54]|(p[54]&(g[53]|(p[53]&(g[52]|(p[51]&(g[51]|(p[51]&(g[50]|(p[50]&(g[49]|(p[49]&(g[48]|(p[48]&(g[47]|(p[47]&(g[46]|(p[46]&(g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[62]=g[62]|(p[62]&(g[61]|(p[61]&(g[60]|(p[60]&(g[59]|(p[59]&(g[58]|(p[58]&(g[57]|(p[57]&(g[56]|(p[56]&(g[55]|(p[55]&(g[54]|(p[54]&(g[53]|(p[53]&(g[52]|(p[51]&(g[51]|(p[51]&(g[50]|(p[50]&(g[49]|(p[49]&(g[48]|(p[48]&(g[47]|(p[47]&(g[46]|(p[46]&(g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[63]=g[63]|(p[63]&(g[62]|(p[62]&(g[61]|(p[61]&(g[60]|(p[60]&(g[59]|(p[59]&(g[58]|(p[58]&(g[57]|(p[57]&(g[56]|(p[56]&(g[55]|(p[55]&(g[54]|(p[54]&(g[53]|(p[53]&(g[52]|(p[51]&(g[51]|(p[51]&(g[50]|(p[50]&(g[49]|(p[49]&(g[48]|(p[48]&(g[47]|(p[47]&(g[46]|(p[46]&(g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));




assign s[0]=a[0]^b[0]^cin;
assign s[1]=a[1]^b[1]^c[0];
assign s[2]=a[2]^b[2]^c[1];
assign s[3]=a[3]^b[3]^c[2];
assign s[4]=a[4]^b[4]^c[3];
assign s[5]=a[5]^b[5]^c[4];
assign s[6]=a[6]^b[6]^c[5];
assign s[7]=a[7]^b[7]^c[6];
assign s[8]=a[8]^b[8]^c[7];
assign s[9]=a[9]^b[9]^c[8];
assign s[10]=a[10]^b[10]^c[9];
assign s[11]=a[11]^b[11]^c[10]; 
assign s[12]=a[12]^b[12]^c[11];
assign s[13]=a[13]^b[13]^c[12];
assign s[14]=a[14]^b[14]^c[13];
assign s[15]=a[15]^b[15]^c[14];
assign s[16]=a[16]^b[16]^c[15];
assign s[17]=a[17]^b[17]^c[16];
assign s[18]=a[18]^b[18]^c[17];
assign s[19]=a[19]^b[19]^c[18];
assign s[20]=a[20]^b[20]^c[19];
assign s[21]=a[21]^b[21]^c[20];
assign s[22]=a[22]^b[22]^c[21];
assign s[23]=a[23]^b[23]^c[22];
assign s[24]=a[24]^b[24]^c[23];
assign s[25]=a[25]^b[25]^c[24];
assign s[26]=a[26]^b[26]^c[25];
assign s[27]=a[27]^b[27]^c[26];
assign s[28]=a[28]^b[28]^c[27];
assign s[29]=a[29]^b[29]^c[28];
assign s[30]=a[30]^b[30]^c[29];
assign s[31]=a[31]^b[31]^c[30];
assign s[32]=a[32]^b[32]^c[31];
assign s[33]=a[33]^b[33]^c[32];
assign s[34]=a[34]^b[34]^c[33];
assign s[35]=a[35]^b[35]^c[34];
assign s[36]=a[36]^b[36]^c[35];
assign s[37]=a[37]^b[37]^c[36];
assign s[38]=a[38]^b[38]^c[37];
assign s[39]=a[39]^b[39]^c[38];
assign s[40]=a[40]^b[40]^c[39];
assign s[41]=a[41]^b[41]^c[40];
assign s[42]=a[42]^b[42]^c[41];
assign s[43]=a[43]^b[43]^c[42];
assign s[44]=a[44]^b[44]^c[43];
assign s[45]=a[45]^b[45]^c[44];
assign s[46]=a[46]^b[46]^c[45];
assign s[47]=a[47]^b[47]^c[46];
assign s[48]=a[48]^b[48]^c[47];
assign s[49]=a[49]^b[49]^c[48];
assign s[50]=a[50]^b[50]^c[49];
assign s[51]=a[51]^b[51]^c[50];
assign s[52]=a[52]^b[52]^c[51];
assign s[53]=a[53]^b[53]^c[52];
assign s[54]=a[54]^b[54]^c[53];
assign s[55]=a[55]^b[55]^c[54];
assign s[56]=a[56]^b[56]^c[55];
assign s[57]=a[57]^b[57]^c[56];
assign s[58]=a[58]^b[58]^c[57];
assign s[59]=a[59]^b[59]^c[58];
assign s[60]=a[60]^b[60]^c[59];
assign s[61]=a[61]^b[61]^c[60];
assign s[62]=a[62]^b[62]^c[61];
assign s[63]=a[63]^b[63]^c[62];

assign cout=c[63];





endmodule

////fresh_pipe.v

module fp_add(sa,sb,opcode,ma,mb,ea,eb,mant,sign,exponent,totalcarry,out1,out2,x_temp1,sum,clk);

 input [7:0]ea,eb;
input [22:0]ma,mb;
input sa,sb,opcode;
wire [23:0]ma1,mb1;
input clk;

reg [31:0]maaa,mbbb;
output [23:0]out1,out2;
output [23:0]mant;
output sign,totalcarry;
//output [31:0]sum2;
output [23:0]sum;
wire outcarry,signbit;
wire [31:0]mashift,mbshift;
wire [7:0]r;
wire [4:0]r1;
wire cout;
wire [31:0]madumy,mbdumy;
wire [1:0]elg;
wire [31:0]x1,x2,x11;
wire c111,ccc;
wire [31:0]pri_mant;
wire [31:0]mantissa,mantissa_temp;
output [23:0]out_mant;
wire [7:0] res;

reg [7:0]x_temp;
output [7:0]exponent;
output [7:0]x_temp1;
wire [7:0]temp_sub,out_sub;
wire A;
wire [4:0]out;
sub  sab(ea,eb,r,cout,clk);

assign ma1[23]=1'b1;
assign ma1[22:0]=ma[22:0];

assign mb1[23]=1'b1;
assign mb1[22:0]=mb[22:0];

assign mashift[7:0]=8'b0;
assign mbshift[7:0]=8'b0;

assign mashift[31:8]=ma1[23:0];
assign mbshift[31:8]=mb1[23:0];

always@*
begin
	if(cout==1'b1)
	begin 
	 x_temp<=ea;
	end
	else
	begin
	 x_temp<=eb;
	end
end

//assign exp=x_temp;
assign r1[4:0]=r[4:0];

shifterright32 s0(mashift,r1,x1,clk);//shifting ma

shifterright32 s1(mbshift,r1,x2,clk);//shifting mb

 
//new n0(r1,cout,x1,x2,madumy,mbdumy,elg);//for detecting which mantissa is shifted
new n000(r1,cout,x1,x2,x11,elg,clk);
//
always@(*)
begin
	if(elg==2'b10)
	 begin 
	 	mbbb<=x11;
		maaa<=mashift;
	 end
	if(elg==2'b01)
	 begin 
		maaa<=x11;
		mbbb<=mbshift;
	 end
	if(elg==2'b00)
	 begin 
		maaa<=mashift;
		mbbb<=mbshift;
	 end
end
modinp mod(maaa,mbbb,sa,sb,opcode,elg,out1,out2,sum,outcarry,signbit,clk);
modinput moddd(x_temp,outcarry,x_temp1,clk);


assign sign=signbit;
assign mant=sum;
assign totalcarry=outcarry;
//assign exp=x_temp1;

assign pri_mant[31:24]=7'b0;
assign pri_mant[23:0]=mant[23:0];

pri_enc132 p0000(pri_mant,out,A,clk);
//sub ss(ea,eb,r,cout);
assign temp_sub=8'b00010111;
assign out_sub[4:0]=out[4:0];
assign out_sub[7:5]=3'b0;
sub sdd(temp_sub,out_sub,res,ccc,clk);
//assign res1=res;
assign exponent=x_temp1-res;


//modinput2 mdi(res,mant,out_mant,clk);

//barrellft16(a,s0,s1,s2,s3,s4,o4);
/*assign mantissa[23:0]=mant[23:0];
assign mantissa[31:24]=8'b0;


barrellft16 b0(mantissa,res1[0],res1[1],res1[2],res1[3],res1[4],mantissa_temp);

assign mant[23:0]=mantissa_temp[23:0];*/
endmodule



module modinput2 (res,mant,q,clk);
input [7:0]res;
input [23:0]mant;
input clk;
wire [31:0]mantissa_temp,mantissa;
output [23:0]q;
wire [23:0]out_mant;
	assign mantissa[23:0]=mant[23:0];
	assign mantissa[31:24]=8'b0;

	barrellft16 b0(mantissa,res[4],res[1],res[2],res[3],res[0],mantissa_temp);
	assign out_mant[23:0]=mantissa_temp[23:0];
modinput2_dff dbfjd(out_mant,q,clk);
endmodule

module modinput2_dff (out_mant,q,clk);
   input [23:0]out_mant;
   input clk;
   output reg [23:0]q;
   wire clk;
  
   always @ (posedge clk)
   begin
    q <= out_mant;
    
 end
endmodule


module modinput (x_temp,q,q1,clk);
input [7:0]x_temp;
input clk;
wire outcarry;
 reg [7:0]x_temp1;
wire [7:0]exp1;
output [7:0]q1;
output q;
cla8 c000(c111,exp1,x_temp,00000001,0);//for final ops

always@*
begin
	if(outcarry==1'b1)
	begin 
	 x_temp1<=exp1;
	end
	else
	begin
	 x_temp1<=x_temp;
	end
end
modinput_dff df0000(outcarry,x_temp1,q,q1,clk);
endmodule

module modinput_dff ( outcarry,x_temp1,q,q1,clk);
   input outcarry;
   input [7:0]x_temp1;
   input clk;
   output reg q;
   output  reg [7:0] q1;
   wire clk;
  
   always @ (posedge clk)
   begin
    q <= outcarry;
    q1 <=x_temp1;
    
 end
endmodule

//mod(maaa,mbbb,sa,sb,opcode,elg,sum,outcarry,signbit,out1,out2,clk)
module modinp (maaa,mbbb,sa,sb,opcode,elg,q,q1,q2,q3,q4,clk);
input [31:0]maaa,mbbb;
input sa,sb,opcode;
input [1:0]elg;
input clk;
wire [23:0]out1,out2,sum;
wire signbit;
wire outcarry;

output [23:0]q,q1,q2;
output q3,q4;
assign out1[23:0]=maaa[31:8];
assign out2[23:0]=mbbb[31:8]; 

comb n1(sa,sb,opcode,out1,out2,elg,sum,outcarry,signbit);
modinp_dff  dffffff(out1,out2,sum,signbit,outcarry,q,q1,q2,q3,q4,clk);
endmodule


module modinp_dff ( out1,out2,sum,signbit,outcarry,q,q1,q2,q3,q4,clk);
   input [23:0]out1,out2,sum;
   input signbit,outcarry;
   //input [63:0]c;
   input clk;
   output reg[23:0]q,q1,q2;
   output  reg q3,q4;
   wire clk;
  
   always @ (posedge clk)
   begin
    q <= out1;
    q1 <=out2;
    q2<= sum;
    q3 <= signbit;
	q4<=outcarry;
    //q63 <= c;
 end
endmodule
module comb(sa,sb,opcode,out1,out2,elg,sum,cout,signbit);

input sa,sb,opcode;
input [1:0]elg;
input [23:0]out1,out2;
output [23:0]sum;
output cout,signbit;
reg [31:0]sum_temp;
reg carry_temp,sign_temp;
wire [31:0]s1,s2,s3,s4,s5,s6;
wire c1,c2,c3,c4,c5,c6;
wire [31:0]macomb,mbcomb;
wire [31:0]sum_temp1;

reg [31:0]x1;
wire temppp;


assign macomb[31:24]=7'b0;
assign macomb[23:0]=out1[23:0];

assign mbcomb[31:24]=7'b0;
assign mbcomb[23:0]=out2[23:0];


cla321 c11(temppp,s1,macomb,mbcomb,0);//1
assign c1=s1[24];
cla321 c12(temppp,s2,macomb,~mbcomb,1);//2
assign c2=s2[24];
cla321 c13(temppp,s4,macomb,mbcomb,0);//4
assign c4=s4[24];
cla321 c14(temppp,s5,~macomb,mbcomb,1);//5
assign c5=s5[24];
cla321 c15(temppp,s3,~s5,1,0);//3
assign c3=s3[24];

always@(*)
begin
 if(elg==2'b10)
 begin
 	if(sa==1'b0 && sb==1'b0 && opcode==1'b0)
	begin
		sum_temp<=s1;
		carry_temp<=c1;
		sign_temp<=1'b0;
		//sum2<=s1;
		
  	end
	if(sa==1'b0 && sb==1'b0 && opcode==1'b1)
	begin
		sum_temp<=s2;
		carry_temp<=1'b0;
		sign_temp<=1'b0;
		
  	end
	if(sa==1'b0 && sb==1'b1 && opcode==1'b0)
	begin
		sum_temp<=s2;
		carry_temp<=1'b0;
		sign_temp<=1'b0;
  	end
	if(sa==1'b0 && sb==1'b1 && opcode==1'b1)
	begin
		sum_temp<=s1;
		carry_temp<=c1;
		sign_temp<=1'b0;
  	end
	if(sa==1'b1 && sb==1'b0 && opcode==1'b0)
	begin
		sum_temp<=s3;
		carry_temp<=1'b0;
		sign_temp<=1'b1;
  	end
	if(sa==1'b1 && sb==1'b0 && opcode==1'b1)
	begin
		sum_temp<=s4;
		carry_temp<=c4;
		sign_temp<=1'b1;
  	end
	if(sa==1'b1 && sb==1'b1 && opcode==1'b0)
	begin
		sum_temp<=s4;
		carry_temp<=c4;
		sign_temp<=1'b1;
  	end
	if(sa==1'b1 && sb==1'b1 && opcode==1'b1)
	begin
		sum_temp<=s3;
		carry_temp<=1'b0;
		sign_temp<=1'b1;
  	end

 end


 if(elg==2'b01)
 begin
 	if(sa==1'b0 && sb==1'b0 && opcode==1'b0)
	begin
		sum_temp<=s1;
		carry_temp<=c1;
		sign_temp<=1'b0;
  	end
	if(sa==1'b0 && sb==1'b0 && opcode==1'b1)
	begin
		sum_temp<=s3;
		carry_temp<=1'b0;
		sign_temp<=1'b1;
  	end
	if(sa==1'b0 && sb==1'b1 && opcode==1'b0)
	begin
		sum_temp<=s3;
		carry_temp<=1'b0;
		sign_temp<=1'b1;
  	end
	if(sa==1'b0 && sb==1'b1 && opcode==1'b1)
	begin
		sum_temp<=s1;
		carry_temp<=c1;
		sign_temp<=1'b0;
  	end
	if(sa==1'b1 && sb==1'b0 && opcode==1'b0)
	begin
		sum_temp<=s5;
		carry_temp<=1'b0;
		sign_temp<=1'b0;
  	end
	if(sa==1'b1 && sb==1'b0 && opcode==1'b1)
	begin
		sum_temp<=s4;
		carry_temp<=c4;
		sign_temp<=1'b1;
  	end
	if(sa==1'b1 && sb==1'b1 && opcode==1'b0)
	begin
		sum_temp<=s4;
		carry_temp<=c4;
		sign_temp<=1'b1;
  	end
	if(sa==1'b1 && sb==1'b1 && opcode==1'b1)
	begin
		sum_temp<=s5;
		carry_temp<=1'b0;
		sign_temp<=1'b0;
  	end

 end


 if(elg==2'b00)
 begin
 	if(sa==1'b0 && sb==1'b0 && opcode==1'b0)
	begin
		sum_temp<=s1;
		carry_temp<=c1;
		sign_temp<=1'b0;
  	end
	if(sa==1'b0 && sb==1'b0 && opcode==1'b1)
	begin
		sum_temp<=1'b0;
		carry_temp<=1'b0;
		sign_temp<=1'b0;
  	end
	if(sa==1'b0 && sb==1'b1 && opcode==1'b0)
	begin
		sum_temp<=1'b0;
		carry_temp<=1'b0;
		sign_temp<=1'b0;
  	end
	if(sa==1'b0 && sb==1'b1 && opcode==1'b1)
	begin
		sum_temp<=s1;
		carry_temp<=c1;
		sign_temp<=1'b0;
  	end
	if(sa==1'b1 && sb==1'b0 && opcode==1'b0)
	begin
		sum_temp<=1'b0;
		carry_temp<=1'b0;
		sign_temp<=1'b0;
  	end
	if(sa==1'b1 && sb==1'b0 && opcode==1'b1)
	begin
		sum_temp<=s1;
		carry_temp<=c1;
		sign_temp<=1'b1;
  	end
	if(sa==1'b1 && sb==1'b1 && opcode==1'b0)
	begin
		sum_temp<=s1;
		carry_temp<=c1;
		sign_temp<=1'b1;
  	end
	if(sa==1'b1 && sb==1'b1 && opcode==1'b1)
	begin
		sum_temp<=1'b0;
		carry_temp<=1'b0;
		sign_temp<=1'b0;
  	end

 end

end


shifterright32  s0(sum_temp,00001,sum_temp1,clk);
always@(*)
begin
 if(carry_temp==1'b1)
 begin 
  x1<=sum_temp1;
 end
 else 
 begin 
  x1<=sum_temp;
 end
end


assign sum[23:0]=x1[23:0];
assign cout=carry_temp;
assign signbit=sign_temp;
endmodule


module pri_enc1(d,z,y,x,a);
	input [7:0]d;
	output x,y,z,a;
	wire [5:0]k;

	assign k[0]=(~d[7]&d[6]);//a
	assign k[1]=(~d[7]&~d[6]&d[5]);//b
	assign k[2]=(~d[7]&~d[6]&~d[5]&d[4]);//c
	assign k[3]=(~d[7]&~d[6]&~d[5]&~d[4]&d[3]);//d
	assign k[4]=(~d[7]&~d[6]&~d[5]&~d[4]&~d[3]&d[2]);//e
	assign k[5]=(~d[7]&~d[6]&~d[5]&~d[4]&~d[3]&~d[2]&d[1]);//f

	assign z=d[7]|k[0]|k[1]|k[2];
	assign y=d[7]|k[0]|k[4]|k[3];
	assign x=d[7]|k[1]|k[3]|k[5];
	assign a=(~d[7]&~d[6]&~d[5]&~d[4]&~d[3]&~d[2]&~d[1]&~d[0]);


endmodule

//`include "pri_enc1.v"
//
module pri_enc132(d,q,q1,clk);

	input [31:0]d;
	input clk;
	wire [4:0]out;
	wire A;
	output [4:0]q;
	output q1;
	wire [3:0]z,y,x,p,a;



	pri_enc1 p0(d[7:0],z[0],y[0],x[0],a[0]);
	pri_enc1 p1(d[15:8],z[1],y[1],x[1],a[1]);
	pri_enc1 p2(d[23:16],z[2],y[2],x[2],a[2]);
	pri_enc1 p3(d[31:24],z[3],y[3],x[3],a[3]);

	assign A=a[3]&a[2]&a[1]&a[0];
	assign out[4]=~a[3]|~a[2];
	assign out[3]=~a[3]|(a[2]&(~a[1]));

	assign p[0]=~(out[4]|out[3]);
	assign p[1]=out[3]&(~out[4]);
	assign p[2]=(~out[3])&out[4];
	assign p[3]=out[3]&out[4];

	assign out[2]=(z[0]&p[0] | z[1]&p[1] | z[2]&p[2] | z[3]&p[3]) ;
	assign out[1]=(y[0]&p[0] | y[1]&p[1] | y[2]&p[2] | y[3]&p[3] );
	assign out[0]=(x[0]&p[0] | x[1]&p[1] | x[2]&p[2] | x[3]&p[3]) ;

pri_enc1_dff dfppp(out ,A,q,q1,clk);
endmodule

module pri_enc1_dff ( out ,A,q,q1,clk);
   input [4:0]out;
   input A;
   input clk;
   output reg[4:0]q;
   output  reg q1;
   wire clk;
  
   always @ (posedge clk)
   begin
    q <= out;
    q1 <=A;
 
 end
endmodule

module new(r,cout,x1,x2,q,q1,clk);

input [4:0]r;
input cout;
input clk;
input [31:0]x1,x2;
 reg [31:0]x11;
reg [1:0]elg;
output  [31:0]q;
output  [1:0]q1;
//reg [31:0]ma;

always@(*)
begin
 if(cout==1'b1)
 begin
 x11<=x2;
 elg<=2'b10;
 end
 if(cout==1'b0 && r!=7'b0)
 begin
  x11<=x1;
  elg<=2'b01;
 end
 if(cout==1'b0 && r==7'b0)
 begin
   x11<=32'b0;
   elg<=2'b00;
 end
end

new_dff dff5(x11,elg,clk,q,q1);


endmodule

module new_dff ( a,b, clk, q,q1);
   input [31:0]a;
   input [1:0]b;
   input clk;
   output reg [31:0]q;
   output  reg [1:0]q1;	
   wire clk;
   always @ (posedge clk)
   begin
    q <= a;
    q1 <=b;
 end

endmodule


module sub(a,b,q,q1,clk);
input [7:0]a,b;
input clk;
 reg [7:0]r;
 reg cout;
output [7:0]q;
output q1;
wire e,l,g;
wire [7:0]x1,x2;
wire cout1,cout2; 
//comparator8(a,b,e,l,g)
comp8 c1(a,b,e,l,g);


//cla8(cout,s,a,b,cin);
cla8  c11(cout1,x1,a,~b,1);

cla8  c22(cout2,x2,b,~a,1);
always@(*)
begin
  if(l==1'b1)
  begin
	 r<=x2;
	 cout<=1'b0;
	 
  end
  if(g==1'b1)
  begin
	 r<=x1;
	 cout<=1'b1;
  end
  if(e==1'b1)
  begin
	r<=8'b0;
	cout<=1'b0;
  end

end
  sub_dff dffff1(r,cout,clk,q,q1);
endmodule

module sub_dff ( a,b, clk, q,q1);
   input [7:0]a;
   input b;
   input clk;
   output reg [7:0]q;
   output  reg q1;	
   wire clk;
   always @ (posedge clk)
   begin
    q <= a;
    q1 <=b;
 end

endmodule




module comp8(a,b,e,l,g);
input [7:0]a,b;
output e,l,g;
wire [7:0]temp,sum;
wire cout;

assign temp=(a&b)|(~a&~b);
assign e=&temp;
//cla8(cout,s,a,b,cin);
cla8 c0(cout,sum,a,~b,1);

assign l=(~cout)&1;

assign g=(~e)&(~l);

endmodule



module cla8(cout,s,a,b,cin);

input [7:0]a,b;
input cin;

output [7:0]s;
output cout;
wire [7:0]g,p,c;
assign g=a&b;
assign p=a^b;
assign c[0]=g[0]|(p[0]&cin);
assign c[1]=g[1]|(p[1]&(g[0]|(p[0]&cin)));
assign c[2]=g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))));
assign c[3]=g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))));
assign c[4]=g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))));

assign c[5]=g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))));

assign c[6]=g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))));

assign c[7]=g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))));


assign cout=c[7];
assign s[0]=p[0]^cin;
assign s[1]=p[1]^c[0];
assign s[2]=p[2]^c[1];
assign s[3]=p[3]^c[2];
assign s[4]=p[4]^c[3];
assign s[5]=p[5]^c[4];
assign s[6]=p[6]^c[5];
assign s[7]=p[7]^c[6];


endmodule



module barrellft16(a,s0,s1,s2,s3,s4,o4);
input [31:0]a;
input s0,s1,s2,s3,s4;
output [31:0]o4;
wire [31:0]o,o1,o2,o3;
wire ns0,ns1,ns2,ns3,ns4;
assign ns0=~s0;
assign ns1=~s1;
assign ns2=~s2;

assign ns3=~s3;
assign ns4=~s4;
//mux2 m33(a[31],a[32],ns0,o[32]);
mux2 m32(a[30],a[31],ns0,o[31]);
mux2 m31(a[29],a[30],ns0,o[30]);
mux2 m30(a[28],a[29],ns0,o[29]);
mux2 m29(a[27],a[28],ns0,o[28]);
mux2 m28(a[26],a[27],ns0,o[27]);
mux2 m27(a[25],a[26],ns0,o[26]);
mux2 m26(a[24],a[25],ns0,o[25]);
mux2 m25(a[23],a[24],ns0,o[24]);
mux2 m24(a[22],a[23],ns0,o[23]);
mux2 m23(a[21],a[22],ns0,o[22]);
mux2 m22(a[20],a[21],ns0,o[21]);
mux2 m21(a[19],a[20],ns0,o[20]);
mux2 m20(a[18],a[19],ns0,o[19]);
mux2 m19(a[17],a[18],ns0,o[18]);
mux2 m18(a[16],a[17],ns0,o[17]);
mux2 m17(a[15],a[16],ns0,o[16]);
mux2 m16(a[14],a[15],ns0,o[15]);
mux2 m1(a[13],a[14],ns0,o[14]);
mux2 m2(a[12],a[13],ns0,o[13]);
mux2 m3(a[11],a[12],ns0,o[12]);
mux2 m4(a[10],a[11],ns0,o[11]);
mux2 m5(a[9],a[10],ns0,o[10]);
mux2 m6(a[8],a[9],ns0,o[9]);
mux2 m7(a[7],a[8],ns0,o[8]);
mux2 m8(a[6],a[7],ns0,o[7]);
mux2 m9(a[5],a[6],ns0,o[6]);
mux2 m10(a[4],a[5],ns0,o[5]);
mux2 m11(a[3],a[4],ns0,o[4]);
mux2 m12(a[2],a[3],ns0,o[3]);
mux2 m13(a[1],a[2],ns0,o[2]);
mux2 m14(a[0],a[1],ns0,o[1]);
mux2 m15(0,a[0],ns0,o[0]);



//mux2 m33(0[30],a[32],ns0,o[32]);
mux2 M32(o[29],a[31],ns1,o1[31]);
mux2 M31(o[28],a[30],ns1,o1[30]);
mux2 M30(o[27],a[29],ns1,o1[29]);
mux2 M29(o[26],a[28],ns1,o1[28]);
mux2 M28(o[25],a[27],ns1,o1[27]);
mux2 M27(o[24],a[26],ns1,o1[26]);
mux2 M26(o[23],a[25],ns1,o1[25]);
mux2 M25(o[22],a[24],ns1,o1[24]);
mux2 M24(o[21],a[23],ns1,o1[23]);
mux2 M23(o[20],a[22],ns1,o1[22]);
mux2 M22(o[19],a[21],ns1,o1[21]);
mux2 M21(o[18],a[20],ns1,o1[20]);
mux2 M20(o[17],a[19],ns1,o1[19]);
mux2 M19(o[16],a[18],ns1,o1[18]);
mux2 M18(o[15],a[17],ns1,o1[17]);
mux2 M17(o[14],a[16],ns1,o1[16]);
mux2 M16(o[13],a[15],ns1,o1[15]);
mux2 M1(o[12],a[14],ns1,o1[14]);
mux2 M2(o[11],a[13],ns1,o1[13]);
mux2 M3(o[10],a[12],ns1,o1[12]);
mux2 M4(o[9],a[11],ns1,o1[11]);
mux2 M5(o[8],a[10],ns1,o1[10]);
mux2 M6(o[7],a[9],ns1,o1[9]);
mux2 M7(o[6],a[8],ns1,o1[8]);
mux2 M8(o[5],a[7],ns1,o1[7]);
mux2 M9(o[4],a[6],ns1,o1[6]);
mux2 M10(o[3],a[5],ns1,o1[5]);
mux2 M11(o[2],a[4],ns1,o1[4]);
mux2 M12(o[1],a[3],ns1,o1[3]);
mux2 M13(o[0],a[2],ns1,o1[2]);
mux2 M14(0,a[1],ns1,o1[1]);
mux2 M15(0,a[0],ns1,o1[0]);


mux2 R32(o1[27],o1[31],ns2,o2[31]);
mux2 R31(o1[26],o1[30],ns2,o2[30]);
mux2 R30(o1[25],o1[29],ns2,o2[29]);
mux2 R29(o1[24],o1[28],ns2,o2[28]);
mux2 R28(o1[23],o1[27],ns2,o2[27]);
mux2 R27(o1[22],o1[26],ns2,o2[26]);
mux2 R26(o1[21],o1[25],ns2,o2[25]);
mux2 R25(o1[20],o1[24],ns2,o2[24]);
mux2 R24(o1[19],o1[23],ns2,o2[23]);
mux2 R23(o1[18],o1[22],ns2,o2[22]);
mux2 R22(o1[17],o1[21],ns2,o2[21]);
mux2 R21(o1[16],o1[20],ns2,o2[20]);
mux2 R20(o1[15],o1[19],ns2,o2[19]);
mux2 R19(o1[14],o1[18],ns2,o2[18]);
mux2 R18(o1[13],o1[17],ns2,o2[17]);
mux2 R17(o1[12],o1[16],ns2,o2[16]);
mux2 R16(o1[11],o1[15],ns2,o2[15]);
mux2 R1(o1[10],o1[14],ns2,o2[14]);
mux2 R2(o1[9],o1[13],ns2,o2[13]);
mux2 R3(o1[8],o1[12],ns2,o2[12]);
mux2 R4(o1[7],o1[11],ns2,o2[11]);
mux2 R5(o1[6],o1[10],ns2,o2[10]);
mux2 R6(o1[5],o1[9],ns2,o2[9]);
mux2 R7(o1[4],o1[8],ns2,o2[8]);
mux2 R8(o1[3],o1[7],ns2,o2[7]);
mux2 R9(o1[2],o1[6],ns2,o2[6]);
mux2 R10(o1[1],o1[5],ns2,o2[5]);
mux2 R11(o1[0],o1[4],ns2,o2[4]);
mux2 R12(0,o1[3],ns2,o2[3]);
mux2 R13(0,o1[2],ns2,o2[2]);
mux2 R14(0,o1[1],ns2,o2[1]);
mux2 R15(0,o1[0],ns2,o2[0]);



mux2 S32(o2[23],o2[31],ns3,o3[31]);
mux2 S31(o2[22],o2[30],ns3,o3[30]);
mux2 S30(o2[21],o2[29],ns3,o3[29]);
mux2 S29(o2[20],o2[28],ns3,o3[28]);
mux2 S28(o2[19],o2[27],ns3,o3[27]);
mux2 S27(o2[18],o2[26],ns3,o3[26]);
mux2 S26(o2[17],o2[25],ns3,o3[25]);
mux2 S25(o2[16],o2[24],ns3,o3[24]);
mux2 S24(o2[15],o2[23],ns3,o3[23]);
mux2 S23(o2[14],o2[22],ns3,o3[22]);
mux2 S22(o2[13],o2[21],ns3,o3[21]);
mux2 S21(o2[12],o2[20],ns3,o3[20]);
mux2 S20(o2[11],o2[19],ns3,o3[19]);
mux2 S19(o2[10],o2[18],ns3,o3[18]);
mux2 S18(o2[9],o2[17],ns3,o3[17]);
mux2 S17(o2[8],o2[16],ns3,o3[16]);
mux2 S16(o2[7],o2[15],ns3,o3[15]);
mux2 S1(o2[6],o2[14],ns3,o3[14]);
mux2 S2(o2[5],o2[13],ns3,o3[13]);
mux2 S3(o2[4],o2[12],ns3,o3[12]);
mux2 S4(o2[3],o2[11],ns3,o3[11]);
mux2 S5(o2[2],o2[10],ns3,o3[10]);
mux2 S6(o2[1],o2[9],ns3,o3[9]);
mux2 S7(o2[0],o2[8],ns3,o3[8]);
mux2 S8(0,o2[7],ns3,o3[7]);
mux2 S9(0,o2[6],ns3,o3[6]);
mux2 S10(0,o2[5],ns3,o3[5]);
mux2 S11(0,o2[4],ns3,o3[4]);
mux2 S12(0,o2[3],ns3,o3[3]);
mux2 S13(0,o2[2],ns3,o3[2]);
mux2 S14(0,o2[1],ns3,o3[1]);
mux2 S15(0,o2[0],ns3,o3[0]);


mux2 L32(o3[15],o3[31],ns4,o4[31]);
mux2 L31(o3[14],o3[30],ns4,o4[30]);
mux2 L30(o3[13],o3[29],ns4,o4[29]);
mux2 L29(o3[12],o3[28],ns4,o4[28]);
mux2 L28(o3[11],o3[27],ns4,o4[27]);
mux2 L27(o3[10],o3[26],ns4,o4[26]);
mux2 L26(o3[9],o3[25],ns4,o4[25]);
mux2 L25(o3[8],o3[24],ns4,o4[24]);
mux2 L24(o3[7],o3[23],ns4,o4[23]);
mux2 L23(o3[6],o3[22],ns4,o4[22]);
mux2 L22(o3[5],o3[21],ns4,o4[21]);
mux2 L21(o3[4],o3[20],ns4,o4[20]);
mux2 L20(o3[3],o3[19],ns4,o4[19]);
mux2 L19(o3[2],o3[18],ns4,o4[18]);
mux2 L18(o3[1],o3[17],ns4,o4[17]);
mux2 L17(o3[0],o3[16],ns4,o4[16]);
mux2 L16(0,o3[15],ns4,o4[15]);
mux2 L1(0,o3[14],ns4,o4[14]);
mux2 L2(0,o3[13],ns4,o4[13]);
mux2 L3(0,o3[12],ns4,o4[12]);
mux2 L4(0,o3[11],ns4,o4[11]);
mux2 L5(0,o3[10],ns4,o4[10]);
mux2 L6(0,o3[9],ns4,o4[9]);
mux2 L7(0,o3[8],ns4,o4[8]);
mux2 L8(0,o3[7],ns4,o4[7]);
mux2 L9(0,o3[6],ns4,o4[6]);
mux2 L10(0,o3[5],ns4,o4[5]);
mux2 L11(0,o3[4],ns4,o4[4]);
mux2 L12(0,o3[3],ns4,o4[3]);
mux2 L13(0,o3[2],ns4,o4[2]);
mux2 L14(0,o3[1],ns4,o4[1]);
mux2 L15(0,o3[0],ns4,o4[0]);

endmodule


module mux2(a,b,s,r);
input a,b,s;
output r;
wire w1,w2,w3;
assign w1=~s;
assign w2=w1&a;
assign w3=b&s;
assign r=w2|w3;
endmodule


//`include "mux2.v"
module shifterright32(a,s,q,clk);
input [31:0]a;
input [4:0]s;
input clk;
output [31:0]q;
wire [37:0]w;
wire [31:0]g;
wire [31:0]m,p;
wire [31:0]b;
assign w[1]=~s[0];
assign w[2]=~s[1];
assign w[3]=~s[2];
assign w[4]=~s[3];
assign w[5]=~s[4];
assign w[0]=0;	
	mux2	f0(w[0],a[31],w[1],w[6]);
	mux2 	f1(a[31],a[30],w[1],w[7]);
	mux2	f2(a[30],a[29],w[1],w[8]);
	mux2 	f3(a[29],a[28],w[1],w[9]);
	mux2	f4(a[28],a[27],w[1],w[10]);
	mux2 	f5(a[27],a[26],w[1],w[11]);
	mux2	f6(a[26],a[25],w[1],w[12]);
	mux2 	f7(a[25],a[24],w[1],w[13]);
	mux2	f8(a[24],a[23],w[1],w[14]);
	mux2 	f9(a[23],a[22],w[1],w[15]);
	mux2	f10(a[22],a[21],w[1],w[16]);
	mux2 	f11(a[21],a[20],w[1],w[17]);
	mux2	f12(a[20],a[19],w[1],w[18]);
	mux2 	f13(a[19],a[18],w[1],w[19]);
	mux2	f14(a[18],a[17],w[1],w[20]);
	mux2 	f15(a[17],a[16],w[1],w[21]);
	mux2	f16(w[16],a[15],w[1],w[22]);
	mux2 	f17(a[15],a[14],w[1],w[23]);
	mux2	f18(a[14],a[13],w[1],w[24]);
	mux2 	f19(a[13],a[12],w[1],w[25]);
	mux2	f20(a[12],a[11],w[1],w[26]);
	mux2 	f21(a[11],a[10],w[1],w[27]);
	mux2	f22(a[10],a[9],w[1],w[28]);
	mux2 	f23(a[9],a[8],w[1],w[29]);
	mux2	f24(a[8],a[7],w[1],w[30]);
	mux2 	f25(a[7],a[6],w[1],w[31]);
	mux2	f26(a[6],a[5],w[1],w[32]);
	mux2 	f27(a[5],a[4],w[1],w[33]);
	mux2	f28(a[4],a[3],w[1],w[34]);
	mux2 	f29(a[3],a[2],w[1],w[35]);
	mux2	f30(a[2],a[1],w[1],w[36]);
	mux2 	f31(a[1],a[0],w[1],w[37]);


	mux2	p0(w[0],w[6],w[2],g[0]);
	mux2	p1(w[0],w[7],w[2],g[1]);
	mux2	p2(w[6],w[8],w[2],g[2]);
	mux2	p3(w[7],w[9],w[2],g[3]);
	mux2	p4(w[8],w[10],w[2],g[4]);
	mux2	p5(w[9],w[11],w[2],g[5]);
	mux2	p6(w[10],w[12],w[2],g[6]);
	mux2	p7(w[11],w[13],w[2],g[7]);
	mux2	p8(w[12],w[14],w[2],g[8]);
	mux2	p9(w[13],w[15],w[2],g[9]);
	mux2	p10(w[14],w[16],w[2],g[10]);
	mux2	p11(w[15],w[17],w[2],g[11]);
	mux2	p12(w[16],w[18],w[2],g[12]);
	mux2	p13(w[17],w[19],w[2],g[13]);
	mux2	p14(w[18],w[20],w[2],g[14]);
	mux2	p15(w[19],w[21],w[2],g[15]);
	mux2	p00(w[20],w[22],w[2],g[16]);
	mux2	p01(w[21],w[23],w[2],g[17]);
	mux2	p02(w[22],w[24],w[2],g[18]);
	mux2	p03(w[23],w[25],w[2],g[19]);
	mux2	p04(w[24],w[26],w[2],g[20]);
	mux2	p05(w[25],w[27],w[2],g[21]);
	mux2	p06(w[26],w[28],w[2],g[22]);
	mux2	p07(w[27],w[29],w[2],g[23]);
	mux2	p08(w[28],w[30],w[2],g[24]);
	mux2	p09(w[29],w[31],w[2],g[25]);
	mux2	p010(w[30],w[32],w[2],g[26]);
	mux2	p011(w[31],w[33],w[2],g[27]);
	mux2	p012(w[32],w[34],w[2],g[28]);
	mux2	p013(w[33],w[35],w[2],g[29]);
	mux2	p014(w[34],w[36],w[2],g[30]);
	mux2	p015(w[35],w[37],w[2],g[31]);

	mux2	k0(w[0],g[0],w[3],m[0]);
	mux2	k1(w[0],g[1],w[3],m[1]);
	mux2	k2(w[0],g[2],w[3],m[2]);
	mux2	k3(w[0],g[3],w[3],m[3]);
	mux2	k4(g[0],g[4],w[3],m[4]);
	mux2	k5(g[1],g[5],w[3],m[5]);
	mux2	k6(g[2],g[6],w[3],m[6]);
	mux2	k7(g[3],g[7],w[3],m[7]);
	mux2	k8(g[4],g[8],w[3],m[8]);
	mux2	k9(g[5],g[9],w[3],m[9]);
	mux2	k10(g[6],g[10],w[3],m[10]);
	mux2	k11(g[7],g[11],w[3],m[11]);
	mux2	k12(g[8],g[12],w[3],m[12]);
	mux2	k13(g[9],g[13],w[3],m[13]);
	mux2	k14(g[10],g[14],w[3],m[14]);
	mux2	k15(g[11],g[15],w[3],m[15]);
	mux2	k00(g[12],g[16],w[3],m[16]);
	mux2	k01(g[13],g[17],w[3],m[17]);
	mux2	k02(g[14],g[18],w[3],m[18]);
	mux2	k03(g[15],g[19],w[3],m[19]);
	mux2	k04(g[16],g[20],w[3],m[20]);
	mux2	k05(g[17],g[21],w[3],m[21]);
	mux2	k06(g[18],g[22],w[3],m[22]);
	mux2	k07(g[19],g[23],w[3],m[23]);
	mux2	k08(g[20],g[24],w[3],m[24]);
	mux2	k09(g[21],g[25],w[3],m[25]);
	mux2	k010(g[22],g[26],w[3],m[26]);
	mux2	k011(g[23],g[27],w[3],m[27]);
	mux2	k012(g[24],g[28],w[3],m[28]);
	mux2	k013(g[25],g[29],w[3],m[29]);
	mux2	k014(g[26],g[30],w[3],m[30]);
	mux2	k015(g[27],g[31],w[3],m[31]);
	

	mux2	b0(w[0],m[0],w[4],p[0]);
	mux2	b1(w[0],m[1],w[4],p[1]);
	mux2	b2(w[0],m[2],w[4],p[2]);
	mux2	b3(w[0],m[3],w[4],p[3]);
	mux2	b4(w[0],m[4],w[4],p[4]);
	mux2	b5(w[0],m[5],w[4],p[5]);
	mux2	b6(w[0],m[6],w[4],p[6]);
	mux2	b7(w[0],m[7],w[4],p[7]);
	mux2	b8(m[0],m[8],w[4],p[8]);
	mux2	b9(m[1],m[9],w[4],p[9]);
	mux2	b10(m[2],m[10],w[4],p[10]);
	mux2	b11(m[3],m[11],w[4],p[11]);
	mux2	b12(m[4],m[12],w[4],p[12]);
	mux2	b13(m[5],m[13],w[4],p[13]);
	mux2	b14(m[6],m[14],w[4],p[14]);
	mux2	b15(m[7],m[15],w[4],p[15]);
	mux2	b00(m[8],m[16],w[4],p[16]);
	mux2	b01(m[9],m[17],w[4],p[17]);
	mux2	b02(m[10],m[18],w[4],p[18]);
	mux2	b03(m[11],m[19],w[4],p[19]);
	mux2	b04(m[12],m[20],w[4],p[20]);
	mux2	b05(m[13],m[21],w[4],p[21]);
	mux2	b06(m[14],m[22],w[4],p[22]);
	mux2	b07(m[15],m[23],w[4],p[23]);
	mux2	b08(m[16],m[24],w[4],p[24]);
	mux2	b09(m[17],m[25],w[4],p[25]);
	mux2	b010(m[18],m[26],w[4],p[26]);
	mux2	b011(m[19],m[27],w[4],p[27]);
	mux2	b012(m[20],m[28],w[4],p[28]);
	mux2	b013(m[21],m[29],w[4],p[29]);
	mux2	b014(m[22],m[30],w[4],p[30]);
	mux2	b015(m[23],m[31],w[4],p[31]);

	mux2	m0(w[0],p[0],w[5],b[31]);
	mux2	m1(w[0],p[1],w[5],b[30]);
	mux2	m2(w[0],p[2],w[5],b[29]);
	mux2	m3(w[0],p[3],w[5],b[28]);
	mux2	m4(w[0],p[4],w[5],b[27]);
	mux2	m5(w[0],p[5],w[5],b[26]);
	mux2	m6(w[0],p[6],w[5],b[25]);
	mux2	m7(w[0],p[7],w[5],b[24]);
	mux2	m8(w[0],p[8],w[5],b[23]);
	mux2	m9(w[0],p[9],w[5],b[22]);
	mux2	m10(w[0],p[10],w[5],b[21]);
	mux2	m11(w[0],p[11],w[5],b[20]);
	mux2	m12(w[0],p[12],w[5],b[19]);
	mux2	m13(w[0],p[13],w[5],b[18]);
	mux2	m14(w[0],p[14],w[5],b[17]);
	mux2	m15(w[0],p[15],w[5],b[16]);
	mux2	m00(m[0],p[16],w[5],b[15]);
	mux2	m01(m[1],p[17],w[5],b[14]);
	mux2	m02(m[2],p[18],w[5],b[13]);
	mux2	m03(m[3],p[19],w[5],b[12]);
	mux2	m04(m[4],p[20],w[5],b[11]);
	mux2	m05(m[5],p[21],w[5],b[10]);
	mux2	m06(m[6],p[22],w[5],b[9]);
	mux2	m07(m[7],p[23],w[5],b[8]);
	mux2	m08(m[8],p[24],w[5],b[7]);
	mux2	m09(m[9],p[25],w[5],b[6]);
	mux2	m010(m[10],p[26],w[5],b[5]);
	mux2	m011(m[11],p[27],w[5],b[4]);
	mux2	m012(m[12],p[28],w[5],b[3]);
	mux2	m013(m[13],p[29],w[5],b[2]);
	mux2	m014(m[14],p[30],w[5],b[1]);
	mux2	m015(m[15],p[31],w[5],b[0]);
shiftright_dff s0(b,clk,q);
endmodule

module shiftright_dff ( a, clk, q);
   input [31:0]a;
   input clk;
   output reg [31:0]q;
   wire clk;
   always @ (posedge clk)
   begin
    q <= a;
 end
endmodule




module cla321(cout,sum,a,b,cin);
input [31:0]a,b;
input cin;
           output [31:0]sum;
output cout;
	   
           wire [31:0]p,g;
	 
	wire [31:0]c;
	//wire [31:0]w;
              
          assign p=a^b;
          assign g=a&b;

assign c[0]=cin;
assign c[1]=g[0]|(p[0]&cin);
assign c[2]=g[1]|(p[1]&(g[0]|(p[0]&cin)));
assign c[3]=g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))));
assign c[4]=g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))));
assign c[5]=g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))));

assign c[6]=g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))));

assign c[7]=g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))));

assign c[8]=g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))));

assign c[9]=g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))));

assign c[10]=g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))));

assign c[11]=g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))));

assign c[12]=g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))));

assign c[13]=g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))));

assign c[14]=g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))));

assign c[15]=g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))))));

assign c[16]=g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))))))));

assign c[17]=g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))))))))));

assign c[18]=g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))))))))))));

assign c[19]=g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))))))))))))));

assign c[20]=g[19]|(p[19]&(g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))))))))))))))));

assign c[21]=g[20]|(p[20]&(g[19]|(p[19]&(g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))))))))))))))))));

assign c[22]=g[21]|(p[21]&(g[20]|(p[20]&(g[19]|(p[19]&(g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))))))))))))))))))));

assign c[23]=g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(p[20]&(g[19]|(p[19]&(g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))))))))))))))))))))));

assign c[24]=g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(p[20]&(g[19]|(p[19]&(g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))))))))))))))))))))))));

assign c[25]=g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(p[20]&(g[19]|(p[19]&(g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))))))))))))))))))))))))));

assign c[26]=g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(p[20]&(g[19]|(p[19]&(g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[27]=g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(p[20]&(g[19]|(p[19]&(g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[28]=g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(p[20]&(g[19]|(p[19]&(g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[29]=g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(p[20]&(g[19]|(p[19]&(g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[30]=g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(p[20]&(g[19]|(p[19]&(g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[31]=g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(p[20]&(g[19]|(p[19]&(g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));





//assign temp1=g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(p[20]&(g[19]|(p[19]&(g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));
	  
          assign sum=a^b^c;
          assign cout=c[31];
	  

endmodule

////fp_mult.v

module fp_mult(sa,sb,ea,eb,ma,mb,ma_out,signbit,exponent,clk);
input [22:0]ma,mb;
input sa,sb;
input [7:0]ea,eb;

output reg [23:0]ma_out;
 reg result;
output signbit;
input clk;
output [7:0]exponent;
wire [31:0]ma1,mb1;
wire [63:0]R;
wire [8:0]exp1;
wire [8:0]ea1,eb1,exp;
wire [8:0]subt;
wire cout,cout1;
assign ma1[22:0]=ma[22:0];
assign ma1[23]=1'b1;
assign ma1[31:24]=8'b0;

assign mb1[22:0]=mb[22:0];
assign mb1[23]=1'b1;
assign mb1[31:24]=8'b0;

wallactreemult1 maaaa(ma1,mb1,R,carry,clk);

always@(*)
begin
if(R[46]==1'b1)
begin
	ma_out[23:0]<=R[47:23];
	result<=R[47];
	
end
else if(R[47]==1'b1)
begin
	ma_out[23:0]<=R[48:24];
	//ma_out[23]<=R[47];
	
end
end

assign signbit=sa^sb;

assign ea1[7:0]=ea[7:0];
assign ea1[8]=1'b0;

assign eb1[7:0]=eb[7:0];
assign eb1[8]=1'b0;

cla9 chjkl(cout,exp,ea1,eb1,0);


assign subt=9'd127;

cla9 cabg(cout1,exp1,exp,~subt,1);

assign exponent[7:0]=exp1[7:0];

endmodule

module cla9(cout,s,a,b,cin);

input [8:0]a,b;
input cin;

output [8:0]s;
output cout;
wire [8:0]g,p,c;
assign g=a&b;
assign p=a^b;
assign c[0]=g[0]|(p[0]&cin);
assign c[1]=g[1]|(p[1]&(g[0]|(p[0]&cin)));
assign c[2]=g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))));
assign c[3]=g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))));
assign c[4]=g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))));

assign c[5]=g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))));

assign c[6]=g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))));

assign c[7]=g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))));

assign c[8]=g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))));

assign cout=c[8];

assign s[0]=p[0]^cin;
assign s[1]=p[1]^c[0];
assign s[2]=p[2]^c[1];
assign s[3]=p[3]^c[2];
assign s[4]=p[4]^c[3];
assign s[5]=p[5]^c[4];
assign s[6]=p[6]^c[5];
assign s[7]=p[7]^c[6];
assign s[8]=p[8]^c[7];


endmodule

//`include "csa.v"
//`include "cla3299.v"
//`include "ff.v"
module wallactreemult1(A,B,R,carry,clk);
input [31:0]A,B;
input clk;
wire [63:0]U[63:0];
wire [63:0]V[63:0];
output [63:0]R;
output carry;
wire [63:0]P[63:0];
wire [63:0]q[63:0];
wire [63:0]q1[63:0];
wire [63:0]p1[63:0];


assign P[0]=B[0]?{32'b000,A}:64'h0000;
assign P[1]=B[1]?{32'b000,A}<<1:64'h0000;
assign P[2]=B[2]?{32'b000,A}<<2:64'h0000;
assign P[3]=B[3]?{32'b000,A}<<3:64'h0000;
assign P[4]=B[4]?{32'b000,A}<<4:64'h0000;
assign P[5]=B[5]?{32'b000,A}<<5:64'h0000;
assign P[6]=B[6]?{32'b000,A}<<6:64'h0000;
assign P[7]=B[7]?{32'b000,A}<<7:64'h0000;

assign P[8]=B[8]?{32'b000,A}<<8:64'h0000;
assign P[9]=B[9]?{32'b000,A}<<9:64'h0000;
assign P[10]=B[10]?{32'b000,A}<<10:64'h0000;
assign P[11]=B[11]?{32'b000,A}<<11:64'h0000;
assign P[12]=B[12]?{32'b000,A}<<12:64'h0000;
assign P[13]=B[13]?{32'b000,A}<<13:64'h0000;
assign P[14]=B[14]?{32'b000,A}<<14:64'h0000;
assign P[15]=B[15]?{32'b000,A}<<15:64'h0000;

assign P[16]=B[16]?{32'b000,A}<<16:64'h0000;
assign P[17]=B[17]?{32'b000,A}<<17:64'h0000;
assign P[18]=B[18]?{32'b000,A}<<18:64'h0000;
assign P[19]=B[19]?{32'b000,A}<<19:64'h0000;
assign P[20]=B[20]?{32'b000,A}<<20:64'h0000;
assign P[21]=B[21]?{32'b000,A}<<21:64'h0000;
assign P[22]=B[22]?{32'b000,A}<<22:64'h0000;
assign P[23]=B[23]?{32'b000,A}<<23:64'h0000;

assign P[24]=B[24]?{32'b000,A}<<24:64'h0000;
assign P[25]=B[25]?{32'b000,A}<<25:64'h0000;
assign P[26]=B[26]?{32'b000,A}<<26:64'h0000;
assign P[27]=B[27]?{32'b000,A}<<27:64'h0000;
assign P[28]=B[28]?{32'b000,A}<<28:64'h0000;
assign P[29]=B[29]?{32'b000,A}<<29:64'h0000;
assign P[30]=B[30]?{32'b000,A}<<30:64'h0000;
assign P[31]=B[31]?{32'b000,A}<<31:64'h0000;

//1ssst
csaveadder11 c1(P[0],P[1],P[2],U[0],V[0]);
d_ff55 f0 ( U[0],V[0], clk, q[0],q1[0]);

csaveadder11 c2(P[3],P[4],P[5],U[1],V[1]);
d_ff55 f1 ( U[1],V[1], clk, q[1],q1[1]);

csaveadder11 c3(P[6],P[7],P[8],U[2],V[2]);
d_ff55 f2 ( U[2],V[2], clk, q[2],q1[2]);

csaveadder11 c4(P[9],P[10],P[11],U[3],V[3]);
d_ff55 f3 ( U[3],V[3], clk, q[3],q1[3]);

csaveadder11 c5(P[12],P[13],P[14],U[4],V[4]);
d_ff55 f4 ( U[4],V[4], clk, q[4],q1[4]);

csaveadder11 c6(P[15],P[16],P[17],U[5],V[5]);
d_ff55 f5 ( U[5],V[5], clk, q[5],q1[5]);

csaveadder11 c7(P[18],P[19],P[20],U[6],V[6]);
d_ff55 f6 ( U[6],V[6], clk, q[6],q1[6]);

csaveadder11 c8(P[21],P[22],P[23],U[7],V[7]);
d_ff55 f7 ( U[7],V[7], clk, q[7],q1[7]);

csaveadder11 c9(P[24],P[25],P[26],U[8],V[8]);
d_ff55 f8 ( U[8],V[8], clk, q[8],q1[8]);

csaveadder11 c10(P[27],P[28],P[29],U[9],V[9]);
d_ff55 f9 ( U[9],V[9], clk, q[9],q1[9]);

d_ff55 f10( P[30],P[31],clk,p1[1],p1[2]);
//

//2ndd
csaveadder11 c11(q[0],q1[0],q[1],U[10],V[10]);
d_ff55 f11 ( U[10],V[10], clk, q[10],q1[10]);
 
csaveadder11 c12(q1[1],q[2],q1[2],U[11],V[11]);
d_ff55 f12 ( U[11],V[11], clk, q[11],q1[11]);

csaveadder11 c13(q[3],q1[3],q[4],U[12],V[12]);
d_ff55 f13 ( U[12],V[12], clk, q[12],q1[12]);

csaveadder11 c14(q1[4],q[5],q1[5],U[13],V[13]);
d_ff55 f14 ( U[13],V[13], clk, q[13],q1[13]);

csaveadder11 c15(q[6],q1[6],q[7],U[14],V[14]);
d_ff55 f15 ( U[14],V[14], clk, q[14],q1[14]);

csaveadder11 c16(q1[7],q[8],q1[8],U[15],V[15]);
d_ff55 f16 ( U[15],V[15], clk, q[15],q1[15]);

csaveadder11 c17(q[9],q1[9],p1[1],U[16],V[16]);
d_ff55 f17 ( U[16],V[16], clk, q[16],q1[16]);

d_ff55 f18 ( p1[2],p1[2],clk,p1[3],p1[4]);//p1[3]=p1[4]
//


//3rdd
csaveadder11 c18(q[10],q1[10],q[11],U[17],V[17]);
d_ff55 f19 ( U[17],V[17], clk, q[17],q1[17]);

csaveadder11 c19(q1[11],q[12],q1[12],U[18],V[18]);
d_ff55 f20 ( U[18],V[18], clk, q[18],q1[18]);

csaveadder11 c20(q[13],q1[13],q[14],U[19],V[19]);
d_ff55 f21 ( U[19],V[19], clk, q[19],q1[19]);

csaveadder11 c21(q1[14],q[15],q1[15],U[20],V[20]);
d_ff55 f22 ( U[20],V[20], clk, q[20],q1[20]);

csaveadder11 c22(q[16],q1[16],p1[4],U[21],V[21]);//p1[3]=p[31]
d_ff55 f23 ( U[21],V[21], clk, q[21],q1[21]);

d_ff55 f24 ( V[21],V[21], clk, p1[5],p1[6]);
//


//4th
csaveadder11 c23(q[17],q1[17],q[18],U[22],V[22]);
d_ff55 f25 ( U[22],V[22], clk, q[22],q1[22]);

csaveadder11 c24(q1[18],q[19],q1[19],U[23],V[23]);
d_ff55 f26 ( U[23],V[23], clk, q[23],q1[23]);

csaveadder11 c25(q[20],q1[20],q[21],U[24],V[24]);
d_ff55 f27 ( U[24],V[24], clk, q[24],q1[24]);

d_ff55 f28 ( p1[5],p1[6], clk, p1[7],p1[8]);
//


//5thhh

csaveadder11 c26(q[22],q1[22],q[23],U[25],V[25]);
d_ff55 f29 ( U[25],V[25], clk, q[25],q1[25]);

csaveadder11 c27(q1[23],q[24],q1[24],U[26],V[26]);
d_ff55 f30 ( U[26],V[26], clk, q[26],q1[26]);

d_ff55 f31 ( p1[7],p1[8], clk, p1[9],p1[10]);
//


//6thh
csaveadder11 c28(q[25],q1[25],q[26],U[27],V[27]);
d_ff55 f32 ( U[27],V[27], clk, q[27],q1[27]);

d_ff55 f33 ( p1[9],p1[10], clk, p1[11],p1[12]);
//


//7thh
csaveadder11 c29(q1[26],q[27],q1[27],U[28],V[28]);
d_ff55 f34 ( U[28],V[28], clk, q[28],q1[28]);

d_ff55 f35 ( p1[11],p1[12], clk, p1[13],p1[14]);

//


//8thh
csaveadder11 c30(q[28],q1[28],p1[13],U[29],V[29]);
d_ff55 f36 ( U[29],V[29], clk, q[29],q1[29]);

//
//rca_64bit q0(q[29],q1[29],0,R,carry);
cla3299 c000(carry,R,q[29],q1[29],0);
//cla3299(cout,s,a,b,cin);
endmodule


module fulladder222(a,b,cin,s,cout);

input a,b,cin;
output s,cout;

assign s=a^b^cin;
assign cout=(a&b)|(b&cin)|(a&cin);

endmodule


//`include "fa.v"
module csaveadder11(X,Y,Z,U,V);
input [63:0]X;
input [63:0]Y;
input [63:0]Z;
output [63:0]U;
output [63:0]V;
wire temp;

assign V[0]=0;

fulladder222 fa0(X[0],Y[0],Z[0],U[0],V[1]);
fulladder222 fa1(X[1],Y[1],Z[1],U[1],V[2]);
fulladder222 fa2(X[2],Y[2],Z[2],U[2],V[3]);
fulladder222 fa3(X[3],Y[3],Z[3],U[3],V[4]);
fulladder222 fa4(X[4],Y[4],Z[4],U[4],V[5]);
fulladder222 fa5(X[5],Y[5],Z[5],U[5],V[6]);
fulladder222 fa6(X[6],Y[6],Z[6],U[6],V[7]);
fulladder222 fa7(X[7],Y[7],Z[7],U[7],V[8]);
fulladder222 fa8(X[8],Y[8],Z[8],U[8],V[9]);
fulladder222 fa9(X[9],Y[9],Z[9],U[9],V[10]);
fulladder222 fa10(X[10],Y[10],Z[10],U[10],V[11]);
fulladder222 fa11(X[11],Y[11],Z[11],U[11],V[12]);
fulladder222 fa12(X[12],Y[12],Z[12],U[12],V[13]);
fulladder222 fa13(X[13],Y[13],Z[13],U[13],V[14]);
fulladder222 fa14(X[14],Y[14],Z[14],U[14],V[15]);
fulladder222 fa15(X[15],Y[15],Z[15],U[15],V[16]);
fulladder222 fa16(X[16],Y[16],Z[16],U[16],V[17]);
fulladder222 fa17(X[17],Y[17],Z[17],U[17],V[18]);
fulladder222 fa18(X[18],Y[18],Z[18],U[18],V[19]);
fulladder222 fa19(X[19],Y[19],Z[19],U[19],V[20]);
fulladder222 fa20(X[20],Y[20],Z[20],U[20],V[21]);
fulladder222 fa21(X[21],Y[21],Z[21],U[21],V[22]);
fulladder222 fa22(X[22],Y[22],Z[22],U[22],V[23]);
fulladder222 fa23(X[23],Y[23],Z[23],U[23],V[24]);
fulladder222 fa24(X[24],Y[24],Z[24],U[24],V[25]);
fulladder222 fa25(X[25],Y[25],Z[25],U[25],V[26]);
fulladder222 fa26(X[26],Y[26],Z[26],U[26],V[27]);
fulladder222 fa27(X[27],Y[27],Z[27],U[27],V[28]);
fulladder222 fa28(X[28],Y[28],Z[28],U[28],V[29]);
fulladder222 fa29(X[29],Y[29],Z[29],U[29],V[30]);
fulladder222 fa30(X[30],Y[30],Z[30],U[30],V[31]);
fulladder222 fa31(X[31],Y[31],Z[31],U[31],V[32]);
fulladder222 fa32(X[32],Y[32],Z[32],U[32],V[33]);
fulladder222 fa33(X[33],Y[33],Z[33],U[33],V[34]);
fulladder222 fa34(X[34],Y[34],Z[34],U[34],V[35]);
fulladder222 fa35(X[35],Y[35],Z[35],U[35],V[36]);
fulladder222 fa36(X[36],Y[36],Z[36],U[36],V[37]);
fulladder222 fa37(X[37],Y[37],Z[37],U[37],V[38]);
fulladder222 fa38(X[38],Y[38],Z[38],U[38],V[39]);
fulladder222 fa39(X[39],Y[39],Z[39],U[39],V[40]);
fulladder222 fa40(X[40],Y[40],Z[40],U[40],V[41]);
fulladder222 fa41(X[41],Y[41],Z[41],U[41],V[42]);
fulladder222 fa42(X[42],Y[42],Z[42],U[42],V[43]);
fulladder222 fa43(X[43],Y[43],Z[43],U[43],V[44]);
fulladder222 fa44(X[44],Y[44],Z[44],U[44],V[45]);
fulladder222 fa45(X[45],Y[45],Z[45],U[45],V[46]);
fulladder222 fa46(X[46],Y[46],Z[46],U[46],V[47]);
fulladder222 fa47(X[47],Y[47],Z[47],U[47],V[48]);
fulladder222 fa48(X[48],Y[48],Z[48],U[48],V[49]);
fulladder222 fa49(X[49],Y[49],Z[49],U[49],V[50]);
fulladder222 fa50(X[50],Y[50],Z[50],U[50],V[51]);
fulladder222 fa51(X[51],Y[51],Z[51],U[51],V[52]);
fulladder222 fa52(X[52],Y[52],Z[52],U[52],V[53]);
fulladder222 fa53(X[53],Y[53],Z[53],U[53],V[54]);
fulladder222 fa54(X[54],Y[54],Z[54],U[54],V[55]);
fulladder222 fa55(X[55],Y[55],Z[55],U[55],V[56]);
fulladder222 fa56(X[56],Y[56],Z[56],U[56],V[57]);
fulladder222 fa57(X[57],Y[57],Z[57],U[57],V[58]);
fulladder222 fa58(X[58],Y[58],Z[58],U[58],V[59]);
fulladder222 fa59(X[59],Y[59],Z[59],U[59],V[60]);
fulladder222 fa60(X[60],Y[60],Z[60],U[60],V[61]);
fulladder222 fa61(X[61],Y[61],Z[61],U[61],V[62]);
fulladder222 fa62(X[62],Y[62],Z[62],U[62],V[63]);
fulladder222 fa63(X[63],Y[63],Z[63],U[63],temp);

endmodule

module d_ff55 ( a,b, clk, q,q1);
   input [63:0]a;
   input [63:0]b;
   //input [63:0]c;
   input clk;
   output [63:0]q;
   output  [63:0]q1;	
  // output [63:0]q63;
   wire clk;
   reg [63:0]q;
   reg  [63:0]q1;
   //reg [63:0]q63;
   always @ (posedge clk)
   begin
    q <= a;
    q1 <=b;
    //q63 <= c;
 end

endmodule

//`include "ff.v"
//`include "dff1.v"
module cla3299(cout,s,a,b,cin);
output [63:0]s;
output cout;
input [63:0]a,b;
//input clk;
input cin;
wire [63:0]g,p,c,s;
//wire cin1,cout;
//wire [31:0]q,b,c,q3;

assign g=a&b;
assign p=a^b;

//d_ff55 faa(a[0],b[0],cin,clk,q[0],b[0],cin1);
assign c[0]=g[0]|(p[0]&cin);
//d_ff55 f0(a[0],b[0],c[0],clk,q[0],b[0],c[0]);


assign c[1]=g[1]|(p[1]&(g[0]|(p[0]&cin)));
//d_ff55 f1(a[1],b[1],c[1],clk,q[1],b[1],c[1]);

assign c[2]=g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))));
//d_ff55 f2(a[2],b[2],c[2],clk,q[2],b[2],c[2]);

assign c[3]=g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))));
//d_ff55 f3(a[3],b[3],c[3],clk,q[3],b[3],c[3]);

assign c[4]=g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))));
//d_ff55 f4(a[4],b[4],c[4],clk,q[4],b[4],c[4]);

assign c[5]=g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))));
//d_ff55 f5(a[5],b[5],c[5],clk,q[5],b[5],c[5]);

assign c[6]=g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))));
//d_ff55 f6(a[6],b[6],c[6],clk,q[6],b[6],c[6]);

assign c[7]=g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))));
//d_ff55 f7(a[7],b[7],c[7],clk,q[7],b[7],c[7]);

assign c[8]=g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))));
//d_ff55 f8(a[8],b[8],c[8],clk,q[8],b[8],c[8]);

assign c[9]=g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))));
//d_ff55 f9(a[9],b[9],c[9],clk,q[9],b[9],c[9]);

assign c[10]=g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))));
//d_ff55 f10(a[10],b[10],c[10],clk,q[10],b[10],c[10]);

assign c[11]=g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))));
//d_ff55 f11(a[11],b[11],c[11],clk,q[11],b[11],c[11]);

assign c[12]=g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))));
//d_ff55 f12(a[12],b[12],c[12],clk,q[12],b[12],c[12]);

assign c[13]=g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))));
//d_ff55 f13(a[13],b[13],c[13],clk,q[13],b[13],c[13]);

assign c[14]=g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))))));
//d_ff55 f14(a[14],b[14],c[14],clk,q[14],b[14],c[14]);

assign c[15]=g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))))))));
//d_ff55 f15(a[15],b[15],c[15],clk,q[15],b[15],c[15]);

assign c[16]=g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))))))))));
//d_ff55 f16(a[16],b[16],c[16],clk,q[16],b[16],c[16]);

assign c[17]=g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))))))))))));
//d_ff55 f17(a[17],b[17],c[17],clk,q[17],b[17],c[17]);

assign c[18]=g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin)))))))))))))))))))))))))))))))))))));
//d_ff55 f18(a[18],b[18],c[18],clk,q[18],b[18],c[18]);

assign c[19]=g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))));
//d_ff55 f19(a[19],b[19],c[19],clk,q[19],b[19],c[19]);

assign c[20]=g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))));
//d_ff55 f20(a[20],b[20],c[20],clk,q[20],b[20],c[20]);

assign c[21]=g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))));
//d_ff55 f21(a[21],b[21],c[21],clk,q[21],b[21],c[21]);

assign c[22]=g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))));
//d_ff55 f22(a[22],b[22],c[22],clk,q[22],b[22],c[22]);

assign c[23]=g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))));
//d_ff55 f23(a[23],b[23],c[23],clk,q[23],b[23],c[23]);

assign c[24]=g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))));
//d_ff55 f24(a[24],b[24],c[24],clk,q[24],b[24],c[24]);

assign c[25]=g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))));
//d_ff55 f25(a[25],b[25],c[25],clk,q[25],b[25],c[25]);

assign c[26]=g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p
[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))));
//d_ff55 f26(a[26],b[26],c[26],clk,q[26],b[26],c[26]);

assign c[27]=g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))));
//d_ff55 f27(a[27],b[27],c[27],clk,q[27],b[27],c[27]);

assign c[28]=g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))));
//d_ff55 f28(a[28],b[28],c[28],clk,q[28],b[28],c[28]);

assign c[29]=g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))));
//d_ff55 f29(a[29],b[29],c[29],clk,q[29],b[29],c[29]);

assign c[30]=g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));
//d_ff55 f30(a[30],b[30],c[30],clk,q[30],b[30],c[30]);

assign c[31]=g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));
//d_ff55 f31(a[31],b[31],c[31],clk,q[31],b[31],c[31]);

assign c[32]=g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[33]=g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[34]=g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[35]=g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[36]=g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[37]=g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[38]=g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[39]=g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[40]=g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[41]=g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[42]=g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[43]=g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));


assign c[44]=g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));


assign c[45]=g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));


assign c[46]=g[46]|(p[46]&(g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));


assign c[47]=g[47]|(p[47]&(g[46]|(p[46]&(g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));


assign c[48]=g[48]|(p[48]&(g[47]|(p[47]&(g[46]|(p[46]&(g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));


assign c[49]=g[49]|(p[49]&(g[48]|(p[48]&(g[47]|(p[47]&(g[46]|(p[46]&(g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));


assign c[50]=g[50]|(p[50]&(g[49]|(p[49]&(g[48]|(p[48]&(g[47]|(p[47]&(g[46]|(p[46]&(g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));


assign c[51]=g[51]|(p[51]&(g[50]|(p[50]&(g[49]|(p[49]&(g[48]|(p[48]&(g[47]|(p[47]&(g[46]|(p[46]&(g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));


assign c[52]=g[52]|(p[51]&(g[51]|(p[51]&(g[50]|(p[50]&(g[49]|(p[49]&(g[48]|(p[48]&(g[47]|(p[47]&(g[46]|(p[46]&(g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));


assign c[53]=g[53]|(p[53]&(g[52]|(p[51]&(g[51]|(p[51]&(g[50]|(p[50]&(g[49]|(p[49]&(g[48]|(p[48]&(g[47]|(p[47]&(g[46]|(p[46]&(g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));


assign c[54]=g[54]|(p[54]&(g[53]|(p[53]&(g[52]|(p[51]&(g[51]|(p[51]&(g[50]|(p[50]&(g[49]|(p[49]&(g[48]|(p[48]&(g[47]|(p[47]&(g[46]|(p[46]&(g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));


assign c[55]=g[55]|(p[55]&(g[54]|(p[54]&(g[53]|(p[53]&(g[52]|(p[51]&(g[51]|(p[51]&(g[50]|(p[50]&(g[49]|(p[49]&(g[48]|(p[48]&(g[47]|(p[47]&(g[46]|(p[46]&(g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));


assign c[56]=g[56]|(p[56]&(g[55]|(p[55]&(g[54]|(p[54]&(g[53]|(p[53]&(g[52]|(p[51]&(g[51]|(p[51]&(g[50]|(p[50]&(g[49]|(p[49]&(g[48]|(p[48]&(g[47]|(p[47]&(g[46]|(p[46]&(g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[57]=g[57]|(p[57]&(g[56]|(p[56]&(g[55]|(p[55]&(g[54]|(p[54]&(g[53]|(p[53]&(g[52]|(p[51]&(g[51]|(p[51]&(g[50]|(p[50]&(g[49]|(p[49]&(g[48]|(p[48]&(g[47]|(p[47]&(g[46]|(p[46]&(g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[58]=g[58]|(p[58]&(g[57]|(p[57]&(g[56]|(p[56]&(g[55]|(p[55]&(g[54]|(p[54]&(g[53]|(p[53]&(g[52]|(p[51]&(g[51]|(p[51]&(g[50]|(p[50]&(g[49]|(p[49]&(g[48]|(p[48]&(g[47]|(p[47]&(g[46]|(p[46]&(g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[59]=g[59]|(p[59]&(g[58]|(p[58]&(g[57]|(p[57]&(g[56]|(p[56]&(g[55]|(p[55]&(g[54]|(p[54]&(g[53]|(p[53]&(g[52]|(p[51]&(g[51]|(p[51]&(g[50]|(p[50]&(g[49]|(p[49]&(g[48]|(p[48]&(g[47]|(p[47]&(g[46]|(p[46]&(g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[60]=g[60]|(p[60]&(g[59]|(p[59]&(g[58]|(p[58]&(g[57]|(p[57]&(g[56]|(p[56]&(g[55]|(p[55]&(g[54]|(p[54]&(g[53]|(p[53]&(g[52]|(p[51]&(g[51]|(p[51]&(g[50]|(p[50]&(g[49]|(p[49]&(g[48]|(p[48]&(g[47]|(p[47]&(g[46]|(p[46]&(g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[61]=g[61]|(p[61]&(g[60]|(p[60]&(g[59]|(p[59]&(g[58]|(p[58]&(g[57]|(p[57]&(g[56]|(p[56]&(g[55]|(p[55]&(g[54]|(p[54]&(g[53]|(p[53]&(g[52]|(p[51]&(g[51]|(p[51]&(g[50]|(p[50]&(g[49]|(p[49]&(g[48]|(p[48]&(g[47]|(p[47]&(g[46]|(p[46]&(g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[62]=g[62]|(p[62]&(g[61]|(p[61]&(g[60]|(p[60]&(g[59]|(p[59]&(g[58]|(p[58]&(g[57]|(p[57]&(g[56]|(p[56]&(g[55]|(p[55]&(g[54]|(p[54]&(g[53]|(p[53]&(g[52]|(p[51]&(g[51]|(p[51]&(g[50]|(p[50]&(g[49]|(p[49]&(g[48]|(p[48]&(g[47]|(p[47]&(g[46]|(p[46]&(g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

assign c[63]=g[63]|(p[63]&(g[62]|(p[62]&(g[61]|(p[61]&(g[60]|(p[60]&(g[59]|(p[59]&(g[58]|(p[58]&(g[57]|(p[57]&(g[56]|(p[56]&(g[55]|(p[55]&(g[54]|(p[54]&(g[53]|(p[53]&(g[52]|(p[51]&(g[51]|(p[51]&(g[50]|(p[50]&(g[49]|(p[49]&(g[48]|(p[48]&(g[47]|(p[47]&(g[46]|(p[46]&(g[45]|(p[45]&(g[44]|(p[44]&(g[43]|(p[43]&(g[42]|(p[42]&(g[41]|(p[41]&(g[40]|(p[40]&(g[39]|(p[39]&(g[38]|(p[38]&(g[37]|(p[37]&(g[36]|(p[36]&(g[35]|(p[35]&(g[34]|(p[34]&(g[33]|(p[33]&(g[32]|(p[32]&(g[31]|(p[31]&(g[30]|(p[30]&(g[29]|(p[29]&(g[28]|(p[28]&(g[27]|(p[27]&(g[26]|(p[26]&(g[25]|(p[25]&(g[24]|(p[24]&(g[23]|(p[23]&(g[22]|(p[22]&(g[21]|(p[21]&(g[20]|(g[20]&(g[19]|(p[19]&g[18]|(p[18]&(g[17]|(p[17]&(g[16]|(p[16]&(g[15]|(p[15]&(g[14]|(p[14]&(g[13]|(p[13]&(g[12]|(p[12]&(g[11]|(p[11]&(g[10]|(p[10]&(g[9]|(p[9]&(g[8]|(p[8]&(g[7]|(p[7]&(g[6]|(p[6]&(g[5]|(p[5]&(g[4]|(p[4]&(g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&cin))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));




assign s[0]=a[0]^b[0]^cin;
assign s[1]=a[1]^b[1]^c[0];
assign s[2]=a[2]^b[2]^c[1];
assign s[3]=a[3]^b[3]^c[2];
assign s[4]=a[4]^b[4]^c[3];
assign s[5]=a[5]^b[5]^c[4];
assign s[6]=a[6]^b[6]^c[5];
assign s[7]=a[7]^b[7]^c[6];
assign s[8]=a[8]^b[8]^c[7];
assign s[9]=a[9]^b[9]^c[8];
assign s[10]=a[10]^b[10]^c[9];
assign s[11]=a[11]^b[11]^c[10]; 
assign s[12]=a[12]^b[12]^c[11];
assign s[13]=a[13]^b[13]^c[12];
assign s[14]=a[14]^b[14]^c[13];
assign s[15]=a[15]^b[15]^c[14];
assign s[16]=a[16]^b[16]^c[15];
assign s[17]=a[17]^b[17]^c[16];
assign s[18]=a[18]^b[18]^c[17];
assign s[19]=a[19]^b[19]^c[18];
assign s[20]=a[20]^b[20]^c[19];
assign s[21]=a[21]^b[21]^c[20];
assign s[22]=a[22]^b[22]^c[21];
assign s[23]=a[23]^b[23]^c[22];
assign s[24]=a[24]^b[24]^c[23];
assign s[25]=a[25]^b[25]^c[24];
assign s[26]=a[26]^b[26]^c[25];
assign s[27]=a[27]^b[27]^c[26];
assign s[28]=a[28]^b[28]^c[27];
assign s[29]=a[29]^b[29]^c[28];
assign s[30]=a[30]^b[30]^c[29];
assign s[31]=a[31]^b[31]^c[30];
assign s[32]=a[32]^b[32]^c[31];
assign s[33]=a[33]^b[33]^c[32];
assign s[34]=a[34]^b[34]^c[33];
assign s[35]=a[35]^b[35]^c[34];
assign s[36]=a[36]^b[36]^c[35];
assign s[37]=a[37]^b[37]^c[36];
assign s[38]=a[38]^b[38]^c[37];
assign s[39]=a[39]^b[39]^c[38];
assign s[40]=a[40]^b[40]^c[39];
assign s[41]=a[41]^b[41]^c[40];
assign s[42]=a[42]^b[42]^c[41];
assign s[43]=a[43]^b[43]^c[42];
assign s[44]=a[44]^b[44]^c[43];
assign s[45]=a[45]^b[45]^c[44];
assign s[46]=a[46]^b[46]^c[45];
assign s[47]=a[47]^b[47]^c[46];
assign s[48]=a[48]^b[48]^c[47];
assign s[49]=a[49]^b[49]^c[48];
assign s[50]=a[50]^b[50]^c[49];
assign s[51]=a[51]^b[51]^c[50];
assign s[52]=a[52]^b[52]^c[51];
assign s[53]=a[53]^b[53]^c[52];
assign s[54]=a[54]^b[54]^c[53];
assign s[55]=a[55]^b[55]^c[54];
assign s[56]=a[56]^b[56]^c[55];
assign s[57]=a[57]^b[57]^c[56];
assign s[58]=a[58]^b[58]^c[57];
assign s[59]=a[59]^b[59]^c[58];
assign s[60]=a[60]^b[60]^c[59];
assign s[61]=a[61]^b[61]^c[60];
assign s[62]=a[62]^b[62]^c[61];
assign s[63]=a[63]^b[63]^c[62];

assign cout=c[63];

endmodule


////left barrel.v...............

//`include "mux222.v"

module leftbarrelshift32(a,s0,s1,s2,s3,s4,out);

input [31:0]a;
input s0,s1,s2,s3,s4;
output [31:0]out;
wire [31:0]i1,i2,i3,i4;

mux222 ar1(a[31],a[30],s0,i1[31]);
mux222 ar2(a[30],a[29],s0,i1[30]);
mux222 ar3(a[29],a[28],s0,i1[29]);
mux222 ar4(a[28],a[27],s0,i1[28]);
mux222 ar5(a[27],a[26],s0,i1[27]);
mux222 ar6(a[26],a[25],s0,i1[26]);
mux222 ar7(a[25],a[24],s0,i1[25]);
mux222 ar8(a[24],a[23],s0,i1[24]);
mux222 ar9(a[23],a[22],s0,i1[23]);
mux222 ar10(a[22],a[21],s0,i1[22]);
mux222 ar11(a[21],a[20],s0,i1[21]);
mux222 ar12(a[20],a[19],s0,i1[20]);
mux222 ar13(a[19],a[18],s0,i1[19]);
mux222 ar14(a[18],a[17],s0,i1[18]);
mux222 ar15(a[17],a[16],s0,i1[17]);
mux222 ar16(a[16],a[15],s0,i1[16]);
mux222 ar17(a[15],a[14],s0,i1[15]);
mux222 ar18(a[14],a[13],s0,i1[14]);
mux222 ar19(a[13],a[12],s0,i1[13]);
mux222 ar20(a[12],a[11],s0,i1[12]);
mux222 ar21(a[11],a[10],s0,i1[11]);
mux222 ar22(a[10],a[9],s0,i1[10]);
mux222 ar23(a[9],a[8],s0,i1[9]);
mux222 ar24(a[8],a[7],s0,i1[8]);
mux222 ar25(a[7],a[6],s0,i1[7]);
mux222 ar26(a[6],a[5],s0,i1[6]);
mux222 ar27(a[5],a[4],s0,i1[5]);
mux222 ar28(a[4],a[3],s0,i1[4]);
mux222 ar29(a[3],a[2],s0,i1[3]);
mux222 ar30(a[2],a[1],s0,i1[2]);
mux222 ar31(a[1],a[0],s0,i1[1]);
mux222 ar32(a[0],0,s0,i1[0]);




mux222 br1(i1[31],i1[29],s1,i2[31]);
mux222 br2(i1[30],i1[28],s1,i2[30]);
mux222 br3(i1[29],i1[27],s1,i2[29]);
mux222 br4(i1[28],i1[26],s1,i2[28]);
mux222 br5(i1[27],i1[25],s1,i2[27]);
mux222 br6(i1[26],i1[24],s1,i2[26]);
mux222 br7(i1[25],i1[23],s1,i2[25]);
mux222 br8(i1[24],i1[22],s1,i2[24]);
mux222 br9(i1[23],i1[21],s1,i2[23]);
mux222 br10(i1[22],i1[20],s1,i2[22]);
mux222 br11(i1[21],i1[19],s1,i2[21]);
mux222 br12(i1[20],i1[18],s1,i2[20]);
mux222 br13(i1[19],i1[17],s1,i2[19]);
mux222 br14(i1[18],i1[16],s1,i2[18]);
mux222 br15(i1[17],i1[15],s1,i2[17]);
mux222 br16(i1[16],i1[14],s1,i2[16]);
mux222 br17(i1[15],i1[13],s1,i2[15]);
mux222 br18(i1[14],i1[12],s1,i2[14]);
mux222 br19(i1[13],i1[11],s1,i2[13]);
mux222 br20(i1[12],i1[10],s1,i2[12]);
mux222 br21(i1[11],i1[9],s1,i2[11]);
mux222 br22(i1[10],i1[8],s1,i2[10]);
mux222 br23(i1[9],i1[7],s1,i2[9]);
mux222 br24(i1[8],i1[6],s1,i2[8]);
mux222 br25(i1[7],i1[5],s1,i2[7]);
mux222 br26(i1[6],i1[4],s1,i2[6]);
mux222 br27(i1[5],i1[3],s1,i2[5]);
mux222 br28(i1[4],i1[2],s1,i2[4]);
mux222 br29(i1[3],i1[1],s1,i2[3]);
mux222 br30(i1[2],i1[0],s1,i2[2]);
mux222 br31(i1[1],0,s1,i2[1]);
mux222 br32(i1[0],0,s1,i2[0]);

mux222 cr1(i2[31],i2[27],s2,i3[31]);
mux222 cr2(i2[30],i2[26],s2,i3[30]);
mux222 cr3(i2[29],i2[25],s2,i3[29]);
mux222 cr4(i2[28],i2[24],s2,i3[28]);
mux222 cr5(i2[27],i2[23],s2,i3[27]);
mux222 cr6(i2[26],i2[22],s2,i3[26]);
mux222 cr7(i2[25],i2[21],s2,i3[25]);
mux222 cr8(i2[24],i2[20],s2,i3[24]);
mux222 cr9(i2[23],i2[19],s2,i3[23]);
mux222 cr10(i2[22],i2[18],s2,i3[22]);
mux222 cr11(i2[21],i2[17],s2,i3[21]);
mux222 cr12(i2[20],i2[16],s2,i3[20]);
mux222 cr13(i2[19],i2[15],s2,i3[19]);
mux222 cr14(i2[18],i2[14],s2,i3[18]);
mux222 cr15(i2[17],i2[13],s2,i3[17]);
mux222 cr16(i2[16],i2[12],s2,i3[16]);
mux222 cr17(i2[15],i2[11],s2,i3[15]);
mux222 cr18(i2[14],i2[10],s2,i3[14]);
mux222 cr19(i2[13],i2[9],s2,i3[13]);
mux222 cr20(i2[12],i2[8],s2,i3[12]);
mux222 cr21(i2[11],i2[7],s2,i3[11]);
mux222 cr22(i2[10],i2[6],s2,i3[10]);
mux222 cr23(i2[9],i2[5],s2,i3[9]);
mux222 cr24(i2[8],i2[4],s2,i3[8]);
mux222 cr25(i2[7],i2[3],s2,i3[7]);
mux222 cr26(i2[6],i2[2],s2,i3[6]);
mux222 cr27(i2[5],i2[1],s2,i3[5]);
mux222 cr28(i2[4],i2[0],s2,i3[4]);
mux222 cr29(i2[3],0,s2,i3[3]);
mux222 cr30(i2[2],0,s2,i3[2]);
mux222 cr31(i2[1],0,s2,i3[1]);
mux222 cr32(i2[0],0,s2,i3[0]);

mux222 dr1(i3[31],i3[23],s3,i4[31]);
mux222 dr2(i3[30],i3[22],s3,i4[30]);
mux222 dr3(i3[29],i3[21],s3,i4[29]);
mux222 dr4(i3[28],i3[20],s3,i4[28]);
mux222 dr5(i3[27],i3[19],s3,i4[27]);
mux222 dr6(i3[26],i3[18],s3,i4[26]);
mux222 dr7(i3[25],i3[17],s3,i4[25]);
mux222 dr8(i3[24],i3[16],s3,i4[24]);
mux222 dr9(i3[23],i3[15],s3,i4[23]);
mux222 dr10(i3[22],i3[14],s3,i4[22]);
mux222 dr11(i3[21],i3[13],s3,i4[21]);
mux222 dr12(i3[20],i3[12],s3,i4[20]);
mux222 dr13(i3[19],i3[11],s3,i4[19]);
mux222 dr14(i3[18],i3[10],s3,i4[18]);
mux222 dr15(i3[17],i3[9],s3,i4[17]);
mux222 dr16(i3[16],i3[8],s3,i4[16]);
mux222 dr17(i3[15],i3[7],s3,i4[15]);
mux222 dr18(i3[14],i3[6],s3,i4[14]);
mux222 dr19(i3[13],i3[5],s3,i4[13]);
mux222 dr20(i3[12],i3[4],s3,i4[12]);
mux222 dr21(i3[11],i3[3],s3,i4[11]);
mux222 dr22(i3[10],i3[2],s3,i4[10]);
mux222 dr23(i3[9],i3[1],s3,i4[9]);
mux222 dr24(i3[8],i3[0],s3,i4[8]);
mux222 dr25(i3[7],0,s3,i4[7]);
mux222 dr26(i3[6],0,s3,i4[6]);
mux222 dr27(i3[5],0,s3,i4[5]);
mux222 dr28(i3[4],0,s3,i4[4]);
mux222 dr29(i3[3],0,s3,i4[3]);
mux222 dr30(i3[2],0,s3,i4[2]);
mux222 dr31(i3[1],0,s3,i4[1]);
mux222 dr32(i3[0],0,s3,i4[0]);

mux222 er1(i4[31],i4[15],s4,out[31]);
mux222 er2(i4[30],i4[14],s4,out[30]);
mux222 er3(i4[29],i4[13],s4,out[29]);
mux222 er4(i4[28],i4[12],s4,out[28]);
mux222 er5(i4[27],i4[11],s4,out[27]);
mux222 er6(i4[26],i4[10],s4,out[26]);
mux222 er7(i4[25],i4[9],s4,out[25]);
mux222 er8(i4[24],i4[8],s4,out[24]);
mux222 er9(i4[23],i4[7],s4,out[23]);
mux222 er10(i4[22],i4[6],s4,out[22]);
mux222 er11(i4[21],i4[5],s4,out[21]);
mux222 er12(i4[20],i4[4],s4,out[20]);
mux222 er13(i4[19],i4[3],s4,out[19]);
mux222 er14(i4[18],i4[2],s4,out[18]);
mux222 er15(i4[17],i4[1],s4,out[17]);
mux222 er16(i4[16],i4[0],s4,out[16]);
mux222 er17(i4[15],0,s4,out[15]);
mux222 er18(i4[14],0,s4,out[14]);
mux222 er19(i4[13],0,s4,out[13]);
mux222 er20(i4[12],0,s4,out[12]);
mux222 er21(i4[11],0,s4,out[11]);
mux222 er22(i4[10],0,s4,out[10]);
mux222 er23(i4[9],0,s4,out[9]);
mux222 er24(i4[8],0,s4,out[8]);
mux222 er25(i4[7],0,s4,out[7]);
mux222 er26(i4[6],0,s4,out[6]);
mux222 er27(i4[5],0,s4,out[5]);
mux222 er28(i4[4],0,s4,out[4]);
mux222 er29(i4[3],0,s4,out[3]);
mux222 er30(i4[2],0,s4,out[2]);
mux222 er31(i4[1],0,s4,out[1]);
mux222 er32(i4[0],0,s4,out[0]);

endmodule

module mux222(a,b,s,o);
input a,b,s;
output  o;
wire w1,w2,w3;
assign w1=~(s);
assign w2=w1&a;
assign w3=s&b;
assign o=w2|w3;
endmodule


/////barrelshift32.v............right


//`include "mux2345.v"

module barrelshift32(a,s0,s1,s2,s3,s4,out);

input [31:0]a;
input s0,s1,s2,s3,s4;
output [31:0]out;
wire [31:0]i1,i2,i3,i4;

mux2345 br1(a[31],0,s0,i1[31]);
mux2345 br2(a[30],a[31],s0,i1[30]);
mux2345 br3(a[29],a[30],s0,i1[29]);
mux2345 br4(a[28],a[29],s0,i1[28]);
mux2345 br5(a[27],a[28],s0,i1[27]);
mux2345 br6(a[26],a[27],s0,i1[26]);
mux2345 br7(a[25],a[26],s0,i1[25]);
mux2345 br8(a[24],a[25],s0,i1[24]);
mux2345 br9(a[23],a[24],s0,i1[23]);
mux2345 br10(a[22],a[23],s0,i1[22]);
mux2345 br11(a[21],a[22],s0,i1[21]);
mux2345 br12(a[20],a[21],s0,i1[20]);
mux2345 br13(a[19],a[20],s0,i1[19]);
mux2345 br14(a[18],a[19],s0,i1[18]);
mux2345 br15(a[17],a[18],s0,i1[17]);
mux2345 br16(a[16],a[17],s0,i1[16]);
mux2345 br17(a[15],a[16],s0,i1[15]);
mux2345 br18(a[14],a[15],s0,i1[14]);
mux2345 br19(a[13],a[14],s0,i1[13]);
mux2345 br20(a[12],a[13],s0,i1[12]);
mux2345 br21(a[11],a[12],s0,i1[11]);
mux2345 br22(a[10],a[11],s0,i1[10]);
mux2345 br23(a[9],a[10],s0,i1[9]);
mux2345 br24(a[8],a[9],s0,i1[8]);
mux2345 br25(a[7],a[8],s0,i1[7]);
mux2345 br26(a[6],a[7],s0,i1[6]);
mux2345 br27(a[5],a[6],s0,i1[5]);
mux2345 br28(a[4],a[5],s0,i1[4]);
mux2345 br29(a[3],a[4],s0,i1[3]);
mux2345 br30(a[2],a[3],s0,i1[2]);
mux2345 br31(a[1],a[2],s0,i1[1]);
mux2345 br32(a[0],a[1],s0,i1[0]);


mux2345 cbr1(i1[31],0,s1,i2[31]);
mux2345 cbr2(i1[30],0,s1,i2[30]);
mux2345 cbr3(i1[29],i1[31],s1,i2[29]);
mux2345 cbr4(i1[28],i1[30],s1,i2[28]);
mux2345 cbr5(i1[27],i1[29],s1,i2[27]);
mux2345 cbr6(i1[26],i1[28],s1,i2[26]);
mux2345 cbr7(i1[25],i1[27],s1,i2[25]);
mux2345 cbr8(i1[24],i1[26],s1,i2[24]);
mux2345 cbr9(i1[23],i1[25],s1,i2[23]);
mux2345 cbr10(i1[22],i1[24],s1,i2[22]);
mux2345 cbr11(i1[21],i1[23],s1,i2[21]);
mux2345 cbr12(i1[20],i1[22],s1,i2[20]);
mux2345 cbr13(i1[19],i1[21],s1,i2[19]);
mux2345 cbr14(i1[18],i1[20],s1,i2[18]);
mux2345 cbr15(i1[17],i1[19],s1,i2[17]);
mux2345 cbr16(i1[16],i1[18],s1,i2[16]);
mux2345 cbr17(i1[15],i1[17],s1,i2[15]);
mux2345 cbr18(i1[14],i1[16],s1,i2[14]);
mux2345 cbr19(i1[13],i1[15],s1,i2[13]);
mux2345 cbr20(i1[12],i1[14],s1,i2[12]);
mux2345 cbr21(i1[11],i1[13],s1,i2[11]);
mux2345 cbr22(i1[10],i1[12],s1,i2[10]);
mux2345 cbr23(i1[9],i1[11],s1,i2[9]);
mux2345 cbr24(i1[8],i1[10],s1,i2[8]);
mux2345 cbr25(i1[7],i1[9],s1,i2[7]);
mux2345 cbr26(i1[6],i1[8],s1,i2[6]);
mux2345 cbr27(i1[5],i1[7],s1,i2[5]);
mux2345 cbr28(i1[4],i1[6],s1,i2[4]);
mux2345 cbr29(i1[3],i1[5],s1,i2[3]);
mux2345 cbr30(i1[2],i1[4],s1,i2[2]);
mux2345 cbr31(i1[1],i1[3],s1,i2[1]);
mux2345 cbr32(i1[0],i1[2],s1,i2[0]);



mux2345 dbr1(i2[31],0,s2,i3[31]);
mux2345 dbr2(i2[30],0,s2,i3[30]);
mux2345 dbr3(i2[29],0,s2,i3[29]);
mux2345 dbr4(i2[28],0,s2,i3[28]);
mux2345 dbr5(i2[27],i2[31],s2,i3[27]);
mux2345 dbr6(i2[26],i2[30],s2,i3[26]);
mux2345 dbr7(i2[25],i2[29],s2,i3[25]);
mux2345 dbr8(i2[24],i2[28],s2,i3[24]);
mux2345 dbr9(i2[23],i2[27],s2,i3[23]);
mux2345 dbr10(i2[22],i2[26],s2,i3[22]);
mux2345 dbr11(i2[21],i2[25],s2,i3[21]);
mux2345 dbr12(i2[20],i2[24],s2,i3[20]);
mux2345 dbr13(i2[19],i2[23],s2,i3[19]);
mux2345 dbr14(i2[18],i2[22],s2,i3[18]);
mux2345 dbr15(i2[17],i2[21],s2,i3[17]);
mux2345 dbr16(i2[16],i2[20],s2,i3[16]);
mux2345 dbr17(i2[15],i2[19],s2,i3[15]);
mux2345 dbr18(i2[14],i2[18],s2,i3[14]);
mux2345 dbr19(i2[13],i2[17],s2,i3[13]);
mux2345 dbr20(i2[12],i2[16],s2,i3[12]);
mux2345 dbr21(i2[11],i2[15],s2,i3[11]);
mux2345 dbr22(i2[10],i2[14],s2,i3[10]);
mux2345 dbr23(i2[9],i2[13],s2,i3[9]);
mux2345 dbr24(i2[8],i2[12],s2,i3[8]);
mux2345 dbr25(i2[7],i2[11],s2,i3[7]);
mux2345 dbr26(i2[6],i2[10],s2,i3[6]);
mux2345 dbr27(i2[5],i2[9],s2,i3[5]);
mux2345 dbr28(i2[4],i2[8],s2,i3[4]);
mux2345 dbr29(i2[3],i2[7],s2,i3[3]);
mux2345 dbr30(i2[2],i2[6],s2,i3[2]);
mux2345 dbr31(i2[1],i2[5],s2,i3[1]);
mux2345 dbr32(i2[0],i2[4],s2,i3[0]);


mux2345 abr1(i3[31],0,s3,i4[31]);
mux2345 abr2(i3[30],0,s3,i4[30]);
mux2345 abr3(i3[29],0,s3,i4[29]);
mux2345 abr4(i3[28],0,s3,i4[28]);
mux2345 abr5(i3[27],0,s3,i4[27]);
mux2345 abr6(i3[26],0,s3,i4[26]);
mux2345 abr7(i3[25],0,s3,i4[25]);
mux2345 abr8(i3[24],0,s3,i4[24]);
mux2345 abr9(i3[23],i3[31],s3,i4[23]);
mux2345 abr10(i3[22],i3[30],s3,i4[22]);
mux2345 abr11(i3[21],i3[29],s3,i4[21]);
mux2345 abr12(i3[20],i3[28],s3,i4[20]);
mux2345 abr13(i3[19],i3[27],s3,i4[19]);
mux2345 abr14(i3[18],i3[26],s3,i4[18]);
mux2345 abr15(i3[17],i3[25],s3,i4[17]);
mux2345 abr16(i3[16],i3[24],s3,i4[16]);
mux2345 abr17(i3[15],i3[23],s3,i4[15]);
mux2345 abr18(i3[14],i3[22],s3,i4[14]);
mux2345 abr19(i3[13],i3[21],s3,i4[13]);
mux2345 abr20(i3[12],i3[20],s3,i4[12]);
mux2345 abr21(i3[11],i3[19],s3,i4[11]);
mux2345 abr22(i3[10],i3[18],s3,i4[10]);
mux2345 abr23(i3[9],i3[17],s3,i4[9]);
mux2345 abr24(i3[8],i3[16],s3,i4[8]);
mux2345 abr25(i3[7],i3[15],s3,i4[7]);
mux2345 abr26(i3[6],i3[14],s3,i4[6]);
mux2345 abr27(i3[5],i3[13],s3,i4[5]);
mux2345 abr28(i3[4],i3[12],s3,i4[4]);
mux2345 abr29(i3[3],i3[11],s3,i4[3]);
mux2345 abr30(i3[2],i3[10],s3,i4[2]);
mux2345 abr31(i3[1],i3[9],s3,i4[1]);
mux2345 abr32(i3[0],i3[8],s3,i4[0]);

mux2345 ebr1(i4[31],0,s4,out[31]);
mux2345 ebr2(i4[30],0,s4,out[30]);
mux2345 ebr3(i4[29],0,s4,out[29]);
mux2345 ebr4(i4[28],0,s4,out[28]);
mux2345 ebr5(i4[27],0,s4,out[27]);
mux2345 ebr6(i4[26],0,s4,out[26]);
mux2345 ebr7(i4[25],0,s4,out[25]);
mux2345 ebr8(i4[24],0,s4,out[24]);
mux2345 ebr9(i4[23],0,s4,out[23]);
mux2345 ebr10(i4[22],0,s4,out[22]);
mux2345 ebr11(i4[21],0,s4,out[21]);
mux2345 ebr12(i4[20],0,s4,out[20]);
mux2345 ebr13(i4[19],0,s4,out[19]);
mux2345 ebr14(i4[18],0,s4,out[18]);
mux2345 ebr15(i4[17],0,s4,out[17]);
mux2345 ebr16(i4[16],0,s4,out[16]);
mux2345 ebr17(i4[15],i4[31],s4,out[15]);
mux2345 ebr18(i4[14],i4[30],s4,out[14]);
mux2345 ebr319(i4[13],i4[29],s4,out[13]);
mux2345 ebr320(i4[12],i4[28],s4,out[12]);
mux2345 ebr321(i4[11],i4[27],s4,out[11]);
mux2345 ebr322(i4[10],i4[26],s4,out[10]);
mux2345 ebr323(i4[9],i4[25],s4,out[9]);
mux2345 ebr324(i4[8],i4[24],s4,out[8]);
mux2345 ebr325(i4[7],i4[23],s4,out[7]);
mux2345 ebr326(i4[6],i4[22],s4,out[6]);
mux2345 ebr327(i4[5],i4[21],s4,out[5]);
mux2345 ebr328(i4[4],i4[20],s4,out[4]);
mux2345 ebr329(i4[3],i4[19],s4,out[3]);
mux2345 ebr330(i4[2],i4[18],s4,out[2]);
mux2345 ebr331(i4[1],i4[17],s4,out[1]);
mux2345 ebr332(i4[0],i4[16],s4,out[0]);


endmodule
module mux2345(a,b,s,o);
input a,b,s;
output  o;
wire w1,w2,w3;
assign w1=~(s);
assign w2=w1&a;
assign w3=s&b;
assign o=w2|w3;
endmodule



///ff_proc.v  ............

module ff_fetch ( inst_ip, clk, inst_op);
   input [31:0]inst_ip;
   input clk;
   output reg [31:0]inst_op;
     	 
   always @ (posedge clk)
   begin
    inst_op <= inst_ip;
   end

endmodule

module ff_decode ( insa,insb,data_a,data_b,write_back, clk, q,q1,q2,q3,q4);
   input [7:0]insa,insb,write_back;
   input [31:0]data_a,data_b;
   input clk;
   output [7:0]q,q1,q4;
   output [31:0]q2,q3;
   wire clk;
   reg [7:0]q,q1,q4;
   reg [31:0] q2,q3;
     	 
   always @ (posedge clk)
   begin
    q <= insa;
    q1 <= insb;
    q2 <= data_a;
    q3 <= data_b;
    q4 <= write_back;
 end

endmodule

module ff_execute ( execout_ip, clk, execout_op);
   input [31:0]execout_ip;
   input clk;
   output reg [31:0]execout_op;
     	 
   always @ (posedge clk)
   begin
    execout_op <= execout_ip;
   end

endmodule


module ff_writeback ( writeback_ip, clk, writeback_op);
   input [31:0]writeback_ip;
   input clk;
   output reg [31:0]writeback_op;
     	 
   always @ (posedge clk)
   begin
     writeback_op <= writeback_ip;
   end

endmodule





