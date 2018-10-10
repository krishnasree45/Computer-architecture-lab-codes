module cache(cache_read,cache_miss,wr_data,data,op,e,c,out,clk);//module for accessing memory 16*8 
	
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
	cache_tag[6]=8'd8;
	cache_tag[7]=8'd6;
	cache_tag[8]=8'd7;
	cache_tag[9]=8'd9;
	cache_tag[10]=8'd10;
	cache_tag[11]=8'd11;
	cache_tag[12]=8'd12;
	cache_tag[13]=8'd13;
	cache_tag[14]=8'd14;
	cache_tag[15]=8'd15;

	
	cache_data[0]=32'b00000000000000000000000000011111;
	cache_data[1]=32'b00000000000000000000000000000001;
	cache_data[2]=32'b00000000000000000000000000000010;
	cache_data[3]=32'b00000000000000000000000000000011;
	cache_data[4]=32'b00000000000000000000000000000100;
	cache_data[5]=32'b00000000000000000000000000000101;
	cache_data[6]=32'b00000000000000000000000000000110;
	cache_data[7]=32'b00000000000000000000000000000111;
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

