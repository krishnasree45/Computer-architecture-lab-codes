`include"main_pip.v"
module maintb();


reg sa,sb,opcode;
reg [22:0]ma,mb;
reg [7:0]ea,eb;
reg clk;
wire [7:0]exponent,exp,res;
wire sign,totalcarry,case1;
wire [23:0]out_mant,sss,mant;
wire [31:0]sum2;


//wire [7:0]exp;

//main(sa,sb,opcode,ma,mb,ea,eb,out_mant,sign,exponent,totalcarry,sum2,sss,mant,res,clk);
main  m00(sa,sb,opcode,ma,mb,ea,eb,out_mant,sign,exponent,totalcarry,sum2,sss,mant,res,clk);

initial 
clk=1'b0;

initial
begin
sa=1'b0;
sb=1'b0;
opcode=1'b0;
ma=23'b10000000000000000000000;
mb=23'b01000000000000000000000;
ea=8'b10000000;
eb=8'b10000001;
#30 $finish;
end

always
#2 clk=~clk;

initial
$monitor($time,"clock=%b,sa=%b , sb=%b, opcode=%b,ma=%b, mb=%b,ea=%d,eb=%d,out_mant=%b,sign=%b,exponent=%b,carryout=%b,sum2=%b,sss=%b,mant=%b,res=%b",clk,sa,sb,opcode,ma,mb,ea,eb,out_mant,sign,exponent,totalcarry,sum2,sss,mant,res);

endmodule


/*reg [7:0]ea,eb;
reg [22:0]ma,mb;

wire [31:0]maaa,mbbb;
wire [23:0]out1,out2;

main  m00(ea,eb,ma,mb,maaa,mbbb,out1,out2);
initial
begin
ea=8'd4;
eb=8'd2;

ma=23'b10000000000000000000011;
mb=23'b01000000000000000000111;



$monitor("ea=%d,eb=%d,ma=%b,mb=%b,maaa=%b,mbbb=%b,out1=%b,out2=%b,",ea,eb,ma,mb,maaa,mbbb,out1,out2);
end 
endmodule*/

