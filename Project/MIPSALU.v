module bit_adder(F,A,B,cin,cout);
input A,B,cin;
output F,cout;
assign F = A^B^cin;
assign cout = A&B|B&cin|A&cin;
endmodule


module ALU(ALU_out,A,B,alucon,cin,cout,);
input [31:0] A,B;
input [2:0]alucon;
output reg [31:0]ALU_out;
input cin;
output cout;

localparam add = 3'b000;
localparam sub = 3'b001;
localparam AND = 3'b010;
localparam OR = 3'b011;

wire [31:0] holder;
wire [32:0] carry;
reg [31:0]temp_B;
reg x;
genvar i;


always @(*)
	begin
	if (alucon == add)
		begin
		temp_B = B;
		x = 1'b0;
		end
	else 
		begin
		temp_B = ~B;
		x = 1'b1;
		end	
	end
assign carry[0] = x;
	
generate
	for(i= 0; i<32 ; i = i+1)
	bit_adder dut0(.F(holder[i]),.A(A[i]),.B(temp_B[i]),.cin(carry[i]),.cout(carry[i+1]));
endgenerate

assign cout = carry[32];

always @ (*)
	begin
		case(alucon)
			add: ALU_out = holder;
			sub:ALU_out = holder;
			OR: ALU_out = A|B;
			AND: ALU_out = A&B;
			default: ALU_out = 32'b0;
		endcase
	//$display("%b : %h",alucon, ALU_out);
	end

endmodule	


			
			
			