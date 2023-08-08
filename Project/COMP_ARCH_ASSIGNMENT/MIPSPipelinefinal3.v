`include "MIPSALU.v"

module ID_EX(PC,Sign_extended_val,reg_A,reg_B,alucon_inp,mem_sig_inp,alu_or_mem_op,Rs,Rt,Rd,,dst_control,branch_in,
clk,Sign_extended_val_out,PC_out,idexA,idexB,alucon_out,mem_sig_out,alu_or_mem_op_1,rs_out,rt_out,rtc_out,rd_out,dst_control_out,branch_out); //control signals and other register values are inputs at this stage
input [31:0]reg_A,reg_B,Sign_extended_val;
input[7:0] PC;
input clk;
input [2:0] alucon_inp;
input mem_sig_inp,alu_or_mem_op,dst_control,branch_in;
input [4:0] Rs,Rt,Rd;


output reg [4:0] rs_out,rt_out,rtc_out,rd_out;
output reg [31:0]idexA,idexB,PC_out,Sign_extended_val_out;
output reg [2:0]alucon_out;
output reg mem_sig_out,alu_or_mem_op_1,dst_control_out,branch_out;

always @(posedge clk)
	begin
	// Passing the register values
	idexA = reg_A;
	idexB = reg_B;
	// Passing the control signals
	alucon_out = alucon_inp;
	mem_sig_out = mem_sig_inp;
	alu_or_mem_op_1  = alu_or_mem_op;
	dst_control_out = dst_control;
	branch_out = branch_in;
	// Passing the register number of source, destination
	//$display("A: %h, B= %h", reg_A, reg_B);
	rs_out = Rs;
	rt_out = Rt;
	rd_out = Rd;
	rtc_out = Rt;
	//Passing PC and a sign extended number
	PC_out = PC;
	Sign_extended_val_out = Sign_extended_val;
	//$display($time," A,B = %d,%d",idexA,idexB);
	end

endmodule	

module EX_MEM(branch_value,aluout_inp,mem_sig_inp,alu_or_mem_op,Rdest,branch_in,clk,aluout_out,mem_sig_out,alu_or_mem_op1,Rdestout,branch_out,branch_value_out);
input [31:0]aluout_inp, branch_value;
input clk;
input mem_sig_inp,alu_or_mem_op,branch_in;
input [4:0] Rdest;

output reg [31:0]aluout_out;
output reg branch_value_out;
output reg mem_sig_out,alu_or_mem_op1,branch_out;
output reg [4:0] Rdestout;
always @(posedge clk)
	begin
	//$display("ALU_out: %h",aluout_inp);
	aluout_out = aluout_inp;
	//control signals
	mem_sig_out = mem_sig_inp;
	alu_or_mem_op1 = alu_or_mem_op;
	branch_out = branch_in;
	//destination register
	Rdestout = Rdest;
	// 32 bit branch value
	branch_value_out = branch_value;
	//$display($time," Alu val in mem stage = %d",aluout_out);
	end
	
endmodule	

module MEM_WB(aluout_inp,mem_val_inp,alu_or_mem_op,Rdest,clk,aluout_out,mem_val_out,wb_sig,Rdestout);
input [31:0]aluout_inp,mem_val_inp;
input clk,alu_or_mem_op;
input [4:0] Rdest;

output reg [31:0]aluout_out,mem_val_out;
output reg [4:0] Rdestout;
output reg wb_sig;
always @(posedge clk)
	begin
	aluout_out = aluout_inp;
	mem_val_out = mem_val_inp;
	wb_sig	= alu_or_mem_op;
	Rdestout = Rdest;
	//$display($time," Alu val in wb stage = %d",aluout_out);
	end
	
endmodule


module datamemory(mem_sig,clk,mem_out_val,mem_in_val);
input mem_sig,clk;
input [31:0] mem_in_val;
output reg [31:0] mem_out_val;

localparam read = 1'b0;
localparam write = 1'b1;

always @(negedge clk)
	begin
	if (mem_sig == read)
		begin
		mem_out_val = mem_in_val; // here we need to add the code to access memory and read from it
		end
	else
		begin
		mem_out_val = 32'bx; // here we need to add the code to access memory and write into it
		end
	end
endmodule

module adder(output reg[7:0] out, input[7:0] inp1, input[7:0] inp2, input clk);
always@(negedge clk)
begin
out=(inp1+inp2);
//$display("OUT: %h",out);
end
endmodule

module register_field(output reg[31:0] out1, output reg[31:0] out2, input[4:0] address1, input[4:0] address2, input[4:0] writereg, input[31:0] writedata, input readEn, input writeEn, input clk, input rst);
reg[31:0] regfile[20:0];
integer i;
reg[31:0] A,B;  

always @ (negedge clk)
begin

A<=regfile[address1][31:0];
B<=regfile[address2][31:0];
//$display("yoma: %h,%h", A, B);
end

always @ (negedge clk)
begin
	#2
	//$display("yo: %h,%h", A, B);
	if(readEn)
		begin
		out1=A;
		out2=B;
		end
	//$display("address1: %h, address2: %h",address1, address2);
end

always@(rst)
begin
if(rst)
begin
regfile[0][31:0]<= 32'd0;
regfile[1][31:0]<= 10;
regfile[2][31:0]<= 0;
regfile[3][31:0]<= 5;
regfile[4][31:0]<= 0;
regfile[5][31:0]<= 0;
regfile[6][31:0]<= 3;
regfile[7][31:0]<= 0;
regfile[8][31:0]<= 0;
regfile[9][31:0]<= 0;
#2
$display("Initial value of registers:");
for(i=0;i<10;i++)
	$display("regfile[%d] = %d", i, regfile[i][31:0]);
end 
end

always @ (negedge clk)
begin
	if(writeEn)
		regfile[writereg][31:0]<=writedata;
end
endmodule

module memory(output reg [31:0] memOut, input [7:0] address, input [31:0] dataIn, input clk, input readEnable, input writeEnable, input initializeMemory);
reg [31:0] memory [0:5];
integer i, f;

always @ (posedge initializeMemory)
begin
	$display("File Read");
	$readmemh("Instn_mem.txt", memory);
end
always @ (negedge clk)	
	if(readEnable)
		begin
		//$display(address);
		memOut <= memory[address];
		//$display("INSTN= %h",memOut);
		end
	else if (writeEnable)
		memory [address] <= dataIn;
	else memOut <= 31'd0;

endmodule

module control(input[39:0] ifid, output reg[2:0] alucon_inp, output reg alu_or_mem_op, output reg mem_sig, output reg dest_control, output reg branch_input, input clk);
always@(negedge clk)
begin
#2
dest_control<=1'b1;
mem_sig<=1'b0;
case(ifid[31:26])
	6'b000000:
		begin
			//ALU op
			alu_or_mem_op<=1'b1;
			branch_input=1'b0;
			case(ifid[5:0])
				6'b100000: alucon_inp[2:0]<=3'b000;
				6'b100001: alucon_inp[2:0]<=3'b001;
				6'b100010: alucon_inp[2:0]<=3'b010;
				6'b100011: alucon_inp[2:0]<=3'b11;
			endcase
        end
	6'b000100:
		begin
			//$display("BRANCH");
			alu_or_mem_op<=1'b0;
			branch_input<=1'b1;
			alucon_inp[2:0]<=3'b001;
		end
endcase
end
endmodule

////////////////////PROCESSOR////////////////////////


module main(clk,rst,final_postwb_mux_output, in1, in2, inst, PC_final);
input clk,rst;
output reg[31:0] in1,in2;
reg[7:0] PC;
output reg[7:0] PC_final;
output reg[31:0] inst;
wire[4:0] rdest_mem_wb;
wire[31:0] post_wb_MEM_op,branch_val_out;
wire [31:0]reg_A,reg_B;
reg[31:0] Sign_extended_val;
reg PC_control;
wire[39:0] ifid_inp;
reg[39:0] ifid;
wire[2:0] alucon_inp;
wire mem_sig_inp;
wire alu_or_mem_op_inp;
input dest_control,branch_in;
memory instn_memory (ifid_inp[31:0], PC, 32'd0, clk, 1'b1, 1'b0, rst);
adder a1(ifid_inp[39:32],PC,8'd1,clk);
register_field rf (reg_A, reg_B, ifid[25:21], ifid[20:16], rdest_mem_wb, post_wb_MEM_op, 1'b1, 1'b1, clk, rst);
control cu(ifid, alucon_inp, alu_or_mem_op_inp, mem_sig_inp, dest_control, branch_in, clk);
reg[4:0] Rs,Rt,Rd;
reg [1:0] forwardA = 2'b00, forwardB = 2'b00;
wire [4:0] rs_out,rd_out,rt_out,rtc_out,rdest_ex_mem;
wire [31:0] out,aluout_out,tryer,A,B;
wire [2:0] alucon;
wire y,temp,x,temp1,temp2,wb_sig_final,dest_control_out,branch_out_temp,branch_out_temp1;
wire [31:0] post_wb_ALU_op,PC_out,Sign_extended_val_out;
output reg [31:0] final_postwb_mux_output;
reg mem_inp_to_pipeline2,wb_signal,wb_signal1,branch_out1,branch_out_final;
reg [31:0]alu_inp_to_mem,inpA_toALU,inpB_toALU;
reg [4:0] dest_out,dest_inp_memwb;
reg [31:0] final_value_for_branch;
wire [31:0] PC_memWb_in,sign_extended_shifted;

always@(rst)
begin
if(rst)
PC<=8'd0;
PC_control=1'b0;
end

always@(negedge clk)
begin
Sign_extended_val[31:16]=16'd0;
Sign_extended_val[15:0]<=ifid[15:0];
Rs<=ifid[25:21];
Rt<=ifid[20:16];
Rd<=ifid[15:11];
end

always@(PC)
PC_final<=PC;

//if -id pipeline
always@(posedge clk)
	begin
		//$display("%h",ifid_inp[39:32]);
		inst<=ifid[31:0];
		ifid[31:0]<=ifid_inp[31:0];
		ifid[39:32]<=ifid_inp[39:32];
		//$display("%b",PC_control);
		//PC<=ifid_inp[39:32];
		case(PC_control)
		  1'b0:PC<=ifid_inp[39:32];
		  1'b1:
			begin
				PC<= 8'd0;;
				PC_control = 1'b0;
			end
		endcase
		//$display("%h",PC);
	end


// for selectline of mux
localparam case_a = 2'b00;
localparam case_b = 2'b01;
localparam case_c = 2'b10;

ID_EX dut0(.PC(PC),.Sign_extended_val(Sign_extended_val),.reg_A(reg_A),.reg_B(reg_B),.alucon_inp(alucon_inp),.mem_sig_inp(mem_sig_inp),.alu_or_mem_op(alu_or_mem_op),.Rs(Rs),.Rd(Rd),.Rt(Rt),.dst_control(dest_control),.branch_in(branch_in),.clk(clk),.PC_out(PC_out),.Sign_extended_val_out(Sign_extended_val_out),
.idexA(A),.idexB(B),.alucon_out(alucon),.mem_sig_out(temp),.alu_or_mem_op_1(temp1),.rs_out(rs_out),.rt_out(rt_out),.rtc_out(rtc_out),.rd_out(rd_out),.dst_control_out(dest_control_out),.branch_out(branch_out_temp));
// EX stage  add pcout,branch and sign thing above
// Muxes for alu input 
always @(*)
	begin
		mem_inp_to_pipeline2 = temp;
		wb_signal = temp1;
		branch_out1 = branch_out_temp;
	end
	
always @(*)
	begin
		case(forwardA)
				case_a: inpA_toALU = A;
				case_b: inpA_toALU = aluout_out;
				case_c: inpA_toALU = final_postwb_mux_output;
				default : inpA_toALU = 32'b0;
		endcase
		case(forwardB)
				case_a: inpB_toALU = B;
				case_b: inpB_toALU = aluout_out;
				case_c: inpB_toALU = final_postwb_mux_output;
				default : inpA_toALU = 32'b0;
		endcase	
	//$display( $time," a  is %d, and b is %d, %d,%d",inpA_toALU,inpB_toALU,forwardA,forwardB);		

	end	
// make another adder here.	

wire [32:0]carry_temp;
assign sign_extended_shifted = Sign_extended_val_out << 2;  // left shifting the 32 bit sign extended value by 2.
assign carry_temp[0] =0;

genvar i;
generate
	for(i= 0; i<32 ; i = i+1)
	begin
	bit_adder dut7(.F(PC_memWb_in[i]),.A(PC_out[i]),.B(sign_extended_shifted[i]),.cin(carry_temp[i]),.cout(carry_temp[i+1]));
	end
endgenerate

always @(PC_memWb_in)
	begin
		final_value_for_branch = PC_memWb_in;
	end
ALU dut1(.ALU_out(out),.A(inpA_toALU),.B(inpB_toALU),.alucon(alucon),.cin(x),.cout(y));

always@(posedge clk)
begin
in1<=inpA_toALU;
in2<=inpB_toALU;
end

//Mem Stage	
EX_MEM dut2(.branch_value(final_value_for_branch),.aluout_inp(out),.mem_sig_inp(mem_inp_to_pipeline2),.alu_or_mem_op(wb_signal),.Rdest(dest_out),.branch_in(branch_out1),.clk(clk),
.aluout_out(aluout_out),.mem_sig_out(mem_sig_out),.alu_or_mem_op1(temp2),.Rdestout(rdest_ex_mem),.branch_out(branch_out_temp1),.branch_value_out(branch_value_out));

assign branch_value_out = final_value_for_branch;
always @(aluout_out)
	begin
		alu_inp_to_mem = aluout_out;
		wb_signal1 = temp2;
		dest_inp_memwb = rdest_ex_mem;
		branch_out_final = branch_out_temp1;
		//$display($time,"branch out final is %d",branch_out_final);
	end

always @(branch_value_out)
	begin
		if (aluout_out == 32'b0 )
			begin
				if (branch_out_final ==1'b1)
				begin
					PC_control = 1'b1;
					#5
					$display($time," PC control has been updated successfully");
				end
			end
	end
datamemory dut3(.mem_sig(mem_sig_out),.clk(clk),.mem_out_val(tryer),.mem_in_val(alu_inp_to_mem));
// WB stage
MEM_WB dut4(.aluout_inp(alu_inp_to_mem),.mem_val_inp(tryer),.alu_or_mem_op(wb_signal1),.Rdest(dest_inp_memwb),.clk(clk),
.aluout_out(post_wb_ALU_op),.mem_val_out(post_wb_MEM_op),.wb_sig(wb_sig_final),.Rdestout(rdest_mem_wb));
assign wb_sig_final = 1'b1;
always @ (*)
	begin
		case(wb_sig_final)
			1'b0 : final_postwb_mux_output = post_wb_MEM_op;
			1'b1: final_postwb_mux_output = post_wb_ALU_op;
		endcase
		case(dest_control_out)
			1'b0 : dest_out = rtc_out;
			1'b1 : dest_out = rd_out;
		endcase
	//$display($time,"val of WB signal is %d",wb_sig_final);	
	//$display($time,"final alu val pre mux is %d and post mux after wb stage is %d",post_wb_ALU_op,final_postwb_mux_output);	
	end
	
//Forwarding unit
always @(negedge clk )
		begin
			if (dest_inp_memwb == rs_out) forwardA = case_b;
			else if (rdest_mem_wb == rs_out) forwardA = case_c;
			else forwardA = case_a;
			
			if	(dest_inp_memwb == rt_out) forwardB = case_b;
			else if (rdest_mem_wb == rt_out) forwardB = case_c;
			else forwardB = case_a;
				
		end
	
endmodule	



module testbench;
reg clk,rst = 1'b0;
wire [31:0]final_postwb_mux_output;
wire[31:0] in1, in2;
wire[31:0] inst;
wire[7:0] PC_final;
main try_2(.clk(clk),.rst(rst),.final_postwb_mux_output(final_postwb_mux_output),.in1(in1),.in2(in2),.inst(inst), .PC_final(PC_final)); 	

initial
		#10
		begin
		clk = 1'b0;
		forever
		#10 clk = ~clk;
		end
		

initial
	begin
	$dumpfile("ALU123.vcd");
	$dumpvars;
	$monitor($time," PC = %h IFID_INSTN_VAL = %h ALU_input1 = %d ALU_input2 = %d final_output = %d ", PC_final, inst, in1, in2, final_postwb_mux_output);
	#1 rst=1'b1;
	#2 rst=1'b0;
	#180 $finish;
	end
endmodule	
		
		






