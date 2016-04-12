module full_adder(x,y,cin,s,cout);
	input x,y,cin;
	output s,cout;
	wire s1,c1,c2,c3;
	xor(s1,x,y);
	xor(s,cin,s1);
	and(c1,x,y);
	and(c2,y,cin);
	and(c3,x,cin);
	or(cout,c1,c2,c3);
endmodule

module clock(output clk);
  reg clk;
  initial begin
    repeat( 18 ) begin
      clk = 1'b0;
      #1000000 clk = ~clk;
    end
  end
endmodule

module mux(input A, input B, input sel, output out);
  wire C;
	not(C,sel);
	wire D, E;
	and(D,A,C);
	and(E,B,sel);
	or(out,D,E);
endmodule

module adder16bit(input [15:0] A, input [15:0] B, output [15:0] result);
  wire [16:0] carry;
  assign carry[0] = 0;
  full_adder adder1(.x(A[0]),.y(B[0]),.cin(carry[0]),.s(result[0]),.cout(carry[1]));
  full_adder adder2(.x(A[1]),.y(B[1]),.cin(carry[1]),.s(result[1]),.cout(carry[2]));
  full_adder adder3(.x(A[2]),.y(B[2]),.cin(carry[2]),.s(result[2]),.cout(carry[3]));
  full_adder adder4(.x(A[3]),.y(B[3]),.cin(carry[3]),.s(result[3]),.cout(carry[4]));
  full_adder adder5(.x(A[4]),.y(B[4]),.cin(carry[4]),.s(result[4]),.cout(carry[5]));
  full_adder adder6(.x(A[5]),.y(B[5]),.cin(carry[5]),.s(result[5]),.cout(carry[6]));
  full_adder adder7(.x(A[6]),.y(B[6]),.cin(carry[6]),.s(result[6]),.cout(carry[7]));
  full_adder adder8(.x(A[7]),.y(B[7]),.cin(carry[7]),.s(result[7]),.cout(carry[8]));
  full_adder adder9(.x(A[8]),.y(B[8]),.cin(carry[8]),.s(result[8]),.cout(carry[9]));
  full_adder adder10(.x(A[9]),.y(B[9]),.cin(carry[9]),.s(result[9]),.cout(carry[10]));
  full_adder adder11(.x(A[10]),.y(B[10]),.cin(carry[10]),.s(result[10]),.cout(carry[11]));
  full_adder adder12(.x(A[11]),.y(B[11]),.cin(carry[11]),.s(result[11]),.cout(carry[12]));
  full_adder adder13(.x(A[12]),.y(B[12]),.cin(carry[12]),.s(result[12]),.cout(carry[13]));
  full_adder adder14(.x(A[13]),.y(B[13]),.cin(carry[13]),.s(result[13]),.cout(carry[14]));
  full_adder adder15(.x(A[14]),.y(B[14]),.cin(carry[14]),.s(result[14]),.cout(carry[15]));
  full_adder adder16(.x(A[15]),.y(B[15]),.cin(carry[15]),.s(result[15]),.cout(carry[16]));
endmodule

module ALU(input A, input B, input Cin, input [3:0] sel, input less, output out, output Cout, output set);
	wire notA, notB, C, D, E, F, G, H, I, J;
	not(notA,A);
	not(notB,B);
	mux mux1(A, notA, sel[3], C);
	mux mux2(B, notB, sel[2], D);
	and(E, C, D);
	or(F, C, D);
	full_adder add(C,D,Cin,G,Cout);
	assign H = less;
	assign set = G;
	mux mux3(E, F, sel[0], I);
	mux mux4(G, H, sel[0], J);
	mux mux5(I, J, sel[1], out);
endmodule

module ALU16bit(input [15:0] A, input [15:0] B, input [3:0] control, output [15:0] result, output overflow);
		wire [14:0] carry;
		wire less = 1'b0;
		wire [16:0] set;
		wire temp1;

		ALU alu1(.A(A[0]),.B(B[0]),.Cin(control[2]),.sel(control),.less(less),.out(temp1),.Cout(carry[0]),.set(set[0]));
		ALU alu2(.A(A[1]),.B(B[1]),.Cin(carry[0]),.sel(control),.less(less),.out(result[1]),.Cout(carry[1]),.set(set[1]));
		ALU alu3(.A(A[2]),.B(B[2]),.Cin(carry[1]),.sel(control),.less(less),.out(result[2]),.Cout(carry[2]),.set(set[2]));
		ALU alu4(.A(A[3]),.B(B[3]),.Cin(carry[2]),.sel(control),.less(less),.out(result[3]),.Cout(carry[3]),.set(set[3]));
		ALU alu5(.A(A[4]),.B(B[4]),.Cin(carry[3]),.sel(control),.less(less),.out(result[4]),.Cout(carry[4]),.set(set[4]));
		ALU alu6(.A(A[5]),.B(B[5]),.Cin(carry[4]),.sel(control),.less(less),.out(result[5]),.Cout(carry[5]),.set(set[5]));
		ALU alu7(.A(A[6]),.B(B[6]),.Cin(carry[5]),.sel(control),.less(less),.out(result[6]),.Cout(carry[6]),.set(set[6]));
		ALU alu8(.A(A[7]),.B(B[7]),.Cin(carry[6]),.sel(control),.less(less),.out(result[7]),.Cout(carry[7]),.set(set[7]));
		ALU alu9(.A(A[8]),.B(B[8]),.Cin(carry[7]),.sel(control),.less(less),.out(result[8]),.Cout(carry[8]),.set(set[8]));
		ALU alu10(.A(A[9]),.B(B[9]),.Cin(carry[8]),.sel(control),.less(less),.out(result[9]),.Cout(carry[9]),.set(set[9]));
		ALU alu11(.A(A[10]),.B(B[10]),.Cin(carry[9]),.sel(control),.less(less),.out(result[10]),.Cout(carry[10]),.set(set[10]));
		ALU alu12(.A(A[11]),.B(B[11]),.Cin(carry[10]),.sel(control),.less(less),.out(result[11]),.Cout(carry[11]),.set(set[11]));
		ALU alu13(.A(A[12]),.B(B[12]),.Cin(carry[11]),.sel(control),.less(less),.out(result[12]),.Cout(carry[12]),.set(set[12]));
		ALU alu14(.A(A[13]),.B(B[13]),.Cin(carry[12]),.sel(control),.less(less),.out(result[13]),.Cout(carry[13]),.set(set[13]));
		ALU alu15(.A(A[14]),.B(B[14]),.Cin(carry[13]),.sel(control),.less(less),.out(result[14]),.Cout(carry[14]),.set(set[14]));
		ALU alu16(.A(A[15]),.B(B[15]),.Cin(carry[14]),.sel(control),.less(less),.out(result[15]),.Cout(Cout),.set(set[15]));
		ALU alu17(.A(A[0]),.B(B[0]),.Cin(control[2]),.sel(control),.less(set[15]),.out(result[0]),.Cout(carry[0]),.set(set[16]));

		wire temp2;
		xor(temp2, Cout, carry[14]);
		and(overflow,temp2,control[1]);
endmodule

module dataMem(input clk, input write, input read, input [15:0] Addr, input [15:0] data_in, output [15:0] data_out);
  reg [15:0] temp_data;
  assign data_out = temp_data;

  reg [7:0] data[0:71];

  initial begin
    $readmemb("data.txt",data);
  end

  always @ ( posedge clk ) begin
    if (read == 1) begin
      temp_data[15:8] <= data[Addr];
      temp_data[7:0] <= data[Addr + 1];
    end
  end
  always @ ( negedge clk ) begin
    if (write == 1) begin
      data[Addr] <= data_in[15:8];
      data[Addr + 1] <= data_in[7:0];
    end
  end
endmodule

module instructMem(input clk, input [15:0] addr, output [31:0] instruction);
  reg [31:0] temp_instruct;
  assign instruction = temp_instruct;

  reg [7:0] instructions[0:71];

  initial begin
    $readmemb("instructions.txt",instructions);
  end
  always @ ( posedge clk ) begin
    temp_instruct[31:24] <= instructions[addr];
    temp_instruct[23:16] <= instructions[addr + 1];
    temp_instruct[15:8] <= instructions[addr + 2];
    temp_instruct[7:0] <= instructions[addr + 3];
  end

endmodule

module PCadjust(input clk, input [15:0] PC, input [15:0] jumpAddr, input [1:0] select, output [15:0] adjustedPC);
  wire [15:0] A, B;
  reg [15:0] four = 16'b0000_0000_0000_0100;
  reg [15:0] temp;
  assign adjustedPC = temp;
  adder16bit add1(.A(PC),.B(four),.result(A));
  adder16bit add2(.A(A),.B(jumpAddr),.result(B));
  always @ ( negedge clk ) begin
    case (select)
      2'b00: begin temp = A; end
      2'b01: begin temp = jumpAddr; end
      2'b10: begin temp = B; end
      default : $display("Error in select");
    endcase
  end

endmodule

module regfile(clk, rs, rt, rd, data_in, A, B);
	input [4:0] rs, rt, rd;
	input [15:0 ] data_in;
	output [15:0] A, B;
	input clk;

	wire [4:0] rs, rt, rd;
	wire [15:0] data_in;
	wire [15:0] A, B;

	reg [15:0] temp_A, temp_B;
	assign A = temp_A;
	assign B = temp_B;

	reg [15:0] register[0:31];

	initial begin
		$readmemb("registers.txt",register);
	end

  always @ ( posedge clk ) begin
    temp_A <= register[rs];
    temp_B <= register[rt];
  end

  always @ ( negedge clk ) begin
    register[rd] <= data_in;
  end
endmodule

module shiftLeft(input [15:0] in, output [15:0] out);
  assign out[15:2] = in[13:0];
  assign out[1] = 0;
  assign out[0] = 0;
endmodule

module mux16bit(sel, i1, i2, o1);
  input sel;
  input [15:0] i1, i2;
  output [15:0] o1;
  reg [15:0] o1;
  always @(sel or i1 or i2) begin
    case (sel)
      1'b0: o1 = i1;
      1'b1: o1 = i2;
    endcase
  end
endmodule

module mux3to1(sel, i1, i2, i3, o1);
  input [1:0] sel;
  input [15:0] i1, i2, i3;
  output [15:0] o1;
  reg [15:0] o1;

  always @(sel or i1 or i2 or i3) begin
    case (sel)
      2'b00: o1 = i1;
      2'b01: o1 = i2;
      2'b10: o1 = i3;
    endcase
  end
endmodule

module control;
	wire clk, overflow;
	reg [1:0] mux2Select;
  wire [15:0] temp_PC, rdData1, rdData2, ALUIn, writeRegData, PCplus4, jumpAddr, ALUResult, branchAddr, rdDataMem;
	reg RegWrite, ALUSrc, PCSrc, MemRead, MemWrite, MemtoReg, half; //if loading half then half = 1, if loading byte half = 0
	reg [3:0] ALUOpCode;
	reg [15:0] PC = 16'b0000_0000_0000_0000;
	reg [15:0] four = 16'b0000_0000_0000_0100;
	wire [31:0] instruction;
	reg [5:0] opcode, funct;
	reg [4:0] writeAddr;

	clock clk1(.clk(clk));
	instructMem instruct(clk, PC, instruction);
	adder16bit adder1(PC, four, PCplus4);
	regfile regRead(clk, instruction[25:21], instruction[20:16], writeAddr, writeRegData, rdData1, rdData2);
	mux16bit mux1(ALUSrc, instruction[15:0], rdData2, ALUIn);
	shiftLeft shift1(instruction[15:0], jumpAddr);
	ALU16bit alu1(rdData1, ALUIn, ALUOpCode, ALUResult, overflow);
	adder16bit adder2(PCplus4, jumpAddr, branchAddr);
	mux3to1 mux2(mux2Select, PCplus4, jumpAddr, branchAddr, temp_PC);
	dataMem data(clk, MemWrite, MemRead, ALUResult, rdData2, rdDataMem);
	mux16bit mux3(MemtoReg, rdDataMem, ALUResult, writeRegData);
	always @ ( posedge clk ) begin
		mux2Select = 2'b00;
		ALUSrc = 1'b1;
		MemtoReg = 1;
		funct = instruction[5:0];
		case (opcode)
			6'b00_0000 : begin //R-type instructions
											funct = instruction[5:0];
											PCSrc = 1'b0;
											ALUSrc = 1'b1;
											MemRead = 1'b0;
											MemWrite = 1'b0;
											MemtoReg = 1'b1;
											writeAddr = instruction[15:11];
									 end
			6'b00_1000 : begin //addi
											funct = 6'b10_0000;
											PCSrc = 1'b0;
											ALUSrc = 1'b0;
											MemRead = 1'b0;
											MemWrite = 1'b0;
											MemtoReg = 1'b1;
											writeAddr = instruction[20:16];
									 end
			6'b00_1100 : begin //andi
											funct = 6'b10_0100;
											PCSrc = 1'b0;
											ALUSrc = 1'b0;
											MemRead = 1'b0;
											MemWrite = 1'b0;
											MemtoReg = 1'b1;
											writeAddr = instruction[20:16];
									 end
			6'b00_1101 : begin //ori
											funct = 6'b10_0101;
											PCSrc = 1'b0;
											ALUSrc = 1'b0;
											MemRead = 1'b0;
											MemWrite = 1'b0;
											MemtoReg = 1'b1;
											writeAddr = instruction[20:16];
									 end
			6'b00_1111 : begin //lui
											funct = 6'b10_0000;
											PCSrc = 1'b0;
											ALUSrc = 1'b0;
											MemRead = 1'b0;
											MemWrite = 1'b0;
											MemtoReg = 1'b1;
											writeAddr = instruction[20:16];
									 end
				6'b00_1010 : begin //slti
											funct = 6'b10_1010;
											PCSrc = 1'b0;
											ALUSrc = 1'b0;
											MemRead = 1'b0;
											MemWrite = 1'b0;
											MemtoReg = 1'b1;
											writeAddr = instruction[20:16];
										 end
				6'b00_0100 : begin //beq
											funct = 6'b10_0010;
											PCSrc = 1'b0;
											ALUSrc = 1'b1;
											MemRead = 1'b0;
											MemWrite = 1'b0;
											MemtoReg = 1'b1;
											writeAddr = instruction[15:11];
										 end
				6'b00_0001 : begin //bqez
 										  funct = 6'b10_0010;
 											PCSrc = 1'b0;
 											ALUSrc = 1'b1;
 											MemRead = 1'b0;
 											MemWrite = 1'b0;
 											MemtoReg = 1'b1;
 											writeAddr = instruction[15:11];
 										 end
				6'b00_0010 : begin //j
				 							funct = 6'b10_0000;
				 							PCSrc = 1'b1;
				 							ALUSrc = 1'b1;
				 							MemRead = 1'b0;
				 							MemWrite = 1'b0;
				 							MemtoReg = 1'b1;
				 							writeAddr = instruction[15:11];
				 						 end
				6'b10_0000 : begin //lb
 											funct = 6'b10_0000;
 											PCSrc = 1'b0;
 											ALUSrc = 1'b0;
 											MemRead = 1'b1;
 											MemWrite = 1'b0;
 											MemtoReg = 1'b0;
 											writeAddr = instruction[20:16];
											half = 1'b0;
 										 end
			  6'b10_0001 : begin //lh
											funct = 6'b10_0000;
											PCSrc = 1'b0;
											ALUSrc = 1'b0;
											MemRead = 1'b1;
											MemWrite = 1'b0;
											MemtoReg = 1'b0;
											writeAddr = instruction[20:16];
											half = 1'b1;
										 end
				6'b10_1000 : begin //sb
 											funct = 6'b10_0000;
 											PCSrc = 1'b0;
 											ALUSrc = 1'b0;
 											MemRead = 1'b0;
 											MemWrite = 1'b1;
 											MemtoReg = 1'b0;
 											writeAddr = instruction[20:16];
 											half = 1'b0;
 										 end
			  6'b10_1001 : begin //sb
											funct = 6'b10_0000;
											PCSrc = 1'b0;
											ALUSrc = 1'b0;
											MemRead = 1'b0;
											MemWrite = 1'b1;
											MemtoReg = 1'b0;
											writeAddr = instruction[20:16];
											half = 1'b1;
										 end
		endcase
    case (funct)
      6'b10_0000 : begin ALUOpCode = 4'b0001; end
      6'b10_0010 : begin ALUOpCode = 4'b0101; end
      6'b10_0100 : begin ALUOpCode = 4'b0000; end
      6'b10_0111 : begin ALUOpCode = 4'b1000; end
      6'b10_0101 : begin ALUOpCode = 4'b0010; end
      6'b10_1010 : begin ALUOpCode = 4'b0111; end
      default : begin ALUOpCode = 4'b0000; end
    endcase
    $display("PC=%b instruction=%b writeRegData=%b",PC,instruction,writeRegData);
  end
	always @ ( negedge clk ) begin
		case (opcode)
			6'b00_0000 : begin //R-type instructions
											RegWrite = 1'b1;
									 end
			6'b00_1000 : begin //addi
											RegWrite = 1'b1;
									 end
			6'b00_1100 : begin //andi
											RegWrite = 1'b1;
									 end
			6'b00_1101 : begin //ori
											RegWrite = 1'b1;
									 end
			6'b00_1111 : begin //lui
											RegWrite = 1'b1;
									 end
				6'b00_1010 : begin //slti
											RegWrite = 1'b1;
										 end
				6'b00_0100 : begin //beq
											RegWrite = 1'b0;
										 end
				6'b00_0001 : begin //bqez
											RegWrite = 1'b0;
										 end
				6'b00_0010 : begin //j
											RegWrite = 1'b0;
										 end
				6'b10_0000 : begin //lb
											RegWrite = 1'b1;
										 end
				6'b10_0001 : begin //lh
											RegWrite = 1'b1;
										 end
				6'b10_1000 : begin //sb
											RegWrite = 1'b0;
										 end
				6'b10_1001 : begin //sb
											RegWrite = 1'b0;
										 end
		endcase
		PC = PCplus4;
	end
endmodule
