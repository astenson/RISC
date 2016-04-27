/*
Master ToDo list:
TODO add mux's before the mult and div units to decide between register input and shift amount
TODO extend the mux at the end of the datapath to accept wires from the high and low registers
TODO Implement division (div)
TODO Implement mfhi
TODO Implement mflo
TODO Implement jr
TODO Implement jal
TODO Write factorial assembly code
TODO Write factorial machine code into a new instruction memrory file
TODO fix branch and jump errors
*/

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

module multiply(input mult, input [15:0] A, input [15:0] B, output [15:0] high, output [15:0] low);
	reg [31:0] temp;
	always @ (mult) begin
		temp = A * B;
	end
	assign high = temp[31:16];
	assign low = temp[15:0];
endmodule

module division(input div, input [15:0] A, input [15:0] B, output [15:0] high, output [15:0] low);
	reg [31:0] temp;
	always @ (div) begin
		temp = A / B;
	end
	assign high = temp[31:16];
	assign low = temp[15:0];
endmodule

module sll(input [15:0] A, input [4:0] shamt, output [15:0] result);
	reg [15:0] temp;
	initial begin
		temp = A;
		repeat (shamt) begin
			temp[15:1] = temp[14:0];
			temp[0] = 1'b0;
		end
	end
	assign result = temp;
endmodule

module srl(input [15:0] A, input [4:0] shamt, output [15:0] result);
	reg [15:0] temp;
	initial begin
		temp = A;
		repeat (shamt) begin
			temp[14:0] = temp[15:1];
			temp[15] = 1'b0;
		end
	end
	assign result = temp;
endmodule

module clock(output clk);
  reg clk;
  initial begin
    repeat( 26 ) begin
			clk = 0;
      #100 clk = !clk;
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

module adder32bit(input [31:0] A, input [31:0] B, output [31:0] result);
  wire [32:0] carry;
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
  full_adder adder17(.x(A[16]),.y(B[16]),.cin(carry[16]),.s(result[16]),.cout(carry[17]));
	full_adder adder18(.x(A[17]),.y(B[17]),.cin(carry[17]),.s(result[17]),.cout(carry[18]));
	full_adder adder19(.x(A[18]),.y(B[18]),.cin(carry[18]),.s(result[18]),.cout(carry[19]));
	full_adder adder20(.x(A[19]),.y(B[19]),.cin(carry[19]),.s(result[19]),.cout(carry[20]));
	full_adder adder21(.x(A[20]),.y(B[20]),.cin(carry[20]),.s(result[20]),.cout(carry[21]));
	full_adder adder22(.x(A[21]),.y(B[21]),.cin(carry[21]),.s(result[21]),.cout(carry[22]));
	full_adder adder23(.x(A[22]),.y(B[22]),.cin(carry[22]),.s(result[22]),.cout(carry[23]));
	full_adder adder24(.x(A[23]),.y(B[23]),.cin(carry[23]),.s(result[23]),.cout(carry[24]));
	full_adder adder25(.x(A[24]),.y(B[24]),.cin(carry[24]),.s(result[24]),.cout(carry[25]));
	full_adder adder26(.x(A[25]),.y(B[25]),.cin(carry[25]),.s(result[25]),.cout(carry[26]));
	full_adder adder27(.x(A[26]),.y(B[26]),.cin(carry[26]),.s(result[26]),.cout(carry[27]));
	full_adder adder28(.x(A[27]),.y(B[27]),.cin(carry[27]),.s(result[27]),.cout(carry[28]));
	full_adder adder29(.x(A[28]),.y(B[28]),.cin(carry[28]),.s(result[28]),.cout(carry[29]));
	full_adder adder30(.x(A[29]),.y(B[29]),.cin(carry[29]),.s(result[29]),.cout(carry[30]));
	full_adder adder31(.x(A[30]),.y(B[30]),.cin(carry[30]),.s(result[30]),.cout(carry[31]));
	full_adder adder32(.x(A[31]),.y(B[31]),.cin(carry[31]),.s(result[31]),.cout(carry[32]));
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

module ALU16bit(input [15:0] A, input [15:0] B, input [3:0] control, input beq, input bqez, output [15:0] result, output zero);
		wire [14:0] carry;
		wire less = 1'b0;
		wire [16:0] set;
		wire temp1;
		reg [15:0] temp2;
		reg zero;

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
		//wire temp2;
		//xor(temp2, Cout, carry[14]);
		//and(overflow,temp2,control[1]);

		always @ (beq) begin
			temp2 = A - B;
			if (temp2 == 16'b0000_0000_0000_0000) begin
				zero = 1'b1;
			end else begin
				zero = 1'b0;
			end
		end

		always @ (bqez) begin
			if (A >= 16'b0000_0000_0000_0000) begin
				zero = 1'b1;
			end else begin
				zero = 1'b0;
			end
		end
endmodule

module dataMem(input write, input read, input [15:0] Addr, input [15:0] data_in, output [15:0] data_out, input half);
  reg [15:0] temp_data;
  assign data_out = temp_data;
	wire notHalf;

  reg [7:0] data[0:71];

  initial begin
    $readmemb("data.txt",data);
  end

  always @ ( read ) begin
    if (half == 1) begin
      temp_data[15:8] <= data[Addr];
      temp_data[7:0] <= data[Addr + 1];
    end else begin
			temp_data[7:0] <= data[Addr];
			temp_data[15:8] = 8'b0000_0000;
		end
  end
  always @ ( write ) begin
    if (half == 1) begin
      data[Addr] <= data_in[15:8];
      data[Addr + 1] <= data_in[7:0];
    end else begin
			data[Addr] <= data_in[7:0];
		end
  end
endmodule

module instructMem(input [31:0] addr, output [31:0] instruction);
  reg [31:0] temp_instruct;

  reg [7:0] instructions[0:103];

  initial begin
    $readmemh("instructions.txt",instructions);
  end
  always @* begin
    temp_instruct[31:24] <= instructions[addr];
    temp_instruct[23:16] <= instructions[addr + 1];
    temp_instruct[15:8] <= instructions[addr + 2];
    temp_instruct[7:0] <= instructions[addr + 3];
  end
	assign instruction = temp_instruct;

endmodule

module regfile(read, write, rs, rt, rd, data_in, A, B);
	input [4:0] rs, rt, rd;
	input [15:0 ] data_in;
	output [15:0] A, B;
	input read, write;

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

  always @ (write) begin
    temp_A <= register[rs];
    temp_B <= register[rt];
  end

  always @ (read) begin
    register[rd] <= data_in;
  end
endmodule

module shiftLeft(input [31:0] in, output [31:0] out);
  assign out[31:2] = in[29:0];
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

module mux6to1_16bit(sel, i1, i2, i3, i4, i5, i6, o1);
  input [2:0] sel;
  input [15:0] i1, i2, i3, i4, i5, i6;
  output [15:0] o1;
  reg [15:0] o1;
  always @(sel or i1 or i2 or i3 or i4 or i5 or i6) begin
    case (sel)
      3'b000: o1 = i1;
      3'b001: o1 = i2;
			3'b010: o1 = i3;
			3'b011: o1 = i4;
			3'b100: o1 = i5;
			3'b101: o1 = i6;
    endcase
  end
endmodule

module mux3to1_1bit(input [1:0] sel, input in1, input in2, input in3, output out);
  wire notSel1, notSel2, A, B, C;
  not(notSel1, sel[0]);
  not(notSel2, sel[1]);
  and(A, in1, notSel1, notSel2);
  and(B, in2, notSel2, sel[0]);
  and(C, in3, sel[1], notSel1);
  or(out, A, B, C);
endmodule

module mux3to1(input [1:0] sel, input [31:0] A, input [31:0] B, input [31:0] C, output [31:0] result);
  mux3to1_1bit mux1(sel, A[0], B[0], C[0], result[0]);
  mux3to1_1bit mux2(sel, A[1], B[1], C[1], result[1]);
  mux3to1_1bit mux3(sel, A[2], B[2], C[2], result[2]);
  mux3to1_1bit mux4(sel, A[3], B[3], C[3], result[3]);
  mux3to1_1bit mux5(sel, A[4], B[4], C[4], result[4]);
  mux3to1_1bit mux6(sel, A[5], B[5], C[5], result[5]);
  mux3to1_1bit mux7(sel, A[6], B[6], C[6], result[6]);
  mux3to1_1bit mux8(sel, A[7], B[7], C[7], result[7]);
  mux3to1_1bit mux9(sel, A[8], B[8], C[8], result[8]);
  mux3to1_1bit mux10(sel, A[9], B[9], C[9], result[9]);
  mux3to1_1bit mux11(sel, A[10], B[10], C[10], result[10]);
  mux3to1_1bit mux12(sel, A[11], B[11], C[11], result[11]);
  mux3to1_1bit mux13(sel, A[12], B[12], C[12], result[12]);
  mux3to1_1bit mux14(sel, A[13], B[13], C[13], result[13]);
  mux3to1_1bit mux15(sel, A[14], B[14], C[14], result[14]);
  mux3to1_1bit mux16(sel, A[15], B[15], C[15], result[15]);
  mux3to1_1bit mux17(sel, A[16], B[16], C[16], result[16]);
  mux3to1_1bit mux18(sel, A[17], B[17], C[17], result[17]);
  mux3to1_1bit mux19(sel, A[18], B[18], C[18], result[18]);
  mux3to1_1bit mux20(sel, A[19], B[19], C[19], result[19]);
  mux3to1_1bit mux21(sel, A[20], B[20], C[20], result[20]);
  mux3to1_1bit mux22(sel, A[21], B[21], C[21], result[21]);
  mux3to1_1bit mux23(sel, A[22], B[22], C[22], result[22]);
  mux3to1_1bit mux24(sel, A[23], B[23], C[23], result[23]);
  mux3to1_1bit mux25(sel, A[24], B[24], C[24], result[24]);
  mux3to1_1bit mux26(sel, A[25], B[25], C[25], result[25]);
  mux3to1_1bit mux27(sel, A[26], B[26], C[26], result[26]);
  mux3to1_1bit mux28(sel, A[27], B[27], C[27], result[27]);
  mux3to1_1bit mux29(sel, A[28], B[28], C[28], result[28]);
  mux3to1_1bit mux30(sel, A[29], B[29], C[29], result[29]);
  mux3to1_1bit mux31(sel, A[30], B[30], C[30], result[30]);
  mux3to1_1bit mux32(sel, A[31], B[31], C[31], result[31]);
endmodule

module padding16to32(input [15:0] in, output [31:0] out);
	assign out[15:0] = in;
	assign out[31:16] = 16'b0000_0000_0000_0000;
endmodule

module padding26to32(input [25:0] in, output [31:0] out);
	assign out[25:0] = in;
	assign out[31:26] = 6'b00_0000;
endmodule

module control;
	wire clk, zero;
	reg [1:0] mux2Select = 2'b00;
	wire [31:0] temp_PC;
  wire [15:0] rdData1, rdData2, ALUIn, writeRegData, ALUResult, rdDataMem, tempHigh, tempLow, sllResult, srlResult;
	reg [15:0] high, low;
	reg RegWrite, ALUSrc, MemRead, MemWrite, half, registerRead, registerWrite, beq;
	reg bqez, mult, div;
	reg [2:0] MemtoReg;
	reg [3:0] ALUOpCode;
	reg [31:0] PC = 32'b0000_0000_0000_0000_0000_0000_0000_0000;
	reg [31:0] four = 32'b0000_0000_0000_0000_0000_0000_0000_0100;
	wire [31:0] instruction, shiftedJA, shiftedBA, PCplus4, tempJA, tempBA, branchAddr;
	reg [5:0] opcode, funct;
	reg [4:0] writeAddr;

	clock clk1(.clk(clk));
	padding16to32 padding1(instruction[15:0], tempBA);
	padding26to32 padding2(instruction[25:0], tempJA);
	shiftLeft shift1(tempJA, shiftedJA);
	shiftLeft shift2(tempBA, shiftedBA);
	adder32bit adder2(PCplus4, shiftedBA, branchAddr);
	instructMem instruct(PC, instruction);
	adder32bit adder1(PC, four, PCplus4);
	regfile regRead(registerRead, RegWrite, instruction[25:21], instruction[20:16], writeAddr, writeRegData, rdData1, rdData2);
	mux16bit mux1(ALUSrc, instruction[15:0], rdData2, ALUIn);
	multiply mult1(mult, rdData1, rdData2, tempHigh, tempLow);
	division div1(div, rdData1, rdData2, tempHigh, tempLow);
	sll sll1(rdData2, instruction[10:6], sllResult);
	srl srl1(rdData2, instruction[10:6], srlResult);
	ALU16bit alu1(rdData1, ALUIn, ALUOpCode, beq, bqez, ALUResult, zero);
	dataMem data(MemWrite, MemRead, ALUResult, rdData2, rdDataMem, half);
	mux6to1_16bit mux3(MemtoReg, rdDataMem, ALUResult, high, low, sllResult, srlResult, writeRegData);
	mux3to1 mux2(mux2Select, PCplus4, shiftedJA, branchAddr, temp_PC);
	always @ ( posedge clk ) begin
		ALUSrc = 1'b1;
		MemtoReg = 1;
		funct = instruction[5:0];
		opcode = instruction[31:25];
		case (opcode)
			6'b00_0000 : begin //R-type instructions
											funct = instruction[5:0];
											ALUSrc = 1'b1;
											MemRead = 1'b0;
											MemWrite = 1'b0;
											MemtoReg = 3'b001;
											writeAddr = instruction[15:11];
											mux2Select = 2'b00;
											registerRead = 1'b1;
											case (funct)
									      6'b10_0000 : begin ALUOpCode = 4'b0001; mult = 1'b0; div = 1'b0; end
									      6'b10_0010 : begin ALUOpCode = 4'b0101; mult = 1'b0; div = 1'b0; end
									      6'b10_0100 : begin ALUOpCode = 4'b0000; mult = 1'b0; div = 1'b0; end
									      6'b10_0111 : begin ALUOpCode = 4'b1000; mult = 1'b0; div = 1'b0; end
									      6'b10_0101 : begin ALUOpCode = 4'b0010; mult = 1'b0; div = 1'b0; end
									      6'b10_1010 : begin ALUOpCode = 4'b0111; mult = 1'b0; div = 1'b0; end
												6'b01_1010 : begin //div
																			ALUOpCode = 4'b0001;
																			ALUSrc = 1'b0;
																			MemRead = 1'b0;
																			MemWrite = 1'b0;
																			MemtoReg = 1'b0;
																			writeAddr = instruction[20:16];
																			mux2Select = 2'b00;
																			registerRead = 1'b1;
																			RegWrite = 1'b0;
																			assign high = tempHigh;
																			assign low = tempLow;
																			div = 1'b1;
																		 end
												6'b01_1000 : begin //mult
																			ALUOpCode = 4'b0001;
																			ALUSrc = 1'b0;
																			MemRead = 1'b0;
																			MemWrite = 1'b0;
																			MemtoReg = 1'b0;
																			writeAddr = instruction[20:16];
																			mux2Select = 2'b00;
																			registerRead = 1'b1;
																			RegWrite = 1'b0;
																			assign high = tempHigh;
																			assign low = tempLow;
																			mult = 1'b1;
																		 end
											 6'b01_0000 : begin //mfhi
																			ALUOpCode = 4'b0001;
																			ALUSrc = 1'b0;
																			MemRead = 1'b0;
																			MemWrite = 1'b0;
																			MemtoReg = 1'b0;
																			writeAddr = instruction[20:16];
																			mux2Select = 2'b00;
																			registerRead = 1'b0;
																			RegWrite = 1'b1;
																		 end
											 6'b01_0010 : begin //mflo
																			ALUOpCode = 4'b0001;
																			ALUSrc = 1'b0;
																			MemRead = 1'b0;
																			MemWrite = 1'b0;
																			MemtoReg = 1'b0;
																			writeAddr = instruction[20:16];
																			mux2Select = 2'b00;
																			registerRead = 1'b0;
																			RegWrite = 1'b1;
																		 end
									      default : begin ALUOpCode = 4'b0000; end
									    endcase
									 end
			6'b00_1000 : begin //addi
											funct = 6'b10_0000;
											ALUSrc = 1'b0;
											MemRead = 1'b0;
											MemWrite = 1'b0;
											MemtoReg = 3'b001;
											writeAddr = instruction[20:16];
											mux2Select = 2'b00;
											registerRead = 1'b1;
											mult = 1'b0;
											div = 1'b0;
									 end
			6'b00_1100 : begin //andi
											funct = 6'b10_0100;
											ALUSrc = 1'b0;
											MemRead = 1'b0;
											MemWrite = 1'b0;
											MemtoReg = 3'b001;
											writeAddr = instruction[20:16];
											mux2Select = 2'b00;
											registerRead = 1'b1;
											mult = 1'b0;
											div = 1'b0;
									 end
			6'b00_1101 : begin //ori
											funct = 6'b10_0101;
											ALUSrc = 1'b0;
											MemRead = 1'b0;
											MemWrite = 1'b0;
											MemtoReg = 3'b001;
											writeAddr = instruction[20:16];
											mux2Select = 2'b00;
											registerRead = 1'b1;
											mult = 1'b0;
											div = 1'b0;
									 end
			6'b00_1111 : begin //lui
											funct = 6'b10_0000;
											ALUSrc = 1'b0;
											MemRead = 1'b0;
											MemWrite = 1'b0;
											MemtoReg = 3'b001;
											writeAddr = instruction[20:16];
											mux2Select = 2'b00;
											registerRead = 1'b1;
											mult = 1'b0;
											div = 1'b0;
									 end
				6'b00_1010 : begin //slti
											funct = 6'b10_1010;
											ALUSrc = 1'b0;
											MemRead = 1'b0;
											MemWrite = 1'b0;
											MemtoReg = 3'b001;
											writeAddr = instruction[20:16];
											mux2Select = 2'b00;
											registerRead = 1'b1;
											mult = 1'b0;
											div = 1'b0;
										 end
				6'b00_0100 : begin //beq
											funct = 6'b10_0010;
											ALUSrc = 1'b1;
											MemRead = 1'b0;
											MemWrite = 1'b0;
											MemtoReg = 3'b001;
											writeAddr = instruction[15:11];
											//mux2Select[0] = 1'b0;
											registerRead = 1'b1;
											beq = 1'b1;
											//mux2Select[1] = zero;
											mult = 1'b0;
											div = 1'b0;
										 end
				6'b00_0001 : begin //bqez
 										  funct = 6'b10_0010;
 											ALUSrc = 1'b1;
 											MemRead = 1'b0;
 											MemWrite = 1'b0;
 											MemtoReg = 3'b001;
 											writeAddr = instruction[15:11];
											//mux2Select[0] = 1'b0;
											registerRead = 1'b1;
											bqez = 1'b1;
											//mux2Select[1] = zero;
											mult = 1'b0;
											div = 1'b0;
 										 end
				6'b00_0010 : begin //j
				 							funct = 6'b10_0000;
				 							ALUSrc = 1'b1;
				 							MemRead = 1'b0;
				 							MemWrite = 1'b0;
				 							MemtoReg = 3'b001;
				 							writeAddr = instruction[15:11];
											mux2Select = 2'b01;
											registerRead = 1'b0;
											mult = 1'b0;
											div = 1'b0;
				 						 end
				6'b10_0000 : begin //lb
 											funct = 6'b10_0000;
 											ALUSrc = 1'b0;
 											MemRead = 1'b1;
 											MemWrite = 1'b0;
 											MemtoReg = 3'b000;
 											writeAddr = instruction[20:16];
											half = 1'b0;
											mux2Select = 2'b00;
											registerRead = 1'b1;
											mult = 1'b0;
											div = 1'b0;
 										 end
			  6'b10_0001 : begin //lh
											funct = 6'b10_0000;
											ALUSrc = 1'b0;
											MemRead = 1'b1;
											MemWrite = 1'b0;
											MemtoReg = 3'b000;
											writeAddr = instruction[20:16];
											half = 1'b1;
											mux2Select = 2'b00;
											registerRead = 1'b1;
											mult = 1'b0;
											div = 1'b0;
										 end
				6'b10_1000 : begin //sb
 											funct = 6'b10_0000;
 											ALUSrc = 1'b0;
 											MemRead = 1'b0;
 											MemWrite = 1'b1;
 											MemtoReg = 3'b000;
 											writeAddr = instruction[20:16];
 											half = 1'b0;
											mux2Select = 2'b00;
											registerRead = 1'b1;
											mult = 1'b0;
											div = 1'b0;
 										 end
			  6'b10_1001 : begin //sh
											funct = 6'b10_0000;
											ALUSrc = 1'b0;
											MemRead = 1'b0;
											MemWrite = 1'b1;
											MemtoReg = 3'b000;
											writeAddr = instruction[20:16];
											half = 1'b1;
											mux2Select = 2'b00;
											registerRead = 1'b1;
											mult = 1'b0;
											div = 1'b0;
										 end
		endcase
		$display("PC=%b instruction=%h opcode=%b writeRegData=%b",PC,instruction,opcode,writeRegData);
		PC =temp_PC;
  end
	always @ ( negedge clk ) begin
		case (opcode)
			6'b00_0000 : begin end
			6'b00_1000 : begin RegWrite = 1'b1; end
			6'b00_1100 : begin RegWrite = 1'b1; end
			6'b00_1101 : begin RegWrite = 1'b1; end
			6'b00_1111 : begin RegWrite = 1'b1; end
			6'b00_1010 : begin RegWrite = 1'b1; end
			6'b00_0100 : begin RegWrite = 1'b0; end
			6'b00_0001 : begin RegWrite = 1'b0; end
			6'b00_0010 : begin RegWrite = 1'b0; end
			6'b10_0000 : begin RegWrite = 1'b1; end
			6'b10_0001 : begin RegWrite = 1'b1; end
			6'b10_1000 : begin RegWrite = 1'b0; end
			6'b10_1001 : begin RegWrite = 1'b0; end
		endcase
		case (funct)
      6'b10_0000 : begin RegWrite = 1'b1; end
      6'b10_0010 : begin RegWrite = 1'b1; end
      6'b10_0100 : begin RegWrite = 1'b1; end
      6'b10_0111 : begin RegWrite = 1'b1; end
      6'b10_0101 : begin RegWrite = 1'b1; end
      6'b10_1010 : begin RegWrite = 1'b1; end
			6'b01_1010 : begin RegWrite = 1'b0; end
			6'b01_1000 : begin RegWrite = 1'b0; end
			6'b01_0000 : begin RegWrite = 1'b1; end
			6'b01_0010 : begin RegWrite = 1'b1; end
			6'b00_0000 : begin RegWrite = 1'b1; end
			6'b00_0010 : begin RegWrite = 1'b1; end
			6'b00_1000 : begin RegWrite = 1'b0; end
      default : begin RegWrite = 1'b0; end
    endcase
	end
endmodule
