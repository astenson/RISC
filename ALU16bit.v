`include "ALU1bit.v"

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
