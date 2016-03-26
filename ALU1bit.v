`include "mux.v"
`include "full_adder.v"

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
