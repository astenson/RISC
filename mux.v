module mux(input A, input B, input sel, output out);
  wire C;
	not(C,sel);
	wire D, E;
	and(D,A,C);
	and(E,B,sel);
	or(out,D,E);
endmodule
