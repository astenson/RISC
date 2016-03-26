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
