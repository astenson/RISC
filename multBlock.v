`include "full_adder.v"

module multBlock(input x, input y, input alt, input Cin, output S, output Cout);
  wire temp;
  and(temp, y, alt);
  full_adder add1(.x(temp),.y(y),.cin(Cin),.s(S),.cout(Cout));
endmodule
