`include "mult.v"

module tb_mult;
  reg [15:0] A, B;
  wire [31:0] result;

  mult uut(.A(A),.B(B),.result(result));
  initial begin
    A = 16'b1010_1010_1010_1010;
    B = 16'b1111_1111_1111_1111;
    #100
    A = 16'b0000_0000_0000_0010;
    B = 16'b0000_0000_0000_0100;
  end
  initial begin
    #100 $monitor("A=%b B=%b : result=%b",A,B,result);
  end
endmodule
