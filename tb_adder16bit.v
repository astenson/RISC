`include "adder16bit.v"

module tb_adder16bit;
  reg [15:0] A, B;
  wire [15:0] result;

  adder16bit uut(.A(A),.B(B),.result(result));
  initial begin
    A = 16'b1111_1111_1111_1111;
    B = 16'b0000_0000_0000_0001;
    #10
    A = 16'b1010_1010_1010_1010;
    B = 16'b0101_0101_0101_0101;
  end
  initial begin
    $monitor("A=%b B=%b : result=%b",A,B,result);
  end
endmodule
