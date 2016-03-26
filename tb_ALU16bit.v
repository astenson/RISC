`include "ALU16bit.v"

module tb_ALU16bit;
  reg [15:0] A, B;
  reg [3:0] control;
  wire [15:0] result;
  wire overflow;

  ALU16bit uut(.A(A),.B(B),.control(control),.result(result),.overflow(overflow));
  initial begin
    A = 16'b0000_1100_0011_0101;
    B = 16'b0000_0000_1100_1010;
    control = 4'b0000;
    #10
    control = 4'b0001;
    #10
    control = 4'b0010;
    #10
    control = 4'b0110;
    #10
    control = 4'b0111;
    #10
    control = 4'b1100;
    #10
    A = 16'b1111_1100_0011_0101;
    B = 16'b1000_0000_1100_1010;
    control = 4'b0000;
    #10
    control = 4'b0001;
    #10
    control = 4'b0010;
    #10
    control = 4'b0110;
    #10
    control = 4'b0111;
    #10
    control = 4'b1100;
  end
  initial begin
    $monitor("A=%b B=%b control=%b : result=%b overflow=%b",A,B,control,result,overflow);
  end
endmodule
