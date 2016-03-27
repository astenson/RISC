`include "shiftLeft.v"

module tb_shiftLeft;
  reg [15:0] in;
  wire [15:0] out;

  shiftLeft uut(.in(in),.out(out));
  initial begin
    in = 16'b0000_0000_0000_0001;
    #10
    in = 16'b1100_1100_1111_0011;
  end
  initial begin
    $monitor("in=%b out=%b",in,out);
  end
endmodule
