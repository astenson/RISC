`include "clock.v"
`include "regfile.v"

module tb_regfile;
  reg [3:0] rs, rt, rd;
  reg [15:0] data_in;
  wire [15:0] A, B;
  wire clk;

  clock c(.clk(clk));
  regfile uut(.clk(clk),.rs(rs),.rt(rt),.rd(rd),.data_in(data_in),.A(A),.B(B));
  initial begin
    #50
    rs = 4'b0000;
    rt = 4'b0001;
    rd = 4'b0010;
    data_in = 16'b1010_1010_1010_1010;
    #10
    rs = 4'b0010;
  end
  initial begin
    $monitor("rs=%b rt=%b rd=%b data in=%b : A=%b B=%b",rs,rt,rd,data_in,A,B);
  end
endmodule
