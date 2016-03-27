`include "clock.v"
`include "instructMem.v"

module tb_instructMem;
  reg [3:0] addr;
  wire [31:0] instruction;
  wire clk;

  clock c(.clk(clk));
  instructMem uut(.clk(clk),.addr(addr),.instruction(instruction));
  initial begin
    #50
    addr = 4'b0100;
  end
  initial begin
    #50 $monitor("addr=%b : instruction=%b",addr,instruction);
    #50 $finish;
  end
endmodule
