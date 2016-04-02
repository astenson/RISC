`include "control.v"
`include "clock.v"

module tb_control;
  wire clk;
  wire [31:0] instruction;
  clock c(.clk(clk));
  control controlTest(.clk(clk),.instruction(instruction));
  initial begin
    $monitor("instruction=%b", instruction);
  end
endmodule
