`include "clock.v"

module tb_clock;
  wire clk;

  clock uut(.clk(clk));
  initial begin
    $monitor("clk",clk);
  end
endmodule
