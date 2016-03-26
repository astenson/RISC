module clock(output clk);
  reg clk;
  always begin
    clk = 1'b0;
    forever #10 clk = ~clk;
  end
endmodule
