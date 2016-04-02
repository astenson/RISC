module clock(output clk);
  reg clk;
  initial begin
    repeat( 18 ) begin
      clk = 1'b0;
      #1000 clk = ~clk;
    end
  end
endmodule
