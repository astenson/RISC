`include "PCadjust.v"
`include "clock.v"

module tb_PCadjust;
  reg [15:0] PC, jumpAddr;
  reg [1:0] select;
  wire [15:0] adjustedPC;
  wire clk;

  /*initial begin
    PC = 16'b0000_0000_1000_0000;
    jumpAddr = 16'b1000_1000_0000_1000;
    select = 2'b00;
  end*/

  clock c(.clk(clk));
  PCadjust uut(.clk(clk),.PC(PC),.jumpAddr(jumpAddr),.select(select),.adjustedPC(adjustedPC));
  initial begin
    PC = 16'b0000_0000_1000_0000;
    jumpAddr = 16'b1000_1000_0000_1000;
    select = 2'b10;
  end
  initial begin
    #10 $monitor("PC=%b jumpAddr=%b select=%b adjustedPC=%b",PC, jumpAddr, select, adjustedPC);
    $finish;
  end
endmodule
