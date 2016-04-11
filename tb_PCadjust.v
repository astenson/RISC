`include "PCadjust.v"
`include "clock.v"

module tb_PCadjust;
  reg [15:0] PC, jumpAddr;
  reg [1:0] select;
  wire [15:0] adjustedPC;
  wire clk;
  clock c(.clk(clk));
  PCadjust uut(.clk(clk),.PC(PC),.jumpAddr(jumpAddr),.select(select),.adjustedPC(adjustedPC));
  initial begin
    PC = 16'b0000_0000_1000_0000;
    jumpAddr = 16'b1000_1000_0000_1000;
    select = 2'b00;
  end
  initial begin
    #10 $display("time=%d PC=%b jumpAddr=%b select=%b adjustedPC=%b",$time,PC, jumpAddr, select, adjustedPC);

  end
endmodule
