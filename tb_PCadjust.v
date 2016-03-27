`include "PCadjust.v"

module tb_PCadjust;
  reg [15:0] PC, jumpAddr;
  reg [1:0] select;
  wire [15:0] adjustedPC;

  PCadjust uut(.PC(PC),.jumpAddr(jumpAddr),.select(select),.adjustedPC(adjustedPC));
  initial begin
    PC = 16'b0000_0000_1000_0000;
    jumpAddr = 16'b1000_1000_0000_1000;
    select = 2'b00;
    #10
    select = 2'b01;
    #10
    select = 2'b10;
    #10
    select = 2'b00;
  end
  initial begin
    $monitor("PC=%b jumpAddr=%b select=%b adjustedPC=%b",PC, jumpAddr, select, adjustedPC);
  end
endmodule
