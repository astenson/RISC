//`include "adder16bit.v"
//`include "ALU16bit.v"
`include "PCadjust.v"
//`include "dataMem.v"
`include "instructMem.v"
//`include "mux.v"
`include "regfile.v"
`include "clock.v"


module control(input clk);
  wire [31:0] instruction;
  reg [15:0] PC = 16'b0000_0000_0000_0000;
  wire [15:0] adjustedPC;
  reg [1:0] select;
  reg [15:0] jumpAddr;
  reg [5:0] opcode;
  reg [3:0] rs, rt, rd;
  reg [15:0] data_in, A, B, Bmux;
  wire [15:0] tempA, tempB;
  wire clk, muxSel1;

  //The control lines
  reg RegWrite, ALUSrc, MemRead, MemWrite, MemtoReg, PCSrc;
  reg [5:0] ALUOperation;
  reg [15:0] immediate;

  clock c(.clk(clk));
  instructMem uut1(.clk(clk),.addr(PC),.instruction(instruction));
  PCadjust uut2(.clk(clk),.PC(PC),.jumpAddr(jumpAddr),.select(select),.adjustedPC(adjustedPC));
  regfile uut3(.clk(clk),.rs(rs),.rt(rt),.rd(rd),.data_in(data_in),.A(tempA),.B(tempB));
  always @ ( posedge clk ) begin
    //PC = 16'b0000_0000_0000_0000;
    select = 2'b00;
    jumpAddr = 16'b0000_0000_0000_0000;
    RegWrite = 1'b0;
    rd = instruction[14:11];
    rt = instruction[19:16];
    rs = instruction[24:21];
    opcode = instruction[31:25];
    ALUOperation = instruction[5:0];
    immediate = instruction[15:0];
    A = tempA;
    B = tempB;
    $display("A=%b B=%b",A,B);
  end
  always @ ( negedge clk ) begin
    assign PC = adjustedPC;
  end

endmodule
