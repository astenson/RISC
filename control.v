//`include "adder16bit.v"
`include "ALU16bit.v"
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
  reg [4:0] rs, rt, rd;
  reg [15:0] data_in, A, B, Bmux;
  wire [15:0] tempA, tempB;
  wire clk, muxSel1, ALUOverflow;

  //The control lines
  reg RegWrite, ALUSrc, MemRead, MemWrite, MemtoReg, PCSrc;
  reg [7:0] funct;
  reg [3:0] ALUOpCode;
  wire [15:0] temp_ALUResult;
  reg [15:0] immediate, ALUResult;

  clock c(.clk(clk));
  instructMem uut1(.clk(clk),.addr(PC),.instruction(instruction));
  regfile uut2(.clk(clk),.rs(rs),.rt(rt),.rd(rd),.data_in(data_in),.A(tempA),.B(tempB));
  PCadjust uut3(.clk(clk),.PC(PC),.jumpAddr(jumpAddr),.select(select),.adjustedPC(adjustedPC));
  ALU16bit uut4(.A(A),.B(B),.control(ALUOpCode),.result(temp_ALUResult),.overflow(ALUOverflow));
  always @ ( posedge clk ) begin
    //PC = 16'b0000_0000_0000_0000;
    rd <= instruction[15:11];
    rt <= instruction[20:16];
    rs <= instruction[25:21];
    opcode = instruction[31:26];
    funct = instruction[5:0];
    immediate = instruction[15:0];
    select = 2'b00;
    jumpAddr = 16'b0000_0000_0000_0000;
    RegWrite = 1'b0;
    A = tempA;
    B = tempB;
    case (funct)
      8'h20: begin ALUOpCode = 4'b0001; end
      8'h22 : begin ALUOpCode = 4'b0101; end
      8'h24 : begin ALUOpCode = 4'b0000; end
      8'h27 : begin ALUOpCode = 4'b1000; end
      8'h25 : begin ALUOpCode = 4'b0010; end
      8'h2a : begin ALUOpCode = 4'b0111; end
    endcase
    ALUResult = temp_ALUResult;
    $display("A=%b B=%b control=%b: Result=%b",A,B,ALUOpCode,ALUResult);
    jumpAddr = 16'b0000_0000_0000_0000;
  end
  always @ ( negedge clk ) begin
    assign PC = adjustedPC;
  end
endmodule
