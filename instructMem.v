module instructMem(input clk, input [3:0] addr, output [31:0] instruction);
  reg [31:0] temp_instruct;
  assign instruction = temp_instruct;

  reg [7:0] instructions[0:72];

  initial begin
    $readmemb("instructions.txt",instructions);
  end

  always @ ( posedge clk ) begin
    temp_instruct[7:0] <= instructions[addr];
    temp_instruct[15:8] <= instructions[addr + 1];
    temp_instruct[23:16] <= instructions[addr + 2];
    temp_instruct[31:24] <= instructions[addr + 3];
  end

endmodule
