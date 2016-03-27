`include "dataMem.v"
`include "clock.v"

module tb_dataMem;
  reg write, read;
  reg [3:0] rdAddr, wrAddr;
  reg [15:0] data_in;
  wire [15:0] data_out;
  wire clk;

  clock c(.clk(clk));
  dataMem uut(.clk(clk),.write(write),.read(read),.rdAddr(rdAddr),.wrAddr(wrAddr),.data_in(data_in),.data_out(data_out));
  initial begin
    write = 1'b0;
    read = 1'b1;
    rdAddr = 4'b0000;
  end
  initial begin
    $monitor("%b",data_out);
  end
endmodule
