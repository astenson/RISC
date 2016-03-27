module dataMem(input clk, input write, input read, input [3:0] rdAddr, input [3:0] wrAddr, input [15:0] data_in, output [15:0] data_out);
  reg [31:0] temp_data;
  assign data_out = temp_data;

  reg [7:0] data[0:71];

  initial begin
    $readmemb("data.txt",data);
  end

  always @ ( posedge clk ) begin
    if (read == 1) begin
      temp_data[15:8] <= data[rdAddr];
      temp_data[7:0] <= data[rdAddr + 1];
    end
  end
  always @ ( negedge clk ) begin
    if (write == 1) begin
      data[wrAddr] <= data_in[15:8];
      data[wrAddr + 1] <= data_in[7:0];
    end
  end
endmodule
