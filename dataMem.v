module dataMem(input clk, input write, input read, input [15:0] Addr, input [15:0] data_in, output [15:0] data_out);
  reg [15:0] temp_data;
  assign data_out = temp_data;

  reg [7:0] data[0:71];

  initial begin
    $readmemb("data.txt",data);
  end

  always @ ( posedge clk ) begin
    if (read == 1) begin
      temp_data[15:8] <= data[Addr];
      temp_data[7:0] <= data[Addr + 1];
    end
  end
  always @ ( negedge clk ) begin
    if (write == 1) begin
      data[Addr] <= data_in[15:8];
      data[Addr + 1] <= data_in[7:0];
    end
  end
endmodule
