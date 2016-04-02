module regfile(clk, rs, rt, rd, data_in, A, B);
	input [4:0] rs, rt, rd;
	input [15:0 ] data_in;
	output [15:0] A, B;
	input clk;

	wire [4:0] rs, rt, rd;
	wire [15:0] data_in;
	wire [15:0] A, B;

	reg [15:0] temp_A, temp_B;
	assign A = temp_A;
	assign B = temp_B;

	reg [15:0] register[0:15];

	initial begin
		$readmemb("registers.txt",register);
	end

  always @ ( posedge clk ) begin
    temp_A <= register[rs];
    temp_B <= register[rt];
  end

  always @ ( negedge clk ) begin
    register[rd] <= data_in;
  end
endmodule
