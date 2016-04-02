`include "adder16bit.v"

module PCadjust(input clk, input [15:0] PC, input [15:0] jumpAddr, input [1:0] select, output [15:0] adjustedPC);
  wire [15:0] A, B;
  reg [15:0] four = 16'b0000_0000_0000_0100;
  reg [15:0] temp;
  assign adjustedPC = temp;
  adder16bit add1(.A(PC),.B(four),.result(A));
  adder16bit add2(.A(A),.B(jumpAddr),.result(B));
  always @ ( negedge clk ) begin
    case (select)
      2'b00: begin temp = A; end
      2'b01: begin temp = jumpAddr; end
      2'b10: begin temp = B; end
      default : $display("Error in select");
    endcase
  end

endmodule
