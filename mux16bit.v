module mux16bit(sel, i1, i2, o1);
  input sel;
  input [15:0] i1, i2;
  output [15:0] o1;
  reg [15:0] o1;
  always @(sel or i1 or i2) begin
    case (sel)
      1'b0: o1 = i1;
      1'b1: o1 = i2;
    endcase
  end
endmodule

module tb_mux16bit;
  reg [15:0] A, B;
  reg sel;
  wire [15:0] result;
  mux16bit uut(sel, A, B, result);
  initial begin
    #10
    A = 16'b1111_1111_1111_1111;
    B = 16'b0000_0000_0000_0000;
    sel = 1'b0;
    #10
    sel = 1'b1;
  end
  initial begin
    $monitor("A=%d B=%d sel=%d : result=%d", A, B, sel, result);
  end
endmodule
