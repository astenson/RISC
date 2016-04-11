module mux3to1(sel, i1, i2, i3, o1);
  input [1:0] sel;
  input [15:0] i1, i2, i3;
  output [15:0] o1;
  reg [15:0] o1;

  always @(sel or i1 or i2 or i3) begin
    case (sel)
      2'b00: o1 = i1;
      2'b01: o1 = i2;
      2'b10: o1 = i3;
    endcase
  end
endmodule

module tb_mux3to1;
  reg [15:0] A, B, C;
  reg [1:0] select;
  wire [15:0] result;

  mux3to1 uut(select, A, B, C, result);
  initial begin
    A = 16'b0000_0000_0000_0000;
    B = 16'b0000_0000_0000_0001;
    C = 16'b0000_0000_0000_0010;
    select = 2'b00;
    #10
    select = 2'b01;
    #10
    select = 2'b10;
  end
  initial begin
    $monitor("time=%d",$time,"select=%b result=%d",select,result);
  end
endmodule
