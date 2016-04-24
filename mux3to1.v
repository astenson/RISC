module mux3to1_1bit(input [1:0] sel, input in1, input in2, input in3, output out);
  wire notSel1, notSel2, A, B, C;
  not(notSel1, sel[0]);
  not(notSel2, sel[1]);
  and(A, in1, notSel1, notSel2);
  and(B, in2, notSel2, sel[0]);
  and(C, in3, sel[1], notSel1);
  or(out, A, B, C);
endmodule

module mux3to1(input [1:0] sel, input [31:0] A, input [31:0] B, input [31:0] C, output [31:0] result);
  mux3to1_1bit mux1(sel, A[0], B[0], C[0], result[0]);
  mux3to1_1bit mux2(sel, A[1], B[1], C[1], result[1]);
  mux3to1_1bit mux3(sel, A[2], B[2], C[2], result[2]);
  mux3to1_1bit mux4(sel, A[3], B[3], C[3], result[3]);
  mux3to1_1bit mux5(sel, A[4], B[4], C[4], result[4]);
  mux3to1_1bit mux6(sel, A[5], B[5], C[5], result[5]);
  mux3to1_1bit mux7(sel, A[6], B[6], C[6], result[6]);
  mux3to1_1bit mux8(sel, A[7], B[7], C[7], result[7]);
  mux3to1_1bit mux9(sel, A[8], B[8], C[8], result[8]);
  mux3to1_1bit mux10(sel, A[9], B[9], C[9], result[9]);
  mux3to1_1bit mux11(sel, A[10], B[10], C[10], result[10]);
  mux3to1_1bit mux12(sel, A[11], B[11], C[11], result[11]);
  mux3to1_1bit mux13(sel, A[12], B[12], C[12], result[12]);
  mux3to1_1bit mux14(sel, A[13], B[13], C[13], result[13]);
  mux3to1_1bit mux15(sel, A[14], B[14], C[14], result[14]);
  mux3to1_1bit mux16(sel, A[15], B[15], C[15], result[15]);
  mux3to1_1bit mux17(sel, A[16], B[16], C[16], result[16]);
  mux3to1_1bit mux18(sel, A[17], B[17], C[17], result[17]);
  mux3to1_1bit mux19(sel, A[18], B[18], C[18], result[18]);
  mux3to1_1bit mux20(sel, A[19], B[19], C[19], result[19]);
  mux3to1_1bit mux21(sel, A[20], B[20], C[20], result[20]);
  mux3to1_1bit mux22(sel, A[21], B[21], C[21], result[21]);
  mux3to1_1bit mux23(sel, A[22], B[22], C[22], result[22]);
  mux3to1_1bit mux24(sel, A[23], B[23], C[23], result[23]);
  mux3to1_1bit mux25(sel, A[24], B[24], C[24], result[24]);
  mux3to1_1bit mux26(sel, A[25], B[25], C[25], result[25]);
  mux3to1_1bit mux27(sel, A[26], B[26], C[26], result[26]);
  mux3to1_1bit mux28(sel, A[27], B[27], C[27], result[27]);
  mux3to1_1bit mux29(sel, A[28], B[28], C[28], result[28]);
  mux3to1_1bit mux30(sel, A[29], B[29], C[29], result[29]);
  mux3to1_1bit mux31(sel, A[30], B[30], C[30], result[30]);
  mux3to1_1bit mux32(sel, A[31], B[31], C[31], result[31]);
endmodule

module tb_mux3to1;
  reg [31:0] A, B, C;
  reg [1:0] select;
  wire [31:0] result;

  mux3to1 uut(select, A, B, C, result);
  initial begin
    A = 32'b0000_0000_0000_0000_0000_0000_0000_0000;
    B = 32'b0000_0000_0000_0000_0000_0000_0000_1111;
    C = 32'b0001_0001_1111_0000_1111_0000_0000_0000;
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
