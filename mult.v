`include "adder32bit.v"

module mult(input [15:0] A, input [15:0] B, output [31:0] result);
  wire [31:0] temp1, temp2, temp3, temp4, temp5, temp6, temp7, temp8, temp9, temp10, temp11, temp12, temp13, temp14, temp15, temp16;

  //For the first "muliplication"
  and(temp1[0],A[0],B[0]);
  and(temp1[1],A[1],B[0]);
  and(temp1[2],A[2],B[0]);
  and(temp1[3],A[3],B[0]);
  and(temp1[4],A[4],B[0]);
  and(temp1[5],A[5],B[0]);
  and(temp1[6],A[6],B[0]);
  and(temp1[7],A[7],B[0]);
  and(temp1[8],A[8],B[0]);
  and(temp1[9],A[9],B[0]);
  and(temp1[10],A[10],B[0]);
  and(temp1[11],A[11],B[0]);
  and(temp1[12],A[12],B[0]);
  and(temp1[13],A[13],B[0]);
  and(temp1[14],A[14],B[0]);
  and(temp1[15],A[15],B[0]);
  assign temp1[31:16] = 16'b0000_0000_0000_0000;
  //For the 2nd "muliplication"

endmodule
