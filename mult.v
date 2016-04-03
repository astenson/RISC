`include "adder32bit.v"
`include "multBlock.v"

module mult(input [15:0] A, input [15:0] B, output [31:0] result);
  wire zero = 1'b0;
  wire [15:0] carry0, carry1, carry2, carry3, carry4, carry5, carry6, carry7, carry8, carry9, carry10, carry11, carry12, carry13, carry14, carry15;
  wire [14:0] add0, add1, add2, add3, add4, add5, add6, add7, add8, add9, add10, add11, add12, add13, add14, add15;
  //1st row of multiplier blocks
  multBlock mult00(.x(A[0]),.y(zero),.alt(B[0]),.Cin(zero),.S(result[0]),.Cout(carry0[0]));
  multBlock mult01(.x(A[1]),.y(zero),.alt(B[0]),.Cin(carry0[0]),.S(add0[0]),.Cout(carry0[1]));
  multBlock mult02(.x(A[2]),.y(zero),.alt(B[0]),.Cin(carry0[1]),.S(add0[1]),.Cout(carry0[2]));
  multBlock mult03(.x(A[3]),.y(zero),.alt(B[0]),.Cin(carry0[2]),.S(add0[2]),.Cout(carry0[3]));
  multBlock mult04(.x(A[4]),.y(zero),.alt(B[0]),.Cin(carry0[3]),.S(add0[3]),.Cout(carry0[4]));
  multBlock mult05(.x(A[5]),.y(zero),.alt(B[0]),.Cin(carry0[4]),.S(add0[4]),.Cout(carry0[5]));
  multBlock mult06(.x(A[6]),.y(zero),.alt(B[0]),.Cin(carry0[5]),.S(add0[5]),.Cout(carry0[6]));
  multBlock mult07(.x(A[7]),.y(zero),.alt(B[0]),.Cin(carry0[6]),.S(add0[6]),.Cout(carry0[7]));
  multBlock mult08(.x(A[8]),.y(zero),.alt(B[0]),.Cin(carry0[7]),.S(add0[7]),.Cout(carry0[8]));
  multBlock mult09(.x(A[9]),.y(zero),.alt(B[0]),.Cin(carry0[8]),.S(add0[8]),.Cout(carry0[9]));
  multBlock mult010(.x(A[10]),.y(zero),.alt(B[0]),.Cin(carry0[9]),.S(add0[9]),.Cout(carry0[10]));
  multBlock mult011(.x(A[11]),.y(zero),.alt(B[0]),.Cin(carry0[10]),.S(add0[10]),.Cout(carry0[11]));
  multBlock mult012(.x(A[12]),.y(zero),.alt(B[0]),.Cin(carry0[11]),.S(add0[11]),.Cout(carry0[12]));
  multBlock mult013(.x(A[13]),.y(zero),.alt(B[0]),.Cin(carry0[12]),.S(add0[12]),.Cout(carry0[13]));
  multBlock mult014(.x(A[14]),.y(zero),.alt(B[0]),.Cin(carry0[13]),.S(add0[13]),.Cout(carry0[14]));
  multBlock mult015(.x(A[15]),.y(zero),.alt(B[0]),.Cin(carry0[14]),.S(add0[14]),.Cout(carry0[15]));
  //2nd row of multiplier blocks
  multBlock mult10(.x(A[0]),.y(add0[0]),.alt(B[1]),.Cin(zero),.S(result[1]),.Cout(carry1[0]));
  multBlock mult11(.x(A[1]),.y(add0[1]),.alt(B[1]),.Cin(carry1[0]),.S(add1[0]),.Cout(carry1[1]));
  multBlock mult12(.x(A[2]),.y(add0[2]),.alt(B[1]),.Cin(carry1[1]),.S(add1[1]),.Cout(carry1[2]));
  multBlock mult13(.x(A[3]),.y(add0[3]),.alt(B[1]),.Cin(carry1[2]),.S(add1[2]),.Cout(carry1[3]));
  multBlock mult14(.x(A[4]),.y(add0[4]),.alt(B[1]),.Cin(carry1[3]),.S(add1[3]),.Cout(carry1[4]));
  multBlock mult15(.x(A[5]),.y(add0[5]),.alt(B[1]),.Cin(carry1[4]),.S(add1[4]),.Cout(carry1[5]));
  multBlock mult16(.x(A[6]),.y(add0[6]),.alt(B[1]),.Cin(carry1[5]),.S(add1[5]),.Cout(carry1[6]));
  multBlock mult17(.x(A[7]),.y(add0[7]),.alt(B[1]),.Cin(carry1[6]),.S(add1[6]),.Cout(carry1[7]));
  multBlock mult18(.x(A[8]),.y(add0[8]),.alt(B[1]),.Cin(carry1[7]),.S(add1[7]),.Cout(carry1[8]));
  multBlock mult19(.x(A[9]),.y(add0[9]),.alt(B[1]),.Cin(carry1[8]),.S(add1[8]),.Cout(carry1[9]));
  multBlock mult110(.x(A[10]),.y(add0[10]),.alt(B[1]),.Cin(carry1[9]),.S(add1[9]),.Cout(carry1[10]));
  multBlock mult111(.x(A[11]),.y(add0[11]),.alt(B[1]),.Cin(carry1[10]),.S(add1[10]),.Cout(carry1[11]));
  multBlock mult112(.x(A[12]),.y(add0[12]),.alt(B[1]),.Cin(carry1[11]),.S(add1[11]),.Cout(carry1[12]));
  multBlock mult113(.x(A[13]),.y(add0[13]),.alt(B[1]),.Cin(carry1[12]),.S(add1[12]),.Cout(carry1[13]));
  multBlock mult114(.x(A[14]),.y(add0[14]),.alt(B[1]),.Cin(carry1[13]),.S(add1[13]),.Cout(carry1[14]));
  multBlock mult115(.x(A[15]),.y(carry0[15]),.alt(B[1]),.Cin(carry1[14]),.S(add1[14]),.Cout(carry1[15]));
  //3rd row of multiplier blocks
  multBlock mult20(.x(A[0]),.y(add1[0]),.alt(B[2]),.Cin(zero),.S(result[2]),.Cout(carry2[0]));
  multBlock mult21(.x(A[1]),.y(add1[1]),.alt(B[2]),.Cin(carry2[0]),.S(add2[0]),.Cout(carry2[1]));
  multBlock mult22(.x(A[2]),.y(add1[2]),.alt(B[2]),.Cin(carry2[1]),.S(add2[1]),.Cout(carry2[2]));
  multBlock mult23(.x(A[3]),.y(add1[3]),.alt(B[2]),.Cin(carry2[2]),.S(add2[2]),.Cout(carry2[3]));
  multBlock mult24(.x(A[4]),.y(add1[4]),.alt(B[2]),.Cin(carry2[3]),.S(add2[3]),.Cout(carry2[4]));
  multBlock mult25(.x(A[5]),.y(add1[5]),.alt(B[2]),.Cin(carry2[4]),.S(add2[4]),.Cout(carry2[5]));
  multBlock mult26(.x(A[6]),.y(add1[6]),.alt(B[2]),.Cin(carry2[5]),.S(add2[5]),.Cout(carry2[6]));
  multBlock mult27(.x(A[7]),.y(add1[7]),.alt(B[2]),.Cin(carry2[6]),.S(add2[6]),.Cout(carry2[7]));
  multBlock mult28(.x(A[8]),.y(add1[8]),.alt(B[2]),.Cin(carry2[7]),.S(add2[7]),.Cout(carry2[8]));
  multBlock mult29(.x(A[9]),.y(add1[9]),.alt(B[2]),.Cin(carry2[8]),.S(add2[8]),.Cout(carry2[9]));
  multBlock mult210(.x(A[10]),.y(add1[10]),.alt(B[2]),.Cin(carry2[9]),.S(add2[9]),.Cout(carry2[10]));
  multBlock mult211(.x(A[11]),.y(add1[11]),.alt(B[2]),.Cin(carry2[10]),.S(add2[10]),.Cout(carry2[11]));
  multBlock mult212(.x(A[12]),.y(add1[12]),.alt(B[2]),.Cin(carry2[11]),.S(add2[11]),.Cout(carry2[12]));
  multBlock mult213(.x(A[13]),.y(add1[13]),.alt(B[2]),.Cin(carry2[12]),.S(add2[12]),.Cout(carry2[13]));
  multBlock mult214(.x(A[14]),.y(add1[14]),.alt(B[2]),.Cin(carry2[13]),.S(add2[13]),.Cout(carry2[14]));
  multBlock mult215(.x(A[15]),.y(carry1[15]),.alt(B[2]),.Cin(carry2[14]),.S(add2[14]),.Cout(carry2[15]));
  //4th row of multiplier blocks
  multBlock mult30(.x(A[0]),.y(add2[0]),.alt(B[3]),.Cin(zero),.S(result[2]),.Cout(carry3[0]));
  multBlock mult31(.x(A[1]),.y(add2[1]),.alt(B[3]),.Cin(carry1[0]),.S(add3[0]),.Cout(carry3[1]));
  multBlock mult32(.x(A[2]),.y(add2[2]),.alt(B[3]),.Cin(carry1[1]),.S(add3[1]),.Cout(carry3[2]));
  multBlock mult33(.x(A[3]),.y(add2[3]),.alt(B[3]),.Cin(carry1[2]),.S(add3[2]),.Cout(carry3[3]));
  multBlock mult34(.x(A[4]),.y(add2[4]),.alt(B[3]),.Cin(carry1[3]),.S(add3[3]),.Cout(carry3[4]));
  multBlock mult35(.x(A[5]),.y(add2[5]),.alt(B[3]),.Cin(carry1[4]),.S(add3[4]),.Cout(carry3[5]));
  multBlock mult36(.x(A[6]),.y(add2[6]),.alt(B[3]),.Cin(carry1[5]),.S(add3[5]),.Cout(carry3[6]));
  multBlock mult37(.x(A[7]),.y(add2[7]),.alt(B[3]),.Cin(carry1[6]),.S(add3[6]),.Cout(carry3[7]));
  multBlock mult38(.x(A[8]),.y(add2[8]),.alt(B[3]),.Cin(carry1[7]),.S(add3[7]),.Cout(carry3[8]));
  multBlock mult39(.x(A[9]),.y(add2[9]),.alt(B[3]),.Cin(carry1[8]),.S(add3[8]),.Cout(carry3[9]));
  multBlock mult310(.x(A[10]),.y(add2[10]),.alt(B[3]),.Cin(carry1[9]),.S(add3[9]),.Cout(carry3[10]));
  multBlock mult311(.x(A[11]),.y(add2[11]),.alt(B[3]),.Cin(carry1[10]),.S(add3[10]),.Cout(carry3[11]));
  multBlock mult312(.x(A[12]),.y(add2[12]),.alt(B[3]),.Cin(carry1[11]),.S(add3[11]),.Cout(carry3[12]));
  multBlock mult313(.x(A[13]),.y(add2[13]),.alt(B[3]),.Cin(carry1[12]),.S(add3[12]),.Cout(carry3[13]));
  multBlock mult314(.x(A[14]),.y(add2[14]),.alt(B[3]),.Cin(carry1[13]),.S(add3[13]),.Cout(carry3[14]));
  multBlock mult315(.x(A[15]),.y(carry2[15]),.alt(B[3]),.Cin(carry1[14]),.S(add3[14]),.Cout(carry3[15]));
endmodule
