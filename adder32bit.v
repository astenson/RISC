`include "full_adder.v"

module adder32bit(input [31:0] A, input [31:0] B, output [31:0] result);
  wire [32:0] carry;
  assign carry[0] = 0;
  full_adder adder1(.x(A[0]),.y(B[0]),.cin(carry[0]),.s(result[0]),.cout(carry[1]));
  full_adder adder2(.x(A[1]),.y(B[1]),.cin(carry[1]),.s(result[1]),.cout(carry[2]));
  full_adder adder3(.x(A[2]),.y(B[2]),.cin(carry[2]),.s(result[2]),.cout(carry[3]));
  full_adder adder4(.x(A[3]),.y(B[3]),.cin(carry[3]),.s(result[3]),.cout(carry[4]));
  full_adder adder5(.x(A[4]),.y(B[4]),.cin(carry[4]),.s(result[4]),.cout(carry[5]));
  full_adder adder6(.x(A[5]),.y(B[5]),.cin(carry[5]),.s(result[5]),.cout(carry[6]));
  full_adder adder7(.x(A[6]),.y(B[6]),.cin(carry[6]),.s(result[6]),.cout(carry[7]));
  full_adder adder8(.x(A[7]),.y(B[7]),.cin(carry[7]),.s(result[7]),.cout(carry[8]));
  full_adder adder9(.x(A[8]),.y(B[8]),.cin(carry[8]),.s(result[8]),.cout(carry[9]));
  full_adder adder10(.x(A[9]),.y(B[9]),.cin(carry[9]),.s(result[9]),.cout(carry[10]));
  full_adder adder11(.x(A[10]),.y(B[10]),.cin(carry[10]),.s(result[10]),.cout(carry[11]));
  full_adder adder12(.x(A[11]),.y(B[11]),.cin(carry[11]),.s(result[11]),.cout(carry[12]));
  full_adder adder13(.x(A[12]),.y(B[12]),.cin(carry[12]),.s(result[12]),.cout(carry[13]));
  full_adder adder14(.x(A[13]),.y(B[13]),.cin(carry[13]),.s(result[13]),.cout(carry[14]));
  full_adder adder15(.x(A[14]),.y(B[14]),.cin(carry[14]),.s(result[14]),.cout(carry[15]));
  full_adder adder16(.x(A[15]),.y(B[15]),.cin(carry[15]),.s(result[15]),.cout(carry[16]));
  full_adder adder17(.x(A[16]),.y(B[16]),.cin(carry[16]),.s(result[16]),.cout(carry[17]));
  full_adder adder18(.x(A[17]),.y(B[17]),.cin(carry[17]),.s(result[17]),.cout(carry[18]));
  full_adder adder19(.x(A[18]),.y(B[18]),.cin(carry[18]),.s(result[18]),.cout(carry[19]));
  full_adder adder20(.x(A[19]),.y(B[19]),.cin(carry[19]),.s(result[19]),.cout(carry[20]));
  full_adder adder21(.x(A[20]),.y(B[20]),.cin(carry[20]),.s(result[20]),.cout(carry[21]));
  full_adder adder22(.x(A[21]),.y(B[21]),.cin(carry[21]),.s(result[21]),.cout(carry[22]));
  full_adder adder23(.x(A[22]),.y(B[22]),.cin(carry[22]),.s(result[22]),.cout(carry[23]));
  full_adder adder24(.x(A[23]),.y(B[23]),.cin(carry[23]),.s(result[23]),.cout(carry[24]));
  full_adder adder25(.x(A[24]),.y(B[24]),.cin(carry[24]),.s(result[24]),.cout(carry[25]));
  full_adder adder26(.x(A[25]),.y(B[25]),.cin(carry[25]),.s(result[25]),.cout(carry[26]));
  full_adder adder27(.x(A[26]),.y(B[26]),.cin(carry[26]),.s(result[26]),.cout(carry[27]));
  full_adder adder28(.x(A[27]),.y(B[27]),.cin(carry[27]),.s(result[27]),.cout(carry[28]));
  full_adder adder29(.x(A[28]),.y(B[28]),.cin(carry[28]),.s(result[28]),.cout(carry[29]));
  full_adder adder30(.x(A[29]),.y(B[29]),.cin(carry[29]),.s(result[29]),.cout(carry[30]));
  full_adder adder31(.x(A[30]),.y(B[30]),.cin(carry[30]),.s(result[30]),.cout(carry[31]));
  full_adder adder32(.x(A[31]),.y(B[31]),.cin(carry[31]),.s(result[31]),.cout(carry[32]));
endmodule
