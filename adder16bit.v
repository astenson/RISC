`include "full_adder.v"
module adder16bit(input [15:0] A, input [15:0] B, output [15:0] result);
  wire [16:0] carry;
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
endmodule
