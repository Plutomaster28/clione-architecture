; Clione64 core/fulls-system smoke program
; Starts at 0x1000 by default in testbenches

.org 0x1000

start:
  ADDI R1, R0, #1
  ADDI R2, R0, #2
  ADD  R3, R1, R2
  XOR  R4, R3, R1
  AND  R5, R4, R2
  OR   R6, R5, R1
  BEQ  R0, R0, start
