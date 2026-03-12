// ============================================================================
// Clione Processor — ALU Pipeline
// Full integer ALU: 64-bit ops, barrel shifter, multiplier, divider
// 4-stage pipeline to accommodate deep computation at lower clock
// ============================================================================
`include "clione_pkg.sv"

module alu_pipe
  import clione_pkg::*;
(
  input  logic                        clk,
  input  logic                        rst_n,

  // Instruction from NoC dispatch
  input  logic [31:0]                 instr,
  input  logic [XLEN-1:0]            src1,
  input  logic [XLEN-1:0]            src2,
  input  logic [XLEN-1:0]            imm,
  input  logic                        has_imm,
  input  logic [ROB_PTR_WIDTH-1:0]    rob_idx,
  input  logic [PREG_WIDTH-1:0]       phys_rd,
  input  logic [TID_WIDTH-1:0]        tid,
  input  logic                        is_branch,
  input  logic [63:0]                 pc,
  input  logic                        valid,
  output logic                        ready,

  // Result
  output logic [XLEN-1:0]            result,
  output logic [ROB_PTR_WIDTH-1:0]    res_rob_idx,
  output logic [PREG_WIDTH-1:0]       res_phys_rd,
  output logic [TID_WIDTH-1:0]        res_tid,
  output logic                        branch_taken,
  output logic [63:0]                 branch_target,
  output logic                        exception,
  output logic [5:0]                  exc_code,
  output logic                        res_valid
);

  // --------------------------------------------------------------------------
  // Stage 1: Operand Select + Operation Decode
  // --------------------------------------------------------------------------
  logic [XLEN-1:0]   s1_a, s1_b;
  logic [6:0]        s1_op;
  logic [2:0]        s1_fn3;
  logic [6:0]        s1_fn7;
  logic              s1_is32;
  logic              s1_valid;
  logic [ROB_PTR_WIDTH-1:0] s1_rob_idx;
  logic [PREG_WIDTH-1:0]    s1_phys_rd;
  logic [TID_WIDTH-1:0]     s1_tid;
  logic              s1_is_branch;
  logic [63:0]       s1_pc;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s1_valid <= 1'b0;
    end else begin
      s1_a          <= has_imm ? src1 : src1;
      s1_b          <= has_imm ? imm  : src2;
      s1_op         <= instr[6:0];
      s1_fn3        <= instr[14:12];
      s1_fn7        <= instr[31:25];
      s1_is32       <= (instr[6:0] == OP_IMM64) || (instr[6:0] == OP_REG64);
      s1_valid      <= valid;
      s1_rob_idx    <= rob_idx;
      s1_phys_rd    <= phys_rd;
      s1_tid        <= tid;
      s1_is_branch  <= is_branch;
      s1_pc         <= pc;
    end
  end

  assign ready = 1'b1; // Pipelined: always accepting

  // --------------------------------------------------------------------------
  // Stage 2: Execute
  // --------------------------------------------------------------------------
  logic [XLEN-1:0]   s2_result;
  logic              s2_valid;
  logic [ROB_PTR_WIDTH-1:0] s2_rob_idx;
  logic [PREG_WIDTH-1:0]    s2_phys_rd;
  logic [TID_WIDTH-1:0]     s2_tid;
  logic              s2_branch_taken;
  logic [63:0]       s2_branch_target;
  logic              s2_is_branch;
  logic [63:0]       s2_pc;
  logic              s2_exc;
  logic [5:0]        s2_exc_code;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s2_valid <= 1'b0;
    end else begin
      s2_valid       <= s1_valid;
      s2_rob_idx     <= s1_rob_idx;
      s2_phys_rd     <= s1_phys_rd;
      s2_tid         <= s1_tid;
      s2_is_branch   <= s1_is_branch;
      s2_pc          <= s1_pc;
      s2_exc         <= 1'b0;
      s2_exc_code    <= '0;
      s2_branch_taken  <= 1'b0;
      s2_branch_target <= '0;

      if (s1_valid) begin
        automatic logic [XLEN-1:0] a = s1_a;
        automatic logic [XLEN-1:0] b = s1_b;
        automatic logic [XLEN-1:0] res;

        unique case ({s1_fn7[0], s1_fn3})
          // ADD / ADDI
          4'b0_000: res = a + b;
          // SUB
          4'b1_000: res = a - b;
          // SLL / SLLI
          4'b0_001: res = a << b[5:0];
          // SLT / SLTI
          4'b0_010: res = ($signed(a) < $signed(b)) ? 64'h1 : 64'h0;
          // SLTU / SLTIU
          4'b0_011: res = (a < b) ? 64'h1 : 64'h0;
          // XOR / XORI
          4'b0_100: res = a ^ b;
          // SRL / SRLI
          4'b0_101: res = a >> b[5:0];
          // SRA / SRAI
          4'b1_101: res = XLEN'($signed(a) >>> b[5:0]);
          // OR / ORI
          4'b0_110: res = a | b;
          // AND / ANDI
          4'b0_111: res = a & b;
          // MUL (using fn7=1)
          4'b1_000: res = a * b; // 64-bit result, lower 64 bits
          // MULH (signed × signed, upper 64)
          4'b1_001: res = XLEN'(($signed(128'($signed(a)) * $signed(128'($signed(b)))) >>> 64));
          // MULHU (unsigned × unsigned, upper 64)
          4'b1_011: res = XLEN'((128'(a) * 128'(b)) >> 64);
          // DIV
          4'b1_100: res = (b == 0) ? 64'hFFFF_FFFF_FFFF_FFFF : XLEN'($signed(a) / $signed(b));
          // DIVU
          4'b1_101: res = (b == 0) ? 64'hFFFF_FFFF_FFFF_FFFF : a / b;
          // REM
          4'b1_110: res = (b == 0) ? a : XLEN'($signed(a) % $signed(b));
          // REMU
          4'b1_111: res = (b == 0) ? a : a % b;
          default:  begin res = '0; s2_exc <= 1'b1; s2_exc_code <= 6'd2; end
        endcase

        // LUI / AUIPC
        if (s1_op == OP_LUI)   res = b; // imm already U-type upper
        if (s1_op == OP_AUIPC) res = s1_pc + b;

        // 32-bit sign extension
        if (s1_is32) res = {{32{res[31]}}, res[31:0]};

        s2_result <= res;

        // Branch condition evaluation
        if (s1_is_branch) begin
          automatic logic taken;
          unique case (s1_fn3)
            3'b000: taken = (a == b);            // BEQ
            3'b001: taken = (a != b);            // BNE
            3'b100: taken = ($signed(a) < $signed(b)); // BLT
            3'b101: taken = ($signed(a) >= $signed(b)); // BGE
            3'b110: taken = (a < b);             // BLTU
            3'b111: taken = (a >= b);            // BGEU
            default: taken = 1'b0;
          endcase
          s2_branch_taken  <= taken;
          // Branch target = PC + sign_ext(imm) for B-type; already in b
          s2_branch_target <= s1_pc + s1_b;
        end
      end
    end
  end

  // --------------------------------------------------------------------------
  // Stage 3: Result passthrough (extra latency for pipelining deep multipliers)
  // --------------------------------------------------------------------------
  logic [XLEN-1:0]   s3_result;
  logic              s3_valid;
  logic [ROB_PTR_WIDTH-1:0] s3_rob_idx;
  logic [PREG_WIDTH-1:0]    s3_phys_rd;
  logic [TID_WIDTH-1:0]     s3_tid;
  logic              s3_branch_taken;
  logic [63:0]       s3_branch_target;
  logic              s3_exc;
  logic [5:0]        s3_exc_code;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s3_valid <= 1'b0;
    end else begin
      s3_result        <= s2_result;
      s3_valid         <= s2_valid;
      s3_rob_idx       <= s2_rob_idx;
      s3_phys_rd       <= s2_phys_rd;
      s3_tid           <= s2_tid;
      s3_branch_taken  <= s2_branch_taken;
      s3_branch_target <= s2_branch_target;
      s3_exc           <= s2_exc;
      s3_exc_code      <= s2_exc_code;
    end
  end

  // --------------------------------------------------------------------------
  // Stage 4: Output
  // --------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      res_valid <= 1'b0;
    end else begin
      result         <= s3_result;
      res_rob_idx    <= s3_rob_idx;
      res_phys_rd    <= s3_phys_rd;
      res_tid        <= s3_tid;
      branch_taken   <= s3_branch_taken;
      branch_target  <= s3_branch_target;
      exception      <= s3_exc;
      exc_code       <= s3_exc_code;
      res_valid      <= s3_valid;
    end
  end

endmodule : alu_pipe
