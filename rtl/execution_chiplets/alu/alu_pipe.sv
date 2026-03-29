// ============================================================================
// Clione Processor — ALU Pipeline
// Full integer ALU: 64-bit ops, barrel shifter, multiplier, divider
// 4-stage pipeline to accommodate deep computation at lower clock
// ============================================================================
`include "clione_pkg.sv"

module alu_pipe
  import clione_pkg::*;
 #(
  parameter bit ENABLE_MULDIV = 1'b1
 )
(
  input  logic                        clk,
  input  logic                        rst_n,

  // Instruction from NoC dispatch
  input  logic [31:0]                 instr,
  input  logic [XLEN-1:0]            src1,
  input  logic [XLEN-1:0]            src2,
  input  logic [XLEN-1:0]            imm,
  input  logic                        has_imm,
  input  seabird_mode_e               mode_i,
  input  op_width_e                   op_width_i,
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
  seabird_mode_e     s1_mode;
  op_width_e         s1_op_width;
  logic              s1_sb_ext;
  logic [7:0]        s1_sb_opcode;
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
      s1_mode       <= mode_i;
      s1_op_width   <= op_width_i;
      s1_sb_ext     <= is_seabird_ext(instr);
      s1_sb_opcode  <= seabird_opcode(instr);
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
        automatic logic [5:0] shamt;
        automatic logic [127:0] mul_u;
        automatic logic signed [127:0] mul_s;
        automatic logic signed [127:0] mul_hsu;
        automatic op_width_e eff_w;
        automatic logic branch_uses_rv_fn3;

        // Width-compatibility path:
        // OP_IMM64/OP_REG64 are treated as 32-bit arithmetic with sign-extended result.
        eff_w = s1_op_width;
        if (s1_is32) eff_w = OPW_32;

        if ((eff_w == OPW_8) || (eff_w == OPW_16) || (eff_w == OPW_32)) begin
          a = sign_extend_width(apply_width_mask(a, eff_w), eff_w);
          b = sign_extend_width(apply_width_mask(b, eff_w), eff_w);
        end

        if (s1_is32) begin
          a = sign_extend_width(apply_width_mask(a, OPW_32), OPW_32);
          b = sign_extend_width(apply_width_mask(b, OPW_32), OPW_32);
          shamt = {1'b0, b[4:0]};
        end else begin
          shamt = b[5:0];
        end

        res = '0;
        branch_uses_rv_fn3 = 1'b1;

        if (s1_sb_ext) begin
          unique case (s1_sb_opcode)
            // Data move + arithmetic
            8'h00, 8'h01: res = b;                 // MOV/MOVI
            8'h1F: begin                           // XCHG (single-dst model -> pass src2)
              res = b;
            end
            8'h20, 8'h21: res = a + b;             // ADD/ADDI
            8'h22, 8'h23: res = a - b;             // SUB/SUBI
            8'h24, 8'h25: res = a * b;             // MUL/MULI low 64
            8'h26, 8'h27: res = (b == 0) ? 64'hFFFF_FFFF_FFFF_FFFF
                                         : XLEN'($signed(a) / $signed(b));
            8'h28, 8'h29: res = (b == 0) ? a : XLEN'($signed(a) % $signed(b));
            8'h2A: res = -a;                       // NEG
            8'h2B: res = a + 64'd1;                // INC
            8'h2C: res = a - 64'd1;                // DEC
            8'h2D: begin mul_s = $signed(a) * $signed(b); res = mul_s[127:64]; end // MULH
            8'h2E: res = a * b;                    // UMUL (low)
            8'h2F: res = (b == 0) ? 64'hFFFF_FFFF_FFFF_FFFF : (a / b); // UDIV
            // Saturating arithmetic
            8'h30: begin // ADDS
              automatic logic signed [63:0] sa, sb, sr;
              sa = $signed(a); sb = $signed(b); sr = sa + sb;
              if ((sa[63] == sb[63]) && (sr[63] != sa[63]))
                res = sa[63] ? 64'h8000_0000_0000_0000 : 64'h7FFF_FFFF_FFFF_FFFF;
              else
                res = sr;
            end
            8'h31: begin // ADDU
              automatic logic [64:0] usum;
              usum = {1'b0,a} + {1'b0,b};
              res = usum[64] ? 64'hFFFF_FFFF_FFFF_FFFF : usum[63:0];
            end
            8'h32: begin // SUBS
              automatic logic signed [63:0] sa, sb, sr;
              sa = $signed(a); sb = $signed(b); sr = sa - sb;
              if ((sa[63] != sb[63]) && (sr[63] != sa[63]))
                res = sa[63] ? 64'h8000_0000_0000_0000 : 64'h7FFF_FFFF_FFFF_FFFF;
              else
                res = sr;
            end
            8'h33: res = (a < b) ? 64'd0 : (a - b); // SUBU saturating
            8'h34: res = $signed(a) < 0 ? -a : a;   // ABS
            8'h35: begin // CLZ
              automatic int cnt;
              automatic logic found;
              cnt = 0;
              found = 1'b0;
              for (int bi = XLEN-1; bi >= 0; bi--) begin
                if (!found) begin
                  if (a[bi]) found = 1'b1;
                  else cnt = cnt + 1;
                end
              end
              res = 64'(cnt);
            end
            8'h36: begin // CTZ
              automatic int cnt;
              automatic logic found;
              cnt = 0;
              found = 1'b0;
              for (int bi = 0; bi < XLEN; bi++) begin
                if (!found) begin
                  if (a[bi]) found = 1'b1;
                  else cnt = cnt + 1;
                end
              end
              res = 64'(cnt);
            end
            8'h37: begin // POPC
              automatic int cnt;
              cnt = 0;
              for (int bi = 0; bi < XLEN; bi++)
                cnt = cnt + int'(a[bi]);
              res = 64'(cnt);
            end
            // Logic/bit ops
            8'h40: res = a & b;
            8'h41: res = a | b;
            8'h42: res = a ^ b;
            8'h43: res = ~a;
            8'h44: res = ~(a & b);
            8'h45: res = ~(a | b);
            8'h46: res = ~(a ^ b);
            8'h47: res = a << shamt;
            8'h48: res = a >> shamt;
            8'h49: res = XLEN'($signed(a) >>> shamt);
            8'h4A: res = (a << shamt) | (a >> (6'(XLEN) - shamt));
            8'h4B: res = (a >> shamt) | (a << (6'(XLEN) - shamt));
            8'h4C: res = a | (64'd1 << b[5:0]);
            8'h4D: res = a & ~(64'd1 << b[5:0]);
            8'h4E: res = a ^ (64'd1 << b[5:0]);
            8'h4F: res = (a >> b[5:0]) & 64'd1;
            // Compare/value-select
            8'h58: res = ($signed(a) > $signed(b)) ? a : b; // MAX
            8'h59: res = ($signed(a) < $signed(b)) ? a : b; // MIN
            8'h5A: res = ($signed(a) < $signed(b)) ? 64'd1 : 64'd0;
            8'h5B: res = ($signed(a) > $signed(b)) ? 64'd1 : 64'd0;
            default: begin
              // Non-ALU SeaBird instructions are routed to other clusters.
              // If they arrive here, treat as illegal for this pipe.
              res = '0;
              s2_exc <= 1'b1;
              s2_exc_code <= 6'd2;
            end
          endcase
          branch_uses_rv_fn3 = 1'b0;
        end else if ((s1_op == OP_IMM) || (s1_op == OP_IMM64)) begin
          unique case (s1_fn3)
            3'b000: res = a + b;                              // ADDI/ADDIW
            3'b001: res = a << shamt;                         // SLLI/SLLIW
            3'b010: res = ($signed(a) < $signed(b)) ? 64'd1 : 64'd0; // SLTI
            3'b011: res = (a < b) ? 64'd1 : 64'd0;            // SLTIU
            3'b100: res = a ^ b;                              // XORI
            3'b101: res = s1_fn7[5] ? XLEN'($signed(a) >>> shamt)
                                      : (a >> shamt);         // SRAI/SRLI
            3'b110: res = a | b;                              // ORI
            3'b111: res = a & b;                              // ANDI
            default: begin res = '0; s2_exc <= 1'b1; s2_exc_code <= 6'd2; end
          endcase
        end else if ((s1_op == OP_REG) || (s1_op == OP_REG64)) begin
          // M-extension when fn7 == 7'b0000001
          if ((s1_fn7 == 7'b0000001) && ENABLE_MULDIV) begin
            mul_u = a * b;
            mul_s = $signed(a) * $signed(b);
            mul_hsu = $signed(a) * $signed({1'b0, b});
            unique case (s1_fn3)
              3'b000: res = mul_u[63:0];                      // MUL/MULW (low)
              3'b001: res = mul_s[127:64];                    // MULH
              3'b010: res = mul_hsu[127:64];                  // MULHSU
              3'b011: res = mul_u[127:64];                    // MULHU
              3'b100: res = (b == 0) ? 64'hFFFF_FFFF_FFFF_FFFF
                                     : XLEN'($signed(a) / $signed(b));   // DIV
              3'b101: res = (b == 0) ? 64'hFFFF_FFFF_FFFF_FFFF
                                     : (a / b);               // DIVU
              3'b110: res = (b == 0) ? a : XLEN'($signed(a) % $signed(b)); // REM
              3'b111: res = (b == 0) ? a : (a % b);           // REMU
              default: begin res = '0; s2_exc <= 1'b1; s2_exc_code <= 6'd2; end
            endcase
          end else if ((s1_fn7 == 7'b0000001) && !ENABLE_MULDIV) begin
            res = '0;
            s2_exc <= 1'b1;
            s2_exc_code <= 6'd2;
          end else begin
            unique case (s1_fn3)
              3'b000: res = s1_fn7[5] ? (a - b) : (a + b);    // SUB/ADD
              3'b001: res = a << shamt;                        // SLL
              3'b010: res = ($signed(a) < $signed(b)) ? 64'd1 : 64'd0; // SLT
              3'b011: res = (a < b) ? 64'd1 : 64'd0;           // SLTU
              3'b100: res = a ^ b;                             // XOR
              3'b101: res = s1_fn7[5] ? XLEN'($signed(a) >>> shamt)
                                        : (a >> shamt);        // SRA/SRL
              3'b110: res = a | b;                             // OR
              3'b111: res = a & b;                             // AND
              default: begin res = '0; s2_exc <= 1'b1; s2_exc_code <= 6'd2; end
            endcase
          end
        end else begin
          res = a + b;
        end

        // LUI / AUIPC
        if (s1_op == OP_LUI)   res = b; // imm already U-type upper
        if (s1_op == OP_AUIPC) res = s1_pc + b;

        // Width writeback policy for compatibility execution.
        if (s1_is32) begin
          res = sign_extend_width(res, OPW_32);
        end else if ((eff_w == OPW_8) || (eff_w == OPW_16) || (eff_w == OPW_32)) begin
          res = sign_extend_width(res, eff_w);
        end

        s2_result <= res;

        // Branch condition evaluation
        if (s1_is_branch) begin
          automatic logic taken;
          if (branch_uses_rv_fn3) begin
            unique case (s1_fn3)
              3'b000: taken = (a == b);            // BEQ
              3'b001: taken = (a != b);            // BNE
              3'b100: taken = ($signed(a) < $signed(b)); // BLT
              3'b101: taken = ($signed(a) >= $signed(b)); // BGE
              3'b110: taken = (a < b);             // BLTU
              3'b111: taken = (a >= b);            // BGEU
              default: taken = 1'b0;
            endcase
          end else begin
            unique case (s1_sb_opcode)
              8'h5C, 8'h5D, 8'h5E, 8'h5F: taken = 1'b1;      // JMP/CALL variants
              8'h61: taken = (a == b);                        // JE
              8'h62: taken = (a != b);                        // JNE
              8'h63: taken = ($signed(a) >  $signed(b));      // JG
              8'h64: taken = ($signed(a) >= $signed(b));      // JGE
              8'h65: taken = ($signed(a) <  $signed(b));      // JL
              8'h66: taken = ($signed(a) <= $signed(b));      // JLE
              8'h67: taken = (a < b);                         // JC  (compat approximation)
              8'h68: taken = (a >= b);                        // JNC (compat approximation)
              8'h69: taken = 1'b0;                            // JO  (flags not modeled)
              8'h6A: taken = 1'b1;                            // JNO (flags not modeled)
              8'h6B: taken = a[63];                           // JS
              8'h6C: taken = ~a[63];                          // JNS
              8'h6D: taken = (a == 0);                        // JZR
              8'h6E: taken = (a != 0);                        // JNZR
              8'h6F: taken = 1'b1;                            // BRR
              default: taken = 1'b0;
            endcase
          end
          s2_branch_taken  <= taken;
          // Branch target = PC + sign_ext(imm) for B-type; already in b
          if (s1_sb_ext && (s1_sb_opcode == 8'h6F))
            s2_branch_target <= a;
          else
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
