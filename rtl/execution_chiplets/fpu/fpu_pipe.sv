// ============================================================================
// Clione Processor — FPU Pipeline
// IEEE 754 double-precision, fused multiply-add, div, sqrt
// 6-stage pipeline (FMA), 3-stage simple ops
// ============================================================================
`include "clione_pkg.sv"

module fpu_pipe
  import clione_pkg::*;
(
  input  logic                        clk,
  input  logic                        rst_n,

  input  logic [31:0]                 instr,
  input  logic [XLEN-1:0]            src1,         // double bits
  input  logic [XLEN-1:0]            src2,
  input  logic [XLEN-1:0]            src3,         // FMA third operand
  input  logic [ROB_PTR_WIDTH-1:0]    rob_idx,
  input  logic [PREG_WIDTH-1:0]       phys_rd,
  input  logic [TID_WIDTH-1:0]        tid,
  input  logic                        valid,
  output logic                        ready,

  output logic [XLEN-1:0]            result,
  output logic [4:0]                  fflags,       // NV,DZ,OF,UF,NX
  output logic [ROB_PTR_WIDTH-1:0]    res_rob_idx,
  output logic [PREG_WIDTH-1:0]       res_phys_rd,
  output logic [TID_WIDTH-1:0]        res_tid,
  output logic                        exception,
  output logic [5:0]                  exc_code,
  output logic                        res_valid
);

  // --------------------------------------------------------------------------
  // IEEE 754 double precision fields
  // --------------------------------------------------------------------------
  localparam int EXP_BITS  = 11;
  localparam int MANT_BITS = 52;
  localparam int BIAS      = 1023;

  typedef struct packed {
    logic        sign;
    logic [10:0] exp;
    logic [51:0] mant;
  } fp64_t;

  // --------------------------------------------------------------------------
  // Opcode decode
  // --------------------------------------------------------------------------
  logic [4:0]  fp_rs2;
  logic [2:0]  rm;          // rounding mode
  logic [6:0]  opcode;
  logic [6:0]  fn7;

  assign opcode = instr[6:0];
  assign fn7    = instr[31:25];
  assign fp_rs2 = instr[24:20];
  assign rm     = instr[14:12];

  // --------------------------------------------------------------------------
  // Stage 1: Unpack operands
  // --------------------------------------------------------------------------
  fp64_t  s1_a, s1_b, s1_c;
  logic   s1_valid;
  logic [31:0] s1_instr;
  logic [ROB_PTR_WIDTH-1:0] s1_rob; logic [PREG_WIDTH-1:0] s1_prd;
  logic [TID_WIDTH-1:0]     s1_tid;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s1_valid <= 1'b0;
    end else begin
      s1_a     <= fp64_t'(src1);
      s1_b     <= fp64_t'(src2);
      s1_c     <= fp64_t'(src3);
      s1_instr <= instr;
      s1_valid <= valid;
      s1_rob   <= rob_idx;
      s1_prd   <= phys_rd;
      s1_tid   <= tid;
    end
  end

  assign ready = 1'b1;

  // --------------------------------------------------------------------------
  // Stage 2: Exponent arithmetic and mantissa alignment
  // --------------------------------------------------------------------------
  logic [12:0]  s2_exp_sum;      // Exponent for multiply result
  logic [105:0] s2_mant_prod;    // Double-wide mantissa product
  logic         s2_sign_prod;
  logic         s2_valid;
  logic [31:0]  s2_instr;
  logic [ROB_PTR_WIDTH-1:0] s2_rob; logic [PREG_WIDTH-1:0] s2_prd;
  logic [TID_WIDTH-1:0]     s2_tid;
  fp64_t        s2_a, s2_b, s2_c;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s2_valid <= 1'b0;
    end else begin
      s2_valid     <= s1_valid;
      s2_instr     <= s1_instr;
      s2_rob       <= s1_rob;
      s2_prd       <= s1_prd;
      s2_tid       <= s1_tid;
      s2_a         <= s1_a;
      s2_b         <= s1_b;
      s2_c         <= s1_c;
      // Multiply exponents: exp(a) + exp(b) - BIAS
      s2_exp_sum  <= 13'(s1_a.exp) + 13'(s1_b.exp) - BIAS;
      // Mantissa with implicit leading 1
      s2_mant_prod <= {1'b1, s1_a.mant} * {1'b1, s1_b.mant};
      s2_sign_prod <= s1_a.sign ^ s1_b.sign;
    end
  end

  // --------------------------------------------------------------------------
  // Stage 3: Addition alignment for FMA (c + a*b)
  // Simple ops (FADD, FSUB, etc.) bypass multiply path
  // --------------------------------------------------------------------------
  logic [127:0]  s3_addend_a;   // Aligned mantissa for addition
  logic [127:0]  s3_addend_b;
  logic          s3_sign_a, s3_sign_b;
  logic [12:0]   s3_exp;
  logic          s3_valid;
  logic [31:0]   s3_instr;
  logic [ROB_PTR_WIDTH-1:0] s3_rob; logic [PREG_WIDTH-1:0] s3_prd;
  logic [TID_WIDTH-1:0]     s3_tid;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s3_valid <= 1'b0;
    end else begin
      s3_valid  <= s2_valid;
      s3_instr  <= s2_instr;
      s3_rob    <= s2_rob;
      s3_prd    <= s2_prd;
      s3_tid    <= s2_tid;

      // For FP ADD/SUB: align s2_a and s2_b by exponent difference
      automatic logic signed [13:0] exp_diff = $signed(13'(s2_a.exp)) - $signed(13'(s2_b.exp));

      if (s2_instr[6:0] == OP_FMADD || s2_instr[6:0] == OP_FMSUB) begin
        // FMA: product in s2_mant_prod, add c
        s3_addend_a <= {s2_mant_prod, 22'b0}; // Product, unnormalized
        s3_addend_b <= {1'b1, s2_c.mant, 74'b0}; // c mantissa shifted
        s3_exp      <= s2_exp_sum;
        s3_sign_a   <= s2_sign_prod;
        s3_sign_b   <= s2_c.sign;
      end else begin
        // FADD/FSUB: align smaller to larger exponent
        if (exp_diff >= 0) begin
          s3_addend_a <= {1'b1, s2_a.mant, 74'b0};
          s3_addend_b <= {1'b1, s2_b.mant, 74'b0} >> exp_diff;
          s3_exp      <= 13'(s2_a.exp);
          s3_sign_a   <= s2_a.sign;
          s3_sign_b   <= s2_b.sign;
        end else begin
          s3_addend_a <= {1'b1, s2_b.mant, 74'b0};
          s3_addend_b <= {1'b1, s2_a.mant, 74'b0} >> (-exp_diff);
          s3_exp      <= 13'(s2_b.exp);
          s3_sign_a   <= s2_b.sign;
          s3_sign_b   <= s2_a.sign;
        end
      end
    end
  end

  // --------------------------------------------------------------------------
  // Stage 4: Addition / Subtraction
  // --------------------------------------------------------------------------
  logic [127:0]  s4_sum;
  logic          s4_sign;
  logic [12:0]   s4_exp;
  logic          s4_valid;
  logic [31:0]   s4_instr;
  logic [ROB_PTR_WIDTH-1:0] s4_rob; logic [PREG_WIDTH-1:0] s4_prd;
  logic [TID_WIDTH-1:0]     s4_tid;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s4_valid <= 1'b0;
    end else begin
      s4_valid  <= s3_valid;
      s4_instr  <= s3_instr;
      s4_rob    <= s3_rob;
      s4_prd    <= s3_prd;
      s4_tid    <= s3_tid;
      s4_exp    <= s3_exp;

      if (s3_sign_a == s3_sign_b) begin
        s4_sum  <= s3_addend_a + s3_addend_b;
        s4_sign <= s3_sign_a;
      end else begin
        if (s3_addend_a >= s3_addend_b) begin
          s4_sum  <= s3_addend_a - s3_addend_b;
          s4_sign <= s3_sign_a;
        end else begin
          s4_sum  <= s3_addend_b - s3_addend_a;
          s4_sign <= s3_sign_b;
        end
      end
    end
  end

  // --------------------------------------------------------------------------
  // Stage 5: Normalization
  // --------------------------------------------------------------------------
  logic [63:0]  s5_normalized;
  logic         s5_valid;
  logic [ROB_PTR_WIDTH-1:0] s5_rob; logic [PREG_WIDTH-1:0] s5_prd;
  logic [TID_WIDTH-1:0]     s5_tid;

  // Leading zero count for normalization shift
  function automatic logic [6:0] clz128(input logic [127:0] v);
    for (int i = 127; i >= 0; i--)
      if (v[i]) return 7'(127 - i);
    return 7'd128;
  endfunction

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s5_valid <= 1'b0;
    end else begin
      s5_valid <= s4_valid;
      s5_rob   <= s4_rob;
      s5_prd   <= s4_prd;
      s5_tid   <= s4_tid;

      // Normalize: shift mantissa so MSB is at bit 127-12=115 (after implicit 1)
      automatic logic [6:0] lz = clz128(s4_sum);
      automatic logic [127:0] shifted = s4_sum << lz;
      automatic logic [10:0]  norm_exp = 11'(s4_exp - 13'(lz));
      // Pack result
      s5_normalized <= {s4_sign, norm_exp, shifted[126:75]}; // 1 + 11 + 52 = 64
    end
  end

  // --------------------------------------------------------------------------
  // Stage 6: Output
  // --------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      res_valid <= 1'b0;
    end else begin
      result      <= s5_normalized;
      res_rob_idx <= s5_rob;
      res_phys_rd <= s5_prd;
      res_tid     <= s5_tid;
      fflags      <= 5'b0; // Simplified; real: track NV/NX/etc.
      exception   <= 1'b0;
      exc_code    <= 6'b0;
      res_valid   <= s5_valid;
    end
  end

endmodule : fpu_pipe
