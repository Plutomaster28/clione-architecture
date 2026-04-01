// ============================================================================
// Clione Processor — Register Alias Table (RAT) / Rename Unit
// Per-thread RAT, free-list management, physical register allocation
// Int + FP register files handled together
// ============================================================================
`include "clione_pkg.sv"

module register_alias_table
  import clione_pkg::*;
(
  input  logic                        clk,
  input  logic                        rst_n,

  // From decode
  input  uop_t                        dec_bundle [DECODE_WIDTH-1:0],
  input  logic                        dec_valid,
  input  logic [TID_WIDTH-1:0]        dec_tid,
  output logic                        dec_ready,

  // To issue queues
  output uop_t                        ren_bundle [DECODE_WIDTH-1:0],
  output logic                        ren_valid,
  output logic [TID_WIDTH-1:0]        ren_tid,
  input  logic                        ren_ready,

  // Free-list returns from ROB retirement (architectural destination freed)
  input  logic                        retire_valid [RETIRE_WIDTH-1:0],
  input  logic [PREG_WIDTH-1:0]       retire_phys  [RETIRE_WIDTH-1:0],
  input  logic [TID_WIDTH-1:0]        retire_tid   [RETIRE_WIDTH-1:0],

  // Flush: restore RAT to checkpoint on mispredict
  input  logic                        flush_valid,
  input  logic [TID_WIDTH-1:0]        flush_tid,
  // Checkpoint RAT (saved at ROB head for each thread)
  input  logic [PREG_WIDTH-1:0]       ckpt_rat_int [SMT_WAYS-1:0][ARCH_INT_REGS-1:0],
  input  logic [PREG_WIDTH-1:0]       ckpt_rat_fp  [SMT_WAYS-1:0][ARCH_FP_REGS-1:0]
);

  // --------------------------------------------------------------------------
  // RAT: per-thread mapping from arch reg → physical reg
  // --------------------------------------------------------------------------
  logic [PREG_WIDTH-1:0] rat_int [SMT_WAYS-1:0][ARCH_INT_REGS-1:0];
  logic [PREG_WIDTH-1:0] rat_fp  [SMT_WAYS-1:0][ARCH_FP_REGS-1:0];

  // --------------------------------------------------------------------------
  // Free List: bit-vector of available physical registers (shared across threads)
  // --------------------------------------------------------------------------
  logic [PHYS_INT_REGS-1:0] free_list_int;
  logic [PHYS_FP_REGS-1:0]  free_list_fp;

  // --------------------------------------------------------------------------
  // Free-list allocator: find DECODE_WIDTH free registers in priority order
  // Simple priority encoder chain, one allocation per dest register per cycle
  // --------------------------------------------------------------------------
  logic [PREG_WIDTH-1:0] alloc_int [DECODE_WIDTH-1:0];
  logic                  alloc_int_ok [DECODE_WIDTH-1:0];
  logic [PREG_WIDTH-1:0] alloc_fp  [DECODE_WIDTH-1:0];
  logic                  alloc_fp_ok  [DECODE_WIDTH-1:0];

  // Compute allocations combinationally before committing
  always_comb begin
    automatic logic [PHYS_INT_REGS-1:0] fl_int_tmp = free_list_int;
    automatic logic [PHYS_FP_REGS-1:0]  fl_fp_tmp  = free_list_fp;

    for (int i = 0; i < DECODE_WIDTH; i++) begin
      alloc_int[i]    = '0;
      alloc_int_ok[i] = 1'b0;
      alloc_fp[i]     = '0;
      alloc_fp_ok[i]  = 1'b0;

      if (dec_bundle[i].valid && dec_bundle[i].arch_rd != '0) begin
        if (is_fp_op(dec_bundle[i].opcode) || is_vec_op(dec_bundle[i].opcode)) begin
          // FP allocation
          for (int p = 1; p < PHYS_FP_REGS; p++) begin
            if (fl_fp_tmp[p] && !alloc_fp_ok[i]) begin
              alloc_fp[i]    = PREG_WIDTH'(p);
              alloc_fp_ok[i] = 1'b1;
              fl_fp_tmp[p]   = 1'b0; // mark as used for next iteration
            end
          end
        end else begin
          // INT allocation
          for (int p = 1; p < PHYS_INT_REGS; p++) begin
            if (fl_int_tmp[p] && !alloc_int_ok[i]) begin
              alloc_int[i]    = PREG_WIDTH'(p);
              alloc_int_ok[i] = 1'b1;
              fl_int_tmp[p]   = 1'b0;
            end
          end
        end
      end
    end
  end

  // Reduction of alloc_int_ok (unpacked array cannot use & reduction operator directly)
  logic alloc_int_ok_all;
  always_comb begin
    alloc_int_ok_all = 1'b1;
    for (int i = 0; i < DECODE_WIDTH; i++)
      alloc_int_ok_all &= alloc_int_ok[i];
  end

  logic alloc_ok_for_bundle;
  always_comb begin
    alloc_ok_for_bundle = 1'b1;
    for (int i = 0; i < DECODE_WIDTH; i++) begin
      if (dec_bundle[i].valid && dec_bundle[i].arch_rd != '0) begin
        if (is_fp_op(dec_bundle[i].opcode) || is_vec_op(dec_bundle[i].opcode))
          alloc_ok_for_bundle &= alloc_fp_ok[i];
        else
          alloc_ok_for_bundle &= alloc_int_ok[i];
      end
    end
  end

  // Stall when free list is exhausted
  logic any_valid_in_bundle;
  always_comb begin
    any_valid_in_bundle = 1'b0;
    for (int i = 0; i < DECODE_WIDTH; i++)
      any_valid_in_bundle |= dec_bundle[i].valid;
  end

  assign dec_ready = ren_ready && (alloc_ok_for_bundle | ~any_valid_in_bundle);

  // --------------------------------------------------------------------------
  // Source register lookup: read current RAT for rs1/rs2/rs3 per instruction
  // With intra-bundle forwarding (if earlier instr in bundle writes same arch reg)
  // --------------------------------------------------------------------------
  uop_t ren_out [DECODE_WIDTH-1:0];

  always_comb begin
    for (int i = 0; i < DECODE_WIDTH; i++) begin
      automatic logic [TID_WIDTH-1:0] t;
      automatic logic is_fp;
      ren_out[i] = dec_bundle[i];
      t = '0;
      is_fp = 1'b0;

      if (!dec_bundle[i].valid) begin
        ren_out[i] = '0;
      end else begin
        t = dec_bundle[i].tid;
        is_fp = is_fp_op(dec_bundle[i].opcode) ||
                is_vec_op(dec_bundle[i].opcode);

        // Physical source 1
        ren_out[i].phys_rs1 = is_fp ? rat_fp[t][dec_bundle[i].arch_rs1]
                                    : rat_int[t][dec_bundle[i].arch_rs1];
        // Physical source 2
        ren_out[i].phys_rs2 = is_fp ? rat_fp[t][dec_bundle[i].arch_rs2]
                                    : rat_int[t][dec_bundle[i].arch_rs2];
        // Physical source 3 (FP FMA only)
        ren_out[i].phys_rs3 = is_fp ? rat_fp[t][dec_bundle[i].arch_rs3]
                                    : '0;

        // Intra-bundle forwarding: check if earlier instruction in this bundle
        // writes to the same arch register
        for (int j = 0; j < i; j++) begin
          if (dec_bundle[j].valid && dec_bundle[j].arch_rd != '0) begin
            if (dec_bundle[j].arch_rd == dec_bundle[i].arch_rs1)
              ren_out[i].phys_rs1 = is_fp ? alloc_fp[j] : alloc_int[j];
            if (dec_bundle[j].arch_rd == dec_bundle[i].arch_rs2)
              ren_out[i].phys_rs2 = is_fp ? alloc_fp[j] : alloc_int[j];
          end
        end

        // Physical destination
        if (dec_bundle[i].arch_rd != '0) begin
          if (is_fp) begin
            ren_out[i].phys_rd     = alloc_fp[i];
            ren_out[i].phys_rd_old = rat_fp[t][dec_bundle[i].arch_rd];
          end else begin
            ren_out[i].phys_rd     = alloc_int[i];
            ren_out[i].phys_rd_old = rat_int[t][dec_bundle[i].arch_rd];
          end
        end
      end
    end
  end

  // --------------------------------------------------------------------------
  // RAT Update — write new mappings on rename
  // --------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      // Identity mapping at reset: arch reg N → phys reg N
      for (int t = 0; t < SMT_WAYS; t++) begin
        for (int r = 0; r < ARCH_INT_REGS; r++)
          rat_int[t][r] <= PREG_WIDTH'(r);
        for (int r = 0; r < ARCH_FP_REGS; r++)
          rat_fp[t][r] <= PREG_WIDTH'(r);
      end
    end else if (flush_valid) begin
      // Restore checkpoint RAT
      for (int r = 0; r < ARCH_INT_REGS; r++)
        rat_int[flush_tid][r] <= ckpt_rat_int[flush_tid][r];
      for (int r = 0; r < ARCH_FP_REGS; r++)
        rat_fp[flush_tid][r] <= ckpt_rat_fp[flush_tid][r];
    end else if (dec_valid && ren_ready) begin
      for (int i = 0; i < DECODE_WIDTH; i++) begin
        if (dec_bundle[i].valid && dec_bundle[i].arch_rd != '0) begin
          automatic logic [TID_WIDTH-1:0] t = dec_bundle[i].tid;
          if (is_fp_op(dec_bundle[i].opcode) || is_vec_op(dec_bundle[i].opcode))
            rat_fp[t][dec_bundle[i].arch_rd] <= alloc_fp[i];
          else
            rat_int[t][dec_bundle[i].arch_rd] <= alloc_int[i];
        end
      end
    end
  end

  // --------------------------------------------------------------------------
  // Free List Update
  // --------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      // Bottom ARCH_INT_REGS are reserved for identity mapping
      free_list_int <= {PHYS_INT_REGS{1'b1}} << ARCH_INT_REGS;
      free_list_fp  <= {PHYS_FP_REGS{1'b1}}  << ARCH_FP_REGS;
    end else begin
      // Allocate: clear bits
      if (dec_valid && ren_ready) begin
        for (int i = 0; i < DECODE_WIDTH; i++) begin
          if (dec_bundle[i].valid && dec_bundle[i].arch_rd != '0) begin
            if (is_fp_op(dec_bundle[i].opcode))
              free_list_fp[alloc_fp[i]]   <= 1'b0;
            else
              free_list_int[alloc_int[i]] <= 1'b0;
          end
        end
      end
      // Return retired physical registers to free list
      for (int i = 0; i < RETIRE_WIDTH; i++) begin
        if (retire_valid[i])
          free_list_int[retire_phys[i]] <= 1'b1;
      end
    end
  end

  // --------------------------------------------------------------------------
  // Pipeline register to issue queues
  // --------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ren_valid <= 1'b0;
      ren_tid   <= '0;
    end else if (flush_valid) begin
      ren_valid <= 1'b0;
    end else if (ren_ready) begin
      ren_valid <= dec_valid;
      ren_tid   <= dec_tid;
      for (int i = 0; i < DECODE_WIDTH; i++)
        // ren_bundle driven combinationally from ren_out
        ;
    end
  end

  assign ren_bundle = ren_out;

endmodule : register_alias_table
