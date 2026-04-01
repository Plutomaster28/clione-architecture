// ============================================================================
// Clione Processor — Reorder Buffer (ROB)
// 256 entries, 4 SMT threads, 8-wide retire
// In-order retirement, out-of-order completion
// ============================================================================
`include "clione_pkg.sv"

module reorder_buffer
  import clione_pkg::*;
(
  input  logic                            clk,
  input  logic                            rst_n,

  // Allocation from rename — 8 entries per cycle
  input  uop_t                            alloc_uop   [DECODE_WIDTH-1:0],
  input  logic                            alloc_valid [DECODE_WIDTH-1:0],
  output logic [ROB_PTR_WIDTH-1:0]        alloc_idx   [DECODE_WIDTH-1:0],
  output logic                            alloc_ready,

  // Writeback from execution chiplets via CDB
  input  exec_result_t                    cdb         [ISSUE_WIDTH-1:0],
  input  logic                            cdb_valid   [ISSUE_WIDTH-1:0],

  // Retirement outputs
  output logic                            retire_valid [RETIRE_WIDTH-1:0],
  output rob_entry_t                      retire_entry [RETIRE_WIDTH-1:0],
  output logic [PREG_WIDTH-1:0]           retire_phys  [RETIRE_WIDTH-1:0],
  output logic [TID_WIDTH-1:0]            retire_tid   [RETIRE_WIDTH-1:0],

  // Checkpoint RAT (for mispredict recovery)
  output logic [PREG_WIDTH-1:0]           ckpt_rat_int [SMT_WAYS-1:0][ARCH_INT_REGS-1:0],
  output logic [PREG_WIDTH-1:0]           ckpt_rat_fp  [SMT_WAYS-1:0][ARCH_FP_REGS-1:0],

  // Redirect on mispredict / exception
  output logic                            redirect_valid,
  output logic [TID_WIDTH-1:0]            redirect_tid,
  output logic [63:0]                     redirect_pc,
  output logic                            flush_valid,
  output logic [TID_WIDTH-1:0]            flush_tid,

  // ROB fullness indicators
  output logic [ROB_PTR_WIDTH:0]          rob_used_count
);

  // --------------------------------------------------------------------------
  // ROB Storage
  // --------------------------------------------------------------------------
  rob_entry_t   rob          [ROB_DEPTH-1:0];
  logic [ROB_PTR_WIDTH-1:0] head;   // oldest instruction (retire from here)
  logic [ROB_PTR_WIDTH-1:0] tail;   // next allocation slot
  logic [ROB_PTR_WIDTH:0]   count;  // number of valid entries

  assign rob_used_count = count;

  // Sufficient free space check: need DECODE_WIDTH free entries
  assign alloc_ready = (count + (ROB_PTR_WIDTH+1)'(DECODE_WIDTH)
                        <= (ROB_PTR_WIDTH+1)'(ROB_DEPTH));

  // --------------------------------------------------------------------------
  // Allocation: write new ROB entries at tail
  // --------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      head  <= '0;
      tail  <= '0;
      count <= '0;
      for (int i = 0; i < ROB_DEPTH; i++)
        rob[i] = '0;
    end else begin
      // ----- Allocation -----
      if (alloc_ready) begin
        automatic logic [ROB_PTR_WIDTH-1:0] next_tail = tail;
        for (int i = 0; i < DECODE_WIDTH; i++) begin
          if (alloc_valid[i] && alloc_uop[i].valid) begin
            rob[next_tail].pc                  <= alloc_uop[i].pc;
            rob[next_tail].rob_idx             <= next_tail;
            rob[next_tail].phys_rd             <= alloc_uop[i].phys_rd;
            rob[next_tail].phys_rd_old         <= alloc_uop[i].phys_rd_old;
            rob[next_tail].arch_rd             <= alloc_uop[i].arch_rd;
            rob[next_tail].tid                 <= alloc_uop[i].tid;
            rob[next_tail].complete            <= 1'b0;
            rob[next_tail].exception           <= 1'b0;
            rob[next_tail].is_store            <= alloc_uop[i].is_store;
            rob[next_tail].is_branch           <= alloc_uop[i].is_branch;
            rob[next_tail].branch_mispredict   <= 1'b0;
            rob[next_tail].iclass             <= alloc_uop[i].iclass;
            next_tail = next_tail + ROB_PTR_WIDTH'(1);  // wraps naturally
          end
        end
        tail  <= next_tail;
        count <= count + (ROB_PTR_WIDTH+1)'(DECODE_WIDTH);  // simplified
      end

      // ----- CDB Writeback: mark entries complete -----
      for (int c = 0; c < ISSUE_WIDTH; c++) begin
        if (cdb_valid[c] && cdb[c].valid) begin
          automatic logic [ROB_PTR_WIDTH-1:0] ridx = cdb[c].rob_idx;
          rob[ridx].complete          <= 1'b1;
          rob[ridx].exception         <= cdb[c].exception;
          rob[ridx].exc_code          <= cdb[c].exc_code;
          rob[ridx].branch_mispredict <= cdb[c].branch_taken ^ rob[ridx].is_branch;
          rob[ridx].branch_correct_target <= cdb[c].branch_target;
        end
      end

      // ----- Retirement: retire up to RETIRE_WIDTH instructions from head -----
      if (!flush_valid) begin
        automatic int retire_cnt = 0;
        for (int i = 0; i < RETIRE_WIDTH; i++) begin
          retire_valid[i] = 1'b0;
          retire_entry[i] = '0;
          retire_phys[i]  = '0;
          retire_tid[i]   = '0;
        end

        for (int i = 0; i < RETIRE_WIDTH; i++) begin
          automatic logic [ROB_PTR_WIDTH-1:0] h = head + ROB_PTR_WIDTH'(i);
          if (rob[h].complete && count > 0) begin
            if (rob[h].exception || rob[h].branch_mispredict) begin
              // Trigger redirect and flush pipeline
              redirect_valid <= 1'b1;
              redirect_tid   <= rob[h].tid;
              redirect_pc    <= rob[h].branch_mispredict
                                ? rob[h].branch_correct_target
                                : rob[h].pc + 4; // Exception entry (simplified)
              flush_valid <= 1'b1;
              flush_tid   <= rob[h].tid;
              // Stop retiring additional instructions for this thread
              break;
            end else begin
              retire_valid[i] = 1'b1;
              retire_entry[i] = rob[h];
              retire_phys[i]  = rob[h].phys_rd_old; // free the old mapping
              retire_tid[i]   = rob[h].tid;
              retire_cnt++;
            end
          end else begin
            break; // Non-complete at head: stall
          end
        end

        head  <= head  + ROB_PTR_WIDTH'(retire_cnt);
        count <= count - ROB_PTR_WIDTH'(retire_cnt);
      end else begin
        // After flush: clear the pipeline and reset to redirect_pc
        redirect_valid <= 1'b0;
        flush_valid    <= 1'b0;
        // Count becomes 0 for flushed thread (simplified: full flush)
        head  <= tail;
        count <= '0;
      end
    end
  end

  // --------------------------------------------------------------------------
  // Provide alloc_idx so rename can stamp ROB pointer into uops
  // --------------------------------------------------------------------------
  always_comb begin
    for (int i = 0; i < DECODE_WIDTH; i++)
      alloc_idx[i] = tail + ROB_PTR_WIDTH'(i);
  end

  // --------------------------------------------------------------------------
  // Checkpoint RAT: snapshot current RAT state at ROB head for recovery
  // (In a real design, this would be maintained by the RAT module.
  //  Here we expose placeholder outputs; the RAT drives actual content.)
  // --------------------------------------------------------------------------
  // ckpt_rat_int/_fp are placeholders driven to zero (RAT module owns real values)
  always_comb begin
    for (int t = 0; t < SMT_WAYS;     t++)
      for (int r = 0; r < ARCH_INT_REGS; r++)
        ckpt_rat_int[t][r] = '0;
  end
  always_comb begin
    for (int t = 0; t < SMT_WAYS;   t++)
      for (int r = 0; r < ARCH_FP_REGS; r++)
        ckpt_rat_fp[t][r] = '0;
  end

endmodule : reorder_buffer
