// ============================================================================
// Clione Processor — Issue Queue (per-cluster, parameterized)
// Distributed issue queues: INT, FP, VEC, MEM each have their own
// Wakes on CDB (Common Data Bus) writeback
// ============================================================================
`include "clione_pkg.sv"

module issue_queue
  import clione_pkg::*;
#(
  parameter int unsigned IQ_DEPTH       = 32,
  // Bitmask of accepted iclasses (bit N = accept iclass value N).
  // ICLASS_INT=0, ICLASS_MUL=1, ICLASS_FP=2, ICLASS_VEC=3,
  // ICLASS_MEM=4, ICLASS_BRANCH=5, ICLASS_SYS=6, ICLASS_CRYPTO=7
  parameter logic [7:0]  IQ_ICLASS_MASK = 8'b0000_0001, // default: INT only
  parameter int unsigned IQ_ISSUE_WIDTH = 4              // How many uops issued per cycle
)(
  input  logic                           clk,
  input  logic                           rst_n,

  // From rename
  input  uop_t                           alloc_uop    [DECODE_WIDTH-1:0],
  input  logic                           alloc_valid  [DECODE_WIDTH-1:0],
  output logic                           alloc_ready,

  // To execution chiplet
  output uop_t                           issue_uop    [IQ_ISSUE_WIDTH-1:0],
  output logic                           issue_valid  [IQ_ISSUE_WIDTH-1:0],
  input  logic                           issue_ready  [IQ_ISSUE_WIDTH-1:0],

  // CDB — broadcast results from ALL execution clusters
  input  exec_result_t                   cdb          [ISSUE_WIDTH-1:0],
  input  logic                           cdb_valid    [ISSUE_WIDTH-1:0],

  // Flush
  input  logic                           flush_valid,
  input  logic [TID_WIDTH-1:0]           flush_tid
);

  // --------------------------------------------------------------------------
  // IQ Entry
  // --------------------------------------------------------------------------
  typedef struct packed {
    uop_t  uop;
    logic  rs1_ready;
    logic  rs2_ready;
    logic  rs3_ready;
    logic  valid;
    logic  issued;  // Speculatively dispatched, not yet confirmed
  } iq_entry_t;

  iq_entry_t iq [IQ_DEPTH-1:0];

  // --------------------------------------------------------------------------
  // Allocation pointer — find empty slots
  // --------------------------------------------------------------------------
  logic [$clog2(IQ_DEPTH)-1:0] alloc_slot [DECODE_WIDTH-1:0];
  logic                         alloc_slot_ok [DECODE_WIDTH-1:0];

  always_comb begin
    automatic logic [IQ_DEPTH-1:0] used;
    automatic logic [IQ_DEPTH-1:0] used_after;
    for (int i = 0; i < IQ_DEPTH; i++)
      used[i] = iq[i].valid;

    for (int k = 0; k < DECODE_WIDTH; k++) begin
      alloc_slot[k]    = '0;
      alloc_slot_ok[k] = 1'b0;
      used_after = used;
      for (int i = 0; i < IQ_DEPTH; i++) begin
        if (!used_after[i] && !alloc_slot_ok[k]) begin
          alloc_slot[k]    = $clog2(IQ_DEPTH)'(i);
          alloc_slot_ok[k] = 1'b1;
          used_after[i]    = 1'b1;
        end
      end
      used = used_after;
    end
    alloc_ready = alloc_slot_ok[0]; // Simplified: ready if at least one slot free
  end

  // --------------------------------------------------------------------------
  // CDB Wake-up: broadcast writeback to all waiting entries
  // --------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i < IQ_DEPTH; i++) begin
        iq[i] <= '0;
      end
    end else begin
      // Flush: invalidate entries for the flushed thread
      if (flush_valid) begin
        for (int i = 0; i < IQ_DEPTH; i++) begin
          if (iq[i].valid && iq[i].uop.tid == flush_tid)
            iq[i].valid <= 1'b0;
        end
      end

      // CDB wakeup: update readiness of source registers
      for (int i = 0; i < IQ_DEPTH; i++) begin
        if (iq[i].valid && !iq[i].issued) begin
          for (int c = 0; c < ISSUE_WIDTH; c++) begin
            if (cdb_valid[c] && cdb[c].valid) begin
              if (!iq[i].rs1_ready &&
                  iq[i].uop.phys_rs1 == cdb[c].phys_rd)
                iq[i].rs1_ready <= 1'b1;
              if (!iq[i].rs2_ready &&
                  iq[i].uop.phys_rs2 == cdb[c].phys_rd)
                iq[i].rs2_ready <= 1'b1;
              if (!iq[i].rs3_ready &&
                  iq[i].uop.phys_rs3 == cdb[c].phys_rd)
                iq[i].rs3_ready <= 1'b1;
            end
          end
        end
      end

      // Allocation of new uops from rename stage
      for (int k = 0; k < DECODE_WIDTH; k++) begin
        if (alloc_valid[k] && alloc_uop[k].valid &&
            IQ_ICLASS_MASK[alloc_uop[k].iclass] && alloc_slot_ok[k]) begin
          iq[alloc_slot[k]].uop       <= alloc_uop[k];
          iq[alloc_slot[k]].rs1_ready <= alloc_uop[k].rs1_ready;
          iq[alloc_slot[k]].rs2_ready <= alloc_uop[k].rs2_ready;
          iq[alloc_slot[k]].rs3_ready <= alloc_uop[k].rs3_ready;
          iq[alloc_slot[k]].valid     <= 1'b1;
          iq[alloc_slot[k]].issued    <= 1'b0;
          // Check CDB at same cycle for single-cycle producers
          for (int c = 0; c < ISSUE_WIDTH; c++) begin
            if (cdb_valid[c] && cdb[c].valid) begin
              if (alloc_uop[k].phys_rs1 == cdb[c].phys_rd)
                iq[alloc_slot[k]].rs1_ready <= 1'b1;
              if (alloc_uop[k].phys_rs2 == cdb[c].phys_rd)
                iq[alloc_slot[k]].rs2_ready <= 1'b1;
            end
          end
        end
      end

      // Dequeue issued entries (fire-and-forget on ack from execution side)
      for (int j = 0; j < IQ_ISSUE_WIDTH; j++) begin
        if (issue_valid[j] && issue_ready[j]) begin
          // Find and invalidate the matching IQ slot (by rob_idx)
          for (int i = 0; i < IQ_DEPTH; i++) begin
            if (iq[i].valid && iq[i].issued &&
                iq[i].uop.rob_idx == issue_uop[j].rob_idx)
              iq[i].valid <= 1'b0;
          end
        end
      end
    end
  end

  // --------------------------------------------------------------------------
  // Issue Selection: oldest-ready-first (age matrix simplified to linear scan)
  // --------------------------------------------------------------------------
  always_comb begin
    automatic int sel_cnt = 0;
    for (int j = 0; j < IQ_ISSUE_WIDTH; j++) begin
      issue_uop[j]   = '0;
      issue_valid[j] = 1'b0;
    end

    for (int i = 0; i < IQ_DEPTH && sel_cnt < IQ_ISSUE_WIDTH; i++) begin
      if (iq[i].valid && !iq[i].issued &&
          iq[i].rs1_ready && iq[i].rs2_ready && iq[i].rs3_ready) begin
        issue_uop[sel_cnt]   = iq[i].uop;
        issue_valid[sel_cnt] = 1'b1;
        sel_cnt++;
      end
    end
  end

endmodule : issue_queue
