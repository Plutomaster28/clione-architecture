// ============================================================================
// Clione Processor — Instruction Fetch Unit (IFU)
// 8-wide fetch, virtual-to-physical translation, ITLB, L1I interface
// ============================================================================
`include "clione_pkg.sv"

module ifu
  import clione_pkg::*;
(
  input  logic                              clk,
  input  logic                              rst_n,

  // SMT thread control — which threads are active
  input  logic [SMT_WAYS-1:0]              thread_active,

  // PC redirect from backend (branch mispredict / exception)
  input  logic                             redirect_valid,
  input  logic [TID_WIDTH-1:0]             redirect_tid,
  input  logic [63:0]                      redirect_pc,

  // Branch predictor feedback
  input  logic [63:0]                      bp_next_pc   [SMT_WAYS-1:0],
  input  logic                             bp_taken      [SMT_WAYS-1:0],
  output logic [63:0]                      fetch_pc      [SMT_WAYS-1:0], // to BP
  output logic                             fetch_valid   [SMT_WAYS-1:0],

  // L1I Cache interface
  output logic [PADDR_WIDTH-1:0]           icache_paddr,
  output logic                             icache_req_valid,
  input  logic                             icache_req_ready,
  input  logic [CACHE_LINE_BITS-1:0]       icache_resp_data,
  input  logic                             icache_resp_valid,
  input  logic                             icache_resp_error,

  // ITLB interface
  output logic [63:0]                      tlb_vaddr,
  output logic [TID_WIDTH-1:0]             tlb_tid,
  output logic                             tlb_req_valid,
  input  logic [PADDR_WIDTH-1:0]           tlb_paddr,
  input  logic                             tlb_hit,
  input  logic                             tlb_fault,

  // Output: fetched instruction bundle to decode stage
  output uop_t                             fetch_bundle [FETCH_WIDTH-1:0],
  output logic                             fetch_bundle_valid,
  output logic [TID_WIDTH-1:0]             fetch_bundle_tid,
  input  logic                             fetch_bundle_ready   // back-pressure from decode
);

  // --------------------------------------------------------------------------
  // Internal State
  // --------------------------------------------------------------------------
  logic [63:0]  pc_reg    [SMT_WAYS-1:0];
  logic [15:0]  bundle_seq[SMT_WAYS-1:0];
  logic [TID_WIDTH-1:0] active_tid;   // Which thread is fetching this cycle
  logic [TID_WIDTH-1:0] tid_rr;       // Round-robin thread selector

  // Fetch buffer: pre-decoded raw instructions
  logic [ILEN-1:0]      fetch_raw     [FETCH_WIDTH-1:0];
  logic [FETCH_WIDTH-1:0] fetch_valid_mask;

  // TLB pipeline state
  logic [63:0]           tlb_req_pc;
  logic                  tlb_inflight;

  // Cache line alignment: extract FETCH_WIDTH instructions from 64-byte line
  logic [CACHE_LINE_BITS-1:0] icache_line;
  logic [5:0]                 line_offset;  // byte offset within cache line

  // --------------------------------------------------------------------------
  // Thread Round-Robin Arbiter
  // --------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tid_rr <= '0;
    end else if (!tlb_inflight && !icache_req_valid) begin
      // Advance to next active thread
      tid_rr <= (tid_rr == TID_WIDTH'(SMT_WAYS-1)) ? '0 : tid_rr + 1'b1;
    end
  end

  // Always fetch the highest-priority active thread
  always_comb begin
    active_tid = tid_rr;
    for (int i = 0; i < SMT_WAYS; i++) begin
      automatic int unsigned cand_tid;
      cand_tid = (int'(tid_rr) + i) % SMT_WAYS;
      if (thread_active[TID_WIDTH'(cand_tid)]) begin
        active_tid = TID_WIDTH'(cand_tid);
        break;
      end
    end
  end

  // --------------------------------------------------------------------------
  // PC Management per thread
  // --------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int t = 0; t < SMT_WAYS; t++)
        pc_reg[t] <= 64'h0000_0000_0000_1000;  // Reset vector
      for (int t = 0; t < SMT_WAYS; t++)
        bundle_seq[t] <= '0;
    end else begin
      // Handle backend redirects (highest priority)
      if (redirect_valid) begin
        pc_reg[redirect_tid] <= redirect_pc;
      end
      // Update PC when fetch completes for that thread
      if (icache_resp_valid && !redirect_valid) begin
        pc_reg[active_tid] <= pc_reg[active_tid] + (FETCH_WIDTH * 4);
        bundle_seq[active_tid] <= bundle_seq[active_tid] + 16'd1;
      end
    end
  end

  // Expose current PC to branch predictor
  always_comb begin
    for (int t = 0; t < SMT_WAYS; t++) begin
      fetch_pc[t]    = pc_reg[t];
      fetch_valid[t] = thread_active[t];
    end
  end

  // --------------------------------------------------------------------------
  // ITLB Request
  // --------------------------------------------------------------------------
  assign tlb_vaddr     = pc_reg[active_tid];
  assign tlb_tid       = active_tid;
  assign tlb_req_valid = thread_active[active_tid] && !tlb_inflight;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tlb_inflight <= 1'b0;
      tlb_req_pc   <= '0;
    end else begin
      if (tlb_req_valid && !tlb_inflight) begin
        tlb_inflight <= 1'b1;
        tlb_req_pc   <= tlb_vaddr;
      end else if (tlb_hit || tlb_fault) begin
        tlb_inflight <= 1'b0;
      end
    end
  end

  // --------------------------------------------------------------------------
  // L1I Cache Request (after TLB translation)
  // --------------------------------------------------------------------------
  assign icache_paddr     = tlb_paddr;
  assign icache_req_valid = tlb_hit && !icache_resp_valid;

  // --------------------------------------------------------------------------
  // Instruction Extraction from Cache Line
  // --------------------------------------------------------------------------
  assign icache_line = icache_resp_data;
  assign line_offset = tlb_req_pc[5:0]; // byte offset within 64-byte line

  always_comb begin
    fetch_valid_mask = '0;
    for (int i = 0; i < FETCH_WIDTH; i++) begin
      int unsigned byte_idx;
      byte_idx = int'({line_offset[5:2], 2'b00}) + (i * 4);
      if (byte_idx + 3 < CACHE_LINE_BYTES) begin
        fetch_raw[i]         = icache_line[byte_idx*8 +: 32];
        fetch_valid_mask[i]  = icache_resp_valid && !icache_resp_error;
      end else begin
        fetch_raw[i]         = '0;
        fetch_valid_mask[i]  = 1'b0;
      end
    end
  end

  // --------------------------------------------------------------------------
  // Lightweight Pre-decode: populate uop fields known at fetch time
  // (full decode happens in decode_unit.sv)
  // --------------------------------------------------------------------------
  always_comb begin
    fetch_bundle_valid = |fetch_valid_mask && icache_resp_valid;
    fetch_bundle_tid   = active_tid;
    for (int i = 0; i < FETCH_WIDTH; i++) begin
      fetch_bundle[i]           = '0;
      fetch_bundle[i].raw_instr = fetch_raw[i];
      fetch_bundle[i].bundle_id = bundle_seq[active_tid];
      fetch_bundle[i].slot_idx  = 3'(i);
      fetch_bundle[i].bundle_start = (i == 0);
      fetch_bundle[i].vliw_mode = 1'b1;
      fetch_bundle[i].pc        = pc_reg[active_tid] + (i * 4);
      fetch_bundle[i].tid       = active_tid;
      fetch_bundle[i].valid     = fetch_valid_mask[i];
      // Preliminary opcode tag (decode stage fills the rest)
      fetch_bundle[i].opcode    = opcode_e'(fetch_raw[i][6:0]);
      // Quick branch detection for BP feedback
      fetch_bundle[i].is_branch = (fetch_raw[i][6:0] == OP_BRANCH) ||
                                  (fetch_raw[i][6:0] == OP_JAL)    ||
                                  (fetch_raw[i][6:0] == OP_JALR);
    end
  end

endmodule : ifu
