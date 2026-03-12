// ============================================================================
// Clione Processor — Core (Control + Scheduler) Die — Top Level
// Integrates: IFU, Branch Predictor, Decode, RAT/Rename, Issue Queues,
//             ROB, Physical RF, LSU, L1I, L1D, Scheduler → NoC
// This die is the "brain" — no heavy math execution happens here.
// ============================================================================
`include "clione_pkg.sv"

module core_die_top
  import clione_pkg::*;
(
  input  logic                            clk,
  input  logic                            rst_n,

  // SMT thread control (from OS/hypervisor)
  input  logic [SMT_WAYS-1:0]             thread_active,

  // NoC interface to execution chiplets
  output noc_pkt_t                        noc_tx_pkt   [ISSUE_WIDTH-1:0],
  output logic                            noc_tx_valid [ISSUE_WIDTH-1:0],
  input  logic                            noc_tx_ready [ISSUE_WIDTH-1:0],

  input  noc_pkt_t                        noc_rx_pkt   [ISSUE_WIDTH-1:0],
  input  logic                            noc_rx_valid [ISSUE_WIDTH-1:0],
  output logic                            noc_rx_ready [ISSUE_WIDTH-1:0],

  // NoC interface to cache chiplets (L2/L3)
  output noc_pkt_t                        cache_tx_pkt,
  output logic                            cache_tx_valid,
  input  logic                            cache_tx_ready,
  input  noc_pkt_t                        cache_rx_pkt,
  input  logic                            cache_rx_valid,
  output logic                            cache_rx_ready,

  // System interrupt / exception entry
  input  logic                            ext_interrupt,
  input  logic [63:0]                     int_vector,
  input  logic [TID_WIDTH-1:0]            int_tid
);

  // ==========================================================================
  // Internal Wire Declarations
  // ==========================================================================

  // IFU ↔ BP
  logic [63:0]          bp_next_pc   [SMT_WAYS-1:0];
  logic                 bp_taken      [SMT_WAYS-1:0];
  logic [63:0]          fetch_pc_to_bp [SMT_WAYS-1:0];
  logic                 fetch_valid_to_bp [SMT_WAYS-1:0];

  // IFU ↔ L1I
  logic [PADDR_WIDTH-1:0]          icache_paddr;
  logic                            icache_req_valid;
  logic                            icache_req_ready;
  logic [CACHE_LINE_BITS-1:0]      icache_resp_data;
  logic                            icache_resp_valid;
  logic                            icache_resp_error;

  // L1I ↔ L2 (via NoC)
  logic [CACHE_LINE_BITS-1:0]      l1i_fill_data;
  logic [PADDR_WIDTH-1:0]          l1i_fill_paddr;
  logic                            l1i_fill_valid;
  logic                            l1i_inv_valid;
  logic [PADDR_WIDTH-1:0]          l1i_inv_paddr;

  // IFU ↔ ITLB (simplified: tie off for now – real impl has separate ITLB)
  logic [PADDR_WIDTH-1:0]          itlb_paddr;
  logic                            itlb_hit;
  logic                            itlb_fault;
  assign itlb_paddr = {4'b0, icache_paddr[PADDR_WIDTH-5:0]}; // identity map stub
  assign itlb_hit   = 1'b1;
  assign itlb_fault = 1'b0;

  // IFU → Decode
  uop_t                            fetch_bundle [FETCH_WIDTH-1:0];
  logic                            fetch_bundle_valid;
  logic [TID_WIDTH-1:0]            fetch_bundle_tid;
  logic                            fetch_bundle_ready;

  // Decode → Rename
  uop_t                            decoded_bundle [DECODE_WIDTH-1:0];
  logic                            decoded_valid;
  logic [TID_WIDTH-1:0]            decoded_tid;
  logic                            decoded_ready;

  // Rename → ROB + IQ
  uop_t                            renamed_bundle [DECODE_WIDTH-1:0];
  logic                            renamed_valid;
  logic [TID_WIDTH-1:0]            renamed_tid;
  logic                            renamed_ready;

  // ROB allocation
  logic [ROB_PTR_WIDTH-1:0]        rob_alloc_idx [DECODE_WIDTH-1:0];
  logic                            rob_alloc_ready;

  // CDB from execution results
  exec_result_t                    cdb      [ISSUE_WIDTH-1:0];
  logic                            cdb_valid [ISSUE_WIDTH-1:0];

  // Issue queue outputs (per class)
  uop_t                            iq_int_uop [4-1:0];
  logic                            iq_int_vld [4-1:0];
  logic                            iq_int_rdy [4-1:0];
  uop_t                            iq_fp_uop  [4-1:0];
  logic                            iq_fp_vld  [4-1:0];
  logic                            iq_fp_rdy  [4-1:0];
  uop_t                            iq_vec_uop [4-1:0];
  logic                            iq_vec_vld [4-1:0];
  logic                            iq_vec_rdy [4-1:0];
  uop_t                            iq_mem_uop [4-1:0];
  logic                            iq_mem_vld [4-1:0];
  logic                            iq_mem_rdy [4-1:0];

  // Physical Register File
  logic [PREG_WIDTH-1:0]           rf_rd_addr [ISSUE_WIDTH*3-1:0];
  logic [XLEN-1:0]                rf_rd_data [ISSUE_WIDTH*3-1:0];
  logic [PREG_WIDTH-1:0]           rf_wr_addr [ISSUE_WIDTH-1:0];
  logic [XLEN-1:0]                rf_wr_data [ISSUE_WIDTH-1:0];
  logic                            rf_wr_valid[ISSUE_WIDTH-1:0];

  // ROB → retire feedback
  logic                            retire_valid [RETIRE_WIDTH-1:0];
  rob_entry_t                      retire_entry [RETIRE_WIDTH-1:0];
  logic [PREG_WIDTH-1:0]           retire_phys  [RETIRE_WIDTH-1:0];
  logic [TID_WIDTH-1:0]            retire_tid   [RETIRE_WIDTH-1:0];

  // Redirect / flush
  logic                            redirect_valid;
  logic [TID_WIDTH-1:0]            redirect_tid;
  logic [63:0]                     redirect_pc;
  logic                            flush_valid;
  logic [TID_WIDTH-1:0]            flush_tid;

  // RAT checkpoints
  logic [PREG_WIDTH-1:0]           ckpt_rat_int [SMT_WAYS-1:0][ARCH_INT_REGS-1:0];
  logic [PREG_WIDTH-1:0]           ckpt_rat_fp  [SMT_WAYS-1:0][ARCH_FP_REGS-1:0];

  // L1D
  logic [VADDR_WIDTH-1:0]          dtlb_vaddr_w;      // virtual address from LSU (57-bit)
  logic [TID_WIDTH-1:0]            dtlb_tid_w;
  logic                            dtlb_req_valid_w;
  logic                            dtlb_is_store_w;
  logic [PADDR_WIDTH-1:0]          dtlb_paddr_w;
  logic                            dtlb_hit_w;
  logic                            dtlb_fault_w;
  // Identity DTLB stub — strip upper canonical bits, give physical address
  assign dtlb_paddr_w = dtlb_vaddr_w[PADDR_WIDTH-1:0];
  assign dtlb_hit_w   = 1'b1;
  assign dtlb_fault_w = 1'b0;

  logic [PADDR_WIDTH-1:0]          ld_paddr_w;
  logic [1:0]                      ld_size_w;
  logic                            ld_signed_w;
  logic [ROB_PTR_WIDTH-1:0]        ld_rob_idx_w;
  logic [TID_WIDTH-1:0]            ld_tid_w;
  logic                            ld_valid_w;
  logic                            ld_ready_w;
  logic [XLEN-1:0]                ld_data_w;
  logic                            ld_resp_valid_w;
  logic [ROB_PTR_WIDTH-1:0]        ld_resp_rob_idx_w;

  logic [PADDR_WIDTH-1:0]          st_paddr_w;
  logic [XLEN-1:0]                st_data_w;
  logic [7:0]                      st_be_w;
  logic                            st_valid_w;
  logic                            st_ready_w;

  // L1D ↔ L2 NoC
  logic [CACHE_LINE_BITS-1:0]      l1d_fill_data;
  logic [PADDR_WIDTH-1:0]          l1d_fill_paddr;
  logic                            l1d_fill_valid;
  logic [CACHE_LINE_BITS-1:0]      l1d_wb_data;
  logic [PADDR_WIDTH-1:0]          l1d_wb_paddr;
  logic                            l1d_wb_valid;
  logic                            l1d_wb_ready;
  logic [PADDR_WIDTH-1:0]          l1d_snoop_paddr;
  logic                            l1d_snoop_inv;
  logic                            l1d_snoop_valid;
  logic [CACHE_LINE_BITS-1:0]      l1d_snoop_data;
  logic                            l1d_snoop_hit;

  // LSU result
  exec_result_t                    lsu_result [2-1:0];
  logic                            lsu_valid  [2-1:0];

  // ROB retire store
  logic                            rob_retire_store_w;
  logic [ROB_PTR_WIDTH-1:0]        rob_retire_rob_idx_w;

  // BP update
  logic                            bp_upd_valid;
  logic [TID_WIDTH-1:0]            bp_upd_tid;
  logic [63:0]                     bp_upd_pc;
  logic                            bp_upd_taken;
  logic                            bp_upd_mispredict;
  logic [63:0]                     bp_upd_target;
  logic [2:0]                      bp_upd_pred_tag;
  logic                            bp_upd_is_call;
  logic                            bp_upd_is_ret;

  // Fanout wires (iverilog-compatible alternative to '{N{scalar}} in ports)
  logic                    alloc_valid_fanout  [DECODE_WIDTH-1:0];
  logic [3:0]              cluster_hint_zero   [SMT_WAYS-1:0];
  always_comb begin
    for (int i = 0; i < DECODE_WIDTH; i++) alloc_valid_fanout[i] = renamed_valid;
    for (int i = 0; i < SMT_WAYS;     i++) cluster_hint_zero[i]  = 4'h0;
  end

  // ==========================================================================
  // CDB multiplexing: merge results from NoC receive and LSU
  // ==========================================================================
  always_comb begin
    for (int i = 0; i < ISSUE_WIDTH; i++) begin
      noc_rx_ready[i] = 1'b1;
      cdb[i]          = '0;
      cdb_valid[i]    = 1'b0;
      if (i < 2 && lsu_valid[i]) begin
        cdb[i]       = lsu_result[i];
        cdb_valid[i] = 1'b1;
      end else if (noc_rx_valid[i] && noc_rx_pkt[i].pkt_type == NOC_RESULT) begin
        cdb[i].phys_rd  = noc_rx_pkt[i].data[PREG_WIDTH-1:0]; // packed result
        cdb[i].result   = noc_rx_pkt[i].data[127:64];
        cdb[i].rob_idx  = noc_rx_pkt[i].rob_idx;
        cdb[i].tid      = noc_rx_pkt[i].tid;
        cdb[i].valid    = 1'b1;
        cdb_valid[i]    = 1'b1;
      end
    end
  end

  // RF writeback ports from CDB
  always_comb begin
    for (int i = 0; i < ISSUE_WIDTH; i++) begin
      rf_wr_addr[i]  = cdb[i].phys_rd;
      rf_wr_data[i]  = cdb[i].result;
      rf_wr_valid[i] = cdb_valid[i];
    end
  end

  // Retire BP update: look at retiring branch entries
  always_comb begin
    bp_upd_valid     = 1'b0;
    bp_upd_tid       = '0;
    bp_upd_pc        = '0;
    bp_upd_taken     = 1'b0;
    bp_upd_mispredict= flush_valid;
    bp_upd_target    = redirect_pc;
    bp_upd_pred_tag  = 3'd0;
    bp_upd_is_call   = 1'b0;
    bp_upd_is_ret    = 1'b0;

    for (int i = 0; i < RETIRE_WIDTH; i++) begin
      if (retire_valid[i] && retire_entry[i].is_branch) begin
        bp_upd_valid = 1'b1;
        bp_upd_tid   = retire_entry[i].tid;
        bp_upd_pc    = retire_entry[i].pc;
        break;
      end
    end
  end

  // Retire store signal for LSU SQ drain
  always_comb begin
    rob_retire_store_w    = 1'b0;
    rob_retire_rob_idx_w  = '0;
    for (int i = 0; i < RETIRE_WIDTH; i++) begin
      if (retire_valid[i] && retire_entry[i].is_store) begin
        rob_retire_store_w   = 1'b1;
        rob_retire_rob_idx_w = retire_entry[i].rob_idx;
      end
    end
  end

  // Renamed uops → ROB allocation index stamping
  always_comb begin
    renamed_ready = rob_alloc_ready;
    for (int i = 0; i < DECODE_WIDTH; i++)
      renamed_bundle[i].rob_idx = rob_alloc_idx[i];
  end

  // L2 NoC cache interface (simplified passthrough)
  assign cache_tx_valid = l1d_wb_valid;
  always_comb begin
    cache_tx_pkt          = '0;  // zero seq_id, data_mask, last, tid, rob_idx, etc.
    cache_tx_pkt.pkt_type = NOC_STORE_REQ;
    cache_tx_pkt.data[CACHE_LINE_BITS-1:0] = l1d_wb_data;
    cache_tx_pkt.addr     = l1d_wb_paddr;
    cache_tx_pkt.src_node = 5'h00;
    cache_tx_pkt.dst_node = 5'h10; // L2 cache chiplet node ID
    cache_tx_pkt.valid    = l1d_wb_valid;
  end
  assign l1d_wb_ready = cache_tx_ready;
  assign l1d_fill_valid = cache_rx_valid && cache_rx_pkt.pkt_type == NOC_LOAD_RESP;
  assign l1d_fill_data  = cache_rx_pkt.data[CACHE_LINE_BITS-1:0];
  assign l1d_fill_paddr = cache_rx_pkt.addr;
  assign cache_rx_ready = 1'b1;

  // I-cache fill (from L2 for instruction misses)
  assign l1i_fill_valid = 1'b0; // tied off; real: mux with L2 response channel
  assign l1i_fill_data  = '0;
  assign l1i_fill_paddr = '0;
  assign l1i_inv_valid  = 1'b0;
  assign l1i_inv_paddr  = '0;

  // Snoop: not driven from execution side in this die (simplified)
  assign l1d_snoop_paddr = '0;
  assign l1d_snoop_inv   = 1'b0;
  assign l1d_snoop_valid = 1'b0;

  // ==========================================================================
  // Submodule Instantiation
  // ==========================================================================

  // --- IFU ---
  ifu u_ifu (
    .clk                   (clk),
    .rst_n                 (rst_n),
    .thread_active         (thread_active),
    .redirect_valid        (redirect_valid),
    .redirect_tid          (redirect_tid),
    .redirect_pc           (redirect_pc),
    .bp_next_pc            (bp_next_pc),
    .bp_taken              (bp_taken),
    .fetch_pc              (fetch_pc_to_bp),
    .fetch_valid           (fetch_valid_to_bp),
    .icache_paddr          (icache_paddr),
    .icache_req_valid      (icache_req_valid),
    .icache_req_ready      (icache_req_ready),
    .icache_resp_data      (icache_resp_data),
    .icache_resp_valid     (icache_resp_valid),
    .icache_resp_error     (icache_resp_error),
    .tlb_vaddr             (),
    .tlb_tid               (),
    .tlb_req_valid         (),
    .tlb_paddr             (itlb_paddr),
    .tlb_hit               (itlb_hit),
    .tlb_fault             (itlb_fault),
    .fetch_bundle          (fetch_bundle),
    .fetch_bundle_valid    (fetch_bundle_valid),
    .fetch_bundle_tid      (fetch_bundle_tid),
    .fetch_bundle_ready    (fetch_bundle_ready)
  );

  // --- Branch Predictor ---
  branch_predictor u_bp (
    .clk            (clk),
    .rst_n          (rst_n),
    .req_pc         (fetch_pc_to_bp),
    .req_valid      (fetch_valid_to_bp),
    .pred_taken     (bp_taken),
    .pred_target    (bp_next_pc),
    .pred_valid     (),
    .pred_tag       (),
    .upd_valid      (bp_upd_valid),
    .upd_tid        (bp_upd_tid),
    .upd_pc         (bp_upd_pc),
    .upd_taken      (bp_upd_taken),
    .upd_mispredict (bp_upd_mispredict),
    .upd_target     (bp_upd_target),
    .upd_pred_tag   (bp_upd_pred_tag),
    .upd_is_call    (bp_upd_is_call),
    .upd_is_ret     (bp_upd_is_ret)
  );

  // --- L1I Cache ---
  l1i_cache u_l1i (
    .clk              (clk),
    .rst_n            (rst_n),
    .req_paddr        (icache_paddr),
    .req_valid        (icache_req_valid),
    .req_ready        (icache_req_ready),
    .resp_data        (icache_resp_data),
    .resp_valid       (icache_resp_valid),
    .resp_error       (icache_resp_error),
    .fill_data        (l1i_fill_data),
    .fill_paddr       (l1i_fill_paddr),
    .fill_valid       (l1i_fill_valid),
    .inv_valid        (l1i_inv_valid),
    .inv_paddr        (l1i_inv_paddr)
  );

  // --- Decode Unit ---
  decode_unit u_decode (
    .clk            (clk),
    .rst_n          (rst_n),
    .fetch_bundle   (fetch_bundle),
    .fetch_valid    (fetch_bundle_valid),
    .fetch_tid      (fetch_bundle_tid),
    .fetch_ready    (fetch_bundle_ready),
    .decode_bundle  (decoded_bundle),
    .decode_valid   (decoded_valid),
    .decode_tid     (decoded_tid),
    .decode_ready   (decoded_ready),
    .flush_valid    (flush_valid),
    .flush_tid      (flush_tid)
  );

  // --- Register Alias Table (Rename) ---
  register_alias_table u_rat (
    .clk              (clk),
    .rst_n            (rst_n),
    .dec_bundle       (decoded_bundle),
    .dec_valid        (decoded_valid),
    .dec_tid          (decoded_tid),
    .dec_ready        (decoded_ready),
    .ren_bundle       (renamed_bundle),
    .ren_valid        (renamed_valid),
    .ren_tid          (renamed_tid),
    .ren_ready        (renamed_ready),
    .retire_valid     (retire_valid),
    .retire_phys      (retire_phys),
    .retire_tid       (retire_tid),
    .flush_valid      (flush_valid),
    .flush_tid        (flush_tid),
    .ckpt_rat_int     (ckpt_rat_int),
    .ckpt_rat_fp      (ckpt_rat_fp)
  );

  // --- Reorder Buffer ---
  reorder_buffer u_rob (
    .clk                  (clk),
    .rst_n                (rst_n),
    .alloc_uop            (renamed_bundle),
    .alloc_valid          (alloc_valid_fanout),
    .alloc_idx            (rob_alloc_idx),
    .alloc_ready          (rob_alloc_ready),
    .cdb                  (cdb),
    .cdb_valid            (cdb_valid),
    .retire_valid         (retire_valid),
    .retire_entry         (retire_entry),
    .retire_phys          (retire_phys),
    .retire_tid           (retire_tid),
    .ckpt_rat_int         (ckpt_rat_int),
    .ckpt_rat_fp          (ckpt_rat_fp),
    .redirect_valid       (redirect_valid),
    .redirect_tid         (redirect_tid),
    .redirect_pc          (redirect_pc),
    .flush_valid          (flush_valid),
    .flush_tid            (flush_tid),
    .rob_used_count       ()
  );

  // --- Physical Register File ---
  phys_register_file u_prf (
    .clk      (clk),
    .rst_n    (rst_n),
    .rd_addr  (rf_rd_addr),
    .rd_data  (rf_rd_data),
    .wr_addr  (rf_wr_addr),
    .wr_data  (rf_wr_data),
    .wr_valid (rf_wr_valid)
  );

  // --- Issue Queues (one per class) ---
  // INT IQ: accepts INT(0), MUL(1), BRANCH(5), SYS(6), CRYPTO(7) — routed to correct chiplet by scheduler
  issue_queue #(.IQ_DEPTH(IQ_INT_DEPTH), .IQ_ICLASS_MASK(8'hE3), .IQ_ISSUE_WIDTH(4))
  u_iq_int (
    .clk          (clk),
    .rst_n        (rst_n),
    .alloc_uop    (renamed_bundle),
    .alloc_valid  (alloc_valid_fanout),
    .alloc_ready  (),
    .issue_uop    (iq_int_uop),
    .issue_valid  (iq_int_vld),
    .issue_ready  (iq_int_rdy),
    .cdb          (cdb),
    .cdb_valid    (cdb_valid),
    .flush_valid  (flush_valid),
    .flush_tid    (flush_tid)
  );

  // FP IQ: accepts FP(2) only
  issue_queue #(.IQ_DEPTH(IQ_FP_DEPTH), .IQ_ICLASS_MASK(8'h04), .IQ_ISSUE_WIDTH(4))
  u_iq_fp (
    .clk          (clk),
    .rst_n        (rst_n),
    .alloc_uop    (renamed_bundle),
    .alloc_valid  (alloc_valid_fanout),
    .alloc_ready  (),
    .issue_uop    (iq_fp_uop),
    .issue_valid  (iq_fp_vld),
    .issue_ready  (iq_fp_rdy),
    .cdb          (cdb),
    .cdb_valid    (cdb_valid),
    .flush_valid  (flush_valid),
    .flush_tid    (flush_tid)
  );

  // VEC IQ: accepts VEC(3) only
  issue_queue #(.IQ_DEPTH(IQ_VEC_DEPTH), .IQ_ICLASS_MASK(8'h08), .IQ_ISSUE_WIDTH(4))
  u_iq_vec (
    .clk          (clk),
    .rst_n        (rst_n),
    .alloc_uop    (renamed_bundle),
    .alloc_valid  (alloc_valid_fanout),
    .alloc_ready  (),
    .issue_uop    (iq_vec_uop),
    .issue_valid  (iq_vec_vld),
    .issue_ready  (iq_vec_rdy),
    .cdb          (cdb),
    .cdb_valid    (cdb_valid),
    .flush_valid  (flush_valid),
    .flush_tid    (flush_tid)
  );

  // MEM IQ: accepts MEM(4) only
  issue_queue #(.IQ_DEPTH(IQ_MEM_DEPTH), .IQ_ICLASS_MASK(8'h10), .IQ_ISSUE_WIDTH(4))
  u_iq_mem (
    .clk          (clk),
    .rst_n        (rst_n),
    .alloc_uop    (renamed_bundle),
    .alloc_valid  (alloc_valid_fanout),
    .alloc_ready  (),
    .issue_uop    (iq_mem_uop),
    .issue_valid  (iq_mem_vld),
    .issue_ready  (iq_mem_rdy),
    .cdb          (cdb),
    .cdb_valid    (cdb_valid),
    .flush_valid  (flush_valid),
    .flush_tid    (flush_tid)
  );

  // --- Clustered OoO Scheduler → NoC ---
  clustered_ooo_scheduler u_sched (
    .clk                  (clk),
    .rst_n                (rst_n),
    .iq_uop_int           (iq_int_uop),
    .iq_vld_int           (iq_int_vld),
    .iq_rdy_int           (iq_int_rdy),
    .iq_uop_fp            (iq_fp_uop),
    .iq_vld_fp            (iq_fp_vld),
    .iq_rdy_fp            (iq_fp_rdy),
    .iq_uop_vec           (iq_vec_uop),
    .iq_vld_vec           (iq_vec_vld),
    .iq_rdy_vec           (iq_vec_rdy),
    .iq_uop_mem           (iq_mem_uop),
    .iq_vld_mem           (iq_mem_vld),
    .iq_rdy_mem           (iq_mem_rdy),
    .dispatch_pkt         (noc_tx_pkt),
    .dispatch_valid       (noc_tx_valid),
    .dispatch_ready       (noc_tx_ready),
    .rf_rd_addr           (rf_rd_addr),
    .rf_rd_data           (rf_rd_data),
    .thread_cluster_hint  (cluster_hint_zero),
    .cdb                  (cdb),
    .cdb_valid            (cdb_valid),
    .flush_valid          (flush_valid),
    .flush_tid            (flush_tid)
  );

  // --- Load/Store Unit ---
  load_store_unit u_lsu (
    .clk                  (clk),
    .rst_n                (rst_n),
    .issue_uop            (iq_mem_uop),
    .issue_valid          (iq_mem_vld),
    .issue_ready          (iq_mem_rdy),
    .lsu_result           (lsu_result),
    .lsu_valid            (lsu_valid),
    .dtlb_vaddr           (dtlb_vaddr_w),
    .dtlb_tid             (dtlb_tid_w),
    .dtlb_req_valid       (dtlb_req_valid_w),
    .dtlb_is_store        (dtlb_is_store_w),
    .dtlb_paddr           (dtlb_paddr_w),
    .dtlb_hit             (dtlb_hit_w),
    .dtlb_fault           (dtlb_fault_w),
    .ld_paddr             (ld_paddr_w),
    .ld_size              (ld_size_w),
    .ld_signed            (ld_signed_w),
    .ld_rob_idx           (ld_rob_idx_w),
    .ld_tid               (ld_tid_w),
    .ld_valid             (ld_valid_w),
    .ld_ready             (ld_ready_w),
    .ld_data              (ld_data_w),
    .ld_resp_valid        (ld_resp_valid_w),
    .ld_resp_rob_idx      (ld_resp_rob_idx_w),
    .st_paddr             (st_paddr_w),
    .st_data              (st_data_w),
    .st_be                (st_be_w),
    .st_valid             (st_valid_w),
    .st_ready             (st_ready_w),
    .rob_retire_store     (rob_retire_store_w),
    .rob_retire_rob_idx   (rob_retire_rob_idx_w),
    .flush_valid          (flush_valid),
    .flush_tid            (flush_tid)
  );

  // --- L1D Cache ---
  l1d_cache u_l1d (
    .clk              (clk),
    .rst_n            (rst_n),
    .ld_paddr         (ld_paddr_w),
    .ld_size          (ld_size_w),
    .ld_signed        (ld_signed_w),
    .ld_rob_idx       (ld_rob_idx_w),
    .ld_tid           (ld_tid_w),
    .ld_valid         (ld_valid_w),
    .ld_ready         (ld_ready_w),
    .ld_data          (ld_data_w),
    .ld_resp_valid    (ld_resp_valid_w),
    .ld_resp_rob_idx  (ld_resp_rob_idx_w),
    .st_paddr         (st_paddr_w),
    .st_data          (st_data_w),
    .st_be            (st_be_w),
    .st_valid         (st_valid_w),
    .st_ready         (st_ready_w),
    .fill_data        (l1d_fill_data),
    .fill_paddr       (l1d_fill_paddr),
    .fill_valid       (l1d_fill_valid),
    .wb_data          (l1d_wb_data),
    .wb_paddr         (l1d_wb_paddr),
    .wb_valid         (l1d_wb_valid),
    .wb_ready         (l1d_wb_ready),
    .snoop_paddr      (l1d_snoop_paddr),
    .snoop_inv        (l1d_snoop_inv),
    .snoop_valid      (l1d_snoop_valid),
    .snoop_data       (l1d_snoop_data),
    .snoop_hit        (l1d_snoop_hit)
  );

endmodule : core_die_top
