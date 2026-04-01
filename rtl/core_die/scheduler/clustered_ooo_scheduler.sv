// ============================================================================
// Clione Processor — Clustered Out-of-Order Scheduler
// Dispatches uops to execution chiplet clusters based on iclass
// Manages per-chiplet occupancy, back-pressure, and SMT thread affinity
// ============================================================================
`include "clione_pkg.sv"

module clustered_ooo_scheduler
  import clione_pkg::*;
(
  input  logic                            clk,
  input  logic                            rst_n,

  // From rename / issue queues — renamed uops ready to issue
  input  uop_t                            iq_uop_int  [4-1:0],  // 4 INT slots
  input  logic                            iq_vld_int  [4-1:0],
  output logic                            iq_rdy_int  [4-1:0],

  input  uop_t                            iq_uop_fp   [4-1:0],  // 4 FP slots
  input  logic                            iq_vld_fp   [4-1:0],
  output logic                            iq_rdy_fp   [4-1:0],

  input  uop_t                            iq_uop_vec  [4-1:0],  // 4 VEC slots
  input  logic                            iq_vld_vec  [4-1:0],
  output logic                            iq_rdy_vec  [4-1:0],

  input  uop_t                            iq_uop_mem  [4-1:0],  // 4 MEM slots
  input  logic                            iq_vld_mem  [4-1:0],
  output logic                            iq_rdy_mem  [4-1:0],

  // Outputs: instruction packets sent to NoC for chiplet dispatch
  // Each output maps to one NoC injection port toward the chiplet
  output noc_pkt_t                        dispatch_pkt     [ISSUE_WIDTH-1:0],
  output logic                            dispatch_valid   [ISSUE_WIDTH-1:0],
  input  logic                            dispatch_ready   [ISSUE_WIDTH-1:0], // NoC backpressure

  // Register file reads: scheduler issues read requests
  // (RF is on control die, result forwarded to chiplet via NoC)
  output logic [PREG_WIDTH-1:0]           rf_rd_addr [ISSUE_WIDTH*3-1:0], // rs1, rs2, rs3 per uop
  input  logic [XLEN-1:0]                 rf_rd_data [ISSUE_WIDTH*3-1:0],

  // SMT thread-to-cluster affinity hints (from OS/runtime)
  input  logic [3:0]                      thread_cluster_hint [SMT_WAYS-1:0],

  // CDB for operand forwarding to dispatch packets
  input  exec_result_t                    cdb         [ISSUE_WIDTH-1:0],
  input  logic                            cdb_valid   [ISSUE_WIDTH-1:0],

  // Flush
  input  logic                            flush_valid,
  input  logic [TID_WIDTH-1:0]            flush_tid
);

  // --------------------------------------------------------------------------
  // Cluster Occupancy Tracking: how many in-flight uops per chiplet cluster
  // --------------------------------------------------------------------------
  localparam int MAX_INFLIGHT = 32;
  logic [$clog2(MAX_INFLIGHT):0] cluster_inflight [16];  // 16 possible cluster IDs

  // --------------------------------------------------------------------------
  // Dispatch Arbitration: round-robin across cluster instances
  // For INT: 4 ALU chiplets, pick least-loaded
  // For FP:  2 FPU chiplets
  // For VEC: 2 SIMD chiplets
  // For MEM: 1 LSU (on control die, no NoC hop needed)
  // --------------------------------------------------------------------------
  function automatic logic [3:0] pick_cluster(
    input iclass_e iclass,
    input logic [$clog2(MAX_INFLIGHT):0] inflight [16]
  );
    // Select least-loaded cluster of the appropriate type
    unique case (iclass)
      ICLASS_INT, ICLASS_MUL: begin
        // ALU chiplets: CLUSTER_ALU0..ALU3, pick min inflight
        automatic logic [3:0] best = 4'(CLUSTER_ALU0);
        for (int i = 1; i <= 3; i++) begin
          if (inflight[i] < inflight[best]) best = 4'(i);
        end
        return best;
      end
      ICLASS_FP: begin
        automatic logic [3:0] best = 4'(CLUSTER_FPU0);
        return (inflight[CLUSTER_FPU1] < inflight[CLUSTER_FPU0])
               ? 4'(CLUSTER_FPU1) : 4'(CLUSTER_FPU0);
      end
      ICLASS_VEC: begin
        return (inflight[CLUSTER_SIMD1] < inflight[CLUSTER_SIMD0])
               ? 4'(CLUSTER_SIMD1) : 4'(CLUSTER_SIMD0);
      end
      ICLASS_CRYPTO: return 4'(CLUSTER_CRYPTO);
      ICLASS_MEM:    return 4'(CLUSTER_LSU);
      default:       return 4'(CLUSTER_ALU0);
    endcase
  endfunction

  // --------------------------------------------------------------------------
  // Dispatch Packet Construction
  // Pack uop + register data into NoC packet for chiplet
  // --------------------------------------------------------------------------
  // We have ISSUE_WIDTH=8 dispatch slots total
  // Assign: slots 0-3 for INT, 4-5 for FP, 6-7 for VEC/MEM/crypto
  // (simplified static assignment; dynamic arbitration in full impl)

  typedef struct packed {
    uop_t  uop;
    logic [XLEN-1:0] src1_data;
    logic [XLEN-1:0] src2_data;
    logic [XLEN-1:0] src3_data;
  } dispatch_entry_t;

  dispatch_entry_t disp_ent [ISSUE_WIDTH-1:0];

  always_comb begin
    // Zero defaults
    for (int i = 0; i < ISSUE_WIDTH; i++) begin
      dispatch_pkt[i]   = '0;
      dispatch_valid[i] = 1'b0;
      disp_ent[i]       = '0;
    end
    for (int i = 0; i < 4; i++) begin
      iq_rdy_int[i] = 1'b0;
      iq_rdy_fp[i]  = 1'b0;
      iq_rdy_vec[i] = 1'b0;
      iq_rdy_mem[i] = 1'b0;
    end

    // Slots 0-3: INT cluster
    for (int i = 0; i < 4; i++) begin
      logic [3:0] cid;
      cid = 4'(CLUSTER_ALU0);
      if (iq_vld_int[i] && dispatch_ready[i] && !flush_valid) begin
        cid = pick_cluster(iq_uop_int[i].iclass, cluster_inflight);
        disp_ent[i].uop       = iq_uop_int[i];
        disp_ent[i].src1_data = rf_rd_data[i*3+0];
        disp_ent[i].src2_data = rf_rd_data[i*3+1];
        disp_ent[i].src3_data = rf_rd_data[i*3+2];

        // CDB forwarding override: if result just became available, bypass RF
        for (int c = 0; c < ISSUE_WIDTH; c++) begin
          if (cdb_valid[c] && cdb[c].valid) begin
            if (cdb[c].phys_rd == iq_uop_int[i].phys_rs1)
              disp_ent[i].src1_data = cdb[c].result;
            if (cdb[c].phys_rd == iq_uop_int[i].phys_rs2)
              disp_ent[i].src2_data = cdb[c].result;
          end
        end

        dispatch_pkt[i].pkt_type = NOC_DISPATCH;
        dispatch_pkt[i].src_node = 5'h00;   // Control die = node 0
        dispatch_pkt[i].dst_node = 5'(cid); // Cluster node
        dispatch_pkt[i].tid      = iq_uop_int[i].tid;
        dispatch_pkt[i].rob_idx  = iq_uop_int[i].rob_idx;
        dispatch_pkt[i].seq_id   = 8'(i);

        // Encode uop + operand data into the 512-bit data payload
        // Layout: [511:448]=src1 [447:384]=src2 [383:320]=src3 [319:0]=uop bits
        dispatch_pkt[i].data[511:448] = disp_ent[i].src1_data;
        dispatch_pkt[i].data[447:384] = disp_ent[i].src2_data;
        dispatch_pkt[i].data[383:320] = disp_ent[i].src3_data;
        // Pack key uop fields consumed by chiplets
        dispatch_pkt[i].data[31:0]    = iq_uop_int[i].raw_instr;
        dispatch_pkt[i].data[32]      = iq_uop_int[i].has_imm;
        dispatch_pkt[i].data[33]      = iq_uop_int[i].is_branch;
        dispatch_pkt[i].data[35:34]   = iq_uop_int[i].mode;
        dispatch_pkt[i].data[38:36]   = iq_uop_int[i].op_width;
        dispatch_pkt[i].data[PREG_WIDTH+39:40] = iq_uop_int[i].phys_rd;
        dispatch_pkt[i].data[127:64]  = iq_uop_int[i].pc;

        dispatch_valid[i] = 1'b1;
      end
      iq_rdy_int[i] = dispatch_ready[i];
    end

    // Slots 4-5: FP cluster
    for (int i = 0; i < 2; i++) begin
      logic [3:0] cid;
      cid = 4'(CLUSTER_FPU0);
      if (iq_vld_fp[i] && dispatch_ready[4+i] && !flush_valid) begin
        cid = pick_cluster(ICLASS_FP, cluster_inflight);
        dispatch_pkt[4+i].pkt_type = NOC_DISPATCH;
        dispatch_pkt[4+i].src_node = 5'h00;
        dispatch_pkt[4+i].dst_node = 5'(cid);
        dispatch_pkt[4+i].tid      = iq_uop_fp[i].tid;
        dispatch_pkt[4+i].rob_idx  = iq_uop_fp[i].rob_idx;
        dispatch_pkt[4+i].data[511:448] = rf_rd_data[(4+i)*3+0];
        dispatch_pkt[4+i].data[447:384] = rf_rd_data[(4+i)*3+1];
        dispatch_pkt[4+i].data[383:320] = rf_rd_data[(4+i)*3+2];
        dispatch_pkt[4+i].data[31:0]    = iq_uop_fp[i].raw_instr;
        dispatch_pkt[4+i].data[32]      = iq_uop_fp[i].has_imm;
        dispatch_pkt[4+i].data[33]      = iq_uop_fp[i].is_branch;
        dispatch_pkt[4+i].data[35:34]   = iq_uop_fp[i].mode;
        dispatch_pkt[4+i].data[38:36]   = iq_uop_fp[i].op_width;
        dispatch_pkt[4+i].data[PREG_WIDTH+39:40] = iq_uop_fp[i].phys_rd;
        dispatch_pkt[4+i].data[127:64]  = iq_uop_fp[i].pc;
        dispatch_valid[4+i] = 1'b1;
      end
      iq_rdy_fp[i] = dispatch_ready[4+i];
    end

    // Slots 6-7: VEC / CRYPTO
    for (int i = 0; i < 2; i++) begin
      if (iq_vld_vec[i] && dispatch_ready[6+i] && !flush_valid) begin
        dispatch_pkt[6+i].pkt_type = NOC_DISPATCH;
        dispatch_pkt[6+i].src_node = 5'h00;
        dispatch_pkt[6+i].dst_node = 5'(CLUSTER_SIMD0) + 5'(i);
        dispatch_pkt[6+i].data[511:448] = rf_rd_data[(6+i)*3+0];
        dispatch_pkt[6+i].data[447:384] = rf_rd_data[(6+i)*3+1];
        dispatch_pkt[6+i].data[31:0]    = iq_uop_vec[i].raw_instr;
        dispatch_pkt[6+i].data[32]      = iq_uop_vec[i].has_imm;
        dispatch_pkt[6+i].data[33]      = iq_uop_vec[i].is_branch;
        dispatch_pkt[6+i].data[35:34]   = iq_uop_vec[i].mode;
        dispatch_pkt[6+i].data[38:36]   = iq_uop_vec[i].op_width;
        dispatch_pkt[6+i].data[PREG_WIDTH+39:40] = iq_uop_vec[i].phys_rd;
        dispatch_pkt[6+i].data[127:64]  = iq_uop_vec[i].pc;
        dispatch_valid[6+i] = 1'b1;
      end
      iq_rdy_vec[i] = dispatch_ready[6+i];
    end

    // RF read address ports
    for (int i = 0; i < ISSUE_WIDTH; i++) begin
      rf_rd_addr[i*3+0] = '0;
      rf_rd_addr[i*3+1] = '0;
      rf_rd_addr[i*3+2] = '0;
    end
    for (int i = 0; i < 4; i++) begin
      if (iq_vld_int[i]) begin
        rf_rd_addr[i*3+0] = iq_uop_int[i].phys_rs1;
        rf_rd_addr[i*3+1] = iq_uop_int[i].phys_rs2;
        rf_rd_addr[i*3+2] = iq_uop_int[i].phys_rs3;
      end
    end
    for (int i = 0; i < 2; i++) begin
      if (iq_vld_fp[i]) begin
        rf_rd_addr[(4+i)*3+0] = iq_uop_fp[i].phys_rs1;
        rf_rd_addr[(4+i)*3+1] = iq_uop_fp[i].phys_rs2;
        rf_rd_addr[(4+i)*3+2] = iq_uop_fp[i].phys_rs3;
      end
    end

    // MEM: always handled on control die, forwarded to LSU directly
    for (int i = 0; i < 4; i++)
      iq_rdy_mem[i] = 1'b1; // stall when LSU full (simplified)
  end

  // --------------------------------------------------------------------------
  // Cluster Inflight Counter
  // --------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i < 16; i++)
        cluster_inflight[i] <= '0;
    end else begin
      for (int i = 0; i < ISSUE_WIDTH; i++) begin
        if (dispatch_valid[i] && dispatch_ready[i]) begin
          automatic logic [3:0] cid = dispatch_pkt[i].dst_node[3:0];
          cluster_inflight[cid] <= cluster_inflight[cid] + 1'b1;
        end
      end
      // Decrement when result comes back on CDB
      for (int c = 0; c < ISSUE_WIDTH; c++) begin
        if (cdb_valid[c] && cdb[c].valid) begin
          // Determine cluster from result (need extra field in real design)
          // Simplified: decrement all by 1 on valid result
          // Real impl: tag each in-flight with cluster ID
        end
      end
      // Flush: zero out inflight for flushed thread
      if (flush_valid) begin
        for (int i = 0; i < 16; i++)
          cluster_inflight[i] <= '0; // Conservative: flush all (refine per TID)
      end
    end
  end

endmodule : clustered_ooo_scheduler
