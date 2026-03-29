// ============================================================================
// Clione Processor — ALU Chiplet Top
// Receives NoC dispatch packets, runs 4× parallel ALU pipelines,
// returns results via NoC result packets
// ============================================================================
`include "clione_pkg.sv"

module alu_chiplet_top
  import clione_pkg::*;
#(
  parameter logic [4:0] CHIPLET_NODE_ID = 5'h01  // Node ID on NoC
)(
  input  logic                            clk,
  input  logic                            rst_n,

  // NoC receive: dispatch from control die
  input  noc_pkt_t                        rx_pkt,
  input  logic                            rx_valid,
  output logic                            rx_ready,

  // NoC transmit: result back to control die
  output noc_pkt_t                        tx_pkt,
  output logic                            tx_valid,
  input  logic                            tx_ready,

  // Chiplet-level bypass: forward result directly to sibling chiplets
  // (avoids round-trip to control die for dependent instructions)
  output exec_result_t                    bypass_result,
  output logic                            bypass_valid,
  input  exec_result_t                    peer_bypass   [7:0],  // from other chiplets
  input  logic                            peer_valid    [7:0]
);

  // --------------------------------------------------------------------------
  // Dispatch Queue: buffer incoming NoC packets before ALU pipelines
  // --------------------------------------------------------------------------
  localparam int DISP_DEPTH = 16;

  typedef struct packed {
    logic [31:0]              instr;
    logic [XLEN-1:0]         src1;
    logic [XLEN-1:0]         src2;
    logic [XLEN-1:0]         imm;
    logic                     has_imm;
    seabird_mode_e            mode;
    op_width_e                op_width;
    logic [ROB_PTR_WIDTH-1:0] rob_idx;
    logic [PREG_WIDTH-1:0]    phys_rd;
    logic [TID_WIDTH-1:0]     tid;
    logic                     is_branch;
    logic [63:0]              pc;
    logic                     valid;
  } disp_entry_t;

  disp_entry_t disp_q  [DISP_DEPTH-1:0];
  logic [$clog2(DISP_DEPTH)-1:0] dq_head, dq_tail;
  logic                          dq_empty, dq_full;

  assign dq_empty = (dq_head == dq_tail);
  assign dq_full  = ((dq_tail + 1'b1) == dq_head);
  assign rx_ready = !dq_full;

  // --------------------------------------------------------------------------
  // 4× ALU Pipe instances
  // --------------------------------------------------------------------------
  localparam int NUM_ALUS = 4;

  logic [XLEN-1:0]          alu_result     [NUM_ALUS-1:0];
  logic [ROB_PTR_WIDTH-1:0] alu_rob_idx    [NUM_ALUS-1:0];
  logic [PREG_WIDTH-1:0]    alu_phys_rd    [NUM_ALUS-1:0];
  logic [TID_WIDTH-1:0]     alu_tid        [NUM_ALUS-1:0];
  logic                     alu_branch_taken [NUM_ALUS-1:0];
  logic [63:0]              alu_branch_target[NUM_ALUS-1:0];
  logic                     alu_exc        [NUM_ALUS-1:0];
  logic [5:0]               alu_exc_code   [NUM_ALUS-1:0];
  logic                     alu_res_valid  [NUM_ALUS-1:0];
  logic                     alu_ready      [NUM_ALUS-1:0];

  // Dispatch side: feed dispatch queue head to least-loaded ALU
  logic [1:0]  rr_alu; // round-robin ALU selector

  generate
    genvar ga;
    for (ga = 0; ga < NUM_ALUS; ga++) begin : gen_alu
      disp_entry_t alu_in;
      logic        alu_in_valid;

      // Each ALU gets instructions when dispatch queue has entries and it's
      // this ALU's turn (round-robin)
      assign alu_in       = disp_q[dq_head];
      assign alu_in_valid = !dq_empty && (rr_alu == ga);

      alu_pipe u_alu (
        .clk            (clk),
        .rst_n          (rst_n),
        .instr          (alu_in.instr),
        .src1           (alu_in.src1),
        .src2           (alu_in.src2),
        .imm            (alu_in.imm),
        .has_imm        (alu_in.has_imm),
        .mode_i         (alu_in.mode),
        .op_width_i     (alu_in.op_width),
        .rob_idx        (alu_in.rob_idx),
        .phys_rd        (alu_in.phys_rd),
        .tid            (alu_in.tid),
        .is_branch      (alu_in.is_branch),
        .pc             (alu_in.pc),
        .valid          (alu_in_valid),
        .ready          (alu_ready[ga]),
        .result         (alu_result[ga]),
        .res_rob_idx    (alu_rob_idx[ga]),
        .res_phys_rd    (alu_phys_rd[ga]),
        .res_tid        (alu_tid[ga]),
        .branch_taken   (alu_branch_taken[ga]),
        .branch_target  (alu_branch_target[ga]),
        .exception      (alu_exc[ga]),
        .exc_code       (alu_exc_code[ga]),
        .res_valid      (alu_res_valid[ga])
      );
    end
  endgenerate

  // --------------------------------------------------------------------------
  // Dispatch Queue Management
  // --------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      dq_head <= '0;
      dq_tail <= '0;
      rr_alu  <= '0;
      for (int i = 0; i < DISP_DEPTH; i++)
        disp_q[i] <= '0;
    end else begin
      // Enqueue incoming NoC dispatch packet
      if (rx_valid && !dq_full && rx_pkt.pkt_type == NOC_DISPATCH) begin
        disp_q[dq_tail].instr     <= rx_pkt.data[31:0];
        disp_q[dq_tail].src1      <= rx_pkt.data[511:448];
        disp_q[dq_tail].src2      <= rx_pkt.data[447:384];
        disp_q[dq_tail].imm       <= rx_pkt.data[383:320];
        disp_q[dq_tail].has_imm   <= rx_pkt.data[32];
        disp_q[dq_tail].mode      <= seabird_mode_e'(rx_pkt.data[35:34]);
        disp_q[dq_tail].op_width  <= op_width_e'(rx_pkt.data[38:36]);
        disp_q[dq_tail].rob_idx   <= rx_pkt.rob_idx;
        disp_q[dq_tail].phys_rd   <= rx_pkt.data[PREG_WIDTH+39:40];
        disp_q[dq_tail].tid       <= rx_pkt.tid;
        disp_q[dq_tail].is_branch <= rx_pkt.data[33];
        disp_q[dq_tail].pc        <= rx_pkt.data[127:64];
        disp_q[dq_tail].valid     <= 1'b1;
        dq_tail <= dq_tail + 1'b1;
      end

      // Dispatch to ALU (dequeue from head)
      if (!dq_empty && alu_ready[rr_alu]) begin
        dq_head <= dq_head + 1'b1;
        rr_alu  <= rr_alu + 1'b1;
      end
    end
  end

  // --------------------------------------------------------------------------
  // Result Arbitration: merge 4 ALU outputs → single NoC TX port
  // Round-robin with priority on valid outputs
  // --------------------------------------------------------------------------
  logic [1:0] res_rr;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tx_valid   <= 1'b0;
      tx_pkt     <= '0;
      bypass_valid  <= 1'b0;
      bypass_result <= '0;
      res_rr     <= '0;
    end else begin
      tx_valid      <= 1'b0;
      bypass_valid  <= 1'b0;

      for (int i = 0; i < NUM_ALUS; i++) begin
        automatic int ai = (res_rr + i) % NUM_ALUS;
        if (alu_res_valid[ai] && !tx_valid) begin
          // Pack result into NoC packet
          tx_pkt.pkt_type <= NOC_RESULT;
          tx_pkt.src_node <= CHIPLET_NODE_ID;
          tx_pkt.dst_node <= 5'h00;  // Back to control die
          tx_pkt.rob_idx  <= alu_rob_idx[ai];
          tx_pkt.tid      <= alu_tid[ai];
          tx_pkt.valid    <= 1'b1;

          // Result data payload layout
          tx_pkt.data[PREG_WIDTH-1:0]  <= alu_phys_rd[ai];
          tx_pkt.data[127:64]          <= alu_result[ai];
          tx_pkt.data[128]             <= alu_exc[ai];
          tx_pkt.data[134:129]         <= alu_exc_code[ai];
          tx_pkt.data[135]             <= alu_branch_taken[ai];
          tx_pkt.data[199:136]         <= alu_branch_target[ai];

          tx_valid <= 1'b1;

          // Chiplet bypass
          bypass_result.phys_rd       <= alu_phys_rd[ai];
          bypass_result.result        <= alu_result[ai];
          bypass_result.rob_idx       <= alu_rob_idx[ai];
          bypass_result.tid           <= alu_tid[ai];
          bypass_result.exception     <= alu_exc[ai];
          bypass_result.exc_code      <= alu_exc_code[ai];
          bypass_result.branch_taken  <= alu_branch_taken[ai];
          bypass_result.branch_target <= alu_branch_target[ai];
          bypass_result.valid         <= 1'b1;
          bypass_valid <= 1'b1;

          res_rr <= 2'(ai + 1);
          break;
        end
      end
    end
  end

endmodule : alu_chiplet_top
