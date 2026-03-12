// ============================================================================
// Clione Processor — FPU Chiplet Top
// 2× FPU pipelines, NoC interface, bypass network
// ============================================================================
`include "clione_pkg.sv"

module fpu_chiplet_top
  import clione_pkg::*;
#(
  parameter logic [4:0] CHIPLET_NODE_ID = 5'h04
)(
  input  logic                            clk,
  input  logic                            rst_n,
  input  noc_pkt_t                        rx_pkt,
  input  logic                            rx_valid,
  output logic                            rx_ready,
  output noc_pkt_t                        tx_pkt,
  output logic                            tx_valid,
  input  logic                            tx_ready,
  output exec_result_t                    bypass_result,
  output logic                            bypass_valid,
  input  exec_result_t                    peer_bypass [7:0],
  input  logic                            peer_valid  [7:0]
);

  localparam int NUM_FPUS = 2;
  localparam int DISP_DEPTH = 8;

  typedef struct packed {
    logic [31:0]              instr;
    logic [XLEN-1:0]         src1, src2, src3;
    logic [ROB_PTR_WIDTH-1:0] rob_idx;
    logic [PREG_WIDTH-1:0]    phys_rd;
    logic [TID_WIDTH-1:0]     tid;
    logic                     valid;
  } fpu_disp_t;

  fpu_disp_t disp_q [DISP_DEPTH-1:0];
  logic [$clog2(DISP_DEPTH)-1:0] dq_head, dq_tail;
  assign rx_ready = (dq_tail + 1'b1) != dq_head;

  logic [XLEN-1:0]          fpu_result  [NUM_FPUS-1:0];
  logic [ROB_PTR_WIDTH-1:0] fpu_rob_idx [NUM_FPUS-1:0];
  logic [PREG_WIDTH-1:0]    fpu_phys_rd [NUM_FPUS-1:0];
  logic [TID_WIDTH-1:0]     fpu_tid     [NUM_FPUS-1:0];
  logic                     fpu_exc     [NUM_FPUS-1:0];
  logic [5:0]               fpu_exc_code[NUM_FPUS-1:0];
  logic                     fpu_valid   [NUM_FPUS-1:0];
  logic                     fpu_ready   [NUM_FPUS-1:0];
  logic [1:0]  rr_fpu;

  generate
    genvar gf;
    for (gf = 0; gf < NUM_FPUS; gf++) begin : gen_fpu
      fpu_pipe u_fpu (
        .clk          (clk),
        .rst_n        (rst_n),
        .instr        (disp_q[dq_head].instr),
        .src1         (disp_q[dq_head].src1),
        .src2         (disp_q[dq_head].src2),
        .src3         (disp_q[dq_head].src3),
        .rob_idx      (disp_q[dq_head].rob_idx),
        .phys_rd      (disp_q[dq_head].phys_rd),
        .tid          (disp_q[dq_head].tid),
        .valid        (!((dq_head == dq_tail)) && (rr_fpu == gf)),
        .ready        (fpu_ready[gf]),
        .result       (fpu_result[gf]),
        .fflags       (),
        .res_rob_idx  (fpu_rob_idx[gf]),
        .res_phys_rd  (fpu_phys_rd[gf]),
        .res_tid      (fpu_tid[gf]),
        .exception    (fpu_exc[gf]),
        .exc_code     (fpu_exc_code[gf]),
        .res_valid    (fpu_valid[gf])
      );
    end
  endgenerate

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      dq_head <= '0; dq_tail <= '0;
      rr_fpu  <= '0;
      tx_valid <= 1'b0;
      bypass_valid <= 1'b0;
    end else begin
      // Enqueue
      if (rx_valid && rx_ready && rx_pkt.pkt_type == NOC_DISPATCH) begin
        disp_q[dq_tail].instr   <= rx_pkt.data[31:0];
        disp_q[dq_tail].src1    <= rx_pkt.data[511:448];
        disp_q[dq_tail].src2    <= rx_pkt.data[447:384];
        disp_q[dq_tail].src3    <= rx_pkt.data[383:320];
        disp_q[dq_tail].rob_idx <= rx_pkt.rob_idx;
        disp_q[dq_tail].phys_rd <= rx_pkt.data[PREG_WIDTH+39:40];
        disp_q[dq_tail].tid     <= rx_pkt.tid;
        disp_q[dq_tail].valid   <= 1'b1;
        dq_tail <= dq_tail + 1'b1;
      end
      // Dequeue on dispatch
      if (dq_head != dq_tail && fpu_ready[rr_fpu]) begin
        dq_head <= dq_head + 1'b1;
        rr_fpu  <= rr_fpu + 1'b1;
      end
      // Result → NoC
      tx_valid     <= 1'b0;
      bypass_valid <= 1'b0;
      for (int i = 0; i < NUM_FPUS; i++) begin
        if (fpu_valid[i] && !tx_valid) begin
          tx_pkt.pkt_type      <= NOC_RESULT;
          tx_pkt.src_node      <= CHIPLET_NODE_ID;
          tx_pkt.dst_node      <= 5'h00;
          tx_pkt.rob_idx       <= fpu_rob_idx[i];
          tx_pkt.tid           <= fpu_tid[i];
          tx_pkt.data[PREG_WIDTH-1:0] <= fpu_phys_rd[i];
          tx_pkt.data[127:64]  <= fpu_result[i];
          tx_pkt.data[128]     <= fpu_exc[i];
          tx_pkt.valid         <= 1'b1;
          tx_valid <= 1'b1;
          bypass_result.phys_rd <= fpu_phys_rd[i];
          bypass_result.result  <= fpu_result[i];
          bypass_result.rob_idx <= fpu_rob_idx[i];
          bypass_result.tid     <= fpu_tid[i];
          bypass_result.valid   <= 1'b1;
          bypass_valid <= 1'b1;
        end
      end
    end
  end

endmodule : fpu_chiplet_top
