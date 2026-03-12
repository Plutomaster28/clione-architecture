// ============================================================================
// Clione Processor — SIMD / Vector Execution Unit
// 512-bit vector registers, RISC-V V-extension inspired
// Operations: vadd, vsub, vmul, vfma, vshift, vload, vstore, vdot
// Also handles matrix multiply tiles for AI workloads (outer product)
// ============================================================================
`include "clione_pkg.sv"

module simd_chiplet_top
  import clione_pkg::*;
#(
  parameter logic [4:0] CHIPLET_NODE_ID = 5'h06,
  parameter int VLEN = 512   // Vector register width in bits
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

  // --------------------------------------------------------------------------
  // Vector Register File: 64 physical vector regs × 512 bits
  // --------------------------------------------------------------------------
  localparam int VREG_COUNT = 64;
  localparam int VREG_WIDTH = VLEN; // 512 bits each

  logic [VREG_WIDTH-1:0] vrf [VREG_COUNT-1:0];

  // --------------------------------------------------------------------------
  // Dispatch Queue
  // --------------------------------------------------------------------------
  localparam int DISP_DEPTH = 8;

  typedef struct packed {
    logic [31:0]              instr;
    logic [XLEN-1:0]         src_scalar; // Scalar value if mixed op
    logic [VREG_WIDTH-1:0]   vd_data;    // destination vector (for writes)
    logic [5:0]               vrs1;       // Physical vector source 1
    logic [5:0]               vrs2;       // Physical vector source 2
    logic [5:0]               vrd;        // Physical vector destination
    logic [ROB_PTR_WIDTH-1:0] rob_idx;
    logic [PREG_WIDTH-1:0]    phys_rd;    // Scalar physical dest (for vreductions)
    logic [TID_WIDTH-1:0]     tid;
    logic                     valid;
  } vec_disp_t;

  vec_disp_t disp_q [DISP_DEPTH-1:0];
  logic [$clog2(DISP_DEPTH)-1:0] dq_head, dq_tail;
  assign rx_ready = (dq_tail + 1'b1) != dq_head;

  // --------------------------------------------------------------------------
  // SIMD Function Codes (stored in instr[14:12] = fn3, instr[31:25] = fn7)
  // --------------------------------------------------------------------------
  localparam logic [2:0] VEC_ADD   = 3'b000;
  localparam logic [2:0] VEC_SUB   = 3'b001;
  localparam logic [2:0] VEC_MUL   = 3'b010;
  localparam logic [2:0] VEC_FMA   = 3'b011; // dst[i] += src1[i] * src2[i]
  localparam logic [2:0] VEC_DOT   = 3'b100; // horizontal dot product
  localparam logic [2:0] VEC_SHIFT = 3'b101;
  localparam logic [2:0] VEC_AND   = 3'b110;
  localparam logic [2:0] VEC_OR    = 3'b111;
  // fn7 selects element width: 00=int8, 01=int16, 10=int32, 11=fp32

  // --------------------------------------------------------------------------
  // Execution Pipeline: 4-stage SIMD pipelines
  // --------------------------------------------------------------------------
  typedef struct packed {
    logic [VREG_WIDTH-1:0]   vs1, vs2, vs3;
    logic [2:0]               fn3;
    logic [1:0]               ewidth;
    logic [5:0]               vrd;
    logic [ROB_PTR_WIDTH-1:0] rob_idx;
    logic [PREG_WIDTH-1:0]    phys_rd;
    logic [TID_WIDTH-1:0]     tid;
    logic                     valid;
  } vec_pipe_s;

  vec_pipe_s pvec [4:0]; // 5-stage pipeline registers

  // --------------------------------------------------------------------------
  // Integer element-wise operations (parameterized by element width)
  // --------------------------------------------------------------------------
  function automatic logic [VREG_WIDTH-1:0] vec_add_fw(
    input logic [VREG_WIDTH-1:0] a, b,
    input logic [1:0] ewidth
  );
    automatic logic [VREG_WIDTH-1:0] res = '0;
    unique case (ewidth)
      2'b00: for (int i = 0; i < VREG_WIDTH/8;  i++) res[i*8  +: 8]  = a[i*8  +: 8]  + b[i*8  +: 8];
      2'b01: for (int i = 0; i < VREG_WIDTH/16; i++) res[i*16 +: 16] = a[i*16 +: 16] + b[i*16 +: 16];
      2'b10: for (int i = 0; i < VREG_WIDTH/32; i++) res[i*32 +: 32] = a[i*32 +: 32] + b[i*32 +: 32];
      2'b11: for (int i = 0; i < VREG_WIDTH/64; i++) res[i*64 +: 64] = a[i*64 +: 64] + b[i*64 +: 64];
    endcase
    return res;
  endfunction

  function automatic logic [VREG_WIDTH-1:0] vec_mul_fw(
    input logic [VREG_WIDTH-1:0] a, b,
    input logic [1:0] ewidth
  );
    automatic logic [VREG_WIDTH-1:0] res = '0;
    unique case (ewidth)
      2'b00: for (int i = 0; i < VREG_WIDTH/8;  i++) res[i*8  +: 8]  = a[i*8  +: 8]  * b[i*8  +: 8];
      2'b01: for (int i = 0; i < VREG_WIDTH/16; i++) res[i*16 +: 16] = a[i*16 +: 16] * b[i*16 +: 16];
      2'b10: for (int i = 0; i < VREG_WIDTH/32; i++) res[i*32 +: 32] = a[i*32 +: 32] * b[i*32 +: 32];
      2'b11: for (int i = 0; i < VREG_WIDTH/64; i++) res[i*64 +: 64] = a[i*64 +: 64] * b[i*64 +: 64];
    endcase
    return res;
  endfunction

  function automatic logic [XLEN-1:0] vec_dot_fp32(
    input logic [VREG_WIDTH-1:0] a, b
  );
    // Horizontal dot product of 16× float32 elements (simplified integer approx)
    automatic logic [63:0] acc = 64'b0;
    for (int i = 0; i < 16; i++) begin
      automatic logic [31:0] ea = a[i*32 +: 32];
      automatic logic [31:0] eb = b[i*32 +: 32];
      acc += 64'(ea) * 64'(eb); // Integer proxy; real: IEEE754 mul-add
    end
    return acc;
  endfunction

  // --------------------------------------------------------------------------
  // Pipeline registers
  // --------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i <= 4; i++) pvec[i] <= '0;
    end else begin
      // Stage 0 → 1: Fetch from dispatch queue + VRF read
      pvec[0].valid   <= (dq_head != dq_tail);
      pvec[0].fn3     <= disp_q[dq_head].instr[14:12];
      pvec[0].ewidth  <= disp_q[dq_head].instr[26:25];
      pvec[0].vs1     <= vrf[disp_q[dq_head].vrs1];
      pvec[0].vs2     <= vrf[disp_q[dq_head].vrs2];
      pvec[0].vs3     <= '0; // FMA third source (TBD)
      pvec[0].vrd     <= disp_q[dq_head].vrd;
      pvec[0].rob_idx <= disp_q[dq_head].rob_idx;
      pvec[0].phys_rd <= disp_q[dq_head].phys_rd;
      pvec[0].tid     <= disp_q[dq_head].tid;

      // Stage 1 → 2: Execute first half of SIMD lanes
      pvec[1]         <= pvec[0];
      if (pvec[0].valid) begin
        unique case (pvec[0].fn3)
          VEC_ADD:   pvec[1].vs1 <= vec_add_fw(pvec[0].vs1, pvec[0].vs2, pvec[0].ewidth);
          VEC_SUB:   pvec[1].vs1 <= vec_add_fw(pvec[0].vs1, ~pvec[0].vs2 + 1'b1, pvec[0].ewidth);
          VEC_MUL:   pvec[1].vs1 <= vec_mul_fw(pvec[0].vs1, pvec[0].vs2, pvec[0].ewidth);
          VEC_FMA:   pvec[1].vs1 <= vec_add_fw(pvec[0].vs3, vec_mul_fw(pvec[0].vs1, pvec[0].vs2, pvec[0].ewidth), pvec[0].ewidth);
          VEC_AND:   pvec[1].vs1 <= pvec[0].vs1 & pvec[0].vs2;
          VEC_OR:    pvec[1].vs1 <= pvec[0].vs1 | pvec[0].vs2;
          VEC_SHIFT: pvec[1].vs1 <= pvec[0].vs1 << pvec[0].vs2[5:0];
          default:   pvec[1].vs1 <= pvec[0].vs1;
        endcase
      end

      // Stage 2 → 3: Complete calculation + DOT reduction
      pvec[2] <= pvec[1];
      if (pvec[1].valid && pvec[1].fn3 == VEC_DOT) begin
        // Horizontal reduce
        pvec[2].vs1[63:0] <= vec_dot_fp32(pvec[1].vs1, pvec[1].vs2);
      end

      // Stage 3 → 4: Write-back stage
      pvec[3] <= pvec[2];

      // Stage 4: Output
      pvec[4] <= pvec[3];
    end
  end

  // VRF writeback
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i < VREG_COUNT; i++) vrf[i] <= '0;
    end else if (pvec[3].valid) begin
      vrf[pvec[3].vrd] <= pvec[3].vs1;
    end
  end

  // Dispatch queue management
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      dq_head <= '0; dq_tail <= '0;
    end else begin
      if (rx_valid && rx_ready && rx_pkt.pkt_type == NOC_DISPATCH) begin
        disp_q[dq_tail].instr      <= rx_pkt.data[31:0];
        disp_q[dq_tail].src_scalar <= rx_pkt.data[511:448];
        disp_q[dq_tail].vrs1       <= rx_pkt.data[37:32];
        disp_q[dq_tail].vrs2       <= rx_pkt.data[43:38];
        disp_q[dq_tail].vrd        <= rx_pkt.data[49:44];
        disp_q[dq_tail].rob_idx    <= rx_pkt.rob_idx;
        disp_q[dq_tail].phys_rd    <= rx_pkt.data[PREG_WIDTH+39:40];
        disp_q[dq_tail].tid        <= rx_pkt.tid;
        disp_q[dq_tail].valid      <= 1'b1;
        dq_tail <= dq_tail + 1'b1;
      end
      if (dq_head != dq_tail) dq_head <= dq_head + 1'b1;
    end
  end

  // Output result
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tx_valid     <= 1'b0;
      bypass_valid <= 1'b0;
    end else begin
      tx_valid     <= pvec[4].valid;
      bypass_valid <= pvec[4].valid;

      if (pvec[4].valid) begin
        tx_pkt.pkt_type      <= NOC_RESULT;
        tx_pkt.src_node      <= CHIPLET_NODE_ID;
        tx_pkt.dst_node      <= 5'h00;
        tx_pkt.rob_idx       <= pvec[4].rob_idx;
        tx_pkt.tid           <= pvec[4].tid;
        tx_pkt.data[VREG_WIDTH-1:0] <= pvec[4].vs1; // Vector result to memory
        tx_pkt.data[PREG_WIDTH-1+VREG_WIDTH:VREG_WIDTH] <= pvec[4].phys_rd;
        tx_pkt.valid         <= 1'b1;

        bypass_result.phys_rd <= pvec[4].phys_rd;
        bypass_result.result  <= pvec[4].vs1[63:0]; // Scalar extract
        bypass_result.rob_idx <= pvec[4].rob_idx;
        bypass_result.tid     <= pvec[4].tid;
        bypass_result.valid   <= 1'b1;
      end
    end
  end

endmodule : simd_chiplet_top
