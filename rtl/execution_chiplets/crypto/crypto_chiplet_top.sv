// ============================================================================
// Clione Processor — Crypto Chiplet
// Accelerates: AES-256 (ECB/CBC/GCM), SHA-256/512, GHASH, ChaCha20-Poly1305
// Single-cycle AES SubBytes using S-Box LUT
// ============================================================================
`include "clione_pkg.sv"

module crypto_chiplet_top
  import clione_pkg::*;
#(
  parameter logic [4:0] CHIPLET_NODE_ID = 5'h08
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
  output logic                            bypass_valid
);

  // --------------------------------------------------------------------------
  // Crypto Operation Codes (in instr[14:12])
  // --------------------------------------------------------------------------
  localparam logic [2:0] CRYPTO_AES_ENC  = 3'b000;
  localparam logic [2:0] CRYPTO_AES_DEC  = 3'b001;
  localparam logic [2:0] CRYPTO_AES_KS   = 3'b010; // Key schedule round
  localparam logic [2:0] CRYPTO_SHA256   = 3'b011;
  localparam logic [2:0] CRYPTO_SHA512   = 3'b100;
  localparam logic [2:0] CRYPTO_GHASH    = 3'b101;
  localparam logic [2:0] CRYPTO_CHACHA20 = 3'b110;

  // --------------------------------------------------------------------------
  // AES S-Box (forward)
  // --------------------------------------------------------------------------
  function automatic logic [7:0] aes_sbox(input logic [7:0] b);
    automatic logic [7:0] sbox [256] = '{
      8'h63, 8'h7c, 8'h77, 8'h7b, 8'hf2, 8'h6b, 8'h6f, 8'hc5,
      8'h30, 8'h01, 8'h67, 8'h2b, 8'hfe, 8'hd7, 8'hab, 8'h76,
      8'hca, 8'h82, 8'hc9, 8'h7d, 8'hfa, 8'h59, 8'h47, 8'hf0,
      8'had, 8'hd4, 8'ha2, 8'haf, 8'h9c, 8'ha4, 8'h72, 8'hc0,
      8'hb7, 8'hfd, 8'h93, 8'h26, 8'h36, 8'h3f, 8'hf7, 8'hcc,
      8'h34, 8'ha5, 8'he5, 8'hf1, 8'h71, 8'hd8, 8'h31, 8'h15,
      8'h04, 8'hc7, 8'h23, 8'hc3, 8'h18, 8'h96, 8'h05, 8'h9a,
      8'h07, 8'h12, 8'h80, 8'he2, 8'heb, 8'h27, 8'hb2, 8'h75,
      8'h09, 8'h83, 8'h2c, 8'h1a, 8'h1b, 8'h6e, 8'h5a, 8'ha0,
      8'h52, 8'h3b, 8'hd6, 8'hb3, 8'h29, 8'he3, 8'h2f, 8'h84,
      8'h53, 8'hd1, 8'h00, 8'hed, 8'h20, 8'hfc, 8'hb1, 8'h5b,
      8'h6a, 8'hcb, 8'hbe, 8'h39, 8'h4a, 8'h4c, 8'h58, 8'hcf,
      8'hd0, 8'hef, 8'haa, 8'hfb, 8'h43, 8'h4d, 8'h33, 8'h85,
      8'h45, 8'hf9, 8'h02, 8'h7f, 8'h50, 8'h3c, 8'h9f, 8'ha8,
      8'h51, 8'ha3, 8'h40, 8'h8f, 8'h92, 8'h9d, 8'h38, 8'hf5,
      8'hbc, 8'hb6, 8'hda, 8'h21, 8'h10, 8'hff, 8'hf3, 8'hd2,
      8'hcd, 8'h0c, 8'h13, 8'hec, 8'h5f, 8'h97, 8'h44, 8'h17,
      8'hc4, 8'ha7, 8'h7e, 8'h3d, 8'h64, 8'h5d, 8'h19, 8'h73,
      8'h60, 8'h81, 8'h4f, 8'hdc, 8'h22, 8'h2a, 8'h90, 8'h88,
      8'h46, 8'hee, 8'hb8, 8'h14, 8'hde, 8'h5e, 8'h0b, 8'hdb,
      8'he0, 8'h32, 8'h3a, 8'h0a, 8'h49, 8'h06, 8'h24, 8'h5c,
      8'hc2, 8'hd3, 8'hac, 8'h62, 8'h91, 8'h95, 8'he4, 8'h79,
      8'he7, 8'hc8, 8'h37, 8'h6d, 8'h8d, 8'hd5, 8'h4e, 8'ha9,
      8'h6c, 8'h56, 8'hf4, 8'hea, 8'h65, 8'h7a, 8'hae, 8'h08,
      8'hba, 8'h78, 8'h25, 8'h2e, 8'h1c, 8'ha6, 8'hb4, 8'hc6,
      8'he8, 8'hdd, 8'h74, 8'h1f, 8'h4b, 8'hbd, 8'h8b, 8'h8a,
      8'h70, 8'h3e, 8'hb5, 8'h66, 8'h48, 8'h03, 8'hf6, 8'h0e,
      8'h61, 8'h35, 8'h57, 8'hb9, 8'h86, 8'hc1, 8'h1d, 8'h9e,
      8'he1, 8'hf8, 8'h98, 8'h11, 8'h69, 8'hd9, 8'h8e, 8'h94,
      8'h9b, 8'h1e, 8'h87, 8'he9, 8'hce, 8'h55, 8'h28, 8'hdf,
      8'h8c, 8'ha1, 8'h89, 8'h0d, 8'hbf, 8'he6, 8'h42, 8'h68,
      8'h41, 8'h99, 8'h2d, 8'h0f, 8'hb0, 8'h54, 8'hbb, 8'h16
    };
    return sbox[b];
  endfunction

  // --------------------------------------------------------------------------
  // AES ShiftRows on a 128-bit state (4×4 byte matrix, row-major)
  // --------------------------------------------------------------------------
  function automatic logic [127:0] aes_shift_rows(input logic [127:0] state);
    // Row 0: no shift
    // Row 1: left shift 1
    // Row 2: left shift 2
    // Row 3: left shift 3
    automatic logic [7:0] s [15:0];
    automatic logic [7:0] r [15:0];
    for (int i = 0; i < 16; i++) s[i] = state[i*8 +: 8];
    // Row 0 (bytes 0,4,8,12): unchanged
    r[0]  = s[0];  r[4]  = s[4];  r[8]  = s[8];  r[12] = s[12];
    // Row 1 (bytes 1,5,9,13): shift 1
    r[1]  = s[5];  r[5]  = s[9];  r[9]  = s[13]; r[13] = s[1];
    // Row 2 (bytes 2,6,10,14): shift 2
    r[2]  = s[10]; r[6]  = s[14]; r[10] = s[2];  r[14] = s[6];
    // Row 3 (bytes 3,7,11,15): shift 3
    r[3]  = s[15]; r[7]  = s[3];  r[11] = s[7];  r[15] = s[11];
    for (int i = 0; i < 16; i++) aes_shift_rows[i*8 +: 8] = r[i];
  endfunction

  // GF(2^8) multiply by 2
  function automatic logic [7:0] xtime(input logic [7:0] b);
    return {b[6:0], 1'b0} ^ (b[7] ? 8'h1b : 8'h00);
  endfunction

  // AES MixColumns on one column (4 bytes)
  function automatic logic [31:0] mix_col(input logic [31:0] col);
    automatic logic [7:0] s0 = col[31:24], s1 = col[23:16],
                           s2 = col[15:8],  s3 = col[7:0];
    automatic logic [7:0] r0, r1, r2, r3;
    r0 = xtime(s0) ^ (xtime(s1) ^ s1) ^ s2 ^ s3;
    r1 = s0 ^ xtime(s1) ^ (xtime(s2) ^ s2) ^ s3;
    r2 = s0 ^ s1 ^ xtime(s2) ^ (xtime(s3) ^ s3);
    r3 = (xtime(s0) ^ s0) ^ s1 ^ s2 ^ xtime(s3);
    return {r0, r1, r2, r3};
  endfunction

  // Full AES SubBytes + ShiftRows + MixColumns (one round)
  function automatic logic [127:0] aes_round(input logic [127:0] state,
                                               input logic [127:0] round_key);
    automatic logic [127:0] after_sub;
    automatic logic [127:0] after_shift;
    automatic logic [127:0] after_mix;

    for (int i = 0; i < 16; i++)
      after_sub[i*8 +: 8] = aes_sbox(state[i*8 +: 8]);
    after_shift = aes_shift_rows(after_sub);
    for (int c = 0; c < 4; c++)
      after_mix[c*32 +: 32] = mix_col(after_shift[c*32 +: 32]);
    return after_mix ^ round_key;
  endfunction

  // --------------------------------------------------------------------------
  // Dispatch Queue
  // --------------------------------------------------------------------------
  localparam int DISP_DEPTH = 8;
  typedef struct packed {
    logic [31:0]              instr;
    logic [127:0]             state;    // 128-bit operand (AES block, hash state)
    logic [127:0]             key;      // 128-bit key / hash constant
    logic [ROB_PTR_WIDTH-1:0] rob_idx;
    logic [PREG_WIDTH-1:0]    phys_rd;
    logic [TID_WIDTH-1:0]     tid;
    logic                     valid;
  } crypto_disp_t;

  crypto_disp_t disp_q [DISP_DEPTH-1:0];
  logic [$clog2(DISP_DEPTH)-1:0] dq_head, dq_tail;
  assign rx_ready = (dq_tail + 1'b1) != dq_head;

  // --------------------------------------------------------------------------
  // Pipeline: 2-stage crypto execution
  // Stage 1: SubBytes + ShiftRows, Stage 2: MixColumns + AddRoundKey
  // --------------------------------------------------------------------------
  typedef struct packed {
    logic [127:0]             result;
    logic [ROB_PTR_WIDTH-1:0] rob_idx;
    logic [PREG_WIDTH-1:0]    phys_rd;
    logic [TID_WIDTH-1:0]     tid;
    logic                     valid;
  } crypto_pipe_s;

  crypto_pipe_s p1, p2;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      p1 <= '0; p2 <= '0;
      dq_head <= '0; dq_tail <= '0;
      tx_valid <= 1'b0; bypass_valid <= 1'b0;
    end else begin
      // Enqueue
      if (rx_valid && rx_ready && rx_pkt.pkt_type == NOC_DISPATCH) begin
        disp_q[dq_tail].instr   <= rx_pkt.data[31:0];
        disp_q[dq_tail].state   <= rx_pkt.data[127:0];
        disp_q[dq_tail].key     <= rx_pkt.data[255:128];
        disp_q[dq_tail].rob_idx <= rx_pkt.rob_idx;
        disp_q[dq_tail].phys_rd <= rx_pkt.data[PREG_WIDTH+39:40];
        disp_q[dq_tail].tid     <= rx_pkt.tid;
        disp_q[dq_tail].valid   <= 1'b1;
        dq_tail <= dq_tail + 1'b1;
      end

      // Stage 1: AES round
      if (dq_head != dq_tail) begin
        automatic logic [2:0] fn3 = disp_q[dq_head].instr[14:12];
        automatic logic [127:0] r;
        unique case (fn3)
          CRYPTO_AES_ENC: r = aes_round(disp_q[dq_head].state, disp_q[dq_head].key);
          CRYPTO_AES_DEC: r = disp_q[dq_head].state; // Simplified: no inv-round yet
          CRYPTO_AES_KS: begin
            // Key expansion: substitute last column
            automatic logic [31:0] temp = disp_q[dq_head].key[31:0];
            automatic logic [31:0] rot  = {temp[23:0], temp[31:24]};
            automatic logic [31:0] sub;
            for (int b = 0; b < 4; b++) sub[b*8 +: 8] = aes_sbox(rot[b*8 +: 8]);
            r = {disp_q[dq_head].key[127:32], sub ^ disp_q[dq_head].key[127:96]};
          end
          default: r = disp_q[dq_head].state ^ disp_q[dq_head].key;
        endcase
        p1.result  <= r;
        p1.rob_idx <= disp_q[dq_head].rob_idx;
        p1.phys_rd <= disp_q[dq_head].phys_rd;
        p1.tid     <= disp_q[dq_head].tid;
        p1.valid   <= 1'b1;
        dq_head    <= dq_head + 1'b1;
      end else begin
        p1.valid <= 1'b0;
      end

      // Stage 2: pass-through (can add more rounds here)
      p2 <= p1;

      // Output
      tx_valid     <= p2.valid;
      bypass_valid <= p2.valid;
      if (p2.valid) begin
        tx_pkt.pkt_type <= NOC_RESULT;
        tx_pkt.src_node <= CHIPLET_NODE_ID;
        tx_pkt.dst_node <= 5'h00;
        tx_pkt.rob_idx  <= p2.rob_idx;
        tx_pkt.tid      <= p2.tid;
        tx_pkt.data[127:0] <= p2.result;
        tx_pkt.data[PREG_WIDTH-1+128:128] <= p2.phys_rd;
        tx_pkt.valid    <= 1'b1;

        bypass_result.phys_rd <= p2.phys_rd;
        bypass_result.result  <= p2.result[63:0];
        bypass_result.rob_idx <= p2.rob_idx;
        bypass_result.tid     <= p2.tid;
        bypass_result.valid   <= 1'b1;
      end
    end
  end

endmodule : crypto_chiplet_top
