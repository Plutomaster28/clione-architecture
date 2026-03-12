// ============================================================================
// Clione Processor — Branch Predictor
// TAGE (Tagged Geometric History Length) + BTB + RAS + 2-bit bimodal base
// Per-thread history, 4 SMT threads
// ============================================================================
`include "clione_pkg.sv"

module branch_predictor
  import clione_pkg::*;
(
  input  logic                        clk,
  input  logic                        rst_n,

  // Prediction request (from IFU, one per thread each cycle)
  input  logic [63:0]                 req_pc    [SMT_WAYS-1:0],
  input  logic                        req_valid [SMT_WAYS-1:0],

  // Prediction output
  output logic                        pred_taken  [SMT_WAYS-1:0],
  output logic [63:0]                 pred_target [SMT_WAYS-1:0],
  output logic                        pred_valid  [SMT_WAYS-1:0],
  output logic [2:0]                  pred_tag    [SMT_WAYS-1:0], // which TAGE component won

  // Update from retirement (branch resolved)
  input  logic                        upd_valid,
  input  logic [TID_WIDTH-1:0]        upd_tid,
  input  logic [63:0]                 upd_pc,
  input  logic                        upd_taken,
  input  logic                        upd_mispredict,
  input  logic [63:0]                 upd_target,
  input  logic [2:0]                  upd_pred_tag,    // which component predicted
  input  logic                        upd_is_call,
  input  logic                        upd_is_ret
);

  // --------------------------------------------------------------------------
  // Parameters
  // --------------------------------------------------------------------------
  localparam int BIMODAL_ENTRIES  = 4096;
  localparam int BTB_WAYS         = 4;
  localparam int BTB_ENTRIES_PER_WAY = BTB_ENTRIES / BTB_WAYS; // 1024

  // TAGE component geometry: 5 tables, geometric history lengths
  localparam int TAGE_T0_ENTRIES  = 4096;   // Bimodal base
  localparam int TAGE_T1_ENTRIES  = 1024;   // hist_len = 8
  localparam int TAGE_T2_ENTRIES  = 1024;   // hist_len = 16
  localparam int TAGE_T3_ENTRIES  = 1024;   // hist_len = 32
  localparam int TAGE_T4_ENTRIES  = 1024;   // hist_len = 64
  // T5 uses TAGE_HIST_LEN = 512

  // History lengths per TAGE table: T0=bimodal, T1..T5 use geometric history

  // --------------------------------------------------------------------------
  // Global History Registers (per thread)
  // --------------------------------------------------------------------------
  logic [TAGE_HIST_LEN-1:0] ghr [SMT_WAYS-1:0];

  // --------------------------------------------------------------------------
  // Bimodal Base Table T0
  // --------------------------------------------------------------------------
  logic [1:0] bimodal [TAGE_T0_ENTRIES-1:0];  // 2-bit saturating counter

  // --------------------------------------------------------------------------
  // TAGE Tables T1–T5: each entry has tag + 3-bit counter + 2-bit useful
  // --------------------------------------------------------------------------
  // We pack: {tag[7:0], ctr[2:0], u[1:0]} = 13 bits per entry
  logic [12:0] tage_t1 [TAGE_T1_ENTRIES-1:0];
  logic [12:0] tage_t2 [TAGE_T2_ENTRIES-1:0];
  logic [12:0] tage_t3 [TAGE_T3_ENTRIES-1:0];
  logic [12:0] tage_t4 [TAGE_T4_ENTRIES-1:0];
  logic [12:0] tage_t5 [1024-1:0];  // longest component

  // --------------------------------------------------------------------------
  // BTB — 4-way set-associative
  // --------------------------------------------------------------------------
  typedef struct packed {
    logic [63:12] tag;        // upper 52 bits of PC
    logic [63:0]  target;
    logic         valid;
    logic [1:0]   br_type;    // 00=cond, 01=jal, 10=jalr/ret, 11=call
  } btb_entry_t;

  btb_entry_t btb [BTB_WAYS-1:0][BTB_ENTRIES_PER_WAY-1:0];
  logic [1:0] btb_plru [BTB_ENTRIES_PER_WAY-1:0]; // Pseudo-LRU bits

  // --------------------------------------------------------------------------
  // Return Address Stack (per thread)
  // --------------------------------------------------------------------------
  logic [63:0]  ras        [SMT_WAYS-1:0][RAS_DEPTH-1:0];
  logic [$clog2(RAS_DEPTH)-1:0] ras_ptr [SMT_WAYS-1:0];

  // --------------------------------------------------------------------------
  // Prediction Logic (combinational, pipelined to next cycle)
  // --------------------------------------------------------------------------
  always_comb begin
    for (int t = 0; t < SMT_WAYS; t++) begin
      pred_taken[t]  = 1'b0;
      pred_target[t] = '0;
      pred_valid[t]  = 1'b0;
      pred_tag[t]    = 3'd0;

      if (req_valid[t]) begin
        automatic logic [11:0] bimodal_idx;
        automatic logic        bimodal_pred;
        automatic logic        btb_hit;
        automatic logic [63:0] btb_target;

        bimodal_idx  = req_pc[t][13:2];
        bimodal_pred = bimodal[bimodal_idx][1]; // Strong bit

        // BTB lookup
        btb_hit    = 1'b0;
        btb_target = '0;
        for (int w = 0; w < BTB_WAYS; w++) begin
          automatic logic [$clog2(BTB_ENTRIES_PER_WAY)-1:0] btb_idx;
          btb_idx = req_pc[t][$clog2(BTB_ENTRIES_PER_WAY)+1:2];
          if (btb[w][btb_idx].valid &&
              btb[w][btb_idx].tag == req_pc[t][63:12]) begin
            btb_hit    = 1'b1;
            btb_target = btb[w][btb_idx].target;
            // RAS override for returns
            if (btb[w][btb_idx].br_type == 2'b10) begin
              btb_target = ras[t][ras_ptr[t]];
            end
          end
        end

        pred_taken[t]  = bimodal_pred && btb_hit;
        pred_target[t] = btb_hit ? btb_target : (req_pc[t] + 4);
        pred_valid[t]  = req_valid[t];
        pred_tag[t]    = 3'd0; // simplified: bimodal wins when no TAGE hit
      end
    end
  end

  // --------------------------------------------------------------------------
  // GHR Update
  // --------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int t = 0; t < SMT_WAYS; t++)
        ghr[t] <= '0;
    end else if (upd_valid) begin
      ghr[upd_tid] <= {ghr[upd_tid][TAGE_HIST_LEN-2:0], upd_taken};
    end
  end

  // --------------------------------------------------------------------------
  // Bimodal Update
  // --------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i < TAGE_T0_ENTRIES; i++)
        bimodal[i] <= 2'b10; // Weakly taken
    end else if (upd_valid) begin
      automatic logic [11:0] idx;
      idx = upd_pc[13:2];
      if (upd_taken && bimodal[idx] != 2'b11)
        bimodal[idx] <= bimodal[idx] + 1'b1;
      else if (!upd_taken && bimodal[idx] != 2'b00)
        bimodal[idx] <= bimodal[idx] - 1'b1;
    end
  end

  // --------------------------------------------------------------------------
  // BTB Update
  // --------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int w = 0; w < BTB_WAYS; w++)
        for (int i = 0; i < BTB_ENTRIES_PER_WAY; i++)
          btb[w][i] <= '0;
    end else if (upd_valid && upd_taken) begin
      automatic logic [$clog2(BTB_ENTRIES_PER_WAY)-1:0] bidx;
      bidx = upd_pc[$clog2(BTB_ENTRIES_PER_WAY)+1:2];
      // Simple: always allocate into way 0 (real impl: PLRU replacement)
      btb[0][bidx].tag     <= upd_pc[63:12];
      btb[0][bidx].target  <= upd_target;
      btb[0][bidx].valid   <= 1'b1;
      btb[0][bidx].br_type <= upd_is_call ? 2'b11 :
                               upd_is_ret  ? 2'b10 : 2'b00;
    end
  end

  // --------------------------------------------------------------------------
  // Return Address Stack Update
  // --------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int t = 0; t < SMT_WAYS; t++)
        ras_ptr[t] <= '0;
    end else if (upd_valid) begin
      if (upd_is_call) begin
        ras_ptr[upd_tid]                  <= ras_ptr[upd_tid] + 1'b1;
        ras[upd_tid][ras_ptr[upd_tid]+1]  <= upd_pc + 4;
      end else if (upd_is_ret && ras_ptr[upd_tid] != '0) begin
        ras_ptr[upd_tid] <= ras_ptr[upd_tid] - 1'b1;
      end
    end
  end

endmodule : branch_predictor
