// ============================================================================
// Clione Processor — L1 Data Cache (L1D)
// 64KB, 8-way set-associative, 64B lines, write-back/write-allocate
// MESI coherency, load-store queue integration
// ============================================================================
`include "clione_pkg.sv"

module l1d_cache
  import clione_pkg::*;
(
  input  logic                          clk,
  input  logic                          rst_n,

  // Load port (from LSU)
  input  logic [PADDR_WIDTH-1:0]        ld_paddr,
  input  logic [1:0]                    ld_size,      // 00=1B,01=2B,10=4B,11=8B
  input  logic                          ld_signed,
  input  logic [ROB_PTR_WIDTH-1:0]      ld_rob_idx,
  input  logic [TID_WIDTH-1:0]          ld_tid,
  input  logic                          ld_valid,
  output logic                          ld_ready,
  output logic [XLEN-1:0]              ld_data,
  output logic                          ld_resp_valid,
  output logic [ROB_PTR_WIDTH-1:0]      ld_resp_rob_idx,

  // Store port (from LSU, committed stores only)
  input  logic [PADDR_WIDTH-1:0]        st_paddr,
  input  logic [XLEN-1:0]              st_data,
  input  logic [7:0]                    st_be,        // byte enable
  input  logic                          st_valid,
  output logic                          st_ready,

  // L2 fill/writeback interface
  input  logic [CACHE_LINE_BITS-1:0]    fill_data,
  input  logic [PADDR_WIDTH-1:0]        fill_paddr,
  input  logic                          fill_valid,
  output logic [CACHE_LINE_BITS-1:0]    wb_data,
  output logic [PADDR_WIDTH-1:0]        wb_paddr,
  output logic                          wb_valid,
  input  logic                          wb_ready,

  // Coherency snoops
  input  logic [PADDR_WIDTH-1:0]        snoop_paddr,
  input  logic                          snoop_inv,    // invalidate
  input  logic                          snoop_valid,
  output logic [CACHE_LINE_BITS-1:0]    snoop_data,   // for M→S downgrade
  output logic                          snoop_hit
);

  // --------------------------------------------------------------------------
  // Cache Parameters
  // --------------------------------------------------------------------------
  localparam int WAYS          = L1D_WAYS;
  localparam int SIZE_BYTES    = L1D_SIZE_KB * 1024;
  localparam int LINE_BYTES    = CACHE_LINE_BYTES;
  localparam int NUM_SETS      = SIZE_BYTES / (LINE_BYTES * WAYS); // 128
  localparam int SET_BITS      = $clog2(NUM_SETS);   // 7
  localparam int LINE_OFF_BITS = $clog2(LINE_BYTES); // 6
  localparam int TAG_BITS      = PADDR_WIDTH - SET_BITS - LINE_OFF_BITS;

  // --------------------------------------------------------------------------
  // Storage Arrays
  // --------------------------------------------------------------------------
  logic [TAG_BITS-1:0]          tag_array  [WAYS-1:0][NUM_SETS-1:0];
  logic                          valid_array[WAYS-1:0][NUM_SETS-1:0];
  logic [CACHE_LINE_BITS-1:0]   data_array [WAYS-1:0][NUM_SETS-1:0];
  mesi_state_e                   mesi_array [WAYS-1:0][NUM_SETS-1:0];
  logic                          dirty_array[WAYS-1:0][NUM_SETS-1:0];

  // 3-bit PLRU tree for 8-way
  logic [6:0]                    plru       [NUM_SETS-1:0];

  // --------------------------------------------------------------------------
  // Load Path
  // --------------------------------------------------------------------------
  logic [TAG_BITS-1:0]         ld_tag;
  logic [SET_BITS-1:0]         ld_set;
  logic [LINE_OFF_BITS-1:0]    ld_offset;
  logic [WAYS-1:0]             ld_hit_way_oh;
  logic                         ld_cache_hit;
  logic [CACHE_LINE_BITS-1:0]  ld_hit_line;
  logic [XLEN-1:0]             ld_hit_data_raw;

  assign ld_tag    = ld_paddr[PADDR_WIDTH-1 : SET_BITS+LINE_OFF_BITS];
  assign ld_set    = ld_paddr[SET_BITS+LINE_OFF_BITS-1 : LINE_OFF_BITS];
  assign ld_offset = ld_paddr[LINE_OFF_BITS-1:0];

  always_comb begin
    ld_hit_way_oh = '0;
    ld_hit_line   = '0;
    for (int w = 0; w < WAYS; w++) begin
      if (valid_array[w][ld_set] && tag_array[w][ld_set] == ld_tag &&
          mesi_array[w][ld_set] != MESI_I) begin
        ld_hit_way_oh[w] = 1'b1;
        ld_hit_line      = data_array[w][ld_set];
      end
    end
    ld_cache_hit = |ld_hit_way_oh;

    // Extract data by size from cache line
    ld_hit_data_raw = '0;
    unique case (ld_size)
      2'b00: ld_hit_data_raw = XLEN'(ld_hit_line[ld_offset*8 +: 8]);
      2'b01: ld_hit_data_raw = XLEN'(ld_hit_line[ld_offset*8 +: 16]);
      2'b10: ld_hit_data_raw = XLEN'(ld_hit_line[ld_offset*8 +: 32]);
      2'b11: ld_hit_data_raw = ld_hit_line[ld_offset*8 +: 64];
    endcase

    // Sign extend if needed
    ld_data = ld_signed ? $signed(ld_hit_data_raw) : ld_hit_data_raw;
  end

  assign ld_ready         = 1'b1; // Stall on miss handled via resp_valid delay
  assign ld_resp_valid    = ld_valid && ld_cache_hit;
  assign ld_resp_rob_idx  = ld_rob_idx;

  // --------------------------------------------------------------------------
  // Store Path: write to cache line on hit; writeback if dirty eviction
  // --------------------------------------------------------------------------
  logic [TAG_BITS-1:0]         st_tag;
  logic [SET_BITS-1:0]         st_set;
  logic [WAYS-1:0]             st_hit_way_oh;

  assign st_tag = st_paddr[PADDR_WIDTH-1 : SET_BITS+LINE_OFF_BITS];
  assign st_set = st_paddr[SET_BITS+LINE_OFF_BITS-1 : LINE_OFF_BITS];

  always_comb begin
    st_hit_way_oh = '0;
    for (int w = 0; w < WAYS; w++) begin
      if (valid_array[w][st_set] && tag_array[w][st_set] == st_tag &&
          mesi_array[w][st_set] != MESI_I)
        st_hit_way_oh[w] = 1'b1;
    end
  end

  assign st_ready = 1'b1;

  // --------------------------------------------------------------------------
  // Cache Update (fill, store, MESI transitions)
  // --------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int w = 0; w < WAYS; w++)
        for (int s = 0; s < NUM_SETS; s++) begin
          valid_array[w][s] = 1'b0;
          dirty_array[w][s] = 1'b0;
          mesi_array[w][s]  = MESI_I;
        end
      wb_valid <= 1'b0;
    end else begin
      wb_valid <= 1'b0;

      // === Store hit: update byte lanes ===
      if (st_valid && |st_hit_way_oh) begin
        for (int w = 0; w < WAYS; w++) begin
          if (st_hit_way_oh[w]) begin
            automatic logic [LINE_OFF_BITS-1:0] st_off;
            st_off = st_paddr[LINE_OFF_BITS-1:0];
            for (int b = 0; b < 8; b++) begin
              if (st_be[b])
                data_array[w][st_set][(st_off + b)*8 +: 8] <= st_data[b*8 +: 8];
            end
            dirty_array[w][st_set] <= 1'b1;
            mesi_array[w][st_set]  <= MESI_M;
          end
        end
      end

      // === Fill from L2 ===
      if (fill_valid) begin
        automatic logic [SET_BITS-1:0]  f_set = fill_paddr[SET_BITS+LINE_OFF_BITS-1:LINE_OFF_BITS];
        automatic logic [TAG_BITS-1:0]  f_tag = fill_paddr[PADDR_WIDTH-1:SET_BITS+LINE_OFF_BITS];
        // Victim: use PLRU — simplified to way 0 here
        automatic logic [2:0] victim = 3'(plru[f_set][2:0] ^ 3'b111);
        // Writeback dirty victim
        if (valid_array[victim][f_set] && dirty_array[victim][f_set]) begin
          wb_data  <= data_array[victim][f_set];
          wb_paddr <= {tag_array[victim][f_set], f_set, {LINE_OFF_BITS{1'b0}}};
          wb_valid <= 1'b1;
        end
        data_array[victim][f_set]  <= fill_data;
        tag_array[victim][f_set]   <= f_tag;
        valid_array[victim][f_set] <= 1'b1;
        dirty_array[victim][f_set] <= 1'b0;
        mesi_array[victim][f_set]  <= MESI_E;
      end

      // === Snoop handling ===
      if (snoop_valid) begin
        automatic logic [SET_BITS-1:0] sn_set = snoop_paddr[SET_BITS+LINE_OFF_BITS-1:LINE_OFF_BITS];
        automatic logic [TAG_BITS-1:0] sn_tag = snoop_paddr[PADDR_WIDTH-1:SET_BITS+LINE_OFF_BITS];
        for (int w = 0; w < WAYS; w++) begin
          if (valid_array[w][sn_set] && tag_array[w][sn_set] == sn_tag) begin
            if (snoop_inv) begin
              if (dirty_array[w][sn_set]) begin
                // Must writeback before invalidating
                wb_data  <= data_array[w][sn_set];
                wb_paddr <= snoop_paddr;
                wb_valid <= 1'b1;
              end
              mesi_array[w][sn_set]  <= MESI_I;
              valid_array[w][sn_set] <= 1'b0;
              dirty_array[w][sn_set] <= 1'b0;
            end else begin
              // Downgrade M→S
              mesi_array[w][sn_set] <= MESI_S;
            end
          end
        end
      end
    end
  end

  // Snoop data (for M→S or GETS)
  always_comb begin
    snoop_hit  = 1'b0;
    snoop_data = '0;
    if (snoop_valid) begin
      automatic logic [SET_BITS-1:0] sn_set = snoop_paddr[SET_BITS+LINE_OFF_BITS-1:LINE_OFF_BITS];
      automatic logic [TAG_BITS-1:0] sn_tag = snoop_paddr[PADDR_WIDTH-1:SET_BITS+LINE_OFF_BITS];
      for (int w = 0; w < WAYS; w++) begin
        if (valid_array[w][sn_set] && tag_array[w][sn_set] == sn_tag) begin
          snoop_hit  = 1'b1;
          snoop_data = data_array[w][sn_set];
        end
      end
    end
  end

endmodule : l1d_cache
