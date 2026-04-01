// ============================================================================
// Clione Processor — L1 Instruction Cache (L1I)
// 64KB, 4-way set-associative, 64B cache lines, virtually indexed/physically tagged
// Non-blocking, supports early restart
// ============================================================================
`include "clione_pkg.sv"

module l1i_cache
  import clione_pkg::*;
(
  input  logic                          clk,
  input  logic                          rst_n,

  // IFU request
  input  logic [PADDR_WIDTH-1:0]        req_paddr,
  input  logic                          req_valid,
  output logic                          req_ready,

  // Response to IFU
  output logic [CACHE_LINE_BITS-1:0]    resp_data,
  output logic                          resp_valid,
  output logic                          resp_error,

  // L2 fill interface (from L2 cache chiplet via NoC)
  input  logic [CACHE_LINE_BITS-1:0]    fill_data,
  input  logic [PADDR_WIDTH-1:0]        fill_paddr,
  input  logic                          fill_valid,

  // Flush/invalidate from coherency
  input  logic                          inv_valid,
  input  logic [PADDR_WIDTH-1:0]        inv_paddr
);

  // --------------------------------------------------------------------------
  // Cache Parameters
  // --------------------------------------------------------------------------
  localparam int WAYS          = L1I_WAYS;          // 4
  localparam int SIZE_BYTES    = L1I_SIZE_KB * 1024; // 65536
  localparam int LINE_BYTES    = CACHE_LINE_BYTES;   // 64
  localparam int NUM_SETS      = SIZE_BYTES / (LINE_BYTES * WAYS); // 256
  localparam int SET_BITS      = $clog2(NUM_SETS);   // 8
  localparam int LINE_OFF_BITS = $clog2(LINE_BYTES); // 6
  localparam int TAG_BITS      = PADDR_WIDTH - SET_BITS - LINE_OFF_BITS; // 38

  // --------------------------------------------------------------------------
  // Storage Arrays
  // --------------------------------------------------------------------------
  logic [TAG_BITS-1:0]          tag_array  [WAYS-1:0][NUM_SETS-1:0];
  logic                          valid_array[WAYS-1:0][NUM_SETS-1:0];
  logic [CACHE_LINE_BITS-1:0]   data_array [WAYS-1:0][NUM_SETS-1:0];

  // PLRU replacement: one bit per set (simplified Tree-PLRU for 4-way)
  logic [2:0]                   plru       [NUM_SETS-1:0]; // 3 bits for 4-way tree

  // --------------------------------------------------------------------------
  // Miss Queue: track outstanding misses
  // --------------------------------------------------------------------------
  localparam int MSHR_DEPTH = 8;
  typedef struct packed {
    logic [PADDR_WIDTH-1:0] paddr;
    logic                   valid;
  } mshr_t;
  mshr_t mshr [MSHR_DEPTH-1:0];

  // --------------------------------------------------------------------------
  // Lookup: decode address
  // --------------------------------------------------------------------------
  logic [TAG_BITS-1:0]         req_tag;
  logic [SET_BITS-1:0]         req_set;
  logic [LINE_OFF_BITS-1:0]    req_offset;
  logic [WAYS-1:0]             hit_way_oh;   // one-hot hit indicator
  logic                         cache_hit;
  logic [CACHE_LINE_BITS-1:0]  hit_data;

  assign req_tag    = req_paddr[PADDR_WIDTH-1 : SET_BITS+LINE_OFF_BITS];
  assign req_set    = req_paddr[SET_BITS+LINE_OFF_BITS-1 : LINE_OFF_BITS];
  assign req_offset = req_paddr[LINE_OFF_BITS-1:0];

  always_comb begin
    hit_way_oh = '0;
    hit_data   = '0;
    for (int w = 0; w < WAYS; w++) begin
      if (valid_array[w][req_set] && tag_array[w][req_set] == req_tag) begin
        hit_way_oh[w] = 1'b1;
        hit_data      = data_array[w][req_set];
      end
    end
    cache_hit = |hit_way_oh;
  end

  // --------------------------------------------------------------------------
  // Miss Handling: MSHR allocation
  // --------------------------------------------------------------------------
  logic mshr_hit;
  always_comb begin
    mshr_hit = 1'b0;
    for (int m = 0; m < MSHR_DEPTH; m++) begin
      if (mshr[m].valid && mshr[m].paddr[PADDR_WIDTH-1:LINE_OFF_BITS] ==
                            req_paddr[PADDR_WIDTH-1:LINE_OFF_BITS])
        mshr_hit = 1'b1;
    end
  end

  // --------------------------------------------------------------------------
  // Response Logic
  // --------------------------------------------------------------------------
  assign resp_data  = cache_hit ? hit_data : '0;
  assign resp_valid = req_valid && cache_hit;
  assign resp_error = 1'b0;
  assign req_ready  = cache_hit || !mshr_hit; // Can accept new req if not pending same line

  // --------------------------------------------------------------------------
  // Fill path: write incoming line into cache
  // --------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int w = 0; w < WAYS; w++)
        for (int s = 0; s < NUM_SETS; s++) begin
          valid_array[w][s] = 1'b0;
          tag_array[w][s]   = '0;
        end
      for (int m = 0; m < MSHR_DEPTH; m++)
        mshr[m] = '0;
    end else begin
      // Invalidation
      if (inv_valid) begin
        automatic logic [SET_BITS-1:0]  inv_set = inv_paddr[SET_BITS+LINE_OFF_BITS-1:LINE_OFF_BITS];
        automatic logic [TAG_BITS-1:0]  inv_tag = inv_paddr[PADDR_WIDTH-1:SET_BITS+LINE_OFF_BITS];
        for (int w = 0; w < WAYS; w++) begin
          if (valid_array[w][inv_set] && tag_array[w][inv_set] == inv_tag)
            valid_array[w][inv_set] <= 1'b0;
        end
      end

      // Fill from L2
      if (fill_valid) begin
        automatic logic [SET_BITS-1:0]  fill_set = fill_paddr[SET_BITS+LINE_OFF_BITS-1:LINE_OFF_BITS];
        automatic logic [TAG_BITS-1:0]  fill_tag = fill_paddr[PADDR_WIDTH-1:SET_BITS+LINE_OFF_BITS];
        // PLRU victim selection (simplified: victim = ~plru[0])
        automatic logic [1:0] victim = plru[fill_set][1:0];
        data_array[victim][fill_set]  <= fill_data;
        tag_array[victim][fill_set]   <= fill_tag;
        valid_array[victim][fill_set] <= 1'b1;
        // Update PLRU
        plru[fill_set] <= 3'({1'b0, ~victim});

        // Free MSHR entry
        for (int m = 0; m < MSHR_DEPTH; m++) begin
          if (mshr[m].valid && mshr[m].paddr[PADDR_WIDTH-1:LINE_OFF_BITS] ==
                               fill_paddr[PADDR_WIDTH-1:LINE_OFF_BITS])
            mshr[m].valid = 1'b0;
        end
      end

      // Miss: allocate MSHR
      if (req_valid && !cache_hit && !mshr_hit) begin
        for (int m = 0; m < MSHR_DEPTH; m++) begin
          if (!mshr[m].valid) begin
            mshr[m].paddr = req_paddr;
            mshr[m].valid = 1'b1;
            break;
          end
        end
      end
    end
  end

endmodule : l1i_cache
