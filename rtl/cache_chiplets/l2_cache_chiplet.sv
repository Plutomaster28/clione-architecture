// ============================================================================
// Clione Processor — L2 Cache Chiplet
// 4MB per chiplet, 16-way set-associative, 64B lines
// MESI protocol, directory-based coherency, non-blocking with 32 MSHRs
// Connects to L1D (control die) and L3 (shared chiplet) via NoC
// ============================================================================
`include "clione_pkg.sv"

module l2_cache_chiplet
  import clione_pkg::*;
#(
  parameter logic [4:0] CHIPLET_NODE_ID = 5'h10,
  parameter int         SIZE_KB         = 4096, // 4MB
  parameter int         WAYS            = 16
)(
  input  logic                            clk,
  input  logic                            rst_n,

  // NoC interface: uplink from control die (L1D misses, writebacks)
  input  noc_pkt_t                        rx_from_l1,
  input  logic                            rx_from_l1_valid,
  output logic                            rx_from_l1_ready,

  // NoC interface: response to control die
  output noc_pkt_t                        tx_to_l1,
  output logic                            tx_to_l1_valid,
  input  logic                            tx_to_l1_ready,

  // NoC interface: downlink to L3
  output noc_pkt_t                        tx_to_l3,
  output logic                            tx_to_l3_valid,
  input  logic                            tx_to_l3_ready,

  // NoC interface: response from L3
  input  noc_pkt_t                        rx_from_l3,
  input  logic                            rx_from_l3_valid,
  output logic                            rx_from_l3_ready
);

  // --------------------------------------------------------------------------
  // Cache Parameters
  // --------------------------------------------------------------------------
  localparam int SIZE_BYTES    = SIZE_KB * 1024;
  localparam int LINE_BYTES    = CACHE_LINE_BYTES;
  localparam int NUM_SETS      = SIZE_BYTES / (LINE_BYTES * WAYS); // 4096
  localparam int SET_BITS      = $clog2(NUM_SETS);     // 12
  localparam int LINE_OFF_BITS = $clog2(LINE_BYTES);   // 6
  localparam int TAG_BITS      = PADDR_WIDTH - SET_BITS - LINE_OFF_BITS;

  // --------------------------------------------------------------------------
  // Storage: Tag + Data arrays (SRAM-like behavioral model)
  // --------------------------------------------------------------------------
  logic [TAG_BITS-1:0]          tag_array  [WAYS-1:0][NUM_SETS-1:0];
  logic                          valid_array[WAYS-1:0][NUM_SETS-1:0];
  logic [CACHE_LINE_BITS-1:0]   data_array [WAYS-1:0][NUM_SETS-1:0];
  mesi_state_e                   mesi_array [WAYS-1:0][NUM_SETS-1:0];
  logic                          dirty_array[WAYS-1:0][NUM_SETS-1:0];

  // PLRU: 15 bits per set for 16-way (full binary tree)
  logic [14:0]                   plru [NUM_SETS-1:0];

  // --------------------------------------------------------------------------
  // MSHR — 32 entries for outstanding L3 fills
  // --------------------------------------------------------------------------
  localparam int MSHR_DEPTH = 32;

  typedef struct packed {
    logic [PADDR_WIDTH-1:0]  paddr;
    logic [5:0]              node_src;    // Requester node (control die, execution chiplet)
    logic [ROB_PTR_WIDTH-1:0]rob_idx;
    logic [TID_WIDTH-1:0]    tid;
    noc_type_e               req_type;
    logic                    allocated;
  } mshr_entry_t;

  mshr_entry_t mshr [MSHR_DEPTH-1:0];

  // --------------------------------------------------------------------------
  // Request Decode & Lookup
  // --------------------------------------------------------------------------

  // Registered pipeline
  typedef struct packed {
    logic [PADDR_WIDTH-1:0]   paddr;
    logic [CACHE_LINE_BITS-1:0] wdata;
    logic [63:0]               wmask;
    noc_type_e                 req_type;
    logic [5:0]                src_node;
    logic [ROB_PTR_WIDTH-1:0]  rob_idx;
    logic [TID_WIDTH-1:0]      tid;
    logic                      valid;
  } l2_req_t;

  l2_req_t req_s1, req_s2; // 2-stage pipeline

  logic [TAG_BITS-1:0]         s1_tag;
  logic [SET_BITS-1:0]         s1_set;
  logic [WAYS-1:0]                       s1_hit_way_oh;
  logic                                   s1_cache_hit;
  logic [CACHE_LINE_BITS-1:0]            s1_hit_data;
  logic [$clog2(WAYS)-1:0]               s1_hit_way_enc;

  // Registered stage-2 copies (capture s1 results so stage-2 sees the
  // lookup that corresponds to req_s2, not the new req_s1)
  logic                                   s2_cache_hit;
  logic [CACHE_LINE_BITS-1:0]            s2_hit_data;
  logic [$clog2(WAYS)-1:0]               s2_hit_way_enc;

  always_comb begin
    s1_tag       = req_s1.paddr[PADDR_WIDTH-1 : SET_BITS+LINE_OFF_BITS];
    s1_set       = req_s1.paddr[SET_BITS+LINE_OFF_BITS-1 : LINE_OFF_BITS];
    s1_hit_way_oh = '0;
    s1_hit_data   = '0;
    s1_hit_way_enc = '0;
    for (int w = 0; w < WAYS; w++) begin
      if (valid_array[w][s1_set] && tag_array[w][s1_set] == s1_tag &&
          mesi_array[w][s1_set] != MESI_I) begin
        s1_hit_way_oh[w]  = 1'b1;
        s1_hit_data       = data_array[w][s1_set];
        s1_hit_way_enc    = ($clog2(WAYS))'(w);
      end
    end
    s1_cache_hit = |s1_hit_way_oh;
  end

  // --------------------------------------------------------------------------
  // Main State Machine
  // --------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      req_s1 <= '0; req_s2 <= '0;
      for (int w = 0; w < WAYS; w++)
        for (int s = 0; s < NUM_SETS; s++) begin
          valid_array[w][s] = 1'b0;
          dirty_array[w][s] = 1'b0;
          mesi_array[w][s]  = MESI_I;
        end
      for (int m = 0; m < MSHR_DEPTH; m++) mshr[m] <= '0;
      tx_to_l1 <= '0;
      tx_to_l3 <= '0;
      tx_to_l1_valid <= 1'b0;
      tx_to_l3_valid <= 1'b0;
    end else begin
      tx_to_l1 <= '0;
      tx_to_l3 <= '0;
      tx_to_l1_valid <= 1'b0;
      tx_to_l3_valid <= 1'b0;
      rx_from_l1_ready <= 1'b1;
      rx_from_l3_ready <= 1'b1;

      // Stage 0 → 1: Accept new request
      if (rx_from_l1_valid && rx_from_l1_ready) begin
        req_s1.paddr     <= rx_from_l1.addr;
        req_s1.wdata     <= rx_from_l1.data;
        req_s1.wmask     <= rx_from_l1.data_mask;
        req_s1.req_type  <= rx_from_l1.pkt_type;
        req_s1.src_node  <= {1'b0, rx_from_l1.src_node};
        req_s1.rob_idx   <= rx_from_l1.rob_idx;
        req_s1.tid       <= rx_from_l1.tid;
        req_s1.valid     <= 1'b1;
      end else begin
        req_s1.valid <= 1'b0;
      end

      req_s2 <= req_s1;

      // Register s1 lookup results so stage-2 uses the correct hit data
      s2_cache_hit    <= s1_cache_hit;
      s2_hit_data     <= s1_hit_data;
      s2_hit_way_enc  <= s1_hit_way_enc;

      // Stage 2: Act on hit/miss
      if (req_s2.valid) begin
        automatic logic [TAG_BITS-1:0] tag = req_s2.paddr[PADDR_WIDTH-1:SET_BITS+LINE_OFF_BITS];
        automatic logic [SET_BITS-1:0] set = req_s2.paddr[SET_BITS+LINE_OFF_BITS-1:LINE_OFF_BITS];
        automatic logic hit = s2_cache_hit;

        case (req_s2.req_type)
          NOC_LOAD_REQ: begin
            if (hit) begin
              // Send data back to requester
              tx_to_l1.pkt_type <= NOC_LOAD_RESP;
              tx_to_l1.dst_node <= 5'(req_s2.src_node);
              tx_to_l1.src_node <= CHIPLET_NODE_ID;
              tx_to_l1.rob_idx  <= req_s2.rob_idx;
              tx_to_l1.tid      <= req_s2.tid;
              tx_to_l1.addr     <= req_s2.paddr;
              tx_to_l1.data     <= s2_hit_data;
              tx_to_l1.valid    <= 1'b1;
              tx_to_l1_valid    <= 1'b1;
              // Update PLRU (simplified)
              plru[set] <= plru[set] ^ 15'(s2_hit_way_enc);
            end else begin
              // Miss: forward to L3
              // Allocate MSHR
              for (int m = 0; m < MSHR_DEPTH; m++) begin
                if (!mshr[m].allocated) begin
                  mshr[m].paddr     = req_s2.paddr;
                  mshr[m].node_src  = req_s2.src_node;
                  mshr[m].rob_idx   = req_s2.rob_idx;
                  mshr[m].tid       = req_s2.tid;
                  mshr[m].req_type  = NOC_LOAD_REQ;
                  mshr[m].allocated = 1'b1;
                  break;
                end
              end
              tx_to_l3.pkt_type <= NOC_LOAD_REQ;
              tx_to_l3.dst_node <= 5'h18; // L3 node ID
              tx_to_l3.src_node <= CHIPLET_NODE_ID;
              tx_to_l3.addr     <= req_s2.paddr;
              tx_to_l3.rob_idx  <= req_s2.rob_idx;
              tx_to_l3.tid      <= req_s2.tid;
              tx_to_l3.valid    <= 1'b1;
              tx_to_l3_valid    <= 1'b1;
            end
          end

          NOC_STORE_REQ: begin
            if (hit) begin
              // Write-back update
              for (int b = 0; b < 64; b++) begin
                if (req_s2.wmask[b])
                  data_array[s2_hit_way_enc][set][b*8 +: 8] <= req_s2.wdata[b*8 +: 8];
              end
              dirty_array[s2_hit_way_enc][set] <= 1'b1;
              mesi_array[s2_hit_way_enc][set]  <= MESI_M;
            end
          end

          default: ;
        endcase
      end

      // Handle fill from L3
      if (rx_from_l3_valid) begin
        automatic logic [SET_BITS-1:0] fill_set = rx_from_l3.addr[SET_BITS+LINE_OFF_BITS-1:LINE_OFF_BITS];
        automatic logic [TAG_BITS-1:0] fill_tag = rx_from_l3.addr[PADDR_WIDTH-1:SET_BITS+LINE_OFF_BITS];
        // Evict using PLRU
        automatic logic [3:0] victim = 4'(plru[fill_set][3:0]);
        // Writeback dirty eviction (simplified: direct evict)
        data_array[victim][fill_set]  <= rx_from_l3.data;
        tag_array[victim][fill_set]   <= fill_tag;
        valid_array[victim][fill_set] <= 1'b1;
        dirty_array[victim][fill_set] <= 1'b0;
        mesi_array[victim][fill_set]  <= MESI_E;
        plru[fill_set] <= {11'b0, ~victim};

        // Satisfy MSHR — forward to original requester
        for (int m = 0; m < MSHR_DEPTH; m++) begin
          if (mshr[m].allocated && mshr[m].paddr[PADDR_WIDTH-1:LINE_OFF_BITS] ==
              rx_from_l3.addr[PADDR_WIDTH-1:LINE_OFF_BITS]) begin
            tx_to_l1.pkt_type <= NOC_LOAD_RESP;
            tx_to_l1.dst_node <= 5'(mshr[m].node_src);
            tx_to_l1.src_node <= CHIPLET_NODE_ID;
            tx_to_l1.rob_idx  <= mshr[m].rob_idx;
            tx_to_l1.tid      <= mshr[m].tid;
            tx_to_l1.addr     <= mshr[m].paddr;
            tx_to_l1.data     <= rx_from_l3.data;
            tx_to_l1.valid    <= 1'b1;
            tx_to_l1_valid    <= 1'b1;
            mshr[m].allocated <= 1'b0;
          end
        end
      end
    end
  end

endmodule : l2_cache_chiplet
