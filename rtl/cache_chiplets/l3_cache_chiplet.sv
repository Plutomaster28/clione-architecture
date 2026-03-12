// ============================================================================
// Clione Processor — L3 Cache Chiplet (Shared Last-Level Cache)
// 32MB banked LLC, 32-way SA, directory-based coherency for all nodes
// Connects to all L2 chiplets and Memory Controller via NoC
// ============================================================================
`include "clione_pkg.sv"

module l3_cache_chiplet
  import clione_pkg::*;
#(
  parameter logic [4:0] CHIPLET_NODE_ID = 5'h18,
  parameter int         SIZE_MB         = 32,
  parameter int         WAYS            = 32,
  parameter int         NUM_BANKS       = 16     // Bank interleaving for bandwidth
)(
  input  logic                            clk,
  input  logic                            rst_n,

  // NoC interfaces — 4 L2 chiplets upstream
  input  noc_pkt_t                        rx_pkt [NUM_BANKS-1:0],
  input  logic     [NUM_BANKS-1:0]        rx_valid,
  output logic     [NUM_BANKS-1:0]        rx_ready,

  // Response to L2s
  output noc_pkt_t                        tx_pkt [NUM_BANKS-1:0],
  output logic     [NUM_BANKS-1:0]        tx_valid,
  input  logic     [NUM_BANKS-1:0]        tx_ready,

  // Interface to Memory Controller chiplet
  output noc_pkt_t                        mem_req_pkt,
  output logic                            mem_req_valid,
  input  logic                            mem_req_ready,

  input  noc_pkt_t                        mem_resp_pkt,
  input  logic                            mem_resp_valid,
  output logic                            mem_resp_ready
);

  // --------------------------------------------------------------------------
  // Per-Bank Cache Parameters
  // --------------------------------------------------------------------------
  localparam int BANK_SIZE_BYTES  = (SIZE_MB * 1024 * 1024) / NUM_BANKS;
  localparam int LINE_BYTES       = CACHE_LINE_BYTES;       // 64
  localparam int BANK_SETS        = BANK_SIZE_BYTES / (LINE_BYTES * WAYS); // 1024
  localparam int SET_BITS         = $clog2(BANK_SETS);      // 10
  localparam int LINE_OFF_BITS    = $clog2(LINE_BYTES);      // 6
  localparam int BANK_BITS        = $clog2(NUM_BANKS);       // 4
  localparam int TAG_BITS         = PADDR_WIDTH - SET_BITS - LINE_OFF_BITS - BANK_BITS;

  // --------------------------------------------------------------------------
  // Bank-Local Arrays
  // --------------------------------------------------------------------------
  // Due to parameterization we model as flattened arrays per bank
  logic [TAG_BITS-1:0]          tag_arr   [NUM_BANKS-1:0][WAYS-1:0][BANK_SETS-1:0];
  logic                          valid_arr [NUM_BANKS-1:0][WAYS-1:0][BANK_SETS-1:0];
  logic [CACHE_LINE_BITS-1:0]   data_arr  [NUM_BANKS-1:0][WAYS-1:0][BANK_SETS-1:0];
  mesi_state_e                   mesi_arr  [NUM_BANKS-1:0][WAYS-1:0][BANK_SETS-1:0];

  // 5-bit PLRU tree per set per bank (simplified for 32-way: log2(32)=5 levels => 31 bits, simplified to 8 bits)
  logic [7:0]                    plru_arr  [NUM_BANKS-1:0][BANK_SETS-1:0];

  // --------------------------------------------------------------------------
  // Coherency Directory — tracks presence in L2s
  // Sharers encoded as bitmask (bit = L2 node, up to 16 L2s)
  // --------------------------------------------------------------------------
  typedef struct packed {
    logic [15:0]  sharers;           // Which L2 chiplets hold the line
    logic         owner_valid;       // Is there an M-state owner
    logic [3:0]   owner;             // Node index of M owner
    logic         dir_valid;
  } dir_entry_t;

  dir_entry_t dir [NUM_BANKS-1:0][BANK_SETS-1:0] /* synthesis ramstyle = "no_rw_check" */;

  // --------------------------------------------------------------------------
  // MSHR — 64 entries per bank for outstanding DRAM fills
  // --------------------------------------------------------------------------
  localparam int MSHR_DEPTH = 64;

  typedef struct packed {
    logic [PADDR_WIDTH-1:0]   paddr;
    logic [4:0]               req_node;
    logic [ROB_PTR_WIDTH-1:0] rob_idx;
    logic [TID_WIDTH-1:0]     tid;
    logic                     allocated;
  } l3_mshr_t;

  l3_mshr_t mshr [NUM_BANKS-1:0][MSHR_DEPTH-1:0];

  // --------------------------------------------------------------------------
  // Bank Arbitration — round-robin across sources
  // --------------------------------------------------------------------------
  logic [3:0] rr_bank [NUM_BANKS-1:0];

  // --------------------------------------------------------------------------
  // Per-Bank Processing
  // --------------------------------------------------------------------------
  // We instantiate per-bank logic via generate

  generate
    for (genvar b = 0; b < NUM_BANKS; b++) begin : g_bank

      // Registered pipeline entry
      logic [PADDR_WIDTH-1:0]    p1_paddr;
      logic [CACHE_LINE_BITS-1:0] p1_wdata;
      noc_type_e                  p1_req_type;
      logic [4:0]                 p1_src_node;
      logic [ROB_PTR_WIDTH-1:0]   p1_rob_idx;
      logic [TID_WIDTH-1:0]       p1_tid;
      logic                       p1_valid;

      // Lookup wires
      logic [TAG_BITS-1:0]        p1_tag;
      logic [SET_BITS-1:0]        p1_set;
      logic [$clog2(WAYS)-1:0]    p1_way_hit;
      logic                       p1_hit;
      logic [CACHE_LINE_BITS-1:0] p1_hit_data;

      always_comb begin
        p1_tag     = p1_paddr[PADDR_WIDTH-1 : SET_BITS+LINE_OFF_BITS+BANK_BITS];
        p1_set     = p1_paddr[SET_BITS+LINE_OFF_BITS+BANK_BITS-1 : LINE_OFF_BITS+BANK_BITS];
        p1_way_hit = '0; p1_hit = 1'b0; p1_hit_data = '0;
        for (int w = 0; w < WAYS; w++) begin
          if (valid_arr[b][w][p1_set] && tag_arr[b][w][p1_set] == p1_tag &&
              mesi_arr[b][w][p1_set] != MESI_I) begin
            p1_way_hit = ($clog2(WAYS))'(w);
            p1_hit     = 1'b1;
            p1_hit_data = data_arr[b][w][p1_set];
          end
        end
      end

      always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
          p1_valid     <= 1'b0;
          tx_valid[b]  <= 1'b0;
          rx_ready[b]  <= 1'b1;
          for (int w = 0; w < WAYS; w++)
            for (int s = 0; s < BANK_SETS; s++) begin
              valid_arr[b][w][s]  <= 1'b0;
              mesi_arr[b][w][s]   <= MESI_I;
            end
          for (int m = 0; m < MSHR_DEPTH; m++)
            mshr[b][m] <= '0;
        end else begin
          tx_valid[b] <= 1'b0;
          rx_ready[b] <= 1'b1;

          // Accept request if bank address matches (bits [BANK_BITS+LINE_OFF_BITS-1:LINE_OFF_BITS])
          if (rx_valid[b]) begin
            automatic logic [BANK_BITS-1:0] bank_sel =
              rx_pkt[b].addr[LINE_OFF_BITS+BANK_BITS-1 : LINE_OFF_BITS];
            if (bank_sel == BANK_BITS'(b)) begin
              p1_paddr    <= rx_pkt[b].addr;
              p1_wdata    <= rx_pkt[b].data;
              p1_req_type <= rx_pkt[b].pkt_type;
              p1_src_node <= rx_pkt[b].src_node;
              p1_rob_idx  <= rx_pkt[b].rob_idx;
              p1_tid      <= rx_pkt[b].tid;
              p1_valid    <= 1'b1;
            end else begin
              p1_valid <= 1'b0;
            end
          end else begin
            p1_valid <= 1'b0;
          end

          // Process pipeline stage 1
          if (p1_valid) begin
            case (p1_req_type)
              NOC_LOAD_REQ: begin
                if (p1_hit) begin
                  // Hit — respond to requesting L2
                  tx_pkt[b].pkt_type <= NOC_LOAD_RESP;
                  tx_pkt[b].dst_node <= p1_src_node;
                  tx_pkt[b].src_node <= CHIPLET_NODE_ID;
                  tx_pkt[b].addr     <= p1_paddr;
                  tx_pkt[b].data     <= p1_hit_data;
                  tx_pkt[b].rob_idx  <= p1_rob_idx;
                  tx_pkt[b].tid      <= p1_tid;
                  tx_pkt[b].valid    <= 1'b1;
                  tx_valid[b]        <= 1'b1;
                  // Update directory: add sharer
                  dir[b][p1_set].sharers     <= dir[b][p1_set].sharers | (16'(1) << p1_src_node[3:0]);
                  dir[b][p1_set].dir_valid   <= 1'b1;
                  plru_arr[b][p1_set] <= plru_arr[b][p1_set] ^ 8'(p1_way_hit);
                end else begin
                  // Miss — forward to DRAM controller
                  // Allocate MSHR
                  for (int m = 0; m < MSHR_DEPTH; m++) begin
                    if (!mshr[b][m].allocated) begin
                      mshr[b][m].paddr     <= p1_paddr;
                      mshr[b][m].req_node  <= p1_src_node;
                      mshr[b][m].rob_idx   <= p1_rob_idx;
                      mshr[b][m].tid       <= p1_tid;
                      mshr[b][m].allocated <= 1'b1;
                      break;
                    end
                  end
                  mem_req_pkt.pkt_type <= NOC_LOAD_REQ;
                  mem_req_pkt.dst_node <= 5'h1F; // Memory controller node
                  mem_req_pkt.src_node <= CHIPLET_NODE_ID;
                  mem_req_pkt.addr     <= p1_paddr;
                  mem_req_pkt.rob_idx  <= p1_rob_idx;
                  mem_req_pkt.tid      <= p1_tid;
                  mem_req_pkt.valid    <= 1'b1;
                  mem_req_valid        <= 1'b1;
                end
              end

              NOC_STORE_REQ: begin
                // Write-back from L2 (dirty line writeback)
                if (p1_hit) begin
                  data_arr[b][p1_way_hit][p1_set] <= p1_wdata;
                  mesi_arr[b][p1_way_hit][p1_set] <= MESI_M;
                  dir[b][p1_set].owner       <= p1_src_node[3:0];
                  dir[b][p1_set].owner_valid <= 1'b1;
                end
              end

              NOC_FLUSH: begin
                // L2 is flushing a line — write back to L3 and remove sharer
                if (p1_hit) begin
                  data_arr[b][p1_way_hit][p1_set] <= p1_wdata;
                  mesi_arr[b][p1_way_hit][p1_set] <= MESI_S;
                  dir[b][p1_set].sharers <= dir[b][p1_set].sharers &
                                            ~(16'(1) << p1_src_node[3:0]);
                end
              end

              NOC_SNOOP: begin
                // Snoop from another L2 (GetS/GetX)
                // Invalidate non-requesting sharers
                if (p1_hit && dir[b][p1_set].dir_valid) begin
                  for (int s = 0; s < 16; s++) begin
                    if (dir[b][p1_set].sharers[s] && s != int'(p1_src_node[3:0])) begin
                      tx_pkt[b].pkt_type <= NOC_SNOOP;
                      tx_pkt[b].dst_node <= 5'(s);
                      tx_pkt[b].src_node <= CHIPLET_NODE_ID;
                      tx_pkt[b].addr     <= p1_paddr;
                      tx_pkt[b].valid    <= 1'b1;
                      tx_valid[b]        <= 1'b1;
                    end
                  end
                  mesi_arr[b][p1_way_hit][p1_set] <= MESI_E;
                  dir[b][p1_set].sharers <= 16'(1) << p1_src_node[3:0];
                end
              end

              default: ;
            endcase
          end

          // Handle DRAM fill response for this bank
          if (mem_resp_valid) begin
            automatic logic [BANK_BITS-1:0] resp_bank =
              mem_resp_pkt.addr[LINE_OFF_BITS+BANK_BITS-1 : LINE_OFF_BITS];
            if (resp_bank == BANK_BITS'(b)) begin
              automatic logic [SET_BITS-1:0] resp_set =
                mem_resp_pkt.addr[SET_BITS+LINE_OFF_BITS+BANK_BITS-1 : LINE_OFF_BITS+BANK_BITS];
              automatic logic [TAG_BITS-1:0] resp_tag =
                mem_resp_pkt.addr[PADDR_WIDTH-1 : SET_BITS+LINE_OFF_BITS+BANK_BITS];
              automatic logic [4:0] evict_way = 5'(plru_arr[b][resp_set][4:0]);
              // Install line
              data_arr[b][evict_way][resp_set]  <= mem_resp_pkt.data;
              tag_arr[b][evict_way][resp_set]   <= resp_tag;
              valid_arr[b][evict_way][resp_set] <= 1'b1;
              mesi_arr[b][evict_way][resp_set]  <= MESI_E;
              plru_arr[b][resp_set] <= ~plru_arr[b][resp_set];
              // Satisfy MSHR
              for (int m = 0; m < MSHR_DEPTH; m++) begin
                if (mshr[b][m].allocated &&
                    mshr[b][m].paddr[PADDR_WIDTH-1:LINE_OFF_BITS] ==
                    mem_resp_pkt.addr[PADDR_WIDTH-1:LINE_OFF_BITS]) begin
                  tx_pkt[b].pkt_type <= NOC_LOAD_RESP;
                  tx_pkt[b].dst_node <= mshr[b][m].req_node;
                  tx_pkt[b].src_node <= CHIPLET_NODE_ID;
                  tx_pkt[b].addr     <= mshr[b][m].paddr;
                  tx_pkt[b].data     <= mem_resp_pkt.data;
                  tx_pkt[b].rob_idx  <= mshr[b][m].rob_idx;
                  tx_pkt[b].tid      <= mshr[b][m].tid;
                  tx_pkt[b].valid    <= 1'b1;
                  tx_valid[b]        <= 1'b1;
                  mshr[b][m].allocated <= 1'b0;
                end
              end
              mem_resp_ready <= 1'b1;
            end
          end
        end
      end
    end : g_bank
  endgenerate

endmodule : l3_cache_chiplet
