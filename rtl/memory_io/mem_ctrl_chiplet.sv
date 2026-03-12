// ============================================================================
// Clione Processor — Memory Controller Chiplet
// DDR5-6400 / LPDDR5X interface, 4-channel, 64-bit wide per channel
// Prefetch engine, adaptive row policy, refresh scheduler
// ============================================================================
`include "clione_pkg.sv"

module mem_ctrl_chiplet
  import clione_pkg::*;
#(
  parameter logic [4:0] CHIPLET_NODE_ID = 5'h1F,
  parameter int         NUM_CHANNELS    = 4,
  parameter int         MEM_RANKS       = 2,
  parameter int         MEM_BANKS       = 32,        // per rank (DDR5: 8 bank groups × 4 banks)
  parameter int         COL_BITS        = 11,
  parameter int         ROW_BITS        = 17,
  parameter int         BL              = 16          // Burst length (BL16 for DDR5)
)(
  input  logic                            clk,        // Memory clock domain
  input  logic                            rst_n,

  // NoC interface (from L3 chiplet)
  input  noc_pkt_t                        rx_pkt,
  input  logic                            rx_valid,
  output logic                            rx_ready,

  // NoC response (to L3 chiplet)
  output noc_pkt_t                        tx_pkt,
  output logic                            tx_valid,
  input  logic                            tx_ready,

  // PHY pins (abstracted as signals; pad ring connects to physical IO)
  output logic [NUM_CHANNELS-1:0][15:0]   dram_addr,
  output logic [NUM_CHANNELS-1:0][1:0]    dram_ba,       // Bank address
  output logic [NUM_CHANNELS-1:0][1:0]    dram_bg,       // Bank group
  output logic [NUM_CHANNELS-1:0]         dram_cas_n,
  output logic [NUM_CHANNELS-1:0]         dram_ras_n,
  output logic [NUM_CHANNELS-1:0]         dram_we_n,
  output logic [NUM_CHANNELS-1:0]         dram_cs_n,
  output logic [NUM_CHANNELS-1:0]         dram_act_n,    // DDR5: separate ACT command
  inout  logic [NUM_CHANNELS-1:0][63:0]   dram_dq,
  inout  logic [NUM_CHANNELS-1:0][7:0]    dram_dqs,

  // Refresh / ZQ calibration
  output logic [NUM_CHANNELS-1:0]         dram_ref,
  output logic [NUM_CHANNELS-1:0]         dram_zq
);

  // --------------------------------------------------------------------------
  // Transaction Buffer — absorbs NoC traffic before scheduling
  // --------------------------------------------------------------------------
  localparam int TXBUF_DEPTH = 64;

  typedef struct packed {
    logic [PADDR_WIDTH-1:0]   paddr;
    logic [CACHE_LINE_BITS-1:0] wdata;
    logic                      is_write;
    logic [4:0]                src_node;
    logic [ROB_PTR_WIDTH-1:0]  rob_idx;
    logic [TID_WIDTH-1:0]      tid;
    logic                      valid;
  } tx_entry_t;

  tx_entry_t txbuf [TXBUF_DEPTH-1:0];
  logic [5:0] txbuf_head, txbuf_tail;
  logic [6:0] txbuf_count;

  // --------------------------------------------------------------------------
  // Bank State Machine — DRAM timing model (DDR5 tRCD/tCL/tRP)
  // --------------------------------------------------------------------------
  // tRCD=18, tCL=18, tRP=18 (cycles at memory clock ~3.6GHz for DDR5-7200)
  localparam int tRCD = 18;
  localparam int tCL  = 18;
  localparam int tRP  = 18;
  localparam int tRAS = 45;
  localparam int tRC  = 63;
  localparam int tREFI= 3900; // ~ 7.8us at DDR5 system clock

  typedef enum logic [2:0] {
    BANK_IDLE      = 3'b000,
    BANK_ACTIVATING= 3'b001,
    BANK_ACTIVE    = 3'b010,
    BANK_READING   = 3'b011,
    BANK_WRITING   = 3'b100,
    BANK_PRECHARGING=3'b101,
    BANK_REFRESHING= 3'b110
  } bank_state_e;

  typedef struct packed {
    bank_state_e  state;
    logic [ROW_BITS-1:0]  open_row;
    logic [7:0]   timer;      // Timing countdown
    logic         row_open;
  } bank_ctrl_t;

  bank_ctrl_t banks [NUM_CHANNELS-1:0][MEM_RANKS-1:0][MEM_BANKS-1:0];

  // DRAM command types
  typedef enum logic [2:0] {
    CMD_NOP      = 3'b000,
    CMD_ACTIVATE = 3'b001,
    CMD_READ     = 3'b010,
    CMD_WRITE    = 3'b011,
    CMD_PRECHARGE= 3'b100,
    CMD_REFRESH  = 3'b101
  } dram_cmd_e;

  // --------------------------------------------------------------------------
  // Per-Channel Scheduler — Open-page adaptive policy
  // --------------------------------------------------------------------------
  logic [11:0] refresh_counter [NUM_CHANNELS-1:0];
  logic [3:0]  sched_ptr;
  logic [CACHE_LINE_BITS-1:0] dq_out_reg [NUM_CHANNELS-1:0];
  logic        dq_out_en [NUM_CHANNELS-1:0];

  // Prefetch buffer — 8-entry stream prefetcher per channel
  typedef struct packed {
    logic [PADDR_WIDTH-1:0] base_addr;
    logic [2:0]             depth;               // Steps ahead
    logic                   active;
  } prefetch_engine_t;

  prefetch_engine_t pfeng [NUM_CHANNELS-1:0];

  // Tri-state DQ handling
  logic [NUM_CHANNELS-1:0][63:0] dq_out;
  logic [NUM_CHANNELS-1:0]       dq_oe;

  genvar ch;
  generate
    for (genvar ch = 0; ch < NUM_CHANNELS; ch++) begin : g_dq
      assign dram_dq[ch] = dq_oe[ch] ? dq_out[ch] : 64'hZ;
    end
  endgenerate

  // --------------------------------------------------------------------------
  // NoC → Transaction Buffer
  // --------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      txbuf_head  <= '0;
      txbuf_tail  <= '0;
      txbuf_count <= '0;
      rx_ready    <= 1'b1;
      tx_valid    <= 1'b0;
      sched_ptr   <= '0;
      for (int c = 0; c < NUM_CHANNELS; c++) begin
        refresh_counter[c] <= '0;
        dq_oe[c]           <= 1'b0;
        dq_out[c]          <= '0;
        pfeng[c]           <= '0;
        dram_cs_n[c]       <= 1'b1;
        dram_act_n[c]      <= 1'b1;
        dram_cas_n[c]      <= 1'b1;
        dram_ras_n[c]      <= 1'b1;
        dram_we_n[c]       <= 1'b1;
        dram_ref[c]        <= 1'b0;
        dram_zq[c]         <= 1'b0;
        for (int r = 0; r < MEM_RANKS; r++)
          for (int b = 0; b < MEM_BANKS; b++) begin
            banks[c][r][b].state    <= BANK_IDLE;
            banks[c][r][b].row_open <= 1'b0;
            banks[c][r][b].timer    <= '0;
          end
      end
      for (int i = 0; i < TXBUF_DEPTH; i++) txbuf[i] <= '0;
    end else begin
      rx_ready <= (txbuf_count < TXBUF_DEPTH - 2);

      // Enqueue incoming NoC packet
      if (rx_valid && rx_ready) begin
        txbuf[txbuf_tail].paddr    <= rx_pkt.addr;
        txbuf[txbuf_tail].wdata   <= rx_pkt.data;
        txbuf[txbuf_tail].is_write<= (rx_pkt.pkt_type == NOC_STORE_REQ);
        txbuf[txbuf_tail].src_node<= rx_pkt.src_node;
        txbuf[txbuf_tail].rob_idx <= rx_pkt.rob_idx;
        txbuf[txbuf_tail].tid     <= rx_pkt.tid;
        txbuf[txbuf_tail].valid   <= 1'b1;
        txbuf_tail  <= txbuf_tail + 6'd1;
        txbuf_count <= txbuf_count + 7'd1;
      end

      // Refresh scheduler (pseudo-per-channel tREFI)
      for (int c = 0; c < NUM_CHANNELS; c++) begin
        refresh_counter[c] <= refresh_counter[c] + 12'd1;
        if (refresh_counter[c] == 12'(tREFI)) begin
          refresh_counter[c] <= '0;
          dram_ref[c]        <= 1'b1;
        end else begin
          dram_ref[c] <= 1'b0;
        end
      end

      // Dequeue & schedule (simplified round-robin across channels)
      tx_valid <= 1'b0;
      if (txbuf_count > 0) begin
        automatic tx_entry_t cur = txbuf[txbuf_head];
        automatic logic [1:0] ch_sel = cur.paddr[LINE_OFF_BITS+1 : LINE_OFF_BITS];
        automatic logic [1:0] bg_sel = cur.paddr[LINE_OFF_BITS+3 : LINE_OFF_BITS+2];
        automatic logic [1:0] bk_sel = cur.paddr[LINE_OFF_BITS+5 : LINE_OFF_BITS+4];
        automatic int bk_idx         = int'({bg_sel, bk_sel});

        case (banks[ch_sel][0][bk_idx].state)
          BANK_IDLE: begin
            // Issue ACTIVATE
            dram_cs_n[ch_sel]  <= 1'b0;
            dram_act_n[ch_sel] <= 1'b0;
            dram_addr[ch_sel]  <= 16'(cur.paddr[LINE_OFF_BITS + 6 + COL_BITS +: ROW_BITS]);
            dram_ba[ch_sel]    <= bk_sel;
            dram_bg[ch_sel]    <= bg_sel;
            banks[ch_sel][0][bk_idx].state    <= BANK_ACTIVATING;
            banks[ch_sel][0][bk_idx].open_row <= ROW_BITS'(cur.paddr >> (LINE_OFF_BITS+6+COL_BITS));
            banks[ch_sel][0][bk_idx].timer    <= 8'(tRCD);
          end

          BANK_ACTIVATING: begin
            dram_act_n[ch_sel] <= 1'b1;
            if (banks[ch_sel][0][bk_idx].timer > 0)
              banks[ch_sel][0][bk_idx].timer <= banks[ch_sel][0][bk_idx].timer - 8'd1;
            else begin
              banks[ch_sel][0][bk_idx].state    <= BANK_ACTIVE;
              banks[ch_sel][0][bk_idx].row_open <= 1'b1;
            end
          end

          BANK_ACTIVE: begin
            if (!cur.is_write) begin
              // READ command
              dram_cs_n[ch_sel]  <= 1'b0;
              dram_cas_n[ch_sel] <= 1'b0;
              dram_ras_n[ch_sel] <= 1'b1;
              dram_we_n[ch_sel]  <= 1'b1;
              dram_addr[ch_sel]  <= 16'(cur.paddr[LINE_OFF_BITS +: COL_BITS]);
              banks[ch_sel][0][bk_idx].state <= BANK_READING;
              banks[ch_sel][0][bk_idx].timer <= 8'(tCL);
            end else begin
              // WRITE command
              dram_cs_n[ch_sel]  <= 1'b0;
              dram_cas_n[ch_sel] <= 1'b0;
              dram_ras_n[ch_sel] <= 1'b1;
              dram_we_n[ch_sel]  <= 1'b0;
              dram_addr[ch_sel]  <= 16'(cur.paddr[LINE_OFF_BITS +: COL_BITS]);
              dq_out[ch_sel]     <= cur.wdata[63:0];
              dq_oe[ch_sel]      <= 1'b1;
              banks[ch_sel][0][bk_idx].state <= BANK_WRITING;
              banks[ch_sel][0][bk_idx].timer <= 8'(tCL);
            end
          end

          BANK_READING: begin
            dram_cas_n[ch_sel] <= 1'b1;
            if (banks[ch_sel][0][bk_idx].timer > 0)
              banks[ch_sel][0][bk_idx].timer <= banks[ch_sel][0][bk_idx].timer - 8'd1;
            else begin
              // Data available (in real design: CAS latency pipeline)
              banks[ch_sel][0][bk_idx].state <= BANK_ACTIVE;
              // Send read data back via NoC
              tx_pkt.pkt_type <= NOC_LOAD_RESP;
              tx_pkt.dst_node <= cur.src_node;
              tx_pkt.src_node <= CHIPLET_NODE_ID;
              tx_pkt.addr     <= cur.paddr;
              tx_pkt.data     <= {8{dram_dq[ch_sel]}}; // replicate 64b to 512b line
              tx_pkt.rob_idx  <= cur.rob_idx;
              tx_pkt.tid      <= cur.tid;
              tx_pkt.valid    <= 1'b1;
              tx_valid        <= 1'b1;
              // Dequeue
              txbuf[txbuf_head].valid <= 1'b0;
              txbuf_head  <= txbuf_head + 6'd1;
              txbuf_count <= txbuf_count - 7'd1;
            end
          end

          BANK_WRITING: begin
            dram_cas_n[ch_sel] <= 1'b1;
            dram_we_n[ch_sel]  <= 1'b1;
            if (banks[ch_sel][0][bk_idx].timer > 0)
              banks[ch_sel][0][bk_idx].timer <= banks[ch_sel][0][bk_idx].timer - 8'd1;
            else begin
              dq_oe[ch_sel] <= 1'b0;
              banks[ch_sel][0][bk_idx].state <= BANK_ACTIVE;
              txbuf[txbuf_head].valid <= 1'b0;
              txbuf_head  <= txbuf_head + 6'd1;
              txbuf_count <= txbuf_count - 7'd1;
            end
          end

          default: ;
        endcase
      end
    end
  end

endmodule : mem_ctrl_chiplet
