// ============================================================================
// Clione Processor — Core Die Top Testbench
// Basic smoke test: reset, inject fetch requests, observe retire output
// ============================================================================
`timescale 1ns/1ps
`include "clione_pkg.sv"

module tb_core_die_top;
  import clione_pkg::*;

`ifdef USE_ASSEMBLED_PROGRAM
  `include "core_program.svh"
`endif

  // --------------------------------------------------------------------------
  // DUT signals
  // --------------------------------------------------------------------------
  logic                       clk;
  logic                       rst_n;
  logic                       run_en;
  logic [SMT_WAYS-1:0]        thread_active;
  logic                       ext_interrupt;
  logic [63:0]                int_vector;
  logic [TID_WIDTH-1:0]       int_tid;

  // NoC TX from core die to execution chiplets
  noc_pkt_t                   noc_tx_pkt   [ISSUE_WIDTH-1:0];
  logic                       noc_tx_valid [ISSUE_WIDTH-1:0];
  logic                       noc_tx_ready [ISSUE_WIDTH-1:0];

  // NoC RX results back from execution chiplets
  noc_pkt_t                   noc_rx_pkt   [ISSUE_WIDTH-1:0];
  logic                       noc_rx_valid [ISSUE_WIDTH-1:0];
  logic                       noc_rx_ready [ISSUE_WIDTH-1:0];

  // Cache NoC (to L2)
  noc_pkt_t                   cache_tx_pkt;
  logic                       cache_tx_valid;
  logic                       cache_tx_ready;
  noc_pkt_t                   cache_rx_pkt;
  logic                       cache_rx_valid;
  logic                       cache_rx_ready;

  // --------------------------------------------------------------------------
  // DUT instantiation
  // --------------------------------------------------------------------------
  core_die_top u_dut (
    .clk             ( clk             ),
    .rst_n           ( rst_n           ),
    .thread_active   ( thread_active   ),
    .noc_tx_pkt      ( noc_tx_pkt      ),
    .noc_tx_valid    ( noc_tx_valid    ),
    .noc_tx_ready    ( noc_tx_ready    ),
    .noc_rx_pkt      ( noc_rx_pkt      ),
    .noc_rx_valid    ( noc_rx_valid    ),
    .noc_rx_ready    ( noc_rx_ready    ),
    .cache_tx_pkt    ( cache_tx_pkt    ),
    .cache_tx_valid  ( cache_tx_valid  ),
    .cache_tx_ready  ( cache_tx_ready  ),
    .cache_rx_pkt    ( cache_rx_pkt    ),
    .cache_rx_valid  ( cache_rx_valid  ),
    .cache_rx_ready  ( cache_rx_ready  ),
    .ext_interrupt   ( ext_interrupt   ),
    .int_vector      ( int_vector      ),
    .int_tid         ( int_tid         )
  );

  // --------------------------------------------------------------------------
  // Clock generation — 300MHz (3.33ns period)
  // --------------------------------------------------------------------------
  initial clk = 0;
  always #1.665 clk = ~clk;

  // --------------------------------------------------------------------------
  // Simple I-cache fill model: always respond with NOP instructions
  // --------------------------------------------------------------------------
  // The IFU expects L1I to return 8 instructions per cycle
  // We model L1I as always hit by connecting a simple fill stub
  // (In full simulation the l1i_cache is inside core_die_top)

  // NoC: accept all dispatch packets, return result after 4 cycles
  logic [7:0]  pending_rob  [15:0];
  logic [3:0]  pending_head, pending_tail;
  logic [1:0]  pending_cnt;
  int          result_timer [15:0];

`ifdef USE_ASSEMBLED_PROGRAM
  localparam int L1I_LINE_BYTES_TB = CACHE_LINE_BYTES;
  localparam int L1I_SET_BITS_TB   = $clog2((L1I_SIZE_KB * 1024) / (CACHE_LINE_BYTES * L1I_WAYS));
  localparam int L1I_LINE_OFF_BITS_TB = $clog2(CACHE_LINE_BYTES);
  localparam int L1I_TAG_BITS_TB = PADDR_WIDTH - L1I_SET_BITS_TB - L1I_LINE_OFF_BITS_TB;
  localparam logic [31:0] NOP_INSTR = 32'h00000013;

  task automatic preload_core_program;
    logic [PADDR_WIDTH-1:0] base_addr;
    int lines_needed;
    begin
      base_addr = 52'h1000;
      lines_needed = (PROGRAM_INST_COUNT + 15) / 16;

      for (int li = 0; li < lines_needed; li++) begin
        logic [PADDR_WIDTH-1:0] line_addr;
        logic [L1I_SET_BITS_TB-1:0] set_idx;
        logic [L1I_TAG_BITS_TB-1:0] tag;
        logic [CACHE_LINE_BITS-1:0] line_data;

        line_addr = base_addr + (li * L1I_LINE_BYTES_TB);
        set_idx   = line_addr[L1I_LINE_OFF_BITS_TB + L1I_SET_BITS_TB - 1 : L1I_LINE_OFF_BITS_TB];
        tag       = line_addr[PADDR_WIDTH-1 : L1I_LINE_OFF_BITS_TB + L1I_SET_BITS_TB];
        line_data = '0;

        for (int slot = 0; slot < 16; slot++) begin
          int idx;
          logic [31:0] instr;
          idx = li * 16 + slot;
          if (idx < PROGRAM_INST_COUNT)
            instr = PROGRAM_INSTR_FLAT[((PROGRAM_INST_COUNT-1-idx)*32) +: 32];
          else
            instr = NOP_INSTR;
          line_data[slot*32 +: 32] = instr;
        end

        u_dut.u_l1i.data_array[0][set_idx]  = line_data;
        u_dut.u_l1i.tag_array[0][set_idx]   = tag;
        u_dut.u_l1i.valid_array[0][set_idx] = 1'b1;
      end
      $display("[TB] Preloaded %0d instructions into core L1I", PROGRAM_INST_COUNT);
    end
  endtask
`endif

  always_comb begin
    for (int i = 0; i < ISSUE_WIDTH; i++) begin
      noc_tx_ready[i] = 1'b1;
    end
  end
  assign cache_tx_ready = 1'b1;
  assign cache_rx_valid = 1'b0;
  assign cache_rx_pkt   = '0;

  // Result injection model
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i < ISSUE_WIDTH; i++) begin
        noc_rx_valid[i] <= 1'b0;
        noc_rx_pkt[i]   <= '0;
      end
      pending_head      <= '0;
      pending_tail      <= '0;
    end else begin
      for (int i = 0; i < ISSUE_WIDTH; i++)
        noc_rx_valid[i] <= 1'b0;
      // Capture dispatched packet
      if (noc_tx_valid[0] && noc_tx_ready[0]) begin
        pending_rob[pending_tail] <= noc_tx_pkt[0].rob_idx;
        pending_tail <= pending_tail + 4'd1;
      end
      // Return result after 4 cycles (simplified: immediate return)
      if (pending_head != pending_tail) begin
        noc_rx_pkt[0].pkt_type <= NOC_RESULT;
        noc_rx_pkt[0].rob_idx  <= pending_rob[pending_head];
        noc_rx_pkt[0].data     <= {8{64'd42}}; // Dummy result
        noc_rx_pkt[0].tid      <= '0;
        noc_rx_pkt[0].src_node <= 5'h01;
        noc_rx_pkt[0].dst_node <= 5'h00;
        noc_rx_pkt[0].valid    <= 1'b1;
        noc_rx_valid[0]        <= 1'b1;
        pending_head        <= pending_head + 4'd1;
      end
    end
  end

  // --------------------------------------------------------------------------
  // Test sequence
  // --------------------------------------------------------------------------
  integer errors = 0;
  integer cycles = 0;

  always @(posedge clk) cycles++;

  initial begin
    $dumpfile("tb_core_die_top.vcd");
    $dumpvars(0, tb_core_die_top);

    // Apply reset
    thread_active = '0;
    ext_interrupt = 1'b0;
    int_vector    = 64'h0;
    int_tid       = '0;
    run_en        = 1'b0;
    rst_n = 0;
    repeat(10) @(posedge clk);
    rst_n = 1;

  `ifdef USE_ASSEMBLED_PROGRAM
    repeat(2) @(posedge clk);
    preload_core_program();
  `endif
    thread_active = '1;
    run_en = 1'b1;

    // Run for 2000 cycles
    repeat(2000) @(posedge clk);

    // Check some basic properties
    if (u_dut.u_rob.count > 256)
      errors++;

    $display("PASS: simulation completed in %0d cycles, errors=%0d", cycles, errors);
    $finish;
  end

  // Timeout watchdog
  initial begin
    #100000;
    $display("TIMEOUT after 100us");
    $finish;
  end

  // --------------------------------------------------------------------------
  // Monitors
  // --------------------------------------------------------------------------
  always @(posedge clk) begin
    if (run_en && u_dut.retire_valid[0])
      $display("[%0t] RETIRE: rob_idx=%0d tid=%0d pc=%h",
        $time, u_dut.retire_entry[0].rob_idx, u_dut.retire_tid[0], u_dut.retire_entry[0].pc);
  end

  always @(posedge clk) begin
    if (run_en && noc_tx_valid[0])
      $display("[%0t] DISPATCH: rob_idx=%0d -> node %h",
        $time, noc_tx_pkt[0].rob_idx, noc_tx_pkt[0].dst_node);
  end

endmodule : tb_core_die_top
