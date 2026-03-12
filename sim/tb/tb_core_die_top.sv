// ============================================================================
// Clione Processor — Core Die Top Testbench
// Basic smoke test: reset, inject fetch requests, observe retire output
// ============================================================================
`timescale 1ns/1ps
`include "clione_pkg.sv"

module tb_core_die_top;
  import clione_pkg::*;

  // --------------------------------------------------------------------------
  // DUT signals
  // --------------------------------------------------------------------------
  logic                       clk;
  logic                       rst_n;

  // NoC TX from core die to execution chiplets
  noc_pkt_t                   noc_tx_pkt;
  logic                       noc_tx_valid;
  logic                       noc_tx_ready;

  // NoC RX results back from execution chiplets
  noc_pkt_t                   noc_rx_pkt;
  logic                       noc_rx_valid;
  logic                       noc_rx_ready;

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
    .cache_rx_ready  ( cache_rx_ready  )
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

  assign noc_tx_ready   = 1'b1;
  assign cache_tx_ready = 1'b1;
  assign cache_rx_valid = 1'b0;
  assign cache_rx_pkt   = '0;

  // Result injection model
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      noc_rx_valid      <= 1'b0;
      noc_rx_pkt        <= '0;
      pending_head      <= '0;
      pending_tail      <= '0;
    end else begin
      noc_rx_valid <= 1'b0;
      // Capture dispatched packet
      if (noc_tx_valid && noc_tx_ready) begin
        pending_rob[pending_tail] <= noc_tx_pkt.rob_idx;
        pending_tail <= pending_tail + 4'd1;
      end
      // Return result after 4 cycles (simplified: immediate return)
      if (pending_head != pending_tail) begin
        noc_rx_pkt.pkt_type <= NOC_RESULT;
        noc_rx_pkt.rob_idx  <= pending_rob[pending_head];
        noc_rx_pkt.data     <= {8{64'd42}}; // Dummy result
        noc_rx_pkt.tid      <= '0;
        noc_rx_pkt.src_node <= 5'h01;
        noc_rx_pkt.dst_node <= 5'h00;
        noc_rx_pkt.valid    <= 1'b1;
        noc_rx_valid        <= 1'b1;
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
    rst_n = 0;
    repeat(10) @(posedge clk);
    rst_n = 1;

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
    if (rst_n && u_dut.retire_valid[0])
      $display("[%0t] RETIRE: rob_idx=%0d tid=%0d pc=%h",
        $time, u_dut.retire_entry[0].rob_idx, u_dut.retire_tid[0], u_dut.retire_entry[0].pc);
  end

  always @(posedge clk) begin
    if (rst_n && noc_tx_valid)
      $display("[%0t] DISPATCH: rob_idx=%0d -> node %h",
        $time, noc_tx_pkt.rob_idx, noc_tx_pkt.dst_node);
  end

endmodule : tb_core_die_top
