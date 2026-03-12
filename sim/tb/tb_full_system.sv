// ============================================================================
// Clione Processor — Full System Integration Testbench
// Connects: Core Die + ALU + FPU + SIMD + Crypto + L2 + L3 + NoC Fabric
// ============================================================================
`timescale 1ns/1ps
`include "clione_pkg.sv"

module tb_full_system;
  import clione_pkg::*;

  logic clk, rst_n;

  // --------------------------------------------------------------------------
  // NoC fabric wires — 16 nodes × 5 ports
  // --------------------------------------------------------------------------
  localparam int NUM_NODES = 16;

  noc_pkt_t  node_tx_pkt   [NUM_NODES-1:0];
  logic      node_tx_valid [NUM_NODES-1:0];
  logic      node_tx_credit[NUM_NODES-1:0];
  noc_pkt_t  node_rx_pkt   [NUM_NODES-1:0];
  logic      node_rx_valid [NUM_NODES-1:0];
  logic      node_rx_credit[NUM_NODES-1:0];

  // --------------------------------------------------------------------------
  // NoC Fabric
  // --------------------------------------------------------------------------
  noc_fabric_top u_noc (
    .clk            ( clk            ),
    .rst_n          ( rst_n          ),
    .node_tx_pkt    ( node_tx_pkt    ),
    .node_tx_valid  ( node_tx_valid  ),
    .node_tx_credit ( node_tx_credit ),
    .node_rx_pkt    ( node_rx_pkt    ),
    .node_rx_valid  ( node_rx_valid  ),
    .node_rx_credit ( node_rx_credit )
  );

  // --------------------------------------------------------------------------
  // Core Die (node 0)
  // --------------------------------------------------------------------------
  core_die_top u_core (
    .clk            ( clk                     ),
    .rst_n          ( rst_n                   ),
    .noc_tx_pkt     ( node_tx_pkt  [0]        ),
    .noc_tx_valid   ( node_tx_valid[0]        ),
    .noc_tx_ready   ( node_tx_credit[0]       ),
    .noc_rx_pkt     ( node_rx_pkt  [0]        ),
    .noc_rx_valid   ( node_rx_valid[0]        ),
    .noc_rx_ready   ( node_rx_credit[0]       ),
    .cache_tx_pkt   ( node_tx_pkt  [9]        ), // L2 node 9 = (1,2)
    .cache_tx_valid ( node_tx_valid[9]        ),
    .cache_tx_ready ( node_tx_credit[9]       ),
    .cache_rx_pkt   ( node_rx_pkt  [9]        ),
    .cache_rx_valid ( node_rx_valid[9]        ),
    .cache_rx_ready ( node_rx_credit[9]       )
  );

  // --------------------------------------------------------------------------
  // ALU Chiplet 0 (node 1)
  // --------------------------------------------------------------------------
  noc_pkt_t  alu_bypass;
  logic      alu_bypass_v;
  noc_pkt_t  alu_peer [7:0];
  logic [7:0] alu_peer_v;
  assign alu_peer   = '{default: '0};
  assign alu_peer_v = '0;

  alu_chiplet_top #(.CHIPLET_NODE_ID(5'h01)) u_alu0 (
    .clk              ( clk                    ),
    .rst_n            ( rst_n                  ),
    .rx_pkt           ( node_rx_pkt  [1]       ),
    .rx_valid         ( node_rx_valid[1]       ),
    .rx_ready         ( node_rx_credit[1]      ),
    .tx_pkt           ( node_tx_pkt  [1]       ),
    .tx_valid         ( node_tx_valid[1]       ),
    .tx_ready         ( node_tx_credit[1]      ),
    .bypass_result    ( alu_bypass             ),
    .bypass_valid     ( alu_bypass_v           ),
    .peer_bypass      ( alu_peer               ),
    .peer_bypass_valid( alu_peer_v             )
  );

  // --------------------------------------------------------------------------
  // FPU Chiplet 0 (node 4 = index 4)
  // --------------------------------------------------------------------------
  fpu_chiplet_top #(.CHIPLET_NODE_ID(5'h04)) u_fpu0 (
    .clk    ( clk                    ),
    .rst_n  ( rst_n                  ),
    .rx_pkt ( node_rx_pkt  [4]       ),
    .rx_valid( node_rx_valid[4]      ),
    .rx_ready( node_rx_credit[4]     ),
    .tx_pkt ( node_tx_pkt  [4]       ),
    .tx_valid( node_tx_valid[4]      ),
    .tx_ready( node_tx_credit[4]     )
  );

  // --------------------------------------------------------------------------
  // SIMD Chiplet 0 (node 6)
  // --------------------------------------------------------------------------
  simd_chiplet_top #(.CHIPLET_NODE_ID(5'h06)) u_simd0 (
    .clk    ( clk                    ),
    .rst_n  ( rst_n                  ),
    .rx_pkt ( node_rx_pkt  [6]       ),
    .rx_valid( node_rx_valid[6]      ),
    .rx_ready( node_rx_credit[6]     ),
    .tx_pkt ( node_tx_pkt  [6]       ),
    .tx_valid( node_tx_valid[6]      ),
    .tx_ready( node_tx_credit[6]     )
  );

  // --------------------------------------------------------------------------
  // Crypto Chiplet (node 8)
  // --------------------------------------------------------------------------
  crypto_chiplet_top #(.CHIPLET_NODE_ID(5'h08)) u_crypto (
    .clk    ( clk                    ),
    .rst_n  ( rst_n                  ),
    .rx_pkt ( node_rx_pkt  [8]       ),
    .rx_valid( node_rx_valid[8]      ),
    .rx_ready( node_rx_credit[8]     ),
    .tx_pkt ( node_tx_pkt  [8]       ),
    .tx_valid( node_tx_valid[8]      ),
    .tx_ready( node_tx_credit[8]     )
  );

  // --------------------------------------------------------------------------
  // L2 Cache Chiplet (node 9 = index 9, coordinates (1,2))
  // --------------------------------------------------------------------------
  l2_cache_chiplet #(.CHIPLET_NODE_ID(5'h09)) u_l2 (
    .clk            ( clk                     ),
    .rst_n          ( rst_n                   ),
    .rx_from_l1     ( node_rx_pkt  [9]        ),
    .rx_from_l1_valid( node_rx_valid[9]       ),
    .rx_from_l1_ready( node_rx_credit[9]      ),
    .tx_to_l1       ( node_tx_pkt  [9]        ),
    .tx_to_l1_valid ( node_tx_valid[9]        ),
    .tx_to_l1_ready ( node_tx_credit[9]       ),
    .tx_to_l3       ( node_tx_pkt  [12]       ), // L3 = node 12 = (0,3)
    .tx_to_l3_valid ( node_tx_valid[12]       ),
    .tx_to_l3_ready ( node_tx_credit[12]      ),
    .rx_from_l3     ( node_rx_pkt  [12]       ),
    .rx_from_l3_valid( node_rx_valid[12]      ),
    .rx_from_l3_ready( node_rx_credit[12]     )
  );

  // --------------------------------------------------------------------------
  // Tie off unused nodes
  // --------------------------------------------------------------------------
  generate
    for (genvar n = 0; n < NUM_NODES; n++) begin : g_tieoff
      if (n != 0 && n != 1 && n != 4 && n != 6 && n != 8 && n != 9 && n != 12) begin
        assign node_tx_pkt  [n] = '0;
        assign node_tx_valid[n] = 1'b0;
        assign node_rx_credit[n]= 1'b1;
      end
    end
  endgenerate

  // --------------------------------------------------------------------------
  // Clock: 300MHz
  // --------------------------------------------------------------------------
  initial clk = 0;
  always #1.665 clk = ~clk;

  // --------------------------------------------------------------------------
  // Test
  // --------------------------------------------------------------------------
  initial begin
    $dumpfile("tb_full_system.vcd");
    $dumpvars(1, tb_full_system);

    rst_n = 0;
    repeat(20) @(posedge clk);
    rst_n = 1;

    $display("[%0t] System reset complete — running...", $time);
    repeat(5000) @(posedge clk);

    $display("[%0t] Full-system simulation complete", $time);
    $finish;
  end

  // Watchdog
  initial begin
    #500000;
    $display("WATCHDOG TIMEOUT");
    $finish;
  end

  // Monitor retirements
  always @(posedge clk) begin
    if (rst_n && u_core.retire_valid[0])
      $display("[%0t] RETIRE pc=%h rob=%0d",
        $time, u_core.retire_pc[0], u_core.retire_rob_idx[0]);
  end

  // Monitor NoC traffic
  always @(posedge clk) begin
    if (rst_n && node_tx_valid[0])
      $display("[%0t] CORE→NOC: type=%s dst=%h rob=%0d",
        $time, node_tx_pkt[0].pkt_type.name(), node_tx_pkt[0].dst_node,
        node_tx_pkt[0].rob_idx);
  end

endmodule : tb_full_system
