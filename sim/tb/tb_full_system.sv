// ============================================================================
// Clione Processor — Full System Integration Testbench
// Connects: Core Die + ALU + FPU + SIMD + Crypto + L2 + L3 + NoC Fabric
// ============================================================================
`timescale 1ns/1ps
`include "clione_pkg.sv"

module tb_full_system;
  import clione_pkg::*;

`ifdef USE_ASSEMBLED_PROGRAM
  `include "full_program.svh"
`endif

  logic clk, rst_n;
  logic [SMT_WAYS-1:0] thread_active;
  logic                ext_interrupt;
  logic [63:0]         int_vector;
  logic [TID_WIDTH-1:0] int_tid;
  logic                run_en;

  // --------------------------------------------------------------------------
  // NoC fabric wires — 16 nodes × 5 ports
  // --------------------------------------------------------------------------
  localparam int NUM_NODES = 16;

`ifdef USE_ASSEMBLED_PROGRAM
  localparam int L1I_LINE_BYTES_TB = CACHE_LINE_BYTES;
  localparam int L1I_SET_BITS_TB   = $clog2((L1I_SIZE_KB * 1024) / (CACHE_LINE_BYTES * L1I_WAYS));
  localparam int L1I_LINE_OFF_BITS_TB = $clog2(CACHE_LINE_BYTES);
  localparam int L1I_TAG_BITS_TB = PADDR_WIDTH - L1I_SET_BITS_TB - L1I_LINE_OFF_BITS_TB;
  localparam logic [31:0] NOP_INSTR = 32'h00000013;

  task automatic preload_full_program;
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

        u_core.u_l1i.data_array[0][set_idx]  = line_data;
        u_core.u_l1i.tag_array[0][set_idx]   = tag;
        u_core.u_l1i.valid_array[0][set_idx] = 1'b1;
      end
      $display("[TB] Preloaded %0d instructions into full-system core L1I", PROGRAM_INST_COUNT);
    end
  endtask
`endif

  noc_pkt_t  node_tx_pkt   [NUM_NODES-1:0];
  logic [NUM_NODES-1:0] node_tx_valid;
  logic [NUM_NODES-1:0] node_tx_credit;
  noc_pkt_t  node_rx_pkt   [NUM_NODES-1:0];
  logic [NUM_NODES-1:0] node_rx_valid;
  logic [NUM_NODES-1:0] node_rx_credit;

  // Core die has ISSUE_WIDTH NoC lanes; bridge lane 0 to mesh node 0.
  noc_pkt_t core_noc_tx_pkt   [ISSUE_WIDTH-1:0];
  logic     core_noc_tx_valid [ISSUE_WIDTH-1:0];
  logic     core_noc_tx_ready [ISSUE_WIDTH-1:0];
  noc_pkt_t core_noc_rx_pkt   [ISSUE_WIDTH-1:0];
  logic     core_noc_rx_valid [ISSUE_WIDTH-1:0];
  logic     core_noc_rx_ready [ISSUE_WIDTH-1:0];

  // Dedicated cache-link between core die and local L2 model.
  noc_pkt_t core_cache_tx_pkt;
  logic     core_cache_tx_valid;
  logic     core_cache_tx_ready;
  noc_pkt_t core_cache_rx_pkt;
  logic     core_cache_rx_valid;
  logic     core_cache_rx_ready;

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
  assign node_tx_pkt[0]    = core_noc_tx_pkt[0];
  assign node_tx_valid[0]  = core_noc_tx_valid[0];
  assign core_noc_tx_ready[0] = node_tx_credit[0];
  assign core_noc_rx_pkt[0]   = node_rx_pkt[0];
  assign core_noc_rx_valid[0] = node_rx_valid[0];
  assign node_rx_credit[0]    = core_noc_rx_ready[0];

  generate
    for (genvar i = 1; i < ISSUE_WIDTH; i++) begin : g_core_noc_tieoff
      assign core_noc_tx_ready[i] = 1'b1;
      assign core_noc_rx_pkt[i]   = '0;
      assign core_noc_rx_valid[i] = 1'b0;
    end
  endgenerate

  core_die_top u_core (
    .clk            ( clk                     ),
    .rst_n          ( rst_n                   ),
    .thread_active  ( thread_active           ),
    .noc_tx_pkt     ( core_noc_tx_pkt         ),
    .noc_tx_valid   ( core_noc_tx_valid       ),
    .noc_tx_ready   ( core_noc_tx_ready       ),
    .noc_rx_pkt     ( core_noc_rx_pkt         ),
    .noc_rx_valid   ( core_noc_rx_valid       ),
    .noc_rx_ready   ( core_noc_rx_ready       ),
    .cache_tx_pkt   ( core_cache_tx_pkt       ),
    .cache_tx_valid ( core_cache_tx_valid     ),
    .cache_tx_ready ( core_cache_tx_ready     ),
    .cache_rx_pkt   ( core_cache_rx_pkt       ),
    .cache_rx_valid ( core_cache_rx_valid     ),
    .cache_rx_ready ( core_cache_rx_ready     ),
    .ext_interrupt  ( ext_interrupt           ),
    .int_vector     ( int_vector              ),
    .int_tid        ( int_tid                 )
  );

  // --------------------------------------------------------------------------
  // ALU Chiplet 0 (node 1)
  // --------------------------------------------------------------------------
  exec_result_t alu_bypass;
  logic         alu_bypass_v;
  exec_result_t alu_peer [7:0];
  logic         alu_peer_v [7:0];
  assign alu_peer   = '{default: '0};
  assign alu_peer_v = '{default: 1'b0};

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
    .peer_valid       ( alu_peer_v             )
  );

  // --------------------------------------------------------------------------
  // FPU Chiplet 0 (node 4 = index 4)
  // --------------------------------------------------------------------------
  exec_result_t fpu_bypass;
  logic         fpu_bypass_v;
  exec_result_t fpu_peer [7:0];
  logic         fpu_peer_v [7:0];
  assign fpu_peer   = '{default: '0};
  assign fpu_peer_v = '{default: 1'b0};

  fpu_chiplet_top #(.CHIPLET_NODE_ID(5'h04)) u_fpu0 (
    .clk    ( clk                    ),
    .rst_n  ( rst_n                  ),
    .rx_pkt ( node_rx_pkt  [4]       ),
    .rx_valid( node_rx_valid[4]      ),
    .rx_ready( node_rx_credit[4]     ),
    .tx_pkt ( node_tx_pkt  [4]       ),
    .tx_valid( node_tx_valid[4]      ),
    .tx_ready( node_tx_credit[4]     ),
    .bypass_result( fpu_bypass       ),
    .bypass_valid ( fpu_bypass_v     ),
    .peer_bypass  ( fpu_peer         ),
    .peer_valid   ( fpu_peer_v       )
  );

  // --------------------------------------------------------------------------
  // SIMD Chiplet 0 (node 6)
  // --------------------------------------------------------------------------
  exec_result_t simd_bypass;
  logic         simd_bypass_v;
  exec_result_t simd_peer [7:0];
  logic         simd_peer_v [7:0];
  assign simd_peer   = '{default: '0};
  assign simd_peer_v = '{default: 1'b0};

  simd_chiplet_top #(.CHIPLET_NODE_ID(5'h06)) u_simd0 (
    .clk    ( clk                    ),
    .rst_n  ( rst_n                  ),
    .rx_pkt ( node_rx_pkt  [6]       ),
    .rx_valid( node_rx_valid[6]      ),
    .rx_ready( node_rx_credit[6]     ),
    .tx_pkt ( node_tx_pkt  [6]       ),
    .tx_valid( node_tx_valid[6]      ),
    .tx_ready( node_tx_credit[6]     ),
    .bypass_result( simd_bypass      ),
    .bypass_valid ( simd_bypass_v    ),
    .peer_bypass  ( simd_peer        ),
    .peer_valid   ( simd_peer_v      )
  );

  // --------------------------------------------------------------------------
  // Crypto Chiplet (node 8)
  // --------------------------------------------------------------------------
  exec_result_t crypto_bypass;
  logic         crypto_bypass_v;

  crypto_chiplet_top #(.CHIPLET_NODE_ID(5'h08)) u_crypto (
    .clk    ( clk                    ),
    .rst_n  ( rst_n                  ),
    .rx_pkt ( node_rx_pkt  [8]       ),
    .rx_valid( node_rx_valid[8]      ),
    .rx_ready( node_rx_credit[8]     ),
    .tx_pkt ( node_tx_pkt  [8]       ),
    .tx_valid( node_tx_valid[8]      ),
    .tx_ready( node_tx_credit[8]     ),
    .bypass_result( crypto_bypass    ),
    .bypass_valid ( crypto_bypass_v  )
  );

  // --------------------------------------------------------------------------
  // L2 Cache Chiplet (node 9 = index 9, coordinates (1,2))
  // --------------------------------------------------------------------------
  l2_cache_chiplet #(.CHIPLET_NODE_ID(5'h09)) u_l2 (
    .clk            ( clk                     ),
    .rst_n          ( rst_n                   ),
    .rx_from_l1     ( core_cache_tx_pkt       ),
    .rx_from_l1_valid( core_cache_tx_valid    ),
    .rx_from_l1_ready( core_cache_tx_ready    ),
    .tx_to_l1       ( core_cache_rx_pkt       ),
    .tx_to_l1_valid ( core_cache_rx_valid     ),
    .tx_to_l1_ready ( core_cache_rx_ready     ),
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
      if (n != 0 && n != 1 && n != 4 && n != 6 && n != 8 && n != 12) begin
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
    run_en = 1'b0;
    thread_active = '0;
    ext_interrupt = 1'b0;
    int_vector    = '0;
    int_tid       = '0;
    repeat(20) @(posedge clk);
    rst_n = 1;

  `ifdef USE_ASSEMBLED_PROGRAM
    repeat(2) @(posedge clk);
    preload_full_program();
  `endif
    thread_active = '1;
    run_en = 1'b1;

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
    if (run_en && u_core.retire_valid[0])
      $display("[%0t] RETIRE pc=%h rob=%0d",
        $time, u_core.retire_entry[0].pc, u_core.retire_entry[0].rob_idx);
  end

  // Monitor NoC traffic
  always @(posedge clk) begin
    if (run_en && node_tx_valid[0])
      $display("[%0t] CORE→NOC: type=%s dst=%h rob=%0d",
        $time, node_tx_pkt[0].pkt_type.name(), node_tx_pkt[0].dst_node,
        node_tx_pkt[0].rob_idx);
  end

endmodule : tb_full_system
