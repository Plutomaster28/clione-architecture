// ============================================================================
// Clione Processor — NoC Fabric Top
// 4×4 mesh of routers connecting all chiplet nodes
// Total nodes: core_die × 1, ALU × 3, FPU × 2, SIMD × 2, Crypto × 1,
//              L2 × 4, L3 × 1, MemCtrl × 1, IO × 1
// Node ID assignment:
//   (0,0) = 0x00 — Core Die / Control
//   (1,0) = 0x01 — ALU Chiplet 0
//   (2,0) = 0x02 — ALU Chiplet 1
//   (3,0) = 0x03 — ALU Chiplet 2
//   (0,1) = 0x04 — FPU Chiplet 0
//   (1,1) = 0x05 — FPU Chiplet 1
//   (2,1) = 0x06 — SIMD Chiplet 0
//   (3,1) = 0x07 — SIMD Chiplet 1
//   (0,2) = 0x08 — Crypto Chiplet
//   (1,2) = 0x09 — L2 Cache 0
//   (2,2) = 0x10 — L2 Cache 1
//   (3,2) = 0x11 — L2 Cache 2
//   (0,3) = 0x18 — L3 Cache
//   (1,3) = 0x1E — IO Chiplet
//   (2,3) = 0x1F — Memory Controller
//   (3,3) = 0x0C — L2 Cache 3
// ============================================================================
`include "clione_pkg.sv"

module noc_fabric_top
  import clione_pkg::*;
#(
  parameter int MESH_X = 4,
  parameter int MESH_Y = 4,
  parameter int NUM_VCS = 5
)(
  input  logic                            clk,
  input  logic                            rst_n,

  // Local port interfaces (inject/eject per node slot)
  input  noc_pkt_t                        node_tx_pkt  [MESH_X*MESH_Y-1:0],
  input  logic     [MESH_X*MESH_Y-1:0]   node_tx_valid,
  output logic     [MESH_X*MESH_Y-1:0]   node_tx_credit,

  output noc_pkt_t                        node_rx_pkt  [MESH_X*MESH_Y-1:0],
  output logic     [MESH_X*MESH_Y-1:0]   node_rx_valid,
  input  logic     [MESH_X*MESH_Y-1:0]   node_rx_credit
);

  localparam int NUM_NODES   = MESH_X * MESH_Y;  // 16
  localparam int NUM_PORTS   = 5; // N/S/E/W/LOCAL

  // Node ID grid — static assignment
  // node_id[y][x] = 5'(y*MESH_X + x) — simple flat indexing
  // (matches bit encoding in router's compute_out_port if Y=[4:3], X=[2:0])
  function automatic logic [4:0] xy_to_nodeid(input int x, y);
    return 5'({2'(y), 3'(x)});
  endfunction

  // --------------------------------------------------------------------------
  // Router instances and inter-router wires
  // --------------------------------------------------------------------------
  // For each router: ports 0=N, 1=S, 2=E, 3=W, 4=LOCAL
  // We generate wires between neighbors

  noc_pkt_t  r_in_pkt   [NUM_NODES-1:0][NUM_PORTS-1:0];
  logic [NUM_PORTS-1:0] r_in_valid  [NUM_NODES-1:0];
  logic [NUM_PORTS-1:0] r_in_credit [NUM_NODES-1:0];
  noc_pkt_t  r_out_pkt  [NUM_NODES-1:0][NUM_PORTS-1:0];
  logic [NUM_PORTS-1:0] r_out_valid [NUM_NODES-1:0];
  logic [NUM_PORTS-1:0] r_out_credit[NUM_NODES-1:0];

  // --------------------------------------------------------------------------
  // Generate routers
  // --------------------------------------------------------------------------
  generate
    for (genvar y = 0; y < MESH_Y; y++) begin : g_row
      for (genvar x = 0; x < MESH_X; x++) begin : g_col
        localparam int NODE = y * MESH_X + x;

        noc_router #(
          .THIS_NODE_ID ( 5'({2'(y), 3'(x)}) ),
          .NUM_VCS      ( NUM_VCS )
        ) u_router (
          .clk       ( clk ),
          .rst_n     ( rst_n ),
          .in_pkt    ( r_in_pkt  [NODE] ),
          .in_valid  ( r_in_valid [NODE] ),
          .in_credit ( r_in_credit[NODE] ),
          .out_pkt   ( r_out_pkt  [NODE] ),
          .out_valid ( r_out_valid [NODE] ),
          .out_credit( r_out_credit[NODE] )
        );
      end
    end
  endgenerate

  // --------------------------------------------------------------------------
  // Wire mesh inter-router connections
  // --------------------------------------------------------------------------
  // PORT_N = 0: src.N → north neighbor.S
  // PORT_S = 1: src.S → south neighbor.N
  // PORT_E = 2: src.E → east neighbor.W
  // PORT_W = 3: src.W → west neighbor.E
  // PORT_LOCAL = 4: connected to node inject/eject

  generate
    for (genvar y = 0; y < MESH_Y; y++) begin : g_row_w
      for (genvar x = 0; x < MESH_X; x++) begin : g_col_w
        localparam int NODE = y * MESH_X + x;

        // ---- NORTH connection ----
        if (y > 0) begin : g_has_north
          localparam int NORTH_NODE = (y-1) * MESH_X + x;
          // This node's PORT_N output → north node's PORT_S input
          assign r_in_pkt  [NORTH_NODE][1] = r_out_pkt  [NODE][0];
          assign r_in_valid [NORTH_NODE][1] = r_out_valid [NODE][0];
          assign r_out_credit[NODE][0]       = r_in_credit[NORTH_NODE][1];
        end else begin : g_no_north
          // Boundary: tie off
          assign r_in_pkt  [NODE][0] = '0;
          assign r_in_valid [NODE][0] = 1'b0;
          assign r_out_credit[NODE][0] = 1'b0;
        end

        // ---- SOUTH connection ----
        if (y < MESH_Y - 1) begin : g_has_south
          localparam int SOUTH_NODE = (y+1) * MESH_X + x;
          assign r_in_pkt  [SOUTH_NODE][0] = r_out_pkt  [NODE][1];
          assign r_in_valid [SOUTH_NODE][0] = r_out_valid [NODE][1];
          assign r_out_credit[NODE][1]       = r_in_credit[SOUTH_NODE][0];
        end else begin : g_no_south
          assign r_in_pkt  [NODE][1] = '0;
          assign r_in_valid [NODE][1] = 1'b0;
          assign r_out_credit[NODE][1] = 1'b0;
        end

        // ---- EAST connection ----
        if (x < MESH_X - 1) begin : g_has_east
          localparam int EAST_NODE = y * MESH_X + (x+1);
          assign r_in_pkt  [EAST_NODE][3] = r_out_pkt  [NODE][2];
          assign r_in_valid [EAST_NODE][3] = r_out_valid [NODE][2];
          assign r_out_credit[NODE][2]      = r_in_credit[EAST_NODE][3];
        end else begin : g_no_east
          assign r_in_pkt  [NODE][2] = '0;
          assign r_in_valid [NODE][2] = 1'b0;
          assign r_out_credit[NODE][2] = 1'b0;
        end

        // ---- WEST connection ----
        if (x > 0) begin : g_has_west
          localparam int WEST_NODE = y * MESH_X + (x-1);
          assign r_in_pkt  [WEST_NODE][2] = r_out_pkt  [NODE][3];
          assign r_in_valid [WEST_NODE][2] = r_out_valid [NODE][3];
          assign r_out_credit[NODE][3]      = r_in_credit[WEST_NODE][2];
        end else begin : g_no_west
          assign r_in_pkt  [NODE][3] = '0;
          assign r_in_valid [NODE][3] = 1'b0;
          assign r_out_credit[NODE][3] = 1'b0;
        end

        // ---- LOCAL port: connect to node interfaces ----
        assign r_in_pkt  [NODE][4]     = node_tx_pkt  [NODE];
        assign r_in_valid [NODE][4]    = node_tx_valid [NODE];
        assign node_tx_credit[NODE]    = r_in_credit[NODE][4];
        assign node_rx_pkt   [NODE]    = r_out_pkt  [NODE][4];
        assign node_rx_valid [NODE]    = r_out_valid [NODE][4];
        assign r_out_credit[NODE][4]   = node_rx_credit[NODE];
      end
    end
  endgenerate

endmodule : noc_fabric_top
