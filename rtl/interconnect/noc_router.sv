// ============================================================================
// Clione Processor — NoC Router
// Wormhole routing, 5 virtual channels, 8 directions (mesh topology)
// 512-bit data path, credit-based flow control
// ============================================================================
`include "clione_pkg.sv"

module noc_router
  import clione_pkg::*;
#(
  parameter logic [4:0] THIS_NODE_ID = 5'h00,
  parameter int         NUM_PORTS    = 5,   // N, S, E, W, LOCAL
  parameter int         NUM_VCS      = 5,
  parameter int         VC_BUF_DEPTH = 8
)(
  input  logic                            clk,
  input  logic                            rst_n,

  // Input ports (one per direction)
  input  noc_pkt_t                        in_pkt  [NUM_PORTS-1:0],
  input  logic     [NUM_PORTS-1:0]        in_valid,
  output logic     [NUM_PORTS-1:0]        in_credit,   // Credit back to upstream

  // Output ports
  output noc_pkt_t                        out_pkt  [NUM_PORTS-1:0],
  output logic     [NUM_PORTS-1:0]        out_valid,
  input  logic     [NUM_PORTS-1:0]        out_credit   // Credits from downstream
);

  // --------------------------------------------------------------------------
  // Port => Direction mapping
  // --------------------------------------------------------------------------
  localparam int PORT_N     = 0;
  localparam int PORT_S     = 1;
  localparam int PORT_E     = 2;
  localparam int PORT_W     = 3;
  localparam int PORT_LOCAL = 4;

  // --------------------------------------------------------------------------
  // VC Buffers — input queuing
  // --------------------------------------------------------------------------
  noc_pkt_t  vc_buf    [NUM_PORTS-1:0][NUM_VCS-1:0][VC_BUF_DEPTH-1:0];
  logic [2:0] vc_rdptr [NUM_PORTS-1:0][NUM_VCS-1:0];
  logic [2:0] vc_wrptr [NUM_PORTS-1:0][NUM_VCS-1:0];
  logic [3:0] vc_count [NUM_PORTS-1:0][NUM_VCS-1:0];
  logic [2:0] credit_cnt [NUM_PORTS-1:0]; // Outbound credits to peers

  // --------------------------------------------------------------------------
  // Route Computation — XY deterministic routing
  // Node ID encoding: [4:3] = Y coordinate, [2:0] = X coordinate (3x3+ mesh)
  // --------------------------------------------------------------------------
  function automatic logic [2:0] compute_out_port(
    input logic [4:0] dst
  );
    automatic logic [2:0] this_x = THIS_NODE_ID[2:0];
    automatic logic [1:0] this_y = THIS_NODE_ID[4:3];
    automatic logic [2:0] dst_x  = dst[2:0];
    automatic logic [1:0] dst_y  = dst[4:3];

    if      (dst_x > this_x) return 3'(PORT_E);
    else if (dst_x < this_x) return 3'(PORT_W);
    else if (dst_y > this_y) return 3'(PORT_S);
    else if (dst_y < this_y) return 3'(PORT_N);
    else                     return 3'(PORT_LOCAL);
  endfunction

  // --------------------------------------------------------------------------
  // VC selection — assign based on message type for deadlock avoidance
  // --------------------------------------------------------------------------
  function automatic logic [2:0] select_vc(input noc_type_e pkt_type);
    case (pkt_type)
      NOC_DISPATCH:  return 3'd0;
      NOC_RESULT:    return 3'd1;
      NOC_LOAD_REQ:  return 3'd2;
      NOC_LOAD_RESP: return 3'd3;
      NOC_STORE_REQ: return 3'd2;
      NOC_SNOOP:     return 3'd4;
      NOC_FLUSH:     return 3'd4;
      default:       return 3'd0;
    endcase
  endfunction

  // --------------------------------------------------------------------------
  // Output arbitration state (per output port, across input VCs)
  // --------------------------------------------------------------------------
  logic [2:0] out_arb_rr [NUM_PORTS-1:0]; // Round-robin arbitration pointer

  // Crossbar select signals
  logic [2:0]  xbar_sel   [NUM_PORTS-1:0]; // Which input port drives each output
  logic [2:0]  xbar_vc    [NUM_PORTS-1:0]; // Which VC in that input
  logic        xbar_valid [NUM_PORTS-1:0];

  // --------------------------------------------------------------------------
  // Sequential Logic
  // --------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int p = 0; p < NUM_PORTS; p++) begin
        for (int v = 0; v < NUM_VCS; v++) begin
          vc_rdptr[p][v] <= '0; vc_wrptr[p][v] <= '0; vc_count[p][v] <= '0;
        end
        in_credit[p]  <= 1'b0;
        out_valid[p]  <= 1'b0;
        out_arb_rr[p] <= '0;
        credit_cnt[p] <= 3'(VC_BUF_DEPTH);
      end
    end else begin
      // Default de-assert
      for (int p = 0; p < NUM_PORTS; p++) begin
        out_valid[p] <= 1'b0;
        in_credit[p] <= 1'b0;
      end

      // --------------------------------------------------------
      // Stage 1: Enqueue arriving packets into VC buffers
      // --------------------------------------------------------
      for (int p = 0; p < NUM_PORTS; p++) begin
        if (in_valid[p]) begin
          automatic logic [2:0] vc = select_vc(in_pkt[p].pkt_type);
          if (vc_count[p][vc] < VC_BUF_DEPTH) begin
            vc_buf[p][vc][vc_wrptr[p][vc]] <= in_pkt[p];
            vc_wrptr[p][vc] <= vc_wrptr[p][vc] + 3'd1;
            vc_count[p][vc] <= vc_count[p][vc] + 4'd1;
            in_credit[p]    <= 1'b1; // Grant credit back
          end
        end
      end

      // --------------------------------------------------------
      // Stage 2: Switch allocation + crossbar traversal
      // Per output port: scan input ports round-robin to find
      // a VC with a packet routed to this output port
      // --------------------------------------------------------
      for (int op = 0; op < NUM_PORTS; op++) begin
        if (out_credit[op]) begin // Downstream has space
          automatic logic found = 1'b0;
          for (int i = 0; i < NUM_PORTS && !found; i++) begin
            automatic int ip = (int'(out_arb_rr[op]) + i) % NUM_PORTS;
            // Check each VC priority order
            for (int v = 0; v < NUM_VCS && !found; v++) begin
              if (vc_count[ip][v] > 0) begin
                automatic noc_pkt_t pkt = vc_buf[ip][v][vc_rdptr[ip][v]];
                automatic logic [2:0] dst_port = compute_out_port(pkt.dst_node);
                if (int'(dst_port) == op) begin
                  // Grant: route this packet
                  out_pkt[op]       <= pkt;
                  out_valid[op]     <= 1'b1;
                  vc_rdptr[ip][v]   <= vc_rdptr[ip][v] + 3'd1;
                  vc_count[ip][v]   <= vc_count[ip][v] - 4'd1;
                  out_arb_rr[op]    <= 3'(ip + 1);
                  found              = 1'b1;
                end
              end
            end
          end
        end
      end
    end
  end

endmodule : noc_router
