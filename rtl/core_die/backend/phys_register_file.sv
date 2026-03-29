// ============================================================================
// Clione Processor — Physical Register File (INT + FP)
// Dual-ported: ISSUE_WIDTH read ports, RETIRE_WIDTH write ports
// ============================================================================
`include "clione_pkg.sv"

module phys_register_file
  import clione_pkg::*;
(
  input  logic                        clk,
  input  logic                        rst_n,

  // Read ports: ISSUE_WIDTH * 3 sources (rs1, rs2, rs3 per slot)
  input  logic [PREG_WIDTH-1:0]       rd_addr  [ISSUE_WIDTH*3-1:0],
  output logic [XLEN-1:0]            rd_data  [ISSUE_WIDTH*3-1:0],

  // Write ports: result from ISSUE_WIDTH execution results
  input  logic [PREG_WIDTH-1:0]       wr_addr  [ISSUE_WIDTH-1:0],
  input  logic [XLEN-1:0]            wr_data  [ISSUE_WIDTH-1:0],
  input  logic                        wr_valid [ISSUE_WIDTH-1:0]
);

  // --------------------------------------------------------------------------
  // Register arrays: separate INT and FP (same structure, different usage)
  // --------------------------------------------------------------------------
  logic [XLEN-1:0] rf_int [PHYS_INT_REGS-1:0];
  logic [XLEN-1:0] rf_fp  [PHYS_FP_REGS-1:0];

  // --------------------------------------------------------------------------
  // Writes (from CDB writeback)
  // --------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i < PHYS_INT_REGS; i++) rf_int[i] = '0;
      for (int i = 0; i < PHYS_FP_REGS;  i++) rf_fp[i]  = '0;
    end else begin
      for (int i = 0; i < ISSUE_WIDTH; i++) begin
        if (wr_valid[i]) begin
          // High bit of address distinguishes INT (0) from FP (1)
          if (wr_addr[i][PREG_WIDTH-1] == 1'b0)
            rf_int[wr_addr[i]] <= wr_data[i];
          else
            rf_fp[wr_addr[i]]  <= wr_data[i];
        end
      end
    end
  end

  // --------------------------------------------------------------------------
  // Reads (combinational)
  // --------------------------------------------------------------------------
  always_comb begin
    for (int i = 0; i < ISSUE_WIDTH*3; i++) begin
      if (rd_addr[i][PREG_WIDTH-1] == 1'b0)
        rd_data[i] = rf_int[rd_addr[i]];
      else
        rd_data[i] = rf_fp[rd_addr[i]];
    end
  end

endmodule : phys_register_file
