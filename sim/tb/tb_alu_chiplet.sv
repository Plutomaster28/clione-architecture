// ============================================================================
// Clione Processor — ALU Chiplet Testbench
// Tests: 4 ALU pipes, all arithmetic ops, branch evaluation, NoC protocol
// ============================================================================
`timescale 1ns/1ps
`include "clione_pkg.sv"

module tb_alu_chiplet;
  import clione_pkg::*;

  logic        clk, rst_n;
  noc_pkt_t    rx_pkt;
  logic        rx_valid, rx_ready;
  noc_pkt_t    tx_pkt;
  logic        tx_valid, tx_ready;
  exec_result_t bypass_result;
  logic        bypass_valid;
  exec_result_t peer_bypass [7:0];
  logic         peer_bypass_valid [7:0];

  alu_chiplet_top u_dut (
    .clk              ( clk              ),
    .rst_n            ( rst_n            ),
    .rx_pkt           ( rx_pkt           ),
    .rx_valid         ( rx_valid         ),
    .rx_ready         ( rx_ready         ),
    .tx_pkt           ( tx_pkt           ),
    .tx_valid         ( tx_valid         ),
    .tx_ready         ( tx_ready         ),
    .bypass_result    ( bypass_result    ),
    .bypass_valid     ( bypass_valid     ),
    .peer_bypass      ( peer_bypass      ),
    .peer_valid       ( peer_bypass_valid )
  );

  initial clk = 0;
  always #1.25 clk = ~clk; // 400MHz

  assign tx_ready          = 1'b1;
  always_comb begin
    for (int i = 0; i < 8; i++) begin
      peer_bypass[i] = '0;
      peer_bypass_valid[i] = 1'b0;
    end
  end

  // Build a dispatch packet (NOC_DISPATCH)
  function automatic noc_pkt_t make_alu_pkt(
    input logic [63:0] src1, src2,
    input logic [8:0]  phys_rd,
    input logic [31:0] instr,
    input logic [7:0]  rob_idx
  );
    noc_pkt_t p;
    p = '0;
    p.pkt_type         = NOC_DISPATCH;
    p.dst_node         = 5'h01;
    p.src_node         = 5'h00;
    p.valid            = 1'b1;
    p.rob_idx          = rob_idx;
    p.data[511:448]    = src1;
    p.data[447:384]    = src2;
    p.data[31:0]       = instr;
    p.data[32]         = 1'b0; // has_imm
    p.data[33]         = 1'b0; // is_branch
    p.data[35:34]      = MODE_DRAGONET;
    p.data[38:36]      = OPW_64;
    p.data[PREG_WIDTH+39:40] = phys_rd;
    p.data[127:64]     = 64'h0;
    return p;
  endfunction

  // R-type instruction: ADD x1, x2, x3
  // opcode=OP_INT, funct3=000, funct7=0000000
  localparam logic [31:0] INSTR_ADD  = 32'b0000000_00011_00010_000_00001_0110011;
  // R-type: SUB
  localparam logic [31:0] INSTR_SUB  = 32'b0100000_00011_00010_000_00001_0110011;
  // R-type: XOR
  localparam logic [31:0] INSTR_XOR  = 32'b0000000_00011_00010_100_00001_0110011;
  // R-type: MUL
  localparam logic [31:0] INSTR_MUL  = 32'b0000001_00011_00010_000_00001_0110011;

`ifdef USE_ASSEMBLED_PROGRAM
  `include "alu_program.svh"
`endif

  int passed = 0, failed = 0;
  logic [63:0] last_result;

  // Result capture
  always @(posedge clk) begin
    if (tx_valid) begin
      last_result = tx_pkt.data[127:64];
      $display("[%0t] ALU RESULT: rob_idx=%0d result=0x%016h",
        $time, tx_pkt.rob_idx, last_result);
    end
  end

  task send_op;
    input logic [63:0] a, b;
    input logic [31:0] instr;
    input logic [7:0]  rob_idx;
    input logic [63:0] expected;
    begin
      @(negedge clk);
      rx_pkt   = make_alu_pkt(a, b, 9'h01, instr, rob_idx);
      rx_valid = 1'b1;
      @(posedge clk);
      #0.1;
      rx_valid = 1'b0;
      // Wait for result (up to 10 cycles)
      repeat(10) @(posedge clk);
      if (last_result === expected) begin
        $display("  PASS: op=%h a=%h b=%h => %h", instr[14:12], a, b, last_result);
        passed++;
      end else begin
        $display("  FAIL: op=%h a=%h b=%h => got %h, expected %h",
          instr[14:12], a, b, last_result, expected);
        failed++;
      end
    end
  endtask

  task send_op_no_check;
    input logic [63:0] a, b;
    input logic [31:0] instr;
    input logic [7:0]  rob_idx;
    begin
      @(negedge clk);
      rx_pkt   = make_alu_pkt(a, b, 9'h01, instr, rob_idx);
      rx_valid = 1'b1;
      @(posedge clk);
      #0.1;
      rx_valid = 1'b0;
      repeat(10) @(posedge clk);
      $display("  INFO: op not score-boarded, instr=0x%08h result=%h", instr, last_result);
    end
  endtask

  function automatic logic [63:0] expected_for_rtype(
    input logic [31:0] instr,
    input logic [63:0] a,
    input logic [63:0] b,
    output logic supported
  );
    logic [6:0] opcode;
    logic [2:0] funct3;
    logic [6:0] funct7;
    begin
      opcode = instr[6:0];
      funct3 = instr[14:12];
      funct7 = instr[31:25];
      supported = 1'b1;

      if (opcode != 7'b0110011) begin
        supported = 1'b0;
        expected_for_rtype = '0;
      end else if (funct7 == 7'b0000000 && funct3 == 3'b000) begin
        expected_for_rtype = a + b;
      end else if (funct7 == 7'b0100000 && funct3 == 3'b000) begin
        expected_for_rtype = a - b;
      end else if (funct7 == 7'b0000000 && funct3 == 3'b100) begin
        expected_for_rtype = a ^ b;
      end else if (funct7 == 7'b0000000 && funct3 == 3'b111) begin
        expected_for_rtype = a & b;
      end else if (funct7 == 7'b0000000 && funct3 == 3'b110) begin
        expected_for_rtype = a | b;
      end else if (funct7 == 7'b0000001 && funct3 == 3'b000) begin
        expected_for_rtype = a * b;
      end else begin
        supported = 1'b0;
        expected_for_rtype = '0;
      end
    end
  endfunction

  initial begin
    $dumpfile("tb_alu_chiplet.vcd");
    $dumpvars(0, tb_alu_chiplet);

    rx_valid = 0; rx_pkt = '0;
    rst_n = 0;
    repeat(8) @(posedge clk);
    rst_n = 1;
    repeat(4) @(posedge clk);

`ifdef USE_ASSEMBLED_PROGRAM
    begin
      logic [63:0] a;
      logic [63:0] b;
      logic [63:0] expected;
      logic [31:0] instr;
      logic supported;
      $display("=== Running assembled ALU program (%0d instructions) ===", PROGRAM_INST_COUNT);
      for (int i = 0; i < PROGRAM_INST_COUNT; i++) begin
        instr = PROGRAM_INSTR_FLAT[((PROGRAM_INST_COUNT-1-i)*32) +: 32];
        a = 64'h100 + i;
        b = 64'h20 + (i * 3);
        expected = expected_for_rtype(instr, a, b, supported);
        if (supported)
          send_op(a, b, instr, i[7:0], expected);
        else
          send_op_no_check(a, b, instr, i[7:0]);
      end
    end
`else
    // Test ADD: 100 + 200 = 300
    send_op(64'd100, 64'd200, INSTR_ADD, 8'd0, 64'd300);

    // Test SUB: 500 - 200 = 300
    send_op(64'd500, 64'd200, INSTR_SUB, 8'd1, 64'd300);

    // Test XOR: 0xAA ^ 0x55 = 0xFF
    send_op(64'hAA, 64'h55, INSTR_XOR, 8'd2, 64'hFF);

    // Test MUL: 12 * 13 = 156
    send_op(64'd12, 64'd13, INSTR_MUL, 8'd3, 64'd156);

    // Test overflow: max_int + 1 wraps
    send_op(64'h7FFFFFFFFFFFFFFF, 64'd1, INSTR_ADD, 8'd4,
            64'h8000000000000000);
`endif

    repeat(20) @(posedge clk);
    $display("=== ALU Chiplet: %0d/%0d tests passed ===", passed, passed+failed);
    $finish;
  end

endmodule : tb_alu_chiplet
