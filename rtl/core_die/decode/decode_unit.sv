// ============================================================================
// Clione Processor — Decode Unit
// 8-wide decode, full instruction decode to uop, iclass assignment
// ============================================================================
`include "clione_pkg.sv"

module decode_unit
  import clione_pkg::*;
 #(
  parameter bit ENABLE_ROTATING_REGS = 1'b0,
  parameter bit AUTO_ROTATE_BUNDLES  = 1'b0
 )
(
  input  logic                        clk,
  input  logic                        rst_n,

  // From IFU — raw instruction bundle
  input  uop_t                        fetch_bundle [FETCH_WIDTH-1:0],
  input  logic                        fetch_valid,
  input  logic [TID_WIDTH-1:0]        fetch_tid,
  output logic                        fetch_ready, // back-pressure to IFU

  // To Rename — decoded uops
  output uop_t                        decode_bundle [DECODE_WIDTH-1:0],
  output logic                        decode_valid,
  output logic [TID_WIDTH-1:0]        decode_tid,
  input  logic                        decode_ready, // from rename/RAT

  // Flush (redirect)
  input  logic                        flush_valid,
  input  logic [TID_WIDTH-1:0]        flush_tid
);

  // --------------------------------------------------------------------------
  // Decode pipeline register
  // --------------------------------------------------------------------------
  uop_t      dec_buf    [DECODE_WIDTH-1:0];
  logic      dec_valid;
  logic [TID_WIDTH-1:0] dec_tid;
  logic [4:0] rot_base [SMT_WAYS-1:0];

  function automatic logic [AREG_WIDTH-1:0] map_arch_reg(
    input logic [4:0] r,
    input logic [TID_WIDTH-1:0] tid
  );
    if (ENABLE_ROTATING_REGS)
      return map_rotating_logical_reg(r, rot_base[tid]);
    else
      return AREG_WIDTH'(r);
  endfunction

  // --------------------------------------------------------------------------
  // Immediate extraction helpers
  // --------------------------------------------------------------------------
  function automatic logic [63:0] extract_imm_i(input logic [31:0] ins);
    return {{52{ins[31]}}, ins[31:20]};
  endfunction

  function automatic logic [63:0] extract_imm_s(input logic [31:0] ins);
    return {{52{ins[31]}}, ins[31:25], ins[11:7]};
  endfunction

  function automatic logic [63:0] extract_imm_b(input logic [31:0] ins);
    return {{51{ins[31]}}, ins[31], ins[7], ins[30:25], ins[11:8], 1'b0};
  endfunction

  function automatic logic [63:0] extract_imm_u(input logic [31:0] ins);
    return {{32{ins[31]}}, ins[31:12], 12'b0};
  endfunction

  function automatic logic [63:0] extract_imm_j(input logic [31:0] ins);
    return {{43{ins[31]}}, ins[31], ins[19:12], ins[20], ins[30:21], 1'b0};
  endfunction

  // --------------------------------------------------------------------------
  // Decoding function: raw instruction → fully-decoded uop
  // --------------------------------------------------------------------------
  function automatic uop_t do_decode(input logic [31:0] raw, input logic [63:0] pc,
                                      input logic [TID_WIDTH-1:0] tid);
    automatic uop_t u;
    automatic logic [6:0] op;
    automatic logic [2:0] fn3;
    automatic logic [6:0] fn7;
    automatic logic [7:0] sb_op;
    u          = '0;
    u.raw_instr = raw;
    u.pc        = pc;
    u.tid       = tid;
    u.valid     = 1'b1;
    u.mode      = MODE_DRAGONET;
    u.op_width  = OPW_64;

    op  = raw[6:0];
    fn3 = raw[14:12];
    fn7 = raw[31:25];
    sb_op = seabird_opcode(raw);

    u.arch_rd  = map_arch_reg(raw[11:7], tid);
    u.arch_rs1 = map_arch_reg(raw[19:15], tid);
    u.arch_rs2 = map_arch_reg(raw[24:20], tid);
    u.arch_rs3 = map_arch_reg(raw[31:27], tid); // FP-only field

    u.opcode = opcode_e'(op);

    // SeaBird compatibility path (encapsulated in extended opcode namespace)
    if (is_seabird_ext(raw)) begin
      u.mode      = seabird_mode(raw);
      u.op_width  = seabird_width(raw);
      u.iclass    = seabird_iclass(sb_op);
      u.is_load   = seabird_is_load(sb_op);
      u.is_store  = seabird_is_store(sb_op);
      u.is_branch = seabird_is_branch(sb_op);
      u.has_imm   = seabird_has_imm(raw);
      u.imm       = extract_imm_i(raw);

      // Branch/store/system-like instructions without architectural destination.
      if (u.is_branch || u.is_store ||
          ((sb_op >= 8'h70) && (sb_op <= 8'h96))) begin
        u.arch_rd = '0;
      end

      // Canonical width for explicit memory width opcodes.
      if (u.is_load || u.is_store)
        u.op_width = seabird_mem_width(sb_op);

      // Compatibility translation for backend clusters that decode canonical
      // internal opcodes from `raw_instr`.
      // Keep ALU/branch ops in SeaBird-extended form for native ALU handling.
      unique case (u.iclass)
        ICLASS_MEM: begin
          u.raw_instr[6:0]   = u.is_store ? OP_STORE : OP_LOAD;
          u.raw_instr[14:12] = {~u.mem_signed, u.mem_size};
        end
        ICLASS_SYS: begin
          u.raw_instr[6:0]   = OP_SYSTEM;
        end
        ICLASS_VEC: begin
          u.raw_instr[6:0] = OP_VEC;
          unique case (sb_op)
            8'h97: u.raw_instr[14:12] = 3'b000; // VADD
            8'h98: u.raw_instr[14:12] = 3'b001; // VSUB
            8'h99: u.raw_instr[14:12] = 3'b010; // VMUL
            8'h9B: u.raw_instr[14:12] = 3'b110; // VAND
            8'h9C: u.raw_instr[14:12] = 3'b111; // VOR
            8'h9D: u.raw_instr[14:12] = 3'b110; // VXOR (mapped to AND/OR class fallback)
            8'h9E,
            8'h9F: u.raw_instr[14:12] = 3'b101; // VSHL/VSHR
            default: u.raw_instr[14:12] = 3'b000;
          endcase
          unique case (u.op_width)
            OPW_8:  u.raw_instr[26:25] = 2'b00;
            OPW_16: u.raw_instr[26:25] = 2'b01;
            OPW_32: u.raw_instr[26:25] = 2'b10;
            default:u.raw_instr[26:25] = 2'b11;
          endcase
        end
        ICLASS_FP: begin
          // FPU pipeline currently keys primarily on opcode domain.
          u.raw_instr[6:0] = OP_FP;
          unique case (sb_op)
            8'hA6: u.raw_instr[31:25] = 7'b0000000; // FADD
            8'hA7: u.raw_instr[31:25] = 7'b0000100; // FSUB
            8'hA8: u.raw_instr[31:25] = 7'b0001000; // FMUL
            8'hA9: u.raw_instr[31:25] = 7'b0001100; // FDIV
            8'hAA: u.raw_instr[31:25] = 7'b0101100; // FSQRT
            8'hAB: u.raw_instr[31:25] = 7'b1010000; // FCMP
            8'hAC,
            8'hAD: u.raw_instr[31:25] = 7'b1100000; // FCVTI/FCVTS
            8'hAE,
            8'hAF: u.raw_instr[31:25] = 7'b0010000; // FNEG/FABS
            default: u.raw_instr[31:25] = 7'b0000000;
          endcase
        end
        default: begin
          // Keep SeaBird-extended transport for ALU/BRANCH paths.
        end
      endcase

      // Signedness for loads: LDB/LDH/LDW signed by default, LDQ natural width.
      if (u.is_load) begin
        u.mem_signed = (u.op_width != OPW_64);
        unique case (u.op_width)
          OPW_8:  u.mem_size = 2'b00;
          OPW_16: u.mem_size = 2'b01;
          OPW_32: u.mem_size = 2'b10;
          default:u.mem_size = 2'b11;
        endcase
      end

      return u;
    end

    unique case (op)
      OP_LUI: begin
        u.iclass  = ICLASS_INT;
        u.imm     = extract_imm_u(raw);
        u.has_imm = 1'b1;
      end
      OP_AUIPC: begin
        u.iclass  = ICLASS_INT;
        u.imm     = extract_imm_u(raw);
        u.has_imm = 1'b1;
      end
      OP_JAL: begin
        u.iclass    = ICLASS_BRANCH;
        u.is_branch = 1'b1;
        u.imm       = extract_imm_j(raw);
        u.has_imm   = 1'b1;
      end
      OP_JALR: begin
        u.iclass    = ICLASS_BRANCH;
        u.is_branch = 1'b1;
        u.imm       = extract_imm_i(raw);
        u.has_imm   = 1'b1;
      end
      OP_BRANCH: begin
        u.iclass    = ICLASS_BRANCH;
        u.is_branch = 1'b1;
        u.imm       = extract_imm_b(raw);
        u.has_imm   = 1'b1;
        // No destination register for branches
        u.arch_rd   = '0;
      end
      OP_LOAD: begin
        u.iclass    = ICLASS_MEM;
        u.is_load   = 1'b1;
        u.imm       = extract_imm_i(raw);
        u.has_imm   = 1'b1;
        u.mem_size  = fn3[1:0];
        u.mem_signed= ~fn3[2];
        unique case (fn3[1:0])
          2'b00: u.op_width = OPW_8;
          2'b01: u.op_width = OPW_16;
          2'b10: u.op_width = OPW_32;
          default: u.op_width = OPW_64;
        endcase
      end
      OP_STORE: begin
        u.iclass    = ICLASS_MEM;
        u.is_store  = 1'b1;
        u.imm       = extract_imm_s(raw);
        u.has_imm   = 1'b1;
        u.mem_size  = fn3[1:0];
        u.arch_rd   = '0;  // No destination
        unique case (fn3[1:0])
          2'b00: u.op_width = OPW_8;
          2'b01: u.op_width = OPW_16;
          2'b10: u.op_width = OPW_32;
          default: u.op_width = OPW_64;
        endcase
      end
      OP_IMM,
      OP_IMM64: begin
        u.iclass  = ICLASS_INT;
        u.imm     = extract_imm_i(raw);
        u.has_imm = 1'b1;
        if (op == OP_IMM64) u.op_width = OPW_32;
      end
      OP_REG,
      OP_REG64: begin
        u.iclass = (fn7[0]) ? ICLASS_MUL : ICLASS_INT;
        if (op == OP_REG64) u.op_width = OPW_32;
      end
      OP_FP,
      OP_FMADD, OP_FMSUB,
      OP_FNMSUB, OP_FNMADD: begin
        u.iclass = ICLASS_FP;
      end
      OP_VEC: begin
        u.iclass = ICLASS_VEC;
      end
      OP_CRYPTO: begin
        u.iclass = ICLASS_CRYPTO;
      end
      OP_SYSTEM: begin
        u.iclass = ICLASS_SYS;
      end
      default: begin
        u.exception = 1'b1;
        u.exc_code  = 6'd2; // Illegal instruction
      end
    endcase

    return u;
  endfunction

  // --------------------------------------------------------------------------
  // Decode Stage — combinational decode all FETCH_WIDTH instructions
  // --------------------------------------------------------------------------
  uop_t dec_out [DECODE_WIDTH-1:0];

  always_comb begin
    fetch_ready = decode_ready; // direct flow-through for now (no interlock)
    for (int i = 0; i < DECODE_WIDTH; i++) begin
      if (fetch_valid && fetch_bundle[i].valid &&
          !(flush_valid && flush_tid == fetch_tid)) begin
        dec_out[i] = do_decode(fetch_bundle[i].raw_instr,
                               fetch_bundle[i].pc,
                               fetch_bundle[i].tid);
        // Preserve BP prediction from IFU
        dec_out[i].is_branch          = fetch_bundle[i].is_branch;
        dec_out[i].branch_pred_taken  = fetch_bundle[i].branch_pred_taken;
        dec_out[i].branch_target      = fetch_bundle[i].branch_target;
        dec_out[i].bundle_id          = fetch_bundle[i].bundle_id;
        dec_out[i].slot_idx           = fetch_bundle[i].slot_idx;
        dec_out[i].bundle_start       = fetch_bundle[i].bundle_start;
        dec_out[i].vliw_mode          = fetch_bundle[i].vliw_mode;
      end else begin
        dec_out[i] = '0;
      end
    end
  end

  // --------------------------------------------------------------------------
  // Pipeline Register: fetch→decode boundary
  // --------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      dec_valid <= 1'b0;
      dec_tid   <= '0;
      for (int t = 0; t < SMT_WAYS; t++)
        rot_base[t] <= '0;
      for (int i = 0; i < DECODE_WIDTH; i++)
        dec_buf[i] <= '0;
    end else if (flush_valid) begin
      dec_valid <= 1'b0;
      for (int i = 0; i < DECODE_WIDTH; i++)
        dec_buf[i].valid <= 1'b0;
    end else if (decode_ready) begin
      dec_valid <= fetch_valid;
      dec_tid   <= fetch_tid;
      for (int i = 0; i < DECODE_WIDTH; i++)
        dec_buf[i] <= dec_out[i];

      if (ENABLE_ROTATING_REGS && AUTO_ROTATE_BUNDLES && fetch_valid)
        rot_base[fetch_tid] <= 5'((int'(rot_base[fetch_tid]) + 1)
                                  % ROTATING_LOGICAL_REGS);
    end
  end

  // --------------------------------------------------------------------------
  // Outputs
  // --------------------------------------------------------------------------
  assign decode_bundle = dec_buf;
  assign decode_valid  = dec_valid;
  assign decode_tid    = dec_tid;

endmodule : decode_unit
