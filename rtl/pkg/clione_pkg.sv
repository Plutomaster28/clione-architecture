// ============================================================================
// Clione Processor — Global Package
// 64-bit chiplet-based OoO superscalar architecture
// Target: ~250mm² multi-die system
// ============================================================================
`ifndef CLIONE_PKG_SV
`define CLIONE_PKG_SV

package clione_pkg;

  // --------------------------------------------------------------------------
  // Global Width Parameters
  // --------------------------------------------------------------------------
  parameter int unsigned XLEN            = 64;    // Arch register width
  parameter int unsigned ILEN            = 32;    // Instruction width
  parameter int unsigned PADDR_WIDTH     = 52;    // Physical address bits
  parameter int unsigned VADDR_WIDTH     = 57;    // Virtual address bits (5-level paging)

  // --------------------------------------------------------------------------
  // Superscalar / SMT Parameters
  // --------------------------------------------------------------------------
  parameter int unsigned FETCH_WIDTH     = 8;    // Instructions fetched per cycle
  parameter int unsigned DECODE_WIDTH    = 8;    // Decode / rename width
  parameter int unsigned ISSUE_WIDTH     = 8;    // Maximum issue slots per cycle
  parameter int unsigned RETIRE_WIDTH    = 8;    // Retire width per cycle
  parameter int unsigned SMT_WAYS        = 4;    // Simultaneous multithreading threads
  parameter int unsigned TID_WIDTH       = $clog2(SMT_WAYS);

  // --------------------------------------------------------------------------
  // VLIW Frontend Parameters
  // --------------------------------------------------------------------------
  parameter int unsigned VLIW_SLOTS       = 8;
  parameter int unsigned VLIW_SLOT_BITS   = 32;
  parameter int unsigned VLIW_BUNDLE_BITS = VLIW_SLOTS * VLIW_SLOT_BITS; // 256

  // Logical ISA register model used by frontend/compiler contract.
  // R0..R7 fixed, R8..R31 rotating.
  parameter int unsigned LOGICAL_INT_REGS       = 32;
  parameter int unsigned FIXED_LOGICAL_REGS     = 8;
  parameter int unsigned ROTATING_LOGICAL_REGS  = LOGICAL_INT_REGS - FIXED_LOGICAL_REGS;

  // --------------------------------------------------------------------------
  // Register File
  // --------------------------------------------------------------------------
  parameter int unsigned ARCH_INT_REGS   = 64;   // Architectural integer registers
  parameter int unsigned ARCH_FP_REGS    = 64;   // Architectural FP/SIMD registers
  parameter int unsigned PHYS_INT_REGS   = 320;  // Physical integer registers
  parameter int unsigned PHYS_FP_REGS    = 320;  // Physical FP registers
  parameter int unsigned PREG_WIDTH      = $clog2(PHYS_INT_REGS);  // 9 bits
  parameter int unsigned AREG_WIDTH      = $clog2(ARCH_INT_REGS);  // 6 bits

  // --------------------------------------------------------------------------
  // Reorder Buffer
  // --------------------------------------------------------------------------
  parameter int unsigned ROB_DEPTH       = 256;
  parameter int unsigned ROB_PTR_WIDTH   = $clog2(ROB_DEPTH);   // 8 bits

  // --------------------------------------------------------------------------
  // Issue Queues (per cluster type)
  // --------------------------------------------------------------------------
  parameter int unsigned IQ_INT_DEPTH    = 32;   // Integer cluster issue queue
  parameter int unsigned IQ_FP_DEPTH     = 32;   // FP cluster issue queue
  parameter int unsigned IQ_VEC_DEPTH    = 32;   // Vector/SIMD cluster issue queue
  parameter int unsigned IQ_MEM_DEPTH    = 32;   // Load/store unit issue queue

  // --------------------------------------------------------------------------
  // Branch Predictor (TAGE-like)
  // --------------------------------------------------------------------------
  parameter int unsigned BTB_ENTRIES     = 4096;
  parameter int unsigned TAGE_TABLES     = 5;
  parameter int unsigned TAGE_HIST_LEN   = 512;  // Max global history length
  parameter int unsigned RAS_DEPTH       = 64;   // Return address stack

  // --------------------------------------------------------------------------
  // L1 Cache (on Control Die)
  // --------------------------------------------------------------------------
  parameter int unsigned L1I_SIZE_KB     = 64;
  parameter int unsigned L1I_WAYS        = 4;
  parameter int unsigned L1D_SIZE_KB     = 64;
  parameter int unsigned L1D_WAYS        = 8;
  parameter int unsigned CACHE_LINE_BITS = 512;  // 64 bytes
  parameter int unsigned CACHE_LINE_BYTES= CACHE_LINE_BITS / 8;

  // --------------------------------------------------------------------------
  // Chiplet Cluster IDs
  // --------------------------------------------------------------------------
  typedef enum logic [3:0] {
    CLUSTER_ALU0   = 4'h0,
    CLUSTER_ALU1   = 4'h1,
    CLUSTER_ALU2   = 4'h2,
    CLUSTER_ALU3   = 4'h3,
    CLUSTER_FPU0   = 4'h4,
    CLUSTER_FPU1   = 4'h5,
    CLUSTER_SIMD0  = 4'h6,
    CLUSTER_SIMD1  = 4'h7,
    CLUSTER_CRYPTO = 4'h8,
    CLUSTER_LSU    = 4'h9,
    CLUSTER_NONE   = 4'hF
  } cluster_id_e;

  // --------------------------------------------------------------------------
  // Instruction Classes (for scheduler routing)
  // --------------------------------------------------------------------------
  typedef enum logic [2:0] {
    ICLASS_INT    = 3'b000,  // Integer ALU
    ICLASS_MUL    = 3'b001,  // Integer multiply/divide
    ICLASS_FP     = 3'b010,  // Floating-point
    ICLASS_VEC    = 3'b011,  // Vector / SIMD
    ICLASS_MEM    = 3'b100,  // Load / store
    ICLASS_BRANCH = 3'b101,  // Branch / jump
    ICLASS_SYS    = 3'b110,  // System / CSR
    ICLASS_CRYPTO = 3'b111   // Cryptographic
  } iclass_e;

  // --------------------------------------------------------------------------
  // SeaBird Compatibility Modes
  // Native datapath is 64-bit; lower-width modes are represented by masks.
  // --------------------------------------------------------------------------
  typedef enum logic [1:0] {
    MODE_CLOWNFISH = 2'b00, // 16-bit
    MODE_TETRA     = 2'b01, // 32-bit
    MODE_DRAGONET  = 2'b10, // 64-bit
    MODE_DROPLET   = 2'b11  // 128-bit (emulated via pair/combine on 64-bit backend)
  } seabird_mode_e;

  typedef enum logic [2:0] {
    OPW_8  = 3'b000,
    OPW_16 = 3'b001,
    OPW_32 = 3'b010,
    OPW_64 = 3'b011,
    OPW_128= 3'b100
  } op_width_e;

  // --------------------------------------------------------------------------
  // Clione64 Opcode Encoding
  // --------------------------------------------------------------------------
  typedef enum logic [6:0] {
    OP_LUI    = 7'b0110111,
    OP_AUIPC  = 7'b0010111,
    OP_JAL    = 7'b1101111,
    OP_JALR   = 7'b1100111,
    OP_BRANCH = 7'b1100011,
    OP_LOAD   = 7'b0000011,
    OP_STORE  = 7'b0100011,
    OP_IMM    = 7'b0010011,
    OP_REG    = 7'b0110011,
    OP_IMM64  = 7'b0011011,
    OP_REG64  = 7'b0111011,
    OP_FP     = 7'b1010011,
    OP_VEC    = 7'b1010111,
    OP_SYSTEM = 7'b1110011,
    OP_CRYPTO = 7'b0001011,
    OP_FMADD  = 7'b1000011,
    OP_FMSUB  = 7'b1000111,
    OP_FNMSUB = 7'b1001011,
    OP_FNMADD = 7'b1001111
  } opcode_e;

  // --------------------------------------------------------------------------
  // Decoded Micro-Op (uop)
  // --------------------------------------------------------------------------
  typedef struct packed {
    logic [ILEN-1:0]         raw_instr;    // Original encoded instruction
    logic [15:0]             bundle_id;    // 8-slot bundle sequence ID
    logic [2:0]              slot_idx;     // 0..7 slot in bundle
    logic                    bundle_start; // slot 0 marker
    logic                    vliw_mode;    // fixed-size bundle decode path
    opcode_e                  opcode;
    iclass_e                  iclass;
    logic [AREG_WIDTH-1:0]   arch_rs1;
    logic [AREG_WIDTH-1:0]   arch_rs2;
    logic [AREG_WIDTH-1:0]   arch_rs3;    // FMA third source
    logic [AREG_WIDTH-1:0]   arch_rd;
    logic [PREG_WIDTH-1:0]   phys_rs1;
    logic [PREG_WIDTH-1:0]   phys_rs2;
    logic [PREG_WIDTH-1:0]   phys_rs3;
    logic [PREG_WIDTH-1:0]   phys_rd;
    logic [PREG_WIDTH-1:0]   phys_rd_old; // For ROB rollback
    logic [63:0]              imm;         // Sign-extended immediate
    seabird_mode_e            mode;        // Architectural mode for this uop
    op_width_e                op_width;    // Intended operation width
    logic                     has_imm;
    logic                     rs1_ready;
    logic                     rs2_ready;
    logic                     rs3_ready;
    logic [ROB_PTR_WIDTH-1:0] rob_idx;
    logic [TID_WIDTH-1:0]     tid;         // SMT thread ID
    logic [63:0]              pc;
    logic                     is_branch;
    logic                     branch_pred_taken;
    logic [63:0]              branch_target;
    logic                     is_load;
    logic                     is_store;
    logic [1:0]               mem_size;    // 00=byte,01=half,10=word,11=dword
    logic                     mem_signed;
    logic                     valid;
    logic                     exception;
    logic [5:0]               exc_code;
  } uop_t;

  // --------------------------------------------------------------------------
  // ROB Entry
  // --------------------------------------------------------------------------
  typedef struct packed {
    logic [63:0]              pc;
    logic [ROB_PTR_WIDTH-1:0] rob_idx;
    logic [PREG_WIDTH-1:0]    phys_rd;
    logic [PREG_WIDTH-1:0]    phys_rd_old;
    logic [AREG_WIDTH-1:0]    arch_rd;
    logic [TID_WIDTH-1:0]     tid;
    logic                     complete;
    logic                     exception;
    logic [5:0]               exc_code;
    logic                     is_store;
    logic                     is_branch;
    logic                     branch_mispredict;
    logic [63:0]              branch_correct_target;
    iclass_e                  iclass;
  } rob_entry_t;

  // --------------------------------------------------------------------------
  // Chiplet NoC Packet
  // --------------------------------------------------------------------------
  parameter int unsigned NOC_DATA_WIDTH  = 512;   // 64-byte data bus
  parameter int unsigned NOC_ADDR_WIDTH  = 52;
  parameter int unsigned NOC_NODE_BITS   = 5;     // Up to 32 nodes

  typedef enum logic [2:0] {
    NOC_DISPATCH  = 3'b000,   // Instruction dispatch to exec chiplet
    NOC_RESULT    = 3'b001,   // Execution result back to control die
    NOC_LOAD_REQ  = 3'b010,   // Load request
    NOC_LOAD_RESP = 3'b011,   // Load response with data
    NOC_STORE_REQ = 3'b100,   // Store request
    NOC_FLUSH     = 3'b101,   // Cache flush / invalidation
    NOC_SNOOP     = 3'b110,   // Coherency snoop
    NOC_ACK       = 3'b111    // Generic acknowledgment
  } noc_type_e;

  typedef struct packed {
    noc_type_e                    pkt_type;
    logic [NOC_NODE_BITS-1:0]     src_node;
    logic [NOC_NODE_BITS-1:0]     dst_node;
    logic [7:0]                   seq_id;      // Packet sequence for ordering
    logic [TID_WIDTH-1:0]         tid;
    logic [ROB_PTR_WIDTH-1:0]     rob_idx;
    logic [NOC_ADDR_WIDTH-1:0]    addr;
    logic [NOC_DATA_WIDTH-1:0]    data;
    logic [63:0]                  data_mask;  // Byte enables
    logic                         last;       // Last flit
    logic                         valid;
  } noc_pkt_t;

  // --------------------------------------------------------------------------
  // Execution Result (from chiplet back to control die)
  // --------------------------------------------------------------------------
  typedef struct packed {
    logic [PREG_WIDTH-1:0]   phys_rd;
    logic [XLEN-1:0]         result;
    logic [ROB_PTR_WIDTH-1:0]rob_idx;
    logic [TID_WIDTH-1:0]    tid;
    logic                    exception;
    logic [5:0]              exc_code;
    logic                    branch_taken;
    logic [63:0]             branch_target;
    logic                    valid;
  } exec_result_t;

  // --------------------------------------------------------------------------
  // Cache Coherency States (MESI)
  // --------------------------------------------------------------------------
  typedef enum logic [1:0] {
    MESI_M = 2'b11,  // Modified
    MESI_E = 2'b10,  // Exclusive
    MESI_S = 2'b01,  // Shared
    MESI_I = 2'b00   // Invalid
  } mesi_state_e;

  // --------------------------------------------------------------------------
  // Helper Functions
  // --------------------------------------------------------------------------
  function automatic logic [63:0] sign_extend_imm12(input logic [11:0] imm12);
    return {{52{imm12[11]}}, imm12};
  endfunction

  function automatic logic [63:0] sign_extend_imm20(input logic [19:0] imm20);
    return {{44{imm20[19]}}, imm20};
  endfunction

  function automatic logic [63:0] mode_mask(input seabird_mode_e mode);
    unique case (mode)
      MODE_CLOWNFISH: return 64'h0000_0000_0000_FFFF;
      MODE_TETRA:     return 64'h0000_0000_FFFF_FFFF;
      default:        return 64'hFFFF_FFFF_FFFF_FFFF;
    endcase
  endfunction

  function automatic logic [63:0] apply_mode_mask(
    input logic [63:0] v,
    input seabird_mode_e mode
  );
    return v & mode_mask(mode);
  endfunction

  function automatic logic [63:0] sign_extend_mode(
    input logic [63:0] v,
    input seabird_mode_e mode
  );
    unique case (mode)
      MODE_CLOWNFISH: return {{48{v[15]}}, v[15:0]};
      MODE_TETRA:     return {{32{v[31]}}, v[31:0]};
      default:        return v;
    endcase
  endfunction

  function automatic logic [63:0] width_mask(input op_width_e w);
    unique case (w)
      OPW_8:   return 64'h0000_0000_0000_00FF;
      OPW_16:  return 64'h0000_0000_0000_FFFF;
      OPW_32:  return 64'h0000_0000_FFFF_FFFF;
      default: return 64'hFFFF_FFFF_FFFF_FFFF;
    endcase
  endfunction

  function automatic logic [63:0] apply_width_mask(
    input logic [63:0] v,
    input op_width_e w
  );
    return v & width_mask(w);
  endfunction

  function automatic logic [63:0] sign_extend_width(
    input logic [63:0] v,
    input op_width_e w
  );
    unique case (w)
      OPW_8:   return {{56{v[7]}}, v[7:0]};
      OPW_16:  return {{48{v[15]}}, v[15:0]};
      OPW_32:  return {{32{v[31]}}, v[31:0]};
      default: return v;
    endcase
  endfunction

  // 128-bit helper (for future Droplet emulation on 64-bit backend)
  function automatic logic [127:0] combine_u128(
    input logic [63:0] lo,
    input logic [63:0] hi
  );
    return {hi, lo};
  endfunction

  function automatic logic [63:0] merge_masked64(
    input logic [63:0] old_v,
    input logic [63:0] new_v,
    input logic [63:0] mask
  );
    return (old_v & ~mask) | (new_v & mask);
  endfunction

  function automatic logic is_fp_op(input opcode_e op);
    return (op == OP_FP) || (op == OP_FMADD) || (op == OP_FMSUB) ||
           (op == OP_FNMSUB) || (op == OP_FNMADD);
  endfunction

  function automatic logic is_vec_op(input opcode_e op);
    return (op == OP_VEC);
  endfunction

  function automatic logic is_mem_op(input opcode_e op);
    return (op == OP_LOAD) || (op == OP_STORE);
  endfunction

  // --------------------------------------------------------------------------
  // SeaBird compatibility encoding helpers
  // Encapsulation format inside 32-bit transport word:
  //   [31:24] SeaBird opcode (0x00..0xAF)
  //   [23:22] SeaBird mode
  //   [21:19] SeaBird operation width override
  //   [18]    Immediate-present hint
  //   [17:0]  Operand/immediate payload (frontend-defined)
  // Escape selector: raw[6:0] == OP_CRYPTO and raw[14:12] == 3'b111
  // --------------------------------------------------------------------------
  function automatic logic is_seabird_ext(input logic [31:0] raw);
    return (raw[6:0] == OP_CRYPTO) && (raw[14:12] == 3'b111);
  endfunction

  function automatic logic [7:0] seabird_opcode(input logic [31:0] raw);
    return raw[31:24];
  endfunction

  function automatic seabird_mode_e seabird_mode(input logic [31:0] raw);
    return seabird_mode_e'(raw[23:22]);
  endfunction

  function automatic op_width_e seabird_width(input logic [31:0] raw);
    return op_width_e'(raw[21:19]);
  endfunction

  function automatic logic seabird_has_imm(input logic [31:0] raw);
    return raw[18];
  endfunction

  function automatic iclass_e seabird_iclass(input logic [7:0] sb_op);
    if (sb_op <= 8'h1F) begin
      if ((sb_op >= 8'h10) && (sb_op <= 8'h1A)) return ICLASS_MEM;
      return ICLASS_INT;
    end
    if (sb_op <= 8'h3F) begin
      if ((sb_op >= 8'h24) && (sb_op <= 8'h2F)) return ICLASS_MUL;
      return ICLASS_INT;
    end
    if (sb_op <= 8'h51) return ICLASS_INT;
    if (sb_op <= 8'h5B) return ICLASS_INT;
    if (sb_op <= 8'h71) begin
      if ((sb_op == 8'h70) || (sb_op == 8'h71)) return ICLASS_SYS;
      return ICLASS_BRANCH;
    end
    if (sb_op <= 8'h7D) return ICLASS_MEM;
    if (sb_op <= 8'h96) return ICLASS_SYS;
    if (sb_op <= 8'hA5) return ICLASS_VEC;
    return ICLASS_FP;
  endfunction

  function automatic logic seabird_is_load(input logic [7:0] sb_op);
    return (sb_op == 8'h10) || (sb_op == 8'h13) || (sb_op == 8'h14) ||
           (sb_op == 8'h15) || (sb_op == 8'h16) || (sb_op == 8'h1B) ||
           (sb_op == 8'h76) || (sb_op == 8'h78);
  endfunction

  function automatic logic seabird_is_store(input logic [7:0] sb_op);
    return (sb_op == 8'h11) || (sb_op == 8'h17) || (sb_op == 8'h18) ||
           (sb_op == 8'h19) || (sb_op == 8'h1A) || (sb_op == 8'h1C) ||
           (sb_op == 8'h77) || (sb_op == 8'h79);
  endfunction

  function automatic logic seabird_is_branch(input logic [7:0] sb_op);
    return (sb_op >= 8'h5C) && (sb_op <= 8'h6F);
  endfunction

  function automatic op_width_e seabird_mem_width(input logic [7:0] sb_op);
    unique case (sb_op)
      8'h13, 8'h17: return OPW_8;
      8'h14, 8'h18: return OPW_16;
      8'h15, 8'h19: return OPW_32;
      default:      return OPW_64;
    endcase
  endfunction

  // Logical-to-architectural mapping with rotating register window.
  // If logical register is fixed (R0..R7), mapping is identity.
  // Otherwise map R8..R31 by `rot_base` modulo 24.
  function automatic logic [AREG_WIDTH-1:0] map_rotating_logical_reg(
    input logic [4:0] logical_reg,
    input logic [4:0] rot_base
  );
    automatic logic [AREG_WIDTH-1:0] out_reg;
    automatic int unsigned rot_idx;
    if (logical_reg < FIXED_LOGICAL_REGS[4:0]) begin
      out_reg = AREG_WIDTH'(logical_reg);
    end else begin
      rot_idx = (int'(logical_reg) - FIXED_LOGICAL_REGS + int'(rot_base))
                % ROTATING_LOGICAL_REGS;
      out_reg = AREG_WIDTH'(FIXED_LOGICAL_REGS + rot_idx);
    end
    return out_reg;
  endfunction

endpackage : clione_pkg

`endif // CLIONE_PKG_SV
