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

endpackage : clione_pkg

`endif // CLIONE_PKG_SV
