# Clione Processor Architecture

A 250mm² chiplet-based, throughput-optimized out-of-order processor targeting OpenLane-based VLSI implementation.

---

## Design Philosophy

Clione prioritizes **instruction throughput over raw clock frequency**. The architecture achieves this through:

- **8-wide superscalar** decode, dispatch, and retire
- **4-thread SMT** with per-thread branch predictors and GHR
- **Clustered OoO dispatch** routing uops to execution chiplets via a packet-switched NoC
- **Speculative chiplet dispatch** — operands are forwarded before commit; misspeculation flushes in-flight chiplet work
- **256-entry ROB** and **320 physical registers** per class — deep speculation window

---

## Chiplet Map

```
┌─────────────────────────────────────────────────────────────┐
│                     4×4 NoC Mesh (512-bit, XY routing)      │
│                                                             │
│  (0,0) Core Die     (1,0) ALU-0    (2,0) ALU-1  (3,0) ALU-2│
│  (0,1) FPU-0        (1,1) FPU-1    (2,1) SIMD-0 (3,1) SIMD-1│
│  (0,2) Crypto       (1,2) L2-0     (2,2) L2-1   (3,2) L2-2 │
│  (0,3) L3-LLC       (1,3) IO       (2,3) MemCtrl (3,3) L2-3 │
└─────────────────────────────────────────────────────────────┘
```

| Chiplet | Node ID | Die Area (est.) | Description |
|---|---|---|---|
| Core Die | 0x00 | ~80mm² | IFU, BP, Decode, RAT, ROB, PRF, Scheduler, L1I/D, LSU |
| ALU × 3 | 0x01–0x03 | ~15mm² each | 4× ALU pipes, integer/branch ops |
| FPU × 2 | 0x04–0x05 | ~20mm² each | 2× FPU pipes, IEEE-754 double, FMA |
| SIMD × 2 | 0x06–0x07 | ~25mm² each | 512-bit vectors, int8/16/32/64, dot-product |
| Crypto | 0x08 | ~10mm² | AES-256 rounds, SHA-256/512 |
| L2 × 4 | 0x09–0x0C | ~10mm² each | 4MB per chiplet, 16-way, MESI |
| L3 LLC | 0x18 | ~30mm² | 32MB banked, 32-way, coherency directory |
| MemCtrl | 0x1F | ~15mm² | 4-ch DDR5-6400, refresh, prefetch |
| IO | 0x1E | ~15mm² | PCIe Gen5×16, DMA-32ch, UART/SPI/I2C/GPIO |
| NoC Fabric | — | ~20mm² | 4×4 mesh, wormhole routing, 5 VCs |

**Total estimated: ~250mm²**

---

## ISA

Custom 64-bit ISA (`clione64`) based on RISC-V encoding:

- SeaBird compatibility layer is carried via encapsulated opcodes in the decode path.
- Register windowing policy is **invisible/microarchitectural**: software-visible
	architectural mapping stays flat (`r0..r31`) and does not use SPARC-style
	window-control ISA semantics.

| Opcode Class | Encoding | Notes |
|---|---|---|
| OP_INT (R-type) | 7'b0110011 | Full RV64I integer |
| OP_IMM (I-type) | 7'b0010011 | Immediate arithmetic |
| LOAD / STORE | 7'b0000011 / 7'b0100011 | Byte to doubleword |
| BRANCH | 7'b1100011 | BEQ/BNE/BLT/BGE/BLTU/BGEU |
| JAL / JALR | 7'b1101111 / 7'b1100111 | Jump |
| LUI / AUIPC | 7'b0110111 / 7'b0010111 | Upper immediate |
| OP_FP | 7'b1010011 | IEEE-754 FP (FMA, FSQRT, FDIV) |
| OP_VEC | 7'b1010111 | 512-bit SIMD vectors |
| OP_CRYPTO | 7'b0001011 | AES-256 / SHA-256/512 |
| SYSTEM | 7'b1110011 | ECALL, CSR (future) |

---

## Microarchitecture

### Frontend (Core Die)
- **IFU**: 8-wide fetch, round-robin SMT thread selection, 64KB 4-way L1I-cache (PLRU, 8 MSHR)
- **Branch Predictor**: TAGE-style with bimodal base (4K entries × 2-bit), 4-way BTB (4K sets), 64-entry RAS per thread, per-thread GHR (128-bit history)

### Decode & Rename
- **Decode Unit**: 8-wide decode, full immediate extraction (I/S/B/U/J), iclass assignment  
- **RAT**: Separate INT/FP rat tables per SMT thread, intra-bundle forwarding, checkpoint/restore on misprediction
- **Issue Queues**: Parameterized per execution class (INT/FP/VEC/MEM), CDB wakeup, oldest-first selection

### Backend
- **ROB**: 256-entry circular buffer, 8-wide retire, mispredict redirect with full flush
- **Physical RF**: 320 INT + 320 FP physical registers, 24 read ports × 8 write ports
- **LSU**: 48-entry LQ + 48-entry SQ, address disambiguation, store-to-load forwarding, 64KB 8-way L1D (MESI, 8 MSHR)
- **Scheduler**: Least-loaded cluster selection, packs operands into 512-bit NoC packets before dispatch

### Execution Chiplets
- **ALU** (×3): 4× parallel 4-stage integer pipes (ADD/SUB/SHL/SLT/XOR/SRA/OR/AND/MUL/DIV/REM), branch evaluation
- **FPU** (×2): 2× 6-stage IEEE-754 double-precision pipes (FMA, FADD, FMUL, FSUB, FMIN/MAX)
- **SIMD** (×2): 512-bit vector register file (64 regs), 5-stage pipeline, element ops (8/16/32/64-bit), dot-product reduction
- **Crypto** (×1): AES-256 SubBytes/ShiftRows/MixColumns/AddRoundKey, S-Box LUT, key schedule

### Cache Hierarchy
| Level | Size | Ways | Type | Protocol |
|---|---|---|---|---|
| L1I | 64KB | 4-way | Per-core | Private, PIPT |
| L1D | 64KB | 8-way | Per-core | MESI, WB/WA |
| L2 | 4MB × 4 | 16-way | Per-cluster | MESI, directory |
| L3 | 32MB | 32-way | Shared | Banked (16), directory |

### NoC
- 4×4 mesh, XY deterministic routing
- **512-bit** data payload per packet
- **5 virtual channels** for deadlock avoidance (Dispatch / Result / Load-Req / Load-Resp / Snoop)
- Credit-based flow control, wormhole switching
- 8 packet types: `NOC_DISPATCH`, `NOC_RESULT`, `NOC_LOAD_REQ`, `NOC_LOAD_RESP`, `NOC_STORE_REQ`, `NOC_FLUSH`, `NOC_SNOOP`, `NOC_ACK`

---

## Repository Layout

```
clione-architecture/
├── rtl/
│   ├── pkg/               clione_pkg.sv — global parameters, types, structs
│   ├── core_die/
│   │   ├── fetch/         ifu.sv, branch_predictor.sv
│   │   ├── decode/        decode_unit.sv, register_alias_table.sv, issue_queue.sv
│   │   ├── backend/       reorder_buffer.sv, phys_register_file.sv
│   │   ├── scheduler/     clustered_ooo_scheduler.sv
│   │   ├── cache/         l1i_cache.sv, l1d_cache.sv
│   │   ├── lsu/           load_store_unit.sv
│   │   └── core_die_top.sv
│   ├── execution_chiplets/
│   │   ├── alu/           alu_pipe.sv, alu_chiplet_top.sv
│   │   ├── fpu/           fpu_pipe.sv, fpu_chiplet_top.sv
│   │   ├── simd/          simd_chiplet_top.sv
│   │   └── crypto/        crypto_chiplet_top.sv
│   ├── cache_chiplets/    l2_cache_chiplet.sv, l3_cache_chiplet.sv
│   ├── memory_io/         mem_ctrl_chiplet.sv, io_chiplet.sv
│   └── interconnect/      noc_router.sv, noc_fabric_top.sv
├── openlane/
│   ├── core_die/          config.json, constraints.sdc
│   ├── alu_chiplet/       config.json
│   ├── fpu_chiplet/       config.json
│   ├── simd_chiplet/      config.json
│   ├── crypto_chiplet/    config.json
│   ├── l2_cache_chiplet/  config.json
│   ├── l3_cache_chiplet/  config.json
│   ├── mem_ctrl_chiplet/  config.json
│   ├── io_chiplet/        config.json
│   └── noc_fabric/        config.json
├── sim/
│   └── tb/
│       ├── tb_core_die_top.sv
│       ├── tb_alu_chiplet.sv
│       └── tb_full_system.sv
├── scripts/
│   ├── sim.sh             Simulation runner (Verilator / Icarus / VCS)
│   └── run_flow.sh        OpenLane flow runner per chiplet
└── docs/                  (this file)
```

---

## Quick Start

### Simulate (Icarus Verilog)

```bash
# Install Icarus if needed
sudo apt install iverilog

# Run ALU chiplet unit test
./scripts/sim.sh alu iverilog

# Run core-die smoke test
./scripts/sim.sh core iverilog

# Run full system
./scripts/sim.sh full iverilog
```

### Simulate (Verilator)

```bash
# Install Verilator ≥ 5.0
# https://verilator.org/guide/latest/install.html

./scripts/sim.sh alu verilator
./scripts/sim.sh all verilator

# One-command baseline confidence gate (ALU + Core + Full)
./scripts/verify_baseline.sh
```

### OpenLane VLSI Flow

```bash
# Install OpenLane 2.x (Docker recommended)
# https://github.com/The-OpenROAD-Project/OpenLane

# Run flow for one chiplet
./scripts/run_flow.sh alu_chiplet

# Run all chiplets sequentially
./scripts/run_flow.sh all
```

---

## Key Parameters (`clione_pkg.sv`)

| Parameter | Value | Description |
|---|---|---|
| `XLEN` | 64 | Data width |
| `FETCH_WIDTH` | 8 | Instructions fetched per cycle |
| `DECODE_WIDTH` | 8 | Instructions decoded per cycle |
| `ISSUE_WIDTH` | 8 | Max uops issued per cycle |
| `RETIRE_WIDTH` | 8 | Max retirements per cycle |
| `SMT_WAYS` | 4 | Simultaneous multithreading threads |
| `ROB_DEPTH` | 256 | Reorder buffer entries |
| `PHYS_INT_REGS` | 320 | Physical integer registers |
| `PHYS_FP_REGS` | 320 | Physical FP registers |
| `ARCH_INT_REGS` | 64 | Architectural integer registers in RTL package |
| `LOGICAL_INT_REGS` | 32 | Compiler/ISA-visible logical integer registers |
| `CACHE_LINE_BYTES` | 64 | Cache line size |
| `NOC_DATA_WIDTH` | 512 | NoC packet payload width |
| `NOC_NODE_BITS` | 5 | Node ID bits (up to 32 nodes) |

---

## Known Simplifications / Future Work

- **DTLB/ITLB**: Currently identity-mapped stubs (`paddr = vaddr`). Full TLB + page-table walker needed for OS support.
- **TAGE predictor**: Tagged tables T1–T5 are declared; bimodal base is the active predictor. TAGE update path needs wiring.
- **AES inverse**: AES decryption round (`InvSubBytes`, `InvShiftRows`, `InvMixColumns`) not yet implemented in crypto chiplet.
- **Store-to-load forwarding**: LSU address computation is complete; byte-granule forwarding MUX needs full disambiguation logic.
- **ROB checkpoint**: `ckpt_rat_int/fp` are structurally exported from ROB but actual snapshot storage is in the RAT module.
- **FP SQRT/DIV**: Stubbed in `fpu_pipe.sv`; iterative or Newton-Raphson Implementation TBD.
- **Coherency**: L1D–L2 snoop protocol is structurally complete in RTL; cross-chiplet GETS/GETX sequences need verification.
- **Power gating**: Per-chiplet clock/power domains not yet modeled; OpenLane PDN configs use default ring.
- **Multi-socket**: Architecture currently single-socket; NUMA extension planned.

---

## License

This project is open hardware. RTL and scripts are released under the Apache 2.0 License.
