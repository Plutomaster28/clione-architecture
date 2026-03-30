# Clione Bring-up Assembler Flow

This flow is the shortest path to chip testing without building a full compiler.

## What this gives you

- A machine-readable ISA table at `tools/isa/clione64_isa.json`.
- A strict encoder at `tools/assemble_clione64.py`.
- Repeatable `.hex`, `.bin`, or SystemVerilog constants from assembly.

Important:
- This profile matches current RTL decode behavior (Clione64 / RV-style fields).
- It is **not** the full SeaBird textual ISA surface yet.
- For SeaBird boot-file bring-up, use the bridge profile at
  `tools/isa/seabird_boot_profile.json`.

## Assemble a sample program

```bash
python3 tools/assemble_clione64.py \
  tools/examples/alu_smoke.s \
  -o sim/programs/alu_smoke.hex \
  --spec tools/isa/clione64_isa.json \
  --format hex
```

Generate SV constants instead:

```bash
python3 tools/assemble_clione64.py \
  tools/examples/alu_smoke.s \
  -o sim/programs/alu_smoke.svh \
  --spec tools/isa/clione64_isa.json \
  --format sv
```

## Supported syntax (current)

- Labels: `label:`
- Registers: `R0..R31` or `X0..X31`
- Immediates: decimal/hex/binary and `#` prefix (`#42`, `#0x2a`, `0b1010`)
- R-format: `ADD R1, R2, R3`
- I-format: `ADDI R1, R2, #4`
- Loads: `LD R1, [R2+16]`, `LW R1, [R2]`, `LD R1, [0x1000]`
- Stores: `SD [R2+16], R1`, `SW [R2], R1`
- Branches: `BEQ R1, R2, label`
- Directives: `.org`, `.align`, `.word`, `.dword`, `.qword`, `.space`

## Bring-up recommendation

1. Use this assembler to generate deterministic instruction vectors.
2. Feed generated words into ALU/core testbenches.
3. Expand instruction coverage in `tools/isa/clione64_isa.json` as RTL grows.
4. Add SeaBird-surface aliases only after encoding/RTL behavior is locked.

## Direct ALU replay flow

You can run ALU simulation with an assembled program in one command:

```bash
./scripts/sim.sh alu verilator tools/examples/alu_smoke.s
```

What happens:

1. `sim.sh` assembles the input ASM into:
  - `sim/programs/alu_program.hex`
  - `sim/programs/alu_program.svh`
2. `sim.sh` compiles ALU testbench with `USE_ASSEMBLED_PROGRAM`.
3. `tb_alu_chiplet.sv` replays `PROGRAM_INSTR[]` from `alu_program.svh`.

If you omit `PROGRAM_ASM`, ALU testbench runs its built-in directed checks.

Core die replay:

```bash
./scripts/sim.sh core verilator tools/examples/core_smoke.s
```

Full-system replay:

```bash
./scripts/sim.sh full verilator tools/examples/core_smoke.s
```

Boot-profile replay (SeaBird syntax accepted by bridge lowering):

```bash
./scripts/sim.sh core verilator firmware_and_bios/clione_control.asm tools/isa/seabird_boot_profile.json
```

In `core`/`full` modes, testbenches preload the assembled instruction stream
into core L1I sets after reset, then enable SMT threads for fetch.

## Boot-stub compatibility check

To track progress toward running the SeaBird boot stub end-to-end, run:

```bash
python3 tools/lint_seabird_boot_compat.py firmware_and_bios/clione_control.asm
```

Profile-aware lint (recommended for boot bring-up):

```bash
python3 tools/lint_seabird_boot_compat.py \
  firmware_and_bios/clione_control.asm \
  --spec tools/isa/seabird_boot_profile.json
```

This reports mnemonics still unsupported by the current Clione64 bring-up
assembler profile so you can prioritize extension work.
