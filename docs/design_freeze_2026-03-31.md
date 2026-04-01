# Clione Design Freeze Baseline (2026-03-31)

This document captures the agreed baseline after RTL stabilization and regression.

## Frozen Architectural Choices

- ISA-visible register mapping is flat and stable.
- Register windowing is invisible (microarchitectural only).
- No SPARC-style window-control ISA instructions are part of the baseline.
- Frontend bundle contract remains fixed 8 slots x 32 bits (256-bit bundle metadata path).

## Baseline RTL/Spec Lock

- Decode remap controls are disabled at integration:
  - `decode_unit.ENABLE_ROTATING_REGS = 0`
  - `decode_unit.AUTO_ROTATE_BUNDLES = 0`
- ISA JSON policy is explicit:
  - `register_windowing.model = invisible_microarchitectural`
  - `register_windowing.isa_visible = false`
  - `register_windowing.architectural_register_mapping = flat_r0_to_r31`

## Regression Status

Validated with Verilator script flow:

- `./scripts/sim.sh alu verilator` pass
- `./scripts/sim.sh core verilator` pass
- `./scripts/sim.sh full verilator` pass
- `./scripts/sim.sh all verilator` pass

## Verification Hardening Update (2026-04-01)

- Added a one-command regression gate: `./scripts/verify_baseline.sh`.
- The gate now fails on:
  - any fatal markers (`%Error`, `***FAILED`, `WATCHDOG`, `Assertion`)
  - critical warning classes (`LATCH`, `MULTIDRIVEN`, `SELRANGE`,
    `UNDRIVEN`, `SYNCASYNCNET`, `WIDTHEXPAND`, `WIDTHTRUNC`)
- Clean-rebuild baseline currently passes this gate for ALU, Core, and Full.
- Remaining warnings are mostly non-functional lint noise
  (`UNUSEDSIGNAL`, `BLKSEQ`, `TIMESCALEMOD`, `PINCONNECTEMPTY`, `UNUSEDPARAM`).

## Silicon Confidence Statement

- Current evidence supports a strong **functional RTL confidence** statement.
- This is not yet a full silicon signoff claim until timing/physical checks are complete:
  - synthesis equivalence/timing closure
  - DRC/LVS/antenna checks
  - power/IR/EM checks

## OpenLane/GDSII Status

- Per-chiplet OpenLane configs exist under `openlane/*`.
- Docker-based OpenLane execution is wired in `scripts/run_flow.sh`.
- Current known blocker: OpenLane v1/Yosys frontend compatibility with package-heavy SystemVerilog in some flows; this is a tooling/frontend issue, not a functional regression issue.

## Immediate Next Steps Toward Tapeout

1. Lock toolchain versions for reproducibility (OpenLane + PDK hash).
2. Use one chiplet (ALU) as physical closure pilot and tune floorplan/density.
3. Roll tuned constraints to other chiplets.
4. Perform per-chiplet signoff checks (timing, DRC, LVS, antenna).
5. Assemble multi-die/package integration plan from chiplet GDS outputs.
