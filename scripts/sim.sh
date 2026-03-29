#!/usr/bin/env bash
# =============================================================================
# Clione Architecture — Simulation Script
# Supported simulators: Verilator, Icarus Verilog, VCS, Xcelium
# Usage: ./sim.sh [TARGET] [SIMULATOR]
#   TARGET     : core | alu | fpu | simd | crypto | l2 | full (default: all)
#   SIMULATOR  : verilator | iverilog | vcs | xcelium (default: verilator)
# =============================================================================
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RTL_DIR="$REPO_ROOT/rtl"
SIM_DIR="$REPO_ROOT/sim"
TB_DIR="$SIM_DIR/tb"
BUILD_DIR="$SIM_DIR/build"
WAVE_DIR="$SIM_DIR/waves"

TARGET="${1:-all}"
SIM="${2:-verilator}"

mkdir -p "$BUILD_DIR" "$WAVE_DIR"

# Common RTL file lists
PKG_FILES=(
  "$RTL_DIR/pkg/clione_pkg.sv"
)

CORE_FILES=(
  "${PKG_FILES[@]}"
  "$RTL_DIR/core_die/fetch/branch_predictor.sv"
  "$RTL_DIR/core_die/fetch/ifu.sv"
  "$RTL_DIR/core_die/decode/decode_unit.sv"
  "$RTL_DIR/core_die/decode/register_alias_table.sv"
  "$RTL_DIR/core_die/decode/issue_queue.sv"
  "$RTL_DIR/core_die/backend/reorder_buffer.sv"
  "$RTL_DIR/core_die/backend/phys_register_file.sv"
  "$RTL_DIR/core_die/scheduler/clustered_ooo_scheduler.sv"
  "$RTL_DIR/core_die/cache/l1i_cache.sv"
  "$RTL_DIR/core_die/cache/l1d_cache.sv"
  "$RTL_DIR/core_die/lsu/load_store_unit.sv"
  "$RTL_DIR/core_die/core_die_top.sv"
)

ALU_FILES=(
  "${PKG_FILES[@]}"
  "$RTL_DIR/execution_chiplets/alu/alu_pipe.sv"
  "$RTL_DIR/execution_chiplets/alu/alu_chiplet_top.sv"
)

FPU_FILES=(
  "${PKG_FILES[@]}"
  "$RTL_DIR/execution_chiplets/fpu/fpu_pipe.sv"
  "$RTL_DIR/execution_chiplets/fpu/fpu_chiplet_top.sv"
)

EXEC_FILES=(
  "${ALU_FILES[@]}"
  "$RTL_DIR/execution_chiplets/fpu/fpu_pipe.sv"
  "$RTL_DIR/execution_chiplets/fpu/fpu_chiplet_top.sv"
  "$RTL_DIR/execution_chiplets/simd/simd_chiplet_top.sv"
  "$RTL_DIR/execution_chiplets/crypto/crypto_chiplet_top.sv"
)

CACHE_FILES=(
  "${PKG_FILES[@]}"
  "$RTL_DIR/cache_chiplets/l2_cache_chiplet.sv"
  "$RTL_DIR/cache_chiplets/l3_cache_chiplet.sv"
)

NOC_FILES=(
  "${PKG_FILES[@]}"
  "$RTL_DIR/interconnect/noc_router.sv"
  "$RTL_DIR/interconnect/noc_fabric_top.sv"
)

FULL_FILES=(
  "${CORE_FILES[@]}"
  "${ALU_FILES[@]}"
  "$RTL_DIR/execution_chiplets/fpu/fpu_pipe.sv"
  "$RTL_DIR/execution_chiplets/fpu/fpu_chiplet_top.sv"
  "$RTL_DIR/execution_chiplets/simd/simd_chiplet_top.sv"
  "$RTL_DIR/execution_chiplets/crypto/crypto_chiplet_top.sv"
  "${CACHE_FILES[@]}"
  "${NOC_FILES[@]}"
  "$RTL_DIR/memory_io/mem_ctrl_chiplet.sv"
  "$RTL_DIR/memory_io/io_chiplet.sv"
)

run_verilator() {
  local top="$1"
  local tb="$2"
  shift 2
  local files=("$@")

  echo "=== Verilating $top ==="
  verilator --sv --binary --trace \
    --timing \
    -Wall -Wno-fatal -Wno-UNOPTFLAT -Wno-BLKANDNBLK \
    --top-module "$top" \
    -I"$RTL_DIR/pkg" \
    "${files[@]}" \
    "$tb" \
    --Mdir "$BUILD_DIR/${top}" \
    -o "$BUILD_DIR/${top}/sim_${top}"

  echo "=== Running $top ==="
  "$BUILD_DIR/${top}/sim_${top}" --trace \
    2>&1 | tee "$BUILD_DIR/${top}/sim.log" | tail -20
}

run_iverilog() {
  local top="$1"
  local tb="$2"
  shift 2
  local files=("$@")

  echo "=== Compiling $top with Icarus Verilog ==="
  iverilog -g2012 \
    -I"$RTL_DIR/pkg" \
    -o "$BUILD_DIR/${top}.vvp" \
    "${files[@]}" "$tb"

  echo "=== Simulating $top ==="
  cd "$WAVE_DIR"
  vvp "$BUILD_DIR/${top}.vvp" 2>&1 | tee "$BUILD_DIR/${top}.log" | tail -30
  cd - > /dev/null
}

run_vcs() {
  local top="$1"
  local tb="$2"
  shift 2
  local files=("$@")

  echo "=== Compiling $top with VCS ==="
  vcs -sverilog \
    -timescale=1ns/1ps \
    +incdir+"$RTL_DIR/pkg" \
    -o "$BUILD_DIR/${top}_vcs" \
    "${files[@]}" "$tb"

  echo "=== Simulating $top ==="
  "$BUILD_DIR/${top}_vcs" -ucli -do "run; exit" \
    2>&1 | tee "$BUILD_DIR/${top}_vcs.log" | tail -30
}

sim_target() {
  local name="$1"
  local top="$2"
  local tb="$3"
  shift 3
  local files=("$@")

  echo ""
  echo "############################################################"
  echo "# Simulating: $name"
  echo "############################################################"

  case "$SIM" in
    verilator)  run_verilator "$top" "$tb" "${files[@]}" ;;
    iverilog)   run_iverilog  "$top" "$tb" "${files[@]}" ;;
    vcs)        run_vcs       "$top" "$tb" "${files[@]}" ;;
    *)
      echo "Unknown simulator: $SIM. Use: verilator | iverilog | vcs"
      exit 1 ;;
  esac
}

# --------------------------------------------------------------------------
# Target selection
# --------------------------------------------------------------------------
case "$TARGET" in
  core)
    sim_target "Core Die" "tb_core_die_top" \
      "$TB_DIR/tb_core_die_top.sv" "${CORE_FILES[@]}"
    ;;
  alu)
    sim_target "ALU Chiplet" "tb_alu_chiplet" \
      "$TB_DIR/tb_alu_chiplet.sv" "${ALU_FILES[@]}"
    ;;
  fpu)
    sim_target "FPU Chiplet" "tb_fpu_chiplet" \
      "$TB_DIR/tb_fpu_chiplet.sv" "${FPU_FILES[@]}"
    ;;
  full)
    sim_target "Full System" "tb_full_system" \
      "$TB_DIR/tb_full_system.sv" "${FULL_FILES[@]}"
    ;;
  all)
    sim_target "ALU Chiplet" "tb_alu_chiplet" \
      "$TB_DIR/tb_alu_chiplet.sv" "${ALU_FILES[@]}"
    sim_target "Core Die" "tb_core_die_top" \
      "$TB_DIR/tb_core_die_top.sv" "${CORE_FILES[@]}"
    sim_target "Full System" "tb_full_system" \
      "$TB_DIR/tb_full_system.sv" "${FULL_FILES[@]}"
    ;;
  *)
    echo "Unknown target: $TARGET. Use: core | alu | fpu | full | all"
    exit 1 ;;
esac

echo ""
echo "All done. Logs in $BUILD_DIR, waves in $WAVE_DIR"
