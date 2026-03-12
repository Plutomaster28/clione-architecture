#!/usr/bin/env bash
# =============================================================================
# Clione Architecture — OpenLane Flow Runner
# Runs synthesis, placement, routing, and GDSII generation for each chiplet
# Usage: ./run_flow.sh [CHIPLET] [STEP]
#   CHIPLET : all | core_die | alu_chiplet | fpu_chiplet | simd_chiplet |
#             crypto_chiplet | l2_cache_chiplet | l3_cache_chiplet |
#             mem_ctrl_chiplet | io_chiplet | noc_fabric
#   STEP    : all | synth | floorplan | place | cts | route | lvs | gds
# =============================================================================
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OL_DIR="$REPO_ROOT/openlane"
RESULTS_DIR="$REPO_ROOT/results"

CHIPLET="${1:-all}"
STEP="${2:-all}"

CHIPLETS=(
  core_die
  alu_chiplet
  fpu_chiplet
  simd_chiplet
  crypto_chiplet
  l2_cache_chiplet
  l3_cache_chiplet
  mem_ctrl_chiplet
  io_chiplet
  noc_fabric
)

check_openlane() {
  if ! command -v flow.tcl &>/dev/null; then
    # Try Docker-based OpenLane
    if command -v docker &>/dev/null; then
      echo "[INFO] OpenLane flow.tcl not found in PATH. Using Docker container."
      export USE_DOCKER=1
    else
      echo "[ERROR] Neither OpenLane flow.tcl nor Docker found."
      echo "  Install OpenLane: https://github.com/The-OpenROAD-Project/OpenLane"
      exit 1
    fi
  fi
}

run_chiplet() {
  local name="$1"
  local cfg_dir="$OL_DIR/$name"
  local out_dir="$RESULTS_DIR/$name"

  if [ ! -f "$cfg_dir/config.json" ]; then
    echo "[WARN] No config.json found for $name, skipping."
    return
  fi

  mkdir -p "$out_dir"
  echo ""
  echo "============================================================"
  echo " Running OpenLane for: $name"
  echo " Config: $cfg_dir/config.json"
  echo "============================================================"

  if [ "${USE_DOCKER:-0}" = "1" ]; then
    docker run --rm \
      -v "$REPO_ROOT:$REPO_ROOT" \
      -w "$REPO_ROOT" \
      efabless/openlane:latest \
      flow.tcl -design "$cfg_dir" -tag "$name" -overwrite \
        -run_steps "$STEP"
  else
    flow.tcl \
      -design "$cfg_dir" \
      -tag "$name" \
      -overwrite \
      -run_steps "$STEP" \
      2>&1 | tee "$out_dir/flow.log"
  fi

  local exit_code=$?
  if [ $exit_code -eq 0 ]; then
    echo "[PASS] $name completed successfully."
    # Copy key outputs
    if [ -d "$cfg_dir/runs/$name" ]; then
      cp -r "$cfg_dir/runs/$name/reports" "$out_dir/" 2>/dev/null || true
      cp    "$cfg_dir/runs/$name/results/final/gds/${name}.gds" "$out_dir/" 2>/dev/null || true
      echo "  GDS: $out_dir/${name}.gds"
      echo "  Reports: $out_dir/reports/"
    fi
  else
    echo "[FAIL] $name failed with exit code $exit_code."
    echo "  Log: $out_dir/flow.log"
  fi
}

check_openlane

case "$CHIPLET" in
  all)
    for c in "${CHIPLETS[@]}"; do
      run_chiplet "$c"
    done
    ;;
  *)
    if [[ " ${CHIPLETS[*]} " =~ " ${CHIPLET} " ]]; then
      run_chiplet "$CHIPLET"
    else
      echo "Unknown chiplet: $CHIPLET"
      echo "Valid: ${CHIPLETS[*]}"
      exit 1
    fi
    ;;
esac

echo ""
echo "Flow complete. Results in: $RESULTS_DIR"
