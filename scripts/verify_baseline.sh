#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="$REPO_ROOT/sim/build/verification"
mkdir -p "$LOG_DIR"

run_and_log() {
  local target="$1"
  local log="$LOG_DIR/${target}_verilator.log"
  echo "[RUN] $target (verilator)"
  (cd "$REPO_ROOT" && ./scripts/sim.sh "$target" verilator) >"$log" 2>&1
  echo "[OK ] log: $log"
}

check_no_fatal_markers() {
  local log="$1"
  if grep -E "%Error|\*\*\*FAILED|WATCHDOG|Assertion" "$log" >/dev/null; then
    echo "[FAIL] fatal marker found in $log"
    grep -nE "%Error|\*\*\*FAILED|WATCHDOG|Assertion" "$log" | head -n 20
    exit 1
  fi
}

check_no_critical_warnings() {
  local log="$1"
  # These classes are treated as functional/hardware-risk warnings.
  if grep -E '^%Warning-(LATCH|MULTIDRIVEN|SELRANGE|UNDRIVEN|SYNCASYNCNET|WIDTHEXPAND|WIDTHTRUNC)' "$log" >/dev/null; then
    echo "[FAIL] critical warning class found in $log"
    grep -nE '^%Warning-(LATCH|MULTIDRIVEN|SELRANGE|UNDRIVEN|SYNCASYNCNET|WIDTHEXPAND|WIDTHTRUNC)' "$log" | head -n 30
    exit 1
  fi
}

check_marker() {
  local log="$1"
  local regex="$2"
  local label="$3"
  if grep -E "$regex" "$log" >/dev/null; then
    echo "[PASS] $label"
  else
    echo "[FAIL] missing marker for $label"
    echo "       log: $log"
    exit 1
  fi
}

run_and_log alu
run_and_log core
run_and_log full

ALU_LOG="$LOG_DIR/alu_verilator.log"
CORE_LOG="$LOG_DIR/core_verilator.log"
FULL_LOG="$LOG_DIR/full_verilator.log"

check_no_fatal_markers "$ALU_LOG"
check_no_fatal_markers "$CORE_LOG"
check_no_fatal_markers "$FULL_LOG"
check_no_critical_warnings "$ALU_LOG"
check_no_critical_warnings "$CORE_LOG"
check_no_critical_warnings "$FULL_LOG"

check_marker "$ALU_LOG" "PASS:" "ALU functional checks"
check_marker "$CORE_LOG" "PASS: simulation completed.*errors=0" "Core smoke regression"
check_marker "$FULL_LOG" "Full-system simulation complete" "Full-system integration regression"

echo ""
echo "Verification baseline PASSED."
echo "Logs: $LOG_DIR"
