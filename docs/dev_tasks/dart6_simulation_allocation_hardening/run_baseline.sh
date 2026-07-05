#!/usr/bin/env bash
# WP-D6M.2 baseline capture for the DART 6 simulation-loop allocation
# hardening task. Run from the repo root on a quiet machine (no concurrent
# builds). Re-run identically on the improved tree for the comparison packet.
set -euo pipefail

LABEL="${1:?usage: run_baseline.sh <label: baseline|improved>}"
ROOT="$(git rev-parse --show-toplevel)"
BIN="$ROOT/build/default/cpp/Release/bin"
OUT="$ROOT/docs/dev_tasks/dart6_simulation_allocation_hardening/$LABEL"
mkdir -p "$OUT"

{
  echo "date: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "commit: $(git rev-parse HEAD)"
  if git diff --quiet --ignore-submodules -- && git diff --cached --quiet --ignore-submodules --; then
    echo "worktree_dirty: no"
  else
    echo "worktree_dirty: yes"
    echo "diff_stat:"
    git diff --stat | sed 's/^/  /'
  fi
  echo "host: $(uname -srmo)"
  echo "cpu: $(grep -m1 'model name' /proc/cpuinfo | cut -d: -f2 | xargs)"
  echo "cores: $(nproc)"
  echo "governor: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo unknown)"
} > "$OUT/host.txt"

for exe in empty boxes contact_container; do
  "$BIN/BM_INTEGRATION_$exe" \
    --benchmark_repetitions=5 \
    --benchmark_report_aggregates_only=true \
    --benchmark_out="$OUT/bm_$exe.json" \
    --benchmark_out_format=json \
    2>&1 | tee "$OUT/bm_$exe.log"
done

# Determinism oracle: state checksums every 500 steps for bit-exactness
# comparison across the runtime changes (timing in this log is not evidence).
"$ROOT/build/default/cpp/Release/tests/benchmark/integration/boxes_headless" \
  8 2000 500 2>&1 | tee "$OUT/boxes_headless.log"

echo "Baseline artifacts written to $OUT"
