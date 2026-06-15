#!/usr/bin/env bash
# A/B micro-benchmark for the headless "boxes" scene.
#
# Compares a frozen BASELINE build against the CURRENT build by running the
# boxes_headless driver interleaved (A,B,A,B,...) so the *ratio* stays stable
# even when CPU frequency scaling / turbo make absolute wall-clock noisy. It
# first gates on a bit-for-bit checksum match (correctness must be preserved),
# then reports min/median elapsed for each side and the speedup.
#
# Usage: scripts/ab_boxes.sh [dim] [steps] [reps] [core]
#   dim    box grid dimension (dim^3 boxes), default 8
#   steps  simulation steps,                 default 2000
#   reps   interleaved repetitions,          default 7
#   core   CPU core to pin to,               default 4
#
# Env: BASELINE_DIR (default /tmp/dart_baseline), OPT_DIR (current build).
set -u

DIM=${1:-8}
STEPS=${2:-2000}
REPS=${3:-7}
CORE=${4:-4}

BASELINE_DIR=${BASELINE_DIR:-/tmp/dart_baseline}
OPT_DIR=${OPT_DIR:-build/perf/Release/tests/benchmark/integration}

BASE_BIN="$BASELINE_DIR/boxes_headless"
OPT_BIN="$OPT_DIR/boxes_headless"

if [[ ! -x "$BASE_BIN" ]]; then echo "missing baseline: $BASE_BIN" >&2; exit 2; fi
if [[ ! -x "$OPT_BIN" ]]; then echo "missing current:  $OPT_BIN" >&2; exit 2; fi

run_base() { LD_LIBRARY_PATH="$BASELINE_DIR" taskset -c "$CORE" "$BASE_BIN" "$@"; }
run_opt()  { taskset -c "$CORE" "$OPT_BIN" "$@"; }

echo "== correctness gate (dim=$DIM steps=$STEPS) =="
bsum=$(run_base "$DIM" "$STEPS" "$STEPS" 2>/dev/null | grep '^step')
osum=$(run_opt  "$DIM" "$STEPS" "$STEPS" 2>/dev/null | grep '^step')
if [[ "$bsum" == "$osum" ]]; then
  echo "CHECKSUM OK (results bit-for-bit identical)"
else
  echo "CHECKSUM MISMATCH:"; diff <(echo "$bsum") <(echo "$osum"); echo "ABORTING"; exit 1
fi

elapsed() { sed -n 's/^elapsed_ms \([0-9.]*\).*/\1/p'; }
median() { sort -n | awk '{a[NR]=$1} END{print (NR%2)?a[(NR+1)/2]:(a[NR/2]+a[NR/2+1])/2}'; }
min()    { sort -n | head -1; }

echo "== timing: $REPS interleaved reps (dim=$DIM steps=$STEPS core=$CORE) =="
bvals=""; ovals=""
for ((i=1;i<=REPS;i++)); do
  b=$(run_base "$DIM" "$STEPS" 0 2>/dev/null | elapsed)
  o=$(run_opt  "$DIM" "$STEPS" 0 2>/dev/null | elapsed)
  bvals+="$b"$'\n'; ovals+="$o"$'\n'
  printf "  rep %2d  baseline %9s ms   optimized %9s ms   ratio %.4f\n" \
    "$i" "$b" "$o" "$(awk -v a="$b" -v c="$o" 'BEGIN{print (c>0)?a/c:0}')"
done

bmin=$(echo "$bvals" | grep . | min);  omin=$(echo "$ovals" | grep . | min)
bmed=$(echo "$bvals" | grep . | median); omed=$(echo "$ovals" | grep . | median)
echo "-----------------------------------------------------------------"
printf "baseline   min %9s ms   median %9s ms\n" "$bmin" "$bmed"
printf "optimized  min %9s ms   median %9s ms\n" "$omin" "$omed"
awk -v bm="$bmin" -v om="$omin" -v bd="$bmed" -v od="$omed" 'BEGIN{
  printf "speedup    min-based %.3fx (%.1f%%)   median-based %.3fx (%.1f%%)\n",
    bm/om, 100*(bm-om)/bm, bd/od, 100*(bd-od)/bd }'
