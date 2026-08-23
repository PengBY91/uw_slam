#!/usr/bin/env bash
# Wall-clock/iteration/cost/ATE comparison between gauss_newton_v1 (the
# current default) and ceres_v1 (the benchmark-decision-gate candidate) —
# the data docs/superpowers/specs/2026-08-23-solver-and-mapping-oss-adoption.md
# §5.1/§9 says is missing before "should the default solver change?" can be
# revisited. Not a pass/fail gate: prints a comparison table and exits 0 as
# long as both binaries ran, regardless of which backend looks better or
# whether either converged (non-convergence at the 1000-keyframe stress
# scale is itself a data point, not a script failure — see
# configs/experiment/synthetic_stress.yaml's gates.require_converged: false).
#
# Runs three pairs, skipping any whose inputs aren't available:
#   1. synthetic_smoke   (12 keyframes)  — the existing demo scenario
#   2. synthetic_stress  (1000 keyframes) — the scale point this benchmark
#      exists for
#   3. real_holoocean_vo (50 keyframes, real HoloOcean recording) — only run
#      if --real-bag is passed (or the well-known /tmp path from
#      project memory exists); real-data correctness cross-check per the
#      design doc's §1.2.1/§6.1.1, not a replacement for #2's scale point.
#
# Usage:
#   tools/bench/solver_benchmark.sh [--build-dir DIR] [--out-dir DIR] [--real-bag PATH]
#
# Requires a build configured with -DUW_BUILD_CERES_SOLVER=ON
# (cmake -S . -B build -DUW_BUILD_CERES_SOLVER=ON ...).
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

BUILD_DIR="build"
OUT_DIR="${TMPDIR:-/tmp}/uw_slam_solver_benchmark/$(date +%Y%m%d_%H%M%S)"
REAL_BAG="/tmp/real_session_depth_fixed.mcap"  # well-known path, see project memory

while [ $# -gt 0 ]; do
  case "$1" in
    --build-dir) BUILD_DIR="$2"; shift 2 ;;
    --out-dir) OUT_DIR="$2"; shift 2 ;;
    --real-bag) REAL_BAG="$2"; shift 2 ;;
    -h|--help) sed -n '2,25p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 1 ;;
  esac
done

REPLAY_DEMO="$BUILD_DIR/bin/replay_demo"
SYNTH_BAG_GEN="$BUILD_DIR/bin/synth_bag_gen"
if [ ! -x "$REPLAY_DEMO" ] || [ ! -x "$SYNTH_BAG_GEN" ]; then
  echo "error: $REPLAY_DEMO / $SYNTH_BAG_GEN not found — build first (see README.md)" >&2
  exit 1
fi
mkdir -p "$OUT_DIR"
printf '%-16s %-14s %8s %6s %14s %20s %10s\n' \
  SCENARIO BACKEND MS ITERS FINAL_COST "ATE_RMSE_M" CONVERGED
RESULTS_TABLE="$OUT_DIR/results.txt"
{
  printf '%-16s %-14s %8s %6s %14s %20s %10s\n' \
    SCENARIO BACKEND MS ITERS FINAL_COST "ATE_RMSE_M" CONVERGED
} > "$RESULTS_TABLE"

# run_one <scenario_label> <bag> <experiment_yaml> <backend_label> [extra replay_demo args...]
run_one() {
  local scenario_label="$1" bag="$2" experiment="$3" backend_label="$4"
  shift 4
  local log="$OUT_DIR/${scenario_label}_${backend_label}.log"
  local prefix="$OUT_DIR/${scenario_label}_${backend_label}"
  local start_ns end_ns elapsed_ms
  start_ns=$(date +%s%N)
  "$REPLAY_DEMO" --bag "$bag" --experiment "$experiment" --out "$prefix" "$@" > "$log" 2>&1
  local status=$?
  end_ns=$(date +%s%N)
  elapsed_ms=$(( (end_ns - start_ns) / 1000000 ))

  local iters cost converged ate
  iters=$(grep -oP '(?<=: )\d+(?= iterations)' "$log" | head -1)
  cost=$(grep -oP '(?<=-> )[0-9.eE+-]+(?= \()' "$log" | head -1)
  converged=$(grep -q '(converged)' "$log" && echo yes || echo no)
  ate=$(grep -oP '(?<=rmse=)[0-9.eE+-]+' "$log" | head -1)
  [ -z "$iters" ] && iters="-"
  [ -z "$cost" ] && cost="-"
  [ -z "$ate" ] && ate="-"
  if [ "$status" -ne 0 ]; then
    echo "  ! $scenario_label/$backend_label exited $status (see $log)" >&2
    converged="ERROR"
  fi

  printf '%-16s %-14s %8s %6s %14s %20s %10s\n' \
    "$scenario_label" "$backend_label" "$elapsed_ms" "$iters" "$cost" "$ate" "$converged" | tee -a "$RESULTS_TABLE"
}

# --- Pair 1: synthetic_smoke (12 keyframes) ---
SMOKE_BAG="$OUT_DIR/synthetic_smoke.mcap"
"$SYNTH_BAG_GEN" --experiment configs/experiment/synthetic_smoke.yaml --out "$SMOKE_BAG" > "$OUT_DIR/synthetic_smoke_gen.log" 2>&1
run_one synthetic_smoke "$SMOKE_BAG" configs/experiment/synthetic_smoke.yaml gauss_newton_v1
run_one synthetic_smoke "$SMOKE_BAG" configs/experiment/synthetic_smoke_ceres.yaml ceres_v1

# --- Pair 2: synthetic_stress (1000 keyframes) — the scale point this
# benchmark exists for. Both backends read the SAME bag, generated once from
# the gauss_newton_v1 experiment file (scenario/rig/factor config is
# identical between the two experiment files — only estimation.solver
# differs — so one bag is valid for both).
#
# --max-iterations 3 (not each backend's natural default of 30): a
# calibration run showed gauss_newton_v1's dense O(N^3) LDLT factorization
# (999 free keyframes * 7 = 6993-dim normal equations) costs roughly 40s per
# LM iteration at this scale, so a full 30-iteration run (up to 8 retries
# each) would take on the order of tens of minutes — this benchmark reports
# per-iteration wall-clock and cost reduction within a fixed, small budget
# instead of waiting for either backend to fully converge, which is itself
# already the headline finding for this scale point (see the design doc's
# §9/§11.2 follow-up notes): dense linear algebra does not scale to 1000
# keyframes in interactive time, independent of which backend runs it.
STRESS_BAG="$OUT_DIR/synthetic_stress.mcap"
"$SYNTH_BAG_GEN" --experiment configs/experiment/synthetic_stress.yaml --out "$STRESS_BAG" > "$OUT_DIR/synthetic_stress_gen.log" 2>&1
run_one synthetic_stress "$STRESS_BAG" configs/experiment/synthetic_stress.yaml gauss_newton_v1 --max-iterations 3
run_one synthetic_stress "$STRESS_BAG" configs/experiment/synthetic_stress_ceres.yaml ceres_v1 --max-iterations 3

# --- Pair 3: real_holoocean_vo (50 keyframes, real HoloOcean recording) —
# only if the bag is available; not part of the standard synthetic-only
# test suite, see the design doc's §1.2.1 for why this is a correctness
# cross-check, not a scale benchmark.
if [ -f "$REAL_BAG" ]; then
  run_one real_holoocean_vo "$REAL_BAG" configs/experiment/real_holoocean_vo.yaml gauss_newton_v1 --align-ate
  run_one real_holoocean_vo "$REAL_BAG" configs/experiment/real_holoocean_vo_ceres.yaml ceres_v1 --align-ate
else
  echo "  (skipping real_holoocean_vo pair: $REAL_BAG not found)" | tee -a "$RESULTS_TABLE"
fi

echo ""
echo "full logs and trajectories: $OUT_DIR"
echo "results table:              $RESULTS_TABLE"
