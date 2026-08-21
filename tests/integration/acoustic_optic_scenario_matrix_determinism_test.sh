#!/usr/bin/env bash
# L2: same seed/config -> byte-identical scenario-matrix report (plan 5),
# EXCEPT p95_latency_ms — that field is a real wall-clock measurement and
# will legitimately differ run-to-run on any real machine; stripping it is
# not hiding nondeterminism, it's excluding the one field that was never
# meant to be deterministic. Deliberately does NOT round-trip through MCAP
# (see plan 5's file header for why) — this proves the pipeline itself
# (RNG usage, iteration order, floating-point formatting) is deterministic,
# the same property determinism_test.sh proves for the pose-graph replay
# path.
set -euo pipefail

MATRIX_BIN="$1"
EXPERIMENT_CONFIG="$2"
WORKDIR="$(mktemp -d)"
trap 'rm -rf "$WORKDIR"' EXIT

strip_latency() {
  sed -E 's/p95_latency_ms=[0-9.]+/p95_latency_ms=<omitted>/'
}

# The matrix binary's own exit code reflects its reduced MVP gate set (see
# plan 5), which is orthogonal to whether its output is reproducible —
# don't let `set -e` abort this determinism check on a real gate failure.
"$MATRIX_BIN" --experiment "$EXPERIMENT_CONFIG" --seed 4242 --trials-per-scenario 5 > "$WORKDIR/raw1.txt" 2>&1 || true
"$MATRIX_BIN" --experiment "$EXPERIMENT_CONFIG" --seed 4242 --trials-per-scenario 5 > "$WORKDIR/raw2.txt" 2>&1 || true
strip_latency < "$WORKDIR/raw1.txt" > "$WORKDIR/run1.txt"
strip_latency < "$WORKDIR/raw2.txt" > "$WORKDIR/run2.txt"

if ! diff -q "$WORKDIR/run1.txt" "$WORKDIR/run2.txt" >/dev/null; then
  echo "FAIL: acoustic_optic_scenario_matrix produced different output on two runs of the same seed"
  diff "$WORKDIR/run1.txt" "$WORKDIR/run2.txt" || true
  exit 1
fi
echo "OK: deterministic scenario-matrix report confirmed (latency field excluded)"
