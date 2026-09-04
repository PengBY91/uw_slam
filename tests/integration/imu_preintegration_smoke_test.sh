#!/usr/bin/env bash
# PREP-B-01 end-to-end acceptance: estimator_mode: imu_preintegration solves
# a synthetic bag to within 0.15 m using ONLY legitimate inputs.
#
# Two things are being gated here, and both matter:
#
#  1. THE ESTIMATE IS GOOD. Explicit keyframe boundaries, IMU preintegration,
#     depth and sonar range factors, converged, ATE <= 0.15 m -- with all
#     three factor kinds genuinely present, because a run that quietly built
#     zero sonar factors could still hit that ATE on depth and IMU alone and
#     would be reporting a pipeline that does not exist.
#
#  2. THE ESTIMATE IS EARNED. The same bag is regenerated with ground truth
#     deleted, with its poses displaced, and with its timestamps displaced,
#     leaving every algorithm input -- and every /gt/state message's MCAP
#     log time -- bit-identical. All four runs must produce byte-identical
#     trajectories. If any of them differs, ground truth is reaching the
#     algorithm path, and no ATE number from this pipeline means anything.
#     The converse is checked too: the evaluator, which IS allowed to read
#     ground truth, must see each tampering axis, otherwise "identical
#     trajectories" would also pass on a build where the flags did nothing.
#
# Assertions are anchored on whole `summary.<key>=` lines from
# RunReplayPipeline's machine summary, never on prose or on a loose regex
# like "imu_preintegration.*factor". Iteration count is printed as a
# diagnostic and deliberately NOT gated: the plan leaves that until a stable
# baseline exists.
set -euo pipefail

SYNTH_BAG_GEN="$1"
REPLAY_DEMO="$2"
EXPERIMENT="$3"

WORKDIR="$(mktemp -d)"
trap 'rm -rf "$WORKDIR"' EXIT

failures=0
fail() {
  echo "FAIL: $*" >&2
  failures=$((failures + 1))
}

# A generator/replay command that exits non-zero is not an assertion
# failure to be collected and reported at the end -- nothing after it is
# meaningful -- so it dumps its log and stops immediately.
run_or_dump() {  # <log path> <description> <command...>
  local log="$1" description="$2"
  shift 2
  if ! "$@" >"$log" 2>&1; then
    echo "--- $description failed; full output follows ---" >&2
    cat "$log" >&2
    echo "--- end of $description output ---" >&2
    exit 1
  fi
}

generate() {  # <name> [extra synth_bag_gen args...]
  local name="$1"
  shift
  run_or_dump "$WORKDIR/$name.gen.log" "synth_bag_gen ($name)" \
    "$SYNTH_BAG_GEN" --experiment "$EXPERIMENT" --out "$WORKDIR/$name.mcap" "$@"
}

replay() {  # <name>
  local name="$1"
  run_or_dump "$WORKDIR/$name.replay.log" "replay_demo ($name)" \
    "$REPLAY_DEMO" --bag "$WORKDIR/$name.mcap" --experiment "$EXPERIMENT" \
    --out "$WORKDIR/$name"
}

value() {  # <name> <summary key>
  local line
  line="$(grep -E "^summary\.$2=" "$WORKDIR/$1.replay.log" || true)"
  if [ -z "$line" ]; then
    echo "--- replay_demo ($1) output ---" >&2
    cat "$WORKDIR/$1.replay.log" >&2
    echo "FAIL: run '$1' printed no summary.$2 line" >&2
    exit 1
  fi
  printf '%s' "${line#*=}"
}

expect_eq() { [ "$2" = "$3" ] || fail "$1: expected '$3', got '$2'"; }
expect_num() {  # <what> <actual> <awk comparison, e.g. "<= 0.15">
  awk -v a="$2" "BEGIN { exit (a $3) ? 0 : 1 }" || fail "$1: expected value $3, got $2"
}

# --- generate the four bags -------------------------------------------
# `original` is the only one that carries usable reference truth; the other
# three differ from it in the reference branch alone.
generate original
generate no_ground_truth --omit-ground-truth
generate displaced_ground_truth --ground-truth-pose-offset-m 3.0
generate retimed_ground_truth --ground-truth-time-offset-s 5.0 --ground-truth-pose-offset-m 3.0

for name in original no_ground_truth displaced_ground_truth retimed_ground_truth; do
  replay "$name"
done

# --- 1) the estimate is good ------------------------------------------
expect_eq "estimator mode" "$(value original estimator_mode)" "imu_preintegration"
expect_eq "solver converged" "$(value original solver_converged)" "true"
expect_eq "initialization mode" "$(value original initialization)" "stationary"

boundary_count="$(value original keyframe_boundary_count)"
expect_num "keyframe boundary count" "$boundary_count" ">= 2"
# One preintegration interval per consecutive pair, with none silently
# dropped: a rejected interval would leave a gap the graph cannot bridge.
expect_eq "imu factor count" "$(value original imu_factor_count)" "$((boundary_count - 1))"
expect_eq "rejected imu intervals" "$(value original imu_interval_rejected_count)" "0"
expect_num "depth factor count" "$(value original depth_factor_count)" "> 0"
expect_num "sonar range factor count" "$(value original sonar_range_factor_count)" "> 0"
expect_eq "relative pose factor count" "$(value original relative_pose_factor_count)" "0"
expect_num "ATE rmse" "$(value original ate_rmse_m)" "<= 0.15"
expect_num "ATE matched poses" "$(value original ate_matched_poses)" ">= 2"

# --- 2) the estimate is earned ----------------------------------------
for name in no_ground_truth displaced_ground_truth retimed_ground_truth; do
  if ! diff -q "$WORKDIR/original_trajectory.tum" "$WORKDIR/${name}_trajectory.tum" >/dev/null; then
    fail "run '$name' produced a different trajectory than 'original' — ground truth is reaching the algorithm path"
    diff "$WORKDIR/original_trajectory.tum" "$WORKDIR/${name}_trajectory.tum" >&2 || true
  fi
done

# ATE is only meaningful where reference truth is present and honest, so it
# is asserted on `original` alone (above). What the other three are for is
# proving each flag actually did something -- read through the evaluator,
# which is the one consumer allowed to see ground truth.
expect_eq "no-ground-truth run has nothing to score against" \
  "$(value no_ground_truth ate_matched_poses)" "0"
# Displacing the poses leaves them matchable in time but wrong in space...
expect_eq "displaced-ground-truth run still matches in time" \
  "$(value displaced_ground_truth ate_matched_poses)" "$(value original ate_matched_poses)"
expect_num "displaced-ground-truth run reports a much larger error" \
  "$(value displaced_ground_truth ate_rmse_m)" "> 1.0"
# ...while displacing their timestamps by 5 s puts them outside the
# evaluator's matching window entirely.
expect_eq "retimed-ground-truth run matches nothing" \
  "$(value retimed_ground_truth ate_matched_poses)" "0"

# --- diagnostics (not gated) -------------------------------------------
echo "diagnostics: iterations=$(value original solver_iterations)" \
     "cost=$(value original initial_cost)->$(value original final_cost)" \
     "free_dims=$(value original free_parameter_dim)" \
     "residual_dims=$(value original residual_dim)" \
     "landmarks=$(value original landmark_count)"

if [ "$failures" -ne 0 ]; then
  echo "FAILED: $failures imu_preintegration smoke assertion(s)" >&2
  echo "--- replay_demo (original) output ---" >&2
  cat "$WORKDIR/original.replay.log" >&2
  exit 1
fi

echo "OK: imu_preintegration end-to-end — $boundary_count boundaries," \
     "$(value original imu_factor_count) IMU +" \
     "$(value original sonar_range_factor_count) sonar +" \
     "$(value original depth_factor_count) depth factors," \
     "$(value original initialization) init, ATE rmse=$(value original ate_rmse_m) m," \
     "identical trajectory with ground truth absent, displaced and retimed"
