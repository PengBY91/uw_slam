#!/usr/bin/env bash
# PREP-B-01: the synthetic IMU fixture contract.
#
# This checks what apps/synth_bag_gen.cpp WRITES, not what any estimator
# makes of it. The IMU-mode bag is the only legitimate input the
# imu_preintegration pipeline gets, so every property it depends on has to
# be pinned here rather than discovered later as an unexplained ATE:
#
#   * one /keyframe/boundary per keyframe, ids unique, times strictly
#     increasing and starting at the end of the pre-roll;
#   * /raw/imu starting at t = 0 at the rig's rate, with a stationary window
#     that reaches the first boundary -- this is the only non-ground-truth
#     source the stationary initializer has for the initial biases and the
#     gravity direction;
#   * every keyframe-anchored topic shifted by exactly that pre-roll, so no
#     evidence predates the keyframe it belongs to;
#   * the IMU stream depending on the seed and on nothing else -- not on
#     whether cameras are rendered, not on whether relative-pose evidence is
#     written. Each noise purpose draws from its own salted RNG stream
#     (MakeStreamRng); this is the regression test for the class of bug
#     CLAUDE.md records under "shared one std::mt19937_64".
#
# Assertions are anchored on bag_audit's `summary.<key>=<value>` lines, not
# on prose, so a wording change in the audit cannot silently loosen them.
set -euo pipefail

SYNTH_BAG_GEN="$1"
BAG_AUDIT="$2"
REPO_ROOT="$3"

WORKDIR="$(mktemp -d)"
trap 'rm -rf "$WORKDIR"' EXIT

IMU_EXPERIMENT="$REPO_ROOT/configs/experiment/synthetic_imu_preintegration.yaml"
IMU_CAMERA_EXPERIMENT="$REPO_ROOT/configs/experiment/synthetic_imu_preintegration_camera.yaml"
BASELINE_EXPERIMENT="$REPO_ROOT/configs/experiment/synthetic_smoke.yaml"
IMU_RIG="$REPO_ROOT/configs/rig/example_auv_sonar_only.yaml"

# configs/scenario/synthetic_smoke.yaml, shared by all three experiments.
NUM_KEYFRAMES=12
KEYFRAME_PERIOD_S=0.2
PRE_ROLL_S=0.75
IMU_RATE_HZ=200
GRAVITY_MPS2=9.80665

failures=0
fail() {
  echo "FAIL: $*" >&2
  failures=$((failures + 1))
}

generate() {  # <bag> <experiment> [extra synth_bag_gen args...]
  local bag="$1" experiment="$2"
  shift 2
  if ! "$SYNTH_BAG_GEN" --experiment "$experiment" --out "$bag" "$@" >"$bag.gen.log" 2>&1; then
    echo "--- synth_bag_gen ($experiment) ---" >&2
    cat "$bag.gen.log" >&2
    exit 1
  fi
}

audit() {  # <bag> [--rig <rig>]
  local bag="$1"
  shift
  if ! "$BAG_AUDIT" --bag "$bag" "$@" >"$bag.audit" 2>&1; then
    echo "--- bag_audit ($bag) ---" >&2
    cat "$bag.audit" >&2
    exit 1
  fi
}

value() {  # <bag> <summary key>
  local line
  line="$(grep -E "^summary\.$2=" "$1.audit" || true)"
  if [ -z "$line" ]; then
    echo "--- bag_audit ($1) ---" >&2
    cat "$1.audit" >&2
    echo "FAIL: audit of $1 has no summary.$2 line" >&2
    exit 1
  fi
  printf '%s' "${line#*=}"
}

expect_eq() {  # <what> <actual> <expected>
  [ "$2" = "$3" ] || fail "$1: expected '$3', got '$2'"
}

expect_close() {  # <what> <actual> <expected> <tolerance>
  awk -v a="$2" -v b="$3" -v tol="$4" 'BEGIN { exit ((a - b) < tol && (b - a) < tol) ? 0 : 1 }' \
    || fail "$1: expected $3 +/- $4, got $2"
}

expect_below() {  # <what> <actual> <limit>
  awk -v a="$2" -v limit="$3" 'BEGIN { exit (a < limit) ? 0 : 1 }' \
    || fail "$1: expected below $3, got $2"
}

seconds() {  # <expression in awk syntax> -> the %.9f form bag_audit prints
  awk -v n="$1" 'BEGIN { printf "%.9f", n }'
}

last_keyframe_s="$(awk -v pre="$PRE_ROLL_S" -v n="$NUM_KEYFRAMES" -v period="$KEYFRAME_PERIOD_S" \
  'BEGIN { print pre + (n - 1) * period }')"

generate "$WORKDIR/imu.mcap" "$IMU_EXPERIMENT"
audit "$WORKDIR/imu.mcap" --rig "$IMU_RIG"
IMU="$WORKDIR/imu.mcap"

# --- 1) keyframe boundary contract -------------------------------------
expect_eq "keyframe boundary count" "$(value "$IMU" keyframe_boundary.count)" "$NUM_KEYFRAMES"
expect_eq "keyframe boundary distinct ids" "$(value "$IMU" keyframe_boundary.distinct_ids)" "$NUM_KEYFRAMES"
expect_eq "keyframe boundary ids unique" "$(value "$IMU" keyframe_boundary.ids_unique)" "true"
expect_eq "keyframe boundary strictly increasing" \
  "$(value "$IMU" keyframe_boundary.capture_strictly_increasing)" "true"
expect_eq "first keyframe boundary" "$(value "$IMU" keyframe_boundary.first_capture_s)" \
  "$(seconds "$PRE_ROLL_S")"
expect_eq "last keyframe boundary" "$(value "$IMU" keyframe_boundary.last_capture_s)" \
  "$(seconds "$last_keyframe_s")"
expect_close "keyframe boundary rate" "$(value "$IMU" keyframe_boundary.mean_rate_hz)" \
  "$(awk -v p="$KEYFRAME_PERIOD_S" 'BEGIN { print 1 / p }')" 1e-6

# --- 2) IMU stream and its stationary pre-roll --------------------------
expect_eq "IMU stream start" "$(value "$IMU" imu.first_capture_s)" "$(seconds 0)"
expect_eq "IMU stream end" "$(value "$IMU" imu.last_capture_s)" "$(seconds "$last_keyframe_s")"
expect_close "IMU rate" "$(value "$IMU" imu.mean_rate_hz)" "$IMU_RATE_HZ" 1e-6
expect_eq "IMU sample count" "$(value "$IMU" imu.count)" \
  "$(awk -v s="$last_keyframe_s" -v r="$IMU_RATE_HZ" 'BEGIN { printf "%d", s * r + 1 }')"
expect_eq "malformed IMU samples" "$(value "$IMU" imu.malformed_sample_count)" "0"
# The pre-roll must REACH the first boundary, not stop one sample short:
# the interval the initializer reads is closed on the boundary.
expect_eq "IMU pre-roll span" "$(value "$IMU" imu.pre_roll_s)" "$(seconds "$PRE_ROLL_S")"
expect_eq "IMU pre-roll sample count" "$(value "$IMU" imu.pre_roll_sample_count)" \
  "$(awk -v p="$PRE_ROLL_S" -v r="$IMU_RATE_HZ" 'BEGIN { printf "%d", p * r + 1 }')"
# The stationary thresholds of docs/imu-preintegration-design-2026-09-03.md
# section 7, applied to the window mean (which is what makes them meaningful
# under a realistic noise density -- see ImuWindowStats in
# include/runtime/bag_audit_checks.hpp).
expect_below "IMU pre-roll mean gyro norm" "$(value "$IMU" imu.pre_roll_gyro_mean_norm_radps)" 0.01
expect_close "IMU pre-roll mean specific force" \
  "$(value "$IMU" imu.pre_roll_accel_mean_norm_mps2)" "$GRAVITY_MPS2" 0.05
# The seam. "Stationary until the first boundary" is worth nothing if the
# body is already at full speed one sample later: the initializer's v0 = 0
# would then be wrong by the whole traversal speed (about 5 m/s on this
# scenario) and no estimator could recover from it. synth_bag_gen traverses
# the arc with a quintic smoothstep precisely so the rate and the specific
# force are continuous here; a return to the constant-rate profile shows up
# as a gyro reading two orders of magnitude above this bound.
expect_below "IMU first sample after the boundary — gyro" \
  "$(value "$IMU" imu.first_post_boundary_gyro_norm_radps)" 0.02
expect_close "IMU first sample after the boundary — specific force" \
  "$(value "$IMU" imu.first_post_boundary_accel_norm_mps2)" "$GRAVITY_MPS2" 0.1

# --- 3) every keyframe-anchored topic shifted by the pre-roll -----------
generate "$WORKDIR/baseline.mcap" "$BASELINE_EXPERIMENT"
audit "$WORKDIR/baseline.mcap"
BASELINE="$WORKDIR/baseline.mcap"

expect_eq "baseline carries no IMU" "$(value "$BASELINE" imu.count)" "0"
expect_eq "baseline carries no keyframe boundaries" \
  "$(value "$BASELINE" keyframe_boundary.count)" "0"

for key in gt_state.first_capture_s gt_state.last_capture_s \
           sonar_frame.first_capture_s sonar_frame.last_capture_s \
           evidence_depth.first_log_time_s evidence_depth.last_log_time_s; do
  expect_eq "$key shifted by the pre-roll" "$(value "$IMU" "$key")" \
    "$(seconds "$(awk -v b="$(value "$BASELINE" "$key")" -v p="$PRE_ROLL_S" 'BEGIN { print b + p }')")"
done
for key in gt_state.count sonar_frame.count evidence_depth.count; do
  expect_eq "$key unchanged by the pre-roll" "$(value "$IMU" "$key")" "$(value "$BASELINE" "$key")"
done

# --- 4) the IMU stream depends on the seed and nothing else -------------
generate "$WORKDIR/rerun.mcap" "$IMU_EXPERIMENT"
audit "$WORKDIR/rerun.mcap" --rig "$IMU_RIG"
for topic in imu sonar_frame evidence_relative_pose; do
  expect_eq "same seed reproduces the $topic stream" \
    "$(value "$WORKDIR/rerun.mcap" "$topic.payload_digest")" "$(value "$IMU" "$topic.payload_digest")"
done

# Rendering cameras consumes landmark_rng; it must not perturb imu_rng.
generate "$WORKDIR/camera.mcap" "$IMU_CAMERA_EXPERIMENT"
audit "$WORKDIR/camera.mcap"
CAMERA="$WORKDIR/camera.mcap"
if [ "$(value "$CAMERA" camera_left.count)" = "0" ]; then
  fail "the camera variant rendered no stereo frames — the comparison below would be vacuous"
fi
for topic in imu sonar_frame evidence_relative_pose; do
  expect_eq "rendering cameras leaves the $topic stream alone" \
    "$(value "$CAMERA" "$topic.payload_digest")" "$(value "$IMU" "$topic.payload_digest")"
done

# Writing relative-pose evidence consumes pose_rng; likewise.
generate "$WORKDIR/no_relpose.mcap" "$IMU_EXPERIMENT" --omit-relative-pose
audit "$WORKDIR/no_relpose.mcap" --rig "$IMU_RIG"
NO_RELPOSE="$WORKDIR/no_relpose.mcap"
if [ "$(value "$IMU" evidence_relative_pose.count)" = "0" ]; then
  fail "the IMU bag has no relative-pose evidence to omit — the comparison below would be vacuous"
fi
expect_eq "--omit-relative-pose drops the topic" "$(value "$NO_RELPOSE" evidence_relative_pose.count)" "0"
for topic in imu sonar_frame; do
  expect_eq "omitting relative pose leaves the $topic stream alone" \
    "$(value "$NO_RELPOSE" "$topic.payload_digest")" "$(value "$IMU" "$topic.payload_digest")"
done

if [ "$failures" -ne 0 ]; then
  echo "FAILED: $failures synthetic IMU fixture assertion(s)" >&2
  exit 1
fi

echo "OK: synthetic IMU fixture — $NUM_KEYFRAMES boundaries from ${PRE_ROLL_S}s, $(value "$IMU" imu.count) IMU samples from 0s at ${IMU_RATE_HZ}Hz, pre-roll stationary, stream digest $(value "$IMU" imu.payload_digest)"
