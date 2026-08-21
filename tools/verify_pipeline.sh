#!/usr/bin/env bash
# End-to-end local verification: runs every step README.md's "构建" /
# "运行端到端 demo" / "测试策略" sections describe, in order, and saves each
# step's exact command + full stdout/stderr + pass/fail + timing under
# --out-dir so a run can be inspected or compared later. Everything here is
# the synth_bag_gen -> replay_demo path plus the C++/Python test suites —
# the part of the platform that does NOT need a live HoloOcean/UE5/ROS2
# simulator (see README.md "运行端到端 demo"). The ROS2 bridge build is
# opt-in via --with-ros2 since it needs a pre-existing colcon workspace
# (see README.md's "adapters/ros2" build instructions) that most machines
# won't have set up.
#
# Usage:
#   tools/verify_pipeline.sh [--out-dir DIR] [--experiment PATH]
#                             [--build-dir DIR] [--clean] [--with-ros2]
#
# Exit status: 0 if every step passed, 1 if any step failed (summary.txt
# and per-step logs are written either way).
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

OUT_DIR="${TMPDIR:-/tmp}/uw_slam_verify/$(date +%Y%m%d_%H%M%S)"
EXPERIMENT="configs/experiment/synthetic_smoke.yaml"
BUILD_DIR="build"
CLEAN=0
WITH_ROS2=0

while [ $# -gt 0 ]; do
  case "$1" in
    --out-dir) OUT_DIR="$2"; shift 2 ;;
    --experiment) EXPERIMENT="$2"; shift 2 ;;
    --build-dir) BUILD_DIR="$2"; shift 2 ;;
    --clean) CLEAN=1; shift ;;
    --with-ros2) WITH_ROS2=1; shift ;;
    -h|--help)
      sed -n '2,18p' "$0" | sed 's/^# \{0,1\}//'
      exit 0
      ;;
    *) echo "unknown argument: $1" >&2; exit 1 ;;
  esac
done

mkdir -p "$OUT_DIR"
SUMMARY="$OUT_DIR/summary.txt"
{
  echo "uw_slam pipeline verification — $(date -Iseconds)"
  echo "root:       $ROOT"
  echo "out dir:    $OUT_DIR"
  echo "experiment: $EXPERIMENT"
  echo ""
  printf '%-28s %-6s %6s  %s\n' STEP STATUS SECS LOG
} > "$SUMMARY"

STEP_NUM=0
FAILED=0
LAST_LOG=""

# step <name> <command...>: runs the command, tees it to a numbered log file
# under $OUT_DIR, records PASS/FAIL + elapsed seconds in $SUMMARY, and on
# failure prints the last 20 log lines immediately so you don't have to go
# hunting for why.
step() {
  local name="$1"; shift
  STEP_NUM=$((STEP_NUM + 1))
  local log; log="$OUT_DIR/$(printf '%02d' "$STEP_NUM")_${name}.log"
  LAST_LOG="$log"
  echo "==> [$STEP_NUM] $name"
  {
    echo "\$ $*"
    echo "----"
  } > "$log"
  local start end status
  start=$(date +%s)
  if "$@" >>"$log" 2>&1; then
    status=PASS
  else
    status=FAIL
    FAILED=1
  fi
  end=$(date +%s)
  printf '%-28s %-6s %6ss  %s\n' "$name" "$status" "$((end - start))" "$log" | tee -a "$SUMMARY"
  if [ "$status" = FAIL ]; then
    echo "    ---- last 20 lines of $log ----"
    tail -n 20 "$log" | sed 's/^/    /'
  fi
}

# 1) configure, only if the build dir isn't already configured (or --clean)
if [ "$CLEAN" -eq 1 ] || [ ! -f "$BUILD_DIR/CMakeCache.txt" ]; then
  [ "$CLEAN" -eq 1 ] && rm -rf "$BUILD_DIR"
  if [ -d "$HOME/miniconda3/envs/uw_slam_build" ]; then
    export PATH="$HOME/miniconda3/envs/uw_slam_build/bin:$PATH"
    step configure cmake -S . -B "$BUILD_DIR" -DCMAKE_PREFIX_PATH="$HOME/miniconda3/envs/uw_slam_build"
  else
    step configure cmake -S . -B "$BUILD_DIR"
  fi
fi

# 2) build
step build cmake --build "$BUILD_DIR" -j"$(nproc)"

# 3) C++ tests (contract + per-layer unit + integration determinism replay —
# all registered via ctest, see cmake/Tests.cmake)
step ctest ctest --test-dir "$BUILD_DIR" --output-on-failure

# 4) Python tests (adapters/holoocean). NOTE: `uv sync`/`uv run` are NOT
# used here even though README.md lists `uv sync` as an option — uv's
# universal resolution tries to solve the optional `holoocean` extra for
# every supported Python/platform combo, and holoocean==0.5.8's Windows-only
# pywin32 pin has no wheel for newer 3.x tags, so `uv sync` fails outright
# on this project regardless of platform (reproduced directly: plain
# `uv sync` in adapters/holoocean errors on "uw-holoocean-adapter[holoocean]
# ... requirements are unsatisfiable"). `pip install -e ".[dev]"` in a plain
# venv sidesteps this (no universal cross-platform resolution) and is what
# actually gives the 9/9 pass README.md/CLAUDE.md describe.
if [ -d adapters/holoocean ]; then
  step pytest_holoocean bash -c '
    cd adapters/holoocean
    if [ ! -x .venv/bin/pytest ]; then
      python3 -m venv .venv
      .venv/bin/pip install -q -e ".[dev]"
    fi
    .venv/bin/pytest -q
  '
fi

# 5) dependency-direction lint (core/algorithms must not include ROS/HoloOcean)
step lint_no_ros_in_core bash tools/lint/check_no_ros_in_core.sh

# 6) synth_bag_gen: generate a synthetic MCAP bag with known ground truth
BAG="$OUT_DIR/synthetic.mcap"
step synth_bag_gen "$BUILD_DIR/bin/synth_bag_gen" \
  --experiment "$EXPERIMENT" --out "$BAG"

# 7) replay_demo: frontend -> factor builders -> pose graph solve -> ATE
DEMO_PREFIX="$OUT_DIR/demo"
step replay_demo "$BUILD_DIR/bin/replay_demo" \
  --bag "$BAG" --experiment "$EXPERIMENT" --out "$DEMO_PREFIX"
REPLAY_LOG="$LAST_LOG"

# 8) optional: ROS2 bridge build (needs a pre-built ~/ros2_ws colcon
# workspace — see README.md's "adapters/ros2" section; not attempted unless
# --with-ros2 is passed, and skipped cleanly if the workspace isn't there).
if [ "$WITH_ROS2" -eq 1 ]; then
  if [ -f "$HOME/ros2_ws/install/setup.bash" ] && [ -f /opt/ros/jazzy/setup.bash ]; then
    ROS2_BUILD_DIR="${BUILD_DIR}_ros2"
    # ROS2's setup.bash references unset variables (e.g. AMENT_TRACE_SETUP_FILES)
    # and trips `set -u`; relax it just for sourcing.
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    # shellcheck disable=SC1091
    source "$HOME/ros2_ws/install/setup.bash"
    set -u
    export PATH="$HOME/miniconda3/envs/uw_slam_build/bin:$PATH"
    step configure_ros2 cmake -S . -B "$ROS2_BUILD_DIR" \
      -DCMAKE_PREFIX_PATH="$HOME/miniconda3/envs/uw_slam_build" -DUW_BUILD_ROS2=ON
    step build_ros2 cmake --build "$ROS2_BUILD_DIR" -j"$(nproc)" --target holoocean_sonar_bridge_node
  else
    echo "==> [--with-ros2] skipped: ~/ros2_ws or /opt/ros/jazzy not found"
    echo "with-ros2               SKIP      -  no ~/ros2_ws/install/setup.bash or /opt/ros/jazzy/setup.bash" >> "$SUMMARY"
  fi
fi

# Pull the ATE line out of the replay_demo log into the summary, and copy
# the artifacts replay_demo produced next to the logs.
ATE_LINE="$(command grep -m1 '^ATE:' "$REPLAY_LOG" 2>/dev/null || true)"
{
  echo ""
  [ -n "$ATE_LINE" ] && echo "$ATE_LINE"
  echo ""
  echo "artifacts:"
  echo "  bag:        $BAG"
  echo "  trajectory: ${DEMO_PREFIX}_trajectory.tum"
  echo "  manifest:   ${DEMO_PREFIX}_run_manifest.json"
} | tee -a "$SUMMARY"

echo ""
if [ "$FAILED" -eq 1 ]; then
  echo "RESULT: one or more steps FAILED — see $SUMMARY and the logs under $OUT_DIR"
  echo "" >> "$SUMMARY"
  echo "RESULT: FAILED" >> "$SUMMARY"
  exit 1
fi
echo "RESULT: all steps passed. summary: $SUMMARY"
echo "" >> "$SUMMARY"
echo "RESULT: PASSED" >> "$SUMMARY"
