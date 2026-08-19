#!/usr/bin/env bash
# Skeleton runner for the unmodified sonar_camera_reconstruction ROS1
# baseline. NOT functional yet — see README.md for the two blockers
# (bruce_slam dependency, no ROS1 on this machine) that must be resolved
# before this script can actually run anything. Kept as a real script
# (not just prose) so the next step is "fill in the TODOs", not "figure out
# the shape from scratch".
set -euo pipefail

BAG=""
OUT_DIR="outputs/baseline"

while [ $# -gt 0 ]; do
  case "$1" in
    --bag) BAG="$2"; shift 2 ;;
    --out) OUT_DIR="$2"; shift 2 ;;
    *) echo "unknown argument: $1" >&2; exit 1 ;;
  esac
done

if [ -z "$BAG" ]; then
  echo "usage: run_baseline.sh --bag <path> [--out <dir>]" >&2
  exit 1
fi

echo "TODO: verify bruce_slam is vendored/available (see README.md blocker #1)"
echo "TODO: convert $BAG's canonical MCAP topics to sonar_camera_reconstruction's expected ROS1 topics"
echo "TODO: roslaunch sonar_camera_reconstruction_pkg merge.launch, recording to $OUT_DIR"
echo "Not implemented in this repo state — see README.md."
exit 1
