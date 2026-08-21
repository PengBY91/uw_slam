#!/usr/bin/env bash
# L2: same bag/config/seed rerun must give identical trajectory output
# (platform architecture section 15's L2 row). Exercises the real
# synth_bag_gen + replay_demo binaries, not a mock.
set -euo pipefail

SYNTH_BAG_GEN="$1"
REPLAY_DEMO="$2"
WORKDIR="$(mktemp -d)"
trap 'rm -rf "$WORKDIR"' EXIT

"$SYNTH_BAG_GEN" --out "$WORKDIR/scenario.mcap" --seed 7

"$REPLAY_DEMO" --bag "$WORKDIR/scenario.mcap" --out "$WORKDIR/run1" >/dev/null
"$REPLAY_DEMO" --bag "$WORKDIR/scenario.mcap" --out "$WORKDIR/run2" >/dev/null

if ! diff -q "$WORKDIR/run1_trajectory.tum" "$WORKDIR/run2_trajectory.tum" >/dev/null; then
  echo "FAIL: replay_demo produced different trajectories on two runs of the same bag"
  diff "$WORKDIR/run1_trajectory.tum" "$WORKDIR/run2_trajectory.tum" || true
  exit 1
fi

echo "OK: deterministic replay confirmed ($(wc -l < "$WORKDIR/run1_trajectory.tum") poses)"
