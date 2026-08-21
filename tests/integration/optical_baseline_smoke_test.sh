#!/usr/bin/env bash
# L2: real synth_stereo_gen + StereoOpticalDepthFrontend + depth metrics,
# exercised end to end (plan 2 of the acoustic-optic series). Not a
# rigorous accuracy benchmark — that's plan 5's ablation/scenario-matrix
# job; this only proves the geometry pipeline recovers a known depth on a
# clean synthetic scene within a small, honestly-loose tolerance.
set -euo pipefail

SYNTH_STEREO_GEN="$1"
OPTICAL_BASELINE_EVAL="$2"
EXPERIMENT_CONFIG="$3"
WORKDIR="$(mktemp -d)"
trap 'rm -rf "$WORKDIR"' EXIT

"$SYNTH_STEREO_GEN" --out "$WORKDIR/stereo.mcap"
"$OPTICAL_BASELINE_EVAL" --bag "$WORKDIR/stereo.mcap" --experiment "$EXPERIMENT_CONFIG" \
  --max-rmse-m 0.05 --min-coverage 0.9
