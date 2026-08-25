#!/usr/bin/env bash
set -euo pipefail

if unknown_output="$("$1" --not-a-real-option 2>&1)"; then
  echo "unknown option unexpectedly succeeded" >&2
  exit 1
fi
grep -q 'unknown argument: --not-a-real-option' <<<"$unknown_output"

output="$("$1" --duration-s 5 --camera-hz 20 --sonar-hz 10 --state-hz 50)"

grep -Eq 'fused_tracks=[1-9][0-9]*' <<<"$output"
grep -q 'truth_delivered=0' <<<"$output"
grep -q 'stale_normal_tracks=0' <<<"$output"
grep -q 'queue_capacity_violations=0' <<<"$output"
grep -Eq 'result_age_p95_ms=([0-9]{1,2}|1[0-9]{2}|2[0-4][0-9])(\.[0-9]+)?' <<<"$output"

# Sanity check that --drop-visual-at-s / --drop-sonar-at-s are accepted and
# still produce a clean run (queues, reference-plane isolation) even with a
# modality deliberately cut mid-run -- not asserting on exact degraded-mode
# timing here, just that the online slice keeps running and stays healthy
# on the invariants that must hold regardless of which modality is down.
for drop_flag in --drop-visual-at-s --drop-sonar-at-s; do
  drop_output="$("$1" --duration-s 4 "$drop_flag" 2 --camera-hz 20 --sonar-hz 10 --state-hz 50)"
  grep -q 'truth_delivered=0' <<<"$drop_output"
  grep -q 'queue_capacity_violations=0' <<<"$drop_output"
done
