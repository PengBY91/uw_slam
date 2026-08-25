#!/usr/bin/env bash
set -euo pipefail

if unknown_output="$("$1" --not-a-real-option 2>&1)"; then
  echo "unknown option unexpectedly succeeded" >&2
  exit 1
fi
grep -q 'unknown argument: --not-a-real-option' <<<"$unknown_output"

output="$("$1" --duration-s 3 --camera-hz 20 --sonar-hz 10 --state-hz 50)"
summary="$(tail -n 1 <<<"$output")"

grep -q 'reference_delivered=0' <<<"$summary"
grep -q 'semantic_rejected=1' <<<"$summary"
grep -q 'queue_capacity_violations=0' <<<"$summary"
grep -q 'flush_count=1' <<<"$summary"
