#!/usr/bin/env bash
set -euo pipefail

if unknown_output="$("$1" --not-a-real-option 2>&1)"; then
  echo "unknown option unexpectedly succeeded" >&2
  exit 1
fi
grep -q 'unknown argument: --not-a-real-option' <<<"$unknown_output"

for arguments in \
  '--duration-s 9223372036' \
  '--duration-s 1e-20' \
  '--camera-hz 1e-300' \
  '--camera-hz 1e300'; do
  invalid_status=0
  timeout 2 "$1" $arguments >/dev/null 2>&1 || invalid_status=$?
  if [[ $invalid_status -eq 124 ]]; then
    echo "invalid timing arguments were not rejected promptly: $arguments" >&2
    exit 1
  fi
  if [[ $invalid_status -ne 2 ]]; then
    echo "invalid timing arguments returned $invalid_status instead of CLI error 2: $arguments" >&2
    exit 1
  fi
done

if stall_output="$("$1" --duration-s 0.3 --camera-hz 20 --sonar-hz 10 --state-hz 50 \
    --inject-stall-ms 120 2>&1)"; then
  echo "injected stall unexpectedly succeeded" >&2
  exit 1
fi
stall_summary="$(tail -n 1 <<<"$stall_output")"
grep -Eq '(^| )rate_count_violations=[1-9][0-9]*( |$)' <<<"$stall_summary"
grep -Eq '(^| )deadline_misses=[1-9][0-9]*( |$)' <<<"$stall_summary"

output="$("$1" --duration-s 3 --camera-hz 20 --sonar-hz 10 --state-hz 50)"
summary="$(tail -n 1 <<<"$output")"

grep -Eq '^reference_delivered=0 semantic_rejected=1 queue_capacity_violations=0 flush_count=1( |$)' <<<"$summary"
for field in reference_rejected left_delivered right_delivered sonar_delivered state_delivered; do
  grep -Eq "(^| )${field}=[1-9][0-9]*( |$)" <<<"$summary"
done
grep -Eq '(^| )rate_count_violations=0( |$)' <<<"$summary"
grep -Eq '(^| )deadline_misses=0( |$)' <<<"$summary"

submitted="$(sed -nE 's/.*(^| )submitted=([0-9]+)( |$).*/\2/p' <<<"$summary")"
delivered="$(sed -nE 's/.*(^| )delivered=([0-9]+)( |$).*/\2/p' <<<"$summary")"
test -n "$submitted" && test "$submitted" = "$delivered"
for stream in left right sonar state; do
  expected="$(sed -nE "s/.*(^| )${stream}_expected=([0-9]+)( |$).*/\2/p" <<<"$summary")"
  actual="$(sed -nE "s/.*(^| )${stream}_actual=([0-9]+)( |$).*/\2/p" <<<"$summary")"
  test -n "$expected" && test "$expected" = "$actual"
done

if "$1" --duration-s 0.1 --camera-hz 20 --sonar-hz 10 --state-hz 50 \
    >/dev/full 2>/dev/null; then
  echo "summary write failure unexpectedly succeeded" >&2
  exit 1
fi
