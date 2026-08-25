#!/usr/bin/env bash
set -euo pipefail

MATRIX_BIN="$1"
SOURCE_ROOT="$2"
TEST_ROOT="$(mktemp -d)"
trap 'rm -rf "$TEST_ROOT"' EXIT

cp -R "$SOURCE_ROOT/configs" "$TEST_ROOT/configs"
sed -i 's/detector_threshold: 50/detector_threshold: 255/' \
  "$TEST_ROOT/configs/defaults/platform.yaml"

OUTPUT="$TEST_ROOT/matrix.txt"
MATRIX_EXIT=0
set +e
"$MATRIX_BIN" --experiment "$TEST_ROOT/configs/experiment/synthetic_smoke.yaml" \
  --seed 4242 --trials-per-scenario 1 > "$OUTPUT" 2>&1
MATRIX_EXIT=$?
set -e

if [ "$MATRIX_EXIT" -ne 1 ]; then
  echo "FAIL: expected scenario matrix gate-failure exit 1, got $MATRIX_EXIT"
  cat "$OUTPUT"
  exit 1
fi

EXPECTED_SCENARIOS="clean_textured low_texture_sonar_visible turbid_sonar_visible repeated_structure elevation_stress time_offset_fault extrinsic_perturbation sonar_dropout optical_invalid_region"
for scenario in $EXPECTED_SCENARIOS; do
  count="$(awk -v expected="scenario=$scenario" '$1 == expected { count++ } END { print count + 0 }' "$OUTPUT")"
  if [ "$count" -ne 1 ]; then
    echo "FAIL: expected exactly one completed report for $scenario, got $count"
    cat "$OUTPUT"
    exit 1
  fi
done

clean_line="$(grep '^scenario=clean_textured ' "$OUTPUT")"
accepted="$(sed -E 's/.* accepted=([0-9]+).*/\1/' <<< "$clean_line")"
if [ "$accepted" -ne 0 ]; then
  echo "FAIL: detector_threshold=255 from experiment defaults was not applied; clean_textured accepted=$accepted"
  cat "$OUTPUT"
  exit 1
fi

if ! grep -q '^GATE FAIL: clean_textured had 0/1 accepted associations' "$OUTPUT"; then
  echo "FAIL: exit 1 was not the expected disabled-detection coverage gate"
  cat "$OUTPUT"
  exit 1
fi

echo "OK: scenario matrix applied typed sonar frontend defaults"
