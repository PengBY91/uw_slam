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
"$MATRIX_BIN" --experiment "$TEST_ROOT/configs/experiment/synthetic_smoke.yaml" \
  --seed 4242 --trials-per-scenario 1 > "$OUTPUT" 2>&1 || true

clean_line="$(grep '^scenario=clean_textured ' "$OUTPUT")"
accepted="$(sed -E 's/.* accepted=([0-9]+).*/\1/' <<< "$clean_line")"
if [ "$accepted" -ne 0 ]; then
  echo "FAIL: detector_threshold=255 from experiment defaults was not applied; clean_textured accepted=$accepted"
  cat "$OUTPUT"
  exit 1
fi

echo "OK: scenario matrix applied typed sonar frontend defaults"
