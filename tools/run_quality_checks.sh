#!/usr/bin/env bash
# P3 workstream A1 (docs/superpowers/plans/2026-08-22-p3-online-
# reconstruction-and-productionization.md): quality checks beyond
# tools/verify_pipeline.sh's main build/test/replay path. Each mode gets
# its own build dir (build_asan/build_cov) so it never disturbs build/ —
# safe to run alongside a normal dev build.
#
# Usage: tools/run_quality_checks.sh [sanitizer|coverage|static-analysis|all]
# Default: all. Exit status: 0 if every requested check passed, 1 otherwise
# (static-analysis findings are reported, not gated — see the plan doc's
# A1 workstream for why: no baseline exists yet to pick a sane threshold).
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

MODE="${1:-all}"
FAILED=0

if [ -d "$HOME/miniconda3/envs/uw_slam_build" ]; then
  export PATH="$HOME/miniconda3/envs/uw_slam_build/bin:$PATH"
  CMAKE_PREFIX_ARGS=(-DCMAKE_PREFIX_PATH="$HOME/miniconda3/envs/uw_slam_build")
else
  CMAKE_PREFIX_ARGS=()
fi

run_sanitizer() {
  local build_dir="build_asan"
  echo "==> [sanitizer] configure (UW_SANITIZER=address)"
  cmake -S . -B "$build_dir" "${CMAKE_PREFIX_ARGS[@]}" -DUW_SANITIZER=address \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo || { FAILED=1; return; }
  echo "==> [sanitizer] build"
  cmake --build "$build_dir" -j"$(nproc)" || { FAILED=1; return; }
  echo "==> [sanitizer] ctest (ASan+UBSan)"
  ctest --test-dir "$build_dir" --output-on-failure || FAILED=1
  # TSan (UW_SANITIZER=thread) is intentionally NOT run here — see
  # CMakeLists.txt's UW_SANITIZER option comment: this environment's
  # prebuilt conda-forge protobuf/gtest shared libraries aren't
  # TSan-instrumented, which produces false-positive race/use-after-free
  # reports on their internals even in single-threaded test bodies
  # (verified 2026-08-22 — the same code passes clean under ASan, which is
  # at least as sensitive to a genuine use-after-free).
}

run_coverage() {
  local build_dir="build_cov"
  echo "==> [coverage] configure (UW_COVERAGE=ON)"
  cmake -S . -B "$build_dir" "${CMAKE_PREFIX_ARGS[@]}" -DUW_COVERAGE=ON \
    -DCMAKE_BUILD_TYPE=Debug || { FAILED=1; return; }
  echo "==> [coverage] build"
  cmake --build "$build_dir" -j"$(nproc)" || { FAILED=1; return; }
  echo "==> [coverage] ctest"
  ctest --test-dir "$build_dir" --output-on-failure || FAILED=1
  echo "==> [coverage] summary (gcov; this environment has no lcov/gcovr —"
  echo "    see the plan doc's A1 workstream for what was checked)"
  # Summarize line coverage only for this repo's own src/*.cpp objects —
  # excludes generated protobuf code and FetchContent'd third-party (MCAP)
  # sources, which aren't meaningful coverage targets for this repo's own
  # test suite. gcov's -o points it at the object directory holding the
  # matching .gcda/.gcno for each .gcno found. Raw gcov output ALSO reports
  # coverage for every header pulled in (Eigen, protobuf-generated, etc.) —
  # filter to this repo's own src/ and include/ paths only, by absolute
  # path (gcov prints absolute paths), excluding generated/ (protobuf).
  (
    cd "$build_dir" || exit 1
    find . -name '*.gcno' -path '*/src/*' | while read -r gcno; do
      obj_dir="$(dirname "$gcno")"
      gcov -o "$obj_dir" -p "$gcno" 2>/dev/null
    done > /tmp/uw_gcov_raw.log
    awk -v root="$ROOT" '
      /^File / { file=$0; next }
      /^Lines executed/ {
        if (index(file, root "/src/") > 0 || index(file, root "/include/") > 0) {
          if (index(file, "/generated/") == 0) print file "\n" $0
        }
      }
    ' /tmp/uw_gcov_raw.log
  )
}

run_static_analysis() {
  echo "==> [static-analysis] cppcheck"
  if ! command -v cppcheck >/dev/null 2>&1; then
    echo "cppcheck not installed on this machine, skipping (present via apt on the CI runner image)"
    return
  fi
  # Reported, not gated (see script header) — no --error-exitcode.
  cppcheck --enable=warning,performance,portability --std=c++17 --language=c++ \
    --suppress=missingInclude --inline-suppr -q \
    -I include \
    include src apps 2>&1 | tee /tmp/uw_cppcheck.log
}

case "$MODE" in
  sanitizer) run_sanitizer ;;
  coverage) run_coverage ;;
  static-analysis) run_static_analysis ;;
  all)
    run_sanitizer
    run_coverage
    run_static_analysis
    ;;
  *)
    echo "unknown mode: $MODE (expected sanitizer|coverage|static-analysis|all)" >&2
    exit 1
    ;;
esac

exit "$FAILED"
