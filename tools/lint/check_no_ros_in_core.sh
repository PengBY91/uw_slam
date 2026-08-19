#!/usr/bin/env bash
# Enforces platform architecture dependency invariants (section 5): core/
# and algorithms/ must never include ROS, HoloOcean, or third-party vendor
# headers. Those only belong in adapters/. Run from anywhere; paths are
# resolved relative to this script.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
FAIL=0

check_dir() {
  local dir="$1"
  shift
  local patterns=("$@")
  if [ ! -d "$ROOT/$dir" ]; then return; fi
  for pattern in "${patterns[@]}"; do
    local hits
    hits="$(grep -rlE "$pattern" "$ROOT/$dir" --include=*.hpp --include=*.cpp --include=*.h --include=*.cc 2>/dev/null || true)"
    if [ -n "$hits" ]; then
      echo "FAIL: forbidden include pattern '$pattern' found under $dir:"
      echo "$hits" | sed 's/^/  /'
      FAIL=1
    fi
  done
}

FORBIDDEN_PATTERNS=(
  '#include\s*[<"](rclcpp|ros|rmw)/'
  '#include\s*[<"]holoocean'
  '#include\s*[<"]okvis/'
  '#include\s*[<"]sonar_oculus'
)

check_dir "core" "${FORBIDDEN_PATTERNS[@]}"
check_dir "algorithms" "${FORBIDDEN_PATTERNS[@]}"

if [ "$FAIL" -eq 0 ]; then
  echo "OK: no ROS/HoloOcean/vendor includes found under core/ or algorithms/"
else
  exit 1
fi
