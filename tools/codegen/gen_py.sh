#!/usr/bin/env bash
# Regenerates Python protobuf bindings for schemas/proto/uw/domain/ into
# adapters/holoocean/uw_holoocean_adapter/schema_pb2/. Generated files are
# gitignored (**/*_pb2.py) — this is a dev-setup step, not a checked-in
# artifact, so the .proto files stay the single source of truth (no risk of
# a stale generated copy drifting from schemas/proto/).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT_DIR="$ROOT/adapters/holoocean/uw_holoocean_adapter/schema_pb2"

command -v protoc >/dev/null 2>&1 || {
  echo "protoc not found on PATH — run tools/setup_dev_env.sh first (and activate the" >&2
  echo "conda env it creates, if apt wasn't usable) before running this script." >&2
  exit 1
}

mkdir -p "$OUT_DIR"
protoc -I "$ROOT/schemas/proto" --python_out="$OUT_DIR" "$ROOT"/schemas/proto/uw/domain/*.proto
echo "OK: generated Python bindings under $OUT_DIR"
