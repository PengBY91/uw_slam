import pathlib
import sys

# Generated protobuf bindings live under uw_holoocean_adapter/schema_pb2/
# and use absolute imports like `from uw.domain import observation_pb2`
# (protoc generates paths relative to the -I root passed in
# tools/codegen/gen_py.sh) — so the schema_pb2 directory itself, not the
# package root, needs to be on sys.path.
_SCHEMA_PB2_DIR = pathlib.Path(__file__).parent.parent / "uw_holoocean_adapter" / "schema_pb2"
if str(_SCHEMA_PB2_DIR) not in sys.path:
    sys.path.insert(0, str(_SCHEMA_PB2_DIR))
