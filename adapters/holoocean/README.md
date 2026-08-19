# `adapters/holoocean` — HoloOcean sensor gateway

Real replacement for the legacy `ocean_t/` scripts (see the platform architecture's section 22.3
remediation list and `uw_holoocean_adapter/__init__.py`'s docstring for the specific audited bugs this
fixes). Produces the same canonical MCAP/protobuf bags (`schemas/proto/uw/domain/`) that
`apps/tools/synth_bag_gen` and `apps/replay_demo` use on the C++ side — a bag written here is directly
readable by `replay_demo` without translation.

## What's real vs. not tested here

This machine has no HoloOcean/Unreal install, so `holoocean_driver.py`'s `HoloOceanSession` (the part
that actually calls into HoloOcean) is written but not exercised. Everything else IS tested (`pytest`,
9/9 passing as of this writing):

- `coordinates.py` — UE↔body coordinate transforms, quaternion-based (fixes the Euler gimbal-lock
  branch found in `ocean_t`'s `CoordTransformer._SE3_to_pose`).
- `canonical_writer.py` — MCAP writer/reader matching `runtime/include/uw/runtime/mcap_io.hpp`'s wire
  format exactly (uncompressed, so bags stay cross-language readable with the C++ side's
  zstd/lz4-disabled MCAP build).
- `scenario_randomization.py` — the programmable multi-axis randomization API that replaces
  `water_control_panel.py`'s two-slider GUI panel.
- `time_utils.py` — capture/receive time separation.

## Setup

```bash
uv venv .venv --python 3.11
uv pip install --python .venv/bin/python -e ".[dev]"
../../tools/codegen/gen_py.sh     # generates schema_pb2/ from schemas/proto/ — not checked in
.venv/bin/python -m pytest tests/
```
