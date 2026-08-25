# `adapters/holoocean` — HoloOcean sensor gateway

Real replacement for the legacy `ocean_t/` scripts (see the platform architecture's section 22.3
remediation list and `uw_holoocean_adapter/__init__.py`'s docstring for the specific audited bugs this
fixes). Produces the same canonical MCAP/protobuf bags (`schemas/proto/uw/domain/`) that
`apps/tools/synth_bag_gen` and `apps/replay_demo` use on the C++ side — a bag written here is directly
readable by `replay_demo` without translation.

## What's real vs. not tested here

This machine has no HoloOcean/Unreal install, so `holoocean_driver.py`'s `HoloOceanSession` (the part
that actually calls into HoloOcean) is written but not exercised. Everything else IS tested (`pytest`,
71/71 passing as of this writing):

- `coordinates.py` — UE↔body coordinate transforms, quaternion-based (fixes the Euler gimbal-lock
  branch found in `ocean_t`'s `CoordTransformer._SE3_to_pose`).
- `canonical_writer.py` — MCAP writer/reader matching `runtime/include/uw/runtime/mcap_io.hpp`'s wire
  format exactly (uncompressed, so bags stay cross-language readable with the C++ side's
  zstd/lz4-disabled MCAP build).
- `scenario_randomization.py` — the programmable multi-axis randomization API that replaces
  `water_control_panel.py`'s two-slider GUI panel.
- `time_utils.py` — capture/receive time separation.
- `record_session.py` — a "keyframe" (stereo pair + the GT/depth evidence keyed off it) forms only on
  ticks where both cameras publish, but sonar/IMU/DVL (and even a lone/monocular camera) are written on
  ANY tick where HoloOcean published them, independent of the camera pair or of each other — matching
  real hardware running each sensor at its own rate. Each of those independently-written sensors'
  `observation_id` is keyed on its own raw simulation-tick index, never on the (much lower-rate) camera
  keyframe counter — see `tests/test_record_session.py`.
- `scenario_manifest.py` — loads/validates the versioned realtime scenario+task manifests under
  `scenarios/` (`blue_rov_aid_sv1213_base.json` for the BlueROV2/AI-D/SV1213 sensor rig,
  `aquaculture_search.yaml`/`structure_inspection.yaml` for the two task truth definitions) into a typed
  `RealtimeScenarioManifest`. Fails fast on duplicate/misrated sensors, missing sonar calibration fields,
  unknown prop types/materials, a task target missing either visual or acoustic properties, and — the
  check most worth calling out — any `algorithm_topics` entry equal to the ground-truth topic
  (`/uw/sim/ground_truth` must never reach the algorithm/pipeline side, only the scorer). Repo-only keys
  (`uw_metadata`, `algorithm_topics`) live alongside HoloOcean's own scenario fields in the same JSON file
  and are stripped by `RealtimeScenarioManifest.holoocean_scenario_cfg()` before the dict is safe to pass
  to `holoocean.make(scenario_cfg=...)`.

## Setup

```bash
uv venv .venv --python 3.11
uv pip install --python .venv/bin/python -e ".[dev]"
../../tools/codegen/gen_py.sh     # generates schema_pb2/ from schemas/proto/ — not checked in
.venv/bin/python -m pytest tests/
```
