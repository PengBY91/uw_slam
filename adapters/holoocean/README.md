# `adapters/holoocean` — HoloOcean sensor gateway

Real replacement for the legacy `ocean_t/` scripts (see the platform architecture's section 22.3
remediation list and `uw_holoocean_adapter/__init__.py`'s docstring for the specific audited bugs this
fixes). Produces the same canonical MCAP/protobuf bags (`schemas/proto/uw/domain/`) that
`apps/tools/synth_bag_gen` and `apps/replay_demo` use on the C++ side — a bag written here is directly
readable by `replay_demo` without translation.

## What's real vs. not tested here

This machine has no HoloOcean/Unreal install and no rclpy/ROS2 install, so `holoocean_driver.py`'s
`HoloOceanSession` and `realtime_ros_session.py`'s `RealtimeRosSession`/`main()` (the parts that actually
call into HoloOcean/rclpy) are written but not exercised — same status, same reason. Everything else IS
tested (`pytest`, 111/111 passing as of this writing):

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
- `pilot_command_model.py` — `PilotCommandModel` shapes raw thruster commands (deadzone, saturation to
  `ActuatorModelSpec.limit`, first-order lag toward the target at `time_constant_s`) for both the realtime
  ROS gateway's `/uw/pilot/thrusters` handling and `scripted_pilot.py`.
- `ros_message_conversion.py` — portable (no rclpy import) conversion of raw HoloOcean sensor readings
  into `sensor_msgs/Image`, `holoocean_interfaces/msg/ImagingSonar`, `nav_msgs/Odometry`
  (`VehicleState`, built only from `VehicleOrientation`+`IMUSensor`+`DepthSensor` — never the ground-truth
  `PoseSensor`) and `rosgraph_msgs/Clock` message shapes, plus `build_topic_map()` — the one place that
  defines every realtime closed-loop topic name and which four are `algorithm_inputs` (never
  `PilotCamera`, `/clock`, or the truth topic).
- `scripted_pilot.py` — `ScriptedPilot` is a bounded simulation-test driver (not production autonomy):
  subscribes only `/uw/hmi/status`, commands all-zero thrust when guidance is invalid or stale (>500ms),
  and runs a conservative sonar-only search mode when the active track's source is `SONAR` (visual loss).
- `realtime_ros_session.py` — `build_realtime_messages()` is the portable core (given one already-obtained
  `RawSensorFrame`, decides which topics need a new message this tick, same independent-per-sensor-rate
  logic as `record_session.py`); `RealtimeRosSession`/`main()` wrap it with a real `HoloOceanSession` +
  rclpy node (lazy-imported, untested here — see above).

## Setup

```bash
uv venv .venv --python 3.11
uv pip install --python .venv/bin/python -e ".[dev]"
../../tools/codegen/gen_py.sh     # generates schema_pb2/ from schemas/proto/ — not checked in
.venv/bin/python -m pytest tests/
```
