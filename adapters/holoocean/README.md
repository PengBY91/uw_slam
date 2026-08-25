# `adapters/holoocean` — HoloOcean sensor gateway

Real replacement for the legacy `ocean_t/` scripts (see the platform architecture's section 22.3
remediation list and `uw_holoocean_adapter/__init__.py`'s docstring for the specific audited bugs this
fixes). Produces the same canonical MCAP/protobuf bags (`schemas/proto/uw/domain/`) that
`apps/tools/synth_bag_gen` and `apps/replay_demo` use on the C++ side — a bag written here is directly
readable by `replay_demo` without translation.

## What's real vs. not tested here

This machine has no HoloOcean/Unreal install and no rclpy/ROS2 install, so `holoocean_driver.py`'s
`HoloOceanSession`, `realtime_ros_session.py`'s `RealtimeRosSession`/`main()`, and `realtime_gate.py`'s
actual process-supervision run (the parts that call into HoloOcean/rclpy or launch a real gateway binary)
are written but not exercised — same status, same reason (`realtime_gate.py`'s argument
parsing/manifest-validation/fail-closed-on-missing-binary logic IS exercised, just not a real end-to-end
run). Everything else IS tested (`pytest`, 166/166 passing as of this writing):

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
- `fault_injector.py` — `build_fault_schedule(seed, profile, duration_s)` deterministically samples a
  per-topic min-heap of drop/duplicate/bounded-reorder/outage events up front from an owned
  `numpy.random.Generator`; `FaultInjector.apply()` drains due events at runtime without ever sleeping the
  simulation loop. `apply_thruster_fault()` scales one already-shaped `PilotCommandModel` output channel
  to model a degraded thruster.
- `sensor_perturbation.py` — applies `scenario_randomization.py`'s `VisualDegradation`/`SonarDegradation`
  axes (plus the new motion-blur/particle/overexposure/stereo-mismatch/blind-zone/false-echo/range-bias
  fields that module gained for this task) to real captured image and sonar intensity arrays, using the
  caller's owned RNG — never a fresh/reseeded one.
- `async_diagnostic_recorder.py` — bounded, drop-oldest, non-blocking diagnostic tap: `try_submit()` never
  waits on the sink or the worker thread; a blocked/failed sink degrades (`stats().worker_alive` goes
  false) without ever affecting the live sensor/algorithm/HMI/pilot-command loop.
- `task_scorer.py` — `TaskScorer` is the ONE place in this entire package allowed to consume
  `/uw/sim/ground_truth` (`observe_truth`); `observe_assist` never sees it. Computes precision/recall,
  false positives/minute, bearing/range/lateral-offset P95, track-valid fraction, task success (against
  Task 1's `TaskSpec.success_conditions`), completion time, and degraded completion.
- `run_report.py` — `RunReport` (code/scenario/task/config/calibration hashes, seed, host, HoloOcean/UE
  version, sensor rates, RTF/result-age/state-age percentiles, queue stats, resource samples, health/fault
  timelines, task score) plus `GateFailure`/`evaluate_gate()`/`minimum_gate()`/`nominal_gate()`/
  `disturbed_gate()`/`overload_gate()` — the exact 350/150, 250/100, 500/200 ms fusion/state age budgets,
  1%/5% deadline-miss ceilings, 256 MiB RSS-growth ceiling and 20% CPU/GPU headroom floor the plan
  specifies, each independently boundary-tested.
- `realtime_gate.py` — supervises the four processes a gate run needs (HoloOcean session, C++ realtime
  gateway, scripted pilot, scorer), fails on any unexpected process exit, and always tears every process
  down via `ProcessGroup.stop()`'s `try`/`finally`. The scripted-pilot/scorer processes run as
  `multiprocessing.Process` targets (still genuinely separate OS processes) rather than a second CLI
  module, since Tasks 3/5 never added one and this task does not modify their files. Only the scorer
  process may ever see `/uw/sim/ground_truth`.
- `docs/traceability/rov-realtime-closed-loop.csv` — one row per `SYS-*`/`FUS-*`/`SIM-*` requirement
  extracted from the three approved specs (`docs/specifications/{rov-competition-online-system-
  requirements,rov-acoustic-optic-online-fusion-spec,holoocean-realtime-closed-loop-simulation-spec}.md`),
  checked by `tools/lint/check_realtime_traceability.py` — completeness against the real spec files is
  itself a passing test (`tests/tools/test_realtime_traceability.py`), not just a hand-maintained claim.

## Setup

```bash
uv venv .venv --python 3.11
uv pip install --python .venv/bin/python -e ".[dev]"
../../tools/codegen/gen_py.sh     # generates schema_pb2/ from schemas/proto/ — not checked in
.venv/bin/python -m pytest tests/
```
