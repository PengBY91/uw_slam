# HoloOcean Realtime Closed-Loop Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Run the approved BlueROV2/AI-D/SV1213 sensor profile through the realtime acoustic-optic assistance chain and operator overlay with deterministic scenes, fault injection, truth-isolated scoring and soak evidence.

**Architecture:** A repository-owned Python launcher creates HoloOcean from a complete versioned scenario dict, applies seeded environment changes, publishes sensor-only ROS2 topics, and keeps truth inside a scorer. A C++ ROS2 gateway converts those topics to canonical events, pushes `LiveEventSource`, runs `OnlineAssistPipeline`, and publishes a replace-latest overlay/status output.

**Tech Stack:** HoloOcean 2.3.x, Python 3.10+, NumPy, PyYAML, rclpy, ROS2 Jazzy, C++17, Protobuf, OpenCV, pytest, GoogleTest/CTest

---

**Test fixture convention:** Python tests define `REPO_ROOT`, `BASE_SCENARIO` and `SEARCH_TASK` from `Path(__file__).resolve()` and load the real files listed by the task. Every snake-case builder shown in a test is implemented in that same test module; `valid_manifest_dict()` and valid report/profile builders deep-copy real versioned inputs before mutation, while fake ROS/HoloOcean/sink types implement only the explicitly exercised protocol. C++ `MakeRawHolo*` and calibration helpers are test-local builders in the named test file and populate valid ROS headers, dimensions, encodings and timestamps before a case mutates one field.

### Task 1: Add versioned BlueROV2 sensor and task manifests

**Files:**
- Create: `adapters/holoocean/scenarios/blue_rov_aid_sv1213_base.json`
- Create: `adapters/holoocean/scenarios/aquaculture_search.yaml`
- Create: `adapters/holoocean/scenarios/structure_inspection.yaml`
- Create: `adapters/holoocean/uw_holoocean_adapter/scenario_manifest.py`
- Create: `adapters/holoocean/tests/test_scenario_manifest.py`
- Modify: `adapters/holoocean/README.md`

- [ ] **Step 1: Write failing manifest validation tests**

```python
def test_base_manifest_has_independent_20_10_50_hz_sensor_rates():
    manifest = load_realtime_manifest(BASE_SCENARIO, SEARCH_TASK)
    assert manifest.ticks_per_sec == 100
    assert manifest.sensor("LeftCamera").hz == 20
    assert manifest.sensor("RightCamera").hz == 20
    assert manifest.sensor("PilotCamera").hz == 20
    assert manifest.sensor("ImagingSonar").hz == 10
    assert manifest.sensor("VehicleOrientation").hz == 50
    assert manifest.sensor("DepthSensor").hz == 50


def test_manifest_rejects_algorithm_truth_subscription():
    data = valid_manifest_dict()
    data["algorithm_topics"].append("/uw/sim/ground_truth")
    with pytest.raises(ValueError, match="ground truth"):
        validate_realtime_manifest(data)
```

Also reject duplicate sensor names, sensor rates that do not divide `ticks_per_sec`, missing sonar calibration,
missing task success conditions and a task target without both visual and acoustic properties.

- [ ] **Step 2: Run and verify failure**

Run:

```bash
adapters/holoocean/.venv/bin/python -m pytest -q adapters/holoocean/tests/test_scenario_manifest.py
```

Expected: import failure because `scenario_manifest` and scenario files do not exist.

- [ ] **Step 3: Create the complete base scenario**

The JSON must use `ticks_per_sec: 100`, `frames_per_sec: 20`, one `BlueROV2` agent and these sensor definitions:

```json
{
  "name": "uw_bluerov_aid_sv1213_v1",
  "package_name": "Ocean",
  "world": "PierHarbor",
  "main_agent": "auv0",
  "ticks_per_sec": 100,
  "frames_per_sec": 20,
  "uw_metadata": {
    "vehicle_variant": "BlueROV2 Heavy nominal HoloOcean model",
    "thruster_count": 8,
    "dynamics_calibration_status": "nominal_not_pool_calibrated",
    "pilot_command_model": {"limit": 100.0, "deadzone": 5.0, "time_constant_s": 0.15},
    "sonar_model": {"operating_frequency_hz": 1200000.0, "calibration_status": "nominal_not_sv1213_calibrated"}
  },
  "agents": [{
    "agent_name": "auv0",
    "agent_type": "BlueROV2",
    "control_scheme": 0,
    "location": [0.0, 0.0, -3.0],
    "rotation": [0.0, 0.0, 0.0],
    "sensors": [
      {"sensor_type": "RGBCamera", "sensor_name": "LeftCamera", "socket": "CameraSocket", "location": [0.20, -0.06, 0.0], "Hz": 20, "configuration": {"CaptureWidth": 1280, "CaptureHeight": 720, "FOVAngle": 90}},
      {"sensor_type": "RGBCamera", "sensor_name": "RightCamera", "socket": "CameraSocket", "location": [0.20, 0.06, 0.0], "Hz": 20, "configuration": {"CaptureWidth": 1280, "CaptureHeight": 720, "FOVAngle": 90}},
      {"sensor_type": "RGBCamera", "sensor_name": "PilotCamera", "socket": "CameraSocket", "location": [0.18, 0.0, 0.02], "Hz": 20, "configuration": {"CaptureWidth": 1280, "CaptureHeight": 720, "FOVAngle": 100}},
      {"sensor_type": "ImagingSonar", "sensor_name": "ImagingSonar", "socket": "CameraSocket", "location": [0.25, 0.0, -0.05], "Hz": 10, "configuration": {"Azimuth": 140, "Elevation": 20, "RangeMin": 0.30, "RangeMax": 30.0, "RangeBins": 512, "AzimuthBins": 768, "AddSigma": 0.01, "MultSigma": 0.02, "RangeSigma": 0.02, "MultiPath": false, "WaterSpeedSound": 1480}},
      {"sensor_type": "OrientationSensor", "sensor_name": "VehicleOrientation", "socket": "COM", "Hz": 50},
      {"sensor_type": "IMUSensor", "sensor_name": "IMUSensor", "socket": "COM", "Hz": 50, "configuration": {"AccelSigma": [0.01, 0.01, 0.01], "AngVelSigma": [0.01, 0.01, 0.01]}},
      {"sensor_type": "DepthSensor", "sensor_name": "DepthSensor", "socket": "COM", "Hz": 50, "configuration": {"Sigma": 0.03}},
      {"sensor_type": "PoseSensor", "sensor_name": "ScoringPose", "socket": "COM", "Hz": 10}
    ]
  }]
}
```

The task YAML files must define task ID/version, arena/structure dimensions, start pose, maximum duration, props
(`box`, `sphere`, `cylinder` or `cone` only), visual material, acoustic reflectivity class, target/path truth and the
exact success metric names used by the system specification.

- [ ] **Step 4: Implement typed loading and validation**

Use frozen dataclasses `SensorSpec`, `PropSpec`, `TaskSpec`, `ActuatorModelSpec`, and `RealtimeScenarioManifest`. Validation must check
rate divisibility, required sensor names/types, camera dimensions, sonar bins/ranges/FOV, unique prop tags, allowed
prop/material values, eight-thruster metadata, bounded actuator values, a nominal-vs-pool-calibrated dynamics label,
and the truth-topic deny list. Strip repository-only `uw_metadata` before calling HoloOcean, but preserve it in the
run manifest. Return a complete scenario dict and task spec; never accept only a scenario name.

- [ ] **Step 5: Run tests and commit**

```bash
adapters/holoocean/.venv/bin/python -m pytest -q adapters/holoocean/tests/test_scenario_manifest.py
git add adapters/holoocean/scenarios adapters/holoocean/uw_holoocean_adapter/scenario_manifest.py adapters/holoocean/tests/test_scenario_manifest.py adapters/holoocean/README.md
git commit -m "feat(sim): define versioned BlueROV realtime scenarios"
```

### Task 2: Make session creation and environment randomization deterministic

**Files:**
- Modify: `adapters/holoocean/uw_holoocean_adapter/holoocean_driver.py`
- Modify: `adapters/holoocean/uw_holoocean_adapter/scenario_randomization.py`
- Create: `adapters/holoocean/tests/test_holoocean_driver.py`
- Modify: `adapters/holoocean/tests/test_scenario_randomization.py`

- [ ] **Step 1: Write failing fake-environment tests**

```python
def test_session_passes_complete_scenario_cfg_and_seeds_python_random(monkeypatch):
    fake = FakeHoloocean()
    monkeypatch.setattr(driver, "_import_holoocean", lambda: fake)
    cfg = minimal_scenario_dict()
    session = HoloOceanSession(cfg, seed=123, randomization=PRESET_TURBID)
    assert fake.make_calls == [{"scenario_cfg": cfg}]
    assert fake.env.reset_calls == 0  # holoocean.make already resets
    assert fake.env.water_fog_calls[-1][0] == pytest.approx(0.6)
    session.close()


def test_same_seed_samples_same_layout_noise_and_faults():
    a = sample_run_randomization(42, PRESET_CLEAR, PRESET_CRITICAL_DEGRADED)
    b = sample_run_randomization(42, PRESET_CLEAR, PRESET_CRITICAL_DEGRADED)
    assert a == b
    assert sample_run_randomization(43, PRESET_CLEAR, PRESET_CRITICAL_DEGRADED) != a
```

- [ ] **Step 2: Run and verify failure**

Run `adapters/holoocean/.venv/bin/python -m pytest -q adapters/holoocean/tests/test_holoocean_driver.py adapters/holoocean/tests/test_scenario_randomization.py`.

Expected: failures because session accepts only a name, resets twice and randomization is not applied.

- [ ] **Step 3: Refactor deterministic construction**

Change the constructor to:

```python
def __init__(
    self,
    scenario_cfg: dict[str, object],
    seed: int,
    randomization: ScenarioRandomization,
) -> None:
```

Save Python global random state, call `random.seed(seed)` before `holoocean.make(scenario_cfg=scenario_cfg)`, use
one owned `np.random.default_rng(seed)`, and restore the prior Python state in `close()`. Do not call a second reset.
Apply `env.water_color`, `env.water_fog`, `env.set_ocean_currents` and flashlight parameters after creation. Apply
sonar `AddSigma`, `MultSigma`, `RangeSigma`, `MultiPath` and `WaterSpeedSound` to the copied scenario dict before
`make`, because those fields are construction-time sensor configuration.

Extend `spawn_prop` to forward `sim_physics`; spawn every task prop only after the environment exists and before
the first measured tick.

- [ ] **Step 4: Run tests**

```bash
adapters/holoocean/.venv/bin/python -m pytest -q adapters/holoocean/tests/test_holoocean_driver.py adapters/holoocean/tests/test_scenario_randomization.py
```

Expected: all deterministic construction, environment command and different-seed tests pass.

- [ ] **Step 5: Commit**

```bash
git add adapters/holoocean/uw_holoocean_adapter/holoocean_driver.py adapters/holoocean/uw_holoocean_adapter/scenario_randomization.py adapters/holoocean/tests/test_holoocean_driver.py adapters/holoocean/tests/test_scenario_randomization.py
git commit -m "feat(sim): apply deterministic HoloOcean randomization"
```

### Task 3: Run pilot commands and publish sensor-only HoloOcean data through ROS2

**Files:**
- Create: `adapters/holoocean/uw_holoocean_adapter/realtime_ros_session.py`
- Create: `adapters/holoocean/uw_holoocean_adapter/ros_message_conversion.py`
- Create: `adapters/holoocean/uw_holoocean_adapter/pilot_command_model.py`
- Create: `adapters/holoocean/uw_holoocean_adapter/scripted_pilot.py`
- Create: `adapters/holoocean/tests/test_ros_message_conversion.py`
- Create: `adapters/holoocean/tests/test_realtime_ros_session.py`
- Create: `adapters/holoocean/tests/test_pilot_command_model.py`
- Create: `adapters/holoocean/tests/test_scripted_pilot.py`
- Modify: `adapters/holoocean/pyproject.toml`
- Modify: `adapters/holoocean/README.md`

- [ ] **Step 1: Write failing conversion and topic-isolation tests**

Use fake ROS message classes so Linux CI does not require rclpy:

```python
def test_vehicle_odometry_uses_noisy_state_not_scoring_pose():
    msg = vehicle_state_to_odometry(
        orientation_matrix=np.eye(3), angular_velocity=np.array([0.1, 0.2, 0.3]),
        raw_depth_z=-2.0, capture_time_s=1.5, noise=StateNoise(0.01, 0.02),
        rng=np.random.default_rng(7), message_types=fake_message_types(),
    )
    assert msg.pose.pose.position.z == pytest.approx(-2.0, abs=0.1)
    assert msg.twist.twist.angular.x != 0.0


def test_algorithm_publishers_never_include_ground_truth_topic():
    topics = build_topic_map()
    assert topics.scoring_truth == "/uw/sim/ground_truth"
    assert topics.scoring_truth not in topics.algorithm_inputs


def test_pilot_thrusters_apply_deadzone_saturation_and_delay():
    model = PilotCommandModel(limit=100.0, deadzone=5.0, time_constant_s=0.15)
    assert model.step([4.0] * 8, dt_s=0.01) == pytest.approx([0.0] * 8)
    first = model.step([120.0] * 8, dt_s=0.01)
    assert all(0.0 < value < 100.0 for value in first)
    assert all(value <= 100.0 for value in model.step([120.0] * 8, dt_s=2.0))


def test_scripted_pilot_uses_assist_only_and_stops_on_invalid_guidance():
    pilot = ScriptedPilot(search_task_spec())
    assert "/uw/sim/ground_truth" not in pilot.subscriptions()
    assert pilot.command(make_assist_status(guidance_valid=False)) == pytest.approx([0.0] * 8)
    assert pilot.command(make_assist_status(bearing_rad=0.3, range_m=6.0))[4] != 0.0
```

- [ ] **Step 2: Run and verify failure**

Run:

```bash
adapters/holoocean/.venv/bin/python -m pytest -q adapters/holoocean/tests/test_ros_message_conversion.py adapters/holoocean/tests/test_realtime_ros_session.py adapters/holoocean/tests/test_pilot_command_model.py adapters/holoocean/tests/test_scripted_pilot.py
```

Expected: import failure because modules do not exist.

- [ ] **Step 3: Implement lazy ROS imports and independent-rate publishing**

`realtime_ros_session.py` must load the complete manifest, create `HoloOceanSession`, spawn task props, step at
100 Hz and publish only when each key appears in the raw sensor frame. Subscribe to `/uw/pilot/thrusters` as eight
normalized `std_msgs/Float32MultiArray` values. Reject any other length, apply per-thruster saturation, deadzone and
first-order response delay, then pass only the filtered eight-value command to `session.step`; never set pose from a
command. A deterministic task control script may publish this topic for unattended gates, while a pilot station can
publish the same topic for interactive runs.

`scripted_pilot.py` is a simulation-test driver, not a production autonomy controller. It subscribes only
`/uw/hmi/status`, uses target bearing/range and structure path offset in bounded search/align/approach state machines,
and publishes `/uw/pilot/thrusters`. It commands zero when guidance is invalid or older than 500 ms, uses a conservative
sonar-only search mode during visual loss, and never subscribes truth. The task YAML supplies gains, bounds and success
hold times so both competition-like scenes close the loop without hard-coded truth coordinates.

```text
/holoocean/auv0/LeftCamera       sensor_msgs/Image
/holoocean/auv0/RightCamera      sensor_msgs/Image
/holoocean/auv0/PilotCamera      sensor_msgs/Image (independent pilot path)
/holoocean/auv0/ImagingSonar     holoocean_interfaces/ImagingSonar
/holoocean/auv0/VehicleState     nav_msgs/Odometry
/clock                            rosgraph_msgs/Clock
/uw/sim/ground_truth             nav_msgs/Odometry (scorer only)
```

Build `VehicleState` input from `VehicleOrientation`, `IMUSensor` angular velocity and `DepthSensor`; add seeded
orientation/depth noise plus simulated leak/power/link health before publishing. Publish `ScoringPose` only on the truth topic. All ROS imports remain
inside `main()`/factory functions so the conversion tests run without ROS2.

- [ ] **Step 4: Run tests and record the native-host command**

```bash
adapters/holoocean/.venv/bin/python -m pytest -q adapters/holoocean/tests/test_ros_message_conversion.py adapters/holoocean/tests/test_realtime_ros_session.py adapters/holoocean/tests/test_pilot_command_model.py adapters/holoocean/tests/test_scripted_pilot.py
python -m uw_holoocean_adapter.realtime_ros_session --scenario adapters/holoocean/scenarios/blue_rov_aid_sv1213_base.json --task adapters/holoocean/scenarios/aquaculture_search.yaml --seed 42 --pilot fixed-step
```

Expected: Linux fake-message tests pass. The second command is executed only on the native HoloOcean/ROS2 host
and must show the six sensor/clock topics plus the isolated truth topic at configured rates. Logged actions must
remain within the eight-thruster bounds and show non-instantaneous response to a command step.

- [ ] **Step 5: Commit**

```bash
git add adapters/holoocean/uw_holoocean_adapter/realtime_ros_session.py adapters/holoocean/uw_holoocean_adapter/ros_message_conversion.py adapters/holoocean/uw_holoocean_adapter/pilot_command_model.py adapters/holoocean/uw_holoocean_adapter/scripted_pilot.py adapters/holoocean/tests/test_ros_message_conversion.py adapters/holoocean/tests/test_realtime_ros_session.py adapters/holoocean/tests/test_pilot_command_model.py adapters/holoocean/tests/test_scripted_pilot.py adapters/holoocean/pyproject.toml adapters/holoocean/README.md
git commit -m "feat(sim): publish realtime HoloOcean sensor streams"
```

### Task 4: Convert ROS2 topics into canonical events and publish the operator overlay

**Files:**
- Create: `include/adapters/holoocean_live_conversion.hpp`
- Create: `src/adapters/holoocean_live_conversion.cpp`
- Create: `adapters/ros2/include/adapters/ros2_holoocean_realtime_gateway.hpp`
- Create: `adapters/ros2/src/holoocean_realtime_node.cpp`
- Modify: `cmake/Libraries.cmake`
- Modify: `cmake/Applications.cmake`
- Modify: `cmake/Dependencies.cmake`
- Create: `tests/adapters/holoocean_live_conversion_test.cpp`
- Modify: `cmake/Tests.cmake`
- Modify: `adapters/ros2/README.md`

- [ ] **Step 1: Write failing portable conversion tests**

```cpp
TEST(HoloOceanLiveConversion, PopulatesSequenceCalibrationAndReceiveClock) {
  RawHoloImage raw = MakeRawHoloImage(/*capture_ns=*/1'000'000'000, 1280, 720);
  const auto frame = ConvertHoloImage(raw, "camera_left", "camera_left_link", 7,
                                      "aid_sim_v1", /*receive_monotonic_ns=*/2'000'000'000);
  EXPECT_EQ(frame.header().sequence_id().value(), 7u);
  EXPECT_EQ(frame.header().calibration_version().value(), "aid_sim_v1");
  EXPECT_EQ(frame.header().receive_clock_domain(), uw::domain::CLOCK_DOMAIN_SYSTEM_MONOTONIC);
}

TEST(HoloOceanLiveConversion, SonarUsesManifestCalibrationNotHardcodedDefaults) {
  const auto frame = ConvertHoloSonar(MakeRawHoloSonar(), TestSonarCalibration30m(), 4, 9);
  EXPECT_FLOAT_EQ(frame.max_range(), 30.0f);
  EXPECT_EQ(frame.num_beams(), 768u);
  EXPECT_FLOAT_EQ(frame.sound_speed_assumption().speed_of_sound_mps(), 1480.0f);
  EXPECT_DOUBLE_EQ(frame.operating_frequency_hz(), 1'200'000.0);
}
```

- [ ] **Step 2: Build and verify failure**

Run `cmake --build build --target adapters_tests -j2`.

Expected: compile failure because live conversion does not exist.

- [ ] **Step 3: Implement portable conversion and the ROS2 node**

Portable conversion takes plain structs and returns canonical messages, allowing all field logic to be tested
without ROS2. The ROS node subscribes to the four algorithm inputs (AI-D left/right, sonar and noisy vehicle state),
converts each callback to a `CanonicalEvent`, sets `log_time_ns` to local steady receive time, and calls
`LiveEventSource::Submit`. It separately subscribes the independent `PilotCamera` for presentation only; that image
must never enter `OnlineAssistPipeline`.

Start `PumpEvents(source, online_pipeline)` on a worker thread. A ROS `AssistOutputSink` composes the latest pilot
image, sonar view and assistance state with `OperatorOverlayRenderer`, publishes `/uw/hmi/overlay` as
`sensor_msgs/Image`, and publishes compact JSON status on `/uw/hmi/status` as `std_msgs/String`. The status includes
target/path values, source, confidence, data age, discrete guidance state, every sensor's health and degradation
reason. Keep the raw `/holoocean/auv0/PilotCamera` topic directly available even if the compositor stops. Use
sensor-data QoS, depth 1 for overlay/status and no unbounded provider deque. The node must not subscribe
`/uw/sim/ground_truth`.

- [ ] **Step 4: Build portable and ROS2 variants**

```bash
cmake --build build --target adapters_tests -j2
ctest --test-dir build -R HoloOceanLiveConversion --output-on-failure
cmake -S . -B build_ros2 -DUW_BUILD_ROS2=ON -DUW_BUILD_TESTS=ON
cmake --build build_ros2 --target holoocean_realtime_node -j2
```

Expected: portable tests pass; ROS2 node builds in the Jazzy workspace with `sensor_msgs`, `nav_msgs`, `std_msgs`,
`rosgraph_msgs` and `holoocean_interfaces` available.

- [ ] **Step 5: Commit**

```bash
git add include/adapters/holoocean_live_conversion.hpp src/adapters/holoocean_live_conversion.cpp adapters/ros2/include/adapters/ros2_holoocean_realtime_gateway.hpp adapters/ros2/src/holoocean_realtime_node.cpp cmake/Libraries.cmake cmake/Applications.cmake cmake/Dependencies.cmake tests/adapters/holoocean_live_conversion_test.cpp cmake/Tests.cmake adapters/ros2/README.md
git commit -m "feat(ros2): connect HoloOcean to realtime assistance"
```

### Task 5: Add deterministic network faults and truth-isolated task scoring

**Files:**
- Create: `adapters/holoocean/uw_holoocean_adapter/fault_injector.py`
- Create: `adapters/holoocean/uw_holoocean_adapter/sensor_perturbation.py`
- Create: `adapters/holoocean/uw_holoocean_adapter/async_diagnostic_recorder.py`
- Create: `adapters/holoocean/uw_holoocean_adapter/task_scorer.py`
- Modify: `adapters/holoocean/uw_holoocean_adapter/realtime_ros_session.py`
- Create: `adapters/holoocean/tests/test_fault_injector.py`
- Create: `adapters/holoocean/tests/test_sensor_perturbation.py`
- Create: `adapters/holoocean/tests/test_async_diagnostic_recorder.py`
- Create: `adapters/holoocean/tests/test_task_scorer.py`

- [ ] **Step 1: Write failing deterministic fault and scorer tests**

```python
def test_fault_schedule_is_repeatable_and_never_reorders_without_request():
    schedule_a = build_fault_schedule(seed=42, profile=critical_profile(), duration_s=30)
    schedule_b = build_fault_schedule(seed=42, profile=critical_profile(), duration_s=30)
    assert schedule_a == schedule_b
    assert all(a.release_time_s <= b.release_time_s for a, b in pairwise(schedule_a))


def test_scorer_consumes_truth_but_algorithm_topic_list_does_not():
    scorer = TaskScorer(search_task_spec())
    scorer.observe_truth(make_truth_pose(), capture_time_s=1.0)
    scorer.observe_assist(make_correct_assist_track(), receive_time_s=1.2)
    assert scorer.report().task_success
    assert "/uw/sim/ground_truth" not in scorer.algorithm_topics


def test_blocked_or_failed_recorder_never_blocks_live_submit():
    recorder = AsyncDiagnosticRecorder(capacity=2, sink=BlockingThenFailingSink())
    started = time.monotonic()
    for sequence in range(100):
        recorder.try_submit(make_record(sequence))
    assert time.monotonic() - started < 0.05
    assert recorder.stats().dropped_oldest > 0
```

- [ ] **Step 2: Run and verify failure**

Run `adapters/holoocean/.venv/bin/python -m pytest -q adapters/holoocean/tests/test_fault_injector.py adapters/holoocean/tests/test_sensor_perturbation.py adapters/holoocean/tests/test_async_diagnostic_recorder.py adapters/holoocean/tests/test_task_scorer.py`.

Expected: import failure because fault injector and scorer do not exist.

- [ ] **Step 3: Implement scheduled faults and metric definitions**

The fault injector owns a seeded RNG and a min-heap keyed by release time. It supports fixed clock offset, jitter,
camera-sonar desync, drop, duplicate, bounded reorder and finite outage intervals per topic. It never sleeps the
simulation loop; ready messages are published from the heap.

`sensor_perturbation.py` uses the run RNG to apply configured underwater image color attenuation, haze/backscatter,
motion blur, particles, local overexposure and left/right exposure mismatch. Its sonar path applies speckle,
additive/multiplicative noise, gain changes, blind zone, false/sidelobe echoes and range-scale bias. Apply a configured
single-thruster effectiveness multiplier in the pilot command model and route current changes through
`set_ocean_currents`; log every active perturbation. Golden-array tests prove same-seed identity, different-seed
change, valid output shapes and non-clean critical profiles.

`AsyncDiagnosticRecorder` is an optional bounded, drop-oldest tap feeding a dedicated worker/process. `try_submit`
never waits for disk; ENOSPC, slow writes and worker exit transition recorder health and increment counters without
changing sensor, algorithm, HMI or pilot-command scheduling. MCAP output remains diagnostic evidence only.

The scorer alone subscribes to truth and `/uw/hmi/status`. It computes detection precision/recall, false positives
per minute, bearing/range P95, structure lateral-offset P95, track-valid fraction, task success, completion time and
degraded completion. It writes a JSON report that identifies task version and seed.

- [ ] **Step 4: Run tests and commit**

```bash
adapters/holoocean/.venv/bin/python -m pytest -q adapters/holoocean/tests/test_fault_injector.py adapters/holoocean/tests/test_sensor_perturbation.py adapters/holoocean/tests/test_async_diagnostic_recorder.py adapters/holoocean/tests/test_task_scorer.py
git add adapters/holoocean/uw_holoocean_adapter/fault_injector.py adapters/holoocean/uw_holoocean_adapter/sensor_perturbation.py adapters/holoocean/uw_holoocean_adapter/async_diagnostic_recorder.py adapters/holoocean/uw_holoocean_adapter/task_scorer.py adapters/holoocean/uw_holoocean_adapter/realtime_ros_session.py adapters/holoocean/tests/test_fault_injector.py adapters/holoocean/tests/test_sensor_perturbation.py adapters/holoocean/tests/test_async_diagnostic_recorder.py adapters/holoocean/tests/test_task_scorer.py
git commit -m "feat(sim): inject faults and score realtime tasks"
```

### Task 6: Produce manifests, realtime metrics and minimum/nominal/overload gates

**Files:**
- Create: `adapters/holoocean/uw_holoocean_adapter/realtime_gate.py`
- Create: `adapters/holoocean/uw_holoocean_adapter/run_report.py`
- Create: `adapters/holoocean/tests/test_run_report.py`
- Create: `docs/traceability/rov-realtime-closed-loop.csv`
- Create: `tools/lint/check_realtime_traceability.py`
- Create: `tests/tools/test_realtime_traceability.py`
- Create: `configs/experiment/rov_realtime_minimum.yaml`
- Create: `configs/experiment/rov_realtime_nominal.yaml`
- Create: `configs/experiment/rov_realtime_disturbed.yaml`
- Create: `configs/experiment/rov_realtime_overload.yaml`
- Modify: `docs/testing-and-verification-guide-2026-08-20.md`
- Modify: `adapters/holoocean/README.md`

- [ ] **Step 1: Write failing report-gate tests**

```python
def test_nominal_gate_rejects_old_results_and_memory_growth():
    report = valid_nominal_report()
    report["result_age_p95_ms"] = 251.0
    with pytest.raises(GateFailure, match="result_age_p95_ms"):
        evaluate_gate(report, nominal_gate())
    report = valid_nominal_report()
    report["rss_growth_after_warmup_mib"] = 257.0
    with pytest.raises(GateFailure, match="rss_growth"):
        evaluate_gate(report, nominal_gate())
```

Also gate RTF P95 ≥1.0, deadline misses ≤1%, no capacity violations, recovery ≤2 s, nonzero sonar/visual/fused
tracks, 2-hour duration, CPU/GPU average headroom ≥20%, and task success counts. Add boundary tests for minimum
fusion/state age P95 `350/150 ms`, nominal `250/100 ms`, overload hard limits `500/200 ms`, overload deadline
misses `5%`, 256 MiB post-warmup RSS growth, and mandatory stale guidance above 500 ms.

Add a traceability-lint test that extracts every `SYS-*`, `FUS-*` and `SIM-*` requirement from the three approved
specifications and requires exactly one CSV row with non-empty scenario, implementation module, test, evidence path
and enum status. Hardware `SYS-PROC-*` and `SIM-S2R-001` rows use status `gated` until their external evidence exists;
simulation execution must not mark them verified.

- [ ] **Step 2: Run and verify failure**

Run:

```bash
adapters/holoocean/.venv/bin/python -m pytest -q adapters/holoocean/tests/test_run_report.py
python -m pytest -q tests/tools/test_realtime_traceability.py
```

Expected: import failure because report/gate modules do not exist.

- [ ] **Step 3: Implement the run report and three profiles**

The report must contain code/scenario/task/config/calibration hashes, seed, host, HoloOcean/UE version, sensor
rates, RTF P50/P95, result age P50/P95/P99, deadline misses, queue high-water/drops/rejects, health timeline, fault
timeline, recovery durations, CPU/GPU/RSS samples and task score. Nominal profile is 20/10/50 Hz; ten-seed task
scoring uses each task manifest's maximum duration, while one separate nominal soak runs for 7200 s. Minimum is
720p stereo at 15 Hz, sonar 5 Hz and state 20 Hz for 1800 s; overload is 1.25× nominal image payload,
sonar 20 Hz and state 100 Hz with faults for 1800 s. Output must be at least 5 Hz for minimum and 10 Hz or
new-valid-observation-triggered for nominal; repeat-publishing an unchanged old state does not count.
The disturbed profile keeps nominal rates but enables the required optical, acoustic, timing, current and
single-thruster perturbation matrix for 10-seed task scoring.
`realtime_gate` supervises four isolated processes—HoloOcean session, C++ realtime gateway/algorithm/HMI, scripted
pilot, and scorer—fails if any required process exits, and always tears them down deterministically. Only the scorer
receives truth; the scripted pilot consumes `/uw/hmi/status` and closes the command loop through
`/uw/pilot/thrusters`.
Populate `docs/traceability/rov-realtime-closed-loop.csv` from the implementation just completed. Allowed statuses
are `implemented`, `verified`, `gated`, and `failed`; `verified` requires an existing evidence artifact named in the
row, while Linux-only tests cannot verify native HoloOcean gates.

- [ ] **Step 4: Run Linux tests and native-host gates**

```bash
adapters/holoocean/.venv/bin/python -m pytest -q adapters/holoocean/tests
python -m pytest -q tests/tools/test_realtime_traceability.py
python tools/lint/check_realtime_traceability.py docs/traceability/rov-realtime-closed-loop.csv
python -m uw_holoocean_adapter.realtime_gate --profile configs/experiment/rov_realtime_minimum.yaml --task adapters/holoocean/scenarios/aquaculture_search.yaml --seed 42
python -m uw_holoocean_adapter.realtime_gate --profile configs/experiment/rov_realtime_nominal.yaml --task adapters/holoocean/scenarios/aquaculture_search.yaml --seeds 40 41 42 43 44 45 46 47 48 49
python -m uw_holoocean_adapter.realtime_gate --profile configs/experiment/rov_realtime_nominal.yaml --task adapters/holoocean/scenarios/structure_inspection.yaml --seeds 40 41 42 43 44 45 46 47 48 49
python -m uw_holoocean_adapter.realtime_gate --profile configs/experiment/rov_realtime_nominal.yaml --task adapters/holoocean/scenarios/structure_inspection.yaml --seed 42 --soak-duration-s 7200
python -m uw_holoocean_adapter.realtime_gate --profile configs/experiment/rov_realtime_disturbed.yaml --task adapters/holoocean/scenarios/aquaculture_search.yaml --seeds 50 51 52 53 54 55 56 57 58 59
python -m uw_holoocean_adapter.realtime_gate --profile configs/experiment/rov_realtime_disturbed.yaml --task adapters/holoocean/scenarios/structure_inspection.yaml --seeds 50 51 52 53 54 55 56 57 58 59
python -m uw_holoocean_adapter.realtime_gate --profile configs/experiment/rov_realtime_overload.yaml --task adapters/holoocean/scenarios/structure_inspection.yaml --seed 42
```

Expected: Linux adapter tests pass. Native-host commands produce manifest, metrics, score and HMI recording; the
minimum command passes the 350/150 ms release gate, nominal passes the 250/100 ms target plus 2-hour resource gate
and at least 8/10 seeds for each task, disturbed seeds reach at least 7/10 for each task, and overload passes the
500/200 ms, 5% deadline, 30-minute bounded-queue/recovery gates. If HoloOcean cannot sustain RTF 1.0, report the
actual failed gate; do not lower the spec.

- [ ] **Step 5: Run repository regression and commit**

```bash
ctest --test-dir build --output-on-failure
git add adapters/holoocean/uw_holoocean_adapter/realtime_gate.py adapters/holoocean/uw_holoocean_adapter/run_report.py adapters/holoocean/tests/test_run_report.py docs/traceability/rov-realtime-closed-loop.csv tools/lint/check_realtime_traceability.py tests/tools/test_realtime_traceability.py configs/experiment/rov_realtime_minimum.yaml configs/experiment/rov_realtime_nominal.yaml configs/experiment/rov_realtime_disturbed.yaml configs/experiment/rov_realtime_overload.yaml docs/testing-and-verification-guide-2026-08-20.md adapters/holoocean/README.md
git commit -m "test(sim): gate HoloOcean realtime closed loop"
```
