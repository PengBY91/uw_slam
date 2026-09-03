"""PREP-A-03 (docs/ROV平台到货前准备工作规格-2026-09-02.md): the contract
vehicle's digital-twin scenario baselines -- mono (stage 1) and stereo
(stage 2) derived from one base file, with fidelity/realtime profiles
(SIM-PERF-002/003) and 750 kHz / 1.2 MHz sonar modes (SIM-SON-004) as
overlays -- plus the session-side wiring they need: MainCamera / IMU
publishing (SIM-CAM-001 stage 1, SIM-IMU-001), heading noise + thrust
bias on VehicleState (SIM-STATE-002), and manifest-derived publishers.

Nothing here touches HoloOcean or rclpy; the 5-minute acceptance runs of
the spec still have to happen on the native Windows host."""
import copy
import json
import pathlib
import tempfile

import numpy as np
import pytest
from uw.domain import image_pb2, imu_pb2

from uw_holoocean_adapter.canonical_writer import read_canonical_messages
from uw_holoocean_adapter.holoocean_driver import RawSensorFrame
from uw_holoocean_adapter.realtime_ros_session import build_publisher_table, build_realtime_messages
from uw_holoocean_adapter.record_session import record_frames
from uw_holoocean_adapter.ros_message_conversion import (
    HeadingNoise,
    StateNoise,
    build_topic_map,
    thrust_fraction_of,
    vehicle_state_to_odometry,
)
from uw_holoocean_adapter.scenario_manifest import (
    REPO_ONLY_SCENARIO_KEYS,
    load_realtime_manifest,
    resolve_scenario_overlays,
    validate_realtime_manifest,
)

from test_record_session import _MODULES, _camera_array  # noqa: E402  (shared helpers, same dir)
from test_ros_message_conversion import fake_message_types  # noqa: E402

REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
SCENARIOS = REPO_ROOT / "adapters/holoocean/scenarios"
BASE = SCENARIOS / "blue_rov_contract_base.json"
MONO = SCENARIOS / "blue_rov_contract_mono.json"
STEREO = SCENARIOS / "blue_rov_contract_stereo.json"
LEGACY = SCENARIOS / "blue_rov_aid_sv1213_base.json"
TASK = SCENARIOS / "aquaculture_search.yaml"


# --- manifest: derivation, profiles, sonar modes ---------------------------


def test_mono_manifest_carries_main_camera_only_and_mono_algorithm_topics():
    manifest = load_realtime_manifest(MONO, TASK)
    names = [s.sensor_name for s in manifest.sensors]
    assert "MainCamera" in names
    assert "LeftCamera" not in names and "RightCamera" not in names
    assert manifest.algorithm_topics == (
        "/holoocean/auv0/MainCamera",
        "/holoocean/auv0/ImagingSonar",
        "/holoocean/auv0/VehicleState",
        "/holoocean/auv0/IMU",
    )
    assert manifest.uw_metadata["vehicle_variant"] == "BlueROV2 Heavy contract 2026-09"
    assert manifest.dynamics_calibration_status == "nominal_not_pool_calibrated"
    assert manifest.actuator_model.thruster_count == 8


def test_stereo_manifest_adds_the_pair_on_top_of_the_mono_set():
    mono = load_realtime_manifest(MONO, TASK)
    stereo = load_realtime_manifest(STEREO, TASK)
    mono_names = {s.sensor_name for s in mono.sensors}
    stereo_names = {s.sensor_name for s in stereo.sensors}
    assert stereo_names == mono_names | {"LeftCamera", "RightCamera"}
    assert set(stereo.algorithm_topics) == set(mono.algorithm_topics) | {
        "/holoocean/auv0/LeftCamera",
        "/holoocean/auv0/RightCamera",
    }
    # PREP-B-08 placeholder geometry: 10 cm pure-y baseline.
    left = np.asarray(stereo.sensor("LeftCamera").location_m)
    right = np.asarray(stereo.sensor("RightCamera").location_m)
    baseline = right - left
    assert baseline[1] == pytest.approx(0.10)
    assert baseline[0] == 0.0 and baseline[2] == 0.0


def test_fidelity_profile_is_the_default_and_matches_sim_perf_003():
    manifest = load_realtime_manifest(STEREO, TASK)
    assert manifest.profile == "fidelity"
    assert manifest.ticks_per_sec == 200
    assert manifest.frames_per_sec is False
    assert manifest.sensor("IMUSensor").hz == 200
    assert manifest.sensor("ImagingSonar").hz == 40
    assert manifest.sensor("ImagingSonar").configuration["RangeBins"] == 512
    assert manifest.sensor("ImagingSonar").configuration["AzimuthBins"] == 768
    for camera in ("MainCamera", "LeftCamera", "RightCamera"):
        spec = manifest.sensor(camera)
        assert spec.hz == 25  # 30 fps is not an integer tick divisor (SIM-PERF-001)
        assert (spec.configuration["CaptureWidth"], spec.configuration["CaptureHeight"]) == (1920, 1080)


def test_realtime_profile_matches_sim_perf_002_including_the_12_5_hz_camera():
    manifest = load_realtime_manifest(MONO, TASK, profile="realtime")
    assert manifest.profile == "realtime"
    assert manifest.ticks_per_sec == 25
    assert manifest.frames_per_sec is True
    camera = manifest.sensor("MainCamera")
    # 12.5 Hz is not an integer rate but IS an exact tick divisor (25 / 12.5 == 2);
    # the divisibility check is ratio-based so this passes deterministically.
    assert camera.hz == 12.5
    assert (camera.configuration["CaptureWidth"], camera.configuration["CaptureHeight"]) == (960, 540)
    assert manifest.sensor("ImagingSonar").hz == 5
    assert manifest.sensor("ImagingSonar").configuration["RangeBins"] == 256
    assert manifest.sensor("IMUSensor").hz == 25
    assert manifest.sensor("VehicleOrientation").hz == 25
    assert manifest.sensor("DepthSensor").hz == 25


def test_rate_check_rejects_a_non_divisor_float_rate():
    data = copy.deepcopy(load_realtime_manifest(MONO, TASK, profile="realtime").validation_data())
    for sensor in data["scenario"]["agents"][0]["sensors"]:
        if sensor["sensor_name"] == "MainCamera":
            sensor["Hz"] = 12.4
    with pytest.raises(ValueError, match="divide"):
        validate_realtime_manifest(data)


def test_unknown_profile_and_profile_on_legacy_file_are_errors():
    with pytest.raises(ValueError, match="unknown profile"):
        load_realtime_manifest(MONO, TASK, profile="turbo")
    with pytest.raises(ValueError, match="no uw_profiles"):
        load_realtime_manifest(LEGACY, TASK, profile="fidelity")
    legacy = load_realtime_manifest(LEGACY, TASK)
    assert legacy.profile is None and legacy.sonar_mode is None
    assert legacy.ticks_per_sec == 100  # untouched


def test_sonar_modes_swap_range_and_noise_floor_but_keep_768_beams():
    default = load_realtime_manifest(MONO, TASK)
    assert default.sonar_mode == "sonar_1200khz"
    assert default.sensor("ImagingSonar").configuration["RangeMax"] == 50.0
    assert default.sonar_operating_frequency_hz == 1_200_000.0

    low = load_realtime_manifest(MONO, TASK, sonar_mode="sonar_750khz")
    cfg = low.sensor("ImagingSonar").configuration
    assert cfg["RangeMax"] == 120.0
    assert cfg["AzimuthBins"] == 768  # beam count, NOT beam width (SIM-SON-004)
    assert cfg["AddSigma"] > default.sensor("ImagingSonar").configuration["AddSigma"]
    assert cfg["RangeSigma"] > default.sensor("ImagingSonar").configuration["RangeSigma"]
    assert low.sonar_operating_frequency_hz == 750_000.0
    with pytest.raises(ValueError, match="unknown sonar mode"):
        load_realtime_manifest(MONO, TASK, sonar_mode="sonar_2mhz")


def test_holoocean_cfg_is_stripped_of_every_repo_only_key_after_overlays():
    manifest = load_realtime_manifest(STEREO, TASK, profile="realtime", sonar_mode="sonar_750khz")
    cfg = manifest.holoocean_scenario_cfg()
    assert not any(key in cfg for key in REPO_ONLY_SCENARIO_KEYS)
    # The overlay result -- not the base values -- is what HoloOcean sees.
    assert cfg["ticks_per_sec"] == 25
    sonar = next(s for s in cfg["agents"][0]["sensors"] if s["sensor_name"] == "ImagingSonar")
    assert sonar["configuration"]["RangeMax"] == 120.0 and sonar["configuration"]["RangeBins"] == 256


def test_resolve_overlays_is_pure():
    scenario = json.loads(BASE.read_text())
    before = copy.deepcopy(scenario)
    resolve_scenario_overlays(scenario, profile="realtime", sonar_mode="sonar_750khz")
    assert scenario == before


def test_camera_rules_reject_no_camera_and_a_lone_stereo_side():
    data = copy.deepcopy(load_realtime_manifest(STEREO, TASK).validation_data())
    sensors = data["scenario"]["agents"][0]["sensors"]
    data["scenario"]["agents"][0]["sensors"] = [s for s in sensors if s["sensor_name"] != "RightCamera"]
    with pytest.raises(ValueError, match="both present"):
        validate_realtime_manifest(data)

    data = copy.deepcopy(load_realtime_manifest(MONO, TASK).validation_data())
    sensors = data["scenario"]["agents"][0]["sensors"]
    data["scenario"]["agents"][0]["sensors"] = [s for s in sensors if s["sensor_name"] != "MainCamera"]
    with pytest.raises(ValueError, match="at least one algorithm camera"):
        validate_realtime_manifest(data)


def test_extends_rejects_unknown_subset_names(tmp_path):
    derived = tmp_path / "bad.json"
    derived.write_text(json.dumps({
        "uw_extends": str(BASE),
        "uw_sensor_subset": ["MainCamera", "NoSuchSensor"],
    }))
    with pytest.raises(ValueError, match="NoSuchSensor"):
        load_realtime_manifest(derived, TASK)


def test_contract_imu_carries_bias_sigmas_and_the_sonar_matches_the_contract_geometry():
    manifest = load_realtime_manifest(MONO, TASK)
    imu = manifest.sensor("IMUSensor").configuration
    for key in ("AccelSigma", "AngVelSigma", "AccelBiasSigma", "AngVelBiasSigma"):
        assert len(imu[key]) == 3 and all(v > 0 for v in imu[key])
    sonar = manifest.sensor("ImagingSonar").configuration
    assert (sonar["Azimuth"], sonar["Elevation"], sonar["AzimuthBins"]) == (140, 20, 768)
    # The orientation truth sensor stays in the file for VehicleState but is
    # never an algorithm topic (SIM-STATE-002).
    assert manifest.has_sensor("VehicleOrientation")
    assert not any("Orientation" in t for t in manifest.algorithm_topics)


# --- session wiring --------------------------------------------------------


def _mono_tick_frame(sim_time_s: float = 1.0) -> RawSensorFrame:
    return RawSensorFrame(
        sim_time_s=sim_time_s,
        receive_time_s=sim_time_s,
        sensors={
            "MainCamera": np.zeros((2, 2, 4), dtype=np.uint8),
            "IMUSensor": np.array([[0.1, 0.2, 9.81], [0.01, 0.02, 0.03]]),
            "VehicleOrientation": np.eye(3),
            "DepthSensor": np.array([-3.0]),
        },
    )


def test_session_publishes_main_camera_and_imu_on_their_own_topics():
    manifest = load_realtime_manifest(MONO, TASK)
    topics = build_topic_map(manifest.agent_name, manifest.algorithm_topics)
    messages = build_realtime_messages(_mono_tick_frame(), topics, fake_message_types(), rng=np.random.default_rng(1))
    by_topic = {topic: msg for topic, msg in messages}
    assert topics.main_camera == "/holoocean/auv0/MainCamera"
    assert by_topic[topics.main_camera].header.frame_id == "camera_main_link"
    imu = by_topic[topics.imu]
    assert topics.imu == "/holoocean/auv0/IMU"
    assert imu.header.frame_id == "imu_link"
    assert (imu.linear_acceleration.x, imu.linear_acceleration.z) == (pytest.approx(0.1), pytest.approx(9.81))
    assert imu.angular_velocity.z == pytest.approx(0.03)
    assert imu.orientation_covariance[0] == -1.0
    assert topics.left_camera not in by_topic and topics.right_camera not in by_topic
    assert set(topics.algorithm_inputs) == set(manifest.algorithm_topics)


def test_imu_reading_without_an_imu_message_type_is_a_loud_error():
    from uw_holoocean_adapter.ros_message_conversion import RosMessageTypes
    from test_ros_message_conversion import FakeClock, FakeImage, FakeImagingSonar, FakeOdometry

    legacy_types = RosMessageTypes(Image=FakeImage, Odometry=FakeOdometry, ImagingSonar=FakeImagingSonar, Clock=FakeClock)
    with pytest.raises(ValueError, match="Imu"):
        build_realtime_messages(_mono_tick_frame(), build_topic_map(), legacy_types, rng=np.random.default_rng(1))


def test_vehicle_state_uses_the_last_imu_sample_on_ticks_where_the_imu_did_not_fire():
    frame = RawSensorFrame(
        sim_time_s=1.0, receive_time_s=1.0,
        sensors={"VehicleOrientation": np.eye(3), "DepthSensor": np.array([-2.0])},
    )
    topics = build_topic_map()
    without = build_realtime_messages(frame, topics, fake_message_types(), rng=np.random.default_rng(1))
    assert topics.vehicle_state not in {t for t, _ in without}
    with_fallback = build_realtime_messages(
        frame, topics, fake_message_types(), rng=np.random.default_rng(1),
        fallback_angular_velocity=np.array([0.0, 0.0, 0.5]),
    )
    state = dict(with_fallback)[topics.vehicle_state]
    assert state.twist.twist.angular.z == pytest.approx(0.5, abs=0.05)


def _yaw_deg_of(msg) -> float:
    o = msg.pose.pose.orientation
    x, y, z, w = o.x, o.y, o.z, o.w
    # yaw from quaternion (xyzw), no Euler state anywhere -- test-only readout
    return float(np.degrees(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))))


def test_heading_noise_adds_a_thrust_proportional_yaw_bias():
    kwargs = dict(
        orientation_matrix=np.eye(3), angular_velocity=np.zeros(3), raw_depth_z=-2.0, capture_time_s=1.0,
        noise=StateNoise(0.0, 0.0), message_types=fake_message_types(),
    )
    heading = HeadingNoise(sigma_rad=0.0, bias_rad_per_full_thrust=np.radians(3.0))
    idle = vehicle_state_to_odometry(rng=np.random.default_rng(1), heading_noise=heading, thrust_fraction=0.0, **kwargs)
    full = vehicle_state_to_odometry(rng=np.random.default_rng(1), heading_noise=heading, thrust_fraction=1.0, **kwargs)
    half = vehicle_state_to_odometry(rng=np.random.default_rng(1), heading_noise=heading, thrust_fraction=0.5, **kwargs)
    assert _yaw_deg_of(idle) == pytest.approx(0.0, abs=1e-6)
    assert _yaw_deg_of(full) == pytest.approx(3.0, abs=1e-6)
    assert _yaw_deg_of(half) == pytest.approx(1.5, abs=1e-6)
    # Pitch/roll untouched: the bias is a heading error about world z only.
    assert full.pose.pose.orientation.x == pytest.approx(0.0, abs=1e-9)
    assert full.pose.pose.orientation.y == pytest.approx(0.0, abs=1e-9)


def test_heading_noise_sigma_is_one_degree_scale_and_seed_deterministic():
    kwargs = dict(
        orientation_matrix=np.eye(3), angular_velocity=np.zeros(3), raw_depth_z=-2.0, capture_time_s=1.0,
        noise=StateNoise(0.0, 0.0), message_types=fake_message_types(),
    )
    heading = HeadingNoise()  # spec defaults: sigma 1 deg, 3 deg per full thrust
    rng = np.random.default_rng(42)
    yaws = np.array([_yaw_deg_of(vehicle_state_to_odometry(rng=rng, heading_noise=heading, **kwargs)) for _ in range(400)])
    assert abs(yaws.mean()) < 0.25
    assert 0.7 < yaws.std() < 1.3
    again = np.array([
        _yaw_deg_of(vehicle_state_to_odometry(rng=np.random.default_rng(42), heading_noise=heading, **kwargs))
        for _ in range(1)
    ])
    assert again[0] == pytest.approx(yaws[0])


def test_heading_noise_none_leaves_the_pre_existing_rng_stream_unchanged():
    kwargs = dict(
        orientation_matrix=np.eye(3), angular_velocity=np.zeros(3), raw_depth_z=-2.0, capture_time_s=1.0,
        noise=StateNoise(0.01, 0.02), message_types=fake_message_types(),
    )
    a = vehicle_state_to_odometry(rng=np.random.default_rng(3), **kwargs)
    b = vehicle_state_to_odometry(rng=np.random.default_rng(3), heading_noise=None, **kwargs)
    assert a.pose.pose.orientation.z == b.pose.pose.orientation.z
    assert a.pose.pose.position.z == b.pose.pose.position.z


def test_thrust_fraction_is_one_at_full_limit_and_zero_at_rest():
    assert thrust_fraction_of([0.0] * 8, 100.0) == 0.0
    assert thrust_fraction_of([100.0] * 8, 100.0) == pytest.approx(1.0)
    assert thrust_fraction_of([-100.0] * 8, 100.0) == pytest.approx(1.0)
    assert 0.0 < thrust_fraction_of([50.0] + [0.0] * 7, 100.0) < 0.2


def test_heading_noise_comes_from_manifest_metadata():
    manifest = load_realtime_manifest(MONO, TASK)
    heading = HeadingNoise.from_manifest_metadata(manifest.uw_metadata.get("heading_noise_model"))
    assert heading is not None
    assert heading.sigma_rad == pytest.approx(np.radians(1.0))
    assert heading.bias_rad_per_full_thrust == pytest.approx(np.radians(3.0))
    legacy = load_realtime_manifest(LEGACY, TASK)
    assert HeadingNoise.from_manifest_metadata(legacy.uw_metadata.get("heading_noise_model")) is None


def test_publisher_table_follows_the_manifest_sensor_set():
    mono = load_realtime_manifest(MONO, TASK)
    topics = build_topic_map(mono.agent_name, mono.algorithm_topics)
    mono_table = dict(build_publisher_table(mono, topics))
    assert mono_table[topics.main_camera] == "Image"
    assert mono_table[topics.imu] == "Imu"
    assert topics.left_camera not in mono_table and topics.right_camera not in mono_table
    assert mono_table[topics.clock] == "Clock" and mono_table[topics.scoring_truth] == "Odometry"

    stereo = load_realtime_manifest(STEREO, TASK)
    stereo_table = dict(build_publisher_table(stereo, build_topic_map(stereo.agent_name, stereo.algorithm_topics)))
    assert topics.left_camera in stereo_table and topics.right_camera in stereo_table

    legacy = load_realtime_manifest(LEGACY, TASK)
    legacy_topics = build_topic_map(legacy.agent_name, legacy.algorithm_topics)
    legacy_table = dict(build_publisher_table(legacy, legacy_topics))
    assert legacy_topics.main_camera not in legacy_table
    assert legacy_table[legacy_topics.imu] == "Imu"  # legacy file has an IMUSensor too


def test_build_topic_map_rejects_truth_or_pilot_topics_as_algorithm_inputs():
    with pytest.raises(ValueError):
        build_topic_map("auv0", ("/uw/sim/ground_truth",))
    with pytest.raises(ValueError):
        build_topic_map("auv0", ("/holoocean/auv0/PilotCamera",))


# --- recording -------------------------------------------------------------


def test_record_session_writes_main_camera_to_raw_camera_main_under_tick_identity():
    frames = [
        RawSensorFrame(
            sim_time_s=t * 0.04, receive_time_s=t * 0.04 + 0.001,
            sensors={"MainCamera": _camera_array(t), "IMUSensor": np.array([[0.0, 0.0, 9.81], [0.0, 0.0, 0.0]])},
        )
        for t in range(5)
    ]
    with tempfile.TemporaryDirectory() as tmp_dir:
        path = str(pathlib.Path(tmp_dir) / "mono_main.mcap")
        assert record_frames(_MODULES, frames, path) == 0
        main = list(read_canonical_messages(path, "/raw/camera/main", image_pb2.ImageFrame))
        assert len(main) == 5
        assert main[0][1].header.sensor_id.value == "camera_main"
        assert main[0][1].header.sensor_frame.value == "camera_main_link"
        assert [m.header.observation_id.value for _, m in main] == [f"tick{t}" for t in range(5)]
        assert list(read_canonical_messages(path, "/raw/camera/left", image_pb2.ImageFrame)) == []
        assert len(list(read_canonical_messages(path, "/raw/imu", imu_pb2.ImuSample))) == 5
