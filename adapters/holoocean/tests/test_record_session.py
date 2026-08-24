import math
import tempfile
from pathlib import Path

import numpy as np
from uw.domain import (  # noqa: E402
    dvl_pb2,
    image_pb2,
    imu_pb2,
    measurement_pb2,
    observation_pb2,
    sonar_pb2,
    state_pb2,
    time_pb2,
)

from uw_holoocean_adapter.canonical_writer import read_canonical_messages
from uw_holoocean_adapter.holoocean_driver import RawSensorFrame
from uw_holoocean_adapter.record_session import SchemaModules, record_frames

_MODULES = SchemaModules(
    image=image_pb2,
    observation=observation_pb2,
    time=time_pb2,
    state=state_pb2,
    measurement=measurement_pb2,
    sonar=sonar_pb2,
    imu=imu_pb2,
    dvl=dvl_pb2,
)


def _camera_array(fill: int) -> np.ndarray:
    arr = np.full((4, 4, 4), fill, dtype=np.uint8)
    arr[:, :, 3] = 255
    return arr


def test_record_frames_emits_a_keyframe_only_for_camera_bearing_frames():
    frames = [
        RawSensorFrame(sim_time_s=0.0, receive_time_s=0.0, sensors={"IMUSensor": np.zeros((2, 3))}),
        RawSensorFrame(
            sim_time_s=0.2,
            receive_time_s=0.2,
            sensors={
                "LeftCamera": _camera_array(10),
                "RightCamera": _camera_array(20),
                "PoseSensor": np.eye(4),
                # Raw HoloOcean DepthSensor is signed world-frame position_z
                # (negative underwater, see state_conversion.py's
                # depth_sensor_to_evidence docstring) — depth_m below is the
                # WIRE convention (positive-down) it gets negated into.
                "DepthSensor": np.array([-5.0]),
            },
        ),
        RawSensorFrame(
            sim_time_s=0.4,
            receive_time_s=0.4,
            sensors={"LeftCamera": _camera_array(30), "RightCamera": _camera_array(40)},
        ),
    ]

    with tempfile.TemporaryDirectory() as tmp_dir:
        path = str(Path(tmp_dir) / "session.mcap")
        num_keyframes = record_frames(_MODULES, frames, path)

        assert num_keyframes == 2

        left_messages = list(read_canonical_messages(path, "/raw/camera/left", image_pb2.ImageFrame))
        assert len(left_messages) == 2
        assert left_messages[0][1].header.observation_id.value == "kf0"
        assert left_messages[1][1].header.observation_id.value == "kf1"

        right_messages = list(read_canonical_messages(path, "/raw/camera/right", image_pb2.ImageFrame))
        assert len(right_messages) == 2

        gt_messages = list(read_canonical_messages(path, "/gt/state", state_pb2.StateSnapshot))
        assert len(gt_messages) == 1  # only the first keyframe carried a PoseSensor reading
        assert gt_messages[0][1].state_id.value == "kf0"

        depth_messages = list(
            read_canonical_messages(path, "/evidence/depth", measurement_pb2.MeasurementEvidence)
        )
        assert len(depth_messages) == 1
        assert depth_messages[0][1].pressure_depth.depth_m == 5.0


def test_record_frames_forms_no_keyframe_for_an_all_non_camera_sequence():
    # Depth has no camera-independent write path (its identity is defined
    # relative to a stereo keyframe -- see _write_sensor_tick), so it stays
    # silently unwritten here; that's expected, not this test's point.
    frames = [
        RawSensorFrame(sim_time_s=0.0, receive_time_s=0.0, sensors={"IMUSensor": np.zeros((2, 3))}),
        RawSensorFrame(sim_time_s=0.1, receive_time_s=0.1, sensors={"DepthSensor": np.array([1.0])}),
    ]

    with tempfile.TemporaryDirectory() as tmp_dir:
        path = str(Path(tmp_dir) / "empty.mcap")
        num_keyframes = record_frames(_MODULES, frames, path)
        assert num_keyframes == 0
        # IMU still gets written even though no keyframe formed -- the
        # multi-rate fix this test suite is guarding.
        assert len(list(read_canonical_messages(path, "/raw/imu", imu_pb2.ImuSample))) == 1
        assert list(read_canonical_messages(path, "/evidence/depth", measurement_pb2.MeasurementEvidence)) == []


def test_record_frames_writes_sonar_imu_dvl_when_present_on_a_camera_bearing_tick():
    imu_no_bias = np.array([[0.0, 0.0, 9.81], [0.01, -0.02, 0.03]])
    dvl_with_ranges = np.array([0.5, -0.1, 0.02, 2.0, 2.1, 2.2, 2.3])
    sonar_image = np.full((4, 6), 0.5, dtype=np.float32)

    frames = [
        RawSensorFrame(
            sim_time_s=0.2,
            receive_time_s=0.2,
            sensors={
                "LeftCamera": _camera_array(10),
                "RightCamera": _camera_array(20),
                "IMUSensor": imu_no_bias,
                "DVLSensor": dvl_with_ranges,
                "ImagingSonar": sonar_image,
            },
        ),
    ]

    with tempfile.TemporaryDirectory() as tmp_dir:
        path = str(Path(tmp_dir) / "sensors.mcap")
        num_keyframes = record_frames(_MODULES, frames, path)
        assert num_keyframes == 1

        imu_messages = list(read_canonical_messages(path, "/raw/imu", imu_pb2.ImuSample))
        assert len(imu_messages) == 1
        imu_sample = imu_messages[0][1]
        # Not "kf0": IMU's identity is keyed on the raw tick index, never on
        # the (much lower-rate) camera keyframe counter -- see
        # test_sensor_observation_ids_are_keyed_on_tick_not_keyframe_counter
        # below for the case that would catch this regressing.
        assert imu_sample.header.observation_id.value == "tick0"
        assert list(imu_sample.linear_acceleration_mps2) == [0.0, 0.0, 9.81]
        assert imu_sample.has_bias is False

        dvl_messages = list(read_canonical_messages(path, "/raw/dvl", dvl_pb2.DvlSample))
        assert len(dvl_messages) == 1
        dvl_sample = dvl_messages[0][1]
        assert list(dvl_sample.velocity_mps) == [0.5, -0.1, 0.02]
        assert dvl_sample.has_beam_ranges is True
        assert list(dvl_sample.beam_ranges_m) == [2.0, 2.1, 2.2, 2.3]

        sonar_messages = list(read_canonical_messages(path, "/raw/sonar_frame", sonar_pb2.SonarFrame))
        assert len(sonar_messages) == 1
        sonar_frame = sonar_messages[0][1]
        assert sonar_frame.num_ranges == 4
        assert sonar_frame.num_beams == 6
        # Ascending azimuth is this platform's required invariant
        # (schemas/proto/uw/domain/sonar.proto's documented contract).
        angles = list(sonar_frame.azimuth_angles)
        assert angles == sorted(angles)
        assert math.isclose(angles[0], -math.radians(60.0), rel_tol=1e-6)
        # 0.5 in [0,1] -> round(0.5*255) = 128 for every (uniform-fill) cell.
        assert set(sonar_frame.intensity_tensor) == {128}


def test_record_frames_populates_receive_time_distinctly_from_capture_time():
    # receive_time_s (1.5, real wall-clock in a live recording) deliberately
    # differs from sim_time_s (0.2, becomes capture_time) so this test can't
    # pass by accident if the two were ever aliased/swapped — a prior gap
    # (both synth_bag_gen and this module) left receive_time at the proto
    # zero-Stamp default on every message, which tools/bag_audit reads as
    # "never populated" rather than "instantaneous"; see that tool's own
    # findings for the full story.
    frames = [
        RawSensorFrame(
            sim_time_s=0.2,
            receive_time_s=1.5,
            sensors={
                "LeftCamera": _camera_array(10),
                "RightCamera": _camera_array(20),
                "IMUSensor": np.array([[0.0, 0.0, 9.81], [0.01, -0.02, 0.03]]),
                "DVLSensor": np.array([0.5, -0.1, 0.02]),
                "ImagingSonar": np.full((4, 6), 0.5, dtype=np.float32),
            },
        ),
    ]

    with tempfile.TemporaryDirectory() as tmp_dir:
        path = str(Path(tmp_dir) / "receive_time.mcap")
        assert record_frames(_MODULES, frames, path) == 1

        def _receive_seconds(topic, pb2_type):
            messages = list(read_canonical_messages(path, topic, pb2_type))
            assert len(messages) == 1
            header = messages[0][1].header
            assert header.capture_time.seconds == 0
            assert header.capture_time.nanos == 200_000_000
            return header.receive_time.seconds + header.receive_time.nanos / 1e9

        for topic, pb2_type in (
            ("/raw/camera/left", image_pb2.ImageFrame),
            ("/raw/camera/right", image_pb2.ImageFrame),
            ("/raw/sonar_frame", sonar_pb2.SonarFrame),
            ("/raw/imu", imu_pb2.ImuSample),
            ("/raw/dvl", dvl_pb2.DvlSample),
        ):
            assert math.isclose(_receive_seconds(topic, pb2_type), 1.5, rel_tol=1e-9), topic


def test_record_frames_writes_imu_on_an_imu_only_tick():
    frames = [
        RawSensorFrame(
            sim_time_s=0.0,
            receive_time_s=0.0,
            sensors={"IMUSensor": np.array([[0.0, 0.0, 9.81], [0.01, -0.02, 0.03]])},
        ),
    ]

    with tempfile.TemporaryDirectory() as tmp_dir:
        path = str(Path(tmp_dir) / "imu_only.mcap")
        num_keyframes = record_frames(_MODULES, frames, path)

        assert num_keyframes == 0  # no stereo pair -> no keyframe formed
        imu_messages = list(read_canonical_messages(path, "/raw/imu", imu_pb2.ImuSample))
        assert len(imu_messages) == 1
        assert imu_messages[0][1].header.observation_id.value == "tick0"
        assert list(read_canonical_messages(path, "/raw/camera/left", image_pb2.ImageFrame)) == []
        assert list(read_canonical_messages(path, "/gt/state", state_pb2.StateSnapshot)) == []


def test_record_frames_writes_dvl_and_sonar_on_a_dvl_and_sonar_only_tick():
    frames = [
        RawSensorFrame(
            sim_time_s=0.0,
            receive_time_s=0.0,
            sensors={
                "DVLSensor": np.array([0.5, -0.1, 0.02]),
                "ImagingSonar": np.full((4, 6), 0.5, dtype=np.float32),
            },
        ),
    ]

    with tempfile.TemporaryDirectory() as tmp_dir:
        path = str(Path(tmp_dir) / "dvl_sonar_only.mcap")
        num_keyframes = record_frames(_MODULES, frames, path)

        assert num_keyframes == 0
        dvl_messages = list(read_canonical_messages(path, "/raw/dvl", dvl_pb2.DvlSample))
        assert len(dvl_messages) == 1
        assert dvl_messages[0][1].header.observation_id.value == "tick0"
        sonar_messages = list(read_canonical_messages(path, "/raw/sonar_frame", sonar_pb2.SonarFrame))
        assert len(sonar_messages) == 1
        assert sonar_messages[0][1].header.observation_id.value == "tick0"
        assert list(read_canonical_messages(path, "/raw/imu", imu_pb2.ImuSample)) == []
        assert list(read_canonical_messages(path, "/raw/camera/left", image_pb2.ImageFrame)) == []


def test_record_frames_records_a_lone_monocular_camera_without_fabricating_a_keyframe():
    frames = [
        RawSensorFrame(sim_time_s=0.0, receive_time_s=0.0, sensors={"LeftCamera": _camera_array(10)}),
    ]

    with tempfile.TemporaryDirectory() as tmp_dir:
        path = str(Path(tmp_dir) / "mono.mcap")
        num_keyframes = record_frames(_MODULES, frames, path)

        assert num_keyframes == 0  # a lone camera never forms a stereo keyframe
        left_messages = list(read_canonical_messages(path, "/raw/camera/left", image_pb2.ImageFrame))
        assert len(left_messages) == 1
        # Not "kf0": no stereo pair formed, so this gets the tick-based
        # identity, same as any other non-keyframe sensor reading.
        assert left_messages[0][1].header.observation_id.value == "tick0"
        assert list(read_canonical_messages(path, "/raw/camera/right", image_pb2.ImageFrame)) == []
        assert list(read_canonical_messages(path, "/gt/state", state_pb2.StateSnapshot)) == []
        assert (
            list(read_canonical_messages(path, "/evidence/depth", measurement_pb2.MeasurementEvidence))
            == []
        )


def test_sensor_observation_ids_are_keyed_on_tick_not_keyframe_counter():
    # Two stereo-bearing ticks (advancing keyframe_index to 0 then 1) sandwich
    # a non-camera tick that only has IMU -- if IMU's identity were ever
    # accidentally derived from keyframe_index instead of the raw tick
    # index, it would collide with kf0 or kf1 instead of getting its own
    # "tick1".
    frames = [
        RawSensorFrame(
            sim_time_s=0.0,
            receive_time_s=0.0,
            sensors={"LeftCamera": _camera_array(1), "RightCamera": _camera_array(2)},
        ),
        RawSensorFrame(sim_time_s=0.1, receive_time_s=0.1, sensors={"IMUSensor": np.zeros((2, 3))}),
        RawSensorFrame(
            sim_time_s=0.2,
            receive_time_s=0.2,
            sensors={"LeftCamera": _camera_array(3), "RightCamera": _camera_array(4)},
        ),
    ]

    with tempfile.TemporaryDirectory() as tmp_dir:
        path = str(Path(tmp_dir) / "tick_identity.mcap")
        num_keyframes = record_frames(_MODULES, frames, path)

        assert num_keyframes == 2
        left_messages = list(read_canonical_messages(path, "/raw/camera/left", image_pb2.ImageFrame))
        assert [m[1].header.observation_id.value for m in left_messages] == ["kf0", "kf1"]

        imu_messages = list(read_canonical_messages(path, "/raw/imu", imu_pb2.ImuSample))
        assert len(imu_messages) == 1
        imu_sample = imu_messages[0][1]
        assert imu_sample.header.observation_id.value == "tick1"
        # All three ticks share real, distinct capture times -- the shared-
        # capture-time-domain guarantee holds independent of the identity
        # scheme change above.
        assert math.isclose(
            imu_sample.header.capture_time.seconds + imu_sample.header.capture_time.nanos / 1e9,
            0.1,
            rel_tol=1e-9,
        )


def test_record_frames_omits_sonar_imu_dvl_topics_when_absent():
    frames = [
        RawSensorFrame(
            sim_time_s=0.0,
            receive_time_s=0.0,
            sensors={"LeftCamera": _camera_array(10), "RightCamera": _camera_array(20)},
        ),
    ]

    with tempfile.TemporaryDirectory() as tmp_dir:
        path = str(Path(tmp_dir) / "no_sensors.mcap")
        num_keyframes = record_frames(_MODULES, frames, path)
        assert num_keyframes == 1
        assert list(read_canonical_messages(path, "/raw/imu", imu_pb2.ImuSample)) == []
        assert list(read_canonical_messages(path, "/raw/dvl", dvl_pb2.DvlSample)) == []
        assert list(read_canonical_messages(path, "/raw/sonar_frame", sonar_pb2.SonarFrame)) == []
