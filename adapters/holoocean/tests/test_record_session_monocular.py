"""PREP-A-04 (docs/ROV平台到货前准备工作规格-2026-09-02.md): the contract
vehicle is monocular (single IMX462), so a stereo keyframe never forms.
The recorder must still write every sensor it is given -- including the
depth evidence and GT state that used to be gated on a stereo pair -- and
must never fabricate a keyframe."""
import tempfile
from pathlib import Path

import numpy as np
from uw.domain import (  # noqa: E402
    image_pb2,
    imu_pb2,
    measurement_pb2,
    sonar_pb2,
    state_pb2,
)

from uw_holoocean_adapter.canonical_writer import read_canonical_messages
from uw_holoocean_adapter.holoocean_driver import RawSensorFrame
from uw_holoocean_adapter.record_session import record_frames

from test_record_session import _MODULES, _camera_array

_TICKS = 100
_IMU_EVERY = 1  # 200 Hz IMU relative to a 200 Hz tick
_CAMERA_EVERY = 7  # ~30 Hz mono camera
_SONAR_EVERY = 5  # 40 Hz sonar
_DEPTH_EVERY = 4  # 50 Hz pressure/depth
_POSE_EVERY = 20  # 10 Hz scoring pose


def _sonar_array() -> np.ndarray:
    return np.zeros((4, 8), dtype=np.float32)


def _mono_frames():
    frames = []
    for tick in range(_TICKS):
        sensors = {}
        if tick % _IMU_EVERY == 0:
            sensors["IMUSensor"] = np.array([[0.0, 0.0, 9.81], [0.0, 0.0, 0.0]], dtype=np.float64)
        if tick % _CAMERA_EVERY == 0:
            sensors["LeftCamera"] = _camera_array(tick % 255)
        if tick % _SONAR_EVERY == 0:
            sensors["ImagingSonar"] = _sonar_array()
        if tick % _DEPTH_EVERY == 0:
            sensors["DepthSensor"] = np.array([-3.0 - 0.01 * tick], dtype=np.float64)
        if tick % _POSE_EVERY == 0:
            sensors["PoseSensor"] = np.eye(4, dtype=np.float64)
        frames.append(RawSensorFrame(sim_time_s=tick * 0.005, receive_time_s=tick * 0.005 + 0.001, sensors=sensors))
    return frames


def _count(path: str, topic: str, message_type) -> int:
    return len(list(read_canonical_messages(path, topic, message_type)))


def test_monocular_sequence_writes_every_sensor_and_forms_no_keyframe():
    frames = _mono_frames()
    expected = {
        "/raw/camera/left": sum(1 for f in frames if "LeftCamera" in f.sensors),
        "/raw/imu": sum(1 for f in frames if "IMUSensor" in f.sensors),
        "/raw/sonar_frame": sum(1 for f in frames if "ImagingSonar" in f.sensors),
        "/evidence/depth": sum(1 for f in frames if "DepthSensor" in f.sensors),
        "/gt/state": sum(1 for f in frames if "PoseSensor" in f.sensors),
    }
    assert expected["/raw/camera/left"] > 0 and expected["/evidence/depth"] > 0 and expected["/gt/state"] > 0

    with tempfile.TemporaryDirectory() as tmp_dir:
        path = str(Path(tmp_dir) / "mono.mcap")
        num_keyframes = record_frames(_MODULES, frames, path)

        assert num_keyframes == 0
        assert _count(path, "/raw/camera/left", image_pb2.ImageFrame) == expected["/raw/camera/left"]
        assert _count(path, "/raw/camera/right", image_pb2.ImageFrame) == 0
        assert _count(path, "/raw/imu", imu_pb2.ImuSample) == expected["/raw/imu"]
        assert _count(path, "/raw/sonar_frame", sonar_pb2.SonarFrame) == expected["/raw/sonar_frame"]
        assert _count(path, "/evidence/depth", measurement_pb2.MeasurementEvidence) == expected["/evidence/depth"]
        assert _count(path, "/gt/state", state_pb2.StateSnapshot) == expected["/gt/state"]


def test_monocular_depth_and_gt_are_keyed_on_tick_not_keyframe():
    frames = _mono_frames()
    with tempfile.TemporaryDirectory() as tmp_dir:
        path = str(Path(tmp_dir) / "mono.mcap")
        record_frames(_MODULES, frames, path)

        depth = list(read_canonical_messages(path, "/evidence/depth", measurement_pb2.MeasurementEvidence))
        ticks_with_depth = [i for i, f in enumerate(frames) if "DepthSensor" in f.sensors]
        assert [m.source_observations[0].value for _, m in depth] == ["tick" + str(t) for t in ticks_with_depth]
        assert all(not m.source_observations[0].value.startswith("kf") for _, m in depth)

        gt = list(read_canonical_messages(path, "/gt/state", state_pb2.StateSnapshot))
        ticks_with_pose = [i for i, f in enumerate(frames) if "PoseSensor" in f.sensors]
        assert [m.state_id.value for _, m in gt] == ["tick" + str(t) for t in ticks_with_pose]


def test_stereo_ticks_still_key_depth_and_gt_on_the_keyframe_id():
    # Regression guard for the stereo path: a formed pair must keep the
    # "kfN"-keyed depth/GT the stereo replay pipeline associates on.
    frames = [
        RawSensorFrame(
            sim_time_s=0.0,
            receive_time_s=0.0,
            sensors={
                "LeftCamera": _camera_array(1),
                "RightCamera": _camera_array(2),
                "DepthSensor": np.array([-2.0]),
                "PoseSensor": np.eye(4),
            },
        ),
        RawSensorFrame(sim_time_s=0.01, receive_time_s=0.01, sensors={"DepthSensor": np.array([-2.1])}),
    ]
    with tempfile.TemporaryDirectory() as tmp_dir:
        path = str(Path(tmp_dir) / "stereo.mcap")
        assert record_frames(_MODULES, frames, path) == 1
        depth = list(read_canonical_messages(path, "/evidence/depth", measurement_pb2.MeasurementEvidence))
        assert [m.source_observations[0].value for _, m in depth] == ["kf0", "tick1"]
        gt = list(read_canonical_messages(path, "/gt/state", state_pb2.StateSnapshot))
        assert [m.state_id.value for _, m in gt] == ["kf0"]
