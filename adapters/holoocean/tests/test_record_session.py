import tempfile
from pathlib import Path

import numpy as np
from uw.domain import image_pb2, measurement_pb2, observation_pb2, state_pb2, time_pb2  # noqa: E402

from uw_holoocean_adapter.canonical_writer import read_canonical_messages
from uw_holoocean_adapter.holoocean_driver import RawSensorFrame
from uw_holoocean_adapter.record_session import record_frames


def _camera_array(fill: int) -> np.ndarray:
    arr = np.full((4, 4, 4), fill, dtype=np.uint8)
    arr[:, :, 3] = 255
    return arr


def test_record_frames_emits_a_keyframe_only_for_camera_bearing_frames():
    frames = [
        RawSensorFrame(sim_time_s=0.0, receive_time_s=0.0, sensors={"IMUSensor": np.zeros(6)}),
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
        num_keyframes = record_frames(
            image_pb2, observation_pb2, time_pb2, state_pb2, measurement_pb2, frames, path
        )

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


def test_record_frames_writes_nothing_for_an_all_non_camera_sequence():
    frames = [
        RawSensorFrame(sim_time_s=0.0, receive_time_s=0.0, sensors={"IMUSensor": np.zeros(6)}),
        RawSensorFrame(sim_time_s=0.1, receive_time_s=0.1, sensors={"DepthSensor": np.array([1.0])}),
    ]

    with tempfile.TemporaryDirectory() as tmp_dir:
        path = str(Path(tmp_dir) / "empty.mcap")
        num_keyframes = record_frames(
            image_pb2, observation_pb2, time_pb2, state_pb2, measurement_pb2, frames, path
        )
        assert num_keyframes == 0
