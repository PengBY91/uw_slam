import numpy as np
import pytest
from uw.domain import measurement_pb2, state_pb2, time_pb2  # noqa: E402

from uw_holoocean_adapter.state_conversion import (
    depth_sensor_to_evidence,
    pose_sensor_to_state_snapshot,
)


def test_pose_sensor_to_state_snapshot_sets_id_time_and_pose():
    matrix = np.eye(4)
    matrix[:3, 3] = [1.0, 2.0, 3.0]

    snapshot = pose_sensor_to_state_snapshot(
        state_pb2, time_pb2, matrix, state_id="kf3", capture_time_s=1.5
    )

    assert snapshot.state_id.value == "kf3"
    assert snapshot.capture_timestamp.seconds == 1
    assert snapshot.capture_timestamp.nanos == 500_000_000
    assert snapshot.tracking_status == state_pb2.StateSnapshot.TRACKING_STATUS_TRACKING
    assert len(snapshot.pose_wb.matrix_row_major) == 16
    recovered_translation = [snapshot.pose_wb.matrix_row_major[3], snapshot.pose_wb.matrix_row_major[7],
                              snapshot.pose_wb.matrix_row_major[11]]
    assert recovered_translation == [1.0, 2.0, 3.0]


def test_depth_sensor_to_evidence_negates_raw_holoocean_reading():
    """HoloOcean's real DepthSensor returns raw world-frame position_z
    (negative underwater — confirmed against a real recording, and against
    external_repos/HoloOcean's own DepthSensor docstring), not an
    already-positive depth magnitude. depth_sensor_to_evidence's output must
    be positive-down (the wire convention apps/replay_demo's kf0_z anchoring
    consumes via `kf0_z = -depth_m`) — this is the regression test for a
    real bug: passing the raw reading through unnegated put the anchor
    keyframe ~585m away from ground truth in a real end-to-end run."""
    evidence = depth_sensor_to_evidence(
        measurement_pb2,
        np.array([-292.71]),
        evidence_id="depth_kf3",
        source_observation_id="kf3",
        sigma_m=0.1,
    )

    assert evidence.evidence_id.value == "depth_kf3"
    assert evidence.WhichOneof("payload") == "pressure_depth"
    assert evidence.pressure_depth.depth_m == pytest.approx(292.71)
    assert evidence.pressure_depth.sigma_m == 0.1
    assert len(evidence.source_observations) == 1
    assert evidence.source_observations[0].value == "kf3"


def test_depth_sensor_to_evidence_accepts_scalar_array():
    evidence = depth_sensor_to_evidence(
        measurement_pb2,
        np.array(-7.0),
        evidence_id="depth_kf0",
        source_observation_id="kf0",
    )
    assert evidence.pressure_depth.depth_m == 7.0
