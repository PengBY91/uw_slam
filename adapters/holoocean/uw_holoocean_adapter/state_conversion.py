"""Converts HoloOcean vehicle-state sensor readings (PoseSensor,
DepthSensor) into canonical uw.domain protobuf messages — the
non-camera counterpart to camera_conversion.py that a recording session
needs to produce a complete bag (/gt/state, /evidence/depth).
"""
from __future__ import annotations

import numpy as np

from uw_holoocean_adapter.coordinates import pose_sensor_to_pose
from uw_holoocean_adapter.time_utils import make_stamp


def pose_sensor_to_state_snapshot(
    state_pb2_module,
    time_pb2_module,
    pose_matrix: np.ndarray,
    *,
    state_id: str,
    capture_time_s: float,
):
    """Converts one HoloOcean PoseSensor reading into a uw.domain.StateSnapshot
    for the /gt/state topic — this is ground truth (HoloOcean's own simulated
    vehicle pose), not an estimate, matching how apps/tools/synth_bag_gen
    writes /gt/state from its own known trajectory.
    """
    pose = pose_sensor_to_pose(pose_matrix)
    snapshot = state_pb2_module.StateSnapshot()
    snapshot.state_id.value = state_id
    snapshot.capture_timestamp.CopyFrom(make_stamp(time_pb2_module, capture_time_s))
    snapshot.pose_wb.matrix_row_major.extend(pose.to_matrix4().flatten().tolist())
    snapshot.tracking_status = state_pb2_module.StateSnapshot.TRACKING_STATUS_TRACKING
    return snapshot


def depth_sensor_to_evidence(
    measurement_pb2_module,
    depth_value: np.ndarray,
    *,
    evidence_id: str,
    source_observation_id: str,
    sigma_m: float = 0.05,
    provenance: str = "uw_holoocean_adapter_v1",
):
    """Converts one HoloOcean DepthSensor reading (shape (1,), meters) into a
    uw.domain.MeasurementEvidence wrapping PressureDepthMeasurement, matching
    apps/tools/synth_bag_gen's /evidence/depth convention: depth_m is a
    positive-down reading (uw::sensor_models::Pose3's z is up, so consumers
    negate this — see replay_demo's kf0_z anchoring, which does exactly
    that: `kf0_z = -depth_m`).
    """
    depth_m = float(np.asarray(depth_value).reshape(-1)[0])

    evidence = measurement_pb2_module.MeasurementEvidence()
    evidence.evidence_id.value = evidence_id
    evidence.algorithm_version = provenance
    evidence.estimated_noise_scale = 1.0
    evidence.source_observations.add().value = source_observation_id
    evidence.pressure_depth.depth_m = depth_m
    evidence.pressure_depth.sigma_m = sigma_m
    return evidence
