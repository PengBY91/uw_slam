"""HoloOcean IMUSensor reading -> canonical uw.domain.ImuSample conversion —
the raw-IMU counterpart to camera_conversion.py/state_conversion.py. No
frontend in this repo consumes IMU data yet (see docs/uw-slam-production-
readiness-and-roadmap-2026-08-21.md P1/P2); this exists so a real IMU
stream can be recorded now.
"""
from __future__ import annotations

import numpy as np

from uw_holoocean_adapter.time_utils import make_stamp


def holoocean_imu_to_imu_sample(
    imu_pb2_module,
    observation_pb2_module,
    time_pb2_module,
    imu_array: np.ndarray,
    *,
    sensor_id: str,
    sensor_frame: str,
    observation_id: str,
    capture_time_s: float,
    receive_time_s: float | None = None,
    provenance: str = "uw_holoocean_adapter_v1",
):
    """Converts one HoloOcean IMUSensor tick into a uw.domain.ImuSample.

    `imu_array` is HoloOcean's own IMUSensor output (holoocean.sensors.
    IMUSensor): a 2D array whose first two rows are always
    [accel_xyz, angular_velocity_xyz] (m/s^2, rad/s) and whose optional
    trailing two rows — present only when that sensor's `ReturnBias` config
    option is set — are [accel_bias_xyz, angular_velocity_bias_xyz]. Shape
    is therefore (2, 3) or (4, 3); anything else is rejected rather than
    guessed at.

    `receive_time_s`, when given, is real wall-clock time — see
    camera_conversion.py's `holoocean_camera_to_image_frame` doc comment for
    the full clock-domain convention this follows. Left unset if omitted.
    """
    if imu_array.ndim != 2 or imu_array.shape[1] != 3 or imu_array.shape[0] not in (2, 4):
        raise ValueError(f"expected a (2,3) or (4,3) IMU array, got shape {imu_array.shape}")

    sample = imu_pb2_module.ImuSample()
    sample.header.observation_id.value = observation_id
    sample.header.sensor_id.value = sensor_id
    sample.header.sensor_frame.value = sensor_frame
    sample.header.capture_time.CopyFrom(make_stamp(time_pb2_module, capture_time_s))
    if receive_time_s is not None:
        sample.header.receive_time.CopyFrom(make_stamp(time_pb2_module, receive_time_s))
    sample.header.clock_domain = time_pb2_module.CLOCK_DOMAIN_SIMULATION
    sample.header.validity = observation_pb2_module.ObservationHeader.VALIDITY_OK
    sample.header.provenance = provenance

    sample.linear_acceleration_mps2.extend(imu_array[0].tolist())
    sample.angular_velocity_radps.extend(imu_array[1].tolist())

    has_bias = imu_array.shape[0] == 4
    sample.has_bias = has_bias
    if has_bias:
        sample.bias_linear_acceleration_mps2.extend(imu_array[2].tolist())
        sample.bias_angular_velocity_radps.extend(imu_array[3].tolist())

    return sample
