"""HoloOcean DVLSensor reading -> canonical uw.domain.DvlSample conversion —
the raw-DVL counterpart to camera_conversion.py/state_conversion.py. No
frontend in this repo consumes DVL data yet; this exists so a real DVL
stream can be recorded now.
"""
from __future__ import annotations

import numpy as np

from uw_holoocean_adapter.time_utils import make_stamp


def holoocean_dvl_to_dvl_sample(
    dvl_pb2_module,
    observation_pb2_module,
    time_pb2_module,
    dvl_array: np.ndarray,
    *,
    sensor_id: str,
    sensor_frame: str,
    observation_id: str,
    capture_time_s: float,
    receive_time_s: float | None = None,
    provenance: str = "uw_holoocean_adapter_v1",
):
    """Converts one HoloOcean DVLSensor tick into a uw.domain.DvlSample.

    `dvl_array` is HoloOcean's own DVLSensor output (holoocean.sensors.
    DVLSensor): a 1D array of [velocity_x, velocity_y, velocity_z] (m/s),
    optionally followed by up to 4 beam ranges (meters) when that sensor's
    `ReturnRange` config option is set (the default). Shape is therefore
    (3,) or (3+N,) for N in [1, 4]; the beam-range count is whatever
    HoloOcean actually returned, not assumed to always be exactly 4.

    `receive_time_s`, when given, is real wall-clock time — see
    camera_conversion.py's `holoocean_camera_to_image_frame` doc comment for
    the full clock-domain convention this follows. Left unset if omitted.
    """
    if dvl_array.ndim != 1 or dvl_array.shape[0] < 3:
        raise ValueError(f"expected a 1D array of at least 3 values, got shape {dvl_array.shape}")

    sample = dvl_pb2_module.DvlSample()
    sample.header.observation_id.value = observation_id
    sample.header.sensor_id.value = sensor_id
    sample.header.sensor_frame.value = sensor_frame
    sample.header.capture_time.CopyFrom(make_stamp(time_pb2_module, capture_time_s))
    if receive_time_s is not None:
        sample.header.receive_time.CopyFrom(make_stamp(time_pb2_module, receive_time_s))
    sample.header.clock_domain = time_pb2_module.CLOCK_DOMAIN_SIMULATION
    sample.header.validity = observation_pb2_module.ObservationHeader.VALIDITY_OK
    sample.header.provenance = provenance

    sample.velocity_mps.extend(dvl_array[:3].tolist())

    beam_ranges = dvl_array[3:]
    sample.has_beam_ranges = beam_ranges.size > 0
    if sample.has_beam_ranges:
        sample.beam_ranges_m.extend(beam_ranges.tolist())

    return sample
