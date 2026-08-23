"""HoloOcean ImagingSonar reading -> canonical uw.domain.SonarFrame
conversion — the raw-sonar counterpart to camera_conversion.py. Kept
algorithmically identical to the established real-HoloOcean-verified
conversion already in this repo for the ROS2 path
(src/adapters/holoocean_ros_bridge_sonar_frame_provider.cpp's
PushImagingSonar — see that file's header comment for the two non-obvious
points this mirrors: HoloOcean's raw intensity is float32 in [0, 1], and
its column (bearing) order runs opposite this platform's ascending-bearing
convention, so each row is reversed).

HoloOcean's raw Python ImagingSonar sensor (holoocean.sensors.ImagingSonar)
carries no per-beam angle array or range calibration in its returned array
— shape is only (RangeBins, AzimuthBins) — so horizontal FOV and min/max
range must come from that sensor's own scenario-JSON configuration
(Azimuth/RangeMin/RangeMax), passed in by the caller, same as the C++
side's HoloOceanImagingSonarParams. NOT independently re-verified against a
live Python-path HoloOcean capture in this sandbox (no HoloOcean install
here) — this assumes the Python sensor's raw array uses the same [0,1]
float32 convention already confirmed for the ROS2 message path, since both
come from the same underlying HoloOcean engine.
"""
from __future__ import annotations

import numpy as np

from uw_holoocean_adapter.time_utils import make_stamp


def holoocean_sonar_to_sonar_frame(
    sonar_pb2_module,
    observation_pb2_module,
    time_pb2_module,
    intensity_array: np.ndarray,
    *,
    sensor_id: str,
    sensor_frame: str,
    observation_id: str,
    capture_time_s: float,
    horizontal_fov_rad: float,
    min_range_m: float,
    max_range_m: float,
    receive_time_s: float | None = None,
    provenance: str = "uw_holoocean_adapter_v1",
):
    """Converts one HoloOcean ImagingSonar tick into a uw.domain.SonarFrame.

    `intensity_array` must be a 2D (num_ranges, num_beams) float array
    (HoloOcean's own row-major [range, azimuth] shape).

    `receive_time_s`, when given, is real wall-clock time — see
    camera_conversion.py's `holoocean_camera_to_image_frame` doc comment for
    the full clock-domain convention this follows. Left unset if omitted.
    """
    if intensity_array.ndim != 2:
        raise ValueError(f"expected a 2D (num_ranges, num_beams) array, got shape {intensity_array.shape}")

    num_ranges, num_beams = intensity_array.shape
    if num_ranges == 0 or num_beams == 0:
        raise ValueError(f"num_ranges/num_beams must both be non-zero, got shape {intensity_array.shape}")

    frame = sonar_pb2_module.SonarFrame()
    frame.header.observation_id.value = observation_id
    frame.header.sensor_id.value = sensor_id
    frame.header.sensor_frame.value = sensor_frame
    frame.header.capture_time.CopyFrom(make_stamp(time_pb2_module, capture_time_s))
    if receive_time_s is not None:
        frame.header.receive_time.CopyFrom(make_stamp(time_pb2_module, receive_time_s))
    frame.header.clock_domain = time_pb2_module.CLOCK_DOMAIN_SIMULATION
    frame.header.validity = observation_pb2_module.ObservationHeader.VALIDITY_OK
    frame.header.provenance = provenance

    frame.num_ranges = num_ranges
    frame.num_beams = num_beams
    frame.min_range = min_range_m
    frame.max_range = max_range_m
    range_resolution = (max_range_m - min_range_m) / num_ranges
    frame.range_resolution = range_resolution
    frame.horizontal_fov = horizontal_fov_rad

    frame.range_bins.extend(min_range_m + np.arange(num_ranges + 1, dtype=np.float64) * range_resolution)

    # Ascending bearing, -fov/2 .. +fov/2 — see module docstring for why the
    # intensity row below is mirrored (reversed) to line up with this order,
    # same correction as the C++ ROS2-path provider.
    half_fov = horizontal_fov_rad / 2.0
    if num_beams > 1:
        t = np.arange(num_beams, dtype=np.float64) / (num_beams - 1)
    else:
        t = np.zeros(1, dtype=np.float64)
    frame.azimuth_angles.extend(-half_fov + horizontal_fov_rad * t)

    mirrored = intensity_array[:, ::-1]
    quantized = np.clip(np.round(np.clip(mirrored, 0.0, 1.0) * 255.0), 0, 255).astype(np.uint8)
    frame.intensity_tensor = quantized.tobytes()
    frame.encoding = sonar_pb2_module.SonarFrame.ENCODING_UINT8_GRAY

    return frame
