import math

import numpy as np
import pytest
from uw.domain import observation_pb2, sonar_pb2, time_pb2  # noqa: E402

from uw_holoocean_adapter.sonar_conversion import holoocean_sonar_to_sonar_frame


def test_mirrors_columns_and_quantizes_to_uint8():
    # 2 ranges x 3 beams. Column c's value is c/2.0 so the mirrored
    # (reversed) row order is independently predictable: HoloOcean's raw
    # column order is [0.0, 0.5, 1.0] -> after mirroring must read
    # [1.0, 0.5, 0.0] -> quantized [255, 128, 0] (round(0.5*255)=128).
    intensity = np.array([[0.0, 0.5, 1.0], [0.0, 0.5, 1.0]], dtype=np.float32)

    frame = holoocean_sonar_to_sonar_frame(
        sonar_pb2,
        observation_pb2,
        time_pb2,
        intensity,
        sensor_id="sonar0",
        sensor_frame="sonar_link",
        observation_id="kf0",
        capture_time_s=1.0,
        horizontal_fov_rad=math.radians(120.0),
        min_range_m=0.1,
        max_range_m=10.0,
    )

    assert frame.num_ranges == 2
    assert frame.num_beams == 3
    assert frame.encoding == sonar_pb2.SonarFrame.ENCODING_UINT8_GRAY
    row = list(frame.intensity_tensor[0:3])
    assert row == [255, 128, 0]
    assert list(frame.intensity_tensor[3:6]) == [255, 128, 0]


def test_azimuth_angles_are_ascending_and_span_the_fov():
    intensity = np.zeros((1, 5), dtype=np.float32)
    frame = holoocean_sonar_to_sonar_frame(
        sonar_pb2,
        observation_pb2,
        time_pb2,
        intensity,
        sensor_id="sonar0",
        sensor_frame="sonar_link",
        observation_id="kf0",
        capture_time_s=0.0,
        horizontal_fov_rad=math.radians(120.0),
        min_range_m=0.1,
        max_range_m=10.0,
    )

    angles = list(frame.azimuth_angles)
    assert len(angles) == 5
    assert angles == sorted(angles)  # sonar.proto's required invariant
    assert math.isclose(angles[0], -math.radians(60.0), rel_tol=1e-6)
    assert math.isclose(angles[-1], math.radians(60.0), rel_tol=1e-6)


def test_range_bins_span_min_to_max_with_num_ranges_plus_one_edges():
    intensity = np.zeros((4, 2), dtype=np.float32)
    frame = holoocean_sonar_to_sonar_frame(
        sonar_pb2,
        observation_pb2,
        time_pb2,
        intensity,
        sensor_id="sonar0",
        sensor_frame="sonar_link",
        observation_id="kf0",
        capture_time_s=0.0,
        horizontal_fov_rad=math.radians(120.0),
        min_range_m=0.0,
        max_range_m=4.0,
    )

    bins = list(frame.range_bins)
    assert len(bins) == 5  # num_ranges + 1 edges
    assert bins == [0.0, 1.0, 2.0, 3.0, 4.0]
    assert math.isclose(frame.range_resolution, 1.0, rel_tol=1e-9)


def test_rejects_non_2d_array():
    with pytest.raises(ValueError):
        holoocean_sonar_to_sonar_frame(
            sonar_pb2,
            observation_pb2,
            time_pb2,
            np.zeros(5),
            sensor_id="sonar0",
            sensor_frame="sonar_link",
            observation_id="kf0",
            capture_time_s=0.0,
            horizontal_fov_rad=1.0,
            min_range_m=0.1,
            max_range_m=10.0,
        )
