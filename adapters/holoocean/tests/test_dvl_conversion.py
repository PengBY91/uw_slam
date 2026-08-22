import numpy as np
import pytest
from uw.domain import dvl_pb2, observation_pb2, time_pb2  # noqa: E402

from uw_holoocean_adapter.dvl_conversion import holoocean_dvl_to_dvl_sample


def test_converts_reading_with_beam_ranges():
    dvl_array = np.array([0.5, -0.1, 0.02, 2.0, 2.1, 2.2, 2.3])

    sample = holoocean_dvl_to_dvl_sample(
        dvl_pb2,
        observation_pb2,
        time_pb2,
        dvl_array,
        sensor_id="dvl0",
        sensor_frame="dvl_link",
        observation_id="kf0",
        capture_time_s=1.0,
    )

    assert list(sample.velocity_mps) == [0.5, -0.1, 0.02]
    assert sample.has_beam_ranges is True
    assert list(sample.beam_ranges_m) == [2.0, 2.1, 2.2, 2.3]
    assert sample.header.observation_id.value == "kf0"
    assert sample.header.sensor_id.value == "dvl0"


def test_converts_velocity_only_reading():
    sample = holoocean_dvl_to_dvl_sample(
        dvl_pb2,
        observation_pb2,
        time_pb2,
        np.array([1.0, 2.0, 3.0]),
        sensor_id="dvl0",
        sensor_frame="dvl_link",
        observation_id="kf1",
        capture_time_s=0.0,
    )

    assert list(sample.velocity_mps) == [1.0, 2.0, 3.0]
    assert sample.has_beam_ranges is False
    assert list(sample.beam_ranges_m) == []


def test_rejects_too_short_array():
    with pytest.raises(ValueError):
        holoocean_dvl_to_dvl_sample(
            dvl_pb2,
            observation_pb2,
            time_pb2,
            np.array([1.0, 2.0]),
            sensor_id="dvl0",
            sensor_frame="dvl_link",
            observation_id="kf0",
            capture_time_s=0.0,
        )
