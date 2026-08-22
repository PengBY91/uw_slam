import numpy as np
import pytest
from uw.domain import imu_pb2, observation_pb2, time_pb2  # noqa: E402

from uw_holoocean_adapter.imu_conversion import holoocean_imu_to_imu_sample


def test_converts_reading_without_bias():
    imu_array = np.array([[0.1, 0.2, 9.81], [0.01, -0.02, 0.03]])

    sample = holoocean_imu_to_imu_sample(
        imu_pb2,
        observation_pb2,
        time_pb2,
        imu_array,
        sensor_id="imu0",
        sensor_frame="imu_link",
        observation_id="kf0",
        capture_time_s=1.5,
    )

    assert list(sample.linear_acceleration_mps2) == [0.1, 0.2, 9.81]
    assert list(sample.angular_velocity_radps) == [0.01, -0.02, 0.03]
    assert sample.has_bias is False
    assert list(sample.bias_linear_acceleration_mps2) == []
    assert list(sample.bias_angular_velocity_radps) == []
    assert sample.header.observation_id.value == "kf0"
    assert sample.header.sensor_id.value == "imu0"
    assert sample.header.sensor_frame.value == "imu_link"
    assert sample.header.clock_domain == time_pb2.CLOCK_DOMAIN_SIMULATION
    assert sample.header.validity == observation_pb2.ObservationHeader.VALIDITY_OK


def test_converts_reading_with_bias():
    imu_array = np.array(
        [[0.1, 0.2, 9.81], [0.01, -0.02, 0.03], [0.001, 0.002, 0.003], [0.0001, 0.0002, 0.0003]]
    )

    sample = holoocean_imu_to_imu_sample(
        imu_pb2,
        observation_pb2,
        time_pb2,
        imu_array,
        sensor_id="imu0",
        sensor_frame="imu_link",
        observation_id="kf1",
        capture_time_s=0.0,
    )

    assert sample.has_bias is True
    assert list(sample.bias_linear_acceleration_mps2) == [0.001, 0.002, 0.003]
    assert list(sample.bias_angular_velocity_radps) == [0.0001, 0.0002, 0.0003]


@pytest.mark.parametrize("bad_shape", [(3, 3), (2, 4), (6,)])
def test_rejects_unexpected_shapes(bad_shape):
    with pytest.raises(ValueError):
        holoocean_imu_to_imu_sample(
            imu_pb2,
            observation_pb2,
            time_pb2,
            np.zeros(bad_shape),
            sensor_id="imu0",
            sensor_frame="imu_link",
            observation_id="kf0",
            capture_time_s=0.0,
        )
