import numpy as np
import pytest

from uw_holoocean_adapter.holoocean_driver import RawSensorFrame
from uw_holoocean_adapter.realtime_ros_session import _validate_thruster_command, build_realtime_messages
from uw_holoocean_adapter.ros_message_conversion import StateNoise, build_topic_map

from test_ros_message_conversion import fake_message_types  # noqa: E402  (shared fakes, same dir)


def _camera_frame():
    return np.zeros((2, 2, 4), dtype=np.uint8)


def test_only_publishes_sensors_present_this_tick():
    frame = RawSensorFrame(sim_time_s=1.0, receive_time_s=1.0, sensors={"LeftCamera": _camera_frame()})
    topics = build_topic_map()
    messages = build_realtime_messages(
        frame, topics, fake_message_types(), state_noise=StateNoise(0.01, 0.02),
        rng=np.random.default_rng(1),
    )
    published_topics = {topic for topic, _ in messages}
    assert topics.left_camera in published_topics
    assert topics.right_camera not in published_topics
    assert topics.vehicle_state not in published_topics
    # /clock is published unconditionally every tick.
    assert topics.clock in published_topics


def test_pose_sensor_present_publishes_ground_truth_topic():
    identity_pose = np.eye(4)
    identity_pose[:3, 3] = [1.0, 2.0, 3.0]
    frame = RawSensorFrame(
        sim_time_s=1.0, receive_time_s=1.0,
        sensors={"LeftCamera": _camera_frame(), "PoseSensor": identity_pose},
    )
    topics = build_topic_map()
    messages = build_realtime_messages(
        frame, topics, fake_message_types(), rng=np.random.default_rng(1),
    )
    truth_messages = [msg for topic, msg in messages if topic == topics.scoring_truth]
    assert len(truth_messages) == 1
    assert truth_messages[0].pose.pose.position.x == pytest.approx(1.0)
    assert truth_messages[0].pose.pose.position.y == pytest.approx(2.0)
    assert truth_messages[0].pose.pose.position.z == pytest.approx(3.0)


def test_pose_sensor_absent_publishes_no_ground_truth_topic():
    frame = RawSensorFrame(sim_time_s=1.0, receive_time_s=1.0, sensors={"LeftCamera": _camera_frame()})
    topics = build_topic_map()
    messages = build_realtime_messages(
        frame, topics, fake_message_types(), rng=np.random.default_rng(1),
    )
    published_topics = {topic for topic, _ in messages}
    assert topics.scoring_truth not in published_topics


def test_vehicle_state_requires_all_three_source_sensors():
    frame = RawSensorFrame(
        sim_time_s=1.0, receive_time_s=1.0,
        sensors={"VehicleOrientation": np.eye(3), "IMUSensor": np.zeros((2, 3))},
    )
    topics = build_topic_map()
    messages = build_realtime_messages(
        frame, topics, fake_message_types(), rng=np.random.default_rng(1),
    )
    published_topics = {topic for topic, _ in messages}
    assert topics.vehicle_state not in published_topics


def test_vehicle_state_published_when_all_three_present():
    frame = RawSensorFrame(
        sim_time_s=1.0, receive_time_s=1.0,
        sensors={
            "VehicleOrientation": np.eye(3),
            "IMUSensor": np.zeros((2, 3)),
            "DepthSensor": np.array([-2.0]),
        },
    )
    topics = build_topic_map()
    messages = build_realtime_messages(
        frame, topics, fake_message_types(), rng=np.random.default_rng(1),
    )
    published_topics = {topic for topic, _ in messages}
    assert topics.vehicle_state in published_topics


def test_pilot_camera_and_sonar_both_publish_independently():
    frame = RawSensorFrame(
        sim_time_s=1.0, receive_time_s=1.0,
        sensors={"PilotCamera": _camera_frame(), "ImagingSonar": np.zeros((4, 6), dtype=np.float32)},
    )
    topics = build_topic_map()
    messages = build_realtime_messages(
        frame, topics, fake_message_types(), rng=np.random.default_rng(1),
    )
    published_topics = {topic for topic, _ in messages}
    assert topics.pilot_camera in published_topics
    assert topics.imaging_sonar in published_topics
    assert topics.scoring_truth not in published_topics


@pytest.mark.parametrize("values,expected", [
    ([1.0] * 8, [1.0] * 8),
    ([1.0] * 7, None),
    ([1.0] * 9, None),
    ([], None),
])
def test_validate_thruster_command_length(values, expected):
    assert _validate_thruster_command(values) == expected
