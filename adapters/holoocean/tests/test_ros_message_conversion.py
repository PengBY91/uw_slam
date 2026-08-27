import numpy as np
import pytest

from uw_holoocean_adapter.coordinates import Pose
from uw_holoocean_adapter.ros_message_conversion import (
    RosMessageTypes,
    StateNoise,
    build_topic_map,
    holoocean_camera_to_ros_image,
    holoocean_sonar_to_imaging_sonar_msg,
    sim_time_to_clock_msg,
    truth_pose_to_odometry,
    vehicle_state_to_odometry,
)


class _FakeStamp:
    def __init__(self):
        self.sec = 0
        self.nanosec = 0


class _FakeHeader:
    def __init__(self):
        self.stamp = _FakeStamp()
        self.frame_id = ""


class _FakeVector3:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class _FakeQuaternion:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 1.0


class _FakePose:
    def __init__(self):
        self.position = _FakeVector3()
        self.orientation = _FakeQuaternion()


class _FakePoseWithCovariance:
    def __init__(self):
        self.pose = _FakePose()


class _FakeTwist:
    def __init__(self):
        self.linear = _FakeVector3()
        self.angular = _FakeVector3()


class _FakeTwistWithCovariance:
    def __init__(self):
        self.twist = _FakeTwist()


class FakeOdometry:
    def __init__(self):
        self.header = _FakeHeader()
        self.pose = _FakePoseWithCovariance()
        self.twist = _FakeTwistWithCovariance()


class FakeImage:
    def __init__(self):
        self.header = _FakeHeader()
        self.height = 0
        self.width = 0
        self.encoding = ""
        self.is_bigendian = 0
        self.step = 0
        self.data = b""


class FakeImagingSonar:
    def __init__(self):
        self.timestamp = 0
        self.bins_range = 0
        self.bins_azimuth = 0
        self.image_range = []


class FakeClock:
    def __init__(self):
        self.clock = _FakeStamp()


def fake_message_types():
    return RosMessageTypes(Image=FakeImage, Odometry=FakeOdometry, ImagingSonar=FakeImagingSonar, Clock=FakeClock)


def test_vehicle_odometry_uses_noisy_state_not_scoring_pose():
    msg = vehicle_state_to_odometry(
        orientation_matrix=np.eye(3), angular_velocity=np.array([0.1, 0.2, 0.3]),
        raw_depth_z=-2.0, capture_time_s=1.5, noise=StateNoise(0.01, 0.02),
        rng=np.random.default_rng(7), message_types=fake_message_types(),
    )
    assert msg.pose.pose.position.z == pytest.approx(-2.0, abs=0.1)
    assert msg.twist.twist.angular.x != 0.0


def test_vehicle_odometry_stamp_matches_capture_time():
    msg = vehicle_state_to_odometry(
        orientation_matrix=np.eye(3), angular_velocity=np.zeros(3),
        raw_depth_z=0.0, capture_time_s=3.25, noise=StateNoise(0.0, 0.0),
        rng=np.random.default_rng(1), message_types=fake_message_types(),
    )
    assert msg.header.stamp.sec == 3
    assert msg.header.stamp.nanosec == pytest.approx(250_000_000, abs=1)


def test_vehicle_odometry_rejects_wrong_orientation_shape():
    with pytest.raises(ValueError):
        vehicle_state_to_odometry(
            orientation_matrix=np.eye(2), angular_velocity=np.zeros(3),
            raw_depth_z=0.0, capture_time_s=0.0, noise=StateNoise(0.01, 0.02),
            rng=np.random.default_rng(1), message_types=fake_message_types(),
        )


def test_algorithm_publishers_never_include_ground_truth_topic():
    topics = build_topic_map()
    assert topics.scoring_truth == "/uw/sim/ground_truth"
    assert topics.scoring_truth not in topics.algorithm_inputs


def test_truth_pose_to_odometry_carries_full_undistorted_pose():
    pose = Pose(translation=np.array([1.0, -2.0, 3.0]), quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]))
    msg = truth_pose_to_odometry(pose, capture_time_s=2.5, message_types=fake_message_types())
    assert msg.pose.pose.position.x == pytest.approx(1.0)
    assert msg.pose.pose.position.y == pytest.approx(-2.0)
    assert msg.pose.pose.position.z == pytest.approx(3.0)
    assert msg.pose.pose.orientation.w == pytest.approx(1.0)
    assert msg.header.stamp.sec == 2
    assert msg.header.stamp.nanosec == pytest.approx(500_000_000, abs=1)


def test_topic_map_excludes_pilot_camera_and_clock_from_algorithm_inputs():
    topics = build_topic_map()
    assert topics.pilot_camera not in topics.algorithm_inputs
    assert topics.clock not in topics.algorithm_inputs
    assert set(topics.algorithm_inputs) == {
        topics.left_camera, topics.right_camera, topics.imaging_sonar, topics.vehicle_state,
    }


def test_camera_conversion_reverses_bgr_to_rgb_and_drops_alpha():
    pixels = np.zeros((2, 3, 4), dtype=np.uint8)
    pixels[..., 0] = 10  # B
    pixels[..., 1] = 20  # G
    pixels[..., 2] = 30  # R
    pixels[..., 3] = 255  # A (dropped)

    msg = holoocean_camera_to_ros_image(
        pixels, capture_time_s=0.0, frame_id="camera_left_link", message_types=fake_message_types(),
    )

    assert msg.encoding == "rgb8"
    assert msg.height == 2
    assert msg.width == 3
    assert msg.step == 3 * 3
    first_pixel = msg.data[:3]
    assert list(first_pixel) == [30, 20, 10]


def test_camera_conversion_rejects_non_uint8():
    pixels = np.zeros((2, 2, 3), dtype=np.float32)
    with pytest.raises(ValueError):
        holoocean_camera_to_ros_image(
            pixels, capture_time_s=0.0, frame_id="x", message_types=fake_message_types(),
        )


def test_sonar_conversion_publishes_raw_intensities_unmirrored():
    intensity = np.array([[0.0, 0.5, 1.0], [0.2, 0.4, 0.6]], dtype=np.float32)
    msg = holoocean_sonar_to_imaging_sonar_msg(
        intensity, capture_time_ns=123, message_types=fake_message_types(),
    )
    assert msg.timestamp == 123
    assert msg.bins_range == 2
    assert msg.bins_azimuth == 3
    assert msg.image_range == pytest.approx(intensity.reshape(-1).tolist())


def test_sonar_conversion_rejects_empty_dimensions():
    with pytest.raises(ValueError):
        holoocean_sonar_to_imaging_sonar_msg(
            np.zeros((0, 5), dtype=np.float32), capture_time_ns=0, message_types=fake_message_types(),
        )


def test_clock_message_matches_sim_time():
    msg = sim_time_to_clock_msg(4.5, fake_message_types())
    assert msg.clock.sec == 4
    assert msg.clock.nanosec == pytest.approx(500_000_000, abs=1)
