"""Covers the portable helpers in scorer_ros_bridge.py -- `_SimTimeTracker`,
`_stamp_to_seconds`, `_pose_from_odometry`. `run_scorer_bridge` itself needs
rclpy (not installed on this dev machine, see module docstring) and is not
exercised here."""
import time

import numpy as np
import pytest

from uw_holoocean_adapter.scorer_ros_bridge import _SimTimeTracker, _pose_from_odometry, _stamp_to_seconds


class _FakeStamp:
    def __init__(self, sec, nanosec):
        self.sec = sec
        self.nanosec = nanosec


class _FakeVector3:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z


class _FakeQuaternion:
    def __init__(self, x, y, z, w):
        self.x, self.y, self.z, self.w = x, y, z, w


class _FakePose:
    def __init__(self, position, orientation):
        self.position = position
        self.orientation = orientation


class _FakePoseWithCovariance:
    def __init__(self, pose):
        self.pose = pose


class _FakeOdometry:
    def __init__(self, position, orientation):
        self.pose = _FakePoseWithCovariance(_FakePose(position, orientation))


def test_stamp_to_seconds_combines_sec_and_nanosec():
    assert _stamp_to_seconds(_FakeStamp(3, 250_000_000)) == pytest.approx(3.25)


def test_pose_from_odometry_reads_position_and_orientation():
    msg = _FakeOdometry(_FakeVector3(1.0, -2.0, 3.0), _FakeQuaternion(0.0, 0.0, 0.7071, 0.7071))
    pose = _pose_from_odometry(msg)
    np.testing.assert_allclose(pose.translation, [1.0, -2.0, 3.0])
    np.testing.assert_allclose(pose.quaternion_xyzw, [0.0, 0.0, 0.7071, 0.7071])


def test_sim_time_tracker_returns_none_before_first_observation():
    tracker = _SimTimeTracker()
    assert tracker.estimate_now() is None


def test_sim_time_tracker_extrapolates_from_wall_clock_elapsed():
    tracker = _SimTimeTracker()
    tracker.observe_truth(100.0)
    now = tracker.estimate_now()
    assert now is not None
    # No meaningful wall-clock time has passed yet -- should be ~100.0, not
    # exactly equal (real time.monotonic() elapses a hair between calls).
    assert now == pytest.approx(100.0, abs=0.05)


def test_sim_time_tracker_reanchors_on_new_truth_observation():
    tracker = _SimTimeTracker()
    tracker.observe_truth(100.0)
    time.sleep(0.02)
    tracker.observe_truth(200.0)  # fresh anchor -- old elapsed wall time must not carry over
    now = tracker.estimate_now()
    assert now == pytest.approx(200.0, abs=0.05)
