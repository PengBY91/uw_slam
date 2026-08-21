import numpy as np

from uw_holoocean_adapter.coordinates import (
    Pose,
    build_body_extrinsic,
    matrix_to_quaternion,
    pose_sensor_to_pose,
    ue_points_to_body,
)


def test_pose_compose_inverse_round_trips_to_identity():
    pose = Pose(
        translation=np.array([1.0, -2.0, 0.5]),
        quaternion_xyzw=matrix_to_quaternion(
            np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)
        ),
    )
    identity = pose.compose(pose.inverse())
    assert np.allclose(identity.translation, np.zeros(3), atol=1e-9)
    assert np.allclose(np.abs(identity.quaternion_xyzw[3]), 1.0, atol=1e-9)


def test_pose_apply_matches_manual_rotation_translation():
    pose = Pose(translation=np.array([1.0, 0.0, 0.0]), quaternion_xyzw=np.array([0, 0, 0, 1.0]))
    point = np.array([0.0, 2.0, 0.0])
    result = pose.apply(point)
    assert np.allclose(result, np.array([1.0, 2.0, 0.0]))


def test_matrix_to_quaternion_is_stable_near_gimbal_lock_configuration():
    # A rotation matrix that would sit exactly at ocean_t's Euler gimbal-lock
    # branch (pitch = +90 deg) — this must not raise or produce NaNs.
    r = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], dtype=float)
    q = matrix_to_quaternion(r)
    assert np.all(np.isfinite(q))
    assert abs(np.linalg.norm(q) - 1.0) < 1e-9


def test_build_body_extrinsic_flips_y_and_z_translation():
    extrinsic = build_body_extrinsic(np.array([0.3, 0.1, -0.05]), np.array([0.0, 0.0, 0.0]))
    assert np.allclose(extrinsic.translation, np.array([0.3, -0.1, 0.05]))


def test_ue_points_to_body_flips_only_y_and_z():
    points = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
    flipped = ue_points_to_body(points)
    assert np.allclose(flipped, np.array([[1.0, -2.0, -3.0], [4.0, -5.0, -6.0]]))


def test_pose_sensor_to_pose_identity_matrix_gives_identity_pose():
    pose = pose_sensor_to_pose(np.eye(4))
    assert np.allclose(pose.translation, np.zeros(3))
    assert np.allclose(np.abs(pose.quaternion_xyzw[3]), 1.0)


def test_pose_sensor_to_pose_does_not_flip_axes():
    # Unlike build_body_extrinsic, this must NOT flip Y/Z — translation
    # passes through untouched.
    matrix = np.eye(4)
    matrix[:3, 3] = [1.0, 2.0, 3.0]
    pose = pose_sensor_to_pose(matrix)
    assert np.allclose(pose.translation, np.array([1.0, 2.0, 3.0]))


def test_pose_sensor_to_pose_round_trips_a_known_rotation():
    rotation = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])  # 90 deg about Z
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = [0.5, -0.5, 1.5]
    pose = pose_sensor_to_pose(matrix)
    assert np.allclose(pose.to_matrix4(), matrix, atol=1e-9)


def test_pose_sensor_to_pose_rejects_wrong_shape():
    try:
        pose_sensor_to_pose(np.eye(3))
        assert False, "expected ValueError"
    except ValueError:
        pass
