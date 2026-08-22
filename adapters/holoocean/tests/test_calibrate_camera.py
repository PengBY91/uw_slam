import cv2
import numpy as np

from uw_holoocean_adapter.calibrate_camera import (
    build_checkerboard_target,
    compute_intrinsics,
    compute_stereo_extrinsics,
    default_calibration_poses,
    stereo_translation_to_body_frame,
)

_TRUE_K = np.array([[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]])
_IMAGE_SIZE = (640, 480)


def _synthetic_views(object_points, k_matrix, num_views, rng, translation=(0.0, 0.0, 1.5)):
    """Projects the same object points through `num_views` randomized
    (but moderate — keeps the board fully in frame) camera poses, matching
    what cv2.findChessboardCorners would hand back from real detections."""
    views = []
    for _ in range(num_views):
        rvec = rng.uniform(-0.25, 0.25, size=3)
        tvec = np.array(translation) + rng.uniform(-0.15, 0.15, size=3)
        projected, _ = cv2.projectPoints(object_points, rvec, tvec, k_matrix, None)
        views.append(projected.reshape(-1, 2).astype(np.float32))
    return views


def test_compute_intrinsics_recovers_known_k():
    target = build_checkerboard_target(rows_squares=7, cols_squares=9, square_size_m=0.05)
    rng = np.random.default_rng(0)
    views = _synthetic_views(target.object_points, _TRUE_K, num_views=15, rng=rng)

    result = compute_intrinsics(target.object_points, views, _IMAGE_SIZE)

    assert abs(result.k_matrix[0, 0] - _TRUE_K[0, 0]) < 5.0
    assert abs(result.k_matrix[1, 1] - _TRUE_K[1, 1]) < 5.0
    assert abs(result.k_matrix[0, 2] - _TRUE_K[0, 2]) < 5.0
    assert abs(result.k_matrix[1, 2] - _TRUE_K[1, 2]) < 5.0
    assert result.rms_reprojection_error < 0.5


def test_compute_intrinsics_rejects_outlier_view():
    """Reproduces the real-run failure mode this rejection logic exists
    for: one view whose correspondence order got scrambled (standing in
    for cv2.findChessboardCornersSB's occasionally-inconsistent corner
    ordering across very different real viewing angles) should get dropped
    automatically rather than dragging fx/fy off by a large factor."""
    target = build_checkerboard_target(rows_squares=7, cols_squares=9, square_size_m=0.05)
    rng = np.random.default_rng(3)
    views = _synthetic_views(target.object_points, _TRUE_K, num_views=15, rng=rng)
    scrambled = views[0].copy()
    rng.shuffle(scrambled)  # same points, wrong correspondence to object_points
    views[0] = scrambled

    naive = compute_intrinsics(target.object_points, views, _IMAGE_SIZE, max_view_rms_px=None)
    robust = compute_intrinsics(target.object_points, views, _IMAGE_SIZE, max_view_rms_px=2.0)

    assert abs(naive.k_matrix[0, 0] - _TRUE_K[0, 0]) > 20.0  # confirms the corrupted view IS disruptive
    assert abs(robust.k_matrix[0, 0] - _TRUE_K[0, 0]) < 5.0
    assert abs(robust.k_matrix[1, 1] - _TRUE_K[1, 1]) < 5.0
    assert robust.num_views_used < len(views)


def test_compute_intrinsics_rejects_too_few_views():
    target = build_checkerboard_target(rows_squares=7, cols_squares=9, square_size_m=0.05)
    rng = np.random.default_rng(1)
    views = _synthetic_views(target.object_points, _TRUE_K, num_views=2, rng=rng)
    try:
        compute_intrinsics(target.object_points, views, _IMAGE_SIZE)
        assert False, "expected ValueError"
    except ValueError:
        pass


def test_compute_stereo_extrinsics_recovers_known_baseline():
    target = build_checkerboard_target(rows_squares=7, cols_squares=9, square_size_m=0.05)
    rng = np.random.default_rng(2)
    left_views = _synthetic_views(target.object_points, _TRUE_K, num_views=15, rng=rng)

    true_baseline_m = 0.12
    right_views = _synthetic_views(
        target.object_points,
        _TRUE_K,
        num_views=15,
        rng=np.random.default_rng(2),  # same seed: same left-camera-frame poses as left_views
        translation=(true_baseline_m, 0.0, 1.5),
    )

    left_result = compute_intrinsics(target.object_points, left_views, _IMAGE_SIZE)
    right_result = compute_intrinsics(target.object_points, right_views, _IMAGE_SIZE)
    stereo_result = compute_stereo_extrinsics(
        target.object_points,
        left_views,
        right_views,
        _IMAGE_SIZE,
        left_result.k_matrix,
        left_result.distortion,
        right_result.k_matrix,
        right_result.distortion,
    )

    recovered_baseline_m = np.linalg.norm(stereo_result.translation_m)
    assert abs(recovered_baseline_m - true_baseline_m) < 0.02
    assert stereo_result.rms_reprojection_error < 0.5


def test_stereo_translation_to_body_frame_recovers_known_body_offset():
    """A rig where the right camera sits at (0, -0.12, 0) relative to the
    left camera in BODY frame (x-forward/y-left/z-up) — i.e. 0.12m to the
    physical right, matching example_auv.yaml's own placeholder baseline
    layout — should round-trip through the optical-frame R/T convention
    and back out to the same body-frame offset. Independently re-derives
    the optical-frame vector here (not by calling the function under test)
    to avoid a tautological check."""
    optical_from_body = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])
    true_offset_body = np.array([0.0, -0.12, 0.0])
    right_in_left_optical = optical_from_body @ true_offset_body
    rotation_matrix = np.eye(3)
    translation_m = -rotation_matrix.T @ right_in_left_optical  # cv2 convention: P_r = R@P_l + T

    recovered = stereo_translation_to_body_frame(rotation_matrix, translation_m)

    np.testing.assert_allclose(recovered, true_offset_body, atol=1e-9)


def test_stereo_translation_to_body_frame_rejects_non_identity_rotation():
    rotation_matrix = cv2.Rodrigues(np.array([0.0, 0.3, 0.0]))[0]  # a real ~17 degree rotation
    translation_m = np.array([-0.12, 0.0, 0.0])
    try:
        stereo_translation_to_body_frame(rotation_matrix, translation_m)
        assert False, "expected ValueError"
    except ValueError:
        pass


def test_build_checkerboard_target_alternates_squares_and_matches_pattern_size():
    target = build_checkerboard_target(rows_squares=5, cols_squares=4, square_size_m=0.1)

    assert target.pattern_size == (3, 4)  # (cols - 1, rows - 1)
    assert target.object_points.shape == (3 * 4, 3)

    square_commands = [c for c in target.spawn_commands if c.tag != "calib_backdrop"]
    backdrop_commands = [c for c in target.spawn_commands if c.tag == "calib_backdrop"]
    assert len(backdrop_commands) == 1
    assert backdrop_commands[0].material == "black"
    # Exactly half (rounded) of a 5x4 grid is "white" in a checkerboard pattern.
    assert len(square_commands) == 10
    assert all(c.material == "white" for c in square_commands)


def test_build_checkerboard_target_rejects_undersized_grid():
    try:
        build_checkerboard_target(rows_squares=2, cols_squares=5, square_size_m=0.1)
        assert False, "expected ValueError"
    except ValueError:
        pass


def test_build_checkerboard_target_rejects_square_grid():
    try:
        build_checkerboard_target(rows_squares=6, cols_squares=6, square_size_m=0.1)
        assert False, "expected ValueError"
    except ValueError:
        pass


def test_default_calibration_poses_face_toward_board():
    target = build_checkerboard_target(rows_squares=7, cols_squares=9, square_size_m=0.05)
    poses = default_calibration_poses(target, distances_m=(3.0,), lateral_offsets_m=(0.0,), vertical_offsets_m=(0.0,))

    assert len(poses) == 1
    location, _rotation = poses[0]
    # board_normal is (-1, 0, 0) by default: the agent should end up on the
    # -X side of the board, i.e. at a smaller X than board_origin_m.
    assert location[0] < target.board_origin_m[0]
