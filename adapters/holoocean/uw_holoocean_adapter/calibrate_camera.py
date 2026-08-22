"""Camera calibration for HoloOcean's real-rendered stereo rig —
record_session.py's counterpart for closing the third (and last) real-data
blocker documented in the platform notes: MONO8 conversion and a real
corner detector are already verified against a real 78MB HoloOcean
recording, but configs/rig/example_auv.yaml's camera intrinsics
(`k_matrix_row_major: [420,0,320, 0,420,240, 0,0,1]`, `distortion:
[0,0,0,0]`) and the camera_left_link/camera_right_link baseline in its
frame_tree are untouched synthetic placeholders that have never been
checked against the real `OpenWater-HoveringCamera` scenario.

Instead of recording a bag, this spawns a known-size checkerboard target in
the sim, captures stereo pairs from several known-varied agent viewpoints,
and runs standard OpenCV camera calibration (cv2.calibrateCamera +
cv2.stereoCalibrate) to recover real K/distortion/stereo-baseline values.

Only runs where HoloOcean itself can render (native Windows — WSL2's Dozen
translation layer lacks the ray-tracing support HoloOcean needs; see
holoocean_driver.py's module docstring). Run with:

    python -m uw_holoocean_adapter.calibrate_camera --out calibration_report.yaml

Start with a small grid (e.g. --rows-squares 4 --cols-squares 5 — NOT a
square grid, see build_checkerboard_target's docstring for why) and check
the printed per-view corner-detection count before committing to the full
default grid — see build_checkerboard_target's docstring for why this is a
real risk, and its fallback.

Everything HoloOcean-independent (the calibration math, the target
geometry) lives in pure functions with no `holoocean` import, so it is
covered by adapters/holoocean/tests/test_calibrate_camera.py on any
machine, including this one. Everything below `run_calibration_session`
drives a real HoloOceanSession and cv2.findChessboardCorners against real
renders, so — like holoocean_driver.py's HoloOceanSession itself — it is
"written against the documented API, not yet proven" until it has actually
been run on Windows.
"""
from __future__ import annotations

import argparse
import dataclasses
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np
import yaml

from uw_holoocean_adapter.holoocean_driver import HoloOceanSession

# HoloOcean's spawn_prop material list has no "checkerboard"/textured
# option (only solid colors: white/gold/cobblestone/brick/wood/grass/
# steel/black) — see external_repos/HoloOcean/client/src/holoocean/
# environments.py's spawn_prop docstring (read-only reference). The board
# is therefore built out of a black backdrop plate plus a grid of white
# plates covering only the checkerboard's "white" squares.
_BACKDROP_MATERIAL = "black"
_SQUARE_MATERIAL = "white"
_BOARD_THICKNESS_M = 0.02
_SQUARE_THICKNESS_M = 0.03  # spawned slightly proud of the backdrop, see below
_SQUARE_STANDOFF_M = 0.02


@dataclasses.dataclass(frozen=True)
class SpawnCommand:
    prop_type: str
    location: Tuple[float, float, float]
    rotation: Tuple[float, float, float]
    scale: Tuple[float, float, float]
    material: str
    tag: Optional[str] = None


@dataclasses.dataclass(frozen=True)
class CheckerboardTarget:
    """A checkerboard calibration target: where to spawn it in HoloOcean,
    and the corresponding object points cv2.calibrateCamera needs. The
    object points are expressed in the board's own local frame (not
    world/body frame) — that's all cv2.calibrateCamera needs, since it
    solves per-view extrinsics itself; the world placement below only
    matters for spawn_commands and for aiming the agent at the board."""

    pattern_size: Tuple[int, int]  # (inner corners across, inner corners down) — cv2 convention
    object_points: np.ndarray  # (N, 3) float32, board-local frame, Z=0
    spawn_commands: List[SpawnCommand]
    board_origin_m: np.ndarray  # (3,) world location of the board's center
    board_normal: np.ndarray  # (3,) unit vector the board faces (agent should approach from this side)


def build_checkerboard_target(
    rows_squares: int,
    cols_squares: int,
    square_size_m: float,
    board_origin_m: Sequence[float] = (30.0, -20.0, -10.0),
    board_normal: Sequence[float] = (-1.0, 0.0, 0.0),
) -> CheckerboardTarget:
    """Builds a checkerboard target lying in the world Y-Z plane at
    `board_origin_m`, facing along `board_normal` (default: facing toward
    -X, i.e. an agent standing at a smaller X than the board, looking in
    +X, sees it face-on). `board_normal` must be axis-aligned along X
    (`(-1,0,0)` or `(1,0,0)`) — the square layout below only varies the
    board's own Y/Z extent, it does not support an arbitrarily-oriented
    plane.

    rows_squares/cols_squares count physical squares (not inner corners);
    cv2's pattern_size is (cols_squares - 1, rows_squares - 1) inner
    corners, matching cv2.findChessboardCorners' own convention.

    Confirmed against a real HoloOcean run: built from discrete spawn_prop
    boxes rather than a real printed/textured checkerboard, the classic
    cv2.findChessboardCorners detector reliably missed this target even
    when it was clearly, cleanly visible in the captured frame — the 3D
    bevel/shading at each box seam isn't a clean enough saddle point for
    it. collect_calibration_views uses cv2.findChessboardCornersSB (the
    newer "sector based" detector) instead, which found it with no other
    change. If a future HoloOcean/scenario combination makes even SB
    unreliable, the fallback is a sparse grid of `gold`-material spheres +
    cv2.findCirclesGrid instead of this function — same object_points
    math, different spawn_commands/detector.
    """
    if rows_squares < 4 or cols_squares < 4:
        # cv2.findChessboardCorners requires pattern_size (inner corners) > 2
        # on both axes; pattern_size = (squares - 1), so squares must be >= 4.
        # Confirmed the hard way: a 3x3 square grid (2x2 inner corners) threw
        # cv2.error "Both width and height of the pattern should have bigger
        # than 2" on a real HoloOcean run.
        raise ValueError("need at least a 4x4 grid (cv2 requires >2 inner corners per axis)")
    if rows_squares == cols_squares:
        # A square pattern_size is rotationally ambiguous — cv2 can't tell
        # which corner is "first" when the same board is viewed rotated
        # ~90/180/270 degrees, so per-view correspondences silently
        # disagree across views. Confirmed the hard way: a 4x4 grid (3x3
        # inner corners, square) detected corners cleanly on 13/27 real
        # views yet calibrated to nonsense (fx=50 vs fy=203 on one camera,
        # a "stereo baseline" of 6.6m on a rig whose real baseline is
        # ~0.12m). This is a well-known cv2 chessboard-calibration pitfall,
        # not specific to this sim.
        raise ValueError(
            f"rows_squares == cols_squares ({rows_squares}) gives a rotationally-ambiguous "
            "square inner-corner grid — use different row/column counts, e.g. 6x8"
        )
    if square_size_m <= 0:
        raise ValueError("square_size_m must be positive")

    origin = np.asarray(board_origin_m, dtype=np.float64)
    normal = np.asarray(board_normal, dtype=np.float64)
    normal = normal / np.linalg.norm(normal)

    pattern_size = (cols_squares - 1, rows_squares - 1)
    object_points = np.zeros((pattern_size[0] * pattern_size[1], 3), dtype=np.float32)
    object_points[:, :2] = (
        np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2).astype(np.float32) * square_size_m
    )
    # Center the corner grid on the board (board spans rows_squares/cols_squares
    # squares; corners occupy the (pattern_size - 1) * square_size interior).
    board_half_width = (cols_squares * square_size_m) / 2.0
    board_half_height = (rows_squares * square_size_m) / 2.0
    object_points[:, 0] -= (pattern_size[0] - 1) * square_size_m / 2.0
    object_points[:, 1] -= (pattern_size[1] - 1) * square_size_m / 2.0

    spawn_commands = [
        SpawnCommand(
            prop_type="box",
            location=(float(origin[0]), float(origin[1]), float(origin[2])),
            rotation=(0.0, 0.0, 0.0),
            scale=(_BOARD_THICKNESS_M, cols_squares * square_size_m, rows_squares * square_size_m),
            material=_BACKDROP_MATERIAL,
            tag="calib_backdrop",
        )
    ]
    for row in range(rows_squares):
        for col in range(cols_squares):
            if (row + col) % 2 != 0:
                continue  # only the checkerboard's "white" squares are physical boxes
            square_y = origin[1] - board_half_width + (col + 0.5) * square_size_m
            square_z = origin[2] - board_half_height + (row + 0.5) * square_size_m
            # Squares sit proud of the backdrop along +normal (toward the
            # approaching agent), not flush — flush placement would
            # z-fight with the backdrop plate.
            square_x = origin[0] + normal[0] * _SQUARE_STANDOFF_M
            spawn_commands.append(
                SpawnCommand(
                    prop_type="box",
                    location=(float(square_x), float(square_y), float(square_z)),
                    rotation=(0.0, 0.0, 0.0),
                    scale=(_SQUARE_THICKNESS_M, square_size_m, square_size_m),
                    material=_SQUARE_MATERIAL,
                    tag=f"calib_square_{row}_{col}",
                )
            )

    return CheckerboardTarget(
        pattern_size=pattern_size,
        object_points=object_points,
        spawn_commands=spawn_commands,
        board_origin_m=origin,
        board_normal=normal,
    )


def default_calibration_poses(
    target: CheckerboardTarget,
    # Tuned for the default 7x9-square / 0.25m board (2.25m x 1.75m physical
    # size) — confirmed the hard way that these need to scale with board
    # size: a 4x5/0.22m board (~1.1m x 0.88m) needed distances around
    # 1.0-3.7m and lateral offsets around +-0.6m to stay framed; this board
    # is roughly 2x bigger in both dimensions, so distances and lateral
    # offsets are scaled up correspondingly.
    distances_m: Sequence[float] = (2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5),
    lateral_offsets_m: Sequence[float] = (-1.2, -0.9, -0.6, -0.3, 0.0, 0.3, 0.6, 0.9, 1.2),
    vertical_offsets_m: Sequence[float] = (0.0,),
) -> List[Tuple[Tuple[float, float, float], Tuple[float, float, float]]]:
    """Agent (location, rotation-degrees) pairs, all looking roughly at the
    board center from the board-normal side, at varied distance/lateral/
    vertical offsets — the viewpoint diversity cv2.calibrateCamera needs to
    separate focal length from principal point.

    Sign/axis convention for the rotation triplet (HoloOcean teleport's
    [roll, pitch, yaw], degrees) is written to match coordinates.py's UE
    convention. Confirmed against real HoloOcean runs: yaw (from
    lateral_offsets_m) reliably lands the board in frame; pitch (from
    vertical_offsets_m) is noisier — a real run mixing both axes has a
    lower per-pose hit rate on vertical-offset poses than on lateral-only
    ones, plausibly a still-unverified sign/scale issue in pitch, plausibly
    just tighter framing margin. collect_calibration_views records each
    side's detections independently (see its docstring), so a poorly-aimed
    vertical-offset pose just contributes nothing rather than corrupting
    anything — the practical fix for a low overall hit rate is to widen
    distances_m/lateral_offsets_m (more poses, mostly-working axes) rather
    than to lean on vertical_offsets_m for view diversity."""
    poses = []
    for distance in distances_m:
        for lateral in lateral_offsets_m:
            for vertical in vertical_offsets_m:
                # Move from the board along +normal (the side it faces, i.e.
                # where an approaching agent should stand to see it face-on).
                location = target.board_origin_m + target.board_normal * distance
                location = location + np.array([0.0, lateral, vertical])
                rotation = _look_at_euler_deg(location, target.board_origin_m)
                poses.append((tuple(location.tolist()), tuple(rotation.tolist())))
    return poses


def _look_at_euler_deg(from_m: np.ndarray, to_m: np.ndarray) -> np.ndarray:
    delta = to_m - from_m
    yaw = np.degrees(np.arctan2(delta[1], delta[0]))
    horizontal_dist = np.hypot(delta[0], delta[1])
    pitch = np.degrees(np.arctan2(delta[2], horizontal_dist))
    return np.array([0.0, pitch, yaw])


@dataclasses.dataclass(frozen=True)
class CalibrationResult:
    k_matrix: np.ndarray  # (3, 3)
    distortion: np.ndarray  # (N,)
    rms_reprojection_error: float
    num_views_used: int = 0  # 0 means "not tracked" (e.g. a hand-built result in a test)


@dataclasses.dataclass(frozen=True)
class StereoExtrinsicsResult:
    rotation_matrix: np.ndarray  # (3, 3), left-to-right
    translation_m: np.ndarray  # (3,), left-to-right
    rms_reprojection_error: float
    num_pairs_used: int = 0


# Must stay byte-for-byte in sync with OpticalFromBodyRotation() in
# include/sensor_models/camera_model.cpp — this is the same fixed hardware
# mounting constant (body convention x-forward/y-left/z-up -> camera
# OPTICAL convention z-forward/x-right/y-down), not a per-rig calibration
# value, so there is exactly one correct matrix and it must match the C++
# side that actually consumes frame_tree at runtime.
_OPTICAL_FROM_BODY_ROTATION = np.array([
    [0.0, -1.0, 0.0],
    [0.0, 0.0, -1.0],
    [1.0, 0.0, 0.0],
])


def stereo_translation_to_body_frame(
    rotation_matrix: np.ndarray,
    translation_m: np.ndarray,
    rotation_tolerance: float = 1e-2,
) -> np.ndarray:
    """Converts an OpenCV stereo extrinsic (`rotation_matrix`/`translation_m`
    from `compute_stereo_extrinsics`, satisfying cv2's convention
    `P_right_optical = rotation_matrix @ P_left_optical + translation_m`)
    into the right camera's position relative to the left camera, expressed
    in BODY frame (x-forward/y-left/z-up) — the same convention
    `configs/rig/*.yaml`'s `frame_tree` translations use. This is the
    missing step to turn a `calibrate_camera.py` report's
    `left_to_right_transform_row_major` into a paste-able frame_tree delta,
    without hand-deriving the optical/body conjugation each time (exactly
    the class of mistake CLAUDE.md's stereo_landmark_vo_frontend
    optical/body bug warns about — this function exists so that
    conversion only has to be gotten right once, here, with a unit test).

    Requires `rotation_matrix` to be close to identity (within
    `rotation_tolerance`): `include/sensor_models/camera_model.hpp`'s
    `StereoGeometry::Resolve` hard-requires both cameras to share identical
    frame_tree mounting rotation (pure-translation baseline only) — a
    genuinely rotated stereo pair cannot be represented by this rig
    convention at all, so this raises rather than silently discarding the
    rotation or guessing a body-frame answer for it.
    """
    max_deviation = float(np.max(np.abs(rotation_matrix - np.eye(3))))
    if max_deviation > rotation_tolerance:
        raise ValueError(
            f"stereo rotation_matrix is not close to identity (max deviation "
            f"{max_deviation:.4f} > tolerance {rotation_tolerance}) — this rig's frame_tree "
            "convention (StereoGeometry::Resolve) only supports a purely-translational "
            "stereo baseline; a genuinely rotated pair needs a design change, not a forced "
            "conversion here"
        )
    # Position of the right camera's own optical origin, expressed in the
    # LEFT camera's optical frame: substitute the right camera's own origin
    # (whose coordinates in the right frame are, by definition, zero) into
    # `0 = rotation_matrix @ right_in_left_optical + translation_m`.
    right_in_left_optical = -rotation_matrix.T @ translation_m
    # Both cameras share identical mounting rotation relative to base_link
    # (checked above via `rotation_matrix`), so the left camera's own
    # body-convention frame has the same orientation as base_link/body —
    # meaning this one fixed rotation converts the vector all the way to
    # body frame, without needing a per-camera frame_tree lookup here.
    return _OPTICAL_FROM_BODY_ROTATION.T @ right_in_left_optical


def _per_view_reprojection_rms(
    object_points: np.ndarray,
    image_points_per_view: Sequence[np.ndarray],
    k_matrix: np.ndarray,
    distortion: np.ndarray,
    rvecs: Sequence[np.ndarray],
    tvecs: Sequence[np.ndarray],
) -> List[float]:
    errors = []
    for image_points, rvec, tvec in zip(image_points_per_view, rvecs, tvecs):
        projected, _ = cv2.projectPoints(object_points, rvec, tvec, k_matrix, distortion)
        diff = projected.reshape(-1, 2) - image_points
        errors.append(float(np.sqrt(np.mean(np.sum(diff * diff, axis=1)))))
    return errors


def _default_initial_k_guess(image_size: Tuple[int, int], assumed_fov_deg: float = 90.0) -> np.ndarray:
    """A neutral starting point for cv2.calibrateCamera's nonlinear
    refinement — assumes a 90-degree horizontal FOV (HoloOcean's own
    documented default for the sensor types that expose FovAngle at all,
    see calibrate_camera.py's module docstring / the real-install research
    behind it) and centered principal point. Only used to seed the
    optimizer (CALIB_USE_INTRINSIC_GUESS); the actual fx/fy/cx/cy this
    module reports come from the fit, not this guess."""
    width, height = image_size
    focal_px = width / (2.0 * np.tan(np.deg2rad(assumed_fov_deg) / 2.0))
    return np.array([[focal_px, 0.0, width / 2.0], [0.0, focal_px, height / 2.0], [0.0, 0.0, 1.0]])


def compute_intrinsics(
    object_points: np.ndarray,
    image_points_per_view: Sequence[np.ndarray],
    image_size: Tuple[int, int],
    max_view_rms_px: Optional[float] = 2.0,
    min_views: int = 5,
) -> CalibrationResult:
    """Wraps cv2.calibrateCamera. `object_points` is the board-local corner
    grid (same for every view, see CheckerboardTarget); `image_points_per_view`
    is one (N, 2) array of detected corner pixels per view. Pure function —
    no HoloOcean import — so it's testable with synthetic projected points.

    Seeds the optimizer with `_default_initial_k_guess` and constrains
    fx == fy (CALIB_USE_INTRINSIC_GUESS + CALIB_FIX_ASPECT_RATIO) instead
    of letting cv2.calibrateCamera's own linear (DLT-style) initializer run
    unconstrained. Confirmed necessary against real HoloOcean runs: that
    initializer is sensitive to limited view diversity / a few
    inconsistent correspondences and repeatedly converged to unphysical
    solutions (fx and fy differing by 5-10x on a simulated square-pixel
    camera, which cannot actually happen) even with plenty of raw views —
    fixing the aspect ratio rules that failure mode out structurally,
    rather than hoping outlier rejection alone catches every bad case.

    When `max_view_rms_px` is set (the default), iteratively drops the
    single worst-fitting view (by per-view reprojection RMS) and
    recalibrates, stopping once every remaining view is under the
    threshold or `min_views` would be violated — see module docstring for
    why cv2.findChessboardCornersSB's correspondences aren't always
    trustworthy across very different viewing angles of a plain, unmarked
    checkerboard. Pass `max_view_rms_px=None` to disable and use every view
    as-is."""
    if len(image_points_per_view) < 3:
        raise ValueError(f"need at least 3 views to calibrate, got {len(image_points_per_view)}")
    points = list(image_points_per_view)
    initial_k = _default_initial_k_guess(image_size)
    calibrate_flags = cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_ASPECT_RATIO
    while True:
        object_points_per_view = [object_points for _ in points]
        rms, k_matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
            object_points_per_view, points, image_size, initial_k.copy(), None, flags=calibrate_flags
        )
        if max_view_rms_px is None or len(points) <= min_views:
            break
        per_view = _per_view_reprojection_rms(object_points, points, k_matrix, distortion, rvecs, tvecs)
        worst_index = int(np.argmax(per_view))
        if per_view[worst_index] <= max_view_rms_px:
            break
        del points[worst_index]
    return CalibrationResult(
        k_matrix=k_matrix, distortion=distortion.flatten(), rms_reprojection_error=rms, num_views_used=len(points)
    )


def compute_stereo_extrinsics(
    object_points: np.ndarray,
    left_image_points_per_view: Sequence[np.ndarray],
    right_image_points_per_view: Sequence[np.ndarray],
    image_size: Tuple[int, int],
    left_k: np.ndarray,
    left_distortion: np.ndarray,
    right_k: np.ndarray,
    right_distortion: np.ndarray,
    max_pair_rms_px: Optional[float] = 2.0,
    min_pairs: int = 5,
) -> StereoExtrinsicsResult:
    """Wraps cv2.stereoCalibrateExtended (the perViewErrors-returning
    variant of stereoCalibrate) with intrinsics fixed (already known from
    compute_intrinsics on each side) to solve only for the left-to-right
    rigid transform. Pure function, same testability note as
    compute_intrinsics — and the same outlier-rejection rationale: iterates
    dropping the single worst-fitting PAIR (by cv2's own perViewErrors,
    max of the two sides) until every remaining pair is under
    `max_pair_rms_px` or `min_pairs` would be violated. Pass
    `max_pair_rms_px=None` to disable and use every pair as-is."""
    if len(left_image_points_per_view) != len(right_image_points_per_view):
        raise ValueError("left/right view counts must match (same capture sequence)")
    left_points = list(left_image_points_per_view)
    right_points = list(right_image_points_per_view)
    while True:
        object_points_per_view = [object_points for _ in left_points]
        (
            rms,
            _left_k,
            _left_dist,
            _right_k,
            _right_dist,
            rotation,
            translation,
            _essential,
            _fundamental,
            _rvecs,
            _tvecs,
            per_pair_errors,
        ) = cv2.stereoCalibrateExtended(
            object_points_per_view,
            left_points,
            right_points,
            left_k,
            left_distortion,
            right_k,
            right_distortion,
            image_size,
            np.eye(3),
            np.zeros(3),
            flags=cv2.CALIB_FIX_INTRINSIC,
        )
        if max_pair_rms_px is None or len(left_points) <= min_pairs:
            break
        worst_index = int(np.argmax(per_pair_errors.max(axis=1)))
        if per_pair_errors[worst_index].max() <= max_pair_rms_px:
            break
        del left_points[worst_index]
        del right_points[worst_index]
    return StereoExtrinsicsResult(
        rotation_matrix=rotation,
        translation_m=translation.flatten(),
        rms_reprojection_error=rms,
        num_pairs_used=len(left_points),
    )


@dataclasses.dataclass(frozen=True)
class CollectedViews:
    left_points: List[np.ndarray]  # every pose where the LEFT image alone detected the board
    right_points: List[np.ndarray]  # every pose where the RIGHT image alone detected the board
    paired_left_points: List[np.ndarray]  # subset where BOTH detected, same pose, same order
    paired_right_points: List[np.ndarray]
    image_size: Tuple[int, int]


def collect_calibration_views(
    session: HoloOceanSession,
    target: CheckerboardTarget,
    poses: Sequence[Tuple[Tuple[float, float, float], Tuple[float, float, float]]],
    settle_ticks: int = 5,
    max_wait_ticks: int = 60,
) -> CollectedViews:
    """Drives a real HoloOceanSession: teleports the agent through `poses`,
    steps `settle_ticks` ticks to let physics/render settle, then keeps
    stepping (up to `max_wait_ticks` total) until a tick actually carries
    both LeftCamera and RightCamera — cameras publish at their own
    configured Hz, slower than the simulation tick rate (same reason
    record_session.py's _write_keyframe only fires on camera-bearing
    ticks; see its module docstring), so most individual ticks have no
    camera keys in frame.sensors at all.

    Records each side's cv2.findChessboardCornersSB detections
    INDEPENDENTLY (not just poses where both sides succeeded): confirmed
    against a real HoloOcean run that "only one eye detects the board" is
    common (small target relative to distance + the real stereo baseline
    this calibration is trying to discover, both push one eye's framing
    past the margin more often than the other's) — requiring both would
    throw away a large fraction of genuinely usable mono views. Only the
    stereo extrinsic step needs synchronized pairs, so those are tracked
    separately in `paired_*`. Prints a per-pose status line so a real run
    makes it obvious how many views were usable, per this module's "verify
    with a small grid first" guidance."""
    left_points: List[np.ndarray] = []
    right_points: List[np.ndarray] = []
    paired_left_points: List[np.ndarray] = []
    paired_right_points: List[np.ndarray] = []
    image_size: Optional[Tuple[int, int]] = None

    for index, (location, rotation) in enumerate(poses):
        # Re-assert the pose on every tick, not just once before the wait
        # loop: confirmed the hard way against a real HoloOcean install
        # that a HoveringAUV with no thruster command (session.step() is
        # always called with action=None here) drifts under
        # gravity/buoyancy/current over the several dozen ticks it can take
        # for a camera-bearing tick to arrive, even from a zeroed velocity —
        # by ~pose 4-9 of a 27-pose run the checkerboard had drifted clean
        # out of frame. Re-teleporting every tick pins the AUV in place
        # regardless of what the physics step does to it.
        session.teleport_agent(location, rotation)
        for _ in range(settle_ticks):
            session.teleport_agent(location, rotation)
            session.step()

        frame = None
        for _ in range(max_wait_ticks):
            session.teleport_agent(location, rotation)
            candidate = session.step()
            if "LeftCamera" in candidate.sensors and "RightCamera" in candidate.sensors:
                frame = candidate
                break
        if frame is None:
            print(f"pose {index + 1}/{len(poses)}: no camera-bearing tick within {max_wait_ticks} ticks, skipped")
            continue

        left_raw = np.asarray(frame.sensors["LeftCamera"])[:, :, :3].astype(np.uint8)
        right_raw = np.asarray(frame.sensors["RightCamera"])[:, :, :3].astype(np.uint8)
        left_gray = cv2.cvtColor(left_raw, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right_raw, cv2.COLOR_BGR2GRAY)
        if image_size is None:
            image_size = (left_gray.shape[1], left_gray.shape[0])

        # cv2.findChessboardCorners (the classic detector) reliably MISSED
        # this target on a real HoloOcean render even when the board was
        # clearly, cleanly visible and well-framed — confirmed by saving a
        # frame and inspecting it directly. Root cause: the board is built
        # from discrete spawn_prop boxes (see build_checkerboard_target's
        # docstring), so each square boundary has a subtle 3D bevel/shading
        # instead of a flat printed edge, which the classic detector's
        # saddle-point search doesn't like. cv2.findChessboardCornersSB
        # (the newer "sector based" detector) does better and already
        # returns sub-pixel-accurate corners itself (no separate
        # cv2.cornerSubPix pass needed) — but its OWN default flags still
        # missed plenty of real, cleanly-visible-but-tilted views (most
        # calibration poses are intentionally off-axis for view diversity).
        # CALIB_CB_EXHAUSTIVE fixed that: confirmed directly against a saved
        # real frame that SB without it returns False and SB with it
        # returns True on the identical image, no other change.
        sb_flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
        left_found, left_corners = cv2.findChessboardCornersSB(left_gray, target.pattern_size, flags=sb_flags)
        right_found, right_corners = cv2.findChessboardCornersSB(right_gray, target.pattern_size, flags=sb_flags)
        if left_found:
            left_points.append(left_corners.reshape(-1, 2))
        if right_found:
            right_points.append(right_corners.reshape(-1, 2))
        if left_found and right_found:
            paired_left_points.append(left_corners.reshape(-1, 2))
            paired_right_points.append(right_corners.reshape(-1, 2))
        print(f"pose {index + 1}/{len(poses)}: left={left_found} right={right_found}")

    if image_size is None:
        raise RuntimeError("no poses ever produced a camera-bearing tick")
    return CollectedViews(
        left_points=left_points,
        right_points=right_points,
        paired_left_points=paired_left_points,
        paired_right_points=paired_right_points,
        image_size=image_size,
    )


def _spawn_target(session: HoloOceanSession, target: CheckerboardTarget) -> None:
    for command in target.spawn_commands:
        session.spawn_prop(
            command.prop_type,
            command.location,
            rotation=command.rotation,
            scale=command.scale,
            material=command.material,
            tag=command.tag,
        )


def _merge_collected_views(batches: Sequence[CollectedViews]) -> CollectedViews:
    image_sizes = {batch.image_size for batch in batches}
    if len(image_sizes) > 1:
        raise RuntimeError(f"inconsistent image sizes across batches: {image_sizes}")
    return CollectedViews(
        left_points=[p for batch in batches for p in batch.left_points],
        right_points=[p for batch in batches for p in batch.right_points],
        paired_left_points=[p for batch in batches for p in batch.paired_left_points],
        paired_right_points=[p for batch in batches for p in batch.paired_right_points],
        image_size=next(iter(image_sizes)),
    )


def run_calibration_session(
    scenario_name: str,
    seed: int,
    target: CheckerboardTarget,
    poses: Sequence[Tuple[Tuple[float, float, float], Tuple[float, float, float]]],
    settle_ticks: int,
    poses_per_session: int = 18,
) -> Tuple[CalibrationResult, CalibrationResult, Optional[StereoExtrinsicsResult], CollectedViews]:
    """Runs `poses` across several independently-restarted HoloOceanSession
    instances, `poses_per_session` poses at a time, then pools every
    batch's detections before calibrating. Confirmed necessary against a
    real HoloOcean install: a single long session's detection success rate
    silently collapses after roughly the first ~2-4 poses of a much longer
    run — the exact same pose that detects cleanly as a session's 1st or
    2nd teleport (or in a short, isolated session) reliably fails as its
    5th+ teleport. The mechanism behind that collapse isn't understood (not
    physics drift — set_physics_state zeroes velocity and gets re-asserted
    every tick already; not a detector/board-size issue — it reproduced
    identically on both the 4x5 and 7x9 boards); restarting the session
    periodically sidesteps it rather than fixing a root cause. Each
    restart re-spawns the checkerboard target (props don't persist across
    HoloOceanSession's own env.reset()) and pays a fresh HoloOcean env
    startup cost, so this is slower per-pose than one long session would
    be if long sessions actually worked — but a long session's poses past
    the first few are otherwise wasted anyway."""
    batches: List[CollectedViews] = []
    for batch_start in range(0, len(poses), poses_per_session):
        batch_poses = poses[batch_start : batch_start + poses_per_session]
        batch_number = batch_start // poses_per_session + 1
        session = HoloOceanSession(scenario_name, seed)
        try:
            _spawn_target(session, target)
            batch_views = collect_calibration_views(session, target, batch_poses, settle_ticks)
        except RuntimeError as exc:
            print(f"batch {batch_number}: skipped ({exc})")
            continue
        finally:
            session.close()
        batches.append(batch_views)
        print(
            f"batch {batch_number} ({len(batch_poses)} poses, session restarted): "
            f"+{len(batch_views.left_points)} left +{len(batch_views.right_points)} right "
            f"+{len(batch_views.paired_left_points)} pairs"
        )

    if not batches:
        raise RuntimeError("every batch failed to produce a single camera-bearing tick")
    views = _merge_collected_views(batches)

    if len(views.left_points) < 5 or len(views.right_points) < 5:
        raise RuntimeError(
            f"only {len(views.left_points)} left / {len(views.right_points)} right usable mono "
            "views (need >=5 each) — the checkerboard may not be rendering cleanly; see "
            "build_checkerboard_target's docstring for the sphere-grid fallback, or check the "
            "printed per-pose status above"
        )

    left_result = compute_intrinsics(target.object_points, views.left_points, views.image_size)
    right_result = compute_intrinsics(target.object_points, views.right_points, views.image_size)

    stereo_result = None
    if len(views.paired_left_points) >= 5:
        stereo_result = compute_stereo_extrinsics(
            target.object_points,
            views.paired_left_points,
            views.paired_right_points,
            views.image_size,
            left_result.k_matrix,
            left_result.distortion,
            right_result.k_matrix,
            right_result.distortion,
        )
    else:
        print(
            f"only {len(views.paired_left_points)} synchronized stereo pairs (need >=5) — "
            "skipping stereo baseline; mono intrinsics for both cameras are still valid"
        )
    return left_result, right_result, stereo_result, views


def _write_report(
    out_path: str,
    left_result: CalibrationResult,
    right_result: CalibrationResult,
    stereo_result: Optional[StereoExtrinsicsResult],
    views: CollectedViews,
) -> None:
    """Writes a report shaped to be copy-pasted into configs/rig/example_auv.yaml's
    `cameras` list and camera_*_link frame_tree entries — see
    schemas/proto/uw/domain/calibration.proto for the target field names."""
    image_size = views.image_size
    report = {
        "num_left_views": len(views.left_points),
        "num_right_views": len(views.right_points),
        "num_stereo_pairs": len(views.paired_left_points),
        "cameras": [
            {
                "sensor_id": "camera_left",
                "width": image_size[0],
                "height": image_size[1],
                "k_matrix_row_major": left_result.k_matrix.flatten().tolist(),
                "distortion": left_result.distortion.tolist(),
                "distortion_model": "plumb_bob",
                "rms_reprojection_error_px": float(left_result.rms_reprojection_error),
                "num_views_used_after_outlier_rejection": left_result.num_views_used,
            },
            {
                "sensor_id": "camera_right",
                "width": image_size[0],
                "height": image_size[1],
                "k_matrix_row_major": right_result.k_matrix.flatten().tolist(),
                "distortion": right_result.distortion.tolist(),
                "distortion_model": "plumb_bob",
                "rms_reprojection_error_px": float(right_result.rms_reprojection_error),
                "num_views_used_after_outlier_rejection": right_result.num_views_used,
            },
        ],
    }
    if stereo_result is not None:
        stereo_transform = np.eye(4)
        stereo_transform[:3, :3] = stereo_result.rotation_matrix
        stereo_transform[:3, 3] = stereo_result.translation_m
        report["left_to_right_transform_row_major"] = stereo_transform.flatten().tolist()
        report["stereo_rms_reprojection_error_px"] = float(stereo_result.rms_reprojection_error)
        report["num_stereo_pairs_used_after_outlier_rejection"] = stereo_result.num_pairs_used
        try:
            offset_body = stereo_translation_to_body_frame(
                stereo_result.rotation_matrix, stereo_result.translation_m
            )
            # Ready to paste directly into camera_right_link's frame_tree
            # translation, relative to camera_left_link's own translation
            # (add this vector to camera_left_link's x/y/z).
            report["right_camera_offset_from_left_link_body_frame_m"] = offset_body.tolist()
        except ValueError as exc:
            report["right_camera_offset_from_left_link_body_frame_m_error"] = str(exc)
    with open(out_path, "w") as handle:
        yaml.safe_dump(report, handle, sort_keys=False)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", default="OpenWater-HoveringCamera")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--rows-squares", type=int, default=7)
    parser.add_argument("--cols-squares", type=int, default=9)
    parser.add_argument("--square-size-m", type=float, default=0.25)
    parser.add_argument("--settle-ticks", type=int, default=5)
    parser.add_argument(
        "--poses-per-session",
        type=int,
        default=18,
        help="restart the HoloOcean session every this many poses — see run_calibration_session's "
        "docstring for why a single long session's detection rate collapses after the first few",
    )
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    target = build_checkerboard_target(args.rows_squares, args.cols_squares, args.square_size_m)
    poses = default_calibration_poses(target)
    left_result, right_result, stereo_result, views = run_calibration_session(
        args.scenario, args.seed, target, poses, args.settle_ticks, args.poses_per_session
    )
    _write_report(args.out, left_result, right_result, stereo_result, views)

    print(f"usable views: left={len(views.left_points)} right={len(views.right_points)} "
          f"stereo_pairs={len(views.paired_left_points)} (out of {len(poses)} poses); "
          f"kept after outlier rejection: left={left_result.num_views_used} "
          f"right={right_result.num_views_used}"
          + (f" stereo={stereo_result.num_pairs_used}" if stereo_result is not None else ""))
    print(
        f"left:  rms={left_result.rms_reprojection_error:.3f}px "
        f"fx={left_result.k_matrix[0, 0]:.1f} fy={left_result.k_matrix[1, 1]:.1f} "
        f"cx={left_result.k_matrix[0, 2]:.1f} cy={left_result.k_matrix[1, 2]:.1f}"
    )
    print(
        f"right: rms={right_result.rms_reprojection_error:.3f}px "
        f"fx={right_result.k_matrix[0, 0]:.1f} fy={right_result.k_matrix[1, 1]:.1f} "
        f"cx={right_result.k_matrix[0, 2]:.1f} cy={right_result.k_matrix[1, 2]:.1f}"
    )
    if stereo_result is not None:
        print(
            f"stereo: rms={stereo_result.rms_reprojection_error:.3f}px "
            f"baseline_m={np.linalg.norm(stereo_result.translation_m):.4f}"
        )
    print(f"wrote calibration report to {args.out}")


if __name__ == "__main__":
    main()
