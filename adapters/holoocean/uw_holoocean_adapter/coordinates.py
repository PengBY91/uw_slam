"""Coordinate transform utilities for the HoloOcean adapter.

Cleaned-up, quaternion-based reimplementation of the coordinate math in
ocean_t/src/svin2_pipeline.py's CoordTransformer (read directly from that
file to port faithfully; ocean_t is this team's own prior code, not a
third-party dependency, so there is no licensing concern in reworking it —
see the platform architecture's ocean_t remediation list, section 22.3).

What changed relative to ocean_t, deliberately:
  - Poses are represented as (translation, quaternion) rather than Euler
    angles internally. ocean_t's _SE3_to_pose has a genuine gimbal-lock
    branch (when cos(pitch) <= 1e-6, yaw is arbitrarily forced to 0) — the
    audit flagged this as a long-term numerical-stability risk. Using
    quaternions throughout (matching core/sensor_models/geometry.hpp's
    Pose3 on the C++ side, for consistency) avoids that branch entirely;
    Euler angles are only ever used at the UE extrinsic-parsing boundary,
    where HoloOcean's config format itself is Euler-based.
  - No module-level extrinsic caching / no hidden JSON_PATH: callers pass
    the scenario config in explicitly.

HoloOcean/Unreal convention: left-handed, Z-up, with location in
centimeters internally (HoloOcean's Python API returns meters) — the Y and
Z axes are flipped relative to this platform's body/world convention
(right-handed, Z-up... actually AUV convention here is right-handed with Y
and Z negated from UE, matching ocean_t's `T_ue_to_auv = diag(1, -1, -1)`).
"""
from __future__ import annotations

import dataclasses

import numpy as np

# UE (location, rotation) -> AUV body frame: flip Y and Z.
_UE_TO_BODY_DIAG = np.diag([1.0, -1.0, -1.0])


@dataclasses.dataclass(frozen=True)
class Pose:
    """Rigid transform: translation (3,) + unit quaternion, order (x, y, z, w)."""

    translation: np.ndarray  # shape (3,)
    quaternion_xyzw: np.ndarray  # shape (4,), unit norm

    @staticmethod
    def identity() -> "Pose":
        return Pose(np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]))

    def rotation_matrix(self) -> np.ndarray:
        return _quaternion_to_matrix(self.quaternion_xyzw)

    def compose(self, other: "Pose") -> "Pose":
        r_self = self.rotation_matrix()
        return Pose(
            translation=self.translation + r_self @ other.translation,
            quaternion_xyzw=_quaternion_multiply(self.quaternion_xyzw, other.quaternion_xyzw),
        )

    def inverse(self) -> "Pose":
        q_inv = self.quaternion_xyzw * np.array([-1.0, -1.0, -1.0, 1.0])
        r_inv = _quaternion_to_matrix(q_inv)
        return Pose(translation=-(r_inv @ self.translation), quaternion_xyzw=q_inv)

    def apply(self, points: np.ndarray) -> np.ndarray:
        """Transforms one point (3,) or an (N,3) array of points."""
        r = self.rotation_matrix()
        if points.ndim == 1:
            return r @ points + self.translation
        return (r @ points.T).T + self.translation

    def to_matrix4(self) -> np.ndarray:
        matrix = np.eye(4)
        matrix[:3, :3] = self.rotation_matrix()
        matrix[:3, 3] = self.translation
        return matrix


def euler_deg_to_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """Intrinsic roll-pitch-yaw (XYZ then applied as Rz @ Ry @ Rx), matching ocean_t's
    convention exactly (same formula as CoordTransformer._build_transform / _pose_to_R)."""
    roll, pitch, yaw = np.deg2rad([roll_deg, pitch_deg, yaw_deg])
    cx, sx = np.cos(roll), np.sin(roll)
    r_x = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    cy, sy = np.cos(pitch), np.sin(pitch)
    r_y = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    cz, sz = np.cos(yaw), np.sin(yaw)
    r_z = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return r_z @ r_y @ r_x


def _quaternion_to_matrix(q_xyzw: np.ndarray) -> np.ndarray:
    x, y, z, w = q_xyzw
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def matrix_to_quaternion(r: np.ndarray) -> np.ndarray:
    """Standard Shepperd's-method-style conversion, numerically stable across all
    rotation angles (unlike an Euler-angle round trip, which has a gimbal-lock branch)."""
    trace = np.trace(r)
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (r[2, 1] - r[1, 2]) * s
        y = (r[0, 2] - r[2, 0]) * s
        z = (r[1, 0] - r[0, 1]) * s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = 2.0 * np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = 2.0 * np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def _quaternion_multiply(a_xyzw: np.ndarray, b_xyzw: np.ndarray) -> np.ndarray:
    ax, ay, az, aw = a_xyzw
    bx, by, bz, bw = b_xyzw
    return np.array(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ]
    )


def build_body_extrinsic(location_m: np.ndarray, rotation_deg: np.ndarray) -> Pose:
    """UE (location, rotation) sensor extrinsic -> body-frame Pose. Matches
    CoordTransformer._build_transform's math exactly (including the [x, -y, -z]
    translation flip), re-expressed with a quaternion rotation instead of a bare matrix."""
    r_sensor = euler_deg_to_matrix(*rotation_deg)
    r_body = _UE_TO_BODY_DIAG @ r_sensor
    x, y, z = location_m
    return Pose(translation=np.array([x, -y, -z]), quaternion_xyzw=matrix_to_quaternion(r_body))


def ue_points_to_body(points_ue: np.ndarray) -> np.ndarray:
    """Flips Y/Z only (no translation) — matches CoordTransformer.body_to_world's
    p_ue construction, factored out so it can be reused/tested independently."""
    flipped = points_ue.copy()
    if flipped.ndim == 1:
        return flipped * np.array([1.0, -1.0, -1.0])
    flipped[:, 1] *= -1.0
    flipped[:, 2] *= -1.0
    return flipped
