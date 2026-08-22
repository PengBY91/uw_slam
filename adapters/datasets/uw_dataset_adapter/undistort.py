"""Plumb-bob (radial-tangential) lens undistortion — a Python port of
include/sensor_models/camera_rectifier.hpp's `UndistortImage`, kept
algorithmically identical on purpose (same forward-remap approach: for each
destination/undistorted pixel, apply the FORWARD distortion model to find
where it came from in the source/distorted image, then bilinearly sample;
out-of-bounds source locations are left black). See that header's own doc
comment for why forward-remap needs no inversion of the distortion
polynomial, and for the exact v1 scope this shares (same K in and out, no
separate rotation-based epipolar step).

This is a second implementation, not a Python binding of the C++ one —
there is no Python/C++ bridge in this repo. Kept deliberately small and
literally parallel to the C++ version (same variable names, same formula
order) so a future reader can diff the two by eye rather than trusting
"they're supposed to match."
"""
from __future__ import annotations

import numpy as np


def undistort_mono8(
    pixel_data: bytes,
    width: int,
    height: int,
    step: int,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    k1: float,
    k2: float,
    p1: float,
    p2: float,
) -> bytes:
    """Undistorts one MONO8 image. `step` is the source's row stride in
    bytes (ROS sensor_msgs/Image's own `step` field) — sliced down to
    `width` before processing in case of row padding, though EuRoC's own
    images have step == width (no padding)."""
    src = np.frombuffer(pixel_data, dtype=np.uint8).reshape(height, step)[:, :width].astype(np.float64)

    u, v = np.meshgrid(np.arange(width, dtype=np.float64), np.arange(height, dtype=np.float64))
    x = (u - cx) / fx
    y = (v - cy) / fy
    r2 = x * x + y * y
    radial = 1.0 + k1 * r2 + k2 * r2 * r2
    x_distorted = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
    y_distorted = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
    src_u = x_distorted * fx + cx
    src_v = y_distorted * fy + cy

    u0 = np.floor(src_u).astype(np.int64)
    v0 = np.floor(src_v).astype(np.int64)
    u1 = u0 + 1
    v1 = v0 + 1
    fu = src_u - u0
    fv = src_v - v0

    valid = (u0 >= 0) & (u1 < width) & (v0 >= 0) & (v1 < height)
    u0c = np.clip(u0, 0, width - 1)
    u1c = np.clip(u1, 0, width - 1)
    v0c = np.clip(v0, 0, height - 1)
    v1c = np.clip(v1, 0, height - 1)

    top = src[v0c, u0c] * (1.0 - fu) + src[v0c, u1c] * fu
    bottom = src[v1c, u0c] * (1.0 - fu) + src[v1c, u1c] * fu
    out = np.where(valid, top * (1.0 - fv) + bottom * fv, 0.0)
    out = np.clip(np.round(out), 0, 255).astype(np.uint8)
    return out.tobytes()
