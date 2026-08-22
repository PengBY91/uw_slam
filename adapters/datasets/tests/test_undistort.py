"""Closed-form correctness check, mirroring
tests/core/camera_rectifier_test.cpp's `MatchesClosedFormForLinearRampUnderRadialDistortion`
test on the C++ side: a source image whose pixel value is a known LINEAR
function of NORMALIZED-x (not warped through distortion), so the expected
output at each destination pixel is independently computable from the same
forward-distortion formula undistort_mono8 uses internally — without
needing to invert the distortion polynomial or trust any external oracle."""
from __future__ import annotations

import numpy as np

from uw_dataset_adapter.undistort import undistort_mono8


def _linear_ramp_image(width, height, fx, cx):
    u = np.arange(width, dtype=np.float64)
    x_norm = (u - cx) / fx
    row = np.clip(np.round(128.0 + 80.0 * x_norm), 0, 255).astype(np.uint8)
    return np.tile(row, (height, 1)).tobytes()


def test_matches_closed_form_for_linear_ramp_under_radial_distortion():
    size = 100
    fx = fy = 100.0
    cx = cy = 50.0
    k1, k2, p1, p2 = 0.05, 0.0, 0.0, 0.0

    raw = _linear_ramp_image(size, size, fx, cx)
    out = undistort_mono8(raw, size, size, size, fx, fy, cx, cy, k1, k2, p1, p2)
    out_arr = np.frombuffer(out, dtype=np.uint8).reshape(size, size)

    for v in range(20, 80, 5):
        for u in range(20, 80, 5):
            x_norm = (u - cx) / fx
            y_norm = (v - cy) / fy
            r2 = x_norm * x_norm + y_norm * y_norm
            radial = 1.0 + k1 * r2 + k2 * r2 * r2
            x_distorted = x_norm * radial
            expected = 128.0 + 80.0 * x_distorted
            actual = float(out_arr[v, u])
            assert abs(actual - expected) <= 2.0, f"u={u} v={v} expected={expected} actual={actual}"


def test_identity_distortion_is_close_to_source():
    size = 50
    fx = fy = 60.0
    cx = cy = 25.0
    raw = _linear_ramp_image(size, size, fx, cx)
    out = undistort_mono8(raw, size, size, size, fx, fy, cx, cy, 0.0, 0.0, 0.0, 0.0)
    src_arr = np.frombuffer(raw, dtype=np.uint8).reshape(size, size)
    out_arr = np.frombuffer(out, dtype=np.uint8).reshape(size, size)
    # Interior pixels only — border pixels can legitimately sample outside
    # the source under zero distortion too, since this maps like-for-like.
    np.testing.assert_allclose(
        out_arr[10:40, 10:40].astype(int), src_arr[10:40, 10:40].astype(int), atol=1
    )
