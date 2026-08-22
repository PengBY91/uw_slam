#pragma once

#include <optional>

#include <Eigen/Core>

#include "domain/domain.hpp"

namespace uw::sensor_models {

// Plumb-bob (radial-tangential) lens distortion, matching
// CameraIntrinsics.distortion's convention when distortion_model ==
// "plumb_bob" (this repo's only supported model, and the default written by
// uw::runtime::LoadRigConfig — see configs/rig/example_auv_real_camera.yaml
// for real calibrated coefficients): [k1, k2, p1, p2] (4 values) or
// [k1, k2, p1, p2, k3] (5 values). This is the same convention ROS/OpenCV
// call "plumb_bob" — deliberately not the OpenCV library itself, which this
// repo does not depend on (see cmake/Dependencies.cmake).
struct PlumbBobDistortion {
  double k1 = 0.0;
  double k2 = 0.0;
  double p1 = 0.0;
  double p2 = 0.0;
  double k3 = 0.0;

  // True when every coefficient is exactly zero — i.e. the source is
  // already distortion-free (true of every synthetic rig in configs/rig/
  // today) and UndistortImage() below can skip processing entirely.
  bool IsIdentity() const;

  // Returns std::nullopt if intrinsics.distortion_model() is set to
  // something other than "plumb_bob", or distortion_size() is a length
  // this model doesn't recognize (must be 0, 4, or 5).
  static std::optional<PlumbBobDistortion> FromIntrinsics(
      const uw::domain::CameraIntrinsics& intrinsics);
};

// Applies the forward plumb-bob distortion model to a point already
// normalized by the camera's inverse K (i.e. (u - cx) / fx, (v - cy) / fy
// for an UNDISTORTED pixel (u, v)), returning the corresponding DISTORTED
// normalized coordinate. Exposed standalone (not just inside UndistortImage)
// so the polynomial itself can be unit-tested against known-good values
// independent of image warping/interpolation.
Eigen::Vector2d ApplyPlumbBobDistortion(const PlumbBobDistortion& distortion,
                                         const Eigen::Vector2d& normalized_undistorted);

// Removes lens distortion from one MONO8/RGB8/BGR8 ImageFrame using
// `intrinsics` (which must match the frame's width/height), producing a new
// ImageFrame with `is_rectified` set to true. v1 scope, matching
// camera_model.hpp's existing StereoGeometry assumption (identical camera
// orientation, purely translational baseline): this only removes lens
// distortion against the SAME K the frame already uses — it does not
// reproject to a different K, and it does not perform a separate
// rotation-based epipolar rectification. That is sufficient to make a
// StereoGeometry::Resolve()-valid pair's rows epipolar-aligned once
// distortion is removed; it is not a general off-axis rectifier (still out
// of scope, per camera_model.hpp's own note).
//
// Uses the standard "remap" approach: for each destination (undistorted)
// pixel, the FORWARD distortion model locates where it came from in the
// source (distorted) image, which is then bilinearly sampled — no inversion
// of the distortion polynomial is needed. Destination pixels whose source
// location falls outside the source image are left black (0), matching
// typical remap border behavior.
//
// Returns std::nullopt if `raw` fails uw::domain::ValidateImageFrame, its
// dimensions don't match `intrinsics`, or `intrinsics` names an
// unsupported distortion model (see PlumbBobDistortion::FromIntrinsics).
// If the distortion is identity (all coefficients zero — true of every
// synthetic rig today), returns `raw` unchanged, INCLUDING its existing
// is_rectified value — there is nothing to rectify, so this deliberately
// does not claim credit for work it didn't do.
std::optional<uw::domain::ImageFrame> UndistortImage(const uw::domain::ImageFrame& raw,
                                                       const uw::domain::CameraIntrinsics& intrinsics);

}  // namespace uw::sensor_models
