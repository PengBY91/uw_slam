#pragma once

#include <cstdint>
#include <string>

#include <Eigen/Core>

#include "domain/domain.hpp"

namespace uw::sensor_models {

// Returns the calibrated intrinsics for `sensor_id`, or nullptr if the rig
// has no camera with that id.
const uw::domain::CameraIntrinsics* FindCamera(const uw::domain::RigCalibrationSnapshot& rig,
                                                const std::string& sensor_id);

// Pinhole intrinsics only. v1 assumes already-undistorted pixel coordinates
// — CameraIntrinsics.distortion is not applied here. This mirrors
// StereoGeometry's parallel-rig assumption below: general
// undistort+rectify (image warping for arbitrary distortion/orientation)
// is out of scope until a later plan needs a non-parallel or distorted rig.
struct PinholeCamera {
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  uint32_t width = 0;
  uint32_t height = 0;

  static PinholeCamera FromIntrinsics(const uw::domain::CameraIntrinsics& intrinsics);

  // Projects a point already expressed in this camera's frame (z > 0 is
  // the caller's responsibility to check) to a pixel coordinate.
  Eigen::Vector2d Project(const Eigen::Vector3d& point_camera) const;
  Eigen::Vector3d Unproject(double u, double v, double depth_m) const;
};

// Validated geometry for a RECTIFIED stereo pair. Resolve() requires: both
// cameras' K matrices to be well-formed and finite, identical image size,
// and the frame_tree edges (converted to OPTICAL convention) to describe
// an orientation-identical, purely-horizontal baseline with a POSITIVE x
// component -- i.e. exactly the geometry opencv_adapters::
// StereoRectificationContext produces (identity fast path or the general
// OpenCV-calibration path alike). `baseline_m` is always positive and
// matches the disparity_px = left_u - right_u sign convention used
// throughout frontends/ (BlockMatcher, StereoLandmarkVoFrontend). `valid`
// is false rather than silently producing wrong/negated depth for a
// misaligned or left/right-swapped pair.
struct StereoGeometry {
  PinholeCamera left;
  PinholeCamera right;
  double baseline_m = 0.0;
  bool valid = false;

  static StereoGeometry Resolve(const uw::domain::RigCalibrationSnapshot& rig,
                                 const std::string& left_sensor_id,
                                 const std::string& left_frame,
                                 const std::string& right_sensor_id,
                                 const std::string& right_frame);
};

// Fixed axis-convention rotation relating a rig body-frame camera link
// (x-forward/y-left/z-up — this platform's frame_tree convention, matching
// the sonar's own local-frame convention) to that
// same physical camera's OPTICAL frame (z-forward/x-right/y-down, REP-103).
// This is a hardware-mounting constant, not a per-rig calibration value.
// Project()/Unproject() above operate in optical convention; any caller
// that derives a camera-frame point via RigCalibrationSnapshot's
// frame_tree Pose3 composition (body convention) must rotate through this
// before calling Project/Unproject. StereoOpticalDepthFrontend does not
// need this conversion because it only uses baseline/fx magnitudes, not a
// composed Pose3.
const Eigen::Matrix3d& OpticalFromBodyRotation();

}  // namespace uw::sensor_models
