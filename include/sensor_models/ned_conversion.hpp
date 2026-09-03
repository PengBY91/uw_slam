// Frame conversions between this repository's world/body convention and
// ArduPilot's (PREP-C-04, docs/ROV平台到货前准备工作规格-2026-09-02.md).
//
// Repository convention (CLAUDE.md: world/body are Z-up; depth z = -depth_m):
//   world = ENU  (x east, y north, z up), right-handed. In a run without an
//           absolute heading reference the x axis is simply the first
//           keyframe's forward direction; the conversion below treats it as
//           east regardless -- the MAVLink adapter only ever sends BODY-frame
//           deltas (VISION_POSITION_DELTA), for which the world convention
//           does not matter, and reads LOCAL_POSITION_NED back only for
//           SITL verification.
//   body  = FLU  (x forward, y left, z up), right-handed.
// ArduPilot convention:
//   world = NED  (x north, y east, z down), body = FRD (x forward, y right,
//           z down).
//
// Both changes of basis are proper rotations (det +1), so they compose with
// Pose3 like any other pose:
//   C_ned_enu = [[0,1,0],[1,0,0],[0,0,-1]]  (180 deg about (1,1,0)/sqrt2)
//   C_frd_flu = diag(1,-1,-1)               (180 deg about x)
// and each is its own inverse.
//
// Everything here is pure Eigen (no third-party, no MAVLink types) so it can
// live in `core`; the MAVLink adapter fills message structs from these.
// Covariance blocks are ordered [translation(3), rotation(3)].
//
// The unit tests pin the axis mapping; PREP-C-03's SITL read-back is the
// end-to-end check -- the camera->body conjugation bug in CLAUDE.md is the
// precedent for why a unit-tested frame conversion still needs that.
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sensor_models/geometry.hpp"

namespace uw::sensor_models {

using Matrix6d = Eigen::Matrix<double, 6, 6>;

// Fixed change-of-basis rotations, exposed for callers that convert bare
// vectors (velocities, angular rates, accelerations).
Eigen::Matrix3d NedFromEnuRotation();  // maps an ENU vector to NED
Eigen::Matrix3d FrdFromFluRotation();  // maps an FLU vector to FRD

// World-frame pose of the body: T_enu_flu -> T_ned_frd and back.
Pose3 WorldPoseToNed(const Pose3& T_enu_flu);
Pose3 WorldPoseFromNed(const Pose3& T_ned_frd);

// Body-frame relative pose (e.g. the delta between two keyframes expressed
// in the earlier keyframe's body frame, what VISION_POSITION_DELTA carries):
// delta_flu -> delta_frd and back. Independent of the world convention.
Pose3 BodyDeltaToFrd(const Pose3& delta_flu);
Pose3 BodyDeltaFromFrd(const Pose3& delta_frd);

// Bare vectors.
Eigen::Vector3d WorldVectorToNed(const Eigen::Vector3d& v_enu);
Eigen::Vector3d BodyVectorToFrd(const Eigen::Vector3d& v_flu);

// Rotation vector (axis * angle, radians) of a unit quaternion, the form
// VISION_POSITION_DELTA's angle_delta expects. Angle is wrapped to [0, pi].
Eigen::Vector3d RotationVector(const Eigen::Quaterniond& q);

// Similarity transform of a [translation, rotation] 6x6 covariance under a
// change of basis C applied to both blocks: blkdiag(C,C) * cov * blkdiag(C,C)^T.
// Use C = FrdFromFluRotation() for a body-frame delta covariance.
Matrix6d RotateCovariance6(const Matrix6d& cov, const Eigen::Matrix3d& C);

}  // namespace uw::sensor_models
