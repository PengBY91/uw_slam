#include "sensor_models/camera_model.hpp"

#include <cmath>
#include <optional>

#include <Eigen/Geometry>

#include "sensor_models/geometry.hpp"

namespace uw::sensor_models {

namespace {

const uw::domain::CameraIntrinsics* FindCamera(const uw::domain::RigCalibrationSnapshot& rig,
                                                const std::string& sensor_id) {
  for (const auto& camera : rig.cameras()) {
    if (camera.sensor_id().value() == sensor_id) return &camera;
  }
  return nullptr;
}

std::optional<Eigen::Matrix4d> FindEdgeTransform(const uw::domain::RigCalibrationSnapshot& rig,
                                                  const std::string& child_frame) {
  for (const auto& edge : rig.frame_tree()) {
    if (edge.child_frame().value() != child_frame) continue;
    if (edge.transform().matrix_row_major_size() != 16) return std::nullopt;
    Eigen::Matrix4d m;
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        const double v = edge.transform().matrix_row_major(row * 4 + col);
        if (!std::isfinite(v)) return std::nullopt;
        m(row, col) = v;
      }
    }
    return m;
  }
  return std::nullopt;
}

bool ValidK(const uw::domain::CameraIntrinsics& intrinsics) {
  if (intrinsics.k_matrix_row_major_size() != 9) return false;
  for (double v : intrinsics.k_matrix_row_major()) {
    if (!std::isfinite(v)) return false;
  }
  return intrinsics.k_matrix_row_major(0) > 0.0 && intrinsics.k_matrix_row_major(4) > 0.0;
}

Pose3 PoseFromMatrix(const Eigen::Matrix4d& m) {
  Pose3 pose;
  pose.translation = m.topRightCorner<3, 1>();
  pose.rotation = Eigen::Quaterniond(m.topLeftCorner<3, 3>()).normalized();
  return pose;
}

// The zero-translation Pose3 that converts a point already expressed in a
// camera's BODY-convention link frame into that same camera's OPTICAL frame
// -- see OpticalFromBodyRotation()'s own doc comment for the axis mapping.
Pose3 LinkToOptical() {
  Pose3 pose;
  pose.rotation = Eigen::Quaterniond(OpticalFromBodyRotation()).conjugate();
  return pose;
}

}  // namespace

PinholeCamera PinholeCamera::FromIntrinsics(const uw::domain::CameraIntrinsics& intrinsics) {
  PinholeCamera camera;
  camera.width = intrinsics.width();
  camera.height = intrinsics.height();
  if (intrinsics.k_matrix_row_major_size() == 9) {
    camera.fx = intrinsics.k_matrix_row_major(0);
    camera.cx = intrinsics.k_matrix_row_major(2);
    camera.fy = intrinsics.k_matrix_row_major(4);
    camera.cy = intrinsics.k_matrix_row_major(5);
  }
  return camera;
}

Eigen::Vector2d PinholeCamera::Project(const Eigen::Vector3d& point_camera) const {
  return Eigen::Vector2d(fx * point_camera.x() / point_camera.z() + cx,
                         fy * point_camera.y() / point_camera.z() + cy);
}

Eigen::Vector3d PinholeCamera::Unproject(double u, double v, double depth_m) const {
  return Eigen::Vector3d((u - cx) / fx * depth_m, (v - cy) / fy * depth_m, depth_m);
}

StereoGeometry StereoGeometry::Resolve(const uw::domain::RigCalibrationSnapshot& rig,
                                       const std::string& left_sensor_id,
                                       const std::string& left_frame,
                                       const std::string& right_sensor_id,
                                       const std::string& right_frame) {
  StereoGeometry geometry;
  const auto* left_intrinsics = FindCamera(rig, left_sensor_id);
  const auto* right_intrinsics = FindCamera(rig, right_sensor_id);
  if (left_intrinsics == nullptr || right_intrinsics == nullptr) return geometry;
  if (!ValidK(*left_intrinsics) || !ValidK(*right_intrinsics)) return geometry;
  if (left_intrinsics->width() == 0 || left_intrinsics->height() == 0) return geometry;
  if (left_intrinsics->width() != right_intrinsics->width() ||
      left_intrinsics->height() != right_intrinsics->height()) {
    return geometry;
  }

  const auto left_transform = FindEdgeTransform(rig, left_frame);
  const auto right_transform = FindEdgeTransform(rig, right_frame);
  if (!left_transform.has_value() || !right_transform.has_value()) return geometry;

  const Pose3 link_to_optical = LinkToOptical();
  const Pose3 b_T_left_optical = PoseFromMatrix(*left_transform) * link_to_optical;
  const Pose3 b_T_right_optical = PoseFromMatrix(*right_transform) * link_to_optical;
  const Pose3 left_T_right = b_T_left_optical.Inverse() * b_T_right_optical;

  if (!left_T_right.rotation.toRotationMatrix().isApprox(Eigen::Matrix3d::Identity(), 1e-8)) {
    return geometry;
  }
  if (std::abs(left_T_right.translation.y()) > 1e-8 ||
      std::abs(left_T_right.translation.z()) > 1e-8) {
    return geometry;
  }
  // Disparity sign convention (BlockMatcher, StereoLandmarkVoFrontend): a
  // point in view produces disparity_px = left_u - right_u > 0, which only
  // holds when the right camera's optical origin sits at a POSITIVE x
  // offset from the left camera's -- not merely a nonzero one.
  if (left_T_right.translation.x() <= 0.0) return geometry;

  geometry.left = PinholeCamera::FromIntrinsics(*left_intrinsics);
  geometry.right = PinholeCamera::FromIntrinsics(*right_intrinsics);
  geometry.baseline_m = left_T_right.translation.x();
  geometry.valid = true;
  return geometry;
}

const Eigen::Matrix3d& OpticalFromBodyRotation() {
  static const Eigen::Matrix3d kRotation = (Eigen::Matrix3d() <<
       0.0, -1.0,  0.0,
       0.0,  0.0, -1.0,
       1.0,  0.0,  0.0).finished();
  return kRotation;
}

}  // namespace uw::sensor_models
