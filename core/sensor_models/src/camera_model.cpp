#include "uw/sensor_models/camera_model.hpp"

#include <optional>

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
        m(row, col) = edge.transform().matrix_row_major(row * 4 + col);
      }
    }
    return m;
  }
  return std::nullopt;
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

  const auto left_transform = FindEdgeTransform(rig, left_frame);
  const auto right_transform = FindEdgeTransform(rig, right_frame);
  if (!left_transform.has_value() || !right_transform.has_value()) return geometry;

  const Eigen::Matrix3d left_rotation = left_transform->topLeftCorner<3, 3>();
  const Eigen::Matrix3d right_rotation = right_transform->topLeftCorner<3, 3>();
  if (!left_rotation.isApprox(right_rotation, 1e-9)) return geometry;

  geometry.left = PinholeCamera::FromIntrinsics(*left_intrinsics);
  geometry.right = PinholeCamera::FromIntrinsics(*right_intrinsics);
  geometry.baseline_m =
      (left_transform->topRightCorner<3, 1>() - right_transform->topRightCorner<3, 1>()).norm();
  geometry.valid = geometry.baseline_m > 1e-9;
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
