#include "uw/sensor_models/geometry.hpp"

namespace uw::sensor_models {

Pose3 Pose3::Identity() { return Pose3{}; }

Pose3 Pose3::operator*(const Pose3& rhs) const {
  Pose3 result;
  result.rotation = (rotation * rhs.rotation).normalized();
  result.translation = translation + rotation * rhs.translation;
  return result;
}

Pose3 Pose3::Inverse() const {
  Pose3 result;
  result.rotation = rotation.conjugate();
  result.translation = -(result.rotation * translation);
  return result;
}

Eigen::Vector3d Pose3::Apply(const Eigen::Vector3d& point_local) const {
  return translation + rotation * point_local;
}

std::array<double, 7> Pose3::ToParameterBlock() const {
  return {translation.x(), translation.y(), translation.z(),
          rotation.x(),    rotation.y(),    rotation.z(), rotation.w()};
}

Pose3 Pose3::FromParameterBlock(const double* seven) {
  Pose3 pose;
  pose.translation = Eigen::Vector3d(seven[0], seven[1], seven[2]);
  pose.rotation = Eigen::Quaterniond(seven[6], seven[3], seven[4], seven[5]);
  pose.rotation.normalize();
  return pose;
}

uw::domain::Transform3D Pose3::ToProto() const {
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
  m.topLeftCorner<3, 3>() = rotation.toRotationMatrix();
  m.topRightCorner<3, 1>() = translation;

  uw::domain::Transform3D transform;
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      transform.add_matrix_row_major(m(row, col));
    }
  }
  return transform;
}

Pose3 Pose3::FromProto(const uw::domain::Transform3D& transform) {
  Pose3 pose;
  if (transform.matrix_row_major_size() != 16) return pose;
  Eigen::Matrix4d m;
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      m(row, col) = transform.matrix_row_major(row * 4 + col);
    }
  }
  pose.translation = m.topRightCorner<3, 1>();
  pose.rotation = Eigen::Quaterniond(m.topLeftCorner<3, 3>()).normalized();
  return pose;
}

}  // namespace uw::sensor_models
