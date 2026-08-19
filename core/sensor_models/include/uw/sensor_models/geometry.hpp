// uw_sensor_models: camera/IMU/sonar/depth physical models, projection and
// noise models (platform architecture section 6). Depends only on
// uw_domain — no ROS/simulator/vendor dependency (section 5 invariant #1).
#pragma once

#include <array>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "uw/domain/domain.hpp"

namespace uw::sensor_models {

// Minimal SE(3) pose: translation + unit quaternion. Not a full Lie-algebra
// library (no Sophus/manif dependency in v1) — just enough for compose /
// inverse / apply and the 7-parameter block layout [tx,ty,tz,qx,qy,qz,qw]
// that algorithms/factor_builders/sonar_range_factor's ported residual
// expects (it mirrors the parameter block layout SVIn/OKVIS uses).
struct Pose3 {
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();

  static Pose3 Identity();

  Pose3 operator*(const Pose3& rhs) const;
  Pose3 Inverse() const;
  Eigen::Vector3d Apply(const Eigen::Vector3d& point_local) const;

  std::array<double, 7> ToParameterBlock() const;
  static Pose3 FromParameterBlock(const double* seven);

  uw::domain::Transform3D ToProto() const;
  static Pose3 FromProto(const uw::domain::Transform3D& transform);
};

}  // namespace uw::sensor_models
