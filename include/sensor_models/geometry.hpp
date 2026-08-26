// sensor_models (part of the `core` library): camera/IMU/sonar/depth
// physical models, projection and noise models (platform architecture
// section 6). Depends only on `domain` — no ROS/simulator/vendor
// dependency (section 5 invariant #1).
#pragma once

#include <array>
#include <optional>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "domain/domain.hpp"

namespace uw::sensor_models {

// Minimal SE(3) pose: translation + unit quaternion. Not a full Lie-algebra
// library (no Sophus/manif dependency in v1) — just enough for compose /
// inverse / apply and the 7-parameter block layout [tx,ty,tz,qx,qy,qz,qw]
// that the sonar_range_factor's ported residual
// (include/factor_builders/sonar_range_residual.hpp) expects (it mirrors
// the parameter block layout SVIn/OKVIS uses).
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

// Returns the pose of `child_frame` from its rig frame_tree edge, or
// nullopt if no edge targets that frame.
std::optional<Pose3> FindEdgePose(const uw::domain::RigCalibrationSnapshot& rig,
                                   const std::string& child_frame);

}  // namespace uw::sensor_models
