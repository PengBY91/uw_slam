#include "sensor_models/ned_conversion.hpp"

#include <cmath>

namespace uw::sensor_models {

namespace {

Pose3 RotationOnlyPose(const Eigen::Matrix3d& R) {
  Pose3 pose;
  pose.rotation = Eigen::Quaterniond(R).normalized();
  return pose;
}

}  // namespace

Eigen::Matrix3d NedFromEnuRotation() {
  Eigen::Matrix3d C;
  C << 0, 1, 0,  //
      1, 0, 0,   //
      0, 0, -1;
  return C;
}

Eigen::Matrix3d FrdFromFluRotation() { return Eigen::Vector3d(1.0, -1.0, -1.0).asDiagonal(); }

Pose3 WorldPoseToNed(const Pose3& T_enu_flu) {
  // T_ned_frd = C_ned_enu * T_enu_flu * C_flu_frd, with C_flu_frd == C_frd_flu.
  return RotationOnlyPose(NedFromEnuRotation()) * T_enu_flu * RotationOnlyPose(FrdFromFluRotation());
}

Pose3 WorldPoseFromNed(const Pose3& T_ned_frd) {
  // Both basis changes are involutions, so the inverse has the same form.
  return RotationOnlyPose(NedFromEnuRotation()) * T_ned_frd * RotationOnlyPose(FrdFromFluRotation());
}

Pose3 BodyDeltaToFrd(const Pose3& delta_flu) {
  const Pose3 C = RotationOnlyPose(FrdFromFluRotation());
  return C * delta_flu * C;
}

Pose3 BodyDeltaFromFrd(const Pose3& delta_frd) { return BodyDeltaToFrd(delta_frd); }

Eigen::Vector3d WorldVectorToNed(const Eigen::Vector3d& v_enu) { return NedFromEnuRotation() * v_enu; }

Eigen::Vector3d BodyVectorToFrd(const Eigen::Vector3d& v_flu) { return FrdFromFluRotation() * v_flu; }

Eigen::Vector3d RotationVector(const Eigen::Quaterniond& q) {
  Eigen::Quaterniond unit = q.normalized();
  if (unit.w() < 0.0) unit.coeffs() *= -1.0;  // shortest arc: angle in [0, pi]
  const Eigen::AngleAxisd aa(unit);
  return aa.axis() * aa.angle();
}

Matrix6d RotateCovariance6(const Matrix6d& cov, const Eigen::Matrix3d& C) {
  Matrix6d J = Matrix6d::Zero();
  J.topLeftCorner<3, 3>() = C;
  J.bottomRightCorner<3, 3>() = C;
  return J * cov * J.transpose();
}

}  // namespace uw::sensor_models
