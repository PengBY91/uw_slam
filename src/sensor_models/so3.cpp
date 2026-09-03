#include "sensor_models/so3.hpp"

#include <cmath>

namespace uw::sensor_models::so3 {

Eigen::Matrix3d Hat(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(),
       v.z(), 0.0, -v.x(),
       -v.y(), v.x(), 0.0;
  return m;
}

Eigen::Vector3d Vee(const Eigen::Matrix3d& skew) {
  return Eigen::Vector3d(skew(2, 1), skew(0, 2), skew(1, 0));
}

Eigen::Matrix3d Exp(const Eigen::Vector3d& phi) {
  const double theta_sq = phi.squaredNorm();
  const Eigen::Matrix3d k = Hat(phi);
  if (theta_sq < kSmallAngleThreshold * kSmallAngleThreshold) {
    // Second-order Taylor expansion: I + K + K^2/2 (error O(theta^3)).
    return Eigen::Matrix3d::Identity() + k + 0.5 * k * k;
  }
  const double theta = std::sqrt(theta_sq);
  return Eigen::Matrix3d::Identity() + (std::sin(theta) / theta) * k +
         ((1.0 - std::cos(theta)) / theta_sq) * k * k;
}

Eigen::Quaterniond ExpQuaternion(const Eigen::Vector3d& phi) {
  const double theta = phi.norm();
  if (theta < kSmallAngleThreshold) {
    Eigen::Quaterniond q(1.0, 0.5 * phi.x(), 0.5 * phi.y(), 0.5 * phi.z());
    q.normalize();
    return q;
  }
  return Eigen::Quaterniond(Eigen::AngleAxisd(theta, phi / theta));
}

Eigen::Vector3d Log(const Eigen::Matrix3d& rotation) {
  // cos(theta) = (tr(R) - 1) / 2, clamped against round-off outside [-1, 1].
  const double cos_theta = std::max(-1.0, std::min(1.0, 0.5 * (rotation.trace() - 1.0)));
  const double theta = std::acos(cos_theta);
  if (theta < kSmallAngleThreshold) {
    // R ~= I + [phi]_x  =>  phi = Vee(R - R^T) / 2 (first order).
    return 0.5 * Vee(rotation - rotation.transpose());
  }
  if (theta > M_PI - 1e-5) {
    // Near pi, sin(theta) -> 0 and the generic formula loses precision;
    // use the quaternion route which stays well conditioned there.
    return Log(Eigen::Quaterniond(rotation));
  }
  return (theta / (2.0 * std::sin(theta))) * Vee(rotation - rotation.transpose());
}

Eigen::Vector3d Log(const Eigen::Quaterniond& rotation) {
  Eigen::Quaterniond q = rotation.normalized();
  if (q.w() < 0.0) q.coeffs() = -q.coeffs();  // shortest arc, angle in [0, pi]
  const double vec_norm = q.vec().norm();
  if (vec_norm < kSmallAngleThreshold) {
    return 2.0 * q.vec() / q.w();  // theta/sin(theta/2) -> 2 as theta -> 0
  }
  const double theta = 2.0 * std::atan2(vec_norm, q.w());
  return (theta / vec_norm) * q.vec();
}

Eigen::Matrix3d RightJacobian(const Eigen::Vector3d& phi) {
  const double theta_sq = phi.squaredNorm();
  const Eigen::Matrix3d k = Hat(phi);
  if (theta_sq < kSmallAngleThreshold * kSmallAngleThreshold) {
    return Eigen::Matrix3d::Identity() - 0.5 * k + (1.0 / 6.0) * k * k;
  }
  const double theta = std::sqrt(theta_sq);
  return Eigen::Matrix3d::Identity() - ((1.0 - std::cos(theta)) / theta_sq) * k +
         ((theta - std::sin(theta)) / (theta_sq * theta)) * k * k;
}

Eigen::Matrix3d RightJacobianInverse(const Eigen::Vector3d& phi) {
  const double theta_sq = phi.squaredNorm();
  const Eigen::Matrix3d k = Hat(phi);
  if (theta_sq < kSmallAngleThreshold * kSmallAngleThreshold) {
    return Eigen::Matrix3d::Identity() + 0.5 * k + (1.0 / 12.0) * k * k;
  }
  const double theta = std::sqrt(theta_sq);
  // 1/theta^2 - (1 + cos theta) / (2 theta sin theta) == 1/theta^2 - cot(theta/2) / (2 theta)
  const double coeff = 1.0 / theta_sq - (1.0 + std::cos(theta)) / (2.0 * theta * std::sin(theta));
  return Eigen::Matrix3d::Identity() + 0.5 * k + coeff * k * k;
}

}  // namespace uw::sensor_models::so3
