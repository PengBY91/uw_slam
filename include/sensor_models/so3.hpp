// Minimal SO(3) Lie-group helpers for the on-manifold IMU preintegration
// (include/sensor_models/imu_preintegration.hpp) and the 15-dim IMU
// residual that will consume it. Written from the standard definitions
// (Forster et al. 2017 appendix / Barfoot "State Estimation for Robotics"
// section 7), NOT ported from any upstream library — same rule as the
// sonar_range_residual Jacobian (CLAUDE.md): derive, then verify
// numerically (tests/core/so3_test.cpp).
//
// Conventions (must stay consistent with rigid_transform_fit.hpp's
// perturbation model and the future residual):
//   Exp(phi) = I + sin|phi|/|phi| [phi]_x + (1-cos|phi|)/|phi|^2 [phi]_x^2
//   Exp(phi + dphi) ~= Exp(phi) Exp(Jr(phi) dphi)          (right Jacobian)
//   Log(Exp(phi) Exp(dphi)) ~= phi + JrInv(phi) dphi
// Small-angle branches switch below kSmallAngleThreshold so the functions
// are C^1 through zero (no 0/0 at the identity).
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace uw::sensor_models::so3 {

inline constexpr double kSmallAngleThreshold = 1e-6;

// [v]_x, the skew-symmetric matrix with Hat(v) * w == v.cross(w).
Eigen::Matrix3d Hat(const Eigen::Vector3d& v);

// Inverse of Hat for a skew-symmetric matrix (uses the lower triangle; no
// symmetry check — callers pass matrices they built with Hat or from a
// rotation-matrix log).
Eigen::Vector3d Vee(const Eigen::Matrix3d& skew);

// Exponential map so(3) -> SO(3) (Rodrigues).
Eigen::Matrix3d Exp(const Eigen::Vector3d& phi);
Eigen::Quaterniond ExpQuaternion(const Eigen::Vector3d& phi);

// Logarithm map SO(3) -> so(3), |result| in [0, pi]. The input is assumed
// to be a proper rotation (orthonormal, det +1); it is NOT re-orthonormalized.
Eigen::Vector3d Log(const Eigen::Matrix3d& rotation);
Eigen::Vector3d Log(const Eigen::Quaterniond& rotation);

// Right Jacobian of Exp and its inverse (Forster 2017 eq. (8)/(9)).
Eigen::Matrix3d RightJacobian(const Eigen::Vector3d& phi);
Eigen::Matrix3d RightJacobianInverse(const Eigen::Vector3d& phi);

}  // namespace uw::sensor_models::so3
