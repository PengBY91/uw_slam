#include "factor_builders/imu_preintegration_residual.hpp"

#include <cmath>

#include <Eigen/Geometry>

#include "sensor_models/geometry.hpp"
#include "sensor_models/so3.hpp"

namespace uw::factor_builders {

namespace {

using Eigen::Matrix3d;
using Eigen::Vector3d;
using uw::sensor_models::Pose3;
namespace so3 = uw::sensor_models::so3;

using Matrix15d = Eigen::Matrix<double, 15, 15>;
using Vector15d = Eigen::Matrix<double, 15, 1>;
using Jacobian15x6 = Eigen::Matrix<double, 15, 6>;
using Jacobian15x9 = Eigen::Matrix<double, 15, 9>;

// d q_raw / d dphi = 0.5 * Q, with Q^T Q = I and Q^T q = 0. Rows are
// ordered [qx, qy, qz, qw] to match Pose3's parameter block.
Eigen::Matrix<double, 4, 3> QuaternionTangentMap(const Eigen::Quaterniond& q) {
  Eigen::Matrix<double, 4, 3> mat;
  mat.topRows<3>() = q.w() * Matrix3d::Identity() + so3::Hat(q.vec());
  mat.bottomRows<1>() = -q.vec().transpose();
  return mat;
}

// Writes a 15 x N row-major Jacobian block.
template <typename Derived>
void WriteJacobian(const Eigen::MatrixBase<Derived>& source, double* out) {
  const int cols = static_cast<int>(source.cols());
  for (int row = 0; row < 15; ++row) {
    for (int col = 0; col < cols; ++col) out[row * cols + col] = source(row, col);
  }
}

// Chains a 15x6 [d/dp | d/ddphi] minimal Jacobian onto the raw 7-parameter
// [t(3), q(4)] block. See the header for why this is exact.
void WritePoseJacobian(const Jacobian15x6& minimal, const double* raw_params, double* out) {
  const Eigen::Quaterniond raw(raw_params[6], raw_params[3], raw_params[4], raw_params[5]);
  const double norm = raw.norm();
  Eigen::Matrix<double, 15, 7> full = Eigen::Matrix<double, 15, 7>::Zero();
  full.leftCols<3>() = minimal.leftCols<3>();
  if (norm > 1e-12) {
    const Eigen::Quaterniond unit = raw.normalized();
    full.rightCols<4>() =
        minimal.rightCols<3>() * (2.0 / norm) * QuaternionTangentMap(unit).transpose();
  }
  WriteJacobian(full, out);
}

}  // namespace

ImuPreintegrationResidual::ImuPreintegrationResidual(
    uw::sensor_models::PreintegratedImuDelta delta, Matrix15d sqrt_information)
    : delta_(std::move(delta)), sqrt_information_(std::move(sqrt_information)) {}

bool ImuPreintegrationResidual::Evaluate(const std::vector<const double*>& parameters,
                                          double* residuals,
                                          std::vector<double*>* jacobians) const {
  if (parameters.size() < 4) return false;

  const Pose3 pose_i = Pose3::FromParameterBlock(parameters[0]);
  const Pose3 pose_j = Pose3::FromParameterBlock(parameters[2]);
  const Eigen::Map<const Vector3d> v_i(parameters[1]);
  const Eigen::Map<const Vector3d> bg_i(parameters[1] + 3);
  const Eigen::Map<const Vector3d> ba_i(parameters[1] + 6);
  const Eigen::Map<const Vector3d> v_j(parameters[3]);
  const Eigen::Map<const Vector3d> bg_j(parameters[3] + 3);
  const Eigen::Map<const Vector3d> ba_j(parameters[3] + 6);

  const Matrix3d R_i = pose_i.rotation.toRotationMatrix();
  const Matrix3d R_j = pose_j.rotation.toRotationMatrix();
  const Matrix3d R_i_T = R_i.transpose();
  const double dt = delta_.delta_time_s;
  // World frame is Z-up (CLAUDE.md), so gravity points along -Z.
  const Vector3d gravity_W(0.0, 0.0, -delta_.gravity_mps2);

  // First-order re-linearisation for the current bias estimate; the
  // linearisation point b_bar is whatever the frontend integrated at.
  const Vector3d d_bias_gyro = bg_i - delta_.bias_gyro;
  const Vector3d d_bias_accel = ba_i - delta_.bias_accel;
  const Vector3d gyro_correction = delta_.d_rotation_d_bias_gyro * d_bias_gyro;
  const Matrix3d delta_rotation_corrected = delta_.CorrectedRotation(d_bias_gyro);
  const Vector3d delta_velocity_corrected =
      delta_.CorrectedVelocity(d_bias_gyro, d_bias_accel);
  const Vector3d delta_position_corrected =
      delta_.CorrectedPosition(d_bias_gyro, d_bias_accel);

  const Matrix3d rotation_error = delta_rotation_corrected.transpose() * R_i_T * R_j;
  const Vector3d velocity_gap = v_j - v_i - gravity_W * dt;
  const Vector3d position_gap =
      pose_j.translation - pose_i.translation - v_i * dt - 0.5 * gravity_W * dt * dt;

  Vector15d raw = Vector15d::Zero();
  const Vector3d r_rotation = so3::Log(rotation_error);
  raw.segment<3>(0) = r_rotation;
  raw.segment<3>(3) = R_i_T * velocity_gap - delta_velocity_corrected;
  raw.segment<3>(6) = R_i_T * position_gap - delta_position_corrected;
  raw.segment<3>(9) = bg_j - bg_i;
  raw.segment<3>(12) = ba_j - ba_i;

  const Vector15d whitened = sqrt_information_ * raw;
  for (int i = 0; i < 15; ++i) {
    if (!std::isfinite(whitened(i))) return false;
    residuals[i] = whitened(i);
  }

  if (jacobians == nullptr || jacobians->size() < 4) return true;

  const Matrix3d jr_inv = so3::RightJacobianInverse(r_rotation);
  const Matrix3d jr_gyro = so3::RightJacobian(gyro_correction);

  // --- pose_i: columns [dp_i (world) | dphi_i (right perturbation)] ---
  Jacobian15x6 j_pose_i = Jacobian15x6::Zero();
  j_pose_i.block<3, 3>(6, 0) = -R_i_T;                                   // r_p / dp_i
  j_pose_i.block<3, 3>(0, 3) = -jr_inv * R_j.transpose() * R_i;          // r_R / dphi_i
  j_pose_i.block<3, 3>(3, 3) = so3::Hat(R_i_T * velocity_gap);           // r_v / dphi_i
  j_pose_i.block<3, 3>(6, 3) = so3::Hat(R_i_T * position_gap);           // r_p / dphi_i

  // --- inertial_i: columns [v_i | bg_i | ba_i] ---
  Jacobian15x9 j_inertial_i = Jacobian15x9::Zero();
  j_inertial_i.block<3, 3>(3, 0) = -R_i_T;
  j_inertial_i.block<3, 3>(6, 0) = -R_i_T * dt;
  j_inertial_i.block<3, 3>(0, 3) =
      -jr_inv * rotation_error.transpose() * jr_gyro * delta_.d_rotation_d_bias_gyro;
  j_inertial_i.block<3, 3>(3, 3) = -delta_.d_velocity_d_bias_gyro;
  j_inertial_i.block<3, 3>(6, 3) = -delta_.d_position_d_bias_gyro;
  j_inertial_i.block<3, 3>(9, 3) = -Matrix3d::Identity();
  j_inertial_i.block<3, 3>(3, 6) = -delta_.d_velocity_d_bias_accel;
  j_inertial_i.block<3, 3>(6, 6) = -delta_.d_position_d_bias_accel;
  j_inertial_i.block<3, 3>(12, 6) = -Matrix3d::Identity();

  // --- pose_j ---
  Jacobian15x6 j_pose_j = Jacobian15x6::Zero();
  j_pose_j.block<3, 3>(6, 0) = R_i_T;   // r_p / dp_j
  j_pose_j.block<3, 3>(0, 3) = jr_inv;  // r_R / dphi_j

  // --- inertial_j ---
  Jacobian15x9 j_inertial_j = Jacobian15x9::Zero();
  j_inertial_j.block<3, 3>(3, 0) = R_i_T;
  j_inertial_j.block<3, 3>(9, 3) = Matrix3d::Identity();
  j_inertial_j.block<3, 3>(12, 6) = Matrix3d::Identity();

  // Whiten before chaining onto the raw quaternion columns (the two
  // operations commute; doing it here keeps the chain code size-agnostic).
  if ((*jacobians)[0] != nullptr) {
    WritePoseJacobian(Jacobian15x6(sqrt_information_ * j_pose_i), parameters[0], (*jacobians)[0]);
  }
  if ((*jacobians)[1] != nullptr) {
    WriteJacobian(Jacobian15x9(sqrt_information_ * j_inertial_i), (*jacobians)[1]);
  }
  if ((*jacobians)[2] != nullptr) {
    WritePoseJacobian(Jacobian15x6(sqrt_information_ * j_pose_j), parameters[2], (*jacobians)[2]);
  }
  if ((*jacobians)[3] != nullptr) {
    WriteJacobian(Jacobian15x9(sqrt_information_ * j_inertial_j), (*jacobians)[3]);
  }
  return true;
}

}  // namespace uw::factor_builders
