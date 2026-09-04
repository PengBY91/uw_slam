#include "sensor_models/imu_preintegration.hpp"

#include <cmath>

#include "sensor_models/geometry.hpp"
#include "sensor_models/so3.hpp"

namespace uw::sensor_models {

namespace {

bool Finite(const Eigen::Vector3d& v) {
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

template <typename Derived>
bool AllFinite(const Eigen::MatrixBase<Derived>& m) {
  for (int r = 0; r < m.rows(); ++r) {
    for (int c = 0; c < m.cols(); ++c) {
      if (!std::isfinite(m(r, c))) return false;
    }
  }
  return true;
}

void AppendMatrix3(google::protobuf::RepeatedField<double>* out, const Eigen::Matrix3d& m) {
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) out->Add(m(r, c));
  }
}

bool ReadMatrix3(const google::protobuf::RepeatedField<double>& in, Eigen::Matrix3d* out) {
  if (in.size() != 9) return false;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) (*out)(r, c) = in.Get(r * 3 + c);
  }
  return true;
}

bool ReadVector3(const google::protobuf::RepeatedField<double>& in, Eigen::Vector3d* out) {
  if (in.size() != 3) return false;
  *out = Eigen::Vector3d(in.Get(0), in.Get(1), in.Get(2));
  return true;
}

// Every 64 samples the accumulated rotation is re-projected onto SO(3):
// products of Rodrigues matrices drift from orthonormality at ~1e-16 per
// multiply, which over a long interval (200 Hz x tens of seconds) is still
// far below any measurement noise but is free to clean up.
constexpr uint32_t kReorthonormalizeEvery = 64;

}  // namespace

std::optional<ImuPreintegrationNoise> ImuPreintegrationNoise::FromRig(
    const uw::domain::RigCalibrationSnapshot& rig, std::string* error) {
  const auto& model = rig.imu_noise();
  ImuPreintegrationNoise noise;
  noise.sigma_gyro_c = model.sigma_gyro_c();
  noise.sigma_accel_c = model.sigma_accel_c();
  noise.sigma_gyro_bias_walk = model.sigma_gyro_bias_walk_c();
  noise.sigma_accel_bias_walk = model.sigma_accel_bias_walk_c();
  auto positive_finite = [](double v) { return std::isfinite(v) && v > 0.0; };
  if (!positive_finite(noise.sigma_gyro_c) || !positive_finite(noise.sigma_accel_c)) {
    if (error) {
      *error = "rig imu_noise.sigma_gyro_c / sigma_accel_c must be finite and > 0 (got " +
               std::to_string(noise.sigma_gyro_c) + ", " + std::to_string(noise.sigma_accel_c) + ")";
    }
    return std::nullopt;
  }
  if (!positive_finite(noise.sigma_gyro_bias_walk) ||
      !positive_finite(noise.sigma_accel_bias_walk)) {
    if (error) {
      *error = "rig imu_noise.sigma_gyro_bias_walk_c / sigma_accel_bias_walk_c "
               "must be finite and > 0 (got " +
               std::to_string(noise.sigma_gyro_bias_walk) + ", " +
               std::to_string(noise.sigma_accel_bias_walk) + ")";
    }
    return std::nullopt;
  }
  return noise;
}

Eigen::Matrix3d PreintegratedImuDelta::CorrectedRotation(const Eigen::Vector3d& d_bias_gyro) const {
  return delta_rotation * so3::Exp(d_rotation_d_bias_gyro * d_bias_gyro);
}

Eigen::Vector3d PreintegratedImuDelta::CorrectedVelocity(const Eigen::Vector3d& d_bias_gyro,
                                                         const Eigen::Vector3d& d_bias_accel) const {
  return delta_velocity + d_velocity_d_bias_gyro * d_bias_gyro + d_velocity_d_bias_accel * d_bias_accel;
}

Eigen::Vector3d PreintegratedImuDelta::CorrectedPosition(const Eigen::Vector3d& d_bias_gyro,
                                                         const Eigen::Vector3d& d_bias_accel) const {
  return delta_position + d_position_d_bias_gyro * d_bias_gyro + d_position_d_bias_accel * d_bias_accel;
}

uw::domain::ImuPreintegrationMeasurement PreintegratedImuDelta::ToProto() const {
  uw::domain::ImuPreintegrationMeasurement message;
  Pose3 pose;
  pose.translation = delta_position;
  pose.rotation = Eigen::Quaterniond(delta_rotation).normalized();
  *message.mutable_delta_pose() = pose.ToProto();
  for (int i = 0; i < 3; ++i) message.add_delta_velocity_mps(delta_velocity(i));
  message.set_delta_time_s(delta_time_s);
  for (int r = 0; r < 15; ++r) {
    for (int c = 0; c < 15; ++c) message.add_covariance_15x15_row_major(covariance(r, c));
  }
  for (int i = 0; i < 3; ++i) message.add_bias_gyro(bias_gyro(i));
  for (int i = 0; i < 3; ++i) message.add_bias_accel(bias_accel(i));
  AppendMatrix3(message.mutable_d_rotation_d_bias_gyro(), d_rotation_d_bias_gyro);
  AppendMatrix3(message.mutable_d_velocity_d_bias_gyro(), d_velocity_d_bias_gyro);
  AppendMatrix3(message.mutable_d_velocity_d_bias_accel(), d_velocity_d_bias_accel);
  AppendMatrix3(message.mutable_d_position_d_bias_gyro(), d_position_d_bias_gyro);
  AppendMatrix3(message.mutable_d_position_d_bias_accel(), d_position_d_bias_accel);
  message.set_gravity_mps2(gravity_mps2);
  message.set_sample_count(sample_count);
  return message;
}

std::optional<PreintegratedImuDelta> PreintegratedImuDelta::FromProto(
    const uw::domain::ImuPreintegrationMeasurement& message, std::string* error) {
  auto fail = [&](const std::string& why) -> std::optional<PreintegratedImuDelta> {
    if (error) *error = "ImuPreintegrationMeasurement: " + why;
    return std::nullopt;
  };
  PreintegratedImuDelta delta;
  if (message.delta_pose().matrix_row_major_size() != 16) return fail("delta_pose must be a 4x4 matrix");
  const Pose3 pose = Pose3::FromProto(message.delta_pose());
  delta.delta_rotation = pose.rotation.toRotationMatrix();
  delta.delta_position = pose.translation;
  if (!ReadVector3(message.delta_velocity_mps(), &delta.delta_velocity)) {
    return fail("delta_velocity_mps must have 3 entries");
  }
  delta.delta_time_s = message.delta_time_s();
  if (!std::isfinite(delta.delta_time_s) || delta.delta_time_s <= 0.0) {
    return fail("delta_time_s must be finite and > 0");
  }
  if (message.covariance_15x15_row_major_size() != 225) {
    return fail("covariance_15x15_row_major must have 225 entries");
  }
  for (int r = 0; r < 15; ++r) {
    for (int c = 0; c < 15; ++c) delta.covariance(r, c) = message.covariance_15x15_row_major(r * 15 + c);
  }
  if (!ReadVector3(message.bias_gyro(), &delta.bias_gyro) || !ReadVector3(message.bias_accel(), &delta.bias_accel)) {
    return fail("bias_gyro / bias_accel must have 3 entries");
  }
  if (!ReadMatrix3(message.d_rotation_d_bias_gyro(), &delta.d_rotation_d_bias_gyro) ||
      !ReadMatrix3(message.d_velocity_d_bias_gyro(), &delta.d_velocity_d_bias_gyro) ||
      !ReadMatrix3(message.d_velocity_d_bias_accel(), &delta.d_velocity_d_bias_accel) ||
      !ReadMatrix3(message.d_position_d_bias_gyro(), &delta.d_position_d_bias_gyro) ||
      !ReadMatrix3(message.d_position_d_bias_accel(), &delta.d_position_d_bias_accel)) {
    return fail("bias Jacobians must each have 9 entries");
  }
  delta.gravity_mps2 = message.gravity_mps2() > 0.0 ? message.gravity_mps2() : 9.80665;
  delta.sample_count = message.sample_count();
  if (!AllFinite(delta.covariance) || !Finite(delta.delta_velocity) || !Finite(delta.delta_position)) {
    return fail("non-finite entries");
  }
  return delta;
}

ImuPreintegrator::ImuPreintegrator(ImuPreintegrationNoise noise, double gravity_mps2) : noise_(noise) {
  delta_.gravity_mps2 = gravity_mps2;
}

void ImuPreintegrator::Reset(const Eigen::Vector3d& bias_gyro, const Eigen::Vector3d& bias_accel) {
  const double gravity = delta_.gravity_mps2;
  delta_ = PreintegratedImuDelta{};
  delta_.gravity_mps2 = gravity;
  delta_.bias_gyro = bias_gyro;
  delta_.bias_accel = bias_accel;
}

bool ImuPreintegrator::Integrate(const Eigen::Vector3d& gyro_radps, const Eigen::Vector3d& accel_mps2,
                                 double dt_s) {
  if (!Finite(gyro_radps) || !Finite(accel_mps2) || !std::isfinite(dt_s) || dt_s <= 0.0) return false;

  const Eigen::Vector3d w = gyro_radps - delta_.bias_gyro;
  const Eigen::Vector3d a = accel_mps2 - delta_.bias_accel;
  const Eigen::Matrix3d dR = delta_.delta_rotation;  // before this step
  const Eigen::Vector3d phi = w * dt_s;
  const Eigen::Matrix3d exp_phi = so3::Exp(phi);
  const Eigen::Matrix3d jr = so3::RightJacobian(phi);
  const double dt2 = dt_s * dt_s;

  // Midpoint rotation for the accelerometer terms: the specific force is
  // held over [t, t+dt], so rotating it with dR at t (plain Euler, as in
  // the paper) leaves a first-order error |w| dt / 2 that is systematic
  // across steps (at 200 Hz and 0.5 rad/s that is ~1 cm/s of velocity
  // drift per second of integration). Rotating with dR Exp(phi/2) instead
  // makes the velocity/position updates second-order accurate; the
  // covariance and bias recursions below are derived for this choice
  // (tests/core/imu_preintegration_test.cpp checks both numerically).
  const Eigen::Vector3d half_phi = 0.5 * phi;
  const Eigen::Matrix3d exp_half = so3::Exp(half_phi);
  const Eigen::Matrix3d jr_half = so3::RightJacobian(half_phi);
  const Eigen::Matrix3d dR_half = dR * exp_half;
  const Eigen::Matrix3d a_hat = so3::Hat(a);
  // d(dR_half a)/d(delta_phi) for a right perturbation dR Exp(delta_phi):
  // dR Exp(delta_phi) Exp(phi/2) = dR_half Exp(Exp(phi/2)^T delta_phi).
  const Eigen::Matrix3d dacc_dphi = -dR_half * a_hat * exp_half.transpose();
  // d(dR_half a)/d(gyro noise or -bias): phi/2 shifts by -(dt/2) Jr(phi/2) d.
  const Eigen::Matrix3d dacc_dgyro = dR_half * a_hat * jr_half * (0.5 * dt_s);

  // --- Covariance propagation on the 9-dim [dR, dv, dp] block (paper eq.
  // (59)-(63), with the midpoint-rotation terms above); bias blocks are
  // pure random walk, appended below.
  Eigen::Matrix<double, 9, 9> A = Eigen::Matrix<double, 9, 9>::Identity();
  A.block<3, 3>(0, 0) = exp_phi.transpose();
  A.block<3, 3>(3, 0) = dacc_dphi * dt_s;
  A.block<3, 3>(6, 0) = 0.5 * dacc_dphi * dt2;
  A.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity() * dt_s;

  Eigen::Matrix<double, 9, 6> B = Eigen::Matrix<double, 9, 6>::Zero();
  B.block<3, 3>(0, 0) = jr * dt_s;
  B.block<3, 3>(3, 0) = -dacc_dgyro * dt_s;
  B.block<3, 3>(6, 0) = -0.5 * dacc_dgyro * dt2;
  B.block<3, 3>(3, 3) = dR_half * dt_s;
  B.block<3, 3>(6, 3) = 0.5 * dR_half * dt2;

  // Discrete per-sample white noise variance = sigma_c^2 / dt.
  Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
  Q.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * (noise_.sigma_gyro_c * noise_.sigma_gyro_c / dt_s);
  Q.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * (noise_.sigma_accel_c * noise_.sigma_accel_c / dt_s);

  Eigen::Matrix<double, 9, 9> sigma = delta_.covariance.block<9, 9>(0, 0);
  sigma = A * sigma * A.transpose() + B * Q * B.transpose();
  delta_.covariance.block<9, 9>(0, 0) = 0.5 * (sigma + sigma.transpose());
  delta_.covariance.block<3, 3>(9, 9) +=
      Eigen::Matrix3d::Identity() * (noise_.sigma_gyro_bias_walk * noise_.sigma_gyro_bias_walk * dt_s);
  delta_.covariance.block<3, 3>(12, 12) +=
      Eigen::Matrix3d::Identity() * (noise_.sigma_accel_bias_walk * noise_.sigma_accel_bias_walk * dt_s);

  // --- Bias Jacobians (paper appendix, first-order recursions, with the
  // midpoint rotation's own bias sensitivity folded in). A gyro-bias
  // change d shifts dR by Exp(J_R d) and phi/2 by -(dt/2) d, so
  // dR_half(bg + d) = dR_half Exp(J_half d) with
  // J_half = Exp(phi/2)^T J_R - (dt/2) Jr(phi/2). Order matters: the
  // position ones use the velocity ones from BEFORE this step.
  const Eigen::Matrix3d dR_dbg = delta_.d_rotation_d_bias_gyro;
  const Eigen::Matrix3d dv_dbg = delta_.d_velocity_d_bias_gyro;
  const Eigen::Matrix3d dv_dba = delta_.d_velocity_d_bias_accel;
  const Eigen::Matrix3d j_half = exp_half.transpose() * dR_dbg - (0.5 * dt_s) * jr_half;
  const Eigen::Matrix3d dacc_dbg = -dR_half * a_hat * j_half;
  delta_.d_position_d_bias_accel += dv_dba * dt_s - 0.5 * dR_half * dt2;
  delta_.d_position_d_bias_gyro += dv_dbg * dt_s + 0.5 * dacc_dbg * dt2;
  delta_.d_velocity_d_bias_accel = dv_dba - dR_half * dt_s;
  delta_.d_velocity_d_bias_gyro = dv_dbg + dacc_dbg * dt_s;
  delta_.d_rotation_d_bias_gyro = exp_phi.transpose() * dR_dbg - jr * dt_s;

  // --- Nominal deltas (zero-order hold over dt, midpoint rotation).
  delta_.delta_position += delta_.delta_velocity * dt_s + 0.5 * dR_half * a * dt2;
  delta_.delta_velocity += dR_half * a * dt_s;
  delta_.delta_rotation = dR * exp_phi;
  delta_.delta_time_s += dt_s;
  ++delta_.sample_count;
  if (delta_.sample_count % kReorthonormalizeEvery == 0) {
    delta_.delta_rotation = Eigen::Quaterniond(delta_.delta_rotation).normalized().toRotationMatrix();
  }
  return true;
}

}  // namespace uw::sensor_models
