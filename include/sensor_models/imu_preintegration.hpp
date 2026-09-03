// On-manifold IMU preintegration (Forster, Carlone, Dellaert, Scaramuzza,
// "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry",
// IEEE T-RO 2017, sections V-VI) — the math shared by
// include/frontends/imu_preintegration_frontend.hpp (which turns ImuSample
// streams into ImuPreintegrationMeasurement evidence) and the 15-dim IMU
// residual in factor_builders (PREP-B-01 step 2). It lives in sensor_models
// because both of those layers may depend on it but not on each other
// (tools/lint/check_layer_dependencies.py).
//
// Written from the paper's equations, not ported from GTSAM/OKVIS/VINS —
// same rule as the sonar_range_residual Jacobian (CLAUDE.md): derive,
// then verify numerically. tests/core/imu_preintegration_test.cpp checks
// the deltas against analytic trajectories and the bias Jacobians /
// covariance against finite differences and Monte-Carlo.
//
// Definitions (i = start keyframe, j = end keyframe, all in the body frame
// of keyframe i; world frame is Z-up so g_w = (0, 0, -gravity_mps2)):
//   dR_ij = R_i^T R_j
//   dv_ij = R_i^T (v_j - v_i - g_w dt_ij)
//   dp_ij = R_i^T (p_j - p_i - v_i dt_ij - 0.5 g_w dt_ij^2)
// Per IMU interval k -> k+1 of length dt with bias-corrected gyro w and
// accel a (zero-order hold; the accelerometer term uses the midpoint
// rotation dR Exp(w dt / 2) instead of the paper's plain dR, which removes
// a first-order systematic error — see the .cpp):
//   dR <- dR Exp(w dt);  dv <- dv + dR_half a dt;  dp <- dp + dv dt + 0.5 dR_half a dt^2
// Covariance and bias Jacobians follow the paper's linearised recursions
// (eq. (59)-(63) / appendix), re-derived for the midpoint rotation. The state ordering everywhere here and on
// the wire is [dR(3), dv(3), dp(3), bg(3), ba(3)]; the bias block is the
// random-walk covariance accumulated over dt_ij (paper eq. (65)).
#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "domain/domain.hpp"

namespace uw::sensor_models {

// Continuous-time noise densities and random-walk densities. Units:
//   sigma_gyro_c          rad/s/sqrt(Hz)   (angle random walk)
//   sigma_accel_c         m/s^2/sqrt(Hz)   (velocity random walk)
//   sigma_gyro_bias_walk  rad/s^2/sqrt(Hz) (gyro bias random walk)
//   sigma_accel_bias_walk m/s^3/sqrt(Hz)   (accel bias random walk)
// Per-sample discrete sigma for an interval dt is sigma_c / sqrt(dt).
struct ImuPreintegrationNoise {
  double sigma_gyro_c = 0.0;
  double sigma_accel_c = 0.0;
  double sigma_gyro_bias_walk = 0.0;
  double sigma_accel_bias_walk = 0.0;

  // Reads the rig's ImuNoiseModel. sigma_*_bias_walk_c is used when > 0;
  // otherwise sigma_*_bias (the rig's older "bias sigma" field, which
  // every checked-in rig sets while none sets the *_walk_c fields) is
  // taken as the random-walk density. Returns nullopt with `error` set
  // when a white-noise density is zero/non-finite, because a singular
  // covariance would make the residual's whitening blow up silently.
  static std::optional<ImuPreintegrationNoise> FromRig(
      const uw::domain::RigCalibrationSnapshot& rig, std::string* error);
};

// The preintegrated quantities between two keyframes plus everything a
// residual needs to re-linearise for a small bias change.
struct PreintegratedImuDelta {
  Eigen::Matrix3d delta_rotation = Eigen::Matrix3d::Identity();   // dR_ij
  Eigen::Vector3d delta_velocity = Eigen::Vector3d::Zero();       // dv_ij
  Eigen::Vector3d delta_position = Eigen::Vector3d::Zero();       // dp_ij
  double delta_time_s = 0.0;
  uint32_t sample_count = 0;

  // Linearisation point.
  Eigen::Vector3d bias_gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d bias_accel = Eigen::Vector3d::Zero();

  // d(delta)/d(bias) at the linearisation point.
  Eigen::Matrix3d d_rotation_d_bias_gyro = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d d_velocity_d_bias_gyro = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d d_velocity_d_bias_accel = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d d_position_d_bias_gyro = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d d_position_d_bias_accel = Eigen::Matrix3d::Zero();

  // [dR, dv, dp, bg, ba] covariance.
  Eigen::Matrix<double, 15, 15> covariance = Eigen::Matrix<double, 15, 15>::Zero();

  double gravity_mps2 = 9.80665;

  // First-order bias correction (paper eq. (44)): the deltas that would
  // have been integrated at (bias_gyro + d_bg, bias_accel + d_ba).
  Eigen::Matrix3d CorrectedRotation(const Eigen::Vector3d& d_bias_gyro) const;
  Eigen::Vector3d CorrectedVelocity(const Eigen::Vector3d& d_bias_gyro,
                                    const Eigen::Vector3d& d_bias_accel) const;
  Eigen::Vector3d CorrectedPosition(const Eigen::Vector3d& d_bias_gyro,
                                    const Eigen::Vector3d& d_bias_accel) const;

  // Wire round-trip. ToProto fills every field of the message (including
  // the five Jacobians); FromProto returns nullopt with `error` set when
  // the message is missing/ill-sized (rejects rather than defaulting,
  // since a zero covariance would be read as "infinitely confident").
  uw::domain::ImuPreintegrationMeasurement ToProto() const;
  static std::optional<PreintegratedImuDelta> FromProto(
      const uw::domain::ImuPreintegrationMeasurement& message, std::string* error);
};

// Incremental integrator. Reset(bias) starts a new interval; Integrate()
// consumes one raw (biased, gravity-including) body-frame IMU reading held
// for `dt_s`. Body frame == the frame the deltas are expressed in; the
// caller (frontend) is responsible for rotating imu_link readings into
// base_link first.
class ImuPreintegrator {
 public:
  explicit ImuPreintegrator(ImuPreintegrationNoise noise, double gravity_mps2 = 9.80665);

  void Reset(const Eigen::Vector3d& bias_gyro, const Eigen::Vector3d& bias_accel);

  // Returns false (and leaves the state untouched) for non-finite input or
  // dt_s <= 0.
  bool Integrate(const Eigen::Vector3d& gyro_radps, const Eigen::Vector3d& accel_mps2, double dt_s);

  const PreintegratedImuDelta& delta() const { return delta_; }
  const ImuPreintegrationNoise& noise() const { return noise_; }

 private:
  ImuPreintegrationNoise noise_;
  PreintegratedImuDelta delta_;
};

}  // namespace uw::sensor_models
