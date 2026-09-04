#include "frontends/imu_stationary_initializer.hpp"

#include <algorithm>
#include <cmath>

#include "sensor_models/geometry.hpp"
#include "sensor_models/imu_preintegration.hpp"

namespace uw::frontends {

namespace {

bool ReadVector3(const google::protobuf::RepeatedField<double>& values, Eigen::Vector3d* out) {
  if (values.size() != 3) return false;
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(values.Get(i))) return false;
    (*out)(i) = values.Get(i);
  }
  return true;
}

bool PositiveFinite(double value) { return std::isfinite(value) && value > 0.0; }

}  // namespace

std::optional<ImuStationaryInitialization> InitializeFromStationaryWindow(
    const std::vector<uw::domain::ImuSample>& samples, double window_end_time_s,
    const uw::domain::RigCalibrationSnapshot& rig,
    const ImuStationaryInitializerParams& params) {
  // The same four densities the preintegrator insists on: without them
  // there is no white-noise scale to put on the measured means, so there is
  // nothing to fall back to either.
  std::string noise_error;
  const auto noise = uw::sensor_models::ImuPreintegrationNoise::FromRig(rig, &noise_error);
  if (!noise.has_value()) return std::nullopt;

  const auto& imu_noise = rig.imu_noise();
  const double gravity_mps2 = imu_noise.gravity_mps2();
  const double sigma_gyro_bias = imu_noise.sigma_gyro_bias();
  const double sigma_accel_bias = imu_noise.sigma_accel_bias();
  if (!PositiveFinite(gravity_mps2) || !PositiveFinite(sigma_gyro_bias) ||
      !PositiveFinite(sigma_accel_bias)) {
    return std::nullopt;
  }
  if (!PositiveFinite(params.wide_velocity_sigma_mps) ||
      !PositiveFinite(params.stationary_velocity_sigma_mps) ||
      !PositiveFinite(params.stationary_window_s) ||
      params.stationary_window_s < params.min_stationary_duration_s) {
    return std::nullopt;
  }

  Eigen::Matrix3d R_body_imu = Eigen::Matrix3d::Identity();
  const auto extrinsic = uw::sensor_models::FindEdgePose(rig, params.imu_frame);
  if (extrinsic.has_value()) {
    R_body_imu = extrinsic->rotation.toRotationMatrix();
  } else if (params.require_extrinsic) {
    return std::nullopt;
  }
  // The lever arm is deliberately not corrected for: a stationary body has
  // zero angular rate and zero angular acceleration, so both lever-arm
  // terms vanish identically. Applying the frontend's centripetal
  // correction here would be arithmetic on zero.

  ImuStationaryInitialization result;

  Eigen::Vector3d gyro_sum = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_sum = Eigen::Vector3d::Zero();
  double first_time_s = 0.0;
  double last_time_s = 0.0;
  for (const auto& sample : samples) {
    if (!params.imu_sensor_id.empty() &&
        sample.header().sensor_id().value() != params.imu_sensor_id) {
      continue;
    }
    const double time_s = uw::domain::ToSeconds(sample.header().capture_time());
    if (!std::isfinite(time_s) || time_s > window_end_time_s ||
        time_s < window_end_time_s - params.stationary_window_s) {
      continue;
    }

    Eigen::Vector3d gyro_imu;
    Eigen::Vector3d accel_imu;
    if (!ReadVector3(sample.angular_velocity_radps(), &gyro_imu) ||
        !ReadVector3(sample.linear_acceleration_mps2(), &accel_imu)) {
      // Fail closed rather than skip: a malformed reading inside the very
      // window the initial state is measured from means the window is not
      // trustworthy, and silently averaging the rest would hide that.
      return std::nullopt;
    }
    if (result.sample_count == 0) {
      first_time_s = time_s;
      last_time_s = time_s;
    } else {
      first_time_s = std::min(first_time_s, time_s);
      last_time_s = std::max(last_time_s, time_s);
    }
    gyro_sum += R_body_imu * gyro_imu;
    accel_sum += R_body_imu * accel_imu;
    ++result.sample_count;
  }

  auto fall_back = [&](const std::string& detail) {
    result.mode = ImuStationaryInitialization::Mode::kWideVelocityPrior;
    result.detail = detail;
    result.rotation_WB = Eigen::Quaterniond::Identity();
    result.velocity_W.setZero();
    result.bias_gyro.setZero();
    result.bias_accel.setZero();
    result.sigma.segment<3>(0).setConstant(params.wide_velocity_sigma_mps);
    result.sigma.segment<3>(3).setConstant(sigma_gyro_bias);
    result.sigma.segment<3>(6).setConstant(sigma_accel_bias);
    return result;
  };

  if (result.sample_count < params.min_samples) {
    return fall_back("only " + std::to_string(result.sample_count) +
                     " IMU sample(s) at or before the first keyframe boundary");
  }

  result.window_duration_s = last_time_s - first_time_s;
  const double scale = 1.0 / static_cast<double>(result.sample_count);
  const Eigen::Vector3d gyro_mean = gyro_sum * scale;
  const Eigen::Vector3d accel_mean = accel_sum * scale;
  result.gyro_mean_norm_radps = gyro_mean.norm();
  result.accel_mean_norm_mps2 = accel_mean.norm();

  if (result.window_duration_s < params.min_stationary_duration_s) {
    return fall_back("stationary window is " + std::to_string(result.window_duration_s) +
                     " s, below the required " +
                     std::to_string(params.min_stationary_duration_s) + " s");
  }
  if (result.gyro_mean_norm_radps >= params.max_gyro_mean_norm_radps) {
    return fall_back("mean angular rate " + std::to_string(result.gyro_mean_norm_radps) +
                     " rad/s is at or above the stationary threshold " +
                     std::to_string(params.max_gyro_mean_norm_radps));
  }
  const double gravity_deviation = std::abs(result.accel_mean_norm_mps2 - gravity_mps2);
  if (gravity_deviation >= params.max_accel_mean_norm_deviation_mps2) {
    return fall_back("mean specific force " + std::to_string(result.accel_mean_norm_mps2) +
                     " m/s^2 deviates from gravity by " + std::to_string(gravity_deviation) +
                     " m/s^2, at or above the stationary threshold " +
                     std::to_string(params.max_accel_mean_norm_deviation_mps2));
  }
  if (!PositiveFinite(result.accel_mean_norm_mps2)) {
    return fall_back("mean specific force has no usable direction");
  }

  // Direction to attitude, magnitude to bias (see the header's point 2).
  // FromTwoVectors gives the minimal rotation taking the measured up
  // direction onto world up; its axis is perpendicular to world up, so the
  // result carries no yaw.
  const Eigen::Vector3d measured_up = accel_mean / result.accel_mean_norm_mps2;
  result.rotation_WB =
      Eigen::Quaterniond::FromTwoVectors(measured_up, Eigen::Vector3d::UnitZ()).normalized();
  result.velocity_W.setZero();
  result.bias_gyro = gyro_mean;
  result.bias_accel = accel_mean - measured_up * gravity_mps2;
  result.mode = ImuStationaryInitialization::Mode::kStationary;

  // The prior is centred on values measured through white noise, so its
  // sigma has to absorb that measurement's standard error on top of the
  // rig's own bias spread. sigma_c is a continuous-time density, so the
  // standard error of a mean over a window of `window_duration_s` is
  // sigma_c / sqrt(window). Combining in quadrature can only widen the
  // prior, never tighten it below what the rig claims -- reporting more
  // confidence than either source supports is how a bias error later
  // surfaces as an unexplained drift.
  const double gyro_standard_error = noise->sigma_gyro_c / std::sqrt(result.window_duration_s);
  const double accel_standard_error = noise->sigma_accel_c / std::sqrt(result.window_duration_s);
  result.sigma.segment<3>(0).setConstant(params.stationary_velocity_sigma_mps);
  result.sigma.segment<3>(3).setConstant(
      std::sqrt(sigma_gyro_bias * sigma_gyro_bias + gyro_standard_error * gyro_standard_error));
  result.sigma.segment<3>(6).setConstant(
      std::sqrt(sigma_accel_bias * sigma_accel_bias + accel_standard_error * accel_standard_error));
  return result;
}

}  // namespace uw::frontends
