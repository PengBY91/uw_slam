#include "sensor_models/imu_preintegration.hpp"

#include <cmath>
#include <random>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "sensor_models/so3.hpp"

using uw::sensor_models::ImuPreintegrationNoise;
using uw::sensor_models::ImuPreintegrator;
using uw::sensor_models::PreintegratedImuDelta;
namespace so3 = uw::sensor_models::so3;

namespace {

constexpr double kGravity = 9.80665;
const Eigen::Vector3d kGravityWorld(0.0, 0.0, -kGravity);  // Z-up world

ImuPreintegrationNoise TestNoise() {
  ImuPreintegrationNoise noise;
  noise.sigma_gyro_c = 1.7e-4;
  noise.sigma_accel_c = 2.0e-3;
  noise.sigma_gyro_bias_walk = 1.9e-5;
  noise.sigma_accel_bias_walk = 3.0e-3;
  return noise;
}

// An analytic trajectory: constant body angular rate (so R(t) = R0 Exp(w t)
// exactly) and a smooth world-frame position p(t). The IMU reading at time
// t is the specific force R(t)^T (a_w(t) - g_w) and the body rate w.
struct AnalyticTrajectory {
  Eigen::Matrix3d R0 = so3::Exp(Eigen::Vector3d(0.2, -0.1, 0.4));
  Eigen::Vector3d body_rate = Eigen::Vector3d(0.3, -0.2, 0.5);

  Eigen::Matrix3d Rotation(double t) const { return R0 * so3::Exp(body_rate * t); }
  Eigen::Vector3d Position(double t) const {
    return Eigen::Vector3d(std::sin(t), 0.5 * std::cos(2.0 * t), 0.2 * t * t - 1.0);
  }
  Eigen::Vector3d Velocity(double t) const {
    return Eigen::Vector3d(std::cos(t), -std::sin(2.0 * t), 0.4 * t);
  }
  Eigen::Vector3d Acceleration(double t) const {
    return Eigen::Vector3d(-std::sin(t), -2.0 * std::cos(2.0 * t), 0.4);
  }
  Eigen::Vector3d ImuAccel(double t) const { return Rotation(t).transpose() * (Acceleration(t) - kGravityWorld); }

  // Ground-truth deltas per the definitions in imu_preintegration.hpp.
  Eigen::Matrix3d TrueDeltaRotation(double ti, double tj) const {
    return Rotation(ti).transpose() * Rotation(tj);
  }
  Eigen::Vector3d TrueDeltaVelocity(double ti, double tj) const {
    return Rotation(ti).transpose() * (Velocity(tj) - Velocity(ti) - kGravityWorld * (tj - ti));
  }
  Eigen::Vector3d TrueDeltaPosition(double ti, double tj) const {
    const double dt = tj - ti;
    return Rotation(ti).transpose() *
           (Position(tj) - Position(ti) - Velocity(ti) * dt - 0.5 * kGravityWorld * dt * dt);
  }
};

// Integrates `trajectory` from ti to tj at `rate_hz`, sampling each
// reading at the midpoint of its hold interval (so the zero-order hold is
// second-order accurate and the test tolerances can be tight), optionally
// adding a constant bias to the readings.
PreintegratedImuDelta IntegrateTrajectory(const AnalyticTrajectory& trajectory, double ti, double tj,
                                          double rate_hz, const Eigen::Vector3d& true_bias_gyro,
                                          const Eigen::Vector3d& true_bias_accel,
                                          const Eigen::Vector3d& assumed_bias_gyro,
                                          const Eigen::Vector3d& assumed_bias_accel) {
  ImuPreintegrator integrator(TestNoise(), kGravity);
  integrator.Reset(assumed_bias_gyro, assumed_bias_accel);
  const double dt = 1.0 / rate_hz;
  const int steps = static_cast<int>(std::llround((tj - ti) * rate_hz));
  for (int k = 0; k < steps; ++k) {
    const double t_mid = ti + (k + 0.5) * dt;
    EXPECT_TRUE(integrator.Integrate(trajectory.body_rate + true_bias_gyro,
                                     trajectory.ImuAccel(t_mid) + true_bias_accel, dt));
  }
  return integrator.delta();
}

TEST(ImuPreintegration, StaticLevelBodyAccumulatesGravityOnly) {
  ImuPreintegrator integrator(TestNoise(), kGravity);
  integrator.Reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  const double dt = 0.005;
  for (int k = 0; k < 200; ++k) {
    // Static, level: gyro 0, accelerometer reads +g up (specific force).
    ASSERT_TRUE(integrator.Integrate(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, kGravity), dt));
  }
  const auto& d = integrator.delta();
  EXPECT_NEAR(d.delta_time_s, 1.0, 1e-12);
  EXPECT_EQ(d.sample_count, 200u);
  EXPECT_LT((d.delta_rotation - Eigen::Matrix3d::Identity()).norm(), 1e-12);
  EXPECT_LT((d.delta_velocity - Eigen::Vector3d(0.0, 0.0, kGravity * 1.0)).norm(), 1e-9);
  EXPECT_LT((d.delta_position - Eigen::Vector3d(0.0, 0.0, 0.5 * kGravity * 1.0)).norm(), 1e-9);
}

TEST(ImuPreintegration, ConstantYawRateMatchesClosedForm) {
  ImuPreintegrator integrator(TestNoise(), kGravity);
  integrator.Reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  const double dt = 0.005;
  const Eigen::Vector3d rate(0.0, 0.0, 0.8);
  for (int k = 0; k < 200; ++k) {
    ASSERT_TRUE(integrator.Integrate(rate, Eigen::Vector3d(0.0, 0.0, kGravity), dt));
  }
  const auto& d = integrator.delta();
  EXPECT_LT((d.delta_rotation - so3::Exp(rate * 1.0)).norm(), 1e-10);
  // Rotation about z leaves the z-axis specific force untouched.
  EXPECT_LT((d.delta_velocity - Eigen::Vector3d(0.0, 0.0, kGravity)).norm(), 1e-9);
  EXPECT_LT((d.delta_position - Eigen::Vector3d(0.0, 0.0, 0.5 * kGravity)).norm(), 1e-9);
}

TEST(ImuPreintegration, DeltasMatchAnalyticTrajectoryDefinitions) {
  AnalyticTrajectory trajectory;
  const double ti = 0.3;
  const double tj = 1.5;
  const auto d = IntegrateTrajectory(trajectory, ti, tj, 400.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                     Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  EXPECT_NEAR(d.delta_time_s, tj - ti, 1e-9);
  EXPECT_LT(so3::Log(trajectory.TrueDeltaRotation(ti, tj).transpose() * d.delta_rotation).norm(), 1e-9);
  EXPECT_LT((d.delta_velocity - trajectory.TrueDeltaVelocity(ti, tj)).norm(), 2e-4)
      << "dv=" << d.delta_velocity.transpose() << " true=" << trajectory.TrueDeltaVelocity(ti, tj).transpose();
  EXPECT_LT((d.delta_position - trajectory.TrueDeltaPosition(ti, tj)).norm(), 2e-4)
      << "dp=" << d.delta_position.transpose() << " true=" << trajectory.TrueDeltaPosition(ti, tj).transpose();
}

TEST(ImuPreintegration, BiasJacobiansPredictReintegrationToFirstOrder) {
  AnalyticTrajectory trajectory;
  const double ti = 0.0;
  const double tj = 1.0;
  const Eigen::Vector3d bg0(0.01, -0.02, 0.015);
  const Eigen::Vector3d ba0(0.05, 0.02, -0.03);
  const auto base = IntegrateTrajectory(trajectory, ti, tj, 200.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                        bg0, ba0);
  const Eigen::Vector3d dbg(2e-3, -1e-3, 1.5e-3);
  const Eigen::Vector3d dba(1e-2, -2e-2, 1.5e-2);
  const auto shifted = IntegrateTrajectory(trajectory, ti, tj, 200.0, Eigen::Vector3d::Zero(),
                                           Eigen::Vector3d::Zero(), bg0 + dbg, ba0 + dba);

  // The change predicted by the Jacobians must match the re-integrated
  // change to well within the change itself (second-order remainder).
  const Eigen::Vector3d rot_change = so3::Log(base.delta_rotation.transpose() * shifted.delta_rotation);
  const Eigen::Vector3d rot_pred = so3::Log(base.delta_rotation.transpose() * base.CorrectedRotation(dbg));
  EXPECT_GT(rot_change.norm(), 1e-4);
  EXPECT_LT((rot_change - rot_pred).norm(), 0.02 * rot_change.norm());

  const Eigen::Vector3d v_change = shifted.delta_velocity - base.delta_velocity;
  const Eigen::Vector3d v_pred = base.CorrectedVelocity(dbg, dba) - base.delta_velocity;
  EXPECT_GT(v_change.norm(), 1e-3);
  EXPECT_LT((v_change - v_pred).norm(), 0.02 * v_change.norm());

  const Eigen::Vector3d p_change = shifted.delta_position - base.delta_position;
  const Eigen::Vector3d p_pred = base.CorrectedPosition(dbg, dba) - base.delta_position;
  EXPECT_GT(p_change.norm(), 1e-3);
  EXPECT_LT((p_change - p_pred).norm(), 0.02 * p_change.norm());
}

TEST(ImuPreintegration, CovarianceMatchesMonteCarloSpread) {
  // Nominal deltas from clean readings, then the sample covariance of the
  // 9-dim error [Log(dR^T dR_noisy), dv_noisy - dv, dp_noisy - dp] over
  // many noisy runs must agree with the propagated covariance. Noise is
  // scaled up so the spread dominates integration round-off.
  ImuPreintegrationNoise noise = TestNoise();
  noise.sigma_gyro_c = 0.02;
  noise.sigma_accel_c = 0.2;
  const double dt = 0.01;
  const int steps = 100;
  const Eigen::Vector3d rate(0.2, -0.1, 0.3);
  const Eigen::Vector3d accel(0.5, -0.3, kGravity + 0.2);

  ImuPreintegrator nominal(noise, kGravity);
  nominal.Reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  for (int k = 0; k < steps; ++k) ASSERT_TRUE(nominal.Integrate(rate, accel, dt));
  const auto& d0 = nominal.delta();

  std::mt19937_64 rng(42);
  std::normal_distribution<double> gauss(0.0, 1.0);
  const double sigma_g = noise.sigma_gyro_c / std::sqrt(dt);
  const double sigma_a = noise.sigma_accel_c / std::sqrt(dt);
  const int runs = 2000;
  Eigen::Matrix<double, 9, 9> sample_cov = Eigen::Matrix<double, 9, 9>::Zero();
  for (int r = 0; r < runs; ++r) {
    ImuPreintegrator noisy(noise, kGravity);
    noisy.Reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    for (int k = 0; k < steps; ++k) {
      const Eigen::Vector3d ng(gauss(rng), gauss(rng), gauss(rng));
      const Eigen::Vector3d na(gauss(rng), gauss(rng), gauss(rng));
      ASSERT_TRUE(noisy.Integrate(rate + sigma_g * ng, accel + sigma_a * na, dt));
    }
    const auto& d = noisy.delta();
    Eigen::Matrix<double, 9, 1> err;
    err.segment<3>(0) = so3::Log(d0.delta_rotation.transpose() * d.delta_rotation);
    err.segment<3>(3) = d.delta_velocity - d0.delta_velocity;
    err.segment<3>(6) = d.delta_position - d0.delta_position;
    sample_cov += err * err.transpose();
  }
  sample_cov /= static_cast<double>(runs - 1);

  const Eigen::Matrix<double, 9, 9> predicted = d0.covariance.block<9, 9>(0, 0);
  for (int i = 0; i < 9; ++i) {
    EXPECT_GT(predicted(i, i), 0.0);
    EXPECT_NEAR(sample_cov(i, i) / predicted(i, i), 1.0, 0.15) << "diag " << i;
  }
  // Off-diagonal velocity/position coupling has a definite sign and size.
  for (int i = 3; i < 6; ++i) {
    EXPECT_NEAR(sample_cov(i, i + 3) / predicted(i, i + 3), 1.0, 0.2) << "dv/dp coupling " << i;
  }
  // Bias random-walk block: sigma^2 * total time on the diagonal, zero cross terms.
  const double total = steps * dt;
  EXPECT_NEAR(d0.covariance(9, 9), noise.sigma_gyro_bias_walk * noise.sigma_gyro_bias_walk * total, 1e-15);
  EXPECT_NEAR(d0.covariance(12, 12), noise.sigma_accel_bias_walk * noise.sigma_accel_bias_walk * total, 1e-15);
  EXPECT_EQ((d0.covariance.block<9, 6>(0, 9)).norm(), 0.0);
}

TEST(ImuPreintegration, ProtoRoundTripPreservesEveryField) {
  AnalyticTrajectory trajectory;
  const auto d = IntegrateTrajectory(trajectory, 0.0, 0.5, 200.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                     Eigen::Vector3d(0.01, 0.0, -0.01), Eigen::Vector3d(0.1, 0.0, 0.0));
  const auto message = d.ToProto();
  ASSERT_EQ(message.covariance_15x15_row_major_size(), 225);
  ASSERT_EQ(message.d_position_d_bias_gyro_size(), 9);
  std::string error;
  const auto back = PreintegratedImuDelta::FromProto(message, &error);
  ASSERT_TRUE(back.has_value()) << error;
  EXPECT_LT((back->delta_rotation - d.delta_rotation).norm(), 1e-12);
  EXPECT_LT((back->delta_velocity - d.delta_velocity).norm(), 1e-12);
  EXPECT_LT((back->delta_position - d.delta_position).norm(), 1e-12);
  EXPECT_LT((back->covariance - d.covariance).norm(), 1e-15);
  EXPECT_LT((back->d_rotation_d_bias_gyro - d.d_rotation_d_bias_gyro).norm(), 1e-15);
  EXPECT_LT((back->d_velocity_d_bias_gyro - d.d_velocity_d_bias_gyro).norm(), 1e-15);
  EXPECT_LT((back->d_velocity_d_bias_accel - d.d_velocity_d_bias_accel).norm(), 1e-15);
  EXPECT_LT((back->d_position_d_bias_gyro - d.d_position_d_bias_gyro).norm(), 1e-15);
  EXPECT_LT((back->d_position_d_bias_accel - d.d_position_d_bias_accel).norm(), 1e-15);
  EXPECT_EQ(back->bias_gyro, d.bias_gyro);
  EXPECT_EQ(back->bias_accel, d.bias_accel);
  EXPECT_EQ(back->sample_count, d.sample_count);
  EXPECT_DOUBLE_EQ(back->delta_time_s, d.delta_time_s);
  EXPECT_DOUBLE_EQ(back->gravity_mps2, kGravity);
}

TEST(ImuPreintegration, FromProtoRejectsIllSizedMessages) {
  uw::domain::ImuPreintegrationMeasurement message;
  std::string error;
  EXPECT_FALSE(PreintegratedImuDelta::FromProto(message, &error).has_value());
  EXPECT_NE(error.find("delta_pose"), std::string::npos);

  AnalyticTrajectory trajectory;
  auto good = IntegrateTrajectory(trajectory, 0.0, 0.5, 200.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                  Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero())
                  .ToProto();
  good.mutable_covariance_15x15_row_major()->RemoveLast();
  EXPECT_FALSE(PreintegratedImuDelta::FromProto(good, &error).has_value());
  EXPECT_NE(error.find("covariance"), std::string::npos);
}

TEST(ImuPreintegration, IntegrateRejectsNonFiniteInputAndNonPositiveDt) {
  ImuPreintegrator integrator(TestNoise(), kGravity);
  integrator.Reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  EXPECT_FALSE(integrator.Integrate(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.0));
  EXPECT_FALSE(integrator.Integrate(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), -0.01));
  EXPECT_FALSE(integrator.Integrate(Eigen::Vector3d(std::nan(""), 0.0, 0.0), Eigen::Vector3d::Zero(), 0.01));
  EXPECT_EQ(integrator.delta().sample_count, 0u);
  EXPECT_DOUBLE_EQ(integrator.delta().delta_time_s, 0.0);
}

TEST(ImuPreintegration, NoiseFromRigFallsBackToBiasSigmaAndRejectsZeroDensity) {
  uw::domain::RigCalibrationSnapshot rig;
  auto* model = rig.mutable_imu_noise();
  model->set_sigma_gyro_c(1.7e-4);
  model->set_sigma_accel_c(2.0e-3);
  model->set_sigma_gyro_bias(1.9e-5);
  model->set_sigma_accel_bias(3.0e-3);
  std::string error;
  auto noise = ImuPreintegrationNoise::FromRig(rig, &error);
  ASSERT_TRUE(noise.has_value()) << error;
  EXPECT_DOUBLE_EQ(noise->sigma_gyro_bias_walk, 1.9e-5);
  EXPECT_DOUBLE_EQ(noise->sigma_accel_bias_walk, 3.0e-3);

  model->set_sigma_gyro_bias_walk_c(5e-6);
  noise = ImuPreintegrationNoise::FromRig(rig, &error);
  ASSERT_TRUE(noise.has_value());
  EXPECT_DOUBLE_EQ(noise->sigma_gyro_bias_walk, 5e-6);

  model->set_sigma_gyro_c(0.0);  // what a rig that omits the key parses to
  EXPECT_FALSE(ImuPreintegrationNoise::FromRig(rig, &error).has_value());
  EXPECT_NE(error.find("sigma_gyro_c"), std::string::npos);
}

}  // namespace
