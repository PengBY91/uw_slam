#include "factor_builders/imu_preintegration_factor_builder.hpp"

#include <random>
#include <vector>

#include <Eigen/Cholesky>
#include <gtest/gtest.h>

#include "domain/domain.hpp"
#include "factor_builders/imu_preintegration_residual.hpp"
#include "sensor_models/imu_preintegration.hpp"
#include "sensor_models/so3.hpp"

using uw::factor_builders::ImuPreintegrationFactorBuilder;
using uw::factor_builders::ImuPreintegrationResidual;
using uw::sensor_models::PreintegratedImuDelta;
namespace so3 = uw::sensor_models::so3;

namespace {

using Matrix15d = Eigen::Matrix<double, 15, 15>;

PreintegratedImuDelta MakeDelta() {
  PreintegratedImuDelta delta;
  delta.delta_time_s = 0.25;
  delta.sample_count = 50;
  delta.gravity_mps2 = 9.80665;
  delta.delta_rotation = so3::Exp(Eigen::Vector3d(0.01, 0.02, -0.03));
  delta.delta_velocity = Eigen::Vector3d(0.2, -0.1, 0.05);
  delta.delta_position = Eigen::Vector3d(0.025, -0.012, 0.006);
  delta.bias_gyro = Eigen::Vector3d(0.001, 0.0, -0.0005);
  delta.bias_accel = Eigen::Vector3d(0.01, -0.02, 0.005);
  delta.d_rotation_d_bias_gyro = -0.25 * Eigen::Matrix3d::Identity();
  delta.d_velocity_d_bias_gyro = 0.01 * Eigen::Matrix3d::Identity();
  delta.d_velocity_d_bias_accel = -0.25 * Eigen::Matrix3d::Identity();
  delta.d_position_d_bias_gyro = 0.002 * Eigen::Matrix3d::Identity();
  delta.d_position_d_bias_accel = -0.03 * Eigen::Matrix3d::Identity();

  Matrix15d root = Matrix15d::Zero();
  std::mt19937_64 rng(4242u);
  std::uniform_real_distribution<double> dist(-0.1, 0.1);
  for (int r = 0; r < 15; ++r) {
    for (int c = 0; c < r; ++c) root(r, c) = dist(rng);
    root(r, r) = 0.2 + 0.01 * r;
  }
  delta.covariance = root * root.transpose();
  return delta;
}

uw::domain::MeasurementEvidence MakeEvidence(const PreintegratedImuDelta& delta,
                                              double noise_scale) {
  uw::domain::EvidenceId id;
  id.set_value("imu_test");
  return uw::domain::MakeEvidence(id, {}, delta.ToProto(), noise_scale, "imu_preintegration_v1");
}

uw::domain::FactorCandidate MakeCandidate(double proposed_noise) {
  uw::domain::FactorCandidate candidate;
  candidate.set_residual_model(ImuPreintegrationFactorBuilder::kResidualModel);
  candidate.set_proposed_noise(proposed_noise);
  return candidate;
}

const ImuPreintegrationResidual* AsImuResidual(
    const std::unique_ptr<uw::measurement_api::ResidualBlock>& block) {
  return dynamic_cast<const ImuPreintegrationResidual*>(block.get());
}

}  // namespace

TEST(ImuPreintegrationFactorBuilder, OnlyClaimsItsOwnResidualModel) {
  ImuPreintegrationFactorBuilder builder;
  EXPECT_TRUE(builder.CanBuild(MakeCandidate(1.0)));
  uw::domain::FactorCandidate other;
  other.set_residual_model("depth_v1");
  EXPECT_FALSE(builder.CanBuild(other));
}

TEST(ImuPreintegrationFactorBuilder, BuildsWhiteningThatInvertsTheEvidenceCovariance) {
  const auto delta = MakeDelta();
  ImuPreintegrationFactorBuilder builder;
  auto block = builder.Build(MakeCandidate(0.0), MakeEvidence(delta, 1.0), {});
  ASSERT_NE(block, nullptr);
  const auto* residual = AsImuResidual(block);
  ASSERT_NE(residual, nullptr);

  const Matrix15d& s = residual->sqrt_information();
  const Matrix15d reconstructed_information = s.transpose() * s;
  const Matrix15d expected_information = delta.covariance.inverse();
  EXPECT_LT((reconstructed_information - expected_information).norm() /
                expected_information.norm(),
            1e-8);
  // Shape/dimensions the estimator binds against.
  EXPECT_EQ(residual->ResidualDim(), 15);
  EXPECT_EQ(residual->ParameterBlockSizes(), (std::vector<int>{7, 9, 7, 9}));
}

TEST(ImuPreintegrationFactorBuilder, RoundTripsTheDeltaThroughTheWire) {
  const auto delta = MakeDelta();
  ImuPreintegrationFactorBuilder builder;
  auto block = builder.Build(MakeCandidate(0.0), MakeEvidence(delta, 1.0), {});
  ASSERT_NE(block, nullptr);
  const auto& rebuilt = AsImuResidual(block)->delta();
  EXPECT_NEAR(rebuilt.delta_time_s, delta.delta_time_s, 1e-12);
  EXPECT_EQ(rebuilt.sample_count, delta.sample_count);
  EXPECT_LT((rebuilt.delta_velocity - delta.delta_velocity).norm(), 1e-12);
  EXPECT_LT((rebuilt.delta_position - delta.delta_position).norm(), 1e-12);
  EXPECT_LT((rebuilt.delta_rotation - delta.delta_rotation).norm(), 1e-9);
  EXPECT_LT((rebuilt.d_position_d_bias_accel - delta.d_position_d_bias_accel).norm(), 1e-12);
}

// Both knobs may only WEAKEN the factor -- the covariance the frontend
// propagated is the floor, never something a caller can sharpen.
TEST(ImuPreintegrationFactorBuilder, NoiseScaleAndProposedNoiseInflateTheCovariance) {
  const auto delta = MakeDelta();
  ImuPreintegrationFactorBuilder builder;
  auto baseline = builder.Build(MakeCandidate(0.0), MakeEvidence(delta, 1.0), {});
  auto scaled = builder.Build(MakeCandidate(0.0), MakeEvidence(delta, 2.0), {});
  auto proposed = builder.Build(MakeCandidate(2.0), MakeEvidence(delta, 1.0), {});
  ASSERT_NE(baseline, nullptr);
  ASSERT_NE(scaled, nullptr);
  ASSERT_NE(proposed, nullptr);

  const Matrix15d base = AsImuResidual(baseline)->sqrt_information();
  EXPECT_LT((AsImuResidual(scaled)->sqrt_information() - 0.5 * base).norm(), 1e-9);
  EXPECT_LT((AsImuResidual(proposed)->sqrt_information() - 0.5 * base).norm(), 1e-9);
}

TEST(ImuPreintegrationFactorBuilder, RejectsWrongPayload) {
  uw::domain::PressureDepthMeasurement depth;
  depth.set_depth_m(1.0);
  uw::domain::EvidenceId id;
  id.set_value("wrong");
  const auto evidence = uw::domain::MakeEvidence(id, {}, depth, 1.0, "depth_v1");
  ImuPreintegrationFactorBuilder builder;
  EXPECT_EQ(builder.Build(MakeCandidate(1.0), evidence, {}), nullptr);
}

TEST(ImuPreintegrationFactorBuilder, RejectsMalformedMessageInsteadOfDefaultingToZeroCovariance) {
  const auto delta = MakeDelta();
  auto message = delta.ToProto();
  message.clear_covariance_15x15_row_major();  // producer that filled no covariance
  uw::domain::EvidenceId id;
  id.set_value("malformed");
  const auto evidence = uw::domain::MakeEvidence(id, {}, message, 1.0, "imu_preintegration_v1");
  ImuPreintegrationFactorBuilder builder;
  EXPECT_EQ(builder.Build(MakeCandidate(1.0), evidence, {}), nullptr);
}

TEST(ImuPreintegrationFactorBuilder, RejectsSingularCovariance) {
  auto delta = MakeDelta();
  delta.covariance.row(9).setZero();  // no gyro-bias random walk at all
  delta.covariance.col(9).setZero();
  ImuPreintegrationFactorBuilder builder;
  EXPECT_EQ(builder.Build(MakeCandidate(1.0), MakeEvidence(delta, 1.0), {}), nullptr);
}
