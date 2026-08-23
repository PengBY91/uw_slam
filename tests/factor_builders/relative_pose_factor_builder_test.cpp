#include "factor_builders/relative_pose_factor_builder.hpp"

#include <array>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "domain/domain.hpp"
#include "sensor_models/geometry.hpp"

using uw::factor_builders::RelativePoseFactorBuilder;
using uw::sensor_models::Pose3;

namespace {

uw::domain::MeasurementEvidence MakeEvidence(const Pose3& measured,
                                             const std::vector<double>& covariance_row_major) {
  uw::domain::RelativePoseMeasurement measurement;
  measurement.mutable_from_keyframe()->set_value("kf0");
  measurement.mutable_to_keyframe()->set_value("kf1");
  *measurement.mutable_relative_pose() = measured.ToProto();
  for (double v : covariance_row_major) measurement.add_covariance_6x6_row_major(v);

  uw::domain::EvidenceId id;
  id.set_value("relpose_test");
  return uw::domain::MakeEvidence(id, {}, measurement, /*noise_scale=*/1.0, "test_v1");
}

uw::domain::FactorCandidate MakeCandidate(double proposed_noise) {
  uw::domain::FactorCandidate candidate;
  candidate.set_residual_model(RelativePoseFactorBuilder::kResidualModel);
  candidate.set_proposed_noise(proposed_noise);
  return candidate;
}

std::vector<double> IdentityTimes(double scale) {
  std::vector<double> out(36, 0.0);
  for (int i = 0; i < 6; ++i) out[i * 6 + i] = scale;
  return out;
}

// Evaluates the built residual at pose_i=Identity, pose_j.translation=(dx,0,0)
// against a zero-translation/zero-rotation measurement, returning
// out[0] (whitened translation-x component) -- raw translation-x residual
// is exactly `dx` in this configuration.
double EvaluateTranslationXResidual(const std::unique_ptr<uw::measurement_api::ResidualBlock>& block,
                                    double dx) {
  Pose3 pose_i = Pose3::Identity();
  Pose3 pose_j = Pose3::Identity();
  pose_j.translation = Eigen::Vector3d(dx, 0.0, 0.0);
  const auto params_i = pose_i.ToParameterBlock();
  const auto params_j = pose_j.ToParameterBlock();
  std::vector<const double*> parameters{params_i.data(), params_j.data()};
  std::array<double, 6> out{};
  block->Evaluate(parameters, out.data(), nullptr);
  return out[0];
}

}  // namespace

TEST(RelativePoseFactorBuilder, RejectsWrongResidualModel) {
  RelativePoseFactorBuilder builder(20.0, 20.0);
  uw::domain::FactorCandidate candidate;
  candidate.set_residual_model("something_else_v1");
  const auto evidence = MakeEvidence(Pose3::Identity(), {});
  EXPECT_EQ(builder.Build(candidate, evidence, {}), nullptr);
}

TEST(RelativePoseFactorBuilder, FallsBackToDiagonalCapWhenCovarianceMissing) {
  RelativePoseFactorBuilder builder(/*translation_cap=*/20.0, /*rotation_cap=*/5.0);
  const auto candidate = MakeCandidate(20.0);
  const auto evidence = MakeEvidence(Pose3::Identity(), {});  // no covariance entries at all

  const auto block = builder.Build(candidate, evidence, {});
  ASSERT_NE(block, nullptr);
  EXPECT_NEAR(EvaluateTranslationXResidual(block, 0.05), 20.0 * 0.05, 1e-9);
}

TEST(RelativePoseFactorBuilder, FallsBackToDiagonalCapWhenCovarianceHasNaN) {
  RelativePoseFactorBuilder builder(20.0, 20.0);
  const auto candidate = MakeCandidate(20.0);
  auto covariance = IdentityTimes(0.01);
  covariance[7] = std::numeric_limits<double>::quiet_NaN();  // (1,1) entry
  const auto evidence = MakeEvidence(Pose3::Identity(), covariance);

  const auto block = builder.Build(candidate, evidence, {});
  ASSERT_NE(block, nullptr);
  EXPECT_NEAR(EvaluateTranslationXResidual(block, 0.05), 20.0 * 0.05, 1e-9);
}

TEST(RelativePoseFactorBuilder, FallsBackToDiagonalCapWhenCovarianceIsNotPositiveDefinite) {
  RelativePoseFactorBuilder builder(20.0, 20.0);
  const auto candidate = MakeCandidate(20.0);
  auto covariance = IdentityTimes(0.01);
  covariance[0] = -0.01;  // (0,0) entry negative: not PD
  const auto evidence = MakeEvidence(Pose3::Identity(), covariance);

  const auto block = builder.Build(candidate, evidence, {});
  ASSERT_NE(block, nullptr);
  EXPECT_NEAR(EvaluateTranslationXResidual(block, 0.05), 20.0 * 0.05, 1e-9);
}

TEST(RelativePoseFactorBuilder, UsesCovarianceDirectionWhenItsGainIsBelowTheCap) {
  RelativePoseFactorBuilder builder(/*translation_cap=*/20.0, /*rotation_cap=*/20.0);
  const auto candidate = MakeCandidate(20.0);
  // sigma=0.1 -> inverse sqrt = 10, well below the cap of 20 -> not clamped.
  const auto evidence = MakeEvidence(Pose3::Identity(), IdentityTimes(0.01));

  const auto block = builder.Build(candidate, evidence, {});
  ASSERT_NE(block, nullptr);
  EXPECT_NEAR(EvaluateTranslationXResidual(block, 0.02), 10.0 * 0.02, 1e-6);
}

TEST(RelativePoseFactorBuilder, ClampsOverconfidentCovarianceGainToTheCap) {
  RelativePoseFactorBuilder builder(/*translation_cap=*/20.0, /*rotation_cap=*/20.0);
  const auto candidate = MakeCandidate(20.0);
  // sigma=0.001 -> inverse sqrt = 1000, far above the cap of 20 -> clamped
  // down to exactly the isotropic cap.
  const auto evidence = MakeEvidence(Pose3::Identity(), IdentityTimes(1e-6));

  const auto block = builder.Build(candidate, evidence, {});
  ASSERT_NE(block, nullptr);
  EXPECT_NEAR(EvaluateTranslationXResidual(block, 0.02), 20.0 * 0.02, 1e-6);
}
