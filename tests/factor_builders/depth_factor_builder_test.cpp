#include "factor_builders/depth_factor_builder.hpp"

#include <array>
#include <limits>

#include <gtest/gtest.h>

#include "domain/domain.hpp"
#include "sensor_models/geometry.hpp"

using uw::factor_builders::DepthFactorBuilder;

namespace {

uw::domain::MeasurementEvidence MakeEvidence(double depth_m, double sigma_m) {
  uw::domain::PressureDepthMeasurement measurement;
  measurement.set_depth_m(depth_m);
  measurement.set_sigma_m(sigma_m);
  uw::domain::EvidenceId id;
  id.set_value("depth_test");
  return uw::domain::MakeEvidence(id, {}, measurement, /*noise_scale=*/1.0, "test_v1");
}

uw::domain::FactorCandidate MakeCandidate(double proposed_noise) {
  uw::domain::FactorCandidate candidate;
  candidate.set_residual_model(DepthFactorBuilder::kResidualModel);
  candidate.set_proposed_noise(proposed_noise);
  return candidate;
}

// Evaluates the built residual at translation.z()=0 against a zero-depth
// measurement: DepthResidual's residual = measured_depth_m +
// translation.z(), so this returns exactly the built sqrt_information
// (weight * 1.0 raw residual).
double EvaluateWeight(const std::unique_ptr<uw::measurement_api::ResidualBlock>& block) {
  uw::sensor_models::Pose3 pose = uw::sensor_models::Pose3::Identity();
  const auto params = pose.ToParameterBlock();
  std::vector<const double*> parameters{params.data()};
  std::array<double, 1> out{};
  block->Evaluate(parameters, out.data(), nullptr);
  return out[0];
}

}  // namespace

TEST(DepthFactorBuilder, RejectsWrongResidualModel) {
  DepthFactorBuilder builder;
  uw::domain::FactorCandidate candidate;
  candidate.set_residual_model("something_else_v1");
  EXPECT_EQ(builder.Build(candidate, MakeEvidence(1.0, 0.2), {}), nullptr);
}

TEST(DepthFactorBuilder, WeighsByInverseSigmaWhenBelowTheCap) {
  DepthFactorBuilder builder;
  // measured_depth_m=1.0 (raw residual at z=0 is 1.0), sigma=0.2 -> 1/sigma=5, cap=10.
  const auto block = builder.Build(MakeCandidate(10.0), MakeEvidence(1.0, 0.2), {});
  ASSERT_NE(block, nullptr);
  EXPECT_NEAR(EvaluateWeight(block), 5.0, 1e-9);
}

TEST(DepthFactorBuilder, ClampsToTheCapWhenSigmaImpliesMoreInformation) {
  DepthFactorBuilder builder;
  // sigma=0.01 -> 1/sigma=100, far above cap=10 -> clamped to 10.
  const auto block = builder.Build(MakeCandidate(10.0), MakeEvidence(1.0, 0.01), {});
  ASSERT_NE(block, nullptr);
  EXPECT_NEAR(EvaluateWeight(block), 10.0, 1e-9);
}

TEST(DepthFactorBuilder, FallsBackToCapWhenSigmaIsZero) {
  DepthFactorBuilder builder;
  const auto block = builder.Build(MakeCandidate(10.0), MakeEvidence(1.0, 0.0), {});
  ASSERT_NE(block, nullptr);
  EXPECT_NEAR(EvaluateWeight(block), 10.0, 1e-9);
}

TEST(DepthFactorBuilder, FallsBackToCapWhenSigmaIsNegative) {
  DepthFactorBuilder builder;
  const auto block = builder.Build(MakeCandidate(10.0), MakeEvidence(1.0, -0.5), {});
  ASSERT_NE(block, nullptr);
  EXPECT_NEAR(EvaluateWeight(block), 10.0, 1e-9);
}

TEST(DepthFactorBuilder, FallsBackToCapWhenSigmaIsNaN) {
  DepthFactorBuilder builder;
  const auto block =
      builder.Build(MakeCandidate(10.0), MakeEvidence(1.0, std::numeric_limits<double>::quiet_NaN()), {});
  ASSERT_NE(block, nullptr);
  EXPECT_NEAR(EvaluateWeight(block), 10.0, 1e-9);
}

TEST(DepthFactorBuilder, FallsBackToOneWhenCandidateCapIsNonPositive) {
  DepthFactorBuilder builder;
  // sigma=0.2 -> 1/sigma=5, but candidate cap is non-positive -> cap
  // itself falls back to 1.0, and min(1.0, 5.0) = 1.0.
  const auto block = builder.Build(MakeCandidate(0.0), MakeEvidence(1.0, 0.2), {});
  ASSERT_NE(block, nullptr);
  EXPECT_NEAR(EvaluateWeight(block), 1.0, 1e-9);
}
