#include "factor_builders/sonar_range_factor_builder.hpp"

#include <array>
#include <limits>

#include <gtest/gtest.h>

#include "domain/domain.hpp"
#include "measurement_api/factor_builder.hpp"
#include "sensor_models/geometry.hpp"

using uw::factor_builders::SonarRangeFactorBuilder;

namespace {

uw::domain::MeasurementEvidence MakeEvidence(double range_m, double range_sigma_m) {
  uw::domain::SonarRangeBearing measurement;
  measurement.set_range_m(range_m);
  measurement.set_bearing_rad(0.0);
  measurement.set_range_sigma_m(range_sigma_m);
  uw::domain::EvidenceId id;
  id.set_value("sonar_test");
  return uw::domain::MakeEvidence(id, {}, measurement, /*noise_scale=*/1.0, "test_v1");
}

uw::domain::FactorCandidate MakeCandidate(double proposed_noise) {
  uw::domain::FactorCandidate candidate;
  candidate.set_residual_model(SonarRangeFactorBuilder::kResidualModel);
  candidate.set_proposed_noise(proposed_noise);
  return candidate;
}

// Landmark at (1,0,0), NOT the origin: SonarRangeResidual treats
// ||pose.translation - landmark|| < 1e-9 as degenerate and refuses to
// evaluate (returns false, leaving residuals untouched) -- see
// sonar_range_residual.cpp.
uw::measurement_api::FactorBuildContext MakeContext() {
  uw::measurement_api::FactorBuildContext context;
  context.nearby_points_W = {Eigen::Vector3d(1.0, 0.0, 0.0)};
  return context;
}

// Evaluates the built residual at pose=Identity against MakeContext()'s
// landmark at (1,0,0): SonarRangeResidual's raw residual = measured_range_m
// - ||pose.translation - landmark|| = range_m - 1.0. Callers pass
// range_m=2.0 so the raw residual is exactly 1.0, isolating the built
// sqrt_information as the output.
double EvaluateWeight(const std::unique_ptr<uw::measurement_api::ResidualBlock>& block) {
  uw::sensor_models::Pose3 pose = uw::sensor_models::Pose3::Identity();
  const auto params = pose.ToParameterBlock();
  std::vector<const double*> parameters{params.data()};
  std::array<double, 1> out{};
  const bool ok = block->Evaluate(parameters, out.data(), nullptr);
  EXPECT_TRUE(ok);
  return out[0];
}

}  // namespace

TEST(SonarRangeFactorBuilder, RejectsWrongResidualModel) {
  SonarRangeFactorBuilder builder;
  uw::domain::FactorCandidate candidate;
  candidate.set_residual_model("something_else_v1");
  EXPECT_EQ(builder.Build(candidate, MakeEvidence(2.0, 0.2), MakeContext()), nullptr);
}

TEST(SonarRangeFactorBuilder, WeighsByInverseRangeSigmaWhenBelowTheCap) {
  SonarRangeFactorBuilder builder;
  // range_sigma_m=0.2 -> 1/sigma=5, cap=10.
  const auto block = builder.Build(MakeCandidate(10.0), MakeEvidence(2.0, 0.2), MakeContext());
  ASSERT_NE(block, nullptr);
  EXPECT_NEAR(EvaluateWeight(block), 5.0, 1e-9);
}

TEST(SonarRangeFactorBuilder, ClampsToTheCapWhenRangeSigmaImpliesMoreInformation) {
  SonarRangeFactorBuilder builder;
  // range_sigma_m=0.01 -> 1/sigma=100, far above cap=10 -> clamped to 10.
  const auto block = builder.Build(MakeCandidate(10.0), MakeEvidence(2.0, 0.01), MakeContext());
  ASSERT_NE(block, nullptr);
  EXPECT_NEAR(EvaluateWeight(block), 10.0, 1e-9);
}

TEST(SonarRangeFactorBuilder, FallsBackToCapWhenRangeSigmaIsZero) {
  SonarRangeFactorBuilder builder;
  const auto block = builder.Build(MakeCandidate(10.0), MakeEvidence(2.0, 0.0), MakeContext());
  ASSERT_NE(block, nullptr);
  EXPECT_NEAR(EvaluateWeight(block), 10.0, 1e-9);
}

TEST(SonarRangeFactorBuilder, FallsBackToCapWhenRangeSigmaIsNegative) {
  SonarRangeFactorBuilder builder;
  const auto block = builder.Build(MakeCandidate(10.0), MakeEvidence(2.0, -0.3), MakeContext());
  ASSERT_NE(block, nullptr);
  EXPECT_NEAR(EvaluateWeight(block), 10.0, 1e-9);
}

TEST(SonarRangeFactorBuilder, FallsBackToCapWhenRangeSigmaIsNaN) {
  SonarRangeFactorBuilder builder;
  const auto block = builder.Build(MakeCandidate(10.0),
                                   MakeEvidence(2.0, std::numeric_limits<double>::quiet_NaN()), MakeContext());
  ASSERT_NE(block, nullptr);
  EXPECT_NEAR(EvaluateWeight(block), 10.0, 1e-9);
}

TEST(SonarRangeFactorBuilder, FallsBackToOneWhenCandidateCapIsNonPositive) {
  SonarRangeFactorBuilder builder;
  const auto block = builder.Build(MakeCandidate(-1.0), MakeEvidence(2.0, 0.2), MakeContext());
  ASSERT_NE(block, nullptr);
  EXPECT_NEAR(EvaluateWeight(block), 1.0, 1e-9);
}

TEST(SonarRangeFactorBuilder, RejectsEmptyNearbyPoints) {
  SonarRangeFactorBuilder builder;
  uw::measurement_api::FactorBuildContext empty_context;
  EXPECT_EQ(builder.Build(MakeCandidate(10.0), MakeEvidence(2.0, 0.2), empty_context), nullptr);
}
