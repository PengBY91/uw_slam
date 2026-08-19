#include "uw/factor_builders/sonar_range_residual.hpp"

#include <array>
#include <cmath>

#include <gtest/gtest.h>

using uw::factor_builders::SonarRangeResidual;

namespace {

std::array<double, 7> MakePoseParams(double x, double y, double z) {
  return {x, y, z, 0.0, 0.0, 0.0, 1.0};  // identity rotation
}

double Evaluate(const SonarRangeResidual& residual, const std::array<double, 7>& pose) {
  const double* params = pose.data();
  std::vector<const double*> parameters{params};
  double out = 0.0;
  residual.Evaluate(parameters, &out, nullptr);
  return out;
}

}  // namespace

TEST(SonarRangeResidual, ResidualMatchesRangeMinusDistanceToLandmarkMean) {
  const std::vector<Eigen::Vector3d> landmarks{{5.0, 0.0, 0.0}, {5.0, 0.0, 0.2}};
  // mean = (5, 0, 0.1)
  SonarRangeResidual residual(/*range_m=*/3.0, /*bearing_rad=*/0.0, /*sqrt_information=*/2.0,
                               landmarks);
  const auto pose = MakePoseParams(0.0, 0.0, 0.0);
  const double expected_distance = (Eigen::Vector3d(0, 0, 0) - Eigen::Vector3d(5.0, 0.0, 0.1)).norm();
  EXPECT_NEAR(Evaluate(residual, pose), 2.0 * (3.0 - expected_distance), 1e-9);
}

TEST(SonarRangeResidual, AnalyticJacobianMatchesFiniteDifference) {
  const std::vector<Eigen::Vector3d> landmarks{{4.0, 1.0, -0.3}, {4.5, 0.8, 0.1}, {3.8, 1.2, 0.0}};
  SonarRangeResidual residual(/*range_m=*/6.0, /*bearing_rad=*/0.2, /*sqrt_information=*/1.7,
                               landmarks);

  auto pose = MakePoseParams(0.5, -0.2, 0.1);
  std::vector<const double*> parameters{pose.data()};

  double analytic_residual = 0.0;
  std::array<double, 7> jacobian{};
  double* jac_ptr = jacobian.data();
  std::vector<double*> jacobians{jac_ptr};
  ASSERT_TRUE(residual.Evaluate(parameters, &analytic_residual, &jacobians));

  constexpr double kEps = 1e-6;
  for (int i = 0; i < 3; ++i) {
    auto plus = pose;
    auto minus = pose;
    plus[i] += kEps;
    minus[i] -= kEps;
    const double r_plus = Evaluate(residual, plus);
    const double r_minus = Evaluate(residual, minus);
    const double numeric = (r_plus - r_minus) / (2.0 * kEps);
    EXPECT_NEAR(jacobian[i], numeric, 1e-5) << "translation index " << i;
  }
  // Orientation columns must be exactly zero (residual has no dependence).
  for (int i = 3; i < 7; ++i) {
    EXPECT_EQ(jacobian[i], 0.0) << "orientation index " << i;
  }
}

TEST(SonarRangeResidual, EmptyLandmarkSubsetFailsCleanly) {
  SonarRangeResidual residual(3.0, 0.0, 1.0, {});
  const auto pose = MakePoseParams(0.0, 0.0, 0.0);
  const double* params = pose.data();
  std::vector<const double*> parameters{params};
  double out = 0.0;
  EXPECT_FALSE(residual.Evaluate(parameters, &out, nullptr));
}
