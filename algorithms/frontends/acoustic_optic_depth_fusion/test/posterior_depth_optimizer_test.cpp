#include <cmath>

#include <gtest/gtest.h>

#include "uw/frontends/posterior_depth_optimizer.hpp"

namespace {

uw::sensor_models::PinholeCamera MakeCamera() {
  uw::sensor_models::PinholeCamera camera;
  camera.fx = 100.0;
  camera.fy = 100.0;
  camera.cx = 10.0;
  camera.cy = 5.0;
  camera.width = 20;
  camera.height = 10;
  return camera;
}

}  // namespace

TEST(PosteriorDepthOptimizer, PullsBoresightPriorTowardWeightedLeastSquaresOptimum) {
  // At boresight (pixel = image center, camera co-located/co-oriented with
  // the sonar), range(d) = d and bearing(d) = 0 for every d — this
  // collapses the cost to a closed-form weighted least squares between
  // the optical prior and the sonar range, letting the test assert an
  // exact expected answer instead of just "moved in the right direction".
  const auto camera = MakeCamera();
  const double prior_depth = 5.2, prior_variance = 0.09;       // sigma_d = 0.3
  const double sonar_range = 5.0, sonar_range_sigma = 0.05;    // sigma_rho = 0.05
  const double sonar_bearing = 0.0, sonar_bearing_sigma = 0.02;

  uw::frontends::PosteriorDepthOptimizerParams params;
  params.iterations = 60;
  params.search_radius_sigma = 3.0;
  const auto result = uw::frontends::OptimizePosteriorDepth(
      /*pixel_u=*/10.0, /*pixel_v=*/5.0, prior_depth, prior_variance, sonar_range,
      sonar_range_sigma, sonar_bearing, sonar_bearing_sigma,
      uw::sensor_models::Pose3::Identity(), camera, params);

  ASSERT_TRUE(result.valid);
  const double w_prior = 1.0 / prior_variance;
  const double w_sonar = 1.0 / (sonar_range_sigma * sonar_range_sigma);
  const double expected_depth = (prior_depth * w_prior + sonar_range * w_sonar) / (w_prior + w_sonar);
  EXPECT_NEAR(result.depth_m, expected_depth, 1e-4);
  const double expected_second_derivative = 2.0 * w_prior + 2.0 * w_sonar;
  EXPECT_NEAR(result.variance_m2, 2.0 / expected_second_derivative, 1e-4);
  EXPECT_LT(result.variance_m2, prior_variance);
  EXPECT_NEAR(result.bearing_residual_rad, 0.0, 1e-6);
}

TEST(PosteriorDepthOptimizer, InvalidWhenSonarSigmaIsNonPositive) {
  const auto camera = MakeCamera();
  uw::frontends::PosteriorDepthOptimizerParams params;
  const auto result = uw::frontends::OptimizePosteriorDepth(
      10.0, 5.0, /*prior_depth_m=*/5.0, /*prior_variance_m2=*/0.09, /*sonar_range_m=*/5.0,
      /*sonar_range_sigma_m=*/0.0,  // invalid: zero uncertainty is not a real measurement
      0.0, 0.02, uw::sensor_models::Pose3::Identity(), camera, params);
  EXPECT_FALSE(result.valid);
}

TEST(PosteriorDepthOptimizer, InvalidWhenPriorVarianceIsNonPositive) {
  const auto camera = MakeCamera();
  uw::frontends::PosteriorDepthOptimizerParams params;
  const auto result = uw::frontends::OptimizePosteriorDepth(
      10.0, 5.0, 5.0, /*prior_variance_m2=*/0.0, 5.0, 0.05, 0.0, 0.02,
      uw::sensor_models::Pose3::Identity(), camera, params);
  EXPECT_FALSE(result.valid);
}
