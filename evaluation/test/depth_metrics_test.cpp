#include <gtest/gtest.h>

#include "uw/evaluation/depth_metrics.hpp"

TEST(DepthMetrics, ComputesRmseMaeAndCoverageOverSharedValidPixels) {
  uw::evaluation::DepthGrid estimated;
  estimated.width = 2;
  estimated.height = 1;
  estimated.depth_m = {5.0f, 6.0f};
  estimated.valid_mask = {1, 1};

  uw::evaluation::DepthGrid ground_truth;
  ground_truth.width = 2;
  ground_truth.height = 1;
  ground_truth.depth_m = {5.5f, 6.0f};
  ground_truth.valid_mask = {1, 0};  // second pixel has no GT -> excluded

  const auto result = uw::evaluation::ComputeDepthMetrics(estimated, ground_truth);
  EXPECT_EQ(result.num_compared_pixels, 1u);
  EXPECT_NEAR(result.mae_m, 0.5, 1e-9);
  EXPECT_NEAR(result.rmse_m, 0.5, 1e-9);
  EXPECT_NEAR(result.valid_coverage_fraction, 1.0, 1e-9);  // 1 of 1 GT-valid pixel also estimated-valid
}

TEST(DepthMetrics, ZeroGtValidPixelsYieldsZeroCoverageNotNan) {
  uw::evaluation::DepthGrid estimated;
  estimated.width = 1;
  estimated.height = 1;
  estimated.depth_m = {1.0f};
  estimated.valid_mask = {0};

  uw::evaluation::DepthGrid ground_truth;
  ground_truth.width = 1;
  ground_truth.height = 1;
  ground_truth.depth_m = {1.0f};
  ground_truth.valid_mask = {0};

  const auto result = uw::evaluation::ComputeDepthMetrics(estimated, ground_truth);
  EXPECT_EQ(result.num_compared_pixels, 0u);
  EXPECT_DOUBLE_EQ(result.valid_coverage_fraction, 0.0);
  EXPECT_DOUBLE_EQ(result.rmse_m, 0.0);
}
