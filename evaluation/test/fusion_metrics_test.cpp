#include <gtest/gtest.h>

#include "uw/evaluation/fusion_metrics.hpp"

TEST(FusionMetrics, FlagsErrorAboveTheAbsoluteFloor) {
  // GT=1.0m: threshold = max(0.05, 0.03*1.0) = 0.05m.
  const auto ok = uw::evaluation::EvaluateFalseFusion(/*estimated_depth_m=*/1.03, /*gt_depth_m=*/1.0);
  EXPECT_FALSE(ok.is_false_fusion);
  EXPECT_NEAR(ok.threshold_m, 0.05, 1e-9);

  const auto bad = uw::evaluation::EvaluateFalseFusion(1.06, 1.0);
  EXPECT_TRUE(bad.is_false_fusion);
}

TEST(FusionMetrics, ThresholdScalesWithGtDepthAboveTheFloor) {
  // GT=10m: threshold = max(0.05, 0.3) = 0.3m.
  const auto ok = uw::evaluation::EvaluateFalseFusion(10.25, 10.0);
  EXPECT_FALSE(ok.is_false_fusion);
  EXPECT_NEAR(ok.threshold_m, 0.3, 1e-9);

  const auto bad = uw::evaluation::EvaluateFalseFusion(10.35, 10.0);
  EXPECT_TRUE(bad.is_false_fusion);
}
