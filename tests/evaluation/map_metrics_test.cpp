#include "evaluation/map_metrics.hpp"

#include <gtest/gtest.h>

TEST(MapMetrics, PerfectOverlapGivesIdealValuesForEveryMetric) {
  const std::vector<Eigen::Vector3d> points = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}};
  const auto result = uw::evaluation::ComputeMapMetrics(points, points, /*distance_threshold_m=*/0.1);
  EXPECT_DOUBLE_EQ(result.chamfer_distance_m, 0.0);
  EXPECT_DOUBLE_EQ(result.mean_estimated_to_reference_m, 0.0);
  EXPECT_DOUBLE_EQ(result.mean_reference_to_estimated_m, 0.0);
  EXPECT_DOUBLE_EQ(result.completeness, 1.0);
  EXPECT_DOUBLE_EQ(result.outlier_ratio, 0.0);
  EXPECT_DOUBLE_EQ(result.f_score, 1.0);  // precision=1, recall=1
  EXPECT_EQ(result.num_estimated_points, 3u);
  EXPECT_EQ(result.num_reference_points, 3u);
}

TEST(MapMetrics, HandComputedAsymmetricCase) {
  // estimated has exactly one point, sitting exactly on the first
  // reference point; the second reference point is 10m away with nothing
  // nearby to explain it.
  const std::vector<Eigen::Vector3d> estimated = {{0.0, 0.0, 0.0}};
  const std::vector<Eigen::Vector3d> reference = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
  const auto result = uw::evaluation::ComputeMapMetrics(estimated, reference, /*distance_threshold_m=*/1.0);

  // estimated -> reference: the one estimated point's nearest reference is
  // itself (distance 0), and it's within the threshold, so no outliers.
  EXPECT_NEAR(result.mean_estimated_to_reference_m, 0.0, 1e-9);
  EXPECT_DOUBLE_EQ(result.outlier_ratio, 0.0);

  // reference -> estimated: distances are 0 and 10, mean 5; only the first
  // reference point (distance 0) is within the 1m threshold -> 1/2 covered.
  EXPECT_NEAR(result.mean_reference_to_estimated_m, 5.0, 1e-9);
  EXPECT_DOUBLE_EQ(result.completeness, 0.5);

  EXPECT_NEAR(result.chamfer_distance_m, 5.0, 1e-9);  // 0.0 + 5.0
  // precision=1 (0 outliers), recall=0.5 -> F = 2*1*0.5/(1+0.5) = 2/3
  EXPECT_NEAR(result.f_score, 2.0 / 3.0, 1e-9);
}

TEST(MapMetrics, NoOverlapMarksEverythingAsOutlierAndIncomplete) {
  const std::vector<Eigen::Vector3d> estimated = {{0.0, 0.0, 0.0}};
  const std::vector<Eigen::Vector3d> reference = {{100.0, 0.0, 0.0}};
  const auto result = uw::evaluation::ComputeMapMetrics(estimated, reference, /*distance_threshold_m=*/0.5);
  EXPECT_DOUBLE_EQ(result.outlier_ratio, 1.0);
  EXPECT_DOUBLE_EQ(result.completeness, 0.0);
  EXPECT_NEAR(result.chamfer_distance_m, 200.0, 1e-9);  // 100 + 100
  EXPECT_DOUBLE_EQ(result.f_score, 0.0);  // precision=0, recall=0
}

TEST(MapMetrics, EmptyEstimatedYieldsZeroNotNan) {
  const std::vector<Eigen::Vector3d> estimated;
  const std::vector<Eigen::Vector3d> reference = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
  const auto result = uw::evaluation::ComputeMapMetrics(estimated, reference, /*distance_threshold_m=*/0.1);
  EXPECT_DOUBLE_EQ(result.mean_estimated_to_reference_m, 0.0);
  EXPECT_DOUBLE_EQ(result.outlier_ratio, 0.0);  // nothing to be an outlier
  EXPECT_DOUBLE_EQ(result.completeness, 0.0);   // nothing covers the reference
  EXPECT_DOUBLE_EQ(result.chamfer_distance_m, 0.0);
  EXPECT_EQ(result.num_estimated_points, 0u);
}

TEST(MapMetrics, EmptyReferenceMarksAllEstimatedAsOutliersNotZero) {
  const std::vector<Eigen::Vector3d> estimated = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
  const std::vector<Eigen::Vector3d> reference;
  const auto result = uw::evaluation::ComputeMapMetrics(estimated, reference, /*distance_threshold_m=*/0.1);
  // Deliberately 1.0, not 0.0 — see ComputeMapMetrics's doc comment: an
  // empty reference means nothing can be confirmed correct.
  EXPECT_DOUBLE_EQ(result.outlier_ratio, 1.0);
  EXPECT_DOUBLE_EQ(result.completeness, 0.0);
  EXPECT_DOUBLE_EQ(result.chamfer_distance_m, 0.0);
}

TEST(MapMetrics, BothEmptyYieldsAllZerosNotNan) {
  const std::vector<Eigen::Vector3d> estimated;
  const std::vector<Eigen::Vector3d> reference;
  const auto result = uw::evaluation::ComputeMapMetrics(estimated, reference, /*distance_threshold_m=*/0.1);
  EXPECT_DOUBLE_EQ(result.chamfer_distance_m, 0.0);
  EXPECT_DOUBLE_EQ(result.completeness, 0.0);
  EXPECT_DOUBLE_EQ(result.outlier_ratio, 0.0);
  EXPECT_EQ(result.num_estimated_points, 0u);
  EXPECT_EQ(result.num_reference_points, 0u);
}
