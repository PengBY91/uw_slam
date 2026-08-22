#include "mapping/surfel_map.hpp"

#include <gtest/gtest.h>

#include "mapping/submap_manager.hpp"

using uw::mapping::SubmapManager;
using uw::mapping::Surfel;
using uw::mapping::SurfelMap;
using uw::mapping::SurfelMapParams;
using uw::sensor_models::Pose3;

TEST(SurfelMap, MergesNearbyPointsIntoOneConfidenceWeightedSurfel) {
  SurfelMap map(SurfelMapParams{/*merge_distance_m=*/0.1});
  map.AddPoint(Eigen::Vector3d(0.0, 0.0, 0.0), /*confidence=*/1.0);
  map.AddPoint(Eigen::Vector3d(0.02, 0.0, 0.0), /*confidence=*/1.0);  // 2cm away, within gate

  ASSERT_EQ(map.NumSurfels(), 1u);
  const Surfel& s = map.Surfels()[0];
  // Equal confidence -> simple average: (0 + 0.02) / 2 = 0.01.
  EXPECT_NEAR(s.position_W.x(), 0.01, 1e-9);
  EXPECT_NEAR(s.confidence, 2.0, 1e-9);  // accumulated, not averaged
}

TEST(SurfelMap, KeepsDistantPointsAsSeparateSurfels) {
  SurfelMap map(SurfelMapParams{/*merge_distance_m=*/0.1});
  map.AddPoint(Eigen::Vector3d(0.0, 0.0, 0.0), 1.0);
  map.AddPoint(Eigen::Vector3d(5.0, 0.0, 0.0), 1.0);  // far outside the merge gate
  EXPECT_EQ(map.NumSurfels(), 2u);
}

TEST(SurfelMap, HigherConfidencePointDominatesMergedPosition) {
  // Models the exact scenario this backend was chosen for (see
  // docs/superpowers/plans/2026-08-22-p3-d8-geometry-backend-decision.md):
  // a low-confidence sonar return and a high-confidence optical depth
  // return of the same physical point should merge toward the optical
  // estimate, not a naive 50/50 average.
  SurfelMap map(SurfelMapParams{/*merge_distance_m=*/1.0});
  constexpr double kSonarConfidence = 1.0;     // e.g. 1/variance_m2 for a noisy sonar return
  constexpr double kOpticalConfidence = 100.0;  // e.g. 1/variance_m2 for a precise optical return
  map.AddPoint(Eigen::Vector3d(0.0, 0.0, 0.0), kSonarConfidence);
  map.AddPoint(Eigen::Vector3d(1.0, 0.0, 0.0), kOpticalConfidence);

  ASSERT_EQ(map.NumSurfels(), 1u);
  const double expected_x =
      (0.0 * kSonarConfidence + 1.0 * kOpticalConfidence) / (kSonarConfidence + kOpticalConfidence);
  EXPECT_NEAR(map.Surfels()[0].position_W.x(), expected_x, 1e-9);
  EXPECT_GT(map.Surfels()[0].position_W.x(), 0.9);  // pulled close to the confident observation
}

TEST(SurfelMap, AddPointWithNormalMergesAndRenormalizesNormal) {
  SurfelMap map(SurfelMapParams{/*merge_distance_m=*/0.1});
  map.AddPointWithNormal(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0), 1.0);
  map.AddPointWithNormal(Eigen::Vector3d(0.01, 0.0, 0.0), Eigen::Vector3d(0.0, 1.0, 0.0), 1.0);

  ASSERT_EQ(map.NumSurfels(), 1u);
  const Eigen::Vector3d& normal = map.Surfels()[0].normal_W;
  EXPECT_NEAR(normal.norm(), 1.0, 1e-9);  // renormalized after averaging
  EXPECT_NEAR(normal.x(), normal.y(), 1e-9);  // equal-confidence average of +X and +Y
}

TEST(SurfelMap, PlainAddPointLeavesNormalUnknown) {
  SurfelMap map;
  map.AddPoint(Eigen::Vector3d(0.0, 0.0, 0.0), 1.0);
  EXPECT_TRUE(map.Surfels()[0].normal_W.isZero());
}

namespace {
uw::domain::MapEvidence MakePointCloudEvidence(const std::string& keyframe_id,
                                                const std::vector<Eigen::Vector3f>& points) {
  uw::domain::MapEvidence ev;
  ev.mutable_keyframe_id()->set_value(keyframe_id);
  ev.set_representation_type(uw::domain::MAP_REPRESENTATION_POINT_CLOUD);
  ev.set_reintegration_policy(uw::domain::MapEvidence::REINTEGRATION_POLICY_TRANSFORM_ONLY);
  std::string bytes(points.size() * 3 * sizeof(float), '\0');
  auto* raw = reinterpret_cast<float*>(bytes.data());
  for (std::size_t i = 0; i < points.size(); ++i) {
    raw[i * 3 + 0] = points[i].x();
    raw[i * 3 + 1] = points[i].y();
    raw[i * 3 + 2] = points[i].z();
  }
  ev.set_geometry_or_occupancy(bytes);
  return ev;
}
}  // namespace

// The actual "integration seam" proof this workstream exists for: real
// MapEvidence, decoded by the pre-existing (unmodified) SubmapManager, fed
// straight into SurfelMap — not a synthetic Eigen::Vector3d fixture
// invented just for this test.
TEST(SurfelMap, ConsumesSubmapManagerWorldPointsForKeyframeOutput) {
  SubmapManager manager;
  manager.AddMapEvidence(
      MakePointCloudEvidence("kf0", {{0.0f, 0.0f, 0.0f}, {0.02f, 0.0f, 0.0f}, {5.0f, 0.0f, 0.0f}}));
  Pose3 pose;
  pose.translation = Eigen::Vector3d(10.0, 0.0, 0.0);  // nontrivial pose, proves world-frame composition works
  manager.UpdateKeyframePose("kf0", pose);

  const auto world_points = manager.WorldPointsForKeyframe("kf0");
  ASSERT_EQ(world_points.size(), 3u);

  SurfelMap surfel_map(SurfelMapParams{/*merge_distance_m=*/0.1});
  for (const auto& point : world_points) surfel_map.AddPoint(point, /*confidence=*/1.0);

  // The first two source points (0m and 2cm apart) merge into one surfel
  // once transformed into world frame; the third (5m away) stays separate.
  EXPECT_EQ(surfel_map.NumSurfels(), 2u);
}
