#include "mapping/submap_manager.hpp"

#include <gtest/gtest.h>

using uw::mapping::SubmapManager;
using uw::sensor_models::Pose3;

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

TEST(SubmapManager, TransformOnlyEvidenceFollowsLatestPoseWithoutReAdding) {
  SubmapManager manager;
  manager.AddMapEvidence(MakePointCloudEvidence("kf0", {{1.0f, 0.0f, 0.0f}}));

  Pose3 pose1;
  pose1.translation = Eigen::Vector3d(5.0, 0.0, 0.0);
  manager.UpdateKeyframePose("kf0", pose1);

  auto points1 = manager.WorldPointsForKeyframe("kf0");
  ASSERT_EQ(points1.size(), 1u);
  EXPECT_NEAR((points1[0] - Eigen::Vector3d(6.0, 0.0, 0.0)).norm(), 0.0, 1e-5);

  // Simulate a pose-graph correction: pose changes, evidence is NOT
  // re-added, but WorldPointsForKeyframe reflects the new pose immediately.
  Pose3 pose2;
  pose2.translation = Eigen::Vector3d(0.0, 10.0, 0.0);
  manager.UpdateKeyframePose("kf0", pose2);

  auto points2 = manager.WorldPointsForKeyframe("kf0");
  ASSERT_EQ(points2.size(), 1u);
  EXPECT_NEAR((points2[0] - Eigen::Vector3d(1.0, 10.0, 0.0)).norm(), 0.0, 1e-5);
  EXPECT_TRUE(manager.StaleKeyframes().empty());
}

TEST(SubmapManager, FullRefuseEvidenceIsMarkedStaleOnPoseChange) {
  SubmapManager manager;
  auto evidence = MakePointCloudEvidence("kf1", {{0.0f, 0.0f, 0.0f}});
  evidence.set_reintegration_policy(uw::domain::MapEvidence::REINTEGRATION_POLICY_FULL_REFUSE);
  manager.AddMapEvidence(evidence);

  manager.UpdateKeyframePose("kf1", Pose3::Identity());
  const auto stale = manager.StaleKeyframes();
  ASSERT_EQ(stale.size(), 1u);
  EXPECT_EQ(stale[0], "kf1");
}

TEST(SubmapManager, QueryNearestPointFindsClosestAcrossKeyframesWithinGate) {
  SubmapManager manager;
  manager.AddMapEvidence(MakePointCloudEvidence("landmarks", {{0.0f, 0.0f, 0.0f}}));
  manager.AddMapEvidence(MakePointCloudEvidence("landmarks", {{5.0f, 0.0f, 0.0f}}));
  manager.UpdateKeyframePose("landmarks", Pose3::Identity());
  // A separate keyframe bucket, to prove the query isn't limited to one.
  manager.AddMapEvidence(MakePointCloudEvidence("kf2", {{0.0f, 0.9f, 0.0f}}));
  manager.UpdateKeyframePose("kf2", Pose3::Identity());

  // (0, 1, 0) is 1.0m from (0,0,0), 0.9m from (0, 0.9, 0), and ~5.1m from
  // (5,0,0) — the second, closer point should win even though it's in a
  // different keyframe bucket than the first candidate found.
  const auto nearest = manager.QueryNearestPoint(Eigen::Vector3d(0.0, 1.0, 0.0), /*max_distance_m=*/2.0);
  ASSERT_TRUE(nearest.has_value());
  EXPECT_NEAR((*nearest - Eigen::Vector3d(0.0, 0.9, 0.0)).norm(), 0.0, 1e-5);

  // Outside the gate entirely -> no match.
  EXPECT_FALSE(manager.QueryNearestPoint(Eigen::Vector3d(100.0, 100.0, 100.0), 2.0).has_value());
}

TEST(SubmapManager, RemoveMapEvidenceDropsOnlyTheMatchingEntry) {
  SubmapManager manager;
  auto keep = MakePointCloudEvidence("landmarks", {{0.0f, 0.0f, 0.0f}});
  keep.mutable_evidence_id()->set_value("keep");
  auto drop = MakePointCloudEvidence("landmarks", {{5.0f, 0.0f, 0.0f}});
  drop.mutable_evidence_id()->set_value("drop");
  manager.AddMapEvidence(keep);
  manager.AddMapEvidence(drop);
  manager.UpdateKeyframePose("landmarks", Pose3::Identity());
  ASSERT_EQ(manager.WorldPointsForKeyframe("landmarks").size(), 2u);

  manager.RemoveMapEvidence("landmarks", "drop");

  const auto remaining = manager.WorldPointsForKeyframe("landmarks");
  ASSERT_EQ(remaining.size(), 1u);
  EXPECT_NEAR((remaining[0] - Eigen::Vector3d(0.0, 0.0, 0.0)).norm(), 0.0, 1e-5);

  // Removing something that no longer exists (or never did) is a no-op,
  // not an error.
  manager.RemoveMapEvidence("landmarks", "drop");
  manager.RemoveMapEvidence("no_such_keyframe", "keep");
  EXPECT_EQ(manager.WorldPointsForKeyframe("landmarks").size(), 1u);
}
