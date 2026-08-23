#include "mapping/surfel_map.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <random>

#include <gtest/gtest.h>

#include "mapping/submap_manager.hpp"

using uw::mapping::SubmapManager;
using uw::mapping::Surfel;
using uw::mapping::SurfelMap;
using uw::mapping::SurfelMapParams;
using uw::mapping::SurfelSpatialIndex;
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
  // A low-confidence sonar return and a high-confidence optical depth
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

// --- Pose-correction reintegration (P3 roadmap item 4: submap transform/
// reintegration and stale-evidence management, applied to surfels) --------

TEST(SurfelMap, ReintegratingAKeyframeAfterPoseCorrectionRefusesItsObservationsAtTheNewPose) {
  SurfelMap map;  // default merge_distance_m = 0.05

  Pose3 kf0_pose;  // identity: local (1,0,0) -> world (1,0,0)
  map.AddKeyframeObservation("kf0", Eigen::Vector3d(1.0, 0.0, 0.0), /*confidence=*/1.0, kf0_pose);

  Pose3 kf1_pose_initial;
  // local (0,0,0) -> world (1,0,0): same physical point as kf0's, so the
  // two must merge under the default 0.05m gate.
  kf1_pose_initial.translation = Eigen::Vector3d(1.0, 0.0, 0.0);
  map.AddKeyframeObservation("kf1", Eigen::Vector3d(0.0, 0.0, 0.0), /*confidence=*/1.0, kf1_pose_initial);

  ASSERT_EQ(map.NumTrackedKeyframes(), 2u);
  ASSERT_EQ(map.NumSurfels(), 1u) << "both observe world point (1,0,0), must merge under the 0.05m gate";
  EXPECT_NEAR(map.Surfels()[0].position_W.x(), 1.0, 1e-9);
  EXPECT_NEAR(map.Surfels()[0].confidence, 2.0, 1e-9);

  // Pose-graph correction: kf1 moves to translation (2,0,0) -> its local
  // (0,0,0) observation now lands at world (2,0,0), a different physical
  // point, well outside the merge gate from kf0's (still) (1,0,0).
  Pose3 kf1_pose_corrected;
  kf1_pose_corrected.translation = Eigen::Vector3d(2.0, 0.0, 0.0);
  map.ReintegrateKeyframe("kf1", kf1_pose_corrected);

  ASSERT_EQ(map.NumSurfels(), 2u) << "after correction, kf0 and kf1 observe different physical points";
  // unordered_map iteration order is not guaranteed — identify each surfel
  // by its (now well-separated) position rather than assuming an index.
  bool found_kf0_point = false;
  bool found_kf1_point = false;
  for (const auto& surfel : map.Surfels()) {
    if (std::abs(surfel.position_W.x() - 1.0) < 1e-9) {
      found_kf0_point = true;
      EXPECT_NEAR(surfel.confidence, 1.0, 1e-9);
    } else if (std::abs(surfel.position_W.x() - 2.0) < 1e-9) {
      found_kf1_point = true;
      EXPECT_NEAR(surfel.confidence, 1.0, 1e-9);
    }
  }
  EXPECT_TRUE(found_kf0_point);
  EXPECT_TRUE(found_kf1_point);
}

TEST(SurfelMap, ReintegratingAKeyframeThatStillMergesUpdatesTheFusedPositionCorrectly) {
  SurfelMap map;  // default merge_distance_m = 0.05

  Pose3 kf0_pose;
  map.AddKeyframeObservation("kf0", Eigen::Vector3d(0.0, 0.0, 0.0), /*confidence=*/3.0, kf0_pose);

  Pose3 kf1_pose_initial;
  kf1_pose_initial.translation = Eigen::Vector3d(0.02, 0.0, 0.0);
  map.AddKeyframeObservation("kf1", Eigen::Vector3d(0.0, 0.0, 0.0), /*confidence=*/1.0, kf1_pose_initial);

  ASSERT_EQ(map.NumSurfels(), 1u);
  EXPECT_NEAR(map.Surfels()[0].position_W.x(), 0.005, 1e-9);  // (0*3 + 0.02*1) / 4
  EXPECT_NEAR(map.Surfels()[0].confidence, 4.0, 1e-9);

  Pose3 kf1_pose_corrected;
  kf1_pose_corrected.translation = Eigen::Vector3d(0.04, 0.0, 0.0);  // still within the 0.05m gate from kf0
  map.ReintegrateKeyframe("kf1", kf1_pose_corrected);

  ASSERT_EQ(map.NumSurfels(), 1u) << "0.04m apart is still within the default 0.05m merge gate";
  EXPECT_NEAR(map.Surfels()[0].position_W.x(), 0.01, 1e-9);  // (0*3 + 0.04*1) / 4
  EXPECT_NEAR(map.Surfels()[0].confidence, 4.0, 1e-9);
}

TEST(SurfelMap, PlainAddPointSurfelsSurviveReintegrationOfAnUnrelatedKeyframe) {
  // Locks in a real bug found and fixed while building this test: a naive
  // ReintegrateKeyframe implementation that unconditionally clears and
  // rebuilds `surfels_` from only the keyframe-attributed ledger would
  // silently destroy points added via the plain, non-keyframe-attributed
  // AddPoint/AddPointWithNormal — see surfel_map.cpp's
  // kUnattributedKeyframeId for the fix (those calls are internally
  // recorded too, under a reserved key, so a rebuild replays them).
  SurfelMap map;
  map.AddPoint(Eigen::Vector3d(100.0, 0.0, 0.0), /*confidence=*/1.0);  // far from anything below; unattributed

  Pose3 kf0_pose;
  map.AddKeyframeObservation("kf0", Eigen::Vector3d(0.0, 0.0, 0.0), /*confidence=*/1.0, kf0_pose);

  ASSERT_EQ(map.NumSurfels(), 2u);
  ASSERT_EQ(map.NumTrackedKeyframes(), 1u) << "the plain AddPoint call above must not count as a tracked keyframe";

  Pose3 kf0_pose_corrected;
  kf0_pose_corrected.translation = Eigen::Vector3d(5.0, 0.0, 0.0);
  map.ReintegrateKeyframe("kf0", kf0_pose_corrected);

  ASSERT_EQ(map.NumSurfels(), 2u) << "the unattributed point must survive a rebuild triggered by kf0's reintegration";
  bool found_unattributed = false;
  bool found_kf0 = false;
  for (const auto& surfel : map.Surfels()) {
    if (std::abs(surfel.position_W.x() - 100.0) < 1e-9) found_unattributed = true;
    if (std::abs(surfel.position_W.x() - 5.0) < 1e-9) found_kf0 = true;
  }
  EXPECT_TRUE(found_unattributed);
  EXPECT_TRUE(found_kf0);
}

TEST(SurfelMap, ReintegrateKeyframeIsNoOpForAnUntrackedKeyframeId) {
  SurfelMap map;
  map.AddPoint(Eigen::Vector3d(1.0, 2.0, 3.0), /*confidence=*/1.0);
  ASSERT_EQ(map.NumSurfels(), 1u);

  Pose3 arbitrary;
  arbitrary.translation = Eigen::Vector3d(50.0, 50.0, 50.0);
  map.ReintegrateKeyframe("kf_never_added", arbitrary);

  ASSERT_EQ(map.NumSurfels(), 1u);
  EXPECT_NEAR(map.Surfels()[0].position_W.x(), 1.0, 1e-9);  // unchanged
}

// --- P3 roadmap item 3 (D10): outlier suppression ------------------------

TEST(SurfelMap, OutlierGateAcceptsANoisyButStatisticallyConsistentReobservation) {
  // Both observations at confidence=100 (variance=0.01 each), so
  // combined_variance=0.02, and default outlier_gate_sigma=3.0 gives a
  // gate threshold of sqrt(0.02)*3 ~= 0.4243m. A 0.3m difference
  // (diff_sq=0.09 <= gate_sq=0.18) is within the gate -> must merge.
  SurfelMap map(SurfelMapParams{/*merge_distance_m=*/1.0});
  map.AddPoint(Eigen::Vector3d(0.0, 0.0, 0.0), /*confidence=*/100.0);
  map.AddPoint(Eigen::Vector3d(0.3, 0.0, 0.0), /*confidence=*/100.0);

  ASSERT_EQ(map.NumSurfels(), 1u) << "0.3m apart must pass the outlier gate at these confidences";
  EXPECT_NEAR(map.Surfels()[0].position_W.x(), 0.15, 1e-9);  // equal-confidence average
  EXPECT_NEAR(map.Surfels()[0].confidence, 200.0, 1e-9);
  EXPECT_EQ(map.NumOutliersRejected(), 0u);
}

TEST(SurfelMap, OutlierGateRejectsAStatisticallyInconsistentObservationAsANewSurfel) {
  // Same confidences as the accepting case above (gate threshold ~=
  // 0.4243m), but 0.5m apart (diff_sq=0.25 > gate_sq=0.18) — must NOT
  // merge, and instead becomes a second, separate surfel (not a discarded
  // observation).
  SurfelMap map(SurfelMapParams{/*merge_distance_m=*/1.0});  // wide enough that FindNearest still finds a candidate
  map.AddPoint(Eigen::Vector3d(0.0, 0.0, 0.0), /*confidence=*/100.0);
  map.AddPoint(Eigen::Vector3d(0.5, 0.0, 0.0), /*confidence=*/100.0);

  ASSERT_EQ(map.NumSurfels(), 2u) << "0.5m apart must fail the outlier gate at these confidences";
  EXPECT_EQ(map.NumOutliersRejected(), 1u);
  // Both original observations preserved verbatim (no averaging happened).
  bool found_first = false, found_second = false;
  for (const auto& surfel : map.Surfels()) {
    if (std::abs(surfel.position_W.x() - 0.0) < 1e-9) {
      found_first = true;
      EXPECT_NEAR(surfel.confidence, 100.0, 1e-9);
    }
    if (std::abs(surfel.position_W.x() - 0.5) < 1e-9) {
      found_second = true;
      EXPECT_NEAR(surfel.confidence, 100.0, 1e-9);
    }
  }
  EXPECT_TRUE(found_first);
  EXPECT_TRUE(found_second);
}

// --- P3 roadmap item 3 (D10): free-space/occlusion handling ---------------

TEST(SurfelMap, CarveFreeSpaceDownweightsAndEventuallyRemovesAContradictedSurfel) {
  // decay=0.5, removal_threshold=0.4: after 1 carve, 1.0*0.5=0.5 (kept,
  // 0.5 > 0.4); after a 2nd carve, 0.5*0.5=0.25 (removed, 0.25 <= 0.4) —
  // both values hand-computable and distinct enough to catch an off-by-one
  // in the decay/removal loop.
  SurfelMapParams params;
  params.free_space_confidence_decay = 0.5;
  params.free_space_removal_confidence_threshold = 0.4;
  SurfelMap map(params);
  map.AddPoint(Eigen::Vector3d(1.0, 0.0, 0.0), /*confidence=*/1.0);
  ASSERT_EQ(map.NumSurfels(), 1u);

  // Ray from origin straight through (1,0,0) to (2,0,0): the surfel sits
  // exactly on the segment (t=0.5), perpendicular distance 0.
  const Eigen::Vector3d origin(0.0, 0.0, 0.0);
  const Eigen::Vector3d end(2.0, 0.0, 0.0);

  EXPECT_EQ(map.CarveFreeSpace(origin, end), 0) << "first carve only downweights, does not remove";
  ASSERT_EQ(map.NumSurfels(), 1u);
  EXPECT_NEAR(map.Surfels()[0].confidence, 0.5, 1e-9);

  EXPECT_EQ(map.CarveFreeSpace(origin, end), 1) << "second carve drops confidence to 0.25, at/below threshold";
  EXPECT_EQ(map.NumSurfels(), 0u);
}

TEST(SurfelMap, CarveFreeSpaceLeavesSurfelsOutsideTheRayCorridorUntouched) {
  SurfelMap map;  // default free_space_corridor_radius_m == merge_distance_m == 0.05
  map.AddPoint(Eigen::Vector3d(1.0, 1.0, 0.0), /*confidence=*/1.0);  // 1m off the x-axis ray below

  const int removed = map.CarveFreeSpace(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(2.0, 0.0, 0.0));

  EXPECT_EQ(removed, 0);
  ASSERT_EQ(map.NumSurfels(), 1u);
  EXPECT_NEAR(map.Surfels()[0].confidence, 1.0, 1e-9) << "far from the ray corridor -> untouched";
}

TEST(SurfelMap, CarveFreeSpaceDoesNotAffectSurfelsAtOrBeyondTheObservedPoint) {
  // A surfel exactly AT the ray's end point (the new observation itself)
  // is the thing being observed, not free space the ray passed through —
  // must not be carved. Same for a surfel BEYOND the end point (t > 1):
  // the ray never reached that far, so nothing is known about it.
  SurfelMapParams params;
  params.free_space_confidence_decay = 0.5;
  SurfelMap map(params);
  map.AddPoint(Eigen::Vector3d(2.0, 0.0, 0.0), /*confidence=*/1.0);   // at the ray's end (t=1)
  map.AddPoint(Eigen::Vector3d(3.0, 0.0, 0.0), /*confidence=*/1.0);   // beyond the ray's end (t=1.5)
  ASSERT_EQ(map.NumSurfels(), 2u);

  const int removed = map.CarveFreeSpace(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(2.0, 0.0, 0.0));

  EXPECT_EQ(removed, 0);
  for (const auto& surfel : map.Surfels()) {
    EXPECT_NEAR(surfel.confidence, 1.0, 1e-9) << "neither at-end nor beyond-end surfels should be carved";
  }
}

TEST(SurfelMap, CarveFreeSpaceDoesNotCarveASurfelNearButNotExactlyAtTheEndpoint) {
  // Regression test for a real self-carve bug caught before it shipped:
  // a surfel at (1.99,0,0), ray [(0,0,0), (2,0,0)] — t = 1.99/2 = 0.995,
  // strictly less than 1.0, and perpendicular distance 0 (sitting exactly
  // on the ray) — would pass the t<1 boundary check alone and get carved,
  // even though it is only 0.01m from the ray's own endpoint (well inside
  // the default 0.05m corridor radius). This is exactly what happens when
  // a confidence-weighted merge nudges a surfel a hair short of the exact
  // pixel it was just fused from (FuseDepthIntoSurfels calls
  // AddKeyframeObservation* immediately before CarveFreeSpace for the same
  // point) — must NOT be carved.
  SurfelMapParams params;
  params.free_space_confidence_decay = 0.5;
  SurfelMap map(params);
  map.AddPoint(Eigen::Vector3d(1.99, 0.0, 0.0), /*confidence=*/1.0);

  const int removed = map.CarveFreeSpace(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(2.0, 0.0, 0.0));

  EXPECT_EQ(removed, 0);
  ASSERT_EQ(map.NumSurfels(), 1u);
  EXPECT_NEAR(map.Surfels()[0].confidence, 1.0, 1e-9) << "near-endpoint surfel must be protected, not carved";
}

TEST(SurfelMap, CarveFreeSpaceEffectDoesNotSurviveReintegrationOfTheContributingKeyframe) {
  // Documents and locks in a real, deliberate interaction: carving mutates
  // surfels_ directly, not the per-keyframe observation ledger, so a
  // ReintegrateKeyframe rebuild — which replays ONLY the ledger — reverts
  // any confidence decay/removal a prior carve caused. This is not a bug;
  // see CarveFreeSpace's doc comment in the header for why.
  SurfelMapParams params;
  params.free_space_confidence_decay = 0.5;
  SurfelMap map(params);

  Pose3 kf0_pose;
  map.AddKeyframeObservation("kf0", Eigen::Vector3d(1.0, 0.0, 0.0), /*confidence=*/1.0, kf0_pose);
  ASSERT_EQ(map.NumSurfels(), 1u);

  map.CarveFreeSpace(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(2.0, 0.0, 0.0));
  ASSERT_EQ(map.NumSurfels(), 1u);
  EXPECT_NEAR(map.Surfels()[0].confidence, 0.5, 1e-9) << "carved down from 1.0";

  // Reintegrating kf0 (even at the SAME pose) rebuilds surfels_ purely from
  // the ledger, which never recorded the carve.
  map.ReintegrateKeyframe("kf0", kf0_pose);

  ASSERT_EQ(map.NumSurfels(), 1u);
  EXPECT_NEAR(map.Surfels()[0].confidence, 1.0, 1e-9) << "carve effect did not survive reintegration";
}

// --- SurfelSpatialIndex wiring: index-assisted path must match brute force
// exactly (2026-08-23 solver-and-mapping-oss-adoption design, workstream B)
// --------------------------------------------------------------------------

namespace {

// Test-only reference SurfelSpatialIndex: mirrors surfels_'s positions via
// the Notify* contract and answers queries by the SAME linear-scan
// algorithm SurfelMap's own brute-force path uses (see FindNearest in
// surfel_map.cpp) — including its exact "<=" last-candidate-wins tie
// break. This exists to prove SurfelMap's Notify*/query call sites
// themselves are correct, independently of whatever real spatial data
// structure (e.g. nanoflann, a later workstream) implements this interface
// for real — a bug here would be a bug in SurfelMap's wiring, not in a
// third-party library.
class BruteForceReferenceIndex : public SurfelSpatialIndex {
 public:
  void Clear() override { points_.clear(); }

  void NotifyInserted(std::size_t index, const Eigen::Vector3d& position_W) override {
    ASSERT_EQ(index, points_.size()) << "NotifyInserted must always append at the current end";
    points_.push_back(position_W);
  }

  void NotifyMoved(std::size_t index, const Eigen::Vector3d& new_position_W) override {
    points_.at(index) = new_position_W;
  }

  void NotifyRemovedBySwapPop(std::size_t index, std::size_t previous_back_index) override {
    ASSERT_EQ(previous_back_index + 1, points_.size())
        << "previous_back_index must be the index's size() - 1 at call time";
    if (index != previous_back_index) points_.at(index) = points_.at(previous_back_index);
    points_.pop_back();
  }

  std::optional<std::size_t> FindNearestWithinRadius(const Eigen::Vector3d& query,
                                                       double radius_m) const override {
    std::optional<std::size_t> best;
    double best_distance = radius_m;
    for (std::size_t i = 0; i < points_.size(); ++i) {
      const double distance = (points_[i] - query).norm();
      if (distance <= best_distance) {
        best_distance = distance;
        best = i;
      }
    }
    return best;
  }

  // Deliberately the maximal (always-safe) over-approximation: every
  // currently-indexed index, unfiltered. Proves CarveFreeSpace's own exact
  // narrow-phase math is sufficient for correctness even when the broad
  // phase narrows nothing — see SurfelSpatialIndex's header doc comment.
  std::vector<std::size_t> FindCandidatesNearSegment(const Eigen::Vector3d&, const Eigen::Vector3d&,
                                                       double) const override {
    std::vector<std::size_t> all(points_.size());
    for (std::size_t i = 0; i < all.size(); ++i) all[i] = i;
    return all;
  }

 private:
  std::vector<Eigen::Vector3d> points_;
};

// Order-independent comparison: existing tests already identify surfels by
// content, not vector index (swap-and-pop never promised index stability),
// and the index-assisted CarveFreeSpace path processes candidates in a
// different order than brute force by design (see surfel_map.cpp) — so
// exact index-for-index vector equality is the wrong thing to assert here.
void ExpectSameSurfelSet(const std::vector<Surfel>& a, const std::vector<Surfel>& b) {
  ASSERT_EQ(a.size(), b.size());
  auto by_position = [](const Surfel& x, const Surfel& y) {
    if (x.position_W.x() != y.position_W.x()) return x.position_W.x() < y.position_W.x();
    if (x.position_W.y() != y.position_W.y()) return x.position_W.y() < y.position_W.y();
    return x.position_W.z() < y.position_W.z();
  };
  std::vector<Surfel> sorted_a = a, sorted_b = b;
  std::sort(sorted_a.begin(), sorted_a.end(), by_position);
  std::sort(sorted_b.begin(), sorted_b.end(), by_position);
  for (std::size_t i = 0; i < sorted_a.size(); ++i) {
    EXPECT_NEAR((sorted_a[i].position_W - sorted_b[i].position_W).norm(), 0.0, 1e-9) << "at i=" << i;
    EXPECT_NEAR((sorted_a[i].normal_W - sorted_b[i].normal_W).norm(), 0.0, 1e-9) << "at i=" << i;
    EXPECT_NEAR(sorted_a[i].confidence, sorted_b[i].confidence, 1e-9) << "at i=" << i;
  }
}

}  // namespace

TEST(SurfelSpatialIndex, MergeCreateAndCarveMatchBruteForceExactly) {
  SurfelMapParams params;
  params.merge_distance_m = 0.3;
  params.free_space_confidence_decay = 0.5;
  params.free_space_removal_confidence_threshold = 0.4;

  SurfelMap brute(params);
  SurfelMap indexed(params, std::make_unique<BruteForceReferenceIndex>());

  const std::vector<Eigen::Vector3d> points = {
      {0.0, 0.0, 0.0}, {0.1, 0.0, 0.0},  // merges with the first (0.1 < 0.3)
      {5.0, 0.0, 0.0},                    // far: separate surfel
      {5.05, 0.0, 0.0},                   // merges with the previous one
  };
  for (const auto& p : points) {
    brute.AddPoint(p, /*confidence=*/1.0);
    indexed.AddPoint(p, /*confidence=*/1.0);
  }
  ASSERT_EQ(brute.NumSurfels(), indexed.NumSurfels());
  EXPECT_EQ(brute.NumOutliersRejected(), indexed.NumOutliersRejected());
  ExpectSameSurfelSet(brute.Surfels(), indexed.Surfels());

  // A ray through the origin-adjacent surfel: one carve downweights (not
  // removed yet at these params), matching on both paths.
  const int removed_brute = brute.CarveFreeSpace({-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
  const int removed_indexed = indexed.CarveFreeSpace({-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
  EXPECT_EQ(removed_brute, removed_indexed);
  ExpectSameSurfelSet(brute.Surfels(), indexed.Surfels());
}

TEST(SurfelSpatialIndex, CarveFreeSpaceRemovingMultipleSurfelsInOneCallMatchesBruteForce) {
  // Exercises the index path's descending-order swap-and-pop handling with
  // more than one removal in a single CarveFreeSpace call — the scenario
  // most likely to expose an off-by-one in candidate index bookkeeping.
  SurfelMapParams params;
  params.free_space_confidence_decay = 0.1;  // one carve drops well below any reasonable threshold
  params.free_space_removal_confidence_threshold = 0.5;

  SurfelMap brute(params);
  SurfelMap indexed(params, std::make_unique<BruteForceReferenceIndex>());

  // Five points sitting on the ray corridor between (0,0,0) and (10,0,0),
  // interleaved with two points far off the ray that must survive
  // untouched — proves candidates outside the corridor are correctly
  // skipped by the narrow-phase math regardless of index order.
  const std::vector<Eigen::Vector3d> points = {
      {1.0, 0.0, 0.0}, {9.0, 9.0, 0.0}, {3.0, 0.0, 0.0}, {5.0, 0.0, 0.0},
      {7.0, 0.0, 0.0}, {2.0, 2.0, 0.0}, {9.0, 0.0, 0.0},
  };
  for (const auto& p : points) {
    brute.AddPoint(p, /*confidence=*/1.0);
    indexed.AddPoint(p, /*confidence=*/1.0);
  }
  ASSERT_EQ(brute.NumSurfels(), points.size());

  const int removed_brute = brute.CarveFreeSpace({0.0, 0.0, 0.0}, {10.0, 0.0, 0.0});
  const int removed_indexed = indexed.CarveFreeSpace({0.0, 0.0, 0.0}, {10.0, 0.0, 0.0});
  EXPECT_EQ(removed_brute, removed_indexed);
  EXPECT_EQ(removed_brute, 5) << "all 5 on-ray points (strictly between origin and the endpoint) removed";
  ExpectSameSurfelSet(brute.Surfels(), indexed.Surfels());
}

TEST(SurfelSpatialIndex, ReintegrateKeyframeRebuildMatchesBruteForce) {
  SurfelMapParams params;  // default merge_distance_m = 0.05
  SurfelMap brute(params);
  SurfelMap indexed(params, std::make_unique<BruteForceReferenceIndex>());

  Pose3 kf0_pose;
  Pose3 kf1_pose_initial;
  kf1_pose_initial.translation = Eigen::Vector3d(1.0, 0.0, 0.0);
  for (SurfelMap* map : {&brute, &indexed}) {
    map->AddKeyframeObservation("kf0", Eigen::Vector3d(1.0, 0.0, 0.0), 1.0, kf0_pose);
    map->AddKeyframeObservation("kf1", Eigen::Vector3d(0.0, 0.0, 0.0), 1.0, kf1_pose_initial);
  }
  ExpectSameSurfelSet(brute.Surfels(), indexed.Surfels());

  Pose3 kf1_pose_corrected;
  kf1_pose_corrected.translation = Eigen::Vector3d(2.0, 0.0, 0.0);
  brute.ReintegrateKeyframe("kf1", kf1_pose_corrected);
  indexed.ReintegrateKeyframe("kf1", kf1_pose_corrected);

  ASSERT_EQ(brute.NumSurfels(), 2u);
  ASSERT_EQ(indexed.NumSurfels(), 2u);
  ExpectSameSurfelSet(brute.Surfels(), indexed.Surfels());
}

TEST(SurfelSpatialIndex, RandomizedOperationSequenceMatchesBruteForce) {
  // Seeded, never-reseeded RNG per this repo's determinism convention
  // (CLAUDE.md) — not for reproducing a physical scenario, just to avoid
  // hand-picking coordinates that might accidentally hide an indexing bug.
  std::mt19937_64 rng(20260823);
  std::uniform_real_distribution<double> coord(-2.0, 2.0);
  std::uniform_real_distribution<double> confidence_dist(0.5, 5.0);

  SurfelMapParams params;
  params.merge_distance_m = 0.2;
  SurfelMap brute(params);
  SurfelMap indexed(params, std::make_unique<BruteForceReferenceIndex>());

  for (int i = 0; i < 200; ++i) {
    const Eigen::Vector3d point(coord(rng), coord(rng), coord(rng));
    const double confidence = confidence_dist(rng);
    brute.AddPoint(point, confidence);
    indexed.AddPoint(point, confidence);
  }
  ASSERT_EQ(brute.NumSurfels(), indexed.NumSurfels());
  EXPECT_EQ(brute.NumOutliersRejected(), indexed.NumOutliersRejected());
  ExpectSameSurfelSet(brute.Surfels(), indexed.Surfels());

  // A handful of carve rays through the same random volume.
  for (int i = 0; i < 20; ++i) {
    const Eigen::Vector3d origin(coord(rng), coord(rng), coord(rng));
    const Eigen::Vector3d end(coord(rng), coord(rng), coord(rng));
    const int removed_brute = brute.CarveFreeSpace(origin, end);
    const int removed_indexed = indexed.CarveFreeSpace(origin, end);
    ASSERT_EQ(removed_brute, removed_indexed) << "carve ray " << i;
    ExpectSameSurfelSet(brute.Surfels(), indexed.Surfels());
  }
}
