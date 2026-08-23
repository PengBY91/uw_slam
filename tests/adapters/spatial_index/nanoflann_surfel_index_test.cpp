#include "adapters/spatial_index/nanoflann_surfel_index.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <random>

#include <gtest/gtest.h>

#include "mapping/surfel_map.hpp"

using uw::adapters::spatial_index::NanoflannSurfelIndex;
using uw::mapping::Surfel;
using uw::mapping::SurfelMap;
using uw::mapping::SurfelMapParams;
using uw::sensor_models::Pose3;

namespace {

// Order-independent comparison — see tests/mapping/surfel_map_test.cpp's
// identical helper for why (swap-and-pop never promised index stability,
// and the index-assisted CarveFreeSpace path processes candidates in a
// different order than brute force by design).
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
    EXPECT_NEAR(sorted_a[i].confidence, sorted_b[i].confidence, 1e-9) << "at i=" << i;
  }
}

}  // namespace

// --- Equivalence: the real nanoflann-backed index must match brute force
// exactly, same contract already proven against a test-only fake in
// tests/mapping/surfel_map_test.cpp (SurfelSpatialIndex.* there) — this
// file proves the REAL third-party-backed implementation honors it too.
// docs/superpowers/specs/2026-08-23-solver-and-mapping-oss-adoption.md §9.

TEST(NanoflannSurfelIndex, MergeCreateAndCarveMatchBruteForceExactly) {
  SurfelMapParams params;
  params.merge_distance_m = 0.3;
  params.free_space_confidence_decay = 0.5;
  params.free_space_removal_confidence_threshold = 0.4;

  SurfelMap brute(params);
  SurfelMap indexed(params, std::make_unique<NanoflannSurfelIndex>());

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

  const int removed_brute = brute.CarveFreeSpace({-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
  const int removed_indexed = indexed.CarveFreeSpace({-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
  EXPECT_EQ(removed_brute, removed_indexed);
  ExpectSameSurfelSet(brute.Surfels(), indexed.Surfels());
}

TEST(NanoflannSurfelIndex, CarveFreeSpaceRemovingMultipleSurfelsInOneCallMatchesBruteForce) {
  SurfelMapParams params;
  params.free_space_confidence_decay = 0.1;
  params.free_space_removal_confidence_threshold = 0.5;

  SurfelMap brute(params);
  SurfelMap indexed(params, std::make_unique<NanoflannSurfelIndex>());

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
  EXPECT_EQ(removed_brute, 5);
  ExpectSameSurfelSet(brute.Surfels(), indexed.Surfels());
}

TEST(NanoflannSurfelIndex, ReintegrateKeyframeRebuildMatchesBruteForce) {
  SurfelMapParams params;
  SurfelMap brute(params);
  SurfelMap indexed(params, std::make_unique<NanoflannSurfelIndex>());

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

// Regression test for the specific footgun called out in
// nanoflann_surfel_index.cpp: nanoflann's radius/distance parameters are
// SQUARED for an L2 metric. A point exactly on the boundary (distance ==
// radius_m, i.e. distance_sq == radius_m^2) must still be found — if the
// implementation accidentally passed a non-squared radius through, this
// boundary point would be spuriously excluded (radius_m < radius_m^2 for
// any radius_m > 1, or spuriously INCLUDED for radius_m < 1 with points
// actually outside — either way this test's exact-boundary construction
// catches it).
TEST(NanoflannSurfelIndex, FindNearestRespectsExactRadiusBoundaryNotItsSquare) {
  SurfelMapParams params;
  params.merge_distance_m = 2.0;  // > 1.0, so radius vs radius^2 diverge measurably
  SurfelMap indexed(params, std::make_unique<NanoflannSurfelIndex>());

  indexed.AddPoint(Eigen::Vector3d(0.0, 0.0, 0.0), 1.0);
  // Exactly at the merge_distance_m boundary (distance == 2.0, not 4.0).
  indexed.AddPoint(Eigen::Vector3d(2.0, 0.0, 0.0), 1.0);

  ASSERT_EQ(indexed.NumSurfels(), 1u) << "the second point must merge — it is exactly AT the "
                                          "2.0m gate, not accidentally gated at 2.0^2=4.0m or 2.0^0.5m";
}

TEST(NanoflannSurfelIndex, RandomizedOperationSequenceMatchesBruteForceAndDemonstratesSpeedup) {
  std::mt19937_64 rng(20260823);
  std::uniform_real_distribution<double> coord(-5.0, 5.0);
  std::uniform_real_distribution<double> confidence_dist(0.5, 5.0);

  SurfelMapParams params;
  params.merge_distance_m = 0.1;  // tight gate -> most points end up distinct surfels
  SurfelMap brute(params);
  SurfelMap indexed(params, std::make_unique<NanoflannSurfelIndex>());

  constexpr int kNumPoints = 3000;
  std::vector<Eigen::Vector3d> pts;
  std::vector<double> confidences;
  pts.reserve(kNumPoints);
  confidences.reserve(kNumPoints);
  for (int i = 0; i < kNumPoints; ++i) {
    pts.push_back(Eigen::Vector3d(coord(rng), coord(rng), coord(rng)));
    confidences.push_back(confidence_dist(rng));
  }

  const auto brute_start = std::chrono::steady_clock::now();
  for (int i = 0; i < kNumPoints; ++i) brute.AddPoint(pts[i], confidences[i]);
  const auto brute_elapsed = std::chrono::steady_clock::now() - brute_start;

  const auto indexed_start = std::chrono::steady_clock::now();
  for (int i = 0; i < kNumPoints; ++i) indexed.AddPoint(pts[i], confidences[i]);
  const auto indexed_elapsed = std::chrono::steady_clock::now() - indexed_start;

  ASSERT_EQ(brute.NumSurfels(), indexed.NumSurfels());
  EXPECT_EQ(brute.NumOutliersRejected(), indexed.NumOutliersRejected());
  ExpectSameSurfelSet(brute.Surfels(), indexed.Surfels());

  // Diagnostic only, deliberately NOT a pass/fail gate — per the design
  // doc's "avoid a hardware-relative absolute-time gate that goes flaky on
  // a different machine" note (§9). Printed so the direction of the
  // speedup is visible in test output without risking CI flakiness.
  const auto brute_ms = std::chrono::duration_cast<std::chrono::milliseconds>(brute_elapsed).count();
  const auto indexed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(indexed_elapsed).count();
  std::cout << "[diagnostic] brute-force AddPoint x" << kNumPoints << ": " << brute_ms
            << "ms, nanoflann-indexed: " << indexed_ms << "ms" << std::endl;
}
