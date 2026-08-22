#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>

namespace uw::evaluation {

struct MapMetricsResult {
  // Standard symmetric Chamfer distance: mean(estimated->reference nearest
  // distance) + mean(reference->estimated nearest distance). Some
  // literature averages instead of sums the two directions — this repo
  // sums them (matches the common point-cloud-registration convention);
  // read mean_estimated_to_reference_m/mean_reference_to_estimated_m
  // directly if a different convention is needed downstream.
  double chamfer_distance_m = 0.0;
  double mean_estimated_to_reference_m = 0.0;
  double mean_reference_to_estimated_m = 0.0;
  // Fraction of `reference` points with an `estimated` point within
  // distance_threshold_m (nearest-neighbor). 0.0 (not NaN) when either
  // input is empty — see ComputeMapMetrics's doc comment for the exact
  // per-case convention.
  double completeness = 0.0;
  // Fraction of `estimated` points with NO `reference` point within
  // distance_threshold_m. 1.0 (not 0.0) when `reference` is empty but
  // `estimated` is not — see ComputeMapMetrics's doc comment for why.
  double outlier_ratio = 0.0;
  // F-score at distance_threshold_m: harmonic mean of precision
  // (1 - outlier_ratio, i.e. the fraction of `estimated` points confirmed
  // by `reference`) and recall (completeness) — the standard point-cloud
  // reconstruction "F-score" metric (e.g. Knapitsch et al., Tanks and
  // Temples). 0.0 when precision + recall is 0 (avoids a 0/0 NaN).
  double f_score = 0.0;
  std::size_t num_estimated_points = 0;
  std::size_t num_reference_points = 0;
};

// Chamfer distance / completeness / outlier ratio between an estimated
// point set (e.g. SubmapManager::WorldPointsForKeyframe's output) and a
// reference point set (ground-truth surface samples). It operates directly
// on point clouds and does not require a particular map backend.
//
// Empty-input convention (avoid NaN, matching ComputeDepthMetrics's
// zero-not-NaN precedent in this same directory):
//   - `estimated` empty: mean_estimated_to_reference_m/outlier_ratio stay
//     0.0 (nothing to measure). completeness also stays 0.0 (an empty
//     estimate covers nothing, regardless of `reference`).
//   - `reference` empty, `estimated` non-empty: outlier_ratio is 1.0, NOT
//     0.0 — an empty reference means nothing can be confirmed correct, so
//     every estimated point is conservatively treated as unconfirmed
//     (fail-closed, matching this repo's general stance on "can't verify").
//     Completeness stays 0.0 (nothing to be complete about).
//   - both empty: everything 0.0.
//
// v1 scale limitation, documented rather than hidden (same pattern as
// other v1 notes in this codebase): brute-force O(|estimated|*|reference|)
// nearest-neighbor search, no spatial index. Fine for the point counts
// this is tested against; NOT fine for real map output — apps/replay_demo
// routinely produces several million map evidence points per run. Wiring
// this against real map output needs a KD-tree/octree first — follow-up
// work, not attempted here.
MapMetricsResult ComputeMapMetrics(const std::vector<Eigen::Vector3d>& estimated,
                                    const std::vector<Eigen::Vector3d>& reference,
                                    double distance_threshold_m);

}  // namespace uw::evaluation
