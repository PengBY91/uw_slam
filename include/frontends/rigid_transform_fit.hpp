#pragma once

#include <cstdint>
#include <optional>
#include <random>
#include <vector>

#include <Eigen/Core>

#include "sensor_models/geometry.hpp"

namespace uw::frontends {

// Closed-form Kabsch/Procrustes rigid-transform fit: finds the Pose3 T
// minimizing sum_i || b[i] - T.Apply(a[i]) ||^2 over matched point pairs
// a[i] (expressed in frame A) / b[i] (expressed in frame B) — i.e. T
// transforms points from frame A into frame B. `a` and `b` must be the
// same length and index-aligned (same convention as PatchMatch pairs).
//
// Requires at least 3 points; returns std::nullopt (rather than a
// numerically garbage transform) if there are too few points or the SVD
// used internally does not converge. Does NOT check for a degenerate
// (near-collinear/coplanar) point configuration beyond what the SVD
// itself reflects — callers that need a stronger conditioning guarantee
// should filter their input point set first. Not robust to outliers — a
// single bad correspondence pulls the whole fit; see
// FitRigidTransformRansac below for a robustified version.
std::optional<uw::sensor_models::Pose3> FitRigidTransform(const std::vector<Eigen::Vector3d>& a,
                                                            const std::vector<Eigen::Vector3d>& b);

struct RansacParams {
  double inlier_threshold_m = 0.3;  // max ||b[i] - T.Apply(a[i])|| to count as an inlier
  int max_iterations = 200;
  int min_inliers = 3;  // best-found inlier set smaller than this rejects the whole fit
};

// RANSAC-robustified FitRigidTransform: repeatedly fits a candidate
// transform from a random minimal (3-point) sample of the correspondences,
// scores it by how many of ALL correspondences it explains within
// `inlier_threshold_m`, keeps the best-scoring candidate's inlier set, then
// does one final FitRigidTransform over that full inlier set (not just the
// 3-point sample) for the returned transform — standard RANSAC refinement.
// Falls back to plain FitRigidTransform when there are exactly 3 points
// (nothing to robustify). Returns std::nullopt if there are fewer than 3
// points, or if even the best sample's inlier count falls short of
// `min_inliers`.
//
// Deterministic given the same `rng` state on entry — `rng` must be an
// explicitly seeded, never-reseeded generator threaded in by the caller
// (CLAUDE.md's RNG discipline / the L2 determinism test), not a
// freshly-constructed or global one; StereoLandmarkVoFrontend owns one
// seeded once at construction for exactly this reason.
std::optional<uw::sensor_models::Pose3> FitRigidTransformRansac(const std::vector<Eigen::Vector3d>& a,
                                                                  const std::vector<Eigen::Vector3d>& b,
                                                                  const RansacParams& params,
                                                                  std::mt19937_64& rng);

}  // namespace uw::frontends
