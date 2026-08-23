#pragma once

#include <cstdint>
#include <limits>
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
// should filter their input point set first (FitRigidTransformRansac
// below does exactly that via its covariance-estimation conditioning
// check). Not robust to outliers — a single bad correspondence pulls the
// whole fit; see FitRigidTransformRansac below for a robustified version.
// Kept as a plain Pose3-returning function since RANSAC's own minimal
// (3-point) sample fits have no meaningful covariance to report.
std::optional<uw::sensor_models::Pose3> FitRigidTransform(const std::vector<Eigen::Vector3d>& a,
                                                            const std::vector<Eigen::Vector3d>& b);

struct RansacParams {
  double inlier_threshold_m = 0.3;  // max ||b[i] - T.Apply(a[i])|| to count as an inlier
  int max_iterations = 200;
  int min_inliers = 3;  // best-found inlier set smaller than this rejects the whole fit
};

// Numerical-Jacobian covariance estimation tuning for
// FitRigidTransformRansac's final refit over the inlier set. Defaults are
// generous (a conditioning FAILURE should mean "genuinely degenerate
// geometry", not "reasonable noise").
struct CovarianceEstimationParams {
  // Reject (return nullopt) rather than report a covariance whose implied
  // uncertainty is this poorly conditioned — collinear/near-collinear or
  // otherwise degenerate inlier geometry produces a normal matrix
  // (J^T J) whose condition number blows up long before it's singular.
  double max_condition_number = 1.0e8;
  // Floor on the residual variance sigma^2 used to scale the covariance,
  // so a suspiciously-perfect (near-zero-residual) fit over very few
  // points doesn't report an implausibly tiny/zero uncertainty.
  double residual_variance_floor_m2 = 1.0e-8;
  // Relative-to-largest-singular-value tolerance for treating a singular
  // value as numerically zero (rank/conditioning check).
  double singular_value_tolerance = 1.0e-10;
  // Reject (return nullopt) when the inlier set's RMS residual exceeds
  // this, even though every individual inlier is within
  // RansacParams::inlier_threshold_m and the fit is well-conditioned.
  // Conditioning alone only catches degenerate/collinear geometry; it says
  // nothing about whether the "consensus" RANSAC found is actually a
  // majority of genuine correspondences versus a small, coincidentally
  // self-consistent handful of false matches (temporal_matcher NCC
  // mismatches, more likely on real imagery than on synthetic scenes).
  // Empirically, on a real HoloOcean bag: fits landing on a spurious small
  // consensus had inlier_rmse_m roughly 2-3x the fits landing on a genuine
  // majority (~0.11-0.25m vs ~0.01-0.09m, with RansacParams's default
  // inlier_threshold_m=0.3m as the per-point classification threshold —
  // see docs/uw-slam-production-readiness-and-roadmap-2026-08-21.md 2.4).
  // Defaults to effectively disabled (matches this codebase's convention of
  // opt-in gates that don't retroactively change synthetic-scenario
  // behavior) — see VisualOdometryConfig::max_inlier_rmse_m for where a
  // real value gets wired in.
  double max_inlier_rmse_m = std::numeric_limits<double>::infinity();
};

// Everything a caller needs to both USE the fit and JUDGE how much to
// trust it — RANSAC's raw pose alone previously left every VO evidence
// message claiming the same fixed, arbitrary confidence regardless of how
// well-supported the fit actually was.
struct RigidTransformFitResult {
  uw::sensor_models::Pose3 pose;
  std::size_t correspondence_count = 0;  // total input correspondences (inliers + outliers)
  std::vector<std::size_t> inlier_indices;  // indices into the ORIGINAL a/b, not sorted
  double inlier_ratio = 0.0;   // inlier_indices.size() / correspondence_count
  double inlier_rmse_m = 0.0;  // RMS of ||b[i] - pose.Apply(a[i])|| over inliers only
  double normal_matrix_condition_number = 0.0;  // condition number of J^T J (see CovarianceEstimationParams)
  // 6x6 covariance of pose's minimal [dt(3), dtheta(3)] LEFT perturbation
  // (pose_perturbed = Exp(dtheta) * pose, translation += dt), estimated
  // from a numerical Jacobian of the inlier residuals — see
  // rigid_transform_fit.cpp for the exact perturbation/Jacobian
  // convention. Expressed in the SAME frame as `a`/`b` (i.e. whatever
  // camera-optical convention the caller passed in); callers that need
  // this in a different frame (e.g. VO converting to BODY convention)
  // must transform it themselves via the SE(3) adjoint.
  Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
};

// RANSAC-robustified FitRigidTransform: repeatedly fits a candidate
// transform from a random minimal (3-point) sample of the correspondences,
// scores it by how many of ALL correspondences it explains within
// `inlier_threshold_m`, keeps the best-scoring candidate's inlier set, then
// does one final FitRigidTransform over that full inlier set (not just the
// 3-point sample) for the returned pose — standard RANSAC refinement —
// followed by a numerical-Jacobian covariance estimate over that same
// inlier set (see CovarianceEstimationParams). Falls back to plain
// FitRigidTransform (still followed by the covariance step) when there are
// exactly 3 points (nothing to robustify). Returns std::nullopt if there
// are fewer than 3 points, the best sample's inlier count falls short of
// `min_inliers`, or the covariance estimate fails its conditioning check
// (degenerate/near-collinear inlier geometry) — a conditioning failure is
// treated the same as any other tracking failure by callers (see
// StereoLandmarkVoFrontend::RecordTrackingFailure()), not silently ignored.
//
// Deterministic given the same `rng` state on entry — `rng` must be an
// explicitly seeded, never-reseeded generator threaded in by the caller
// (CLAUDE.md's RNG discipline / the L2 determinism test), not a
// freshly-constructed or global one; StereoLandmarkVoFrontend owns one
// seeded once at construction for exactly this reason.
std::optional<RigidTransformFitResult> FitRigidTransformRansac(
    const std::vector<Eigen::Vector3d>& a, const std::vector<Eigen::Vector3d>& b,
    const RansacParams& params, std::mt19937_64& rng,
    const CovarianceEstimationParams& covariance_params = {});

}  // namespace uw::frontends
