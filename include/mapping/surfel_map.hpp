// P3 workstream D8 (docs/superpowers/plans/2026-08-22-p3-online-
// reconstruction-and-productionization.md): this repo's first real
// map-geometry backend. See docs/superpowers/plans/2026-08-22-p3-d8-
// geometry-backend-decision.md for why surfel was chosen over TSDF/
// occupancy, and what scope this PoC deliberately does NOT cover.
#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>

namespace uw::mapping {

// A single surfel: an oriented disk approximating a small patch of surface.
// v1 fields only: no radius growth, no confidence decay/cap, no pruning —
// see the decision doc for what's left to D9/D10.
struct Surfel {
  Eigen::Vector3d position_W = Eigen::Vector3d::Zero();
  // Zero() means "unknown" — a single fused point carries no normal
  // information on its own (see AddPoint). Only AddPointWithNormal, or a
  // merge where at least one contributor already had a normal, sets this.
  Eigen::Vector3d normal_W = Eigen::Vector3d::Zero();
  double radius_m = 0.0;
  // Accumulated merge weight — higher means more (and/or more confident)
  // observations support this surfel. Intended convention (see the
  // decision doc): 1/variance_m2, so it composes directly with
  // MapEvidence.uncertainty (already populated as variance_m2 by
  // acoustic_optic_map_bridge.cpp) without any protocol change — but
  // SurfelMap itself is agnostic to what "confidence" means, it only
  // requires "higher = more trusted."
  double confidence = 0.0;
};

struct SurfelMapParams {
  // Two points/surfels merge if within this distance of each other — the
  // brute-force stand-in for a real spatial index's association gate (see
  // this file's own scale-limitation note below).
  double merge_distance_m = 0.05;
};

// Hand-rolled surfel map, no external dependency beyond Eigen (already a
// repo-wide dependency) — see the decision doc for why hand-rolled instead
// of a library (Open3D/voxblox et al. were considered and rejected on
// dependency-footprint grounds, consistent with this repo's existing
// posture toward the hand-rolled Gauss-Newton solver).
//
// v1 scale limitation, documented rather than hidden (same pattern as
// other v1 notes in this codebase, e.g. ComputeMapMetrics's brute-force
// nearest-neighbor search): AddPoint does an O(existing surfels) linear
// scan for the nearest surfel within merge_distance_m. Fine for a PoC and
// for the point counts this is tested against; NOT fine for
// apps/replay_demo's real map evidence volume (millions of points per
// run) — a spatial index (voxel hash, KD-tree) is required before this
// scales, and is explicitly left to a follow-up workstream, not attempted
// here.
class SurfelMap {
 public:
  explicit SurfelMap(SurfelMapParams params = {});

  // Inserts one observed world-frame point with a confidence weight
  // (higher = more trusted; see Surfel::confidence's doc comment for the
  // intended 1/variance_m2 convention). If an existing surfel's position is
  // within merge_distance_m, the point is confidence-weighted-averaged
  // into it (both position and confidence update, normal_W is left
  // untouched since a bare point carries none); otherwise a new surfel is
  // created with normal_W == Zero() (unknown).
  void AddPoint(const Eigen::Vector3d& point_W, double confidence);

  // Same as AddPoint, but the caller already knows (or estimates) a
  // surface normal for this observation — e.g. from a locally-fit plane.
  // Merged into an existing surfel's normal via the same confidence
  // weighting (renormalized after averaging); a brand-new surfel is
  // created with this normal directly.
  void AddPointWithNormal(const Eigen::Vector3d& point_W, const Eigen::Vector3d& normal_W, double confidence);

  const std::vector<Surfel>& Surfels() const { return surfels_; }
  std::size_t NumSurfels() const { return surfels_.size(); }

 private:
  Surfel* FindNearest(const Eigen::Vector3d& point_W);
  void MergeInto(Surfel& target, const Eigen::Vector3d& point_W, const Eigen::Vector3d* normal_W,
                 double confidence);

  SurfelMapParams params_;
  std::vector<Surfel> surfels_;
};

}  // namespace uw::mapping
