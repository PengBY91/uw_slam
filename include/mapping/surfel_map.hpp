// Lightweight surfel map backend for incrementally fusing point evidence.
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "sensor_models/geometry.hpp"

namespace uw::mapping {

// A single surfel: an oriented disk approximating a small patch of surface.
// v1 fields only: no radius growth, confidence decay/cap, or pruning.
struct Surfel {
  Eigen::Vector3d position_W = Eigen::Vector3d::Zero();
  // Zero() means "unknown" — a single fused point carries no normal
  // information on its own (see AddPoint). Only AddPointWithNormal, or a
  // merge where at least one contributor already had a normal, sets this.
  Eigen::Vector3d normal_W = Eigen::Vector3d::Zero();
  double radius_m = 0.0;
  // Accumulated merge weight — higher means more (and/or more confident)
  // observations support this surfel. Intended convention: 1/variance_m2,
  // so it composes directly with
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

  // P3 roadmap item 3 ("uncertainty-aware 融合...异常点抑制"), outlier
  // suppression half: a candidate within merge_distance_m is still only
  // merged if it also passes a sigma-multiple gate on the COMBINED standard
  // deviation of the existing surfel and the new observation — same
  // convention/default as this codebase's other sigma gates
  // (AcousticOpticAssociatorParams::depth_agreement_sigma,
  // AcousticOpticDepthFusionParams::innovation_gate_sigma, both 3.0). A
  // candidate that fails this gate is NOT merged — it starts a new,
  // separate surfel instead (see SurfelMap::FuseWorldPoint), so
  // disagreeing-but-nearby evidence is preserved as two competing
  // hypotheses rather than forced into one average that could be wrong.
  double outlier_gate_sigma = 3.0;

  // P3 roadmap item 3, free-space/occlusion half: SurfelMap::CarveFreeSpace
  // treats any existing surfel within this perpendicular distance of a
  // sensor-to-observation ray (and strictly closer to the sensor than the
  // observation itself) as contradicted by that observation. Defaults to
  // merge_distance_m's value (same physical scale — "close enough to be the
  // same point" and "close enough to the ray to be considered occluding it"
  // are treated as the same tolerance unless a caller has a reason to
  // differ) but is independently configurable.
  double free_space_corridor_radius_m = merge_distance_m;
  // Each carve event multiplies a contradicted surfel's confidence by this
  // factor (not an outright removal on the first contradiction — a single
  // ray is itself noisy evidence, matching this class's general preference
  // for accumulating evidence over multiple observations rather than acting
  // on one). Falls below free_space_removal_confidence_threshold after
  // enough repeated contradictions and the surfel is then removed.
  double free_space_confidence_decay = 0.5;
  double free_space_removal_confidence_threshold = 0.01;
};

// Hand-rolled surfel map with no external dependency beyond Eigen.
//
// v1 scale limitation, documented rather than hidden (same pattern as
// other v1 notes in this codebase, e.g. ComputeMapMetrics's brute-force
// nearest-neighbor search): AddPoint does an O(existing surfels) linear
// scan for the nearest surfel within merge_distance_m. Fine for a PoC and
// for the point counts this is tested against; NOT fine for
// apps/replay_demo's real map evidence volume (millions of points per
// run) — a spatial index (voxel hash, KD-tree) is required before this
// scales.
class SurfelMap {
 public:
  explicit SurfelMap(SurfelMapParams params = {});

  // Inserts one observed world-frame point with a confidence weight
  // (higher = more trusted; see Surfel::confidence's doc comment for the
  // intended 1/variance_m2 convention). If an existing surfel's position is
  // within merge_distance_m, the point is confidence-weighted-averaged
  // into it (both position and confidence update, normal_W is left
  // untouched since a bare point carries none); otherwise a new surfel is
  // created with normal_W == Zero() (unknown). Not attributed to any real
  // keyframe_id (see AddKeyframeObservation below), so ReintegrateKeyframe
  // can never target it directly — but it IS still correctly replayed by a
  // rebuild triggered by reintegrating some OTHER keyframe (implementation
  // detail: internally recorded under a reserved, never-real keyframe_id —
  // see surfel_map.cpp — specifically so a rebuild's surfels_.clear() does
  // not silently lose it).
  void AddPoint(const Eigen::Vector3d& point_W, double confidence);

  // Same as AddPoint, but the caller already knows (or estimates) a
  // surface normal for this observation — e.g. from a locally-fit plane.
  // Merged into an existing surfel's normal via the same confidence
  // weighting (renormalized after averaging); a brand-new surfel is
  // created with this normal directly.
  void AddPointWithNormal(const Eigen::Vector3d& point_W, const Eigen::Vector3d& normal_W, double confidence);

  // P3 roadmap item 4 ("pose correction 后的 submap transform/reintegration
  // 与 stale evidence 管理"), applied to the surfel backend: unlike
  // AddPoint/AddPointWithNormal above (world-frame, one-shot, no record of
  // which keyframe an observation came from), these record one observation
  // in `keyframe_id`'s LOCAL frame — the same convention
  // SubmapManager/MapEvidence already use (local_to_world.Apply(point_local)
  // gives the world point; camera extrinsics are expected already baked
  // into `point_local`/`normal_local`, so `local_to_world` is exactly a
  // keyframe's pose_WB, nothing else). The observation is fused into
  // `surfels_` immediately at `local_to_world`'s current value (so a caller
  // that never corrects a pose sees identical behavior/cost to AddPoint),
  // AND retained so ReintegrateKeyframe can later redo the fusion at a
  // corrected pose. The first Add*Observation call for a given
  // `keyframe_id` establishes that keyframe's tracked pose; a later call
  // with a DIFFERENT `local_to_world` for the same `keyframe_id` updates
  // the tracked pose for future rebuilds but does NOT itself retroactively
  // refuse earlier observations from that keyframe — call
  // ReintegrateKeyframe for that (this mirrors calling AddMapEvidence vs.
  // UpdateKeyframePose as two distinct steps in SubmapManager).
  void AddKeyframeObservation(const std::string& keyframe_id, const Eigen::Vector3d& point_local,
                               double confidence, const uw::sensor_models::Pose3& local_to_world);
  void AddKeyframeObservationWithNormal(const std::string& keyframe_id, const Eigen::Vector3d& point_local,
                                         const Eigen::Vector3d& normal_local, double confidence,
                                         const uw::sensor_models::Pose3& local_to_world);

  // Pose-graph correction entry point: updates `keyframe_id`'s tracked pose
  // to `new_local_to_world` and re-fuses EVERY tracked keyframe's recorded
  // observations from scratch against a cleared `surfels_`. This is an
  // EXACT recompute, not an incremental retract-and-redo of the running
  // weighted average `MergeInto` maintains — deliberately: MergeInto's
  // normal fusion only keeps the current normalized `normal_W`, not the
  // pre-normalization weighted sum, so there is no lossless way to
  // "subtract" one keyframe's past contribution back out of an already-
  // merged surfel without re-deriving it from source, and re-deriving from
  // source is exactly what a full rebuild already does, more simply and
  // without accumulating float round-trip error across repeated
  // retract/redo cycles. Cost: O(total observations across every tracked
  // keyframe) per call, not O(1) amortized like AddPoint — acceptable for
  // the same reason SurfelMap's existing O(existing surfels) linear scan
  // is (see this class's own scale-limitation note): this backend is not
  // yet wired into a real pipeline's per-frame hot path, so exactness beats
  // incrementality until it is. Points/normals added via the plain
  // AddPoint/AddPointWithNormal path (no keyframe attribution) are left
  // untouched by a rebuild — they were never eligible for reintegration in
  // the first place, and rebuilding clears+refuses only the tracked
  // observations, not those.
  //
  // No-op if `keyframe_id` has no recorded observations (nothing to redo).
  //
  // Deliberately NOT paired with a StaleKeyframes()-style query: this
  // repo's own point-cloud path (SubmapManager::StaleKeyframes()) already
  // has one, and nothing in this codebase currently calls it — a
  // detect-and-flag mechanism nobody consumes doesn't actually solve
  // anything. ReintegrateKeyframe is instead the atomic "the correction
  // landed, redo the fusion now" operation: the caller that owns the
  // pose-graph correction event calls it directly, and correctness holds
  // by construction rather than by hoping something later polls a flag.
  void ReintegrateKeyframe(const std::string& keyframe_id, const uw::sensor_models::Pose3& new_local_to_world);

  // Number of distinct keyframe_ids with at least one recorded observation
  // (i.e. that ReintegrateKeyframe would actually do something for) —
  // diagnostic/test use, not itself part of the reintegration contract.
  // Does NOT count AddPoint/AddPointWithNormal's internal bookkeeping (see
  // surfel_map.cpp) — those were never given a real keyframe_id, so calling
  // ReintegrateKeyframe couldn't target them regardless of what this
  // returned.
  std::size_t NumTrackedKeyframes() const;

  // P3 roadmap item 3, free-space/occlusion half: `ray_origin_W` is the
  // sensor's world position at observation time, `ray_end_W` is the
  // observed point itself (the same point just passed to Add*). Any
  // existing surfel within `free_space_corridor_radius_m` of the segment
  // [ray_origin_W, ray_end_W] — and strictly nearer ray_origin_W than
  // ray_end_W is, i.e. genuinely BETWEEN the sensor and the new
  // observation, not at or beyond it — is contradicted: the sensor's own
  // ray just passed through that space unobstructed, so nothing can
  // legitimately occupy it. Each contradicted surfel's confidence is
  // multiplied by free_space_confidence_decay; a surfel whose confidence
  // drops at/below free_space_removal_confidence_threshold is removed.
  // Explicitly safe to call for the SAME pixel/point that was just fused:
  // any surfel within free_space_corridor_radius_m of ray_end_W ITSELF
  // (checked directly against the fixed endpoint, not derived from the
  // ray-parameter projection below) is treated as "the thing being
  // observed," never as something occluded — this is what stops a surfel
  // from carving out the very observation that just created or reinforced
  // it, including when a confidence-weighted merge nudged its position a
  // hair short of the pixel's exact unprojected point.
  // Deliberately scoped to camera/optical-depth ray geometry only (see
  // AcousticOpticMapBridgeParams's FuseDepthIntoSurfels, this method's
  // caller) — this repo's separate sparse sonar-landmark path
  // (SubmapManager::QueryNearestPoint, driven from
  // src/application/replay_pipeline.cpp's sonar block) does not feed
  // SurfelMap at all today, so extending carving to sonar ray geometry has
  // no existing integration point to extend; out of scope here, not
  // silently forgotten.
  //
  // NOT called automatically by Add*/AddKeyframeObservation* above — the
  // caller must invoke it explicitly once it knows the ray's sensor
  // origin, since none of the Add* entry points take one (adding an
  // optional origin parameter to all of them would have widened an
  // already-tested API for a capability most callers don't need; this
  // repo's only real caller, FuseDepthIntoSurfels, already computes a
  // camera world position it can pass here directly). O(existing surfels)
  // per call, same brute-force posture as FindNearest (see this class's
  // own scale-limitation note above) — not attempted to be cheaper.
  //
  // Returns the number of surfels removed by this call (surfels merely
  // downweighted, not removed, are not counted — diagnostic/test use).
  //
  // Interaction with ReintegrateKeyframe, real and deliberately scoped, not
  // an oversight: carving is NOT recorded in the per-keyframe observation
  // ledger (KeyframeRecord) — it only mutates surfels_ in place. A later
  // ReintegrateKeyframe call rebuilds surfels_ purely by replaying that
  // ledger's positive observations from scratch (RebuildFromKeyframeRecords),
  // so any confidence decay or removal a prior CarveFreeSpace call caused is
  // NOT preserved across a rebuild — carving is a live-view effect on the
  // current surfels_, not persistent negative evidence. Threading negative/
  // carving evidence through the ledger too (so it survives reintegration)
  // is a real extension but a bigger one — this repo's ledger has no
  // negative-evidence concept at all today — and is left for whoever next
  // needs reintegration-durable free-space evidence, not attempted here.
  int CarveFreeSpace(const Eigen::Vector3d& ray_origin_W, const Eigen::Vector3d& ray_end_W);

  // Number of merge candidates rejected by the outlier_gate_sigma check
  // (SurfelMapParams) since construction — diagnostic/test use, not part of
  // the fusion contract itself. Each one still became its own new surfel
  // (see FuseWorldPoint), not a discarded observation.
  uint64_t NumOutliersRejected() const { return outliers_rejected_; }

  const std::vector<Surfel>& Surfels() const { return surfels_; }
  std::size_t NumSurfels() const { return surfels_.size(); }

 private:
  struct KeyframeObservation {
    Eigen::Vector3d point_local;
    std::optional<Eigen::Vector3d> normal_local;
    double confidence;
  };
  struct KeyframeRecord {
    uw::sensor_models::Pose3 pose_WB;
    std::vector<KeyframeObservation> observations;
  };

  Surfel* FindNearest(const Eigen::Vector3d& point_W);
  void MergeInto(Surfel& target, const Eigen::Vector3d& point_W, const Eigen::Vector3d* normal_W,
                 double confidence);
  // Shared by AddPoint/AddPointWithNormal and the keyframe-attributed path
  // (both the immediate fuse-on-insert and RebuildFromKeyframeRecords) so
  // there is exactly one merge-or-create code path, not three copies of it.
  // A FindNearest candidate that fails the outlier_gate_sigma check (see
  // SurfelMapParams) is treated the same as FindNearest finding nothing —
  // falls through to creating a new surfel — rather than retrying against a
  // second-nearest candidate; matches this class's existing greedy,
  // not-fully-rigorous-data-association posture (same spirit as the sonar
  // range factor's plain Euclidean landmark gate elsewhere in this repo).
  void FuseWorldPoint(const Eigen::Vector3d& point_W, const Eigen::Vector3d* normal_W, double confidence);
  void RebuildFromKeyframeRecords();

  SurfelMapParams params_;
  std::vector<Surfel> surfels_;
  std::unordered_map<std::string, KeyframeRecord> keyframe_records_;
  uint64_t outliers_rejected_ = 0;
};

}  // namespace uw::mapping
