#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "domain/domain.hpp"
#include "sensor_models/geometry.hpp"

namespace uw::mapping {

// Per platform architecture section 7.8/9/21 invariant #3 ("Mapping 拥有
// 局部地图证据，不拥有另一套轨迹") and #9 ("后端修正必须能传播到 submap 和
// presentation output"): MapEvidence is kept in its LOCAL frame with a
// reference to source observations, never baked into a global pose at
// insertion time. This is the deliberate counter-pattern to
// sonar_camera_reconstruction's merge.py (see NOTICE) — world-frame points
// are only ever computed on demand from the keyframe's CURRENT known pose,
// so a pose-graph correction just changes what WorldPointsForKeyframe
// returns, with no re-running of the frontend.
class SubmapManager {
 public:
  void AddMapEvidence(uw::domain::MapEvidence evidence);

  // Called whenever a keyframe's pose changes (e.g. after pose graph
  // optimization). REINTEGRATION_POLICY_TRANSFORM_ONLY evidence is
  // unaffected by this (WorldPointsForKeyframe already re-applies the
  // latest pose on every call); REINTEGRATION_POLICY_FULL_REFUSE evidence
  // is marked stale so a caller knows it must regenerate from
  // source_observations rather than trust a cached result.
  void UpdateKeyframePose(const std::string& keyframe_id, uw::sensor_models::Pose3 new_pose_WB);

  std::vector<uw::domain::MapEvidence> EvidenceForKeyframe(const std::string& keyframe_id) const;
  std::vector<std::string> StaleKeyframes() const;

  // Decodes MAP_REPRESENTATION_POINT_CLOUD evidence and transforms it into
  // world frame using the keyframe's CURRENT pose. Other representation
  // types return empty (not implemented in v1).
  std::vector<Eigen::Vector3d> WorldPointsForKeyframe(const std::string& keyframe_id) const;

  // v1 data-association gate for factor_builders that need "known nearby
  // points" (sonar_range_factor's documented extension point — see its
  // header comment): nearest world-frame point across EVERY keyframe's
  // evidence within max_distance_m of query_point_W, or nullopt if none is
  // within the gate. Straight Euclidean distance over whatever points are
  // currently stored — no covariance-aware/Mahalanobis gating, and no
  // joint landmark refinement (a caller that gets a hit is expected to use
  // the returned position as-is, not treat it as something this class will
  // later improve).
  std::optional<Eigen::Vector3d> QueryNearestPoint(const Eigen::Vector3d& query_point_W,
                                                    double max_distance_m) const;

  // Removes one evidence entry (e.g. one landmark inserted as its own
  // single-point MapEvidence) by (keyframe_id, evidence_id). No-op if
  // nothing matches — callers that track their own landmark ids can use
  // this to prune a landmark that turned out to be spurious or has aged
  // out, without needing this class to guess a staleness policy itself.
  void RemoveMapEvidence(const std::string& keyframe_id, const std::string& evidence_id);

 private:
  struct KeyframeMapState {
    uw::sensor_models::Pose3 pose_WB;
    std::vector<uw::domain::MapEvidence> evidence;
    bool stale = false;
  };
  std::unordered_map<std::string, KeyframeMapState> keyframes_;
};

}  // namespace uw::mapping
