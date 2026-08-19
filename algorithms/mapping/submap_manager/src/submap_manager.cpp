#include "uw/mapping/submap_manager.hpp"

#include <algorithm>
#include <limits>

namespace uw::mapping {

void SubmapManager::AddMapEvidence(uw::domain::MapEvidence evidence) {
  const std::string keyframe_id = evidence.keyframe_id().value();
  keyframes_[keyframe_id].evidence.push_back(std::move(evidence));
}

void SubmapManager::UpdateKeyframePose(const std::string& keyframe_id,
                                        uw::sensor_models::Pose3 new_pose_WB) {
  auto& state = keyframes_[keyframe_id];
  state.pose_WB = std::move(new_pose_WB);
  for (const auto& ev : state.evidence) {
    if (ev.reintegration_policy() == uw::domain::MapEvidence::REINTEGRATION_POLICY_FULL_REFUSE) {
      state.stale = true;
      break;
    }
  }
}

std::vector<uw::domain::MapEvidence> SubmapManager::EvidenceForKeyframe(
    const std::string& keyframe_id) const {
  auto it = keyframes_.find(keyframe_id);
  if (it == keyframes_.end()) return {};
  return it->second.evidence;
}

std::vector<std::string> SubmapManager::StaleKeyframes() const {
  std::vector<std::string> stale;
  for (const auto& [id, state] : keyframes_) {
    if (state.stale) stale.push_back(id);
  }
  return stale;
}

std::vector<Eigen::Vector3d> SubmapManager::WorldPointsForKeyframe(
    const std::string& keyframe_id) const {
  std::vector<Eigen::Vector3d> world_points;
  auto it = keyframes_.find(keyframe_id);
  if (it == keyframes_.end()) return world_points;

  for (const auto& ev : it->second.evidence) {
    if (ev.representation_type() != uw::domain::MAP_REPRESENTATION_POINT_CLOUD) continue;
    const std::string& bytes = ev.geometry_or_occupancy();
    const std::size_t num_floats = bytes.size() / sizeof(float);
    const std::size_t num_points = num_floats / 3;
    const auto* raw = reinterpret_cast<const float*>(bytes.data());
    for (std::size_t p = 0; p < num_points; ++p) {
      const Eigen::Vector3d local(raw[p * 3 + 0], raw[p * 3 + 1], raw[p * 3 + 2]);
      world_points.push_back(it->second.pose_WB.Apply(local));
    }
  }
  return world_points;
}

std::optional<Eigen::Vector3d> SubmapManager::QueryNearestPoint(const Eigen::Vector3d& query_point_W,
                                                                  double max_distance_m) const {
  std::optional<Eigen::Vector3d> best;
  double best_distance = max_distance_m;
  for (const auto& [keyframe_id, state] : keyframes_) {
    (void)state;
    for (const auto& point : WorldPointsForKeyframe(keyframe_id)) {
      const double distance = (point - query_point_W).norm();
      if (distance <= best_distance) {
        best_distance = distance;
        best = point;
      }
    }
  }
  return best;
}

void SubmapManager::RemoveMapEvidence(const std::string& keyframe_id, const std::string& evidence_id) {
  auto it = keyframes_.find(keyframe_id);
  if (it == keyframes_.end()) return;
  auto& evidence = it->second.evidence;
  evidence.erase(std::remove_if(evidence.begin(), evidence.end(),
                                 [&](const uw::domain::MapEvidence& ev) {
                                   return ev.evidence_id().value() == evidence_id;
                                 }),
                 evidence.end());
}

}  // namespace uw::mapping
