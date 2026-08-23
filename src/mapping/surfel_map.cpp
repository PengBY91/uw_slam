#include "mapping/surfel_map.hpp"

#include <limits>

namespace uw::mapping {

namespace {

// Reserved keyframe_id (never a real one — this repo's keyframe ids are all
// non-empty, e.g. "kf0") under which AddPoint/AddPointWithNormal record
// their observations too, at an identity local_to_world (point_local ==
// point_W, since there is no real per-keyframe transform for them). This is
// what lets RebuildFromKeyframeRecords() (triggered by ReintegrateKeyframe
// on some OTHER, real keyframe_id) replay plain Add* points correctly
// instead of silently dropping them — without this, a rebuild's
// surfels_.clear() would destroy any surfel that had no keyframe
// attribution at all, which is not what this class's public contract
// promises (see ReintegrateKeyframe's doc comment in the header).
constexpr const char* kUnattributedKeyframeId = "";

}  // namespace

SurfelMap::SurfelMap(SurfelMapParams params) : params_(params) {}

Surfel* SurfelMap::FindNearest(const Eigen::Vector3d& point_W) {
  Surfel* best = nullptr;
  double best_distance = params_.merge_distance_m;
  for (auto& surfel : surfels_) {
    const double distance = (surfel.position_W - point_W).norm();
    if (distance <= best_distance) {
      best_distance = distance;
      best = &surfel;
    }
  }
  return best;
}

void SurfelMap::MergeInto(Surfel& target, const Eigen::Vector3d& point_W, const Eigen::Vector3d* normal_W,
                           double confidence) {
  const double total_confidence = target.confidence + confidence;
  target.position_W = (target.position_W * target.confidence + point_W * confidence) / total_confidence;
  if (normal_W != nullptr) {
    const Eigen::Vector3d combined_normal = target.normal_W * target.confidence + (*normal_W) * confidence;
    const double norm = combined_normal.norm();
    target.normal_W = norm > std::numeric_limits<double>::epsilon() ? (combined_normal / norm).eval()
                                                                     : Eigen::Vector3d::Zero();
  }
  target.confidence = total_confidence;
}

void SurfelMap::FuseWorldPoint(const Eigen::Vector3d& point_W, const Eigen::Vector3d* normal_W,
                                double confidence) {
  if (Surfel* existing = FindNearest(point_W)) {
    // Outlier gate: a candidate within merge_distance_m still only merges
    // if it also agrees with the existing surfel within a sigma-multiple of
    // their COMBINED standard deviation — same squared-distance-vs-
    // combined-variance formula as this codebase's other sigma gates (e.g.
    // acoustic_optic_associator.cpp's depth_agreement_sigma check), just in
    // 3D instead of scalar depth. confidence == 1/variance_m2 by this
    // class's documented convention, so 1/confidence recovers each side's
    // variance.
    const double existing_variance = existing->confidence > 0.0 ? 1.0 / existing->confidence : 0.0;
    const double new_variance = confidence > 0.0 ? 1.0 / confidence : 0.0;
    const double combined_variance = existing_variance + new_variance;
    const double distance_sq = (existing->position_W - point_W).squaredNorm();
    const double gate_sq =
        combined_variance * params_.outlier_gate_sigma * params_.outlier_gate_sigma;
    const bool agrees = combined_variance > 0.0 ? distance_sq <= gate_sq : distance_sq == 0.0;
    if (agrees) {
      MergeInto(*existing, point_W, normal_W, confidence);
      return;
    }
    ++outliers_rejected_;
    // Falls through to create a new, separate surfel below — see this
    // method's doc comment in the header for why (preserve both
    // hypotheses rather than force a merge that could be wrong).
  }
  Surfel surfel;
  surfel.position_W = point_W;
  if (normal_W != nullptr) surfel.normal_W = *normal_W;
  surfel.confidence = confidence;
  surfels_.push_back(surfel);
}

int SurfelMap::CarveFreeSpace(const Eigen::Vector3d& ray_origin_W, const Eigen::Vector3d& ray_end_W) {
  const Eigen::Vector3d ray = ray_end_W - ray_origin_W;
  const double ray_length_sq = ray.squaredNorm();
  if (ray_length_sq <= std::numeric_limits<double>::epsilon()) return 0;  // degenerate (coincident) ray

  const double corridor_radius_sq = params_.free_space_corridor_radius_m * params_.free_space_corridor_radius_m;
  int removed = 0;
  for (std::size_t i = 0; i < surfels_.size();) {
    // Guard against carving the very observation this ray just delivered:
    // a surfel within corridor radius of the ray's OWN endpoint is "the
    // same point being observed," never "something the ray passed
    // through" — checked directly against ray_end_W (a fixed point), not
    // via the t-parameter below, so it can't be defeated by t landing at
    // 0.999999... instead of exactly 1.0 after a confidence-weighted merge
    // nudged the surfel's position slightly short of the pixel's own exact
    // unprojected point (see this method's header doc comment for why this
    // matters: FuseDepthIntoSurfels calls Add*/AddKeyframeObservation* for
    // this same pixel immediately before calling CarveFreeSpace).
    if ((surfels_[i].position_W - ray_end_W).squaredNorm() <= corridor_radius_sq) {
      ++i;
      continue;
    }
    const Eigen::Vector3d to_surfel = surfels_[i].position_W - ray_origin_W;
    // Project onto the ray, parameterized t in [0,1] from origin to end.
    // t < 0 (behind the sensor) or t > 1 (at/beyond the observed point
    // itself) is not "confirmed free space" — only the segment strictly
    // between the sensor and the new observation is.
    const double t = to_surfel.dot(ray) / ray_length_sq;
    if (t <= 0.0 || t >= 1.0) {
      ++i;
      continue;
    }
    const Eigen::Vector3d closest_point_on_ray = ray_origin_W + t * ray;
    const double perpendicular_distance_sq = (surfels_[i].position_W - closest_point_on_ray).squaredNorm();
    if (perpendicular_distance_sq > corridor_radius_sq) {
      ++i;
      continue;
    }
    surfels_[i].confidence *= params_.free_space_confidence_decay;
    if (surfels_[i].confidence <= params_.free_space_removal_confidence_threshold) {
      surfels_[i] = surfels_.back();
      surfels_.pop_back();
      ++removed;
      // Do not advance i: the swapped-in element at this index needs its
      // own check.
    } else {
      ++i;
    }
  }
  return removed;
}

void SurfelMap::AddPoint(const Eigen::Vector3d& point_W, double confidence) {
  auto& record = keyframe_records_[kUnattributedKeyframeId];
  record.pose_WB = uw::sensor_models::Pose3::Identity();
  record.observations.push_back(KeyframeObservation{point_W, std::nullopt, confidence});
  FuseWorldPoint(point_W, /*normal_W=*/nullptr, confidence);
}

void SurfelMap::AddPointWithNormal(const Eigen::Vector3d& point_W, const Eigen::Vector3d& normal_W,
                                    double confidence) {
  auto& record = keyframe_records_[kUnattributedKeyframeId];
  record.pose_WB = uw::sensor_models::Pose3::Identity();
  record.observations.push_back(KeyframeObservation{point_W, normal_W, confidence});
  FuseWorldPoint(point_W, &normal_W, confidence);
}

void SurfelMap::AddKeyframeObservation(const std::string& keyframe_id, const Eigen::Vector3d& point_local,
                                        double confidence, const uw::sensor_models::Pose3& local_to_world) {
  auto& record = keyframe_records_[keyframe_id];
  record.pose_WB = local_to_world;
  record.observations.push_back(KeyframeObservation{point_local, std::nullopt, confidence});
  FuseWorldPoint(local_to_world.Apply(point_local), /*normal_W=*/nullptr, confidence);
}

void SurfelMap::AddKeyframeObservationWithNormal(const std::string& keyframe_id,
                                                  const Eigen::Vector3d& point_local,
                                                  const Eigen::Vector3d& normal_local, double confidence,
                                                  const uw::sensor_models::Pose3& local_to_world) {
  auto& record = keyframe_records_[keyframe_id];
  record.pose_WB = local_to_world;
  record.observations.push_back(KeyframeObservation{point_local, normal_local, confidence});
  // Direction-only transform (no translation) — a normal is a direction,
  // not a point (same reasoning as acoustic_optic_map_bridge.cpp's
  // optical_to_world_rotation).
  const Eigen::Vector3d normal_world = local_to_world.rotation * normal_local;
  FuseWorldPoint(local_to_world.Apply(point_local), &normal_world, confidence);
}

void SurfelMap::ReintegrateKeyframe(const std::string& keyframe_id,
                                     const uw::sensor_models::Pose3& new_local_to_world) {
  auto it = keyframe_records_.find(keyframe_id);
  if (it == keyframe_records_.end()) return;  // nothing recorded for this keyframe, nothing to redo
  it->second.pose_WB = new_local_to_world;
  RebuildFromKeyframeRecords();
}

std::size_t SurfelMap::NumTrackedKeyframes() const {
  return keyframe_records_.count(kUnattributedKeyframeId) > 0 ? keyframe_records_.size() - 1
                                                               : keyframe_records_.size();
}

void SurfelMap::RebuildFromKeyframeRecords() {
  surfels_.clear();
  for (const auto& entry : keyframe_records_) {
    const KeyframeRecord& record = entry.second;
    for (const auto& observation : record.observations) {
      const Eigen::Vector3d point_world = record.pose_WB.Apply(observation.point_local);
      if (observation.normal_local.has_value()) {
        const Eigen::Vector3d normal_world = record.pose_WB.rotation * (*observation.normal_local);
        FuseWorldPoint(point_world, &normal_world, observation.confidence);
      } else {
        FuseWorldPoint(point_world, /*normal_W=*/nullptr, observation.confidence);
      }
    }
  }
}

}  // namespace uw::mapping
