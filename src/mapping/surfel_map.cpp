#include "mapping/surfel_map.hpp"

#include <limits>

namespace uw::mapping {

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

void SurfelMap::AddPoint(const Eigen::Vector3d& point_W, double confidence) {
  if (Surfel* existing = FindNearest(point_W)) {
    MergeInto(*existing, point_W, /*normal_W=*/nullptr, confidence);
    return;
  }
  Surfel surfel;
  surfel.position_W = point_W;
  surfel.confidence = confidence;
  surfels_.push_back(surfel);
}

void SurfelMap::AddPointWithNormal(const Eigen::Vector3d& point_W, const Eigen::Vector3d& normal_W,
                                    double confidence) {
  if (Surfel* existing = FindNearest(point_W)) {
    MergeInto(*existing, point_W, &normal_W, confidence);
    return;
  }
  Surfel surfel;
  surfel.position_W = point_W;
  surfel.normal_W = normal_W;
  surfel.confidence = confidence;
  surfels_.push_back(surfel);
}

}  // namespace uw::mapping
