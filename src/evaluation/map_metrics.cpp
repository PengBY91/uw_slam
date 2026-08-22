#include "evaluation/map_metrics.hpp"

#include <algorithm>
#include <limits>

namespace uw::evaluation {

namespace {

struct DirectionalStats {
  double mean_distance_m = 0.0;
  std::size_t within_threshold_count = 0;
};

// Brute-force: for each point in `from`, the distance to its nearest
// neighbor in `to` — see map_metrics.hpp's doc comment for the v1 scale
// limitation. Caller guarantees `to` is non-empty.
DirectionalStats ComputeDirectional(const std::vector<Eigen::Vector3d>& from,
                                     const std::vector<Eigen::Vector3d>& to, double threshold_m) {
  DirectionalStats stats;
  double sum = 0.0;
  for (const auto& p : from) {
    double best = std::numeric_limits<double>::max();
    for (const auto& q : to) best = std::min(best, (p - q).norm());
    sum += best;
    if (best <= threshold_m) ++stats.within_threshold_count;
  }
  stats.mean_distance_m = from.empty() ? 0.0 : sum / static_cast<double>(from.size());
  return stats;
}

}  // namespace

MapMetricsResult ComputeMapMetrics(const std::vector<Eigen::Vector3d>& estimated,
                                    const std::vector<Eigen::Vector3d>& reference,
                                    double distance_threshold_m) {
  MapMetricsResult result;
  result.num_estimated_points = estimated.size();
  result.num_reference_points = reference.size();

  if (!estimated.empty()) {
    if (reference.empty()) {
      result.outlier_ratio = 1.0;  // nothing to confirm any estimated point against
    } else {
      const auto stats = ComputeDirectional(estimated, reference, distance_threshold_m);
      result.mean_estimated_to_reference_m = stats.mean_distance_m;
      result.outlier_ratio =
          1.0 - static_cast<double>(stats.within_threshold_count) / static_cast<double>(estimated.size());
    }
  }

  if (!reference.empty() && !estimated.empty()) {
    const auto stats = ComputeDirectional(reference, estimated, distance_threshold_m);
    result.mean_reference_to_estimated_m = stats.mean_distance_m;
    result.completeness = static_cast<double>(stats.within_threshold_count) / static_cast<double>(reference.size());
  }

  result.chamfer_distance_m = result.mean_estimated_to_reference_m + result.mean_reference_to_estimated_m;

  const double precision = 1.0 - result.outlier_ratio;
  const double recall = result.completeness;
  if (precision + recall > 0.0) {
    result.f_score = 2.0 * precision * recall / (precision + recall);
  }
  return result;
}

}  // namespace uw::evaluation
