#include "uw/evaluation/trajectory_metrics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace uw::evaluation {

AteResult ComputeAte(const std::vector<TrajectoryPose>& estimated,
                     const std::vector<TrajectoryPose>& ground_truth, double max_time_diff_s) {
  AteResult result;
  if (estimated.empty() || ground_truth.empty()) return result;

  double sum_sq = 0.0, sum = 0.0, max_error = 0.0;
  std::size_t matched = 0;

  for (const auto& est : estimated) {
    // Nearest-neighbor match by timestamp (ground truth is small in v1
    // synthetic scenarios; linear scan is fine).
    double best_diff = std::numeric_limits<double>::infinity();
    const TrajectoryPose* best = nullptr;
    for (const auto& gt : ground_truth) {
      const double diff = std::abs(gt.timestamp_s - est.timestamp_s);
      if (diff < best_diff) {
        best_diff = diff;
        best = &gt;
      }
    }
    if (best == nullptr || best_diff > max_time_diff_s) continue;

    const double error = (est.pose_WB.translation - best->pose_WB.translation).norm();
    sum += error;
    sum_sq += error * error;
    max_error = std::max(max_error, error);
    ++matched;
  }

  result.num_matched_poses = matched;
  if (matched > 0) {
    result.mean_m = sum / static_cast<double>(matched);
    result.rmse_m = std::sqrt(sum_sq / static_cast<double>(matched));
    result.max_m = max_error;
  }
  return result;
}

}  // namespace uw::evaluation
