#include "uw/evaluation/depth_metrics.hpp"

#include <cmath>

namespace uw::evaluation {

DepthMetricsResult ComputeDepthMetrics(const DepthGrid& estimated, const DepthGrid& ground_truth) {
  DepthMetricsResult result;
  const std::size_t pixels = static_cast<std::size_t>(ground_truth.width) * ground_truth.height;

  std::size_t gt_valid_count = 0;
  std::size_t both_valid_count = 0;
  double sum_abs_error = 0.0;
  double sum_sq_error = 0.0;

  for (std::size_t i = 0; i < pixels && i < ground_truth.valid_mask.size(); ++i) {
    if (ground_truth.valid_mask[i] == 0) continue;
    ++gt_valid_count;
    if (i >= estimated.valid_mask.size() || estimated.valid_mask[i] == 0) continue;
    ++both_valid_count;
    const double error = static_cast<double>(estimated.depth_m[i]) - static_cast<double>(ground_truth.depth_m[i]);
    sum_abs_error += std::abs(error);
    sum_sq_error += error * error;
  }

  result.num_compared_pixels = both_valid_count;
  if (both_valid_count > 0) {
    result.mae_m = sum_abs_error / static_cast<double>(both_valid_count);
    result.rmse_m = std::sqrt(sum_sq_error / static_cast<double>(both_valid_count));
  }
  result.valid_coverage_fraction =
      gt_valid_count > 0 ? static_cast<double>(both_valid_count) / static_cast<double>(gt_valid_count) : 0.0;
  return result;
}

}  // namespace uw::evaluation
