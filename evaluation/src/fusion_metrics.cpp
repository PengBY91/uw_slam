#include "uw/evaluation/fusion_metrics.hpp"

#include <algorithm>
#include <cmath>

namespace uw::evaluation {

FalseFusionResult EvaluateFalseFusion(double estimated_depth_m, double gt_depth_m) {
  FalseFusionResult result;
  result.abs_error_m = std::abs(estimated_depth_m - gt_depth_m);
  result.threshold_m = std::max(0.05, 0.03 * gt_depth_m);
  result.is_false_fusion = result.abs_error_m > result.threshold_m;
  return result;
}

}  // namespace uw::evaluation
