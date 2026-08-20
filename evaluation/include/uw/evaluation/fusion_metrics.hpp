#pragma once

namespace uw::evaluation {

struct FalseFusionResult {
  bool is_false_fusion = false;
  double abs_error_m = 0.0;
  double threshold_m = 0.0;
};

// Design spec section 12.1: "false fusion" = an accepted update whose
// absolute depth error exceeds max(0.05m, 0.03 * GT depth).
FalseFusionResult EvaluateFalseFusion(double estimated_depth_m, double gt_depth_m);

}  // namespace uw::evaluation
