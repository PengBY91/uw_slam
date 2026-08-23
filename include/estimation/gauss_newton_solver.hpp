#pragma once

#include <string>
#include <unordered_map>

#include <Eigen/Dense>

#include "estimation/pose_graph_problem.hpp"

namespace uw::estimation {

// Declared at namespace scope (not nested in GaussNewtonSolver) so that
// `const GaussNewtonOptions& = {}` as a member-function default argument
// doesn't hit a GCC ordering quirk where a nested class's own default
// member initializers aren't yet available while parsing default arguments
// of the ENCLOSING class's member functions (reproduced independently of
// this codebase: https://godbolt.org/ style minimal case with a
// class-local aggregate + `= {}` default argument on a sibling method).
struct GaussNewtonOptions {
  int max_iterations = 30;
  double initial_lambda = 1e-3;
  double lambda_up_factor = 5.0;
  double lambda_down_factor = 3.0;
  int max_inner_retries = 8;
  double cost_change_tolerance = 1e-12;
};

struct GaussNewtonSummary {
  int iterations = 0;
  double initial_cost = 0.0;
  double final_cost = 0.0;
  bool converged = false;
};

// Hand-rolled Levenberg-Marquardt over dense Eigen linear algebra,
// operating directly on each keyframe's raw 7-parameter block
// (translation + quaternion xyzw), renormalizing the quaternion after every
// accepted step rather than using a proper 6-DOF tangent-space/manifold
// update. This is a deliberate, documented v1 simplification — see
// platform architecture section 20 ("第一版图优化库...延后到实施计划用实测
// 选择"): it exists to validate the FactorBuilder/StateStore/PoseGraphProblem
// contracts end-to-end on small synthetic problems, not as the eventual
// production solver. Swapping this out for Ceres or GTSAM later does not
// require touching factor_builders — they only depend on
// uw::measurement_api::ResidualBlock.
//
// Dense O(N^2) linear algebra is intentional and fine for v1 problem sizes
// (single-digit to low hundreds of keyframes); a real solver would use
// sparse factorization keyed off the graph's actual sparsity.
class GaussNewtonSolver {
 public:
  using Options = GaussNewtonOptions;
  using Summary = GaussNewtonSummary;

  Summary Solve(PoseGraphProblem& problem, const Options& options = Options{}) const;

 private:
  // Evaluates every residual block once. If `jtj`/`jtr` are non-null,
  // accumulates the normal equations restricted to the free (non-fixed)
  // keyframes indexed by `free_index`. Always returns the total cost
  // (0.5 * sum ||r||^2). `param_ptrs` maps keyframe id to its raw 7-double
  // parameter block, obtained once per Solve() call via PoseGraphProblem's
  // public MutableParameterBlocks() (see pose_graph_problem.hpp) — this
  // solver has no special/friend access to PoseGraphProblem, so a second
  // solver can be added the same way.
  static double EvaluateAll(PoseGraphProblem& problem,
                            const std::unordered_map<std::string, double*>& param_ptrs,
                            const std::unordered_map<std::string, int>& free_index,
                            Eigen::MatrixXd* jtj, Eigen::VectorXd* jtr);
};

}  // namespace uw::estimation
