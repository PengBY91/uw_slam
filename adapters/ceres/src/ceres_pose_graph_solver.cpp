#include "adapters/ceres/ceres_pose_graph_solver.hpp"

#include <unordered_map>
#include <vector>

#include <ceres/ceres.h>
#include <ceres/manifold.h>
#include <ceres/product_manifold.h>

namespace uw::adapters::ceres_solver {

namespace {

// Wraps one uw::measurement_api::ResidualBlock as a ceres::CostFunction.
// Non-owning: PoseGraphProblem owns the ResidualBlock instances (via
// AddResidualBlock's unique_ptr), this class just borrows a raw pointer for
// the ceres::Problem's lifetime.
//
// This is a near-trivial passthrough, not a coincidence — ResidualBlock's
// own header comment documents that its Evaluate() signature was
// deliberately shaped to mirror ceres::CostFunction::Evaluate() (same
// `double const* const*` parameter layout, same row-major per-block
// Jacobian convention, same "null jacobians[i] means don't compute it"
// rule) specifically so a Ceres adapter would not need to re-derive any
// residual math.
class ResidualBlockCostFunction : public ::ceres::CostFunction {
 public:
  explicit ResidualBlockCostFunction(uw::measurement_api::ResidualBlock* block) : block_(block) {
    set_num_residuals(block_->ResidualDim());
    for (int size : block_->ParameterBlockSizes()) mutable_parameter_block_sizes()->push_back(size);
  }

  bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override {
    const auto num_blocks = static_cast<std::size_t>(parameter_block_sizes().size());
    const std::vector<const double*> params(parameters, parameters + num_blocks);
    if (jacobians == nullptr) return block_->Evaluate(params, residuals, nullptr);
    std::vector<double*> jac_ptrs(jacobians, jacobians + num_blocks);
    return block_->Evaluate(params, residuals, &jac_ptrs);
  }

 private:
  uw::measurement_api::ResidualBlock* block_;
};

// Pose3::ToParameterBlock()/FromParameterBlock()'s fixed 7-double layout is
// [tx, ty, tz, qx, qy, qz, qw] (CLAUDE.md's Pose3 convention: translation
// then Eigen-ordered xyzw quaternion) — a EuclideanManifold<3> over the
// first 3 ambient dims composed with an EigenQuaternionManifold (Ceres's
// xyzw-layout quaternion manifold, matching this repo's storage exactly,
// unlike ceres::QuaternionManifold which expects wxyz) over the last 4
// covers exactly that layout, in that order.
using PoseManifold = ::ceres::ProductManifold<::ceres::EuclideanManifold<3>, ::ceres::EigenQuaternionManifold>;

}  // namespace

uw::estimation::GaussNewtonSummary CeresPoseGraphSolver::Solve(uw::estimation::PoseGraphProblem& problem,
                                                                const CeresSolverOptions& options) const {
  uw::estimation::GaussNewtonSummary summary;

  ::ceres::Problem ceres_problem;

  // Poses AND inertial states (PREP-B-01 option A): a pose gets the 7-dim
  // ambient block with the quaternion manifold below, an inertial state
  // [v(3), bg(3), ba(3)] is a plain 9-dim Euclidean block with no manifold.
  // Keyed by kind + id, since a keyframe's pose and inertial state are two
  // distinct blocks under the same keyframe id.
  std::unordered_map<std::string, double*> param_ptrs;
  for (const auto& block : problem.MutableAllParameterBlocks()) {
    const bool is_pose = block.ref.kind == uw::estimation::PoseGraphProblem::ParameterKind::kPose;
    if (is_pose) {
      ceres_problem.AddParameterBlock(block.params, block.size, new PoseManifold());
    } else {
      ceres_problem.AddParameterBlock(block.params, block.size);
    }
    if (block.fixed) ceres_problem.SetParameterBlockConstant(block.params);
    param_ptrs.emplace(uw::estimation::GaussNewtonSolver::ParameterKey(block.ref), block.params);
  }

  for (const auto& binding : problem.ResidualBindings()) {
    std::vector<double*> parameter_blocks;
    parameter_blocks.reserve(binding.involved_parameters->size());
    for (const auto& ref : *binding.involved_parameters) {
      parameter_blocks.push_back(
          param_ptrs.at(uw::estimation::GaussNewtonSolver::ParameterKey(ref)));
    }
    // No robust loss (nullptr): matches this repo's current, deliberate
    // scope — see docs/archive/superpowers/specs/2026-08-23-frontend-correctness-
    // closure-design.md §8.2 ("本阶段不实现 robust kernel").
    // TODO: binding.robust_policy (PoseGraphProblem::RobustPolicy, used by
    // GaussNewtonSolver for e.g. loop-closure edges) is not read here yet —
    // a kHuber binding silently gets no robust loss under this backend
    // until a ceres::LossFunction is wired in. Huber only takes effect
    // under estimation.solver == "gauss_newton_v1" (the default) for now.
    ceres_problem.AddResidualBlock(new ResidualBlockCostFunction(binding.block), nullptr, parameter_blocks);
  }

  double initial_cost = 0.0;
  // 0.5 * sum ||r||^2 — same convention uw::estimation::GaussNewtonSolver
  // uses (see gauss_newton_solver.cpp's EvaluateAll), so initial_cost/
  // final_cost are directly comparable between the two backends.
  ceres_problem.Evaluate(::ceres::Problem::EvaluateOptions(), &initial_cost, nullptr, nullptr, nullptr);

  ::ceres::Solver::Options solver_options;
  solver_options.max_num_iterations = options.max_iterations;
  solver_options.num_threads = options.num_threads;
  solver_options.function_tolerance = options.function_tolerance;
  // Dense QR: this repo's problems are v1-scale (single-digit to low
  // hundreds of keyframes, same "intentional and fine for v1" posture
  // GaussNewtonSolver's own header comment documents for its dense O(N^2)
  // normal equations) and dense/deterministic linear algebra is what makes
  // this backend directly comparable to GaussNewtonSolver's own dense
  // solve — a sparse solver choice is exactly the kind of decision the
  // benchmark this adapter exists for should make with real data, not
  // something to default into here.
  solver_options.linear_solver_type = ::ceres::DENSE_QR;

  ::ceres::Solver::Summary ceres_summary;
  ::ceres::Solve(solver_options, &ceres_problem, &ceres_summary);

  summary.iterations = static_cast<int>(ceres_summary.iterations.size());
  summary.initial_cost = initial_cost;
  summary.final_cost = ceres_summary.final_cost;
  summary.converged = (ceres_summary.termination_type == ::ceres::CONVERGENCE);
  return summary;
}

}  // namespace uw::adapters::ceres_solver
