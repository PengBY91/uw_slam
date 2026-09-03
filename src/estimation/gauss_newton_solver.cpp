#include "estimation/gauss_newton_solver.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

namespace uw::estimation {

namespace {
constexpr int kPoseBlockDim = 7;
// Widest optimizable block, used only to size the per-step backup buffers
// without a heap allocation per block (pose = 7, inertial = 9).
constexpr int kMaxBlockDim = PoseGraphProblem::kInertialBlockDim;
static_assert(kMaxBlockDim >= kPoseBlockDim, "backup buffer must fit a pose block");

void RenormalizeQuaternion(double* params) {
  Eigen::Map<Eigen::Vector4d> q(params + 3);  // [qx,qy,qz,qw]
  const double n = q.norm();
  if (n > 1e-12) q /= n;
}
}  // namespace

std::string GaussNewtonSolver::ParameterKey(const PoseGraphProblem::ParameterRef& ref) {
  return (ref.kind == PoseGraphProblem::ParameterKind::kPose ? "P:" : "I:") + ref.keyframe_id;
}

double GaussNewtonSolver::EvaluateAll(PoseGraphProblem& problem,
                                       const std::unordered_map<std::string, double*>& param_ptrs,
                                       const std::unordered_map<std::string, FreeBlock>& free_blocks,
                                       Eigen::MatrixXd* jtj, Eigen::VectorXd* jtr, double huber_delta) {
  double cost = 0.0;

  // Reads/writes go through PoseGraphProblem's public ResidualBindings()/
  // MutableParameterBlocks() accessors (see pose_graph_problem.hpp) rather
  // than friend access to its private storage, so any solver — not just
  // this one — can share the same accessor.
  for (const auto& binding : problem.ResidualBindings()) {
    const int dim = binding.block->ResidualDim();
    const auto block_sizes = binding.block->ParameterBlockSizes();

    std::vector<const double*> params;
    params.reserve(binding.involved_parameters->size());
    for (const auto& ref : *binding.involved_parameters) {
      params.push_back(param_ptrs.at(ParameterKey(ref)));
    }

    std::vector<double> residuals(static_cast<std::size_t>(dim));
    std::vector<std::vector<double>> jac_storage(block_sizes.size());
    std::vector<double*> jac_ptrs(block_sizes.size(), nullptr);
    const bool want_jacobian = (jtj != nullptr);
    if (want_jacobian) {
      for (std::size_t b = 0; b < block_sizes.size(); ++b) {
        jac_storage[b].assign(static_cast<std::size_t>(dim) * block_sizes[b], 0.0);
        jac_ptrs[b] = jac_storage[b].data();
      }
    }

    const bool ok = binding.block->Evaluate(params, residuals.data(), want_jacobian ? &jac_ptrs : nullptr);
    if (!ok) continue;

    // IRLS-style scaled-residual reweighting (Ceres's Corrector, minus the
    // curvature/alpha term a Gauss-Newton-only solver has no use for):
    // scales BOTH the residual and every Jacobian block by the same w, so
    // everything downstream (cost, jtj/jtr accumulation below) only ever
    // reads the already-reweighted values and needs no further changes.
    // kNone bindings (every pre-existing factor type) skip this entirely,
    // so their contribution stays bit-identical to before this existed.
    if (binding.robust_policy == PoseGraphProblem::RobustPolicy::kHuber) {
      Eigen::Map<Eigen::VectorXd> r_mut(residuals.data(), dim);
      const double norm = r_mut.norm();
      if (norm > huber_delta && norm > 1e-12) {
        const double w = std::sqrt(huber_delta / norm);
        r_mut *= w;
        if (want_jacobian) {
          for (auto& storage : jac_storage) {
            for (double& v : storage) v *= w;
          }
        }
      }
    }

    Eigen::Map<const Eigen::VectorXd> r(residuals.data(), dim);
    cost += 0.5 * r.squaredNorm();

    if (!want_jacobian) continue;

    // Column offsets/widths come from free_blocks rather than "index * 7":
    // a binding may now mix 7-wide pose blocks with 9-wide inertial ones
    // (PoseGraphProblem option A). For a graph with no inertial states the
    // offsets are still exactly 7 * keyframe_index, so the assembled normal
    // equations are bit-identical to the pre-PREP-B-01 solver.
    for (std::size_t b = 0; b < binding.involved_parameters->size(); ++b) {
      auto it_b = free_blocks.find(ParameterKey((*binding.involved_parameters)[b]));
      if (it_b == free_blocks.end()) continue;  // fixed block: no column to solve for
      const int col_b = it_b->second.offset;
      const int size_b = it_b->second.size;
      Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> Jb(
          jac_storage[b].data(), dim, size_b);

      jtr->segment(col_b, size_b) += Jb.transpose() * r;

      for (std::size_t b2 = 0; b2 < binding.involved_parameters->size(); ++b2) {
        auto it_b2 = free_blocks.find(ParameterKey((*binding.involved_parameters)[b2]));
        if (it_b2 == free_blocks.end()) continue;
        const int col_b2 = it_b2->second.offset;
        const int size_b2 = it_b2->second.size;
        Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> Jb2(
            jac_storage[b2].data(), dim, size_b2);
        jtj->block(col_b, col_b2, size_b, size_b2) += Jb.transpose() * Jb2;
      }
    }
  }

  return cost;
}

GaussNewtonSolver::Summary GaussNewtonSolver::Solve(PoseGraphProblem& problem,
                                                     const Options& options) const {
  Summary summary;

  // Pointers into PoseGraphProblem's own storage, valid for this whole
  // Solve() call (no blocks are added/removed while solving). Poses come
  // first, then inertial states, so a pose-only graph gets exactly the
  // column layout it got before inertial states existed.
  const auto all_blocks = problem.MutableAllParameterBlocks();
  std::unordered_map<std::string, double*> param_ptrs;
  std::unordered_map<std::string, FreeBlock> free_blocks;
  std::vector<const PoseGraphProblem::ParameterBlockView*> free_order;
  int num_columns = 0;
  for (const auto& block : all_blocks) {
    const std::string key = ParameterKey(block.ref);
    param_ptrs.emplace(key, block.params);
    if (block.fixed) continue;
    free_blocks.emplace(key, FreeBlock{num_columns, block.size});
    free_order.push_back(&block);
    num_columns += block.size;
  }

  if (num_columns == 0) {
    summary.initial_cost = summary.final_cost =
        EvaluateAll(problem, param_ptrs, free_blocks, nullptr, nullptr, options.huber_delta);
    summary.converged = true;
    return summary;
  }

  double lambda = options.initial_lambda;
  double current_cost = EvaluateAll(problem, param_ptrs, free_blocks, nullptr, nullptr, options.huber_delta);
  summary.initial_cost = current_cost;

  for (int iter = 0; iter < options.max_iterations; ++iter) {
    Eigen::MatrixXd jtj = Eigen::MatrixXd::Zero(num_columns, num_columns);
    Eigen::VectorXd jtr = Eigen::VectorXd::Zero(num_columns);
    const double cost_at_linearization =
        EvaluateAll(problem, param_ptrs, free_blocks, &jtj, &jtr, options.huber_delta);

    bool step_accepted = false;
    for (int retry = 0; retry < options.max_inner_retries; ++retry) {
      Eigen::MatrixXd damped = jtj;
      for (int i = 0; i < damped.rows(); ++i) {
        damped(i, i) += lambda * std::max(damped(i, i), 1e-12);
      }
      const Eigen::VectorXd delta = damped.ldlt().solve(-jtr);

      // Backup, apply, evaluate. Iterated over free_order (insertion
      // order) rather than the hash map so the sequence of floating-point
      // updates is deterministic across runs and libstdc++ versions.
      std::vector<std::array<double, kMaxBlockDim>> backup(free_order.size());
      for (std::size_t b = 0; b < free_order.size(); ++b) {
        const auto& view = *free_order[b];
        double* params = view.params;
        std::copy(params, params + view.size, backup[b].begin());
        const FreeBlock& slot = free_blocks.at(ParameterKey(view.ref));
        for (int d = 0; d < view.size; ++d) params[d] += delta(slot.offset + d);
        // Only poses carry a quaternion; inertial blocks are a plain R^9.
        if (view.ref.kind == PoseGraphProblem::ParameterKind::kPose) RenormalizeQuaternion(params);
      }

      const double trial_cost =
          EvaluateAll(problem, param_ptrs, free_blocks, nullptr, nullptr, options.huber_delta);
      if (trial_cost <= cost_at_linearization) {
        current_cost = trial_cost;
        lambda = std::max(lambda / options.lambda_down_factor, 1e-12);
        step_accepted = true;
        break;
      }
      // Reject: restore and increase damping.
      for (std::size_t b = 0; b < free_order.size(); ++b) {
        const auto& view = *free_order[b];
        std::copy(backup[b].begin(), backup[b].begin() + view.size, view.params);
      }
      lambda *= options.lambda_up_factor;
    }

    summary.iterations = iter + 1;
    if (!step_accepted) break;  // damping exhausted: stall, report honestly

    // Scaled by cost magnitude, not a bare absolute difference: an absolute
    // 1e-12 threshold sits right at (or below) the floating-point noise
    // floor of EvaluateAll/LDLT for costs in the tens-to-hundreds range (a
    // normal magnitude once a problem has more than a couple of keyframes),
    // so the last iteration or two before real convergence become a race
    // between "diff crosses the threshold" and "roundoff noise makes no
    // trial step look like an improvement anymore" -- whichever wins is
    // effectively arbitrary, so the exact same well-converged problem can
    // report `converged` or `stalled` depending on unrelated bit-level
    // noise in how it got there (observed directly: separating
    // apps/synth_bag_gen.cpp's noise RNG streams changed nothing about the
    // solved problem's quality, cost still flattened to 12+ significant
    // figures by iteration 4-5, but shifted which side of this race the
    // last iteration landed on, flipping a real replay_demo run from
    // converged to stalled). Scaling by cost keeps the same effective
    // precision for small problems (scale clamped to >= 1) while staying
    // comfortably above the roundoff floor for larger ones.
    const double tolerance_scale = std::max(1.0, std::abs(cost_at_linearization));
    if (std::abs(cost_at_linearization - current_cost) < options.cost_change_tolerance * tolerance_scale) {
      summary.converged = true;
      break;
    }
  }

  summary.final_cost = current_cost;
  return summary;
}

}  // namespace uw::estimation
