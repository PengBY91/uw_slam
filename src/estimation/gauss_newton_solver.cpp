#include "estimation/gauss_newton_solver.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <unordered_map>

#include <Eigen/Dense>

namespace uw::estimation {

namespace {
constexpr int kBlockDim = 7;

void RenormalizeQuaternion(double* params) {
  Eigen::Map<Eigen::Vector4d> q(params + 3);  // [qx,qy,qz,qw]
  const double n = q.norm();
  if (n > 1e-12) q /= n;
}
}  // namespace

double GaussNewtonSolver::EvaluateAll(PoseGraphProblem& problem,
                                       const std::unordered_map<std::string, double*>& param_ptrs,
                                       const std::unordered_map<std::string, int>& free_index,
                                       Eigen::MatrixXd* jtj, Eigen::VectorXd* jtr) {
  double cost = 0.0;

  // Reads/writes go through PoseGraphProblem's public ResidualBindings()/
  // MutableParameterBlocks() accessors (see pose_graph_problem.hpp) rather
  // than friend access to its private storage, so any solver — not just
  // this one — can share the same accessor.
  for (const auto& binding : problem.ResidualBindings()) {
    const int dim = binding.block->ResidualDim();
    const auto block_sizes = binding.block->ParameterBlockSizes();

    std::vector<const double*> params;
    params.reserve(binding.involved_keyframes->size());
    for (const auto& kf_id : *binding.involved_keyframes) {
      params.push_back(param_ptrs.at(kf_id));
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

    Eigen::Map<const Eigen::VectorXd> r(residuals.data(), dim);
    cost += 0.5 * r.squaredNorm();

    if (!want_jacobian) continue;

    for (std::size_t b = 0; b < binding.involved_keyframes->size(); ++b) {
      auto it_b = free_index.find((*binding.involved_keyframes)[b]);
      if (it_b == free_index.end()) continue;  // fixed keyframe: no column to solve for
      const int col_b = it_b->second * kBlockDim;
      Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, kBlockDim, Eigen::RowMajor>> Jb(
          jac_storage[b].data(), dim, kBlockDim);

      jtr->segment(col_b, kBlockDim) += Jb.transpose() * r;

      for (std::size_t b2 = 0; b2 < binding.involved_keyframes->size(); ++b2) {
        auto it_b2 = free_index.find((*binding.involved_keyframes)[b2]);
        if (it_b2 == free_index.end()) continue;
        const int col_b2 = it_b2->second * kBlockDim;
        Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, kBlockDim, Eigen::RowMajor>> Jb2(
            jac_storage[b2].data(), dim, kBlockDim);
        jtj->block(col_b, col_b2, kBlockDim, kBlockDim) += Jb.transpose() * Jb2;
      }
    }
  }

  return cost;
}

GaussNewtonSolver::Summary GaussNewtonSolver::Solve(PoseGraphProblem& problem,
                                                     const Options& options) const {
  Summary summary;

  std::unordered_map<std::string, int> free_index;
  for (auto& id : problem.KeyframeOrder()) {
    if (!problem.IsFixed(id)) {
      free_index[id] = static_cast<int>(free_index.size());
    }
  }
  const int num_free = static_cast<int>(free_index.size());

  // Pointers into PoseGraphProblem's own keyframe storage, valid for this
  // whole Solve() call (no keyframes are added/removed while solving).
  std::unordered_map<std::string, double*> param_ptrs;
  for (auto& block : problem.MutableParameterBlocks()) {
    param_ptrs.emplace(block.keyframe_id, block.params);
  }

  if (num_free == 0) {
    summary.initial_cost = summary.final_cost = EvaluateAll(problem, param_ptrs, free_index, nullptr, nullptr);
    summary.converged = true;
    return summary;
  }

  double lambda = options.initial_lambda;
  double current_cost = EvaluateAll(problem, param_ptrs, free_index, nullptr, nullptr);
  summary.initial_cost = current_cost;

  for (int iter = 0; iter < options.max_iterations; ++iter) {
    Eigen::MatrixXd jtj = Eigen::MatrixXd::Zero(kBlockDim * num_free, kBlockDim * num_free);
    Eigen::VectorXd jtr = Eigen::VectorXd::Zero(kBlockDim * num_free);
    const double cost_at_linearization = EvaluateAll(problem, param_ptrs, free_index, &jtj, &jtr);

    bool step_accepted = false;
    for (int retry = 0; retry < options.max_inner_retries; ++retry) {
      Eigen::MatrixXd damped = jtj;
      for (int i = 0; i < damped.rows(); ++i) {
        damped(i, i) += lambda * std::max(damped(i, i), 1e-12);
      }
      const Eigen::VectorXd delta = damped.ldlt().solve(-jtr);

      // Backup, apply, evaluate.
      std::unordered_map<std::string, std::array<double, kBlockDim>> backup;
      for (auto& [id, idx] : free_index) {
        double* params = param_ptrs.at(id);
        std::array<double, kBlockDim> saved;
        std::copy(params, params + kBlockDim, saved.begin());
        backup.emplace(id, saved);
        for (int d = 0; d < kBlockDim; ++d) params[d] += delta(idx * kBlockDim + d);
        RenormalizeQuaternion(params);
      }

      const double trial_cost = EvaluateAll(problem, param_ptrs, free_index, nullptr, nullptr);
      if (trial_cost <= cost_at_linearization) {
        current_cost = trial_cost;
        lambda = std::max(lambda / options.lambda_down_factor, 1e-12);
        step_accepted = true;
        break;
      }
      // Reject: restore and increase damping.
      for (auto& [id, saved] : backup) {
        double* params = param_ptrs.at(id);
        std::copy(saved.begin(), saved.end(), params);
      }
      lambda *= options.lambda_up_factor;
    }

    summary.iterations = iter + 1;
    if (!step_accepted) break;  // damping exhausted: stall, report honestly

    if (std::abs(cost_at_linearization - current_cost) < options.cost_change_tolerance) {
      summary.converged = true;
      break;
    }
  }

  summary.final_cost = current_cost;
  return summary;
}

}  // namespace uw::estimation
