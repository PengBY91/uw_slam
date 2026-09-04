// Ceres-backed PoseGraphProblem solver — the benchmark-decision-gate
// candidate described in
// docs/archive/superpowers/specs/2026-08-23-solver-and-mapping-oss-adoption.md
// (§5.1/§6.1), evaluated against uw::estimation::GaussNewtonSolver, the
// hand-rolled v1 solver `include/estimation/gauss_newton_solver.hpp`
// documents as a deliberate, deferred choice (platform architecture
// section 20). Lives outside include/ and src/ (mirrors adapters/ros2's and
// adapters/spatial_index's precedent) so Ceres headers never have to be
// visible from `estimation` — only `application` (allowed to depend on
// everything) and the benchmark tool construct this directly.
//
// No Ceres type appears in this public interface: Solve() takes/returns
// only uw::estimation types, so callers that just want "a solver" don't
// need to know or care which backend they linked.
#pragma once

#include "estimation/gauss_newton_solver.hpp"
#include "estimation/pose_graph_problem.hpp"

namespace uw::adapters::ceres_solver {

struct CeresSolverOptions {
  int max_iterations = 30;
  // Default 1, not Ceres's usual multi-threaded default: determinism first
  // (see the design doc's error-handling table) — this repo's replay is
  // required to be bit-reproducible across runs
  // (tests/integration/determinism_test.sh), and Ceres's parallel Jacobian
  // evaluation is not guaranteed deterministic across thread-count/hardware.
  // Only raise this for throughput once a caller has verified it doesn't
  // need bit-for-bit reproducibility.
  int num_threads = 1;
  double function_tolerance = 1e-12;
};

// Solves `problem` using Ceres's trust-region minimizer (Levenberg-Marquardt
// by default) instead of the hand-rolled uw::estimation::GaussNewtonSolver.
// Returns a uw::estimation::GaussNewtonSummary — the same shape
// GaussNewtonSolver::Solve returns — so callers can swap backends without
// caring which one actually ran (matches ResidualBlock's own "swap the
// solver without touching factor_builders" design intent).
class CeresPoseGraphSolver {
 public:
  uw::estimation::GaussNewtonSummary Solve(uw::estimation::PoseGraphProblem& problem,
                                            const CeresSolverOptions& options = {}) const;
};

}  // namespace uw::adapters::ceres_solver
