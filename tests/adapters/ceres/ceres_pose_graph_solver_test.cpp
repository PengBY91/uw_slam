#include <cmath>

#include <gtest/gtest.h>

#include "adapters/ceres/ceres_pose_graph_solver.hpp"
#include "estimation/pose_graph_problem.hpp"
#include "factor_builders/depth_residual.hpp"
#include "factor_builders/relative_pose_residual.hpp"

using uw::adapters::ceres_solver::CeresPoseGraphSolver;
using uw::estimation::PoseGraphProblem;
using uw::factor_builders::DepthResidual;
using uw::factor_builders::RelativePoseResidual;
using uw::sensor_models::Pose3;

namespace {

// Same fixture shape as
// tests/estimation/pose_graph_solver_test.cpp's
// PoseGraphSolver.ThreeKeyframeChainConvergesToTruth — this is the direct
// apples-to-apples comparison the design doc's benchmark (§6.1.1) is built
// around: both solvers must recover the same ground truth from the same
// problem.
PoseGraphProblem BuildThreeKeyframeChain(Pose3* out_true_kf1, Pose3* out_true_kf2, double* out_true_depth) {
  Pose3 true_kf0;  // identity
  Pose3 relative_01;
  relative_01.translation = Eigen::Vector3d(1.0, 0.2, -0.1);
  relative_01.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(0.15, Eigen::Vector3d::UnitZ()));
  Pose3 relative_12;
  relative_12.translation = Eigen::Vector3d(0.8, -0.3, 0.05);
  relative_12.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(-0.1, Eigen::Vector3d::UnitY()));

  *out_true_kf1 = true_kf0 * relative_01;
  *out_true_kf2 = (*out_true_kf1) * relative_12;
  *out_true_depth = -out_true_kf2->translation.z();

  PoseGraphProblem problem;
  problem.AddKeyframe("kf0", true_kf0, /*fixed=*/true);

  Pose3 init_kf1 = *out_true_kf1;
  init_kf1.translation += Eigen::Vector3d(0.3, -0.2, 0.15);
  Pose3 init_kf2 = *out_true_kf2;
  init_kf2.translation += Eigen::Vector3d(-0.25, 0.35, -0.1);
  problem.AddKeyframe("kf1", init_kf1);
  problem.AddKeyframe("kf2", init_kf2);

  problem.AddResidualBlock(
      std::make_unique<RelativePoseResidual>(relative_01, /*sqrt_info_t=*/10.0, /*sqrt_info_r=*/10.0),
      {"kf0", "kf1"});
  problem.AddResidualBlock(
      std::make_unique<RelativePoseResidual>(relative_12, /*sqrt_info_t=*/10.0, /*sqrt_info_r=*/10.0),
      {"kf1", "kf2"});
  problem.AddResidualBlock(std::make_unique<DepthResidual>(*out_true_depth, /*sqrt_information=*/5.0),
                           {"kf2"});
  return problem;
}

}  // namespace

TEST(CeresPoseGraphSolver, ThreeKeyframeChainConvergesToTruth) {
  Pose3 true_kf1, true_kf2;
  double true_depth = 0.0;
  PoseGraphProblem problem = BuildThreeKeyframeChain(&true_kf1, &true_kf2, &true_depth);

  CeresPoseGraphSolver solver;
  const auto summary = solver.Solve(problem);

  EXPECT_TRUE(summary.converged);
  EXPECT_LT(summary.final_cost, summary.initial_cost);
  EXPECT_LT(summary.final_cost, 1e-6);

  const Pose3 solved_kf1 = problem.GetKeyframePose("kf1");
  const Pose3 solved_kf2 = problem.GetKeyframePose("kf2");
  EXPECT_NEAR((solved_kf1.translation - true_kf1.translation).norm(), 0.0, 1e-3);
  EXPECT_NEAR((solved_kf2.translation - true_kf2.translation).norm(), 0.0, 1e-3);
}

TEST(CeresPoseGraphSolver, SolvedQuaternionsStayUnitNorm) {
  // Proof the EigenQuaternionManifold is actually wired correctly (xyzw
  // layout, composed with the translation block in the right order) — a
  // wrong manifold either fails to build, or lets the quaternion drift off
  // the unit sphere the way GaussNewtonSolver's hand-rolled
  // renormalize-after-every-step would be needed to prevent.
  Pose3 true_kf1, true_kf2;
  double true_depth = 0.0;
  PoseGraphProblem problem = BuildThreeKeyframeChain(&true_kf1, &true_kf2, &true_depth);

  CeresPoseGraphSolver solver;
  solver.Solve(problem);

  for (const std::string id : {"kf1", "kf2"}) {
    const Pose3 solved = problem.GetKeyframePose(id);
    EXPECT_NEAR(solved.rotation.norm(), 1.0, 1e-9) << "keyframe " << id;
  }
}

TEST(CeresPoseGraphSolver, FixedKeyframeIsNotMoved) {
  Pose3 true_kf1, true_kf2;
  double true_depth = 0.0;
  PoseGraphProblem problem = BuildThreeKeyframeChain(&true_kf1, &true_kf2, &true_depth);

  CeresPoseGraphSolver solver;
  solver.Solve(problem);

  const Pose3 solved_kf0 = problem.GetKeyframePose("kf0");
  EXPECT_NEAR(solved_kf0.translation.norm(), 0.0, 1e-12) << "kf0 was added fixed=true, must stay at identity";
  EXPECT_NEAR(solved_kf0.rotation.norm(), 1.0, 1e-12);
  EXPECT_NEAR(std::abs(solved_kf0.rotation.w()), 1.0, 1e-12);
}

TEST(CeresPoseGraphSolver, SingleThreadedSolveIsBitDeterministic) {
  // num_threads defaults to 1 specifically for this — see
  // CeresSolverOptions's doc comment. Two independent problem instances
  // (not the same PoseGraphProblem re-solved, which would start from
  // already-converged state) must land on bit-identical results, matching
  // the bar tests/integration/determinism_test.sh holds the rest of this
  // repo's replay pipeline to.
  Pose3 true_kf1_a, true_kf2_a, true_kf1_b, true_kf2_b;
  double true_depth_a = 0.0, true_depth_b = 0.0;
  PoseGraphProblem problem_a = BuildThreeKeyframeChain(&true_kf1_a, &true_kf2_a, &true_depth_a);
  PoseGraphProblem problem_b = BuildThreeKeyframeChain(&true_kf1_b, &true_kf2_b, &true_depth_b);

  CeresPoseGraphSolver solver;
  const auto summary_a = solver.Solve(problem_a);
  const auto summary_b = solver.Solve(problem_b);

  EXPECT_EQ(summary_a.iterations, summary_b.iterations);
  EXPECT_EQ(summary_a.converged, summary_b.converged);
  EXPECT_EQ(summary_a.initial_cost, summary_b.initial_cost);
  EXPECT_EQ(summary_a.final_cost, summary_b.final_cost);

  const Pose3 kf1_a = problem_a.GetKeyframePose("kf1");
  const Pose3 kf1_b = problem_b.GetKeyframePose("kf1");
  EXPECT_EQ(kf1_a.translation, kf1_b.translation);
  EXPECT_TRUE(kf1_a.rotation.coeffs() == kf1_b.rotation.coeffs());
}
