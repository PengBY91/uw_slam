#include <cmath>

#include <gtest/gtest.h>

#include "estimation/gauss_newton_solver.hpp"
#include "estimation/pose_graph_problem.hpp"
#include "factor_builders/depth_residual.hpp"
#include "factor_builders/relative_pose_residual.hpp"

using uw::estimation::GaussNewtonSolver;
using uw::estimation::PoseGraphProblem;
using uw::factor_builders::DepthResidual;
using uw::factor_builders::RelativePoseResidual;
using uw::sensor_models::Pose3;

// End-to-end check that PoseGraphProblem + GaussNewtonSolver correctly
// consume real ResidualBlock implementations from
// algorithms/factor_builders/* — not just synthetic test doubles. This is
// the concrete proof that the FactorBuilder -> ResidualBlock ->
// PoseGraphProblem -> solver chain described in the architecture doc
// actually composes.
TEST(PoseGraphSolver, ThreeKeyframeChainConvergesToTruth) {
  Pose3 true_kf0;  // identity
  Pose3 relative_01;
  relative_01.translation = Eigen::Vector3d(1.0, 0.2, -0.1);
  relative_01.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(0.15, Eigen::Vector3d::UnitZ()));
  Pose3 relative_12;
  relative_12.translation = Eigen::Vector3d(0.8, -0.3, 0.05);
  relative_12.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(-0.1, Eigen::Vector3d::UnitY()));

  const Pose3 true_kf1 = true_kf0 * relative_01;
  const Pose3 true_kf2 = true_kf1 * relative_12;
  const double true_depth = -true_kf2.translation.z();

  PoseGraphProblem problem;
  problem.AddKeyframe("kf0", true_kf0, /*fixed=*/true);

  // Perturbed initial guesses for the free keyframes.
  Pose3 init_kf1 = true_kf1;
  init_kf1.translation += Eigen::Vector3d(0.3, -0.2, 0.15);
  Pose3 init_kf2 = true_kf2;
  init_kf2.translation += Eigen::Vector3d(-0.25, 0.35, -0.1);
  problem.AddKeyframe("kf1", init_kf1);
  problem.AddKeyframe("kf2", init_kf2);

  problem.AddResidualBlock(
      std::make_unique<RelativePoseResidual>(relative_01, Eigen::Matrix<double, 6, 6>::Identity() * 10.0),
      {"kf0", "kf1"});
  problem.AddResidualBlock(
      std::make_unique<RelativePoseResidual>(relative_12, Eigen::Matrix<double, 6, 6>::Identity() * 10.0),
      {"kf1", "kf2"});
  problem.AddResidualBlock(std::make_unique<DepthResidual>(true_depth, /*sqrt_information=*/5.0),
                           {"kf2"});

  GaussNewtonSolver solver;
  const auto summary = solver.Solve(problem);

  EXPECT_LT(summary.final_cost, summary.initial_cost);
  EXPECT_LT(summary.final_cost, 1e-6);

  const Pose3 solved_kf1 = problem.GetKeyframePose("kf1");
  const Pose3 solved_kf2 = problem.GetKeyframePose("kf2");
  EXPECT_NEAR((solved_kf1.translation - true_kf1.translation).norm(), 0.0, 1e-3);
  EXPECT_NEAR((solved_kf2.translation - true_kf2.translation).norm(), 0.0, 1e-3);
}
