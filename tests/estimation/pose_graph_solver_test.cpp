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

// Covers the backend-agnostic accessors that replaced the earlier
// `friend class GaussNewtonSolver` — any future solver adapter (e.g. Ceres)
// relies on these having exactly this shape.
TEST(PoseGraphProblem, MutableParameterBlocksMatchKeyframeOrderAndAreWritable) {
  PoseGraphProblem problem;
  Pose3 pose0;
  pose0.translation = Eigen::Vector3d(1.0, 2.0, 3.0);
  Pose3 pose1;
  pose1.translation = Eigen::Vector3d(4.0, 5.0, 6.0);
  problem.AddKeyframe("kf0", pose0, /*fixed=*/true);
  problem.AddKeyframe("kf1", pose1, /*fixed=*/false);

  const auto blocks = problem.MutableParameterBlocks();
  ASSERT_EQ(blocks.size(), problem.KeyframeOrder().size());
  for (std::size_t i = 0; i < blocks.size(); ++i) {
    EXPECT_EQ(blocks[i].keyframe_id, problem.KeyframeOrder()[i]);
    EXPECT_EQ(blocks[i].fixed, problem.IsFixed(blocks[i].keyframe_id));
    ASSERT_NE(blocks[i].params, nullptr);
  }

  // Mutating through the returned pointer is visible via GetKeyframePose —
  // this is exactly what a solver's optimization step relies on.
  auto* kf1_params = blocks[1].params;
  kf1_params[0] = 42.0;
  EXPECT_DOUBLE_EQ(problem.GetKeyframePose("kf1").translation.x(), 42.0);
}

TEST(PoseGraphProblem, ResidualBindingsMatchAddOrderAndInvolvedKeyframes) {
  PoseGraphProblem problem;
  problem.AddKeyframe("kf0", Pose3{}, /*fixed=*/true);
  problem.AddKeyframe("kf1", Pose3{});
  problem.AddKeyframe("kf2", Pose3{});

  auto* relative_block =
      new RelativePoseResidual(Pose3{}, Eigen::Matrix<double, 6, 6>::Identity() * 1.0);
  auto* depth_block = new DepthResidual(/*measured_depth_m=*/1.0, /*sqrt_information=*/1.0);
  problem.AddResidualBlock(std::unique_ptr<RelativePoseResidual>(relative_block), {"kf0", "kf1"});
  problem.AddResidualBlock(std::unique_ptr<DepthResidual>(depth_block), {"kf2"});

  const auto bindings = problem.ResidualBindings();
  ASSERT_EQ(bindings.size(), 2u);
  EXPECT_EQ(bindings[0].block, relative_block);
  ASSERT_NE(bindings[0].involved_keyframes, nullptr);
  EXPECT_EQ(*bindings[0].involved_keyframes, (std::vector<std::string>{"kf0", "kf1"}));
  EXPECT_EQ(bindings[1].block, depth_block);
  ASSERT_NE(bindings[1].involved_keyframes, nullptr);
  EXPECT_EQ(*bindings[1].involved_keyframes, (std::vector<std::string>{"kf2"}));
}
