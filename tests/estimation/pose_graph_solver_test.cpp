#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

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

  // Default-constructed bindings (2-arg AddResidualBlock, used by every
  // pre-existing factor call site) must report kNone -- this is the
  // guarantee the Huber addition relies on for zero behavior change.
  EXPECT_EQ(bindings[0].robust_policy, PoseGraphProblem::RobustPolicy::kNone);
  EXPECT_EQ(bindings[1].robust_policy, PoseGraphProblem::RobustPolicy::kNone);
}

namespace {
// Builds the ThreeKeyframeChainConvergesToTruth graph, but lets the caller
// choose how the two relative-pose edges attach their AddResidualBlock
// (default 2-arg vs. explicit 3-arg) -- used by
// HuberPolicyNoneIsByteIdenticalToPreExistingDefault below to prove the new
// 3-arg overload with kNone reproduces the old 2-arg call exactly.
GaussNewtonSolver::Summary SolveThreeKeyframeChain(
    PoseGraphProblem& problem,
    const std::function<void(PoseGraphProblem&, std::unique_ptr<RelativePoseResidual>,
                             std::vector<std::string>)>& add_relative_pose_block) {
  Pose3 true_kf0;
  Pose3 relative_01;
  relative_01.translation = Eigen::Vector3d(1.0, 0.2, -0.1);
  relative_01.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(0.15, Eigen::Vector3d::UnitZ()));
  Pose3 relative_12;
  relative_12.translation = Eigen::Vector3d(0.8, -0.3, 0.05);
  relative_12.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(-0.1, Eigen::Vector3d::UnitY()));

  const Pose3 true_kf1 = true_kf0 * relative_01;
  const Pose3 true_kf2 = true_kf1 * relative_12;
  const double true_depth = -true_kf2.translation.z();

  problem.AddKeyframe("kf0", true_kf0, /*fixed=*/true);
  Pose3 init_kf1 = true_kf1;
  init_kf1.translation += Eigen::Vector3d(0.3, -0.2, 0.15);
  Pose3 init_kf2 = true_kf2;
  init_kf2.translation += Eigen::Vector3d(-0.25, 0.35, -0.1);
  problem.AddKeyframe("kf1", init_kf1);
  problem.AddKeyframe("kf2", init_kf2);

  add_relative_pose_block(
      problem,
      std::make_unique<RelativePoseResidual>(relative_01, Eigen::Matrix<double, 6, 6>::Identity() * 10.0),
      {"kf0", "kf1"});
  add_relative_pose_block(
      problem,
      std::make_unique<RelativePoseResidual>(relative_12, Eigen::Matrix<double, 6, 6>::Identity() * 10.0),
      {"kf1", "kf2"});
  problem.AddResidualBlock(std::make_unique<DepthResidual>(true_depth, /*sqrt_information=*/5.0), {"kf2"});

  GaussNewtonSolver solver;
  return solver.Solve(problem);
}
}  // namespace

TEST(PoseGraphSolver, HuberPolicyNoneIsByteIdenticalToPreExistingDefault) {
  PoseGraphProblem problem_default;
  const auto summary_default =
      SolveThreeKeyframeChain(problem_default, [](PoseGraphProblem& p, auto block, auto kfs) {
        p.AddResidualBlock(std::move(block), std::move(kfs));  // old 2-arg call, exactly as before
      });

  PoseGraphProblem problem_explicit_none;
  const auto summary_explicit_none =
      SolveThreeKeyframeChain(problem_explicit_none, [](PoseGraphProblem& p, auto block, auto kfs) {
        p.AddResidualBlock(std::move(block), std::move(kfs), PoseGraphProblem::RobustPolicy::kNone);
      });

  EXPECT_EQ(summary_default.iterations, summary_explicit_none.iterations);
  EXPECT_DOUBLE_EQ(summary_default.initial_cost, summary_explicit_none.initial_cost);
  EXPECT_DOUBLE_EQ(summary_default.final_cost, summary_explicit_none.final_cost);
  EXPECT_EQ(summary_default.converged, summary_explicit_none.converged);

  for (const std::string kf_id : {"kf1", "kf2"}) {
    const Pose3 pose_default = problem_default.GetKeyframePose(kf_id);
    const Pose3 pose_explicit = problem_explicit_none.GetKeyframePose(kf_id);
    EXPECT_DOUBLE_EQ(pose_default.translation.x(), pose_explicit.translation.x());
    EXPECT_DOUBLE_EQ(pose_default.translation.y(), pose_explicit.translation.y());
    EXPECT_DOUBLE_EQ(pose_default.translation.z(), pose_explicit.translation.z());
  }
}

namespace {
// A minimal two-conflicting-edges rig: kf0 fixed at the origin, kf1 free,
// connected by two equally-weighted RelativePoseResidual edges that
// disagree only on translation.x (one says +1m, the other +5m; both agree
// on y/z/rotation=identity). With two kNone edges of equal weight, LM
// settles kf1.x at the unweighted midpoint (3.0) -- classic non-robust
// least-squares behavior. Flagging the "bad" (5.0) edge kHuber with a
// small huber_delta should pull kf1.x much closer to the "good" (1.0)
// edge's measurement instead.
double SolveConflictingEdgesKf1X(uw::estimation::PoseGraphProblem::RobustPolicy bad_edge_policy,
                                 double huber_delta) {
  PoseGraphProblem problem;
  Pose3 kf0;  // identity
  problem.AddKeyframe("kf0", kf0, /*fixed=*/true);

  Pose3 init_kf1;
  init_kf1.translation = Eigen::Vector3d(2.0, 0.0, 0.0);
  problem.AddKeyframe("kf1", init_kf1);

  Pose3 good_relative;
  good_relative.translation = Eigen::Vector3d(1.0, 0.0, 0.0);
  Pose3 bad_relative;
  bad_relative.translation = Eigen::Vector3d(5.0, 0.0, 0.0);
  const auto sqrt_info = Eigen::Matrix<double, 6, 6>::Identity() * 10.0;

  problem.AddResidualBlock(std::make_unique<RelativePoseResidual>(good_relative, sqrt_info), {"kf0", "kf1"},
                           PoseGraphProblem::RobustPolicy::kNone);
  problem.AddResidualBlock(std::make_unique<RelativePoseResidual>(bad_relative, sqrt_info), {"kf0", "kf1"},
                           bad_edge_policy);

  GaussNewtonSolver solver;
  GaussNewtonSolver::Options options;
  options.huber_delta = huber_delta;
  solver.Solve(problem, options);

  return problem.GetKeyframePose("kf1").translation.x();
}
}  // namespace

TEST(PoseGraphSolver, HuberDownweightsOutlierResidual) {
  const double kf1_x_no_robust =
      SolveConflictingEdgesKf1X(PoseGraphProblem::RobustPolicy::kNone, /*huber_delta=*/1.0);
  const double kf1_x_huber =
      SolveConflictingEdgesKf1X(PoseGraphProblem::RobustPolicy::kHuber, /*huber_delta=*/1.0);

  // Unweighted midpoint of the two conflicting edges.
  EXPECT_NEAR(kf1_x_no_robust, 3.0, 1e-2);
  // Huber-flagging the outlier edge must pull the solution meaningfully
  // closer to the "good" edge's measurement (1.0) than the non-robust run.
  EXPECT_LT(std::abs(kf1_x_huber - 1.0), std::abs(kf1_x_no_robust - 1.0));
}

TEST(PoseGraphSolver, HuberInlierResidualUnaffected) {
  // huber_delta generous enough that neither edge's whitened residual ever
  // exceeds it (even the initial-guess residual, ~10*2.0=20 at worst) --
  // confirms the w=1 (no reweighting) branch is a true no-op.
  const double kf1_x_no_robust =
      SolveConflictingEdgesKf1X(PoseGraphProblem::RobustPolicy::kNone, /*huber_delta=*/1000.0);
  const double kf1_x_huber_generous_delta =
      SolveConflictingEdgesKf1X(PoseGraphProblem::RobustPolicy::kHuber, /*huber_delta=*/1000.0);

  EXPECT_DOUBLE_EQ(kf1_x_no_robust, kf1_x_huber_generous_delta);
}
