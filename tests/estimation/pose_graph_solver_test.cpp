#include <cmath>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <utility>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "estimation/gauss_newton_solver.hpp"
#include "estimation/pose_graph_problem.hpp"
#include "factor_builders/depth_residual.hpp"
#include "factor_builders/imu_preintegration_residual.hpp"
#include "factor_builders/relative_pose_residual.hpp"
#include "sensor_models/so3.hpp"

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
  auto ids = [](const std::vector<PoseGraphProblem::ParameterRef>& refs) {
    std::vector<std::string> out;
    for (const auto& ref : refs) out.push_back(ref.keyframe_id);
    return out;
  };
  auto all_pose = [](const std::vector<PoseGraphProblem::ParameterRef>& refs) {
    for (const auto& ref : refs) {
      if (ref.kind != PoseGraphProblem::ParameterKind::kPose) return false;
    }
    return true;
  };
  ASSERT_EQ(bindings.size(), 2u);
  EXPECT_EQ(bindings[0].block, relative_block);
  ASSERT_NE(bindings[0].involved_parameters, nullptr);
  EXPECT_EQ(ids(*bindings[0].involved_parameters), (std::vector<std::string>{"kf0", "kf1"}));
  // The string overload must tag every entry kPose -- that is what keeps
  // every pre-PREP-B-01 factor_builder call site working unchanged.
  EXPECT_TRUE(all_pose(*bindings[0].involved_parameters));
  EXPECT_EQ(bindings[1].block, depth_block);
  ASSERT_NE(bindings[1].involved_parameters, nullptr);
  EXPECT_EQ(ids(*bindings[1].involved_parameters), (std::vector<std::string>{"kf2"}));
  EXPECT_TRUE(all_pose(*bindings[1].involved_parameters));

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

// ---------------------------------------------------------------------------
// PREP-B-01 state extension (docs/imu-preintegration-design-2026-09-03.md
// section 6, option A): keyframes may carry a second, 9-dim inertial
// parameter block [v(3), bg(3), ba(3)] that the solver optimizes alongside
// their 7-dim pose.
// ---------------------------------------------------------------------------

namespace {

using uw::factor_builders::ImuPreintegrationResidual;
using uw::sensor_models::PreintegratedImuDelta;
namespace so3 = uw::sensor_models::so3;

struct InertialTruth {
  Pose3 pose;
  PoseGraphProblem::InertialState inertial;
};

// Propagates a keyframe state through one preintegrated delta using the
// exact delta definitions (imu_preintegration.hpp), so a graph built from
// these deltas has a zero-residual solution at the propagated truth.
InertialTruth Propagate(const InertialTruth& from, const PreintegratedImuDelta& delta) {
  const Eigen::Matrix3d R_i = from.pose.rotation.toRotationMatrix();
  const double dt = delta.delta_time_s;
  const Eigen::Vector3d gravity(0.0, 0.0, -delta.gravity_mps2);

  InertialTruth to;
  to.pose.rotation = Eigen::Quaterniond(R_i * delta.delta_rotation).normalized();
  to.pose.translation = from.pose.translation + from.inertial.velocity_W * dt +
                        0.5 * gravity * dt * dt + R_i * delta.delta_position;
  to.inertial.velocity_W =
      from.inertial.velocity_W + gravity * dt + R_i * delta.delta_velocity;
  to.inertial.bias_gyro = from.inertial.bias_gyro;
  to.inertial.bias_accel = from.inertial.bias_accel;
  return to;
}

PreintegratedImuDelta MakeChainDelta(double dt, const Eigen::Vector3d& rotvec,
                                      const Eigen::Vector3d& dv, const Eigen::Vector3d& dp,
                                      const PoseGraphProblem::InertialState& bias_point) {
  PreintegratedImuDelta delta;
  delta.delta_time_s = dt;
  delta.sample_count = static_cast<uint32_t>(dt * 200.0);
  delta.gravity_mps2 = 9.80665;
  delta.delta_rotation = so3::Exp(rotvec);
  delta.delta_velocity = dv;
  delta.delta_position = dp;
  delta.bias_gyro = bias_point.bias_gyro;
  delta.bias_accel = bias_point.bias_accel;
  delta.d_rotation_d_bias_gyro = -dt * Eigen::Matrix3d::Identity();
  delta.d_velocity_d_bias_gyro = 0.02 * dt * Eigen::Matrix3d::Identity();
  delta.d_velocity_d_bias_accel = -dt * Eigen::Matrix3d::Identity();
  delta.d_position_d_bias_gyro = 0.01 * dt * dt * Eigen::Matrix3d::Identity();
  delta.d_position_d_bias_accel = -0.5 * dt * dt * Eigen::Matrix3d::Identity();
  delta.covariance = Eigen::Matrix<double, 15, 15>::Identity();
  return delta;
}

}  // namespace

// The concrete proof that the option-A extension works end to end: a chain
// whose ONLY edges are 15-dim IMU factors, with the anchor keyframe's pose
// AND inertial state fixed (30 residuals, 30 free parameters), must recover
// the propagated truth for both the poses and the velocities/biases.
TEST(PoseGraphSolver, ImuOnlyChainRecoversPosesAndInertialStates) {
  InertialTruth kf0;
  kf0.pose.translation = Eigen::Vector3d(0.4, -0.2, -2.0);
  kf0.pose.rotation = so3::ExpQuaternion(Eigen::Vector3d(0.02, -0.05, 0.10));
  kf0.inertial.velocity_W = Eigen::Vector3d(0.9, 0.1, -0.05);
  kf0.inertial.bias_gyro = Eigen::Vector3d(0.0012, -0.0004, 0.0009);
  kf0.inertial.bias_accel = Eigen::Vector3d(0.011, -0.023, 0.006);

  const auto delta_01 = MakeChainDelta(0.5, {0.01, -0.02, 0.06}, {0.30, -0.05, 0.08},
                                        {0.075, -0.012, 0.020}, kf0.inertial);
  const InertialTruth kf1 = Propagate(kf0, delta_01);
  const auto delta_12 = MakeChainDelta(0.5, {-0.03, 0.01, 0.04}, {0.10, 0.12, -0.06},
                                        {0.025, 0.030, -0.015}, kf1.inertial);
  const InertialTruth kf2 = Propagate(kf1, delta_12);

  PoseGraphProblem problem;
  problem.AddKeyframe("kf0", kf0.pose, /*fixed=*/true);
  problem.AddInertialState("kf0", kf0.inertial, /*fixed=*/true);

  // Deliberately wrong initial guesses for everything that is free.
  Pose3 init_kf1 = kf1.pose;
  init_kf1.translation += Eigen::Vector3d(0.20, -0.15, 0.10);
  init_kf1.rotation = (init_kf1.rotation * so3::ExpQuaternion({0.03, 0.02, -0.04})).normalized();
  Pose3 init_kf2 = kf2.pose;
  init_kf2.translation += Eigen::Vector3d(-0.25, 0.18, -0.12);
  init_kf2.rotation = (init_kf2.rotation * so3::ExpQuaternion({-0.02, 0.04, 0.03})).normalized();

  PoseGraphProblem::InertialState init_inertial;
  init_inertial.velocity_W = Eigen::Vector3d(0.0, 0.0, 0.0);
  init_inertial.bias_gyro = Eigen::Vector3d::Zero();
  init_inertial.bias_accel = Eigen::Vector3d::Zero();

  problem.AddKeyframe("kf1", init_kf1);
  problem.AddInertialState("kf1", init_inertial);
  problem.AddKeyframe("kf2", init_kf2);
  problem.AddInertialState("kf2", init_inertial);

  problem.AddResidualBlockOnParameters(
      std::make_unique<ImuPreintegrationResidual>(delta_01,
                                                  Eigen::Matrix<double, 15, 15>::Identity()),
      {PoseGraphProblem::PoseRef("kf0"), PoseGraphProblem::InertialRef("kf0"),
       PoseGraphProblem::PoseRef("kf1"), PoseGraphProblem::InertialRef("kf1")});
  problem.AddResidualBlockOnParameters(
      std::make_unique<ImuPreintegrationResidual>(delta_12,
                                                  Eigen::Matrix<double, 15, 15>::Identity()),
      {PoseGraphProblem::PoseRef("kf1"), PoseGraphProblem::InertialRef("kf1"),
       PoseGraphProblem::PoseRef("kf2"), PoseGraphProblem::InertialRef("kf2")});

  GaussNewtonSolver solver;
  const auto summary = solver.Solve(problem);
  EXPECT_TRUE(summary.converged);
  EXPECT_LT(summary.final_cost, 1e-12);

  for (const auto& [id, truth] : std::vector<std::pair<std::string, InertialTruth>>{{"kf1", kf1},
                                                                                    {"kf2", kf2}}) {
    const Pose3 solved_pose = problem.GetKeyframePose(id);
    EXPECT_LT((solved_pose.translation - truth.pose.translation).norm(), 1e-4) << id;
    EXPECT_LT(so3::Log(solved_pose.rotation.toRotationMatrix().transpose() *
                       truth.pose.rotation.toRotationMatrix())
                  .norm(),
              1e-4)
        << id;
    const auto solved = problem.GetInertialState(id);
    EXPECT_LT((solved.velocity_W - truth.inertial.velocity_W).norm(), 1e-4) << id;
    EXPECT_LT((solved.bias_gyro - truth.inertial.bias_gyro).norm(), 1e-5) << id;
    EXPECT_LT((solved.bias_accel - truth.inertial.bias_accel).norm(), 1e-5) << id;
  }
}

TEST(PoseGraphProblem, FixedInertialStateIsNotMovedByTheSolver) {
  InertialTruth kf0;
  kf0.pose.translation = Eigen::Vector3d(0.0, 0.0, -1.0);
  kf0.inertial.velocity_W = Eigen::Vector3d(0.5, 0.0, 0.0);
  kf0.inertial.bias_gyro = Eigen::Vector3d(0.001, 0.0, 0.0);
  kf0.inertial.bias_accel = Eigen::Vector3d(0.0, 0.01, 0.0);
  const auto delta = MakeChainDelta(0.4, {0.0, 0.0, 0.02}, {0.1, 0.0, 0.0}, {0.02, 0.0, 0.0},
                                     kf0.inertial);
  const InertialTruth kf1 = Propagate(kf0, delta);

  PoseGraphProblem problem;
  problem.AddKeyframe("kf0", kf0.pose, /*fixed=*/true);
  problem.AddInertialState("kf0", kf0.inertial, /*fixed=*/true);
  Pose3 init_kf1 = kf1.pose;
  init_kf1.translation += Eigen::Vector3d(0.3, 0.3, 0.3);
  problem.AddKeyframe("kf1", init_kf1);
  PoseGraphProblem::InertialState wrong;
  problem.AddInertialState("kf1", wrong);
  problem.AddResidualBlockOnParameters(
      std::make_unique<ImuPreintegrationResidual>(delta,
                                                  Eigen::Matrix<double, 15, 15>::Identity()),
      {PoseGraphProblem::PoseRef("kf0"), PoseGraphProblem::InertialRef("kf0"),
       PoseGraphProblem::PoseRef("kf1"), PoseGraphProblem::InertialRef("kf1")});

  GaussNewtonSolver solver;
  solver.Solve(problem);

  const auto anchor = problem.GetInertialState("kf0");
  EXPECT_LT((anchor.velocity_W - kf0.inertial.velocity_W).norm(), 1e-15);
  EXPECT_LT((anchor.bias_gyro - kf0.inertial.bias_gyro).norm(), 1e-15);
  EXPECT_LT((anchor.bias_accel - kf0.inertial.bias_accel).norm(), 1e-15);
  EXPECT_LT((problem.GetKeyframePose("kf0").translation - kf0.pose.translation).norm(), 1e-15);
}

// A graph with no inertial state at all must present exactly the parameter
// blocks (and therefore the exact column layout) it presented before the
// extension existed — this is the property the whole "synthetic_smoke ATE
// cannot move" argument rests on.
TEST(PoseGraphProblem, PoseOnlyGraphExposesTheSameBlocksThroughBothAccessors) {
  PoseGraphProblem problem;
  Pose3 pose1;
  pose1.translation = Eigen::Vector3d(1.0, 2.0, 3.0);
  problem.AddKeyframe("kf0", Pose3{}, /*fixed=*/true);
  problem.AddKeyframe("kf1", pose1);

  const auto pose_blocks = problem.MutableParameterBlocks();
  const auto all_blocks = problem.MutableAllParameterBlocks();
  ASSERT_EQ(all_blocks.size(), pose_blocks.size());
  for (std::size_t i = 0; i < all_blocks.size(); ++i) {
    EXPECT_EQ(all_blocks[i].ref.kind, PoseGraphProblem::ParameterKind::kPose);
    EXPECT_EQ(all_blocks[i].ref.keyframe_id, pose_blocks[i].keyframe_id);
    EXPECT_EQ(all_blocks[i].params, pose_blocks[i].params);
    EXPECT_EQ(all_blocks[i].size, 7);
    EXPECT_EQ(all_blocks[i].fixed, pose_blocks[i].fixed);
  }
  EXPECT_EQ(problem.NumInertialStates(), 0u);
  EXPECT_FALSE(problem.HasInertialState("kf0"));
}

// Inertial states come AFTER every pose in the block ordering, so adding
// one cannot shift an existing pose's columns.
TEST(PoseGraphProblem, InertialBlocksAreOrderedAfterAllPoses) {
  PoseGraphProblem problem;
  problem.AddKeyframe("kf0", Pose3{});
  problem.AddKeyframe("kf1", Pose3{});
  problem.AddInertialState("kf0", PoseGraphProblem::InertialState{});
  problem.AddKeyframe("kf2", Pose3{});
  problem.AddInertialState("kf2", PoseGraphProblem::InertialState{});

  const auto blocks = problem.MutableAllParameterBlocks();
  ASSERT_EQ(blocks.size(), 5u);
  EXPECT_EQ(blocks[0].ref.keyframe_id, "kf0");
  EXPECT_EQ(blocks[1].ref.keyframe_id, "kf1");
  EXPECT_EQ(blocks[2].ref.keyframe_id, "kf2");
  for (int i = 0; i < 3; ++i) EXPECT_EQ(blocks[i].ref.kind, PoseGraphProblem::ParameterKind::kPose);
  EXPECT_EQ(blocks[3].ref.kind, PoseGraphProblem::ParameterKind::kInertial);
  EXPECT_EQ(blocks[3].ref.keyframe_id, "kf0");
  EXPECT_EQ(blocks[3].size, 9);
  EXPECT_EQ(blocks[4].ref.kind, PoseGraphProblem::ParameterKind::kInertial);
  EXPECT_EQ(blocks[4].ref.keyframe_id, "kf2");
}

TEST(PoseGraphProblem, InertialStateApiRejectsUnknownKeyframesAndUnbackedRefs) {
  PoseGraphProblem problem;
  problem.AddKeyframe("kf0", Pose3{});
  EXPECT_THROW(problem.AddInertialState("nope", PoseGraphProblem::InertialState{}),
               std::out_of_range);
  EXPECT_THROW(problem.GetInertialState("kf0"), std::out_of_range);
  EXPECT_THROW(problem.SetInertialState("kf0", PoseGraphProblem::InertialState{}),
               std::out_of_range);
  // A ParameterRef naming an inertial state that was never added must be
  // refused at bind time, not silently produce a dangling parameter block.
  EXPECT_THROW(problem.AddResidualBlockOnParameters(
                   std::make_unique<DepthResidual>(1.0, 1.0),
                   {PoseGraphProblem::InertialRef("kf0")}),
               std::out_of_range);
}
