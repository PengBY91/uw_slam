#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include "adapters/ceres/ceres_pose_graph_solver.hpp"
#include "estimation/pose_graph_problem.hpp"
#include "factor_builders/depth_residual.hpp"
#include "factor_builders/imu_preintegration_residual.hpp"
#include "factor_builders/relative_pose_residual.hpp"
#include "sensor_models/so3.hpp"

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
      std::make_unique<RelativePoseResidual>(relative_01, Eigen::Matrix<double, 6, 6>::Identity() * 10.0),
      {"kf0", "kf1"});
  problem.AddResidualBlock(
      std::make_unique<RelativePoseResidual>(relative_12, Eigen::Matrix<double, 6, 6>::Identity() * 10.0),
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

// PREP-B-01: the 9-dim inertial parameter blocks must reach Ceres as plain
// Euclidean blocks (no manifold) while poses keep the quaternion manifold.
// Mirrors tests/estimation/pose_graph_solver_test.cpp's
// PoseGraphSolver.ImuOnlyChainRecoversPosesAndInertialStates so the two
// backends are checked against the same ground truth — the residual's
// quaternion columns are chained so that Ceres's manifold projection
// recovers exactly the minimal Jacobian the hand-rolled solver uses.
TEST(CeresPoseGraphSolver, ImuOnlyChainRecoversPosesAndInertialStates) {
  namespace so3 = uw::sensor_models::so3;
  using uw::factor_builders::ImuPreintegrationResidual;
  using uw::sensor_models::PreintegratedImuDelta;

  Pose3 kf0_pose;
  kf0_pose.translation = Eigen::Vector3d(0.4, -0.2, -2.0);
  kf0_pose.rotation = so3::ExpQuaternion(Eigen::Vector3d(0.02, -0.05, 0.10));
  PoseGraphProblem::InertialState kf0_inertial;
  kf0_inertial.velocity_W = Eigen::Vector3d(0.9, 0.1, -0.05);
  kf0_inertial.bias_gyro = Eigen::Vector3d(0.0012, -0.0004, 0.0009);
  kf0_inertial.bias_accel = Eigen::Vector3d(0.011, -0.023, 0.006);

  PreintegratedImuDelta delta;
  delta.delta_time_s = 0.5;
  delta.sample_count = 100;
  delta.gravity_mps2 = 9.80665;
  delta.delta_rotation = so3::Exp(Eigen::Vector3d(0.01, -0.02, 0.06));
  delta.delta_velocity = Eigen::Vector3d(0.30, -0.05, 0.08);
  delta.delta_position = Eigen::Vector3d(0.075, -0.012, 0.020);
  delta.bias_gyro = kf0_inertial.bias_gyro;
  delta.bias_accel = kf0_inertial.bias_accel;
  delta.d_rotation_d_bias_gyro = -0.5 * Eigen::Matrix3d::Identity();
  delta.d_velocity_d_bias_gyro = 0.01 * Eigen::Matrix3d::Identity();
  delta.d_velocity_d_bias_accel = -0.5 * Eigen::Matrix3d::Identity();
  delta.d_position_d_bias_gyro = 0.0025 * Eigen::Matrix3d::Identity();
  delta.d_position_d_bias_accel = -0.125 * Eigen::Matrix3d::Identity();
  delta.covariance = Eigen::Matrix<double, 15, 15>::Identity();

  const Eigen::Matrix3d R_0 = kf0_pose.rotation.toRotationMatrix();
  const double dt = delta.delta_time_s;
  const Eigen::Vector3d gravity(0.0, 0.0, -delta.gravity_mps2);
  Pose3 true_kf1;
  true_kf1.rotation = Eigen::Quaterniond(R_0 * delta.delta_rotation).normalized();
  true_kf1.translation = kf0_pose.translation + kf0_inertial.velocity_W * dt +
                         0.5 * gravity * dt * dt + R_0 * delta.delta_position;
  const Eigen::Vector3d true_v1 =
      kf0_inertial.velocity_W + gravity * dt + R_0 * delta.delta_velocity;

  PoseGraphProblem problem;
  problem.AddKeyframe("kf0", kf0_pose, /*fixed=*/true);
  problem.AddInertialState("kf0", kf0_inertial, /*fixed=*/true);
  Pose3 init_kf1 = true_kf1;
  init_kf1.translation += Eigen::Vector3d(0.20, -0.15, 0.10);
  init_kf1.rotation = (init_kf1.rotation * so3::ExpQuaternion({0.03, 0.02, -0.04})).normalized();
  problem.AddKeyframe("kf1", init_kf1);
  problem.AddInertialState("kf1", PoseGraphProblem::InertialState{});
  problem.AddResidualBlockOnParameters(
      std::make_unique<ImuPreintegrationResidual>(delta,
                                                  Eigen::Matrix<double, 15, 15>::Identity()),
      {PoseGraphProblem::PoseRef("kf0"), PoseGraphProblem::InertialRef("kf0"),
       PoseGraphProblem::PoseRef("kf1"), PoseGraphProblem::InertialRef("kf1")});

  CeresPoseGraphSolver solver;
  const auto summary = solver.Solve(problem);
  EXPECT_LT(summary.final_cost, 1e-14);

  const Pose3 solved = problem.GetKeyframePose("kf1");
  EXPECT_LT((solved.translation - true_kf1.translation).norm(), 1e-6);
  EXPECT_LT(so3::Log(solved.rotation.toRotationMatrix().transpose() *
                     true_kf1.rotation.toRotationMatrix())
                .norm(),
            1e-6);
  const auto solved_inertial = problem.GetInertialState("kf1");
  EXPECT_LT((solved_inertial.velocity_W - true_v1).norm(), 1e-6);
  EXPECT_LT((solved_inertial.bias_gyro - kf0_inertial.bias_gyro).norm(), 1e-7);
  EXPECT_LT((solved_inertial.bias_accel - kf0_inertial.bias_accel).norm(), 1e-7);
  // The anchor's fixed inertial block must be untouched.
  const auto anchor = problem.GetInertialState("kf0");
  EXPECT_LT((anchor.velocity_W - kf0_inertial.velocity_W).norm(), 1e-15);
}
