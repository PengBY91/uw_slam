#include "evaluation/trajectory_metrics.hpp"

#include <gtest/gtest.h>

using uw::evaluation::ComputeAte;
using uw::evaluation::TrajectoryPose;
using uw::sensor_models::Pose3;

TEST(ComputeAte, ZeroErrorWhenTrajectoriesMatch) {
  std::vector<TrajectoryPose> traj;
  for (int i = 0; i < 5; ++i) {
    Pose3 pose;
    pose.translation = Eigen::Vector3d(i, 0, 0);
    traj.push_back({static_cast<double>(i), pose});
  }
  const auto result = ComputeAte(traj, traj);
  EXPECT_EQ(result.num_matched_poses, 5u);
  EXPECT_NEAR(result.rmse_m, 0.0, 1e-9);
}

TEST(ComputeAte, ReportsKnownOffset) {
  std::vector<TrajectoryPose> estimated, ground_truth;
  for (int i = 0; i < 3; ++i) {
    Pose3 est_pose;
    est_pose.translation = Eigen::Vector3d(i, 1.0, 0.0);  // 1m Y offset
    estimated.push_back({static_cast<double>(i), est_pose});

    Pose3 gt_pose;
    gt_pose.translation = Eigen::Vector3d(i, 0.0, 0.0);
    ground_truth.push_back({static_cast<double>(i), gt_pose});
  }
  const auto result = ComputeAte(estimated, ground_truth);
  EXPECT_EQ(result.num_matched_poses, 3u);
  EXPECT_NEAR(result.rmse_m, 1.0, 1e-9);
}

TEST(ComputeAte, AlignBeforeScoringRemovesAConstantOffsetAndRotation) {
  // Ground truth is a straight line; the "estimated" trajectory is the
  // SAME shape but expressed in a different world frame (rotated 90
  // degrees about Z, then translated far away) — exactly the situation a
  // real recording's pose-graph-local kf0-at-origin anchor is in relative
  // to ground truth's real absolute world position (see this function's
  // header comment). Unaligned, the error is dominated by that offset;
  // aligned, it recovers ~0.
  std::vector<TrajectoryPose> estimated, ground_truth;
  const Eigen::Quaterniond rotate_90_z(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
  const Eigen::Vector3d offset(35.0, -40.0, 0.0);
  for (int i = 0; i < 5; ++i) {
    Pose3 gt_pose;
    gt_pose.translation = Eigen::Vector3d(i, 0.0, 0.0);
    ground_truth.push_back({static_cast<double>(i), gt_pose});

    Pose3 est_pose;
    est_pose.translation = rotate_90_z * gt_pose.translation + offset;
    estimated.push_back({static_cast<double>(i), est_pose});
  }

  const auto unaligned = ComputeAte(estimated, ground_truth, 0.05, /*align_before_scoring=*/false);
  EXPECT_GT(unaligned.rmse_m, 30.0);  // dominated by the ~53m offset, not shape error

  const auto aligned = ComputeAte(estimated, ground_truth, 0.05, /*align_before_scoring=*/true);
  EXPECT_NEAR(aligned.rmse_m, 0.0, 1e-6);
  EXPECT_EQ(aligned.num_matched_poses, 5u);
}

TEST(ComputeAte, AlignBeforeScoringFallsBackGracefullyWithTooFewMatches) {
  std::vector<TrajectoryPose> estimated, ground_truth;
  for (int i = 0; i < 2; ++i) {  // fewer than the 3 points a rigid fit needs
    Pose3 pose;
    pose.translation = Eigen::Vector3d(i, 1.0, 0.0);
    estimated.push_back({static_cast<double>(i), pose});
    Pose3 gt_pose;
    gt_pose.translation = Eigen::Vector3d(i, 0.0, 0.0);
    ground_truth.push_back({static_cast<double>(i), gt_pose});
  }

  const auto result = ComputeAte(estimated, ground_truth, 0.05, /*align_before_scoring=*/true);
  EXPECT_EQ(result.num_matched_poses, 2u);
  EXPECT_NEAR(result.rmse_m, 1.0, 1e-9);  // unaligned fallback, same as the disabled case
}
