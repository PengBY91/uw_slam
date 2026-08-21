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
