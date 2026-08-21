#pragma once

#include <vector>

#include "sensor_models/geometry.hpp"

namespace uw::evaluation {

struct TrajectoryPose {
  double timestamp_s = 0.0;
  uw::sensor_models::Pose3 pose_WB;
};

struct AteResult {
  double rmse_m = 0.0;
  double mean_m = 0.0;
  double max_m = 0.0;
  std::size_t num_matched_poses = 0;
};

// v1 limitation (documented, not hidden): this does NOT perform Umeyama
// SE3/Sim3 alignment before comparing — it assumes estimated and ground
// truth trajectories already share the same world frame and scale, which
// holds for the synthetic replay_demo scenario (both are expressed in the
// same fabricated world frame) but would NOT hold for a real VIO/SLAM
// trajectory compared against a separately-surveyed ground truth. Adding
// alignment is a natural follow-up once evaluation runs against real data
// (platform architecture section 15).
AteResult ComputeAte(const std::vector<TrajectoryPose>& estimated,
                     const std::vector<TrajectoryPose>& ground_truth, double max_time_diff_s = 0.05);

}  // namespace uw::evaluation
