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

// `align_before_scoring` (default false, preserving prior behavior exactly
// for every existing caller): when true, first fits a single rigid
// transform (rotation + translation, NO scale — Kabsch/Umeyama SVD over
// the time-matched translations) that best maps the ESTIMATED trajectory
// onto ground truth, then scores the aligned estimate. Needed once
// estimated and ground truth no longer share a common world frame/origin
// by construction: synth_bag_gen's synthetic scenarios do (the fixed kf0
// anchor and the fabricated ground truth both start at the same place by
// design), but a real recording does not — apps/replay_demo's kf0 anchor
// sits at x=y=0 by pose-graph convention while a real HoloOcean/PoseSensor
// ground truth carries the actual absolute world position (tens of
// meters away), which would otherwise dominate the reported error with a
// near-constant offset that has nothing to do with estimation accuracy —
// confirmed in practice: without alignment, a real-data run's ATE matched
// the raw kf0-to-ground-truth-origin distance almost exactly. Returns
// unaligned results (rather than failing) if there are fewer than 3
// matched poses to fit an alignment from, or if the fit is degenerate.
AteResult ComputeAte(const std::vector<TrajectoryPose>& estimated,
                     const std::vector<TrajectoryPose>& ground_truth, double max_time_diff_s = 0.05,
                     bool align_before_scoring = false);

}  // namespace uw::evaluation
