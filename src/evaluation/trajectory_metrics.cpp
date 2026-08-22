#include "evaluation/trajectory_metrics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

#include <Eigen/SVD>

namespace uw::evaluation {
namespace {

struct Correspondence {
  Eigen::Vector3d estimated;
  Eigen::Vector3d ground_truth;
};

std::vector<Correspondence> MatchByTimestamp(const std::vector<TrajectoryPose>& estimated,
                                              const std::vector<TrajectoryPose>& ground_truth,
                                              double max_time_diff_s) {
  std::vector<Correspondence> matches;
  for (const auto& est : estimated) {
    // Nearest-neighbor match by timestamp (ground truth is small in v1
    // synthetic scenarios; linear scan is fine).
    double best_diff = std::numeric_limits<double>::infinity();
    const TrajectoryPose* best = nullptr;
    for (const auto& gt : ground_truth) {
      const double diff = std::abs(gt.timestamp_s - est.timestamp_s);
      if (diff < best_diff) {
        best_diff = diff;
        best = &gt;
      }
    }
    if (best == nullptr || best_diff > max_time_diff_s) continue;
    matches.push_back({est.pose_WB.translation, best->pose_WB.translation});
  }
  return matches;
}

// Kabsch/Umeyama rigid (no-scale) alignment: returns (R, t) minimizing
// sum ||ground_truth_i - (R * estimated_i + t)||^2. Re-implemented here
// rather than reusing frontends::FitRigidTransform's identical math —
// evaluation must not depend on frontends (tools/lint/
// check_layer_dependencies.py enforces this: both are independent
// consumers of `core`, not of each other).
std::optional<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> FitRigidAlignment(
    const std::vector<Correspondence>& matches) {
  if (matches.size() < 3) return std::nullopt;

  Eigen::Vector3d centroid_est = Eigen::Vector3d::Zero();
  Eigen::Vector3d centroid_gt = Eigen::Vector3d::Zero();
  for (const auto& m : matches) {
    centroid_est += m.estimated;
    centroid_gt += m.ground_truth;
  }
  centroid_est /= static_cast<double>(matches.size());
  centroid_gt /= static_cast<double>(matches.size());

  Eigen::Matrix3d h = Eigen::Matrix3d::Zero();
  for (const auto& m : matches) {
    h += (m.estimated - centroid_est) * (m.ground_truth - centroid_gt).transpose();
  }

  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(h, Eigen::ComputeFullU | Eigen::ComputeFullV);
  if (svd.info() != Eigen::Success) return std::nullopt;

  Eigen::Matrix3d rotation = svd.matrixV() * svd.matrixU().transpose();
  if (rotation.determinant() < 0.0) {
    Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
    correction(2, 2) = -1.0;
    rotation = svd.matrixV() * correction * svd.matrixU().transpose();
  }
  if (!rotation.allFinite()) return std::nullopt;

  const Eigen::Vector3d translation = centroid_gt - rotation * centroid_est;
  return std::make_pair(rotation, translation);
}

}  // namespace

AteResult ComputeAte(const std::vector<TrajectoryPose>& estimated,
                     const std::vector<TrajectoryPose>& ground_truth, double max_time_diff_s,
                     bool align_before_scoring) {
  AteResult result;
  if (estimated.empty() || ground_truth.empty()) return result;

  const auto matches = MatchByTimestamp(estimated, ground_truth, max_time_diff_s);
  if (matches.empty()) return result;

  std::optional<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> alignment;
  if (align_before_scoring) alignment = FitRigidAlignment(matches);

  double sum_sq = 0.0, sum = 0.0, max_error = 0.0;
  for (const auto& m : matches) {
    const Eigen::Vector3d aligned_estimate =
        alignment.has_value() ? (alignment->first * m.estimated + alignment->second) : m.estimated;
    const double error = (aligned_estimate - m.ground_truth).norm();
    sum += error;
    sum_sq += error * error;
    max_error = std::max(max_error, error);
  }

  result.num_matched_poses = matches.size();
  result.mean_m = sum / static_cast<double>(matches.size());
  result.rmse_m = std::sqrt(sum_sq / static_cast<double>(matches.size()));
  result.max_m = max_error;
  return result;
}

}  // namespace uw::evaluation
