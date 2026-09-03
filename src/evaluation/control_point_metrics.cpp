#include "evaluation/control_point_metrics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <set>
#include <unordered_map>
#include <utility>

#include <Eigen/Dense>

namespace uw::evaluation {

namespace {

const TrajectoryPose* FindNearestPose(const std::vector<TrajectoryPose>& estimated,
                                       double timestamp_s, double max_time_diff_s) {
  const TrajectoryPose* best = nullptr;
  double best_diff = std::numeric_limits<double>::max();
  for (const auto& pose : estimated) {
    const double diff = std::abs(pose.timestamp_s - timestamp_s);
    if (diff < best_diff) {
      best_diff = diff;
      best = &pose;
    }
  }
  if (best == nullptr || best_diff > max_time_diff_s) return nullptr;
  return best;
}

// Same Kabsch/Umeyama math as trajectory_metrics.cpp's FitRigidAlignment,
// deliberately duplicated for the same reason that one is: `evaluation`
// must not depend on `frontends` (tools/lint/check_layer_dependencies.py),
// and a shared helper would have to live somewhere both can see. Here the
// correspondences are predicted-vs-surveyed marker positions rather than
// estimated-vs-ground-truth trajectory points.
std::optional<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> FitRigidAlignment(
    const std::vector<Eigen::Vector3d>& from, const std::vector<Eigen::Vector3d>& to) {
  if (from.size() < 3 || from.size() != to.size()) return std::nullopt;

  Eigen::Vector3d centroid_from = Eigen::Vector3d::Zero();
  Eigen::Vector3d centroid_to = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i < from.size(); ++i) {
    centroid_from += from[i];
    centroid_to += to[i];
  }
  centroid_from /= static_cast<double>(from.size());
  centroid_to /= static_cast<double>(to.size());

  Eigen::Matrix3d h = Eigen::Matrix3d::Zero();
  for (std::size_t i = 0; i < from.size(); ++i) {
    h += (from[i] - centroid_from) * (to[i] - centroid_to).transpose();
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
  return std::make_pair(rotation, centroid_to - rotation * centroid_from);
}

double Percentile95(std::vector<double> sorted_errors) {
  if (sorted_errors.empty()) return 0.0;
  std::sort(sorted_errors.begin(), sorted_errors.end());
  // Nearest-rank p95, the same convention the realtime gate and
  // run_report use for latency percentiles — with the small samples this
  // metric works on (a handful of markers x a handful of sightings),
  // interpolating between order statistics would imply a precision the
  // sample size does not support.
  const std::size_t rank = static_cast<std::size_t>(
      std::ceil(0.95 * static_cast<double>(sorted_errors.size())));
  return sorted_errors[std::min(rank, sorted_errors.size()) - 1];
}

}  // namespace

ControlPointResult ComputeControlPointError(const std::vector<TrajectoryPose>& estimated,
                                            const std::vector<ControlPoint>& control_points,
                                            const std::vector<ControlPointObservation>& observations,
                                            double max_time_diff_s, bool align_before_scoring) {
  ControlPointResult result;
  if (estimated.empty() || control_points.empty() || observations.empty()) {
    result.num_unmatched_observations = observations.size();
    return result;
  }

  std::unordered_map<std::string, const ControlPoint*> by_tag;
  for (const auto& point : control_points) by_tag.emplace(point.tag, &point);

  struct Scored {
    ControlPointError row;
  };
  std::vector<Scored> scored;
  std::vector<Eigen::Vector3d> predicted;
  std::vector<Eigen::Vector3d> surveyed;

  for (const auto& observation : observations) {
    auto it = by_tag.find(observation.tag);
    if (it == by_tag.end()) {
      // An observation of a marker the scenario never declared is a
      // labelling error, and counting it separately keeps it from hiding
      // inside a plausible-looking RMSE.
      ++result.num_unknown_tags;
      ++result.num_unmatched_observations;
      continue;
    }
    const TrajectoryPose* pose = FindNearestPose(estimated, observation.timestamp_s, max_time_diff_s);
    if (pose == nullptr) {
      ++result.num_unmatched_observations;
      continue;
    }
    ControlPointError row;
    row.tag = observation.tag;
    row.timestamp_s = observation.timestamp_s;
    row.matched_pose_time_s = pose->timestamp_s;
    row.predicted_W = pose->pose_WB.Apply(observation.position_B);
    row.surveyed_W = it->second->position_W;
    scored.push_back(Scored{row});
    predicted.push_back(row.predicted_W);
    surveyed.push_back(row.surveyed_W);
  }

  if (scored.empty()) return result;

  std::optional<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> alignment;
  if (align_before_scoring) {
    std::set<std::string> distinct_tags;
    for (const auto& entry : scored) distinct_tags.insert(entry.row.tag);
    // Three non-collinear markers are the minimum that pins a rigid
    // transform; three sightings of ONE marker pin nothing, so count
    // distinct tags rather than observations.
    if (distinct_tags.size() >= 3) alignment = FitRigidAlignment(predicted, surveyed);
  }

  std::set<std::string> covered;
  std::vector<double> errors;
  errors.reserve(scored.size());
  double sum = 0.0;
  double sum_sq = 0.0;
  for (auto& entry : scored) {
    if (alignment.has_value()) {
      entry.row.predicted_W = alignment->first * entry.row.predicted_W + alignment->second;
    }
    entry.row.error_m = (entry.row.predicted_W - entry.row.surveyed_W).norm();
    errors.push_back(entry.row.error_m);
    sum += entry.row.error_m;
    sum_sq += entry.row.error_m * entry.row.error_m;
    result.max_m = std::max(result.max_m, entry.row.error_m);
    covered.insert(entry.row.tag);
    result.per_observation.push_back(entry.row);
  }

  const double count = static_cast<double>(errors.size());
  result.num_scored_observations = errors.size();
  result.num_covered_control_points = covered.size();
  result.mean_m = sum / count;
  result.rmse_m = std::sqrt(sum_sq / count);
  result.p95_m = Percentile95(errors);
  return result;
}

}  // namespace uw::evaluation
