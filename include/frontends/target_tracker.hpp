#pragma once

#include <cstdint>
#include <optional>
#include <set>
#include <string>
#include <type_traits>
#include <vector>

#include <Eigen/Core>

#include "frontends/target_associator.hpp"

namespace uw::frontends {

struct TargetTrackerParams {
  double association_mahalanobis_sq = 16.0;
  int confirm_hits = 2;
  int degraded_misses = 3;
  double stale_after_s = 0.5;
  double max_prediction_dt_s = 0.5;
  double bearing_acceleration_noise = 0.05;
  double range_acceleration_noise = 0.5;
  double merge_bearing_threshold_rad = 0.03;
  double merge_range_threshold_m = 0.30;
};

struct TrackedTarget {
  uint64_t numeric_id = 0;
  std::string track_id;
  std::string class_label;
  double class_confidence = 0.0;
  Eigen::Vector4d state = Eigen::Vector4d::Zero();
  Eigen::Matrix4d covariance = Eigen::Matrix4d::Identity();
  bool range_observable = false;
  double first_capture_time_s = 0.0;
  double last_capture_time_s = 0.0;
  uw::domain::TargetTrackStatus status = uw::domain::TARGET_TRACK_STATUS_TENTATIVE;
  std::vector<uw::domain::AssistSource> sources;
  std::vector<uw::domain::ObservationId> observation_ids;

  // The current wire schema cannot represent absent range (proto3 scalar
  // presence is disabled), so bearing-only tracks fail closed instead of
  // exporting a fabricated zero-metre range.
  std::optional<uw::domain::TargetTrack> ToProto(double publish_time_s) const;
};

class TargetTracker {
 public:
  explicit TargetTracker(TargetTrackerParams params = {});
  ~TargetTracker();

  template <typename Config,
            typename = std::enable_if_t<!std::is_same_v<std::decay_t<Config>,
                                                       TargetTrackerParams>>>
  explicit TargetTracker(const Config& config)
      : TargetTracker(TargetTrackerParams{
            config.association_mahalanobis_sq,
            config.confirm_hits,
            config.degraded_misses,
            config.stale_after_s,
            config.max_prediction_dt_s,
            config.bearing_acceleration_noise,
            config.range_acceleration_noise,
            config.merge_bearing_threshold_rad,
            config.merge_range_threshold_m}) {}

  // A batch is atomic: invalid/non-finite/out-of-order input returns false
  // and does not mutate IDs, tracks, or the monotonic time watermark.
  bool Update(const std::vector<TargetMeasurement>& detections, double now_s);
  std::vector<TrackedTarget> Tracks(double now_s) const;
  std::optional<uw::domain::TargetTrackSet> ToProtoSet(double publish_time_s) const;

 private:
  struct Track;
  TargetTrackerParams params_;
  uint64_t next_track_id_ = 1;
  std::optional<double> last_update_time_s_;
  std::optional<double> last_capture_time_s_;
  std::set<std::string> accepted_observation_ids_;
  std::vector<Track> tracks_;
};

}  // namespace uw::frontends
