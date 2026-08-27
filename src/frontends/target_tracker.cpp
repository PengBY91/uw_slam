#include "frontends/target_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <tuple>
#include <utility>

#include <Eigen/Eigenvalues>
#include <Eigen/Cholesky>

namespace uw::frontends {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kMinVariance = 1e-12;
constexpr double kMaxMeasurementVariance = 1e12;
constexpr double kMaxTargetRangeM = 1e6;
constexpr double kUnobservedRangeVariance = 1e6;

bool ValidStampSeconds(double seconds) {
  return std::isfinite(seconds) && seconds >= 0.0 &&
         static_cast<long double>(seconds) <=
             static_cast<long double>(std::numeric_limits<int64_t>::max()) - 1.0L;
}

double WrapBearing(double bearing) {
  bearing = std::remainder(bearing, kTwoPi);
  if (bearing <= -kPi) bearing += kTwoPi;
  return bearing;
}

bool FinitePositive(double value) { return std::isfinite(value) && value > 0.0; }

bool ValidParams(const TargetTrackerParams& params) {
  return FinitePositive(params.association_mahalanobis_sq) &&
         params.confirm_hits > 0 && params.degraded_misses > 0 &&
         FinitePositive(params.stale_after_s) &&
         FinitePositive(params.max_prediction_dt_s) &&
         FinitePositive(params.bearing_acceleration_noise) &&
         FinitePositive(params.range_acceleration_noise) &&
         FinitePositive(params.merge_bearing_threshold_rad) &&
         FinitePositive(params.merge_range_threshold_m) &&
         FinitePositive(params.retention_after_s) &&
         params.retention_after_s > params.stale_after_s;
}

bool ValidCovariance(const Eigen::Matrix2d& covariance, bool has_range) {
  if (!covariance.allFinite() ||
      !covariance.isApprox(covariance.transpose(), 1e-10) ||
      covariance.cwiseAbs().maxCoeff() > kMaxMeasurementVariance ||
      covariance(0, 0) <= 0.0 || (has_range && covariance(1, 1) <= 0.0)) {
    return false;
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
  return solver.info() == Eigen::Success && solver.eigenvalues().minCoeff() >= -1e-10;
}

bool ValidTrackCovariance(const Eigen::Matrix4d& covariance) {
  if (!covariance.allFinite() ||
      !covariance.isApprox(covariance.transpose(), 1e-10) ||
      covariance.cwiseAbs().maxCoeff() > kMaxMeasurementVariance) {
    return false;
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(covariance);
  return solver.info() == Eigen::Success && solver.eigenvalues().minCoeff() >= -1e-10;
}

bool ValidTrackStatus(uw::domain::TargetTrackStatus status) {
  return status == uw::domain::TARGET_TRACK_STATUS_TENTATIVE ||
         status == uw::domain::TARGET_TRACK_STATUS_CONFIRMED ||
         status == uw::domain::TARGET_TRACK_STATUS_DEGRADED ||
         status == uw::domain::TARGET_TRACK_STATUS_STALE;
}

bool ValidSortedSources(const std::vector<uw::domain::AssistSource>& sources) {
  if (sources.empty() || !std::is_sorted(sources.begin(), sources.end())) return false;
  std::set<int> unique;
  for (auto source : sources) {
    if (source != uw::domain::ASSIST_SOURCE_VISUAL &&
        source != uw::domain::ASSIST_SOURCE_SONAR) {
      return false;
    }
    if (!unique.insert(static_cast<int>(source)).second) return false;
  }
  return true;
}

bool ValidTrackProvenance(
    const std::vector<uw::domain::AssistSource>& sources,
    const std::vector<uw::domain::ObservationId>& observations,
    bool range_observable) {
  const bool visual = std::find(sources.begin(), sources.end(),
                                uw::domain::ASSIST_SOURCE_VISUAL) !=
                      sources.end();
  const bool sonar = std::find(sources.begin(), sources.end(),
                               uw::domain::ASSIST_SOURCE_SONAR) !=
                     sources.end();
  if (visual && !sonar) return !range_observable;
  if (!visual && sonar) return range_observable;
  return visual && sonar && range_observable && observations.size() >= 2;
}

bool ValidSortedObservations(const std::vector<uw::domain::ObservationId>& observations) {
  if (observations.empty()) return false;
  std::string previous;
  for (const auto& observation : observations) {
    if (observation.value().empty() ||
        (!previous.empty() && observation.value() <= previous)) {
      return false;
    }
    previous = observation.value();
  }
  return true;
}

bool ValidMeasurement(const TargetMeasurement& measurement, double now_s,
                      std::optional<double> last_capture_time_s) {
  if (!ValidStampSeconds(measurement.corrected_time_s) ||
      measurement.corrected_time_s > now_s + 1e-9 ||
      (last_capture_time_s && measurement.corrected_time_s < *last_capture_time_s - 1e-9) ||
      measurement.class_label.empty() || !std::isfinite(measurement.confidence) ||
      measurement.confidence < 0.0 || measurement.confidence > 1.0 ||
      !std::isfinite(measurement.bearing_rad) ||
      std::abs(measurement.bearing_rad) > kPi ||
      (measurement.range_m &&
       (!FinitePositive(*measurement.range_m) || *measurement.range_m > kMaxTargetRangeM)) ||
      !ValidCovariance(measurement.covariance, measurement.range_m.has_value()) ||
      measurement.sources.empty() || measurement.observation_ids.empty()) {
    return false;
  }
  std::set<int> sources;
  for (auto source : measurement.sources) {
    if (source != uw::domain::ASSIST_SOURCE_VISUAL &&
        source != uw::domain::ASSIST_SOURCE_SONAR) {
      return false;
    }
    sources.insert(static_cast<int>(source));
  }
  if (sources.size() != measurement.sources.size()) return false;
  std::set<std::string> observations;
  for (const auto& observation : measurement.observation_ids) {
    if (observation.value().empty() || !observations.insert(observation.value()).second) {
      return false;
    }
  }
  const bool visual = sources.count(uw::domain::ASSIST_SOURCE_VISUAL) != 0;
  const bool sonar = sources.count(uw::domain::ASSIST_SOURCE_SONAR) != 0;
  if (visual && !sonar) {
    return !measurement.range_m && observations.size() == 1;
  }
  if (!visual && sonar) {
    return measurement.range_m.has_value() && observations.size() == 1;
  }
  return visual && sonar && measurement.range_m.has_value() &&
         observations.size() == 2;
}

bool GenericClass(const std::string& label) {
  return label == "target" || label == "sonar_target";
}

bool ClassesCompatible(const std::string& lhs, const std::string& rhs) {
  return lhs == rhs || GenericClass(lhs) || GenericClass(rhs);
}

template <typename T, typename Less>
void AddSortedUnique(std::vector<T>* destination, const std::vector<T>& source, Less less) {
  destination->insert(destination->end(), source.begin(), source.end());
  std::sort(destination->begin(), destination->end(), less);
  destination->erase(std::unique(destination->begin(), destination->end(),
                                 [&](const auto& lhs, const auto& rhs) {
                                   return !less(lhs, rhs) && !less(rhs, lhs);
                                 }),
                     destination->end());
}

std::optional<uw::domain::Stamp> ToStamp(double seconds) {
  if (!ValidStampSeconds(seconds)) return std::nullopt;
  uw::domain::Stamp stamp;
  const double integral = std::floor(seconds);
  stamp.set_seconds(static_cast<int64_t>(integral));
  int64_t nanos = static_cast<int64_t>(std::llround((seconds - integral) * 1e9));
  if (nanos == 1'000'000'000) {
    stamp.set_seconds(stamp.seconds() + 1);
    nanos = 0;
  }
  stamp.set_nanos(static_cast<int32_t>(nanos));
  return stamp;
}

Eigen::Matrix4d SanitizeCovariance(const Eigen::Matrix4d& covariance) {
  Eigen::Matrix4d symmetric = 0.5 * (covariance + covariance.transpose());
  if (!symmetric.allFinite()) return Eigen::Matrix4d::Identity() * 1e6;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(symmetric);
  if (solver.info() != Eigen::Success) return Eigen::Matrix4d::Identity() * 1e6;
  Eigen::Vector4d eigenvalues = solver.eigenvalues().cwiseMax(1e-12);
  return solver.eigenvectors() * eigenvalues.asDiagonal() * solver.eigenvectors().transpose();
}

std::string FirstObservation(const TargetMeasurement& measurement) {
  if (measurement.observation_ids.empty()) return {};
  return std::min_element(measurement.observation_ids.begin(), measurement.observation_ids.end(),
                          [](const auto& lhs, const auto& rhs) {
                            return lhs.value() < rhs.value();
                          })->value();
}

}  // namespace

struct TargetTracker::Track {
  TrackedTarget target;
  uint64_t hits = 0;
  uint64_t consecutive_misses = 0;
  double state_time_s = 0.0;
};

namespace {

template <typename TrackT>
void Predict(TrackT* track, double now_s, const TargetTrackerParams& params) {
  const double elapsed = now_s - track->state_time_s;
  if (elapsed <= 0.0) return;
  double remaining = elapsed;
  while (remaining > 0.0) {
    const double dt = std::min(remaining, params.max_prediction_dt_s);
    Eigen::Matrix4d transition = Eigen::Matrix4d::Identity();
    transition(0, 2) = dt;
    transition(1, 3) = dt;
    track->target.state = transition * track->target.state;
    track->target.state[0] = WrapBearing(track->target.state[0]);
    if (track->target.range_observable && track->target.state[1] <= 0.0) {
      track->target.state[1] = 1e-6;
      track->target.state[3] = 0.0;
    }
    Eigen::Matrix4d process_noise = Eigen::Matrix4d::Zero();
    const auto add_cv_noise = [&](int position, int velocity, double sigma) {
      const double variance = sigma * sigma;
      process_noise(position, position) =
          dt * dt * dt * variance / 3.0;
      process_noise(position, velocity) = 0.5 * dt * dt * variance;
      process_noise(velocity, position) =
          process_noise(position, velocity);
      process_noise(velocity, velocity) = dt * variance;
    };
    add_cv_noise(0, 2, params.bearing_acceleration_noise);
    add_cv_noise(1, 3, params.range_acceleration_noise);
    track->target.covariance = SanitizeCovariance(
        transition * track->target.covariance * transition.transpose() +
        process_noise);
    remaining -= dt;
    if (remaining <=
        std::numeric_limits<double>::epsilon() * std::max(1.0, elapsed)) {
      remaining = 0.0;
    }
  }
  track->state_time_s = now_s;
}

template <typename TrackT>
double AssociationCost(const TrackT& track, const TargetMeasurement& measurement) {
  const double bearing_residual = WrapBearing(measurement.bearing_rad - track.target.state[0]);
  if (track.target.range_observable && measurement.range_m) {
    Eigen::Vector2d residual(bearing_residual,
                             *measurement.range_m - track.target.state[1]);
    const Eigen::Matrix2d innovation_covariance =
        track.target.covariance.template topLeftCorner<2, 2>() + measurement.covariance;
    Eigen::LDLT<Eigen::Matrix2d> ldlt(innovation_covariance);
    if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
      return std::numeric_limits<double>::infinity();
    }
    const Eigen::Vector2d whitened = ldlt.solve(residual);
    if (ldlt.info() != Eigen::Success || !whitened.allFinite()) {
      return std::numeric_limits<double>::infinity();
    }
    return residual.dot(whitened);
  }
  return bearing_residual * bearing_residual /
         std::max(kMinVariance, track.target.covariance(0, 0) +
                                    measurement.covariance(0, 0));
}

template <typename TrackT>
void JosephBearingUpdate(TrackT* track, const TargetMeasurement& measurement) {
  Eigen::Matrix<double, 1, 4> observation;
  observation << 1.0, 0.0, 0.0, 0.0;
  const double innovation = WrapBearing(measurement.bearing_rad - track->target.state[0]);
  const double innovation_covariance =
      (observation * track->target.covariance * observation.transpose())(0, 0) +
      measurement.covariance(0, 0);
  const Eigen::Matrix<double, 4, 1> gain =
      track->target.covariance * observation.transpose() / innovation_covariance;
  track->target.state += gain * innovation;
  track->target.state[0] = WrapBearing(track->target.state[0]);
  const Eigen::Matrix4d identity = Eigen::Matrix4d::Identity();
  const Eigen::Matrix4d residual = identity - gain * observation;
  track->target.covariance = SanitizeCovariance(
      residual * track->target.covariance * residual.transpose() +
      gain * measurement.covariance(0, 0) * gain.transpose());
}

template <typename TrackT>
void JosephBearingRangeUpdate(TrackT* track, const TargetMeasurement& measurement) {
  Eigen::Matrix<double, 2, 4> observation = Eigen::Matrix<double, 2, 4>::Zero();
  observation(0, 0) = 1.0;
  observation(1, 1) = 1.0;
  Eigen::Vector2d innovation;
  innovation << WrapBearing(measurement.bearing_rad - track->target.state[0]),
      *measurement.range_m - track->target.state[1];
  const Eigen::Matrix2d innovation_covariance =
      observation * track->target.covariance * observation.transpose() +
      measurement.covariance;
  Eigen::LDLT<Eigen::Matrix2d> ldlt(innovation_covariance);
  const Eigen::Matrix<double, 4, 2> gain =
      track->target.covariance * observation.transpose() *
      ldlt.solve(Eigen::Matrix2d::Identity());
  track->target.state += gain * innovation;
  track->target.state[0] = WrapBearing(track->target.state[0]);
  const Eigen::Matrix4d identity = Eigen::Matrix4d::Identity();
  const Eigen::Matrix4d residual = identity - gain * observation;
  track->target.covariance = SanitizeCovariance(
      residual * track->target.covariance * residual.transpose() +
      gain * measurement.covariance * gain.transpose());
}

template <typename TrackT>
void UpdateTrack(TrackT* track, const TargetMeasurement& measurement,
                 const TargetTrackerParams& params) {
  if (measurement.range_m && !track->target.range_observable) {
    // Center the previously unobservable range state on the first physical
    // measurement, but retain a high-uncertainty prior so the full correlated
    // bearing/range measurement covariance participates in the Joseph update.
    track->target.state[1] = *measurement.range_m;
    track->target.state[3] = 0.0;
    track->target.covariance(1, 1) = std::max(
        track->target.covariance(1, 1), kUnobservedRangeVariance);
    track->target.covariance(3, 3) = std::max(
        track->target.covariance(3, 3), kUnobservedRangeVariance);
    track->target.range_observable = true;
    JosephBearingRangeUpdate(track, measurement);
  } else if (measurement.range_m) {
    JosephBearingRangeUpdate(track, measurement);
  } else {
    JosephBearingUpdate(track, measurement);
  }
  ++track->hits;
  track->consecutive_misses = 0;
  track->target.last_capture_time_s = measurement.corrected_time_s;
  if (!GenericClass(measurement.class_label)) {
    track->target.class_label = measurement.class_label;
  }
  track->target.class_confidence =
      std::max(track->target.class_confidence, measurement.confidence);
  AddSortedUnique(&track->target.sources, measurement.sources,
                  [](auto lhs, auto rhs) { return lhs < rhs; });
  AddSortedUnique(&track->target.observation_ids, measurement.observation_ids,
                  [](const auto& lhs, const auto& rhs) {
                    return lhs.value() < rhs.value();
                  });
  track->target.status =
      track->hits >= static_cast<uint64_t>(params.confirm_hits)
          ? uw::domain::TARGET_TRACK_STATUS_CONFIRMED
          : uw::domain::TARGET_TRACK_STATUS_TENTATIVE;
}

template <typename TrackT>
bool MergeClose(const TrackT& lhs, const TrackT& rhs,
                const TargetTrackerParams& params) {
  if (std::abs(WrapBearing(lhs.target.state[0] - rhs.target.state[0])) >
      params.merge_bearing_threshold_rad) {
    return false;
  }
  if (lhs.target.range_observable && rhs.target.range_observable &&
      std::abs(lhs.target.state[1] - rhs.target.state[1]) >
          params.merge_range_threshold_m) {
    return false;
  }
  return ClassesCompatible(lhs.target.class_label, rhs.target.class_label);
}

template <typename TrackT>
void PromoteRangeFrom(TrackT* destination, const TrackT& ranged_source) {
  if (destination->target.range_observable ||
      !ranged_source.target.range_observable) {
    return;
  }
  destination->target.state[1] = ranged_source.target.state[1];
  destination->target.state[3] = ranged_source.target.state[3];

  // Preserve the older track's bearing subsystem and the ranged track's
  // range subsystem. Zeroing unknown cross-subsystem correlation produces a
  // conservative block-diagonal covariance whose two principal blocks were
  // already PSD, then SanitizeCovariance guards round-off.
  constexpr int kBearingIndices[] = {0, 2};
  constexpr int kRangeIndices[] = {1, 3};
  Eigen::Matrix4d promoted = Eigen::Matrix4d::Zero();
  for (int row : kBearingIndices) {
    for (int col : kBearingIndices) {
      promoted(row, col) = destination->target.covariance(row, col);
    }
  }
  for (int row : kRangeIndices) {
    for (int col : kRangeIndices) {
      promoted(row, col) = ranged_source.target.covariance(row, col);
    }
  }
  destination->target.covariance = SanitizeCovariance(promoted);
  destination->target.range_observable = true;
}

}  // namespace

TargetTracker::TargetTracker(TargetTrackerParams params) : params_(params) {
  if (!ValidParams(params_)) throw std::invalid_argument("invalid TargetTrackerParams");
}

TargetTracker::~TargetTracker() = default;

void TargetTracker::PruneExpired(double reference_time_s) {
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                               [&](const Track& track) {
                                 return reference_time_s - track.target.last_capture_time_s >
                                        params_.retention_after_s;
                               }),
               tracks_.end());
}

bool TargetTracker::Update(const std::vector<TargetMeasurement>& detections,
                           double now_s) {
  if (!ValidStampSeconds(now_s) ||
      (last_update_time_s_ && now_s < *last_update_time_s_ - 1e-9)) {
    return false;
  }
  std::optional<double> committed_state_horizon_s;
  for (const auto& track : tracks_) {
    committed_state_horizon_s =
        std::max(committed_state_horizon_s.value_or(track.state_time_s),
                 track.state_time_s);
  }
  std::set<std::string> batch_observation_ids;
  for (const auto& detection : detections) {
    if (!ValidMeasurement(detection, now_s, last_capture_time_s_) ||
        (committed_state_horizon_s &&
         detection.corrected_time_s < *committed_state_horizon_s - 1e-9)) {
      return false;
    }
    for (const auto& observation : detection.observation_ids) {
      if (accepted_observation_ids_.count(observation.value()) != 0 ||
          !batch_observation_ids.insert(observation.value()).second) {
        return false;
      }
    }
  }

  if (detections.empty()) {
    const double processing_delta_s =
        last_update_time_s_ ? now_s - *last_update_time_s_ : 0.0;
    for (auto& track : tracks_) {
      // Stored state lives in corrected-capture time. Advance it by the
      // processing-clock delta so fixed ingress latency is preserved; a
      // future delayed capture can then still be applied chronologically.
      Predict(&track, track.state_time_s + processing_delta_s, params_);
      ++track.consecutive_misses;
      if (track.consecutive_misses >=
          static_cast<uint64_t>(params_.degraded_misses)) {
        track.target.status = uw::domain::TARGET_TRACK_STATUS_DEGRADED;
      }
    }
    PruneExpired(now_s);
    last_update_time_s_ = now_s;
    return true;
  }

  std::vector<TargetMeasurement> ordered_detections = detections;
  std::sort(ordered_detections.begin(), ordered_detections.end(),
            [](const auto& lhs, const auto& rhs) {
              return std::tie(lhs.corrected_time_s, lhs.bearing_rad) <
                         std::tie(rhs.corrected_time_s, rhs.bearing_rad) ||
                     (lhs.corrected_time_s == rhs.corrected_time_s &&
                      lhs.bearing_rad == rhs.bearing_rad &&
                      FirstObservation(lhs) < FirstObservation(rhs));
            });
  struct Candidate {
    std::size_t track_index = 0;
    std::size_t detection_index = 0;
    double cost = 0.0;
  };
  std::vector<Candidate> candidates;
  for (std::size_t ti = 0; ti < tracks_.size(); ++ti) {
    for (std::size_t di = 0; di < ordered_detections.size(); ++di) {
      if (!ClassesCompatible(tracks_[ti].target.class_label,
                             ordered_detections[di].class_label)) {
        continue;
      }
      Track predicted = tracks_[ti];
      Predict(&predicted, ordered_detections[di].corrected_time_s, params_);
      const double cost = AssociationCost(predicted, ordered_detections[di]);
      if (std::isfinite(cost) && cost <= params_.association_mahalanobis_sq) {
        candidates.push_back({ti, di, cost});
      }
    }
  }
  std::sort(candidates.begin(), candidates.end(), [&](const auto& lhs, const auto& rhs) {
    return std::tie(lhs.cost, tracks_[lhs.track_index].target.numeric_id,
                    lhs.detection_index) <
           std::tie(rhs.cost, tracks_[rhs.track_index].target.numeric_id,
                    rhs.detection_index);
  });

  std::vector<std::optional<std::size_t>> detection_for_track(tracks_.size());
  std::vector<std::optional<std::size_t>> track_for_detection(ordered_detections.size());
  for (const auto& candidate : candidates) {
    if (!detection_for_track[candidate.track_index] &&
        !track_for_detection[candidate.detection_index]) {
      detection_for_track[candidate.track_index] = candidate.detection_index;
      track_for_detection[candidate.detection_index] = candidate.track_index;
    }
  }

  // A single observation that gates to multiple already-close tracks is a
  // deterministic merge. Reassign it to the oldest ID even if a younger
  // track had a marginally lower floating-point cost.
  std::vector<bool> merged(tracks_.size(), false);
  for (std::size_t di = 0; di < ordered_detections.size(); ++di) {
    if (!track_for_detection[di] || merged[*track_for_detection[di]]) continue;
    std::vector<std::size_t> close_tracks{*track_for_detection[di]};
    for (const auto& candidate : candidates) {
      if (candidate.detection_index != di ||
          candidate.track_index == *track_for_detection[di] ||
          merged[candidate.track_index] ||
          detection_for_track[candidate.track_index].has_value()) {
        continue;
      }
      Track candidate_at_detection = tracks_[candidate.track_index];
      Track selected_at_detection = tracks_[*track_for_detection[di]];
      Predict(&candidate_at_detection, ordered_detections[di].corrected_time_s, params_);
      Predict(&selected_at_detection, ordered_detections[di].corrected_time_s, params_);
      if (MergeClose(candidate_at_detection, selected_at_detection, params_)) {
        close_tracks.push_back(candidate.track_index);
      }
    }
    if (close_tracks.size() < 2) continue;
    const auto oldest = *std::min_element(
        close_tracks.begin(), close_tracks.end(), [&](std::size_t lhs, std::size_t rhs) {
          return tracks_[lhs].target.numeric_id < tracks_[rhs].target.numeric_id;
        });
    for (std::size_t index : close_tracks) {
      detection_for_track[index].reset();
      if (index == oldest) continue;
      merged[index] = true;
      PromoteRangeFrom(&tracks_[oldest], tracks_[index]);
      AddSortedUnique(&tracks_[oldest].target.sources, tracks_[index].target.sources,
                      [](auto lhs, auto rhs) { return lhs < rhs; });
      AddSortedUnique(&tracks_[oldest].target.observation_ids,
                      tracks_[index].target.observation_ids,
                      [](const auto& lhs, const auto& rhs) {
                        return lhs.value() < rhs.value();
                      });
      tracks_[oldest].hits = std::max(tracks_[oldest].hits, tracks_[index].hits);
      tracks_[oldest].target.first_capture_time_s =
          std::min(tracks_[oldest].target.first_capture_time_s,
                   tracks_[index].target.first_capture_time_s);
    }
    detection_for_track[oldest] = di;
    track_for_detection[di] = oldest;
  }

  for (std::size_t ti = 0; ti < tracks_.size(); ++ti) {
    if (merged[ti]) continue;
    if (detection_for_track[ti]) {
      Predict(&tracks_[ti],
              ordered_detections[*detection_for_track[ti]].corrected_time_s, params_);
      UpdateTrack(&tracks_[ti], ordered_detections[*detection_for_track[ti]], params_);
    } else {
      ++tracks_[ti].consecutive_misses;
      if (tracks_[ti].consecutive_misses >=
          static_cast<uint64_t>(params_.degraded_misses)) {
        tracks_[ti].target.status = uw::domain::TARGET_TRACK_STATUS_DEGRADED;
      }
    }
  }

  std::vector<Track> surviving_tracks;
  surviving_tracks.reserve(tracks_.size());
  for (std::size_t index = 0; index < tracks_.size(); ++index) {
    if (!merged[index]) surviving_tracks.push_back(std::move(tracks_[index]));
  }
  tracks_ = std::move(surviving_tracks);

  for (std::size_t di = 0; di < ordered_detections.size(); ++di) {
    if (track_for_detection[di]) continue;
    const auto& detection = ordered_detections[di];
    Track track;
    track.target.numeric_id = next_track_id_++;
    track.target.track_id = "track_" + std::to_string(track.target.numeric_id);
    track.target.class_label = detection.class_label;
    track.target.class_confidence = detection.confidence;
    track.target.state << WrapBearing(detection.bearing_rad),
        detection.range_m.value_or(0.0), 0.0, 0.0;
    track.target.covariance = Eigen::Matrix4d::Zero();
    track.target.covariance(0, 0) = detection.covariance(0, 0);
    track.target.covariance(1, 1) =
        detection.range_m ? detection.covariance(1, 1)
                          : kUnobservedRangeVariance;
    track.target.covariance(2, 2) = 1.0;
    track.target.covariance(3, 3) =
        detection.range_m ? 1.0 : kUnobservedRangeVariance;
    if (detection.range_m) {
      track.target.covariance(0, 1) = detection.covariance(0, 1);
      track.target.covariance(1, 0) = detection.covariance(1, 0);
      track.target.covariance = SanitizeCovariance(track.target.covariance);
    }
    track.target.range_observable = detection.range_m.has_value();
    track.target.first_capture_time_s = detection.corrected_time_s;
    track.target.last_capture_time_s = detection.corrected_time_s;
    track.target.status = params_.confirm_hits == 1
                              ? uw::domain::TARGET_TRACK_STATUS_CONFIRMED
                              : uw::domain::TARGET_TRACK_STATUS_TENTATIVE;
    track.target.sources = detection.sources;
    std::sort(track.target.sources.begin(), track.target.sources.end());
    track.target.observation_ids = detection.observation_ids;
    std::sort(track.target.observation_ids.begin(), track.target.observation_ids.end(),
              [](const auto& lhs, const auto& rhs) {
                return lhs.value() < rhs.value();
              });
    track.hits = 1;
    track.state_time_s = detection.corrected_time_s;
    tracks_.push_back(std::move(track));
  }
  double batch_capture_time_s = last_capture_time_s_.value_or(0.0);
  for (const auto& detection : ordered_detections) {
    batch_capture_time_s = std::max(batch_capture_time_s, detection.corrected_time_s);
  }
  if (!ordered_detections.empty()) {
    for (auto& track : tracks_) Predict(&track, batch_capture_time_s, params_);
    last_capture_time_s_ = batch_capture_time_s;
    accepted_observation_ids_.insert(batch_observation_ids.begin(),
                                     batch_observation_ids.end());
  }
  PruneExpired(batch_capture_time_s);
  std::sort(tracks_.begin(), tracks_.end(), [](const auto& lhs, const auto& rhs) {
    return lhs.target.numeric_id < rhs.target.numeric_id;
  });
  last_update_time_s_ = now_s;
  return true;
}

std::vector<TrackedTarget> TargetTracker::Tracks(double now_s) const {
  if (!ValidStampSeconds(now_s) ||
      (last_update_time_s_ && now_s < *last_update_time_s_ - 1e-9)) {
    return {};
  }
  std::vector<TrackedTarget> output;
  output.reserve(tracks_.size());
  for (const auto& stored : tracks_) {
    Track predicted = stored;
    Predict(&predicted, now_s, params_);
    const double age = now_s - predicted.target.last_capture_time_s;
    if (age - params_.stale_after_s > 1e-12) {
      predicted.target.status = uw::domain::TARGET_TRACK_STATUS_STALE;
    }
    output.push_back(std::move(predicted.target));
  }
  return output;
}

std::optional<uw::domain::TargetTrackSet> TargetTracker::ToProtoSet(
    double publish_time_s) const {
  const auto publish_stamp = ToStamp(publish_time_s);
  if (!publish_stamp) return std::nullopt;
  const auto tracks = Tracks(publish_time_s);
  if (!tracks_.empty() && tracks.empty()) return std::nullopt;
  uw::domain::TargetTrackSet output;
  *output.mutable_publish_time() = *publish_stamp;
  for (const auto& track : tracks) {
    const auto proto = track.ToProto(publish_time_s);
    if (!proto) return std::nullopt;
    *output.add_tracks() = *proto;
  }
  return output;
}

std::optional<uw::domain::TargetTrack> TrackedTarget::ToProto(
    double publish_time_s) const {
  const auto first_stamp = ToStamp(first_capture_time_s);
  const auto last_stamp = ToStamp(last_capture_time_s);
  const auto publish_stamp = ToStamp(publish_time_s);
  if (numeric_id == 0 || track_id.empty() || class_label.empty() ||
      !std::isfinite(class_confidence) || class_confidence < 0.0 || class_confidence > 1.0 ||
      !state.allFinite() || std::abs(state[0]) > kPi ||
      (range_observable &&
       (!FinitePositive(state[1]) || state[1] > kMaxTargetRangeM)) ||
      !ValidTrackCovariance(covariance) ||
      !first_stamp || !last_stamp || !publish_stamp ||
      first_capture_time_s > last_capture_time_s || last_capture_time_s > publish_time_s ||
      !ValidTrackStatus(status) || !ValidSortedSources(sources) ||
      !ValidSortedObservations(observation_ids) ||
      !ValidTrackProvenance(sources, observation_ids, range_observable)) {
    return std::nullopt;
  }
  uw::domain::TargetTrack proto;
  proto.mutable_track_id()->set_value(track_id);
  proto.set_class_label(class_label);
  proto.set_class_confidence(class_confidence);
  proto.set_bearing_rad(WrapBearing(state[0]));
  if (range_observable) proto.set_range_m(state[1]);
  proto.add_covariance_2x2_row_major(covariance(0, 0));
  proto.add_covariance_2x2_row_major(covariance(0, 1));
  proto.add_covariance_2x2_row_major(covariance(1, 0));
  proto.add_covariance_2x2_row_major(covariance(1, 1));
  *proto.mutable_first_capture_time() = *first_stamp;
  *proto.mutable_last_capture_time() = *last_stamp;
  *proto.mutable_publish_time() = *publish_stamp;
  for (auto source : sources) proto.add_sources(source);
  for (const auto& observation : observation_ids) {
    *proto.add_source_observations() = observation;
  }
  proto.set_status(status);
  return proto;
}

}  // namespace uw::frontends
