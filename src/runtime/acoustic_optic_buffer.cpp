#include "runtime/acoustic_optic_buffer.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <google/protobuf/util/message_differencer.h>

namespace uw::runtime {
namespace {

constexpr int32_t kNanosPerSecond = 1'000'000'000;
constexpr std::size_t kDeltaWindowSize = 256;
using TimeTick = __int128_t;  // half-nanosecond ticks

TimeTick TickAbs(TimeTick value) { return value < 0 ? -value : value; }

bool WithinWindow(TimeTick delta, double window_seconds) {
  return static_cast<long double>(delta) <=
         static_cast<long double>(window_seconds) * 2.0L * kNanosPerSecond;
}

double TickSeconds(TimeTick ticks) {
  return static_cast<double>(static_cast<long double>(ticks) /
                             (2.0L * kNanosPerSecond));
}

std::optional<int64_t> OffsetNanoseconds(double offset_seconds) {
  if (!std::isfinite(offset_seconds) ||
      std::abs(offset_seconds) > kMaxAbsoluteSensorTimeOffsetSeconds) {
    return std::nullopt;
  }
  return static_cast<int64_t>(std::llround(
      static_cast<long double>(offset_seconds) * kNanosPerSecond));
}

std::optional<uw::domain::Stamp> StampFromHalfNanoseconds(TimeTick half_nanoseconds) {
  // Canonical Stamp cannot represent a half nanosecond. Midpoint ties are
  // rounded toward the earlier instant, including for negative epochs.
  TimeTick total_nanoseconds = half_nanoseconds >= 0
                                   ? half_nanoseconds / 2
                                   : -((-half_nanoseconds + 1) / 2);
  TimeTick seconds = total_nanoseconds / kNanosPerSecond;
  TimeTick nanos = total_nanoseconds % kNanosPerSecond;
  if (nanos < 0) {
    --seconds;
    nanos += kNanosPerSecond;
  }
  if (seconds < std::numeric_limits<int64_t>::min() ||
      seconds > std::numeric_limits<int64_t>::max()) {
    return std::nullopt;
  }
  uw::domain::Stamp stamp;
  stamp.set_seconds(static_cast<int64_t>(seconds));
  stamp.set_nanos(static_cast<int32_t>(nanos));
  return stamp;
}

bool ValidStamp(const uw::domain::ObservationHeader& header) {
  return header.has_capture_time() && header.capture_time().nanos() >= 0 &&
         header.capture_time().nanos() < kNanosPerSecond;
}

void ValidateConfig(const AcousticOpticBufferConfig& config) {
  const auto finite_nonnegative = [](double value) {
    return std::isfinite(value) && value >= 0.0;
  };
  if (!finite_nonnegative(config.max_stereo_delta_s) ||
      !finite_nonnegative(config.max_sonar_camera_delta_s) ||
      !finite_nonnegative(config.max_state_bracket_s) ||
      !finite_nonnegative(config.max_residence_s) ||
      config.max_images_per_camera == 0 || config.max_sonar_frames == 0 ||
      config.max_vehicle_states == 0) {
    throw std::invalid_argument("acoustic-optic buffer limits must be finite/non-negative with positive capacities");
  }
}

std::set<std::string> EnabledSonars(const uw::domain::RigCalibrationSnapshot& rig) {
  std::set<std::string> result;
  for (const auto& sonar : rig.sonar_beam_models()) {
    if (sonar.sonar_enabled() && !sonar.sensor_id().value().empty()) {
      result.insert(sonar.sensor_id().value());
    }
  }
  return result;
}

void ValidateRig(const uw::domain::RigCalibrationSnapshot& rig) {
  std::size_t enabled_sonar_count = 0;
  for (const auto& sonar : rig.sonar_beam_models()) {
    if (sonar.sonar_enabled()) ++enabled_sonar_count;
  }
  if (rig.calibration_version().value().empty() || rig.cameras_size() != 2 ||
      rig.vehicle_state_sensors_size() != 1 || enabled_sonar_count != 1) {
    throw std::invalid_argument(
        "online acoustic-optic rig requires a version, two cameras, exactly one enabled sonar, "
        "and exactly one vehicle-state source");
  }
  std::set<std::string> sensors;
  for (const auto& camera : rig.cameras()) sensors.insert(camera.sensor_id().value());
  for (const auto& sonar : EnabledSonars(rig)) sensors.insert(sonar);
  sensors.insert(rig.vehicle_state_sensors(0).value());
  if (sensors.size() != static_cast<std::size_t>(rig.cameras_size()) +
                            EnabledSonars(rig).size() + 1U || sensors.count("") != 0) {
    throw std::invalid_argument("online acoustic-optic sensor roles must be non-empty and unique");
  }
  for (const auto& sensor : sensors) {
    const auto offset = rig.time_offset_seconds().find(sensor);
    const auto provenance = rig.time_offset_provenance().find(sensor);
    if (offset == rig.time_offset_seconds().end() || !OffsetNanoseconds(offset->second) ||
        provenance == rig.time_offset_provenance().end() || provenance->second.empty()) {
      throw std::invalid_argument("online sensor lacks measured time offset/provenance: " + sensor);
    }
  }
}

template <typename Message>
struct Timed {
  TimeTick corrected_time = 0;
  uint64_t sequence = 0;
  Message message;
};

template <typename Message>
bool TimedLess(const Timed<Message>& lhs, const Timed<Message>& rhs) {
  return std::tie(lhs.corrected_time, lhs.sequence,
                  lhs.message.header().sensor_id().value()) <
         std::tie(rhs.corrected_time, rhs.sequence,
                  rhs.message.header().sensor_id().value());
}

double Percentile(std::vector<double> values, double percentile) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const auto rank = static_cast<std::size_t>(std::ceil(percentile * values.size()));
  return values[rank - 1];
}

bool FiniteState(const uw::domain::VehicleState& state) {
  if (state.orientation_xyzw_size() != 4 || state.angular_velocity_radps_size() != 3 ||
      state.covariance_7x7_row_major_size() != 49 ||
      !state.attitude_valid() || !state.depth_valid() || !state.device_health_valid() ||
      !std::isfinite(state.depth_m()) || state.depth_m() < 0.0 ||
      !std::isfinite(state.supply_voltage_v()) || state.supply_voltage_v() <= 0.0 ||
      !std::isfinite(state.supply_current_a()) || state.supply_current_a() <= 0.0 ||
      !std::isfinite(state.link_quality()) || state.link_quality() < 0.0 ||
      state.link_quality() > 1.0) {
    return false;
  }
  double norm2 = 0.0;
  for (double value : state.orientation_xyzw()) {
    if (!std::isfinite(value)) return false;
    norm2 += value * value;
  }
  for (double value : state.angular_velocity_radps()) {
    if (!std::isfinite(value)) return false;
  }
  for (double value : state.covariance_7x7_row_major()) {
    if (!std::isfinite(value)) return false;
  }
  return std::isfinite(norm2) &&
         std::abs(std::sqrt(norm2) - 1.0) <= 1e-3;
}

}  // namespace

class AcousticOpticBuffer::Impl {
 public:
  Impl(AcousticOpticBufferConfig config, uw::domain::RigCalibrationSnapshot rig)
      : config_(config), rig_(std::move(rig)) {
    ValidateConfig(config_);
    ValidateRig(rig_);
    camera_ids_[0] = rig_.cameras(0).sensor_id().value();
    camera_ids_[1] = rig_.cameras(1).sensor_id().value();
    sonar_ids_ = EnabledSonars(rig_);
    state_sensor_id_ = rig_.vehicle_state_sensors(0).value();
  }

  template <typename Message>
  std::optional<TimeTick> CorrectedTime(const Message& message) {
    const auto& header = message.header();
    if (!ValidStamp(header) || header.sensor_id().value().empty()) {
      ++diagnostics_.invalid_time_count;
      return std::nullopt;
    }
    const auto offset = rig_.time_offset_seconds().find(header.sensor_id().value());
    if (offset == rig_.time_offset_seconds().end()) {
      ++diagnostics_.invalid_time_count;
      return std::nullopt;
    }
    const auto offset_nanoseconds = OffsetNanoseconds(offset->second);
    if (!offset_nanoseconds) {
      ++diagnostics_.invalid_time_count;
      return std::nullopt;
    }
    const TimeTick raw_nanoseconds =
        static_cast<TimeTick>(header.capture_time().seconds()) * kNanosPerSecond +
        header.capture_time().nanos();
    return 2 * (raw_nanoseconds + *offset_nanoseconds);
  }

  bool ValidateInputHeader(const uw::domain::ObservationHeader& header) {
    const auto validation = uw::domain::ValidateObservationHeader(header);
    if (!validation.ok()) {
      if (!header.has_capture_time() ||
          (header.has_capture_time() &&
           (header.capture_time().nanos() < 0 ||
            header.capture_time().nanos() >= kNanosPerSecond))) {
        ++diagnostics_.invalid_time_count;
      } else {
        ++diagnostics_.invalid_input_count;
      }
      return false;
    }
    if (header.calibration_version().value() != rig_.calibration_version().value()) {
      ++diagnostics_.invalid_input_count;
      return false;
    }
    return true;
  }

  template <typename Message>
  bool Insert(std::deque<Timed<Message>>* queue, Message message, TimeTick corrected,
              std::size_t capacity) {
    Timed<Message> item{corrected, message.header().sequence_id().value(), std::move(message)};
    const auto position = std::lower_bound(queue->begin(), queue->end(), item, TimedLess<Message>);
    if (position != queue->end() && position->corrected_time == item.corrected_time &&
        position->sequence == item.sequence &&
        position->message.header().sensor_id().value() ==
            item.message.header().sensor_id().value()) {
      ++diagnostics_.invalid_input_count;
      return false;
    }
    queue->insert(position, std::move(item));
    if (queue->size() > capacity) {
      queue->pop_front();
      ++diagnostics_.capacity_drop_count;
    }
    if (!watermark_ || corrected > *watermark_) watermark_ = corrected;
    Expire();
    return true;
  }

  std::optional<OnlineAcousticOpticBundle> AddImage(uw::domain::ImageFrame image) {
    if (!ValidateInputHeader(image.header())) return std::nullopt;
    const auto sensor = image.header().sensor_id().value();
    const int camera_index = sensor == camera_ids_[0] ? 0 : (sensor == camera_ids_[1] ? 1 : -1);
    if (camera_index < 0 || image.header().observation_id().value().empty()) {
      ++diagnostics_.invalid_input_count;
      return std::nullopt;
    }
    const auto time = CorrectedTime(image);
    if (!time || !Insert(&images_[camera_index], std::move(image), *time,
                         config_.max_images_per_camera)) {
      return std::nullopt;
    }
    return TryBundle();
  }

  std::optional<OnlineAcousticOpticBundle> AddSonar(uw::domain::SonarFrame sonar) {
    if (!ValidateInputHeader(sonar.header())) return std::nullopt;
    if (sonar_ids_.count(sonar.header().sensor_id().value()) == 0 ||
        sonar.header().observation_id().value().empty()) {
      ++diagnostics_.invalid_input_count;
      return std::nullopt;
    }
    const auto time = CorrectedTime(sonar);
    if (!time || !Insert(&sonars_, std::move(sonar), *time, config_.max_sonar_frames)) {
      return std::nullopt;
    }
    return TryBundle();
  }

  std::optional<OnlineAcousticOpticBundle> AddVehicleState(uw::domain::VehicleState state) {
    if (!ValidateInputHeader(state.header())) return std::nullopt;
    if (state.header().sensor_id().value() != state_sensor_id_ || !FiniteState(state)) {
      ++diagnostics_.invalid_input_count;
      return std::nullopt;
    }
    const auto time = CorrectedTime(state);
    if (!time || !Insert(&states_, std::move(state), *time, config_.max_vehicle_states)) {
      return std::nullopt;
    }
    return TryBundle();
  }

  void UpdateRig(uw::domain::RigCalibrationSnapshot rig) {
    ValidateRig(rig);
    if (rig.calibration_version().value() == rig_.calibration_version().value()) {
      if (!google::protobuf::util::MessageDifferencer::Equals(rig_, rig)) {
        throw std::invalid_argument("same calibration version cannot change timing or extrinsics");
      }
      return;
    }
    rig_ = std::move(rig);
    camera_ids_[0] = rig_.cameras(0).sensor_id().value();
    camera_ids_[1] = rig_.cameras(1).sensor_id().value();
    sonar_ids_ = EnabledSonars(rig_);
    state_sensor_id_ = rig_.vehicle_state_sensors(0).value();
    images_[0].clear();
    images_[1].clear();
    sonars_.clear();
    states_.clear();
    watermark_.reset();
    const uint64_t reset_count = diagnostics_.calibration_reset_count ==
                                         std::numeric_limits<uint64_t>::max()
                                     ? diagnostics_.calibration_reset_count
                                     : diagnostics_.calibration_reset_count + 1;
    diagnostics_ = {};
    diagnostics_.calibration_reset_count = reset_count;
    corrected_deltas_.clear();
  }

  AcousticOpticBufferDiagnostics Diagnostics() const {
    auto result = diagnostics_;
    result.buffered_image_count = images_[0].size() + images_[1].size();
    result.buffered_sonar_count = sonars_.size();
    result.buffered_vehicle_state_count = states_.size();
    result.corrected_delta_p50_s = Percentile(corrected_deltas_, 0.50);
    result.corrected_delta_p95_s = Percentile(corrected_deltas_, 0.95);
    result.corrected_delta_p99_s = Percentile(corrected_deltas_, 0.99);
    result.corrected_delta_max_s = corrected_deltas_.empty()
                                       ? 0.0
                                       : *std::max_element(corrected_deltas_.begin(), corrected_deltas_.end());
    return result;
  }

 private:
  struct StereoPair {
    std::size_t left = 0;
    std::size_t right = 0;
    TimeTick delta = 0;
    TimeTick time = 0;
  };

  void Expire() {
    if (!watermark_) return;
    const auto expire = [&](auto* queue) {
      while (!queue->empty() &&
             !WithinWindow(*watermark_ - queue->front().corrected_time,
                           config_.max_residence_s)) {
        queue->pop_front();
        ++diagnostics_.expiry_count;
      }
    };
    expire(&images_[0]);
    expire(&images_[1]);
    expire(&sonars_);
    expire(&states_);
  }

  std::vector<StereoPair> StereoPairs() const {
    struct Edge {
      std::size_t left;
      std::size_t right;
      TimeTick delta;
      TimeTick time;
      uint64_t ls;
      uint64_t rs;
    };
    std::vector<Edge> edges;
    for (std::size_t left = 0; left < images_[0].size(); ++left) {
      for (std::size_t right = 0; right < images_[1].size(); ++right) {
        const TimeTick delta =
            TickAbs(images_[0][left].corrected_time - images_[1][right].corrected_time);
        if (WithinWindow(delta, config_.max_stereo_delta_s)) {
          edges.push_back({left, right, delta,
                           (images_[0][left].corrected_time +
                            images_[1][right].corrected_time) /
                               2,
                           images_[0][left].sequence, images_[1][right].sequence});
        }
      }
    }
    std::sort(edges.begin(), edges.end(), [](const Edge& a, const Edge& b) {
      return std::tie(a.delta, a.time, a.ls, a.rs) < std::tie(b.delta, b.time, b.ls, b.rs);
    });
    std::vector<StereoPair> result;
    for (const auto& edge : edges) {
      result.push_back({edge.left, edge.right, edge.delta, edge.time});
    }
    return result;
  }

  std::optional<uw::domain::VehicleState> InterpolateState(TimeTick target) const {
    if (states_.empty()) return std::nullopt;
    auto upper = std::lower_bound(states_.begin(), states_.end(), target,
                                  [](const auto& state, TimeTick time) {
                                    return state.corrected_time < time;
                                  });
    if (upper != states_.end() && upper->corrected_time == target) {
      auto exact = upper->message;
      NormalizeOrientation(&exact);
      exact.mutable_header()->mutable_calibration_version()->set_value(rig_.calibration_version().value());
      return exact;
    }
    if (upper == states_.begin() || upper == states_.end()) return std::nullopt;
    const auto lower = std::prev(upper);
    if (!WithinWindow(upper->corrected_time - lower->corrected_time,
                      config_.max_state_bracket_s)) {
      return std::nullopt;
    }
    const double alpha = static_cast<double>(
        static_cast<long double>(target - lower->corrected_time) /
        static_cast<long double>(upper->corrected_time - lower->corrected_time));
    const Eigen::Quaterniond q0(lower->message.orientation_xyzw(3), lower->message.orientation_xyzw(0),
                                lower->message.orientation_xyzw(1), lower->message.orientation_xyzw(2));
    const Eigen::Quaterniond q1(upper->message.orientation_xyzw(3), upper->message.orientation_xyzw(0),
                                upper->message.orientation_xyzw(1), upper->message.orientation_xyzw(2));
    const Eigen::Quaterniond q = q0.normalized().slerp(alpha, q1.normalized()).normalized();
    const auto& copied = alpha <= 0.5 ? lower->message : upper->message;
    uw::domain::VehicleState result = copied;
    result.clear_orientation_xyzw();
    for (double value : {q.x(), q.y(), q.z(), q.w()}) result.add_orientation_xyzw(value);
    result.clear_angular_velocity_radps();
    for (int i = 0; i < 3; ++i) {
      result.add_angular_velocity_radps((1.0 - alpha) * lower->message.angular_velocity_radps(i) +
                                       alpha * upper->message.angular_velocity_radps(i));
    }
    result.set_depth_m((1.0 - alpha) * lower->message.depth_m() + alpha * upper->message.depth_m());
    auto* header = result.mutable_header();
    header->mutable_observation_id()->set_value("interpolated:" +
                                                lower->message.header().observation_id().value() + ":" +
                                                upper->message.header().observation_id().value());
    const auto offset_nanoseconds =
        OffsetNanoseconds(rig_.time_offset_seconds().at(state_sensor_id_));
    if (!offset_nanoseconds) return std::nullopt;
    const auto capture_stamp =
        StampFromHalfNanoseconds(target - 2 * static_cast<TimeTick>(*offset_nanoseconds));
    if (!capture_stamp) return std::nullopt;
    *header->mutable_capture_time() = *capture_stamp;
    header->mutable_calibration_version()->set_value(rig_.calibration_version().value());
    header->set_provenance("runtime/acoustic_optic_buffer:state_slerp");
    return result;
  }

  static void NormalizeOrientation(uw::domain::VehicleState* state) {
    double norm2 = 0.0;
    for (double value : state->orientation_xyzw()) norm2 += value * value;
    const double norm = std::sqrt(norm2);
    for (int i = 0; i < 4; ++i) state->set_orientation_xyzw(i, state->orientation_xyzw(i) / norm);
  }

  std::optional<OnlineAcousticOpticBundle> TryBundle() {
    if (sonars_.empty()) return std::nullopt;
    const auto pairs = StereoPairs();
    if (pairs.empty()) {
      ++diagnostics_.no_pair_count;
      if (!images_[0].empty() && !images_[1].empty()) ++diagnostics_.over_window_count;
      return std::nullopt;
    }
    struct Candidate {
      std::size_t sonar = 0;
      StereoPair pair;
      TimeTick delta = 0;
    };
    std::optional<Candidate> selected;
    const auto key = [&](const Candidate& candidate) {
      return std::make_tuple(candidate.delta, sonars_[candidate.sonar].corrected_time,
                             candidate.pair.time, sonars_[candidate.sonar].sequence,
                             sonars_[candidate.sonar].message.header().sensor_id().value(),
                             images_[0][candidate.pair.left].sequence,
                             images_[1][candidate.pair.right].sequence);
    };
    for (std::size_t sonar_index = 0; sonar_index < sonars_.size(); ++sonar_index) {
      for (const auto& pair : pairs) {
        Candidate candidate{sonar_index, pair,
                            TickAbs(pair.time - sonars_[sonar_index].corrected_time)};
        if (!selected || key(candidate) < key(*selected)) selected = candidate;
      }
    }
    ++diagnostics_.synchronization_candidate_count;
    if (!selected || !WithinWindow(selected->delta, config_.max_sonar_camera_delta_s)) {
      ++diagnostics_.over_window_count;
      return std::nullopt;
    }
    const auto& pair = selected->pair;
    const auto& left = images_[0][pair.left].message;
    const auto& right = images_[1][pair.right].message;
    if (left.header().observation_id().value() != right.header().observation_id().value()) {
      images_[0].erase(images_[0].begin() + static_cast<std::ptrdiff_t>(pair.left));
      images_[1].erase(images_[1].begin() + static_cast<std::ptrdiff_t>(pair.right));
      ++diagnostics_.integrity_rejection_count;
      return std::nullopt;
    }
    auto state = InterpolateState(pair.time);
    if (!state) {
      ++diagnostics_.no_pair_count;
      return std::nullopt;
    }
    OnlineAcousticOpticBundle bundle;
    bundle.images.primary = left;
    bundle.images.secondary = right;
    bundle.sonar = sonars_[selected->sonar].message;
    bundle.interpolated_vehicle_state = std::move(*state);
    bundle.corrected_time_delta_s = TickSeconds(selected->delta);
    images_[0].erase(images_[0].begin() + static_cast<std::ptrdiff_t>(pair.left));
    images_[1].erase(images_[1].begin() + static_cast<std::ptrdiff_t>(pair.right));
    sonars_.erase(sonars_.begin() + static_cast<std::ptrdiff_t>(selected->sonar));
    corrected_deltas_.push_back(TickSeconds(selected->delta));
    if (corrected_deltas_.size() > kDeltaWindowSize) corrected_deltas_.erase(corrected_deltas_.begin());
    ++diagnostics_.accepted_count;
    return bundle;
  }

  AcousticOpticBufferConfig config_;
  uw::domain::RigCalibrationSnapshot rig_;
  std::string camera_ids_[2];
  std::set<std::string> sonar_ids_;
  std::string state_sensor_id_;
  std::deque<Timed<uw::domain::ImageFrame>> images_[2];
  std::deque<Timed<uw::domain::SonarFrame>> sonars_;
  std::deque<Timed<uw::domain::VehicleState>> states_;
  std::optional<TimeTick> watermark_;
  AcousticOpticBufferDiagnostics diagnostics_;
  std::vector<double> corrected_deltas_;
};

AcousticOpticBuffer::AcousticOpticBuffer(AcousticOpticBufferConfig config,
                                         uw::domain::RigCalibrationSnapshot rig)
    : impl_(std::make_unique<Impl>(config, std::move(rig))) {}
AcousticOpticBuffer::~AcousticOpticBuffer() = default;
AcousticOpticBuffer::AcousticOpticBuffer(AcousticOpticBuffer&&) noexcept = default;
AcousticOpticBuffer& AcousticOpticBuffer::operator=(AcousticOpticBuffer&&) noexcept = default;
std::optional<OnlineAcousticOpticBundle> AcousticOpticBuffer::AddImage(uw::domain::ImageFrame image) {
  return impl_->AddImage(std::move(image));
}
std::optional<OnlineAcousticOpticBundle> AcousticOpticBuffer::AddSonar(uw::domain::SonarFrame sonar) {
  return impl_->AddSonar(std::move(sonar));
}
std::optional<OnlineAcousticOpticBundle> AcousticOpticBuffer::AddVehicleState(uw::domain::VehicleState state) {
  return impl_->AddVehicleState(std::move(state));
}
void AcousticOpticBuffer::UpdateRig(uw::domain::RigCalibrationSnapshot rig) {
  impl_->UpdateRig(std::move(rig));
}
AcousticOpticBufferDiagnostics AcousticOpticBuffer::Diagnostics() const { return impl_->Diagnostics(); }

}  // namespace uw::runtime
