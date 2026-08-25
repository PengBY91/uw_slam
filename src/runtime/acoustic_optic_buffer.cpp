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
constexpr double kComparisonEpsilon = 1e-12;
constexpr std::size_t kDeltaWindowSize = 256;

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
  if (rig.calibration_version().value().empty() || rig.cameras_size() != 2 ||
      rig.vehicle_state_sensors_size() != 1 || EnabledSonars(rig).empty()) {
    throw std::invalid_argument("online acoustic-optic rig requires a version, two cameras, enabled sonar, and exactly one vehicle-state source");
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
    if (offset == rig.time_offset_seconds().end() || !std::isfinite(offset->second) ||
        provenance == rig.time_offset_provenance().end() || provenance->second.empty()) {
      throw std::invalid_argument("online sensor lacks measured time offset/provenance: " + sensor);
    }
  }
}

template <typename Message>
struct Timed {
  double corrected_time = 0.0;
  uint64_t sequence = 0;
  Message message;
};

template <typename Message>
bool TimedLess(const Timed<Message>& lhs, const Timed<Message>& rhs) {
  return std::tie(lhs.corrected_time, lhs.sequence) <
         std::tie(rhs.corrected_time, rhs.sequence);
}

double Percentile(std::vector<double> values, double percentile) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const auto rank = static_cast<std::size_t>(std::ceil(percentile * values.size()));
  return values[rank - 1];
}

bool FiniteState(const uw::domain::VehicleState& state) {
  if (state.orientation_xyzw_size() != 4 || state.angular_velocity_radps_size() != 3 ||
      !state.attitude_valid() || !state.depth_valid() || !state.device_health_valid() ||
      !std::isfinite(state.depth_m()) || state.depth_m() < 0.0 ||
      !std::isfinite(state.supply_voltage_v()) || !std::isfinite(state.supply_current_a()) ||
      !std::isfinite(state.link_quality())) {
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
  return std::isfinite(norm2) && norm2 > std::numeric_limits<double>::epsilon();
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
  std::optional<double> CorrectedTime(const Message& message) {
    const auto& header = message.header();
    if (!ValidStamp(header) || header.sensor_id().value().empty()) {
      ++diagnostics_.invalid_time_count;
      return std::nullopt;
    }
    const auto offset = rig_.time_offset_seconds().find(header.sensor_id().value());
    if (offset == rig_.time_offset_seconds().end() || !std::isfinite(offset->second)) {
      ++diagnostics_.invalid_time_count;
      return std::nullopt;
    }
    const double corrected = uw::domain::ToSeconds(header.capture_time()) + offset->second;
    if (!std::isfinite(corrected)) {
      ++diagnostics_.invalid_time_count;
      return std::nullopt;
    }
    return corrected;
  }

  template <typename Message>
  bool Insert(std::deque<Timed<Message>>* queue, Message message, double corrected,
              std::size_t capacity) {
    Timed<Message> item{corrected, message.header().sequence_id().value(), std::move(message)};
    const auto position = std::lower_bound(queue->begin(), queue->end(), item, TimedLess<Message>);
    if (position != queue->end() && position->corrected_time == item.corrected_time &&
        position->sequence == item.sequence) {
      ++diagnostics_.invalid_input_count;
      return false;
    }
    queue->insert(position, std::move(item));
    if (queue->size() > capacity) {
      queue->pop_front();
      ++diagnostics_.capacity_drop_count;
    }
    watermark_ = std::max(watermark_, corrected);
    Expire();
    return true;
  }

  std::optional<OnlineAcousticOpticBundle> AddImage(uw::domain::ImageFrame image) {
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
    watermark_ = -std::numeric_limits<double>::infinity();
    ++diagnostics_.calibration_reset_count;
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
    double delta = 0.0;
    double time = 0.0;
  };

  void Expire() {
    if (!std::isfinite(watermark_)) return;
    const double oldest = watermark_ - config_.max_residence_s;
    const auto expire = [&](auto* queue) {
      while (!queue->empty() && queue->front().corrected_time + kComparisonEpsilon < oldest) {
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
    struct Edge { std::size_t left; std::size_t right; double delta; double time; uint64_t ls; uint64_t rs; };
    std::vector<Edge> edges;
    for (std::size_t left = 0; left < images_[0].size(); ++left) {
      for (std::size_t right = 0; right < images_[1].size(); ++right) {
        const double delta = std::abs(images_[0][left].corrected_time - images_[1][right].corrected_time);
        if (delta <= config_.max_stereo_delta_s + kComparisonEpsilon) {
          edges.push_back({left, right, delta,
                           0.5 * (images_[0][left].corrected_time + images_[1][right].corrected_time),
                           images_[0][left].sequence, images_[1][right].sequence});
        }
      }
    }
    std::sort(edges.begin(), edges.end(), [](const Edge& a, const Edge& b) {
      return std::tie(a.delta, a.time, a.ls, a.rs) < std::tie(b.delta, b.time, b.ls, b.rs);
    });
    std::set<std::size_t> used_left;
    std::set<std::size_t> used_right;
    std::vector<StereoPair> result;
    for (const auto& edge : edges) {
      if (used_left.count(edge.left) || used_right.count(edge.right)) continue;
      used_left.insert(edge.left);
      used_right.insert(edge.right);
      result.push_back({edge.left, edge.right, edge.delta, edge.time});
    }
    return result;
  }

  std::optional<uw::domain::VehicleState> InterpolateState(double target) const {
    if (states_.empty()) return std::nullopt;
    auto upper = std::lower_bound(states_.begin(), states_.end(), target,
                                  [](const auto& state, double time) {
                                    return state.corrected_time < time;
                                  });
    if (upper != states_.end() && std::abs(upper->corrected_time - target) <= kComparisonEpsilon) {
      auto exact = upper->message;
      NormalizeOrientation(&exact);
      exact.mutable_header()->mutable_calibration_version()->set_value(rig_.calibration_version().value());
      return exact;
    }
    if (upper == states_.begin() || upper == states_.end()) return std::nullopt;
    const auto lower = std::prev(upper);
    if (target - lower->corrected_time > config_.max_state_bracket_s + kComparisonEpsilon ||
        upper->corrected_time - target > config_.max_state_bracket_s + kComparisonEpsilon) {
      return std::nullopt;
    }
    const double alpha = (target - lower->corrected_time) /
                         (upper->corrected_time - lower->corrected_time);
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
    const double offset = rig_.time_offset_seconds().at(state_sensor_id_);
    *header->mutable_capture_time() = uw::domain::FromSeconds(target - offset);
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
    const auto& sonar = sonars_.front();
    const auto selected = std::min_element(pairs.begin(), pairs.end(), [&](const auto& a, const auto& b) {
      return std::make_tuple(std::abs(a.time - sonar.corrected_time), a.time,
                             images_[0][a.left].sequence, images_[1][a.right].sequence) <
             std::make_tuple(std::abs(b.time - sonar.corrected_time), b.time,
                             images_[0][b.left].sequence, images_[1][b.right].sequence);
    });
    ++diagnostics_.synchronization_candidate_count;
    const double delta = std::abs(selected->time - sonar.corrected_time);
    if (delta > config_.max_sonar_camera_delta_s + kComparisonEpsilon) {
      ++diagnostics_.over_window_count;
      return std::nullopt;
    }
    const auto& left = images_[0][selected->left].message;
    const auto& right = images_[1][selected->right].message;
    if (left.header().observation_id().value() != right.header().observation_id().value()) {
      images_[0].erase(images_[0].begin() + static_cast<std::ptrdiff_t>(selected->left));
      images_[1].erase(images_[1].begin() + static_cast<std::ptrdiff_t>(selected->right));
      sonars_.pop_front();
      ++diagnostics_.integrity_rejection_count;
      return std::nullopt;
    }
    auto state = InterpolateState(selected->time);
    if (!state) {
      ++diagnostics_.no_pair_count;
      return std::nullopt;
    }
    OnlineAcousticOpticBundle bundle;
    bundle.images.primary = left;
    bundle.images.secondary = right;
    bundle.sonar = sonar.message;
    bundle.interpolated_vehicle_state = std::move(*state);
    bundle.corrected_time_delta_s = delta;
    images_[0].erase(images_[0].begin() + static_cast<std::ptrdiff_t>(selected->left));
    images_[1].erase(images_[1].begin() + static_cast<std::ptrdiff_t>(selected->right));
    sonars_.pop_front();
    corrected_deltas_.push_back(delta);
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
  double watermark_ = -std::numeric_limits<double>::infinity();
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
