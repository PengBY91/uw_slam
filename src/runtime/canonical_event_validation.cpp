#include "runtime/canonical_event_validation.hpp"

#include <cmath>
#include <cstddef>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>

#include <google/protobuf/descriptor.h>

#include "domain/domain.hpp"
#include "runtime/canonical_topics.hpp"

namespace uw::runtime {
namespace {

CanonicalEventValidationResult Invalid(CanonicalEventValidationCode code,
                                       std::string message) {
  return {code, std::move(message)};
}

CanonicalEventValidationResult ValidateRawHeader(
    const uw::domain::ObservationHeader& header) {
  const auto result = uw::domain::ValidateObservationHeader(header);
  if (!result.ok()) {
    return Invalid(CanonicalEventValidationCode::kHeaderInvalid, result.message);
  }
  return {};
}

CanonicalEventValidationResult ValidatePayload(const uw::domain::ImageFrame& frame) {
  auto result = ValidateRawHeader(frame.header());
  if (!result.ok()) return result;

  const auto image_result = uw::domain::ValidateImageFrame(frame);
  if (!image_result.ok()) {
    return Invalid(CanonicalEventValidationCode::kImageInvalid, image_result.message);
  }
  return {};
}

bool IsFinitePositive(double value) {
  return std::isfinite(value) && value > 0.0;
}

bool HasStrictlyAscendingFiniteRanges(const uw::domain::SonarFrame& frame) {
  for (int index = 0; index < frame.range_bins_size(); ++index) {
    if (!std::isfinite(frame.range_bins(index))) return false;
    if (index > 0 && frame.range_bins(index - 1) >= frame.range_bins(index)) return false;
  }
  return true;
}

bool HasFiniteAzimuths(const uw::domain::SonarFrame& frame) {
  for (float azimuth : frame.azimuth_angles()) {
    if (!std::isfinite(azimuth)) return false;
  }
  return true;
}

CanonicalEventValidationResult ValidatePayload(const uw::domain::SonarFrame& frame) {
  auto result = ValidateRawHeader(frame.header());
  if (!result.ok()) return result;

  const std::size_t expected_size =
      static_cast<std::size_t>(frame.num_ranges()) * frame.num_beams();
  if (frame.intensity_tensor().size() != expected_size) {
    return Invalid(CanonicalEventValidationCode::kSonarPayloadSizeMismatch,
                   "sonar intensity tensor size does not match num_ranges * num_beams");
  }

  if (frame.num_ranges() == 0 || frame.num_beams() == 0 ||
      frame.encoding() != uw::domain::SonarFrame::ENCODING_UINT8_GRAY) {
    return Invalid(CanonicalEventValidationCode::kSonarGeometryInvalid,
                   "sonar shape and encoding must be explicit");
  }
  if (frame.azimuth_angles_size() != static_cast<int>(frame.num_beams()) ||
      !HasFiniteAzimuths(frame) || !uw::domain::IsAzimuthAscending(frame)) {
    return Invalid(CanonicalEventValidationCode::kSonarGeometryInvalid,
                   "sonar azimuths must match num_beams and be finite and strictly ascending");
  }

  const std::size_t range_bin_count = static_cast<std::size_t>(frame.range_bins_size());
  const std::size_t num_ranges = frame.num_ranges();
  if ((range_bin_count != num_ranges && range_bin_count != num_ranges + 1U) ||
      !HasStrictlyAscendingFiniteRanges(frame)) {
    return Invalid(CanonicalEventValidationCode::kSonarGeometryInvalid,
                   "sonar range bins must match num_ranges and be finite and strictly ascending");
  }

  if (!std::isfinite(frame.min_range()) || frame.min_range() < 0.0f ||
      !std::isfinite(frame.max_range()) || frame.max_range() <= frame.min_range() ||
      !IsFinitePositive(frame.range_resolution()) ||
      !IsFinitePositive(frame.horizontal_fov()) ||
      !IsFinitePositive(frame.elevation_aperture())) {
    return Invalid(CanonicalEventValidationCode::kSonarGeometryInvalid,
                   "sonar range and field-of-view geometry must be finite and ordered");
  }

  if (!IsFinitePositive(frame.operating_frequency_hz()) ||
      !IsFinitePositive(frame.gain_metadata().gain()) ||
      !IsFinitePositive(frame.sound_speed_assumption().speed_of_sound_mps())) {
    return Invalid(CanonicalEventValidationCode::kSonarGeometryInvalid,
                   "sonar frequency, gain, and sound speed must be finite and positive");
  }
  return {};
}

CanonicalEventValidationResult ValidatePayload(const uw::domain::ImuSample& sample) {
  return ValidateRawHeader(sample.header());
}

CanonicalEventValidationResult ValidatePayload(const uw::domain::DvlSample& sample) {
  return ValidateRawHeader(sample.header());
}

CanonicalEventValidationResult ValidatePayload(const uw::domain::KeyframeBoundary& boundary) {
  auto result = ValidateRawHeader(boundary.header());
  if (!result.ok()) return result;
  if (boundary.keyframe_id().value().empty()) {
    return Invalid(CanonicalEventValidationCode::kKeyframeBoundaryInvalid,
                   "keyframe boundary keyframe_id must be nonempty");
  }
  if (boundary.source().empty()) {
    return Invalid(CanonicalEventValidationCode::kKeyframeBoundaryInvalid,
                   "keyframe boundary source must be nonempty");
  }
  return {};
}

bool HasFiniteVehicleVectors(const uw::domain::VehicleState& state) {
  for (double value : state.orientation_xyzw()) {
    if (!std::isfinite(value)) return false;
  }
  for (double value : state.angular_velocity_radps()) {
    if (!std::isfinite(value)) return false;
  }
  return true;
}

CanonicalEventValidationResult ValidatePayload(const uw::domain::VehicleState& state) {
  auto result = ValidateRawHeader(state.header());
  if (!result.ok()) return result;

  if (state.orientation_xyzw_size() != 4 || state.angular_velocity_radps_size() != 3) {
    return Invalid(CanonicalEventValidationCode::kVehicleVectorSizeInvalid,
                   "vehicle orientation and angular velocity must have sizes 4 and 3");
  }
  if (!state.attitude_valid() || !state.depth_valid() || !state.device_health_valid()) {
    return Invalid(CanonicalEventValidationCode::kVehicleValueInvalid,
                   "vehicle attitude, depth, and device-health validity must be explicit");
  }
  if (!HasFiniteVehicleVectors(state)) {
    return Invalid(CanonicalEventValidationCode::kVehicleValueInvalid,
                   "vehicle orientation and angular velocity values must be finite");
  }

  double quaternion_squared_norm = 0.0;
  for (double value : state.orientation_xyzw()) quaternion_squared_norm += value * value;
  if (std::abs(std::sqrt(quaternion_squared_norm) - 1.0) > 1e-3) {
    return Invalid(CanonicalEventValidationCode::kVehicleQuaternionInvalid,
                   "vehicle orientation quaternion must have unit norm");
  }

  if (!std::isfinite(state.depth_m()) || state.depth_m() < 0.0) {
    return Invalid(CanonicalEventValidationCode::kVehicleValueInvalid,
                   "positive-down vehicle depth must be finite and non-negative");
  }
  if (!IsFinitePositive(state.supply_voltage_v()) ||
      !IsFinitePositive(state.supply_current_a()) ||
      !std::isfinite(state.link_quality()) || state.link_quality() < 0.0 ||
      state.link_quality() > 1.0) {
    return Invalid(CanonicalEventValidationCode::kVehicleValueInvalid,
                   "vehicle device-health values are invalid");
  }
  return {};
}

template <typename T>
CanonicalEventValidationResult ValidatePayload(const T&) {
  return {};
}

const google::protobuf::Descriptor* PayloadDescriptor(const CanonicalPayload& payload) {
  return std::visit(
      [](const auto& message) {
        using Message = std::decay_t<decltype(message)>;
        return Message::descriptor();
      },
      payload);
}

}  // namespace

CanonicalEventValidationResult ValidateCanonicalEvent(const CanonicalEvent& event) {
  const auto* topic_info = LookupCanonicalTopic(event.topic);
  if (topic_info == nullptr) {
    return Invalid(CanonicalEventValidationCode::kUnknownTopic,
                   "event topic is not in the canonical registry");
  }
  if (topic_info->descriptor != PayloadDescriptor(event.payload)) {
    return Invalid(CanonicalEventValidationCode::kTopicPayloadMismatch,
                   "event payload type does not match the canonical topic");
  }
  return std::visit([](const auto& payload) { return ValidatePayload(payload); }, event.payload);
}

}  // namespace uw::runtime
