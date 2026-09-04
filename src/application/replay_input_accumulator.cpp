#include "application/replay_input_accumulator.hpp"

#include <variant>

namespace uw::application {

bool ReplayInputAccumulator::ValidateRawIdentity(const std::string& sensor_id,
                                                 const std::string& observation_id,
                                                 const std::string& kind_label,
                                                 bool allow_duplicate_identity) {
  if (observation_id.empty()) {
    ++diagnostics_.empty_observation_id_count;
    diagnostics_.messages.push_back("empty observation_id on a " + kind_label +
                                    " (sensor_id=" + sensor_id + ")");
    return false;
  }
  const std::string key = sensor_id + "\x1F" + observation_id;
  const bool is_new = seen_raw_identities_.insert(key).second;
  if (!is_new && !allow_duplicate_identity) {
    ++diagnostics_.duplicate_observation_count;
    diagnostics_.messages.push_back("duplicate (sensor_id, observation_id) = (" + sensor_id + ", " +
                                    observation_id + ") on a " + kind_label);
    return false;
  }
  known_observation_ids_.insert(observation_id);
  return true;
}

bool ReplayInputAccumulator::OnImageFrame(const uw::runtime::CanonicalEvent& event) {
  const auto& frame = std::get<uw::domain::ImageFrame>(event.payload);
  if (ValidateRawIdentity(frame.header().sensor_id().value(), frame.header().observation_id().value(),
                          "ImageFrame", /*allow_duplicate_identity=*/false)) {
    data_.images.push_back(frame);
  }
  return true;
}

bool ReplayInputAccumulator::OnSonarFrame(const uw::runtime::CanonicalEvent& event) {
  const auto& frame = std::get<uw::domain::SonarFrame>(event.payload);
  if (ValidateRawIdentity(frame.header().sensor_id().value(), frame.header().observation_id().value(),
                          "SonarFrame", /*allow_duplicate_identity=*/true)) {
    data_.sonar_frames.push_back(frame);
  }
  return true;
}

bool ReplayInputAccumulator::OnImuSample(const uw::runtime::CanonicalEvent& event) {
  const auto& sample = std::get<uw::domain::ImuSample>(event.payload);
  if (ValidateRawIdentity(sample.header().sensor_id().value(), sample.header().observation_id().value(),
                          "ImuSample", /*allow_duplicate_identity=*/false)) {
    data_.imu_samples.push_back(sample);
  }
  return true;
}

bool ReplayInputAccumulator::OnDvlSample(const uw::runtime::CanonicalEvent& event) {
  const auto& sample = std::get<uw::domain::DvlSample>(event.payload);
  if (ValidateRawIdentity(sample.header().sensor_id().value(), sample.header().observation_id().value(),
                          "DvlSample", /*allow_duplicate_identity=*/false)) {
    data_.dvl_samples.push_back(sample);
  }
  return true;
}

bool ReplayInputAccumulator::OnVehicleState(const uw::runtime::CanonicalEvent& event) {
  const auto& state = std::get<uw::domain::VehicleState>(event.payload);
  if (ValidateRawIdentity(state.header().sensor_id().value(),
                          state.header().observation_id().value(), "VehicleState",
                          /*allow_duplicate_identity=*/false)) {
    data_.vehicle_states.push_back(state);
  }
  return true;
}

bool ReplayInputAccumulator::OnKeyframeBoundary(const uw::runtime::CanonicalEvent& event) {
  const auto& boundary = std::get<uw::domain::KeyframeBoundary>(event.payload);
  const std::string& keyframe_id = boundary.keyframe_id().value();
  if (keyframe_id.empty()) {
    ++diagnostics_.empty_keyframe_id_count;
    diagnostics_.messages.push_back("empty keyframe_id on a KeyframeBoundary");
    return true;
  }
  if (seen_keyframe_ids_.find(keyframe_id) != seen_keyframe_ids_.end()) {
    ++diagnostics_.duplicate_keyframe_id_count;
    diagnostics_.messages.push_back("duplicate keyframe_id '" + keyframe_id +
                                    "' on a KeyframeBoundary");
    return true;
  }

  const auto& capture_time = boundary.header().capture_time();
  if (!data_.keyframe_boundaries.empty()) {
    const auto& previous = data_.keyframe_boundaries.back().header().capture_time();
    if (capture_time.seconds() < previous.seconds() ||
        (capture_time.seconds() == previous.seconds() &&
         capture_time.nanos() <= previous.nanos())) {
      ++diagnostics_.non_increasing_keyframe_capture_time_count;
      diagnostics_.messages.push_back(
          "KeyframeBoundary capture_time is not strictly increasing for keyframe_id '" +
          keyframe_id + "'");
      return true;
    }
  }

  if (!ValidateRawIdentity(boundary.header().sensor_id().value(),
                           boundary.header().observation_id().value(),
                           "KeyframeBoundary", /*allow_duplicate_identity=*/false)) {
    return true;
  }
  seen_keyframe_ids_.insert(keyframe_id);
  data_.keyframe_boundaries.push_back(boundary);
  return true;
}

bool ReplayInputAccumulator::OnMeasurementEvidence(const uw::runtime::CanonicalEvent& event) {
  data_.evidence.push_back(std::get<uw::domain::MeasurementEvidence>(event.payload));
  evidence_log_time_ns_.push_back(event.log_time_ns);
  return true;
}

bool ReplayInputAccumulator::OnReferenceState(const uw::runtime::CanonicalEvent& event) {
  data_.reference_states.push_back(std::get<uw::domain::StateSnapshot>(event.payload));
  return true;
}

bool ReplayInputAccumulator::OnHealthReport(const uw::runtime::CanonicalEvent&) { return true; }

bool ReplayInputAccumulator::OnMapEvidence(const uw::runtime::CanonicalEvent&) { return true; }

bool ReplayInputAccumulator::Flush() {
  for (const auto& evidence : data_.evidence) {
    for (const auto& source : evidence.source_observations()) {
      if (known_observation_ids_.find(source.value()) == known_observation_ids_.end()) {
        ++diagnostics_.dangling_evidence_reference_count;
        diagnostics_.messages.push_back("evidence " + evidence.evidence_id().value() +
                                        " references unknown source observation '" + source.value() +
                                        "'");
      }
    }
  }
  return true;
}

}  // namespace uw::application
