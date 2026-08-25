#include "runtime/acoustic_optic_synchronizer.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace uw::runtime {

namespace {

constexpr int32_t kNanosPerSecond = 1'000'000'000;

// Validates sensor_id/capture_time/looked-up offset and returns the
// reference-clock-corrected time, or std::nullopt if anything is malformed
// -- see this file's header comment for why a malformed header must
// propagate as kInvalidTimestamp rather than being silently treated as
// "no sonar" or coerced into some other status.
std::optional<double> CorrectedTime(const uw::domain::ObservationHeader& header,
                                    const uw::domain::RigCalibrationSnapshot& rig) {
  if (header.sensor_id().value().empty()) return std::nullopt;
  if (!header.has_capture_time()) return std::nullopt;
  const auto& stamp = header.capture_time();
  if (stamp.nanos() < 0 || stamp.nanos() >= kNanosPerSecond) return std::nullopt;

  const auto offset_it = rig.time_offset_seconds().find(header.sensor_id().value());
  if (offset_it == rig.time_offset_seconds().end()) return std::nullopt;
  const double offset = offset_it->second;
  if (!std::isfinite(offset)) return std::nullopt;

  const double corrected = uw::domain::ToSeconds(stamp) + offset;
  if (!std::isfinite(corrected)) return std::nullopt;
  return corrected;
}

}  // namespace

SynchronizationDecision SynchronizeAcousticOptic(
    const uw::domain::ImageFrame& primary, const std::optional<uw::domain::ImageFrame>& secondary,
    const std::optional<uw::domain::SonarFrame>& sonar, const uw::domain::RigCalibrationSnapshot& rig,
    const SynchronizerParams& params) {
  SynchronizationDecision decision;  // defaults to kInvalidTimestamp, no bundle

  const auto primary_time = CorrectedTime(primary.header(), rig);
  if (!primary_time.has_value()) return decision;

  std::optional<double> secondary_time;
  if (secondary.has_value()) {
    secondary_time = CorrectedTime(secondary->header(), rig);
    if (!secondary_time.has_value()) return decision;
  }

  if (!sonar.has_value()) {
    decision.status = SynchronizationStatus::kNoSonar;
    return decision;  // no delta computable without a sonar timestamp
  }

  const auto sonar_time = CorrectedTime(sonar->header(), rig);
  if (!sonar_time.has_value()) return decision;  // malformed sonar header: kInvalidTimestamp, not kNoSonar

  std::vector<double> times = {*primary_time, *sonar_time};
  if (secondary_time.has_value()) times.push_back(*secondary_time);
  const double delta = *std::max_element(times.begin(), times.end()) -
                       *std::min_element(times.begin(), times.end());

  if (delta > params.max_time_delta_s) {
    decision.status = SynchronizationStatus::kTimeDeltaExceeded;
    decision.max_pairwise_time_delta_s = delta;  // real delta, not silently dropped
    return decision;
  }

  decision.status = SynchronizationStatus::kSynchronized;
  decision.max_pairwise_time_delta_s = delta;
  decision.bundle = SynchronizedAcousticOpticBundle{};
  decision.bundle->images.primary = primary;
  decision.bundle->images.secondary = secondary;
  decision.bundle->sonar = *sonar;
  decision.bundle->max_pairwise_time_delta_s = delta;
  return decision;
}

}  // namespace uw::runtime
