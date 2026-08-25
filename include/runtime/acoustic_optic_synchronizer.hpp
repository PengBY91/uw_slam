// Pure, stateless capture-time pairing: given
// already-captured frames and the rig's time_offset_seconds
// (t_reference = t_sensor_capture + time_offset_seconds[sensor_id]),
// decides whether they form one valid synchronized bundle. Uses
// capture_time, not receive_time. Rejects the bundle rather than
// extrapolating when any pairwise corrected-time delta exceeds
// max_time_delta_s — no motion model exists yet to interpolate across.
// A sensor_id missing from time_offset_seconds is invalid. A measured zero
// must be represented explicitly; absence is never calibration evidence.
//
// Returns an explicit SynchronizationDecision rather than a bare
// std::optional<bundle> so a caller can distinguish "no sonar this frame"
// (still fine to run optical-only) from "sonar present but desynchronized"
// (the real max_pairwise_time_delta_s is preserved even when the bundle
// itself is withheld) from "malformed timestamp" — collapsing all three
// into one nullopt previously forced callers to guess, and one caller
// (apps/replay_demo, before this) guessed wrong: it substituted 0.0 for a
// withheld delta, which silently told the fusion frontend "perfectly
// synchronized" for frames that were anything but.
#pragma once

#include <optional>

#include "domain/domain.hpp"
#include "measurement_api/frontend.hpp"

namespace uw::runtime {

struct SynchronizerParams {
  double max_time_delta_s = 0.05;
};

struct SynchronizedAcousticOpticBundle {
  uw::measurement_api::CameraFrameBundle images;
  uw::domain::SonarFrame sonar;
  double max_pairwise_time_delta_s = 0.0;
};

enum class SynchronizationStatus {
  kSynchronized,       // bundle populated, delta <= max_time_delta_s
  kNoSonar,             // no sonar frame this cycle -- not a failure, just optical-only
  kTimeDeltaExceeded,   // sonar present but outside the sync window; delta still reported
  kInvalidTimestamp,    // a header's capture_time/sensor_id/offset failed validation
};

struct SynchronizationDecision {
  SynchronizationStatus status = SynchronizationStatus::kInvalidTimestamp;
  // Real max pairwise delta whenever it was computable (kSynchronized and
  // kTimeDeltaExceeded); 0.0 for kNoSonar/kInvalidTimestamp, where no delta
  // could be computed at all -- never a stand-in for "unknown".
  double max_pairwise_time_delta_s = 0.0;
  std::optional<SynchronizedAcousticOpticBundle> bundle;
};

SynchronizationDecision SynchronizeAcousticOptic(
    const uw::domain::ImageFrame& primary, const std::optional<uw::domain::ImageFrame>& secondary,
    const std::optional<uw::domain::SonarFrame>& sonar, const uw::domain::RigCalibrationSnapshot& rig,
    const SynchronizerParams& params);

}  // namespace uw::runtime
