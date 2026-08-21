// Pure, stateless capture-time pairing (design spec section 5.1): given
// already-captured frames and the rig's time_offset_seconds
// (t_reference = t_sensor_capture + time_offset_seconds[sensor_id]),
// decides whether they form one valid synchronized bundle. Uses
// capture_time, not receive_time. Rejects (nullopt) rather than
// extrapolating when any pairwise corrected-time delta exceeds
// max_time_delta_s — no motion model exists yet to interpolate across.
// A sensor_id missing from time_offset_seconds defaults to a zero offset
// (documented v1 simplification; full audit trail via RunManifest/health
// is a later integration concern, not this pure function's job).
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

std::optional<SynchronizedAcousticOpticBundle> SynchronizeAcousticOptic(
    const uw::domain::ImageFrame& primary, const std::optional<uw::domain::ImageFrame>& secondary,
    const uw::domain::SonarFrame& sonar, const uw::domain::RigCalibrationSnapshot& rig,
    const SynchronizerParams& params);

}  // namespace uw::runtime
