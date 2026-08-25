#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>

#include "domain/domain.hpp"
#include "measurement_api/frontend.hpp"
#include "runtime/config.hpp"

namespace uw::runtime {

struct OnlineAcousticOpticBundle {
  uw::measurement_api::CameraFrameBundle images;
  uw::domain::SonarFrame sonar;
  uw::domain::VehicleState interpolated_vehicle_state;
  double corrected_time_delta_s = 0.0;
};

struct AcousticOpticBufferDiagnostics {
  // All counters and corrected-delta statistics describe the active
  // calibration epoch, except calibration_reset_count, which is lifetime.
  uint64_t synchronization_candidate_count = 0;
  uint64_t accepted_count = 0;
  uint64_t no_pair_count = 0;
  uint64_t over_window_count = 0;
  uint64_t invalid_time_count = 0;
  uint64_t invalid_input_count = 0;
  uint64_t integrity_rejection_count = 0;
  uint64_t capacity_drop_count = 0;
  uint64_t expiry_count = 0;
  uint64_t calibration_reset_count = 0;
  std::size_t buffered_image_count = 0;
  std::size_t buffered_sonar_count = 0;
  std::size_t buffered_vehicle_state_count = 0;
  double corrected_delta_p50_s = 0.0;
  double corrected_delta_p95_s = 0.0;
  double corrected_delta_p99_s = 0.0;
  double corrected_delta_max_s = 0.0;
};

// Bounded online pairing in the rig reference clock. The sign convention is
// t_reference = t_sensor_capture + time_offset_seconds[sensor_id]. State is
// interpolated at the stereo midpoint: orientation uses normalized shortest-
// path quaternion slerp, angular velocity/depth are linear, and covariance
// plus device-health fields are copied from the nearest bracketing state.
// Each Add call decides immediately against observations buffered at that
// moment. Selection is deterministic for that current set; without an
// explicit watermark/end-of-batch API it cannot account for unknowable
// observations that may arrive in the future.
class AcousticOpticBuffer {
 public:
  AcousticOpticBuffer(AcousticOpticBufferConfig config,
                      uw::domain::RigCalibrationSnapshot rig);
  ~AcousticOpticBuffer();
  AcousticOpticBuffer(AcousticOpticBuffer&&) noexcept;
  AcousticOpticBuffer& operator=(AcousticOpticBuffer&&) noexcept;
  AcousticOpticBuffer(const AcousticOpticBuffer&) = delete;
  AcousticOpticBuffer& operator=(const AcousticOpticBuffer&) = delete;

  std::optional<OnlineAcousticOpticBundle> AddImage(uw::domain::ImageFrame image);
  std::optional<OnlineAcousticOpticBundle> AddSonar(uw::domain::SonarFrame sonar);
  std::optional<OnlineAcousticOpticBundle> AddVehicleState(uw::domain::VehicleState state);
  void UpdateRig(uw::domain::RigCalibrationSnapshot rig);
  AcousticOpticBufferDiagnostics Diagnostics() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace uw::runtime
