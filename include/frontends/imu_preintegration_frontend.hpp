// IMU preintegration frontend (PREP-B-01 step 1): turns the ImuSample
// stream between two keyframe capture times into one
// ImuPreintegrationMeasurement evidence, using the on-manifold integrator
// in include/sensor_models/imu_preintegration.hpp. This is the phase-1
// (mono + sonar + IMU + depth + heading, no DVL, no stereo) high-rate
// relative-motion source — see docs/imu-preintegration-design-2026-09-03.md
// for the design note and the estimator-side state extension it needs.
//
// What this frontend does and does not do:
// - It expresses every delta in base_link, applying the rig's
//   base_link_T_imu_link extrinsic to each raw reading (rotation for gyro
//   and accel; a centripetal lever-arm term w x (w x r) for accel; the
//   angular-acceleration term alpha x r is ignored — documented
//   assumption, negligible for a rigidly mounted IMU within a few cm of the
//   body origin at ROV angular rates).
// - It zero-order-holds each sample from its capture time to the next
//   sample's, clipped to [from_time, to_time]. The reading in effect at
//   from_time is the last sample at or before it (or, if none exists, the
//   first sample after it, held backwards) so the interval is covered
//   end-to-end without interpolating measurements.
// - It fails closed: any gap between consecutive held readings longer
//   than max_sample_gap_s, fewer than min_samples inside the interval, a
//   non-positive/oversized interval, non-finite readings, or a rig whose
//   imu_noise has a zero white-noise density all yield nullopt and count as
//   a failure for Health(). It never pads with a constant-acceleration
//   guess.
// - It does NOT estimate bias, gravity direction or the initial velocity;
//   the request carries the linearisation bias and the residual handles
//   re-linearisation through the bias Jacobians on the wire.
// - It holds no cross-call state other than health counters and the
//   evidence id counter, so calling it for the same interval twice yields
//   the same deltas (determinism test friendly).
#pragma once

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "measurement_api/frontend.hpp"
#include "sensor_models/imu_preintegration.hpp"

namespace uw::frontends {

struct ImuPreintegrationFrontendParams {
  // Samples whose header.sensor_id differs are ignored; empty accepts all.
  std::string imu_sensor_id = "imu0";
  // rig frame_tree child frame whose edge gives base_link_T_imu_link. A
  // missing edge is treated as identity unless require_extrinsic is set.
  std::string imu_frame = "imu_link";
  bool require_extrinsic = false;
  // Largest tolerated hold of one reading, i.e. the largest spacing between
  // consecutive samples (and between from_time and the first sample).
  // 50 ms == 10 missed samples at 200 Hz; beyond that the interval is
  // rejected rather than bridged.
  double max_sample_gap_s = 0.05;
  double min_delta_time_s = 1e-3;
  double max_delta_time_s = 5.0;
  int min_samples = 2;
  bool apply_lever_arm_correction = true;
  // Consecutive failures before Health() reports STATUS_UNAVAILABLE.
  int max_consecutive_failures = 3;
};

class ImuPreintegrationFrontend final : public uw::measurement_api::InertialFrontend {
 public:
  explicit ImuPreintegrationFrontend(ImuPreintegrationFrontendParams params);

  std::optional<uw::domain::MeasurementEvidence> Process(
      const std::vector<uw::domain::ImuSample>& samples,
      const uw::measurement_api::ImuPreintegrationRequest& request,
      const uw::domain::RigCalibrationSnapshot& calibration) override;

  uw::domain::HealthReport Health() const override;

  // Why the most recent Process() returned nullopt ("" after a success).
  const std::string& last_rejection_reason() const { return last_rejection_reason_; }

 private:
  std::optional<uw::domain::MeasurementEvidence> Reject(const std::string& reason);

  ImuPreintegrationFrontendParams params_;
  uint64_t intervals_processed_ = 0;
  uint64_t intervals_rejected_ = 0;
  uint64_t consecutive_failures_ = 0;
  uint64_t next_evidence_id_ = 1;
  std::string last_rejection_reason_;
};

}  // namespace uw::frontends
