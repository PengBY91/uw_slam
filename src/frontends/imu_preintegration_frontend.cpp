#include "frontends/imu_preintegration_frontend.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

#include <Eigen/Geometry>

#include "sensor_models/geometry.hpp"

namespace uw::frontends {

namespace {

struct BodyReading {
  double time_s = 0.0;
  Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel = Eigen::Vector3d::Zero();
  const uw::domain::ImuSample* sample = nullptr;
};

bool ReadVector3(const google::protobuf::RepeatedField<double>& in, Eigen::Vector3d* out) {
  if (in.size() != 3) return false;
  *out = Eigen::Vector3d(in.Get(0), in.Get(1), in.Get(2));
  return std::isfinite(out->x()) && std::isfinite(out->y()) && std::isfinite(out->z());
}

}  // namespace

ImuPreintegrationFrontend::ImuPreintegrationFrontend(ImuPreintegrationFrontendParams params)
    : params_(std::move(params)) {}

std::optional<uw::domain::MeasurementEvidence> ImuPreintegrationFrontend::Reject(const std::string& reason) {
  ++intervals_rejected_;
  ++consecutive_failures_;
  last_rejection_reason_ = reason;
  return std::nullopt;
}

std::optional<uw::domain::MeasurementEvidence> ImuPreintegrationFrontend::Process(
    const std::vector<uw::domain::ImuSample>& samples,
    const uw::measurement_api::ImuPreintegrationRequest& request,
    const uw::domain::RigCalibrationSnapshot& rig) {
  ++intervals_processed_;

  const double t0 = uw::domain::ToSeconds(request.from_time);
  const double t1 = uw::domain::ToSeconds(request.to_time);
  const double interval = t1 - t0;
  if (!std::isfinite(interval) || interval < params_.min_delta_time_s || interval > params_.max_delta_time_s) {
    return Reject("interval_out_of_range");
  }

  std::string noise_error;
  const auto noise = uw::sensor_models::ImuPreintegrationNoise::FromRig(rig, &noise_error);
  if (!noise.has_value()) return Reject("rig_imu_noise_invalid");

  // base_link_T_imu_link: rotate readings into the body frame.
  Eigen::Matrix3d R_body_imu = Eigen::Matrix3d::Identity();
  Eigen::Vector3d lever_arm = Eigen::Vector3d::Zero();
  const auto extrinsic = uw::sensor_models::FindEdgePose(rig, params_.imu_frame);
  if (extrinsic.has_value()) {
    R_body_imu = extrinsic->rotation.toRotationMatrix();
    lever_arm = extrinsic->translation;
  } else if (params_.require_extrinsic) {
    return Reject("imu_extrinsic_missing");
  }

  // Convert + filter + order. Input is expected in capture-time order but
  // a stable sort costs nothing at these sizes and removes the assumption.
  std::vector<BodyReading> readings;
  readings.reserve(samples.size());
  for (const auto& sample : samples) {
    if (!params_.imu_sensor_id.empty() && sample.header().sensor_id().value() != params_.imu_sensor_id) continue;
    Eigen::Vector3d gyro_imu;
    Eigen::Vector3d accel_imu;
    if (!ReadVector3(sample.angular_velocity_radps(), &gyro_imu) ||
        !ReadVector3(sample.linear_acceleration_mps2(), &accel_imu)) {
      return Reject("imu_sample_malformed");
    }
    BodyReading reading;
    reading.time_s = uw::domain::ToSeconds(sample.header().capture_time());
    if (!std::isfinite(reading.time_s)) return Reject("imu_sample_malformed");
    reading.gyro = R_body_imu * gyro_imu;
    reading.accel = R_body_imu * accel_imu;
    if (params_.apply_lever_arm_correction && lever_arm.squaredNorm() > 0.0) {
      // Specific force at the body origin = specific force at the IMU
      // minus the centripetal term; angular acceleration is ignored.
      reading.accel -= reading.gyro.cross(reading.gyro.cross(lever_arm));
    }
    reading.sample = &sample;
    readings.push_back(reading);
  }
  std::stable_sort(readings.begin(), readings.end(),
                   [](const BodyReading& a, const BodyReading& b) { return a.time_s < b.time_s; });

  // Reading in effect at t0: last sample at or before t0, else the first
  // sample after t0 (held backwards). Then every sample strictly inside
  // (t0, t1) starts a new hold; t1 closes the last one.
  auto first_after_t0 = std::upper_bound(
      readings.begin(), readings.end(), t0,
      [](double t, const BodyReading& r) { return t < r.time_s; });
  if (readings.empty()) return Reject("no_imu_samples");
  std::size_t held = first_after_t0 == readings.begin() ? 0 : static_cast<std::size_t>(first_after_t0 - readings.begin()) - 1;
  const bool held_backwards = first_after_t0 == readings.begin();

  uw::sensor_models::ImuPreintegrator integrator(*noise, rig.imu_noise().gravity_mps2() > 0.0
                                                              ? rig.imu_noise().gravity_mps2()
                                                              : 9.80665);
  integrator.Reset(request.bias_gyro, request.bias_accel);

  double segment_start = t0;
  double max_hold = 0.0;
  uint32_t samples_inside = 0;
  const uw::domain::ImuSample* first_inside = nullptr;
  const uw::domain::ImuSample* last_inside = nullptr;
  std::size_t next = static_cast<std::size_t>(first_after_t0 - readings.begin());
  while (true) {
    const bool has_next = next < readings.size() && readings[next].time_s < t1;
    const double segment_end = has_next ? readings[next].time_s : t1;
    const double dt = segment_end - segment_start;
    // How stale the held reading is by the end of this segment, measured
    // from the reading's OWN capture time rather than from segment_start.
    // For interior segments the two are the same, but for the first one
    // they are not: a reading captured well before t0 (or, in the
    // held-backwards case, well after t0) would otherwise be applied
    // across a short interval without ever tripping the gap check --
    // exactly the "stale IMU data" case this is meant to fail closed on.
    const double hold_span = std::max(segment_end, readings[held].time_s) -
                             std::min(segment_start, readings[held].time_s);
    if (hold_span > params_.max_sample_gap_s) return Reject("imu_gap_too_large");
    max_hold = std::max(max_hold, hold_span);
    if (dt > 0.0 && !integrator.Integrate(readings[held].gyro, readings[held].accel, dt)) {
      return Reject("imu_integration_failed");
    }
    if (!has_next) break;
    held = next;
    ++next;
    ++samples_inside;
    if (first_inside == nullptr) first_inside = readings[held].sample;
    last_inside = readings[held].sample;
    segment_start = segment_end;
  }
  // A sample exactly at t1 belongs to this interval's sample count too.
  if (next < readings.size() && readings[next].time_s == t1) {
    ++samples_inside;
    if (first_inside == nullptr) first_inside = readings[next].sample;
    last_inside = readings[next].sample;
  }
  if (samples_inside < static_cast<uint32_t>(std::max(params_.min_samples, 0))) return Reject("too_few_imu_samples");

  const auto& delta = integrator.delta();
  uw::domain::ImuPreintegrationMeasurement measurement = delta.ToProto();
  measurement.set_sample_count(samples_inside);
  measurement.mutable_from_keyframe()->set_value(request.from_keyframe_id);
  measurement.mutable_to_keyframe()->set_value(request.to_keyframe_id);

  uw::domain::EvidenceId evidence_id;
  evidence_id.set_value("imu_preintegration_" + std::to_string(next_evidence_id_++));
  // First and last sample only: a 200 Hz stream over a multi-second
  // interval would otherwise carry hundreds of ids per evidence.
  std::vector<uw::domain::ObservationId> sources;
  if (first_inside != nullptr && first_inside->header().has_observation_id()) {
    sources.push_back(first_inside->header().observation_id());
  }
  if (last_inside != nullptr && last_inside != first_inside && last_inside->header().has_observation_id()) {
    sources.push_back(last_inside->header().observation_id());
  }

  auto evidence = uw::domain::MakeEvidence(evidence_id, sources, measurement, /*noise_scale=*/1.0,
                                           "imu_preintegration_frontend_v1");
  auto& quality = *evidence.mutable_quality_features();
  quality["sample_count"] = static_cast<double>(samples_inside);
  quality["delta_time_s"] = delta.delta_time_s;
  quality["max_hold_s"] = max_hold;
  quality["mean_rate_hz"] = delta.delta_time_s > 0.0 ? samples_inside / delta.delta_time_s : 0.0;
  quality["held_backwards_at_start"] = held_backwards ? 1.0 : 0.0;
  quality["lever_arm_m"] = lever_arm.norm();

  consecutive_failures_ = 0;
  last_rejection_reason_.clear();
  return evidence;
}

uw::domain::HealthReport ImuPreintegrationFrontend::Health() const {
  uw::domain::HealthReport report;
  report.set_component_id("imu_preintegration_frontend");
  if (consecutive_failures_ >= static_cast<uint64_t>(std::max(params_.max_consecutive_failures, 1))) {
    report.set_status(uw::domain::HealthReport::STATUS_UNAVAILABLE);
    report.set_reason_code("imu_preintegration_unavailable");
  } else if (consecutive_failures_ > 0) {
    report.set_status(uw::domain::HealthReport::STATUS_SUSPECT);
    report.set_reason_code(last_rejection_reason_);
  } else {
    report.set_status(uw::domain::HealthReport::STATUS_HEALTHY);
  }
  return report;
}

}  // namespace uw::frontends
