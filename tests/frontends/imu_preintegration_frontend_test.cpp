#include "frontends/imu_preintegration_frontend.hpp"

#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "sensor_models/geometry.hpp"
#include "sensor_models/so3.hpp"

using uw::frontends::ImuPreintegrationFrontend;
using uw::frontends::ImuPreintegrationFrontendParams;
using uw::measurement_api::ImuPreintegrationRequest;
using uw::sensor_models::Pose3;
using uw::sensor_models::PreintegratedImuDelta;
namespace so3 = uw::sensor_models::so3;

namespace {

constexpr double kGravity = 9.80665;

// A rig with a valid imu_noise block and an imu_link edge (identity unless
// `imu_pose` is given) — the only two rig fields the frontend reads.
uw::domain::RigCalibrationSnapshot MakeRig(const Pose3& imu_pose = Pose3::Identity()) {
  uw::domain::RigCalibrationSnapshot rig;
  rig.mutable_calibration_version()->set_value("imu_test_rig");
  auto* edge = rig.add_frame_tree();
  edge->mutable_parent_frame()->set_value("base_link");
  edge->mutable_child_frame()->set_value("imu_link");
  *edge->mutable_transform() = imu_pose.ToProto();
  auto* noise = rig.mutable_imu_noise();
  noise->set_sigma_gyro_c(1.7e-4);
  noise->set_sigma_accel_c(2.0e-3);
  noise->set_sigma_gyro_bias(1.9e-5);
  noise->set_sigma_accel_bias(3.0e-3);
  noise->set_rate_hz(200.0);
  noise->set_gravity_mps2(kGravity);
  return rig;
}

uw::domain::ImuSample MakeSample(double t, const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel,
                                 const std::string& sensor_id = "imu0") {
  uw::domain::ImuSample sample;
  auto* header = sample.mutable_header();
  header->mutable_observation_id()->set_value("imu_" + std::to_string(static_cast<int>(std::llround(t * 1e3))));
  header->mutable_sensor_id()->set_value(sensor_id);
  *header->mutable_capture_time() = uw::domain::FromSeconds(t);
  for (int i = 0; i < 3; ++i) sample.add_angular_velocity_radps(gyro(i));
  for (int i = 0; i < 3; ++i) sample.add_linear_acceleration_mps2(accel(i));
  return sample;
}

// 200 Hz static, level readings (gyro 0, specific force +g up in the IMU
// frame given by `R_body_imu`) from t=0 to t=duration.
std::vector<uw::domain::ImuSample> StaticStream(double duration_s, double rate_hz = 200.0,
                                                const Eigen::Matrix3d& R_body_imu = Eigen::Matrix3d::Identity()) {
  std::vector<uw::domain::ImuSample> samples;
  const int n = static_cast<int>(std::llround(duration_s * rate_hz));
  const Eigen::Vector3d accel_imu = R_body_imu.transpose() * Eigen::Vector3d(0.0, 0.0, kGravity);
  for (int k = 0; k <= n; ++k) samples.push_back(MakeSample(k / rate_hz, Eigen::Vector3d::Zero(), accel_imu));
  return samples;
}

ImuPreintegrationRequest MakeRequest(double t0, double t1) {
  ImuPreintegrationRequest request;
  request.from_keyframe_id = "kf0";
  request.to_keyframe_id = "kf1";
  request.from_time = uw::domain::FromSeconds(t0);
  request.to_time = uw::domain::FromSeconds(t1);
  return request;
}

PreintegratedImuDelta Payload(const uw::domain::MeasurementEvidence& evidence) {
  EXPECT_TRUE(uw::domain::HasPayload<uw::domain::ImuPreintegrationMeasurement>(evidence));
  std::string error;
  auto delta = PreintegratedImuDelta::FromProto(
      uw::domain::GetPayload<uw::domain::ImuPreintegrationMeasurement>(evidence), &error);
  EXPECT_TRUE(delta.has_value()) << error;
  return delta.value_or(PreintegratedImuDelta{});
}

TEST(ImuPreintegrationFrontend, StaticIntervalYieldsGravityDeltaAndKeyframeIds) {
  ImuPreintegrationFrontend frontend(ImuPreintegrationFrontendParams{});
  const auto samples = StaticStream(1.0);
  // Interval boundaries deliberately between samples (t0=0.2025, t1=0.8025).
  const auto evidence = frontend.Process(samples, MakeRequest(0.2025, 0.8025), MakeRig());
  ASSERT_TRUE(evidence.has_value()) << frontend.last_rejection_reason();
  const auto& measurement = uw::domain::GetPayload<uw::domain::ImuPreintegrationMeasurement>(*evidence);
  EXPECT_EQ(measurement.from_keyframe().value(), "kf0");
  EXPECT_EQ(measurement.to_keyframe().value(), "kf1");
  EXPECT_EQ(measurement.sample_count(), 120u);  // samples strictly inside (0.2025, 0.8025)
  EXPECT_EQ(evidence->evidence_id().value(), "imu_preintegration_1");
  EXPECT_EQ(evidence->algorithm_version(), "imu_preintegration_frontend_v1");
  EXPECT_EQ(evidence->source_observations_size(), 2);
  const auto delta = Payload(*evidence);
  EXPECT_NEAR(delta.delta_time_s, 0.6, 1e-9);
  EXPECT_LT((delta.delta_rotation - Eigen::Matrix3d::Identity()).norm(), 1e-12);
  EXPECT_LT((delta.delta_velocity - Eigen::Vector3d(0.0, 0.0, kGravity * 0.6)).norm(), 1e-9);
  EXPECT_LT((delta.delta_position - Eigen::Vector3d(0.0, 0.0, 0.5 * kGravity * 0.36)).norm(), 1e-9);
  EXPECT_GT(delta.covariance(3, 3), 0.0);
  EXPECT_EQ(evidence->quality_features().at("sample_count"), 120.0);
  EXPECT_NEAR(evidence->quality_features().at("max_hold_s"), 0.005, 1e-9);
  EXPECT_EQ(frontend.Health().status(), uw::domain::HealthReport::STATUS_HEALTHY);
}

TEST(ImuPreintegrationFrontend, SameIntervalTwiceIsDeterministic) {
  ImuPreintegrationFrontend frontend(ImuPreintegrationFrontendParams{});
  const auto samples = StaticStream(1.0);
  const auto a = frontend.Process(samples, MakeRequest(0.1, 0.5), MakeRig());
  const auto b = frontend.Process(samples, MakeRequest(0.1, 0.5), MakeRig());
  ASSERT_TRUE(a.has_value() && b.has_value());
  const auto da = Payload(*a);
  const auto db = Payload(*b);
  EXPECT_EQ(da.delta_velocity, db.delta_velocity);
  EXPECT_EQ(da.delta_position, db.delta_position);
  EXPECT_EQ(da.covariance, db.covariance);
}

TEST(ImuPreintegrationFrontend, AppliesImuExtrinsicRotationAndLeverArm) {
  // IMU mounted rolled 90 deg about body x and 0.1 m ahead of the origin.
  Pose3 imu_pose;
  imu_pose.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()));
  imu_pose.translation = Eigen::Vector3d(0.1, 0.0, 0.0);
  const Eigen::Matrix3d R_body_imu = imu_pose.rotation.toRotationMatrix();
  const auto rig = MakeRig(imu_pose);

  // Static: the IMU reads gravity along its own axes; the body-frame delta
  // must still come out along body z.
  ImuPreintegrationFrontend frontend(ImuPreintegrationFrontendParams{});
  auto evidence = frontend.Process(StaticStream(1.0, 200.0, R_body_imu), MakeRequest(0.0, 0.5), rig);
  ASSERT_TRUE(evidence.has_value()) << frontend.last_rejection_reason();
  auto delta = Payload(*evidence);
  EXPECT_LT((delta.delta_velocity - Eigen::Vector3d(0.0, 0.0, kGravity * 0.5)).norm(), 1e-9);
  EXPECT_NEAR(evidence->quality_features().at("lever_arm_m"), 0.1, 1e-12);

  // Spinning about body z at 1 rad/s with the IMU 0.1 m ahead: the IMU
  // feels a centripetal specific force of w^2 r = 0.1 m/s^2 towards the
  // origin (body -x), which the lever-arm correction must remove so the
  // body-origin delta is gravity only. Gyro reads body z in IMU axes.
  const Eigen::Vector3d rate_body(0.0, 0.0, 1.0);
  const Eigen::Vector3d accel_body = Eigen::Vector3d(0.0, 0.0, kGravity) - Eigen::Vector3d(0.1, 0.0, 0.0);
  std::vector<uw::domain::ImuSample> spinning;
  for (int k = 0; k <= 200; ++k) {
    spinning.push_back(MakeSample(k / 200.0, R_body_imu.transpose() * rate_body, R_body_imu.transpose() * accel_body));
  }
  evidence = frontend.Process(spinning, MakeRequest(0.0, 1.0), rig);
  ASSERT_TRUE(evidence.has_value()) << frontend.last_rejection_reason();
  delta = Payload(*evidence);
  EXPECT_LT((delta.delta_rotation - so3::Exp(rate_body)).norm(), 1e-9);
  EXPECT_LT((delta.delta_velocity - Eigen::Vector3d(0.0, 0.0, kGravity)).norm(), 1e-9);

  // Without the correction the centripetal term leaks into the delta.
  ImuPreintegrationFrontendParams no_lever;
  no_lever.apply_lever_arm_correction = false;
  ImuPreintegrationFrontend uncorrected(no_lever);
  evidence = uncorrected.Process(spinning, MakeRequest(0.0, 1.0), rig);
  ASSERT_TRUE(evidence.has_value());
  EXPECT_GT((Payload(*evidence).delta_velocity - Eigen::Vector3d(0.0, 0.0, kGravity)).norm(), 0.05);
}

TEST(ImuPreintegrationFrontend, FailsClosedOnGapsTooFewSamplesAndBadIntervals) {
  ImuPreintegrationFrontendParams params;
  params.max_consecutive_failures = 3;
  ImuPreintegrationFrontend frontend(params);
  const auto rig = MakeRig();

  // 60 ms hole in the middle of a 200 Hz stream (> 50 ms max hold).
  std::vector<uw::domain::ImuSample> gappy;
  for (const auto& s : StaticStream(1.0)) {
    const double t = uw::domain::ToSeconds(s.header().capture_time());
    if (t > 0.40 && t < 0.47) continue;
    gappy.push_back(s);
  }
  EXPECT_FALSE(frontend.Process(gappy, MakeRequest(0.2, 0.8), rig).has_value());
  EXPECT_EQ(frontend.last_rejection_reason(), "imu_gap_too_large");
  EXPECT_EQ(frontend.Health().status(), uw::domain::HealthReport::STATUS_SUSPECT);
  // The same stream is fine for an interval that avoids the hole.
  EXPECT_TRUE(frontend.Process(gappy, MakeRequest(0.5, 0.8), rig).has_value());
  EXPECT_EQ(frontend.Health().status(), uw::domain::HealthReport::STATUS_HEALTHY);

  // Interval starting long after the last sample: nothing to hold.
  EXPECT_FALSE(frontend.Process(StaticStream(0.3), MakeRequest(0.5, 0.7), rig).has_value());
  EXPECT_EQ(frontend.last_rejection_reason(), "imu_gap_too_large");

  // Too few samples inside a tiny interval.
  EXPECT_FALSE(frontend.Process(StaticStream(1.0), MakeRequest(0.301, 0.304), rig).has_value());
  EXPECT_EQ(frontend.last_rejection_reason(), "too_few_imu_samples");

  // Reversed / overlong interval.
  EXPECT_FALSE(frontend.Process(StaticStream(1.0), MakeRequest(0.5, 0.4), rig).has_value());
  EXPECT_EQ(frontend.last_rejection_reason(), "interval_out_of_range");
  EXPECT_EQ(frontend.Health().status(), uw::domain::HealthReport::STATUS_UNAVAILABLE);

  // Empty sample vector.
  EXPECT_FALSE(frontend.Process({}, MakeRequest(0.0, 0.5), rig).has_value());
  EXPECT_EQ(frontend.last_rejection_reason(), "no_imu_samples");
}

TEST(ImuPreintegrationFrontend, RejectsAStaleHeldReadingEvenWhenTheIntervalIsShort) {
  // The stream stops at 0.3 s; the interval 0.50 -> 0.53 is itself well
  // under max_sample_gap_s, so only measuring the segment length would
  // silently hold a 200 ms old reading across it. The guard measures
  // staleness from the reading's own capture time instead.
  ImuPreintegrationFrontend frontend(ImuPreintegrationFrontendParams{});
  const auto rig = MakeRig();
  auto samples = StaticStream(0.3);
  EXPECT_FALSE(frontend.Process(samples, MakeRequest(0.50, 0.53), rig).has_value());
  EXPECT_EQ(frontend.last_rejection_reason(), "imu_gap_too_large");

  // Symmetric case at the start: the first sample arrives long after t0,
  // so holding it backwards would cover an interval it never observed.
  std::vector<uw::domain::ImuSample> late;
  for (int k = 0; k <= 100; ++k) {
    late.push_back(MakeSample(0.5 + k / 200.0, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, kGravity)));
  }
  EXPECT_FALSE(frontend.Process(late, MakeRequest(0.20, 0.72), rig).has_value());
  EXPECT_EQ(frontend.last_rejection_reason(), "imu_gap_too_large");

  // Within the tolerance the same held-backwards start is accepted.
  EXPECT_TRUE(frontend.Process(late, MakeRequest(0.47, 0.72), rig).has_value())
      << frontend.last_rejection_reason();
}

TEST(ImuPreintegrationFrontend, IgnoresOtherSensorsAndRejectsMalformedSamples) {
  ImuPreintegrationFrontend frontend(ImuPreintegrationFrontendParams{});
  const auto rig = MakeRig();
  std::vector<uw::domain::ImuSample> mixed;
  for (int k = 0; k <= 100; ++k) {
    mixed.push_back(MakeSample(k / 200.0, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, kGravity), "imu_other"));
  }
  EXPECT_FALSE(frontend.Process(mixed, MakeRequest(0.0, 0.5), rig).has_value());
  EXPECT_EQ(frontend.last_rejection_reason(), "no_imu_samples");

  auto malformed = StaticStream(1.0);
  malformed[10].clear_angular_velocity_radps();
  EXPECT_FALSE(frontend.Process(malformed, MakeRequest(0.0, 0.5), rig).has_value());
  EXPECT_EQ(frontend.last_rejection_reason(), "imu_sample_malformed");
}

TEST(ImuPreintegrationFrontend, RejectsRigWithoutUsableImuNoise) {
  ImuPreintegrationFrontend frontend(ImuPreintegrationFrontendParams{});
  uw::domain::RigCalibrationSnapshot rig = MakeRig();
  rig.mutable_imu_noise()->set_sigma_accel_c(0.0);
  EXPECT_FALSE(frontend.Process(StaticStream(1.0), MakeRequest(0.0, 0.5), rig).has_value());
  EXPECT_EQ(frontend.last_rejection_reason(), "rig_imu_noise_invalid");

  ImuPreintegrationFrontendParams strict;
  strict.require_extrinsic = true;
  ImuPreintegrationFrontend strict_frontend(strict);
  uw::domain::RigCalibrationSnapshot no_edge = MakeRig();
  no_edge.clear_frame_tree();
  EXPECT_FALSE(strict_frontend.Process(StaticStream(1.0), MakeRequest(0.0, 0.5), no_edge).has_value());
  EXPECT_EQ(strict_frontend.last_rejection_reason(), "imu_extrinsic_missing");
  // Non-strict: identity extrinsic, still integrates.
  EXPECT_TRUE(frontend.Process(StaticStream(1.0), MakeRequest(0.0, 0.5), no_edge).has_value());
}

}  // namespace
