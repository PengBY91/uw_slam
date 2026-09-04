#include "frontends/imu_stationary_initializer.hpp"

#include <cmath>
#include <random>
#include <vector>

#include <gtest/gtest.h>

using uw::frontends::ImuStationaryInitialization;
using uw::frontends::ImuStationaryInitializerParams;
using uw::frontends::InitializeFromStationaryWindow;

namespace {

constexpr double kGravity = 9.80665;
constexpr double kRateHz = 200.0;

uw::domain::RigCalibrationSnapshot MakeRig(double sigma_gyro_bias = 1.9393e-5,
                                           double sigma_accel_bias = 3.0e-3,
                                           double sigma_gyro_c = 1.6968e-4,
                                           double sigma_accel_c = 2.0e-3) {
  uw::domain::RigCalibrationSnapshot rig;
  auto* noise = rig.mutable_imu_noise();
  noise->set_sigma_gyro_c(sigma_gyro_c);
  noise->set_sigma_accel_c(sigma_accel_c);
  noise->set_sigma_gyro_bias(sigma_gyro_bias);
  noise->set_sigma_accel_bias(sigma_accel_bias);
  noise->set_sigma_gyro_bias_walk_c(1.0e-5);
  noise->set_sigma_accel_bias_walk_c(1.0e-4);
  noise->set_rate_hz(kRateHz);
  noise->set_gravity_mps2(kGravity);
  auto* edge = rig.add_frame_tree();
  edge->mutable_parent_frame()->set_value("base_link");
  edge->mutable_child_frame()->set_value("imu_link");
  auto* transform = edge->mutable_transform();
  for (double value : {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                       1.0}) {
    transform->add_matrix_row_major(value);
  }
  return rig;
}

uw::domain::ImuSample MakeSample(double time_s, const Eigen::Vector3d& accel,
                                 const Eigen::Vector3d& gyro) {
  uw::domain::ImuSample sample;
  auto* header = sample.mutable_header();
  header->mutable_observation_id()->set_value("imu_" + std::to_string(time_s));
  header->mutable_sensor_id()->set_value("imu0");
  header->mutable_sensor_frame()->set_value("imu_link");
  header->mutable_capture_time()->set_seconds(static_cast<int64_t>(time_s));
  header->mutable_capture_time()->set_nanos(
      static_cast<int32_t>(std::llround((time_s - std::floor(time_s)) * 1e9)));
  *header->mutable_receive_time() = header->capture_time();
  header->set_clock_domain(uw::domain::CLOCK_DOMAIN_SIMULATION);
  header->set_validity(uw::domain::ObservationHeader::VALIDITY_OK);
  for (int i = 0; i < 3; ++i) {
    sample.add_linear_acceleration_mps2(accel(i));
    sample.add_angular_velocity_radps(gyro(i));
  }
  return sample;
}

// A stationary window of `duration_s` ending exactly at `duration_s`, with a
// constant specific force `accel` and rate `gyro` on every sample.
std::vector<uw::domain::ImuSample> MakeWindow(double duration_s, const Eigen::Vector3d& accel,
                                              const Eigen::Vector3d& gyro) {
  std::vector<uw::domain::ImuSample> samples;
  const int count = static_cast<int>(std::llround(duration_s * kRateHz));
  for (int i = 0; i <= count; ++i) {
    samples.push_back(MakeSample(static_cast<double>(i) / kRateHz, accel, gyro));
  }
  return samples;
}

Eigen::Vector3d UprightSpecificForce() { return Eigen::Vector3d(0.0, 0.0, kGravity); }

}  // namespace

TEST(ImuStationaryInitializer, AcceptsAHalfSecondStationaryWindow) {
  const Eigen::Vector3d gyro_bias(1.0e-4, -2.0e-4, 3.0e-4);
  const Eigen::Vector3d accel_bias(0.01, -0.02, 0.0);
  const auto samples = MakeWindow(0.5, UprightSpecificForce() + accel_bias, gyro_bias);
  const auto result =
      InitializeFromStationaryWindow(samples, 0.5, MakeRig(), ImuStationaryInitializerParams());
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->mode, ImuStationaryInitialization::Mode::kStationary);
  EXPECT_TRUE(result->velocity_W.isZero());
  EXPECT_NEAR((result->bias_gyro - gyro_bias).norm(), 0.0, 1e-12);
  // Only the component of the accel bias ALONG the measured specific force
  // is separable from a tilt while stationary; the horizontal part is
  // absorbed into roll/pitch by construction. The z bias here is zero, so
  // the recovered magnitude error must match.
  EXPECT_NEAR(result->bias_accel.norm(), (UprightSpecificForce() + accel_bias).norm() - kGravity,
              1e-9);
}

TEST(ImuStationaryInitializer, FallsBackWhenTheWindowIsShorterThanTheMinimum) {
  const auto samples = MakeWindow(0.495, UprightSpecificForce(), Eigen::Vector3d::Zero());
  const auto result =
      InitializeFromStationaryWindow(samples, 0.495, MakeRig(), ImuStationaryInitializerParams());
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->mode, ImuStationaryInitialization::Mode::kWideVelocityPrior);
  EXPECT_TRUE(result->velocity_W.isZero()) << "zero is still the initial value, just not a claim";
  EXPECT_TRUE(result->bias_gyro.isZero());
  EXPECT_TRUE(result->bias_accel.isZero());
  for (int i = 0; i < 3; ++i) EXPECT_DOUBLE_EQ(result->sigma(i), 0.5);
  EXPECT_FALSE(result->detail.empty());
}

TEST(ImuStationaryInitializer, FallsBackWhenTheAngularRateExceedsTheThreshold) {
  const auto samples = MakeWindow(0.5, UprightSpecificForce(), Eigen::Vector3d(0.02, 0.0, 0.0));
  const auto result =
      InitializeFromStationaryWindow(samples, 0.5, MakeRig(), ImuStationaryInitializerParams());
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->mode, ImuStationaryInitialization::Mode::kWideVelocityPrior);
}

TEST(ImuStationaryInitializer, FallsBackWhenTheSpecificForceMagnitudeIsNotGravity) {
  // 0.2 m/s^2 of sustained acceleration: the magnitude test is the only
  // thing that can catch a vehicle under way but not turning.
  const auto samples = MakeWindow(0.5, Eigen::Vector3d(0.0, 0.0, kGravity + 0.2),
                                  Eigen::Vector3d::Zero());
  const auto result =
      InitializeFromStationaryWindow(samples, 0.5, MakeRig(), ImuStationaryInitializerParams());
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->mode, ImuStationaryInitialization::Mode::kWideVelocityPrior);
}

TEST(ImuStationaryInitializer, JudgesTheWindowMeanNotIndividualSamples) {
  // Per-sample white noise at the rig's own density puts roughly 8% of
  // samples outside the 0.05 m/s^2 gate at 200 Hz; the 0.5 s mean sits two
  // orders of magnitude inside it. A per-sample gate would make a correct
  // stationary recording un-initializable, which is why the threshold is
  // defined on the mean (see the header).
  std::mt19937_64 rng(7);
  std::normal_distribution<double> accel_noise(0.0, 2.0e-3 * std::sqrt(kRateHz));
  std::normal_distribution<double> gyro_noise(0.0, 1.6968e-4 * std::sqrt(kRateHz));
  std::vector<uw::domain::ImuSample> samples;
  int outside_gate = 0;
  const int count = static_cast<int>(std::llround(0.5 * kRateHz));
  for (int i = 0; i <= count; ++i) {
    const Eigen::Vector3d accel = UprightSpecificForce() +
                                   Eigen::Vector3d(accel_noise(rng), accel_noise(rng), accel_noise(rng));
    const Eigen::Vector3d gyro(gyro_noise(rng), gyro_noise(rng), gyro_noise(rng));
    if (std::abs(accel.norm() - kGravity) >= 0.05) ++outside_gate;
    samples.push_back(MakeSample(static_cast<double>(i) / kRateHz, accel, gyro));
  }
  ASSERT_GT(outside_gate, 0) << "the noise draw must actually exercise the distinction";
  const auto result =
      InitializeFromStationaryWindow(samples, 0.5, MakeRig(), ImuStationaryInitializerParams());
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->mode, ImuStationaryInitialization::Mode::kStationary);
}

TEST(ImuStationaryInitializer, RecoversRollAndPitchFromGravityAndLeavesYawAtZero) {
  // 10 degrees of roll: the body-frame specific force tilts with it.
  const double roll = 10.0 * M_PI / 180.0;
  const Eigen::Quaterniond rotation_WB(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
  const Eigen::Vector3d specific_force =
      rotation_WB.conjugate() * Eigen::Vector3d(0.0, 0.0, kGravity);
  const auto samples = MakeWindow(0.5, specific_force, Eigen::Vector3d::Zero());
  const auto result =
      InitializeFromStationaryWindow(samples, 0.5, MakeRig(), ImuStationaryInitializerParams());
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->mode, ImuStationaryInitialization::Mode::kStationary);
  // The recovered attitude must map the measured specific force back onto
  // world up...
  const Eigen::Vector3d world_up = result->rotation_WB * specific_force;
  EXPECT_NEAR(world_up.x(), 0.0, 1e-9);
  EXPECT_NEAR(world_up.y(), 0.0, 1e-9);
  EXPECT_NEAR(world_up.z(), kGravity, 1e-9);
  // ...with no yaw of its own: yaw is unobservable from gravity and stays
  // the canonical zero (PREP-B-02's heading factor takes it from there).
  const Eigen::Vector3d yaw_axis = result->rotation_WB * Eigen::Vector3d::UnitZ();
  const double yaw = std::atan2(result->rotation_WB.toRotationMatrix()(1, 0),
                                result->rotation_WB.toRotationMatrix()(0, 0));
  EXPECT_NEAR(yaw, 0.0, 1e-9) << "recovered yaw axis " << yaw_axis.transpose();
}

TEST(ImuStationaryInitializer, BiasSigmasAreNeverTighterThanTheRigPrior) {
  const auto rig = MakeRig();
  const auto samples = MakeWindow(0.5, UprightSpecificForce(), Eigen::Vector3d::Zero());
  const auto result = InitializeFromStationaryWindow(samples, 0.5, rig, ImuStationaryInitializerParams());
  ASSERT_TRUE(result.has_value());
  // The prior is centred on a value measured through white noise, so its
  // sigma must absorb that measurement's standard error on top of the rig's
  // own spread — never report more confidence than either source supports.
  for (int i = 3; i < 6; ++i) EXPECT_GT(result->sigma(i), rig.imu_noise().sigma_gyro_bias());
  for (int i = 6; i < 9; ++i) EXPECT_GT(result->sigma(i), rig.imu_noise().sigma_accel_bias());
  for (int i = 0; i < 9; ++i) EXPECT_TRUE(std::isfinite(result->sigma(i)) && result->sigma(i) > 0.0);
}

TEST(ImuStationaryInitializer, WideFallbackUsesTheRigBiasPriorUnchanged) {
  const auto rig = MakeRig();
  const auto samples = MakeWindow(0.2, UprightSpecificForce(), Eigen::Vector3d::Zero());
  const auto result = InitializeFromStationaryWindow(samples, 0.2, rig, ImuStationaryInitializerParams());
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->mode, ImuStationaryInitialization::Mode::kWideVelocityPrior);
  // Nothing was measured, so the rig's own prior is reported as-is.
  for (int i = 3; i < 6; ++i) EXPECT_DOUBLE_EQ(result->sigma(i), rig.imu_noise().sigma_gyro_bias());
  for (int i = 6; i < 9; ++i) EXPECT_DOUBLE_EQ(result->sigma(i), rig.imu_noise().sigma_accel_bias());
}

TEST(ImuStationaryInitializer, DoesNotAverageAPreBoundaryManoeuvreIntoLookingStationary) {
  // A real recording may manoeuvre for a long time before its first
  // keyframe and only settle at the end. Oscillatory motion averages toward
  // zero, so an unbounded look-back would pass the criterion on data that
  // is mostly motion -- and would report a window_duration_s of minutes,
  // whose sqrt shrinks the measured-bias standard error far below what the
  // data supports.
  std::vector<uw::domain::ImuSample> samples;
  const int count = static_cast<int>(std::llround(10.0 * kRateHz));
  for (int i = 0; i <= count; ++i) {
    const double time_s = static_cast<double>(i) / kRateHz;
    // 9 s of a hard sinusoidal yaw oscillation (a whole number of periods,
    // so its own mean really does cancel), then 1 s of rest -- exactly the
    // shape that fools an unbounded window.
    const bool moving = time_s < 9.0;
    const double rate = moving ? 0.8 * std::sin(2.0 * M_PI * time_s) : 0.0;
    const double lateral = moving ? 3.0 * std::sin(2.0 * M_PI * time_s) : 0.0;
    samples.push_back(MakeSample(time_s, Eigen::Vector3d(lateral, 0.0, kGravity),
                                 Eigen::Vector3d(0.0, 0.0, rate)));
  }

  const auto result =
      InitializeFromStationaryWindow(samples, 10.0, MakeRig(), ImuStationaryInitializerParams());
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->mode, ImuStationaryInitialization::Mode::kStationary)
      << "the last second is genuinely still and must still initialize";
  // The window is bounded, so its duration -- and therefore the standard
  // error the bias sigma is built from -- reflects the second that was
  // actually examined, not the ten seconds that were available.
  EXPECT_LE(result->window_duration_s, 1.0 + 1e-9);
  EXPECT_LE(result->sample_count, static_cast<int>(std::llround(1.0 * kRateHz)) + 1);

  // ...and with the whole 10 s folded in instead, the manoeuvre's own mean
  // would have sailed through the criterion, which is the failure mode this
  // bound exists to prevent.
  ImuStationaryInitializerParams unbounded;
  unbounded.stationary_window_s = 20.0;
  const auto naive = InitializeFromStationaryWindow(samples, 10.0, MakeRig(), unbounded);
  ASSERT_TRUE(naive.has_value());
  EXPECT_EQ(naive->mode, ImuStationaryInitialization::Mode::kStationary);
  EXPECT_GT(naive->window_duration_s, 9.0);
}

TEST(ImuStationaryInitializer, RejectsAWindowShorterThanTheRequiredStationarySpan) {
  ImuStationaryInitializerParams params;
  params.stationary_window_s = 0.2;  // shorter than min_stationary_duration_s
  const auto samples = MakeWindow(0.5, UprightSpecificForce(), Eigen::Vector3d::Zero());
  EXPECT_FALSE(InitializeFromStationaryWindow(samples, 0.5, MakeRig(), params).has_value())
      << "a look-back shorter than the span it must cover can never be satisfied";
}

TEST(ImuStationaryInitializer, FailsClosedOnARigWithoutUsableImuNoise) {
  const auto samples = MakeWindow(0.5, UprightSpecificForce(), Eigen::Vector3d::Zero());
  // No prior sigma to build a residual from: this is not a fallback, there
  // is nothing to fall back to.
  EXPECT_FALSE(InitializeFromStationaryWindow(samples, 0.5, MakeRig(/*sigma_gyro_bias=*/0.0),
                                              ImuStationaryInitializerParams())
                   .has_value());
  EXPECT_FALSE(InitializeFromStationaryWindow(samples, 0.5, MakeRig(1.9393e-5, -1.0),
                                              ImuStationaryInitializerParams())
                   .has_value());
  EXPECT_FALSE(InitializeFromStationaryWindow(
                   samples, 0.5, MakeRig(1.9393e-5, 3.0e-3, /*sigma_gyro_c=*/0.0),
                   ImuStationaryInitializerParams())
                   .has_value());
}

TEST(ImuStationaryInitializer, IgnoresSamplesAfterTheWindowEndAndRejectsMalformedOnes) {
  auto samples = MakeWindow(0.5, UprightSpecificForce(), Eigen::Vector3d::Zero());
  // A sample past the boundary, moving hard: it must not enter the window.
  samples.push_back(MakeSample(0.6, Eigen::Vector3d(5.0, 0.0, kGravity), Eigen::Vector3d(1.0, 0.0, 0.0)));
  const auto result =
      InitializeFromStationaryWindow(samples, 0.5, MakeRig(), ImuStationaryInitializerParams());
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->mode, ImuStationaryInitialization::Mode::kStationary);

  uw::domain::ImuSample malformed;
  *malformed.mutable_header() = samples.front().header();
  malformed.add_linear_acceleration_mps2(1.0);  // two entries, not three
  malformed.add_linear_acceleration_mps2(2.0);
  samples.push_back(malformed);
  EXPECT_FALSE(
      InitializeFromStationaryWindow(samples, 0.5, MakeRig(), ImuStationaryInitializerParams())
          .has_value())
      << "a malformed reading inside the window must fail closed, not be silently skipped";
}
