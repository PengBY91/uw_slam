#include <gtest/gtest.h>

#include "runtime/acoustic_optic_synchronizer.hpp"

namespace {

uw::domain::ImageFrame MakeImage(const std::string& sensor_id, int64_t seconds, int32_t nanos) {
  uw::domain::ImageFrame image;
  image.mutable_header()->mutable_sensor_id()->set_value(sensor_id);
  image.mutable_header()->mutable_capture_time()->set_seconds(seconds);
  image.mutable_header()->mutable_capture_time()->set_nanos(nanos);
  return image;
}

uw::domain::SonarFrame MakeSonar(const std::string& sensor_id, int64_t seconds, int32_t nanos) {
  uw::domain::SonarFrame sonar;
  sonar.mutable_header()->mutable_sensor_id()->set_value(sensor_id);
  sonar.mutable_header()->mutable_capture_time()->set_seconds(seconds);
  sonar.mutable_header()->mutable_capture_time()->set_nanos(nanos);
  return sonar;
}

uw::domain::RigCalibrationSnapshot MakeRigWithOffsets() {
  uw::domain::RigCalibrationSnapshot rig;
  (*rig.mutable_time_offset_seconds())["camera_left"] = 0.0;
  (*rig.mutable_time_offset_seconds())["sonar0"] = 0.01;  // sonar clock reads 10ms early
  return rig;
}

}  // namespace

TEST(AcousticOpticSynchronizer, AcceptsFramesWithinToleranceAfterOffsetCorrection) {
  const auto rig = MakeRigWithOffsets();
  // corrected(left) = 100.0 + 0.0 = 100.0
  const auto left = MakeImage("camera_left", 100, 0);
  // corrected(sonar) = 99.99 + 0.01 = 100.0 -- exact match after offset correction.
  const auto sonar = MakeSonar("sonar0", 99, 990'000'000);

  uw::runtime::SynchronizerParams params;
  params.max_time_delta_s = 0.02;
  const auto bundle = uw::runtime::SynchronizeAcousticOptic(left, std::nullopt, sonar, rig, params);
  ASSERT_TRUE(bundle.has_value());
  EXPECT_NEAR(bundle->max_pairwise_time_delta_s, 0.0, 1e-6);
}

TEST(AcousticOpticSynchronizer, RejectsFramesBeyondTolerance) {
  const auto rig = MakeRigWithOffsets();
  const auto left = MakeImage("camera_left", 100, 0);
  const auto sonar = MakeSonar("sonar0", 100, 200'000'000);  // 200ms raw drift, dwarfs the 10ms offset

  uw::runtime::SynchronizerParams params;
  params.max_time_delta_s = 0.02;
  const auto bundle = uw::runtime::SynchronizeAcousticOptic(left, std::nullopt, sonar, rig, params);
  EXPECT_FALSE(bundle.has_value());
}

TEST(AcousticOpticSynchronizer, DefaultsMissingSensorOffsetToZero) {
  uw::domain::RigCalibrationSnapshot rig;  // no time_offset_seconds entries at all
  const auto left = MakeImage("camera_left", 100, 0);
  const auto sonar = MakeSonar("sonar0", 100, 5'000'000);  // 5ms drift

  uw::runtime::SynchronizerParams params;
  params.max_time_delta_s = 0.02;
  const auto bundle = uw::runtime::SynchronizeAcousticOptic(left, std::nullopt, sonar, rig, params);
  ASSERT_TRUE(bundle.has_value());
  EXPECT_NEAR(bundle->max_pairwise_time_delta_s, 0.005, 1e-6);
}
