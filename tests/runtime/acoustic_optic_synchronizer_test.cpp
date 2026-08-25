#include <limits>

#include <gtest/gtest.h>

#include "runtime/acoustic_optic_synchronizer.hpp"

namespace {

using uw::runtime::SynchronizationStatus;

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
  const auto decision = uw::runtime::SynchronizeAcousticOptic(left, std::nullopt, sonar, rig, params);
  EXPECT_EQ(decision.status, SynchronizationStatus::kSynchronized);
  ASSERT_TRUE(decision.bundle.has_value());
  EXPECT_NEAR(decision.max_pairwise_time_delta_s, 0.0, 1e-6);
  EXPECT_NEAR(decision.bundle->max_pairwise_time_delta_s, 0.0, 1e-6);
}

TEST(AcousticOpticSynchronizer, RejectsFramesBeyondToleranceButReportsRealDelta) {
  const auto rig = MakeRigWithOffsets();
  const auto left = MakeImage("camera_left", 100, 0);
  const auto sonar = MakeSonar("sonar0", 100, 200'000'000);  // 200ms raw drift, dwarfs the 10ms offset

  uw::runtime::SynchronizerParams params;
  params.max_time_delta_s = 0.02;
  const auto decision = uw::runtime::SynchronizeAcousticOptic(left, std::nullopt, sonar, rig, params);
  EXPECT_EQ(decision.status, SynchronizationStatus::kTimeDeltaExceeded);
  EXPECT_FALSE(decision.bundle.has_value());
  // Real delta (corrected sonar 100.2 + 0.01 offset = 100.21 vs. left's
  // 100.0 = 0.21s), NOT 0.0 -- a caller must be able to see how badly out
  // of sync this was.
  EXPECT_NEAR(decision.max_pairwise_time_delta_s, 0.21, 1e-6);
}

TEST(AcousticOpticSynchronizer, RejectsMissingSensorOffset) {
  uw::domain::RigCalibrationSnapshot rig;  // no time_offset_seconds entries at all
  const auto left = MakeImage("camera_left", 100, 0);
  const auto sonar = MakeSonar("sonar0", 100, 5'000'000);  // 5ms drift

  uw::runtime::SynchronizerParams params;
  params.max_time_delta_s = 0.02;
  const auto decision = uw::runtime::SynchronizeAcousticOptic(left, std::nullopt, sonar, rig, params);
  EXPECT_EQ(decision.status, SynchronizationStatus::kInvalidTimestamp);
  EXPECT_FALSE(decision.bundle.has_value());
}

TEST(AcousticOpticSynchronizer, NoSonarIsNotAFailure) {
  uw::domain::RigCalibrationSnapshot rig;
  (*rig.mutable_time_offset_seconds())["camera_left"] = 0.0;
  const auto left = MakeImage("camera_left", 100, 0);

  uw::runtime::SynchronizerParams params;
  const auto decision =
      uw::runtime::SynchronizeAcousticOptic(left, std::nullopt, std::nullopt, rig, params);
  EXPECT_EQ(decision.status, SynchronizationStatus::kNoSonar);
  EXPECT_FALSE(decision.bundle.has_value());
  EXPECT_NEAR(decision.max_pairwise_time_delta_s, 0.0, 1e-9);
}

TEST(AcousticOpticSynchronizer, AllZeroStampIsALegalTimestamp) {
  uw::domain::RigCalibrationSnapshot rig;
  (*rig.mutable_time_offset_seconds())["camera_left"] = 0.0;
  (*rig.mutable_time_offset_seconds())["sonar0"] = 0.0;
  const auto left = MakeImage("camera_left", 0, 0);
  const auto sonar = MakeSonar("sonar0", 0, 0);

  uw::runtime::SynchronizerParams params;
  const auto decision = uw::runtime::SynchronizeAcousticOptic(left, std::nullopt, sonar, rig, params);
  EXPECT_EQ(decision.status, SynchronizationStatus::kSynchronized);
  ASSERT_TRUE(decision.bundle.has_value());
  EXPECT_NEAR(decision.max_pairwise_time_delta_s, 0.0, 1e-9);
}

TEST(AcousticOpticSynchronizer, RejectsNegativeNanos) {
  uw::domain::RigCalibrationSnapshot rig;
  auto left = MakeImage("camera_left", 100, 0);
  left.mutable_header()->mutable_capture_time()->set_nanos(-1);
  const auto sonar = MakeSonar("sonar0", 100, 0);

  uw::runtime::SynchronizerParams params;
  const auto decision = uw::runtime::SynchronizeAcousticOptic(left, std::nullopt, sonar, rig, params);
  EXPECT_EQ(decision.status, SynchronizationStatus::kInvalidTimestamp);
  EXPECT_FALSE(decision.bundle.has_value());
  EXPECT_NEAR(decision.max_pairwise_time_delta_s, 0.0, 1e-9);
}

TEST(AcousticOpticSynchronizer, RejectsNanosAtOrAboveOneBillion) {
  uw::domain::RigCalibrationSnapshot rig;
  auto left = MakeImage("camera_left", 100, 0);
  left.mutable_header()->mutable_capture_time()->set_nanos(1'000'000'000);
  const auto sonar = MakeSonar("sonar0", 100, 0);

  uw::runtime::SynchronizerParams params;
  const auto decision = uw::runtime::SynchronizeAcousticOptic(left, std::nullopt, sonar, rig, params);
  EXPECT_EQ(decision.status, SynchronizationStatus::kInvalidTimestamp);
  EXPECT_FALSE(decision.bundle.has_value());
}

TEST(AcousticOpticSynchronizer, RejectsAbsentCaptureTime) {
  uw::domain::RigCalibrationSnapshot rig;
  (*rig.mutable_time_offset_seconds())["camera_left"] = 0.0;
  (*rig.mutable_time_offset_seconds())["sonar0"] = 0.0;
  auto left = MakeImage("camera_left", 100, 0);
  left.mutable_header()->clear_capture_time();
  const auto sonar = MakeSonar("sonar0", 100, 0);

  const auto decision = uw::runtime::SynchronizeAcousticOptic(
      left, std::nullopt, sonar, rig, uw::runtime::SynchronizerParams{});
  EXPECT_EQ(decision.status, SynchronizationStatus::kInvalidTimestamp);
  EXPECT_FALSE(decision.bundle.has_value());
}

TEST(AcousticOpticSynchronizer, RejectsEmptySensorId) {
  uw::domain::RigCalibrationSnapshot rig;
  const auto left = MakeImage("", 100, 0);
  const auto sonar = MakeSonar("sonar0", 100, 0);

  uw::runtime::SynchronizerParams params;
  const auto decision = uw::runtime::SynchronizeAcousticOptic(left, std::nullopt, sonar, rig, params);
  EXPECT_EQ(decision.status, SynchronizationStatus::kInvalidTimestamp);
  EXPECT_FALSE(decision.bundle.has_value());
}

TEST(AcousticOpticSynchronizer, RejectsNonFiniteOffset) {
  uw::domain::RigCalibrationSnapshot rig;
  (*rig.mutable_time_offset_seconds())["camera_left"] = std::numeric_limits<double>::infinity();
  const auto left = MakeImage("camera_left", 100, 0);
  const auto sonar = MakeSonar("sonar0", 100, 0);

  uw::runtime::SynchronizerParams params;
  const auto decision = uw::runtime::SynchronizeAcousticOptic(left, std::nullopt, sonar, rig, params);
  EXPECT_EQ(decision.status, SynchronizationStatus::kInvalidTimestamp);
  EXPECT_FALSE(decision.bundle.has_value());
}

TEST(AcousticOpticSynchronizer, InvalidSonarTimestampIsInvalidNotNoSonar) {
  uw::domain::RigCalibrationSnapshot rig;
  const auto left = MakeImage("camera_left", 100, 0);
  auto sonar = MakeSonar("sonar0", 100, 0);
  sonar.mutable_header()->mutable_sensor_id()->set_value("");

  uw::runtime::SynchronizerParams params;
  const auto decision = uw::runtime::SynchronizeAcousticOptic(left, std::nullopt, sonar, rig, params);
  EXPECT_EQ(decision.status, SynchronizationStatus::kInvalidTimestamp);
  EXPECT_FALSE(decision.bundle.has_value());
}
