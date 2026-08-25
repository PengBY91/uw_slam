#include "runtime/canonical_event_validation.hpp"

#include <limits>
#include <string>

#include <gtest/gtest.h>

#include "domain/domain.hpp"
#include "runtime/canonical_topics.hpp"

namespace {

void PopulateValidHeader(uw::domain::ObservationHeader* header) {
  header->mutable_observation_id()->set_value("observation-1");
  header->mutable_sensor_id()->set_value("sensor-1");
  header->mutable_sensor_frame()->set_value("sensor-frame");
  header->mutable_calibration_version()->set_value("calibration-v1");
  header->mutable_capture_time()->set_seconds(10);
  header->mutable_capture_time()->set_nanos(100);
  header->mutable_receive_time()->set_seconds(10);
  header->mutable_receive_time()->set_nanos(200);
  header->set_clock_domain(uw::domain::CLOCK_DOMAIN_SENSOR_HARDWARE);
  header->set_receive_clock_domain(uw::domain::CLOCK_DOMAIN_SYSTEM_MONOTONIC);
  header->set_validity(uw::domain::ObservationHeader::VALIDITY_OK);
}

uw::domain::ImageFrame MakeValidImageFrame() {
  uw::domain::ImageFrame frame;
  PopulateValidHeader(frame.mutable_header());
  frame.set_width(2);
  frame.set_height(2);
  frame.set_row_stride_bytes(2);
  frame.set_encoding(uw::domain::ImageFrame::IMAGE_ENCODING_MONO8);
  frame.set_pixel_data(std::string(4, '\0'));
  return frame;
}

uw::domain::SonarFrame MakeValidSonarFrame() {
  uw::domain::SonarFrame frame;
  PopulateValidHeader(frame.mutable_header());
  frame.set_num_ranges(2);
  frame.set_num_beams(3);
  frame.set_encoding(uw::domain::SonarFrame::ENCODING_UINT8_GRAY);
  frame.set_intensity_tensor(std::string(6, '\0'));
  for (float range : {0.5f, 1.0f, 1.5f}) frame.add_range_bins(range);
  for (float azimuth : {-0.5f, 0.0f, 0.5f}) frame.add_azimuth_angles(azimuth);
  frame.set_min_range(0.5f);
  frame.set_max_range(1.5f);
  frame.set_range_resolution(0.5f);
  frame.set_horizontal_fov(1.0f);
  frame.set_elevation_aperture(0.2f);
  frame.mutable_gain_metadata()->set_gain(2.0f);
  frame.mutable_sound_speed_assumption()->set_speed_of_sound_mps(1500.0f);
  frame.set_operating_frequency_hz(750000.0);
  return frame;
}

uw::domain::VehicleState MakeValidVehicleState() {
  uw::domain::VehicleState state;
  PopulateValidHeader(state.mutable_header());
  for (double value : {0.0, 0.0, 0.0, 1.0}) state.add_orientation_xyzw(value);
  for (double value : {0.1, 0.2, 0.3}) state.add_angular_velocity_radps(value);
  state.set_attitude_valid(true);
  state.set_depth_valid(true);
  state.set_depth_m(0.0);
  state.set_device_health_valid(true);
  state.set_supply_voltage_v(15.0);
  state.set_supply_current_a(2.0);
  state.set_link_quality(0.75);
  return state;
}

TEST(CanonicalEventValidation, RejectsUnknownTopicBeforePayloadValidation) {
  const uw::runtime::CanonicalEvent event{"/unknown", 10, 1, MakeValidImageFrame()};
  EXPECT_EQ(uw::runtime::ValidateCanonicalEvent(event).code,
            uw::runtime::CanonicalEventValidationCode::kUnknownTopic);
}

TEST(CanonicalEventValidation, RejectsTopicPayloadMismatchBeforeSemanticValidation) {
  const uw::runtime::CanonicalEvent event{uw::runtime::kTopicCameraLeft, 10, 1,
                                          MakeValidSonarFrame()};
  EXPECT_EQ(uw::runtime::ValidateCanonicalEvent(event).code,
            uw::runtime::CanonicalEventValidationCode::kTopicPayloadMismatch);
}

TEST(CanonicalEventValidation, RejectsEmptyCalibrationVersion) {
  auto frame = MakeValidImageFrame();
  frame.mutable_header()->mutable_calibration_version()->clear_value();
  const uw::runtime::CanonicalEvent event{uw::runtime::kTopicCameraLeft, 10, 1, frame};
  EXPECT_EQ(uw::runtime::ValidateCanonicalEvent(event).code,
            uw::runtime::CanonicalEventValidationCode::kHeaderInvalid);
}

TEST(CanonicalEventValidation, RejectsOtherMalformedRawObservationHeaders) {
  auto expect_invalid = [](const uw::domain::ImageFrame& frame) {
    const uw::runtime::CanonicalEvent event{uw::runtime::kTopicCameraLeft, 10, 1, frame};
    EXPECT_EQ(uw::runtime::ValidateCanonicalEvent(event).code,
              uw::runtime::CanonicalEventValidationCode::kHeaderInvalid);
  };

  auto frame = MakeValidImageFrame();
  frame.mutable_header()->mutable_observation_id()->clear_value();
  expect_invalid(frame);
  frame = MakeValidImageFrame();
  frame.mutable_header()->mutable_sensor_id()->clear_value();
  expect_invalid(frame);
  frame = MakeValidImageFrame();
  frame.mutable_header()->mutable_sensor_frame()->clear_value();
  expect_invalid(frame);
  frame = MakeValidImageFrame();
  frame.mutable_header()->set_clock_domain(uw::domain::CLOCK_DOMAIN_UNSPECIFIED);
  expect_invalid(frame);
  frame = MakeValidImageFrame();
  frame.mutable_header()->set_receive_clock_domain(uw::domain::CLOCK_DOMAIN_UNSPECIFIED);
  expect_invalid(frame);
  frame = MakeValidImageFrame();
  frame.mutable_header()->mutable_capture_time()->set_nanos(1000000000);
  expect_invalid(frame);
  frame = MakeValidImageFrame();
  frame.mutable_header()->mutable_receive_time()->set_nanos(-1);
  expect_invalid(frame);
  frame = MakeValidImageFrame();
  frame.mutable_header()->set_validity(uw::domain::ObservationHeader::VALIDITY_REJECTED);
  expect_invalid(frame);
}

TEST(CanonicalEventValidation, AcceptsDegradedRawObservationHeader) {
  auto frame = MakeValidImageFrame();
  frame.mutable_header()->set_validity(uw::domain::ObservationHeader::VALIDITY_DEGRADED);
  const uw::runtime::CanonicalEvent event{uw::runtime::kTopicCameraLeft, 10, 1, frame};
  EXPECT_TRUE(uw::runtime::ValidateCanonicalEvent(event).ok());
}

TEST(CanonicalEventValidation, RejectsImagePayloadMismatch) {
  auto frame = MakeValidImageFrame();
  frame.set_pixel_data(std::string(3, '\0'));
  const uw::runtime::CanonicalEvent event{uw::runtime::kTopicCameraLeft, 10, 1, frame};
  EXPECT_EQ(uw::runtime::ValidateCanonicalEvent(event).code,
            uw::runtime::CanonicalEventValidationCode::kImageInvalid);
}

TEST(CanonicalEventValidation, RejectsNonAscendingSonarAzimuths) {
  auto frame = MakeValidSonarFrame();
  frame.set_azimuth_angles(1, -0.5f);
  const uw::runtime::CanonicalEvent event{uw::runtime::kTopicSonarFrame, 10, 1, frame};
  EXPECT_EQ(uw::runtime::ValidateCanonicalEvent(event).code,
            uw::runtime::CanonicalEventValidationCode::kSonarGeometryInvalid);
}

TEST(CanonicalEventValidation, RejectsSonarTensorSizeMismatch) {
  auto frame = MakeValidSonarFrame();
  frame.set_intensity_tensor(std::string(3, '\0'));
  const uw::runtime::CanonicalEvent event{uw::runtime::kTopicSonarFrame, 10, 1, frame};
  const auto result = uw::runtime::ValidateCanonicalEvent(event);
  EXPECT_EQ(result.code,
            uw::runtime::CanonicalEventValidationCode::kSonarPayloadSizeMismatch);
}

TEST(CanonicalEventValidation, RejectsMissingOrNonFiniteSonarFrequency) {
  auto frame = MakeValidSonarFrame();
  frame.set_operating_frequency_hz(0.0);
  uw::runtime::CanonicalEvent event{uw::runtime::kTopicSonarFrame, 10, 1, frame};
  EXPECT_EQ(uw::runtime::ValidateCanonicalEvent(event).code,
            uw::runtime::CanonicalEventValidationCode::kSonarGeometryInvalid);

  frame.set_operating_frequency_hz(std::numeric_limits<double>::infinity());
  event.payload = frame;
  EXPECT_EQ(uw::runtime::ValidateCanonicalEvent(event).code,
            uw::runtime::CanonicalEventValidationCode::kSonarGeometryInvalid);
}

TEST(CanonicalEventValidation, RejectsInvalidSonarRangeFovGainOrSoundSpeed) {
  auto expect_invalid = [](const uw::domain::SonarFrame& frame) {
    const uw::runtime::CanonicalEvent event{uw::runtime::kTopicSonarFrame, 10, 1, frame};
    EXPECT_EQ(uw::runtime::ValidateCanonicalEvent(event).code,
              uw::runtime::CanonicalEventValidationCode::kSonarGeometryInvalid);
  };

  auto frame = MakeValidSonarFrame();
  frame.set_max_range(frame.min_range());
  expect_invalid(frame);
  frame = MakeValidSonarFrame();
  frame.set_range_resolution(std::numeric_limits<float>::infinity());
  expect_invalid(frame);
  frame = MakeValidSonarFrame();
  frame.set_horizontal_fov(0.0f);
  expect_invalid(frame);
  frame = MakeValidSonarFrame();
  frame.set_range_bins(1, frame.range_bins(0));
  expect_invalid(frame);
  frame = MakeValidSonarFrame();
  frame.mutable_gain_metadata()->set_gain(0.0f);
  expect_invalid(frame);
  frame = MakeValidSonarFrame();
  frame.mutable_sound_speed_assumption()->set_speed_of_sound_mps(0.0f);
  expect_invalid(frame);
}

TEST(CanonicalEventValidation, RejectsVehicleVectorSizeMismatch) {
  auto state = MakeValidVehicleState();
  state.mutable_angular_velocity_radps()->RemoveLast();
  const uw::runtime::CanonicalEvent event{uw::runtime::kTopicVehicleState, 10, 1, state};
  EXPECT_EQ(uw::runtime::ValidateCanonicalEvent(event).code,
            uw::runtime::CanonicalEventValidationCode::kVehicleVectorSizeInvalid);
}

TEST(CanonicalEventValidation, RejectsNonUnitVehicleQuaternion) {
  auto state = MakeValidVehicleState();
  state.set_orientation_xyzw(3, 2.0);
  const uw::runtime::CanonicalEvent event{uw::runtime::kTopicVehicleState, 10, 1, state};
  EXPECT_EQ(uw::runtime::ValidateCanonicalEvent(event).code,
            uw::runtime::CanonicalEventValidationCode::kVehicleQuaternionInvalid);
}

TEST(CanonicalEventValidation, RejectsMissingVehicleValidityFlags) {
  auto expect_invalid = [](const uw::domain::VehicleState& state) {
    const uw::runtime::CanonicalEvent event{uw::runtime::kTopicVehicleState, 10, 1, state};
    EXPECT_EQ(uw::runtime::ValidateCanonicalEvent(event).code,
              uw::runtime::CanonicalEventValidationCode::kVehicleValueInvalid);
  };

  auto state = MakeValidVehicleState();
  state.set_attitude_valid(false);
  expect_invalid(state);
  state = MakeValidVehicleState();
  state.set_depth_valid(false);
  expect_invalid(state);
  state = MakeValidVehicleState();
  state.set_device_health_valid(false);
  expect_invalid(state);
}

TEST(CanonicalEventValidation, RejectsInvalidVehicleValues) {
  auto expect_invalid = [](const uw::domain::VehicleState& state) {
    const uw::runtime::CanonicalEvent event{uw::runtime::kTopicVehicleState, 10, 1, state};
    EXPECT_EQ(uw::runtime::ValidateCanonicalEvent(event).code,
              uw::runtime::CanonicalEventValidationCode::kVehicleValueInvalid);
  };

  auto state = MakeValidVehicleState();
  state.set_angular_velocity_radps(0, std::numeric_limits<double>::quiet_NaN());
  expect_invalid(state);
  state = MakeValidVehicleState();
  state.set_depth_m(-0.1);
  expect_invalid(state);
  state = MakeValidVehicleState();
  state.set_supply_voltage_v(0.0);
  expect_invalid(state);
  state = MakeValidVehicleState();
  state.set_supply_current_a(std::numeric_limits<double>::infinity());
  expect_invalid(state);
}

TEST(CanonicalEventValidation, RejectsInvalidLinkQuality) {
  auto state = MakeValidVehicleState();
  state.set_link_quality(-0.01);
  uw::runtime::CanonicalEvent event{uw::runtime::kTopicVehicleState, 10, 1, state};
  EXPECT_EQ(uw::runtime::ValidateCanonicalEvent(event).code,
            uw::runtime::CanonicalEventValidationCode::kVehicleValueInvalid);

  state.set_link_quality(1.01);
  event.payload = state;
  EXPECT_EQ(uw::runtime::ValidateCanonicalEvent(event).code,
            uw::runtime::CanonicalEventValidationCode::kVehicleValueInvalid);
}

TEST(CanonicalEventValidation, AcceptsFullyValidRawEvents) {
  const uw::runtime::CanonicalEvent image{uw::runtime::kTopicCameraLeft, 10, 1,
                                          MakeValidImageFrame()};
  const uw::runtime::CanonicalEvent sonar{uw::runtime::kTopicSonarFrame, 11, 2,
                                          MakeValidSonarFrame()};
  const uw::runtime::CanonicalEvent vehicle{uw::runtime::kTopicVehicleState, 12, 3,
                                            MakeValidVehicleState()};
  EXPECT_TRUE(uw::runtime::ValidateCanonicalEvent(image).ok());
  EXPECT_TRUE(uw::runtime::ValidateCanonicalEvent(sonar).ok());
  EXPECT_TRUE(uw::runtime::ValidateCanonicalEvent(vehicle).ok());
}

TEST(CanonicalEventValidation, AcceptsRegistryMatchedNonRawEventWithoutInventingRules) {
  uw::domain::HealthReport report;
  const uw::runtime::CanonicalEvent event{uw::runtime::kTopicHealth, 10, 1, report};
  EXPECT_TRUE(uw::runtime::ValidateCanonicalEvent(event).ok());
}

}  // namespace
