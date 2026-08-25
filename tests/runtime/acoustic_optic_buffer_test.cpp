#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "runtime/acoustic_optic_buffer.hpp"

namespace {

using uw::runtime::AcousticOpticBuffer;
using uw::runtime::AcousticOpticBufferConfig;

uw::domain::Stamp Stamp(double seconds) { return uw::domain::FromSeconds(seconds); }

void SetHeader(uw::domain::ObservationHeader* header, const std::string& sensor,
               const std::string& observation, double seconds, uint64_t sequence = 1) {
  header->mutable_sensor_id()->set_value(sensor);
  header->mutable_observation_id()->set_value(observation);
  header->mutable_sequence_id()->set_value(sequence);
  *header->mutable_capture_time() = Stamp(seconds);
  *header->mutable_receive_time() = Stamp(seconds);
  header->set_clock_domain(uw::domain::CLOCK_DOMAIN_SIMULATION);
  header->set_receive_clock_domain(uw::domain::CLOCK_DOMAIN_SIMULATION);
  header->mutable_sensor_frame()->set_value(sensor + "_link");
  header->mutable_calibration_version()->set_value("rig_v1");
  header->set_validity(uw::domain::ObservationHeader::VALIDITY_OK);
}

uw::domain::ImageFrame MakeImage(const std::string& sensor, const std::string& observation,
                                 double seconds, uint64_t sequence = 1) {
  uw::domain::ImageFrame image;
  SetHeader(image.mutable_header(), sensor, observation, seconds, sequence);
  return image;
}

uw::domain::SonarFrame MakeSonar(const std::string& observation, double seconds,
                                 uint64_t sequence = 1) {
  uw::domain::SonarFrame sonar;
  SetHeader(sonar.mutable_header(), "sonar0", observation, seconds, sequence);
  return sonar;
}

uw::domain::VehicleState MakeVehicleState(double seconds, double yaw_rad,
                                          uint64_t sequence = 1) {
  uw::domain::VehicleState state;
  SetHeader(state.mutable_header(), "rov-state", "state" + std::to_string(sequence),
            seconds, sequence);
  state.add_orientation_xyzw(0.0);
  state.add_orientation_xyzw(0.0);
  state.add_orientation_xyzw(std::sin(yaw_rad * 0.5));
  state.add_orientation_xyzw(std::cos(yaw_rad * 0.5));
  state.add_angular_velocity_radps(seconds);
  state.add_angular_velocity_radps(seconds + 1.0);
  state.add_angular_velocity_radps(seconds + 2.0);
  state.set_depth_m(seconds);
  state.set_attitude_valid(true);
  state.set_depth_valid(true);
  state.set_device_health_valid(true);
  state.set_supply_voltage_v(15.0 + sequence);
  state.set_supply_current_a(2.0);
  state.set_link_quality(0.9);
  return state;
}

uw::domain::RigCalibrationSnapshot TestRig(const std::string& version = "rig_v1",
                                           double left_offset = 0.0,
                                           double right_offset = 0.0,
                                           double sonar_offset = 0.0,
                                           double state_offset = 0.0) {
  uw::domain::RigCalibrationSnapshot rig;
  rig.mutable_calibration_version()->set_value(version);
  for (const std::string sensor : {"camera_left", "camera_right"}) {
    rig.add_cameras()->mutable_sensor_id()->set_value(sensor);
  }
  auto* sonar = rig.add_sonar_beam_models();
  sonar->mutable_sensor_id()->set_value("sonar0");
  sonar->set_sonar_enabled(true);
  rig.add_vehicle_state_sensors()->set_value("rov-state");
  for (const auto& item : {std::pair{"camera_left", left_offset},
                           std::pair{"camera_right", right_offset},
                           std::pair{"sonar0", sonar_offset},
                           std::pair{"rov-state", state_offset}}) {
    (*rig.mutable_time_offset_seconds())[item.first] = item.second;
    (*rig.mutable_time_offset_provenance())[item.first] = "measured:test";
  }
  return rig;
}

AcousticOpticBufferConfig TestBufferConfig() {
  AcousticOpticBufferConfig config;
  config.max_stereo_delta_s = 0.002;
  config.max_sonar_camera_delta_s = 0.050;
  config.max_state_bracket_s = 0.100;
  config.max_residence_s = 0.500;
  return config;
}

void AddBracket(AcousticOpticBuffer* buffer, double center) {
  buffer->AddVehicleState(MakeVehicleState(center - 0.05, 0.0, 1));
  buffer->AddVehicleState(MakeVehicleState(center + 0.05, 0.1, 2));
}

std::optional<uw::runtime::OnlineAcousticOpticBundle> AddNominal(
    AcousticOpticBuffer* buffer, double left, double right, double sonar,
    const std::string& left_id = "kf8", const std::string& right_id = "kf8") {
  AddBracket(buffer, sonar);
  buffer->AddImage(MakeImage("camera_left", left_id, left, 10));
  buffer->AddImage(MakeImage("camera_right", right_id, right, 11));
  return buffer->AddSonar(MakeSonar("tick1001", sonar, 12));
}

}  // namespace

TEST(AcousticOpticBuffer, PairsNearestTimeNotObservationId) {
  AcousticOpticBuffer buffer(TestBufferConfig(), TestRig());
  buffer.AddVehicleState(MakeVehicleState(9.95, 0.0));
  buffer.AddVehicleState(MakeVehicleState(10.05, 0.1, 2));
  buffer.AddImage(MakeImage("camera_left", "wrong-time", 9.90, 1));
  buffer.AddImage(MakeImage("camera_left", "kf8", 10.01, 2));
  buffer.AddImage(MakeImage("camera_right", "kf8", 10.011, 3));
  const auto bundle = buffer.AddSonar(MakeSonar("tick1001", 10.00));
  ASSERT_TRUE(bundle.has_value());
  EXPECT_EQ(bundle->sonar.header().observation_id().value(), "tick1001");
  EXPECT_EQ(bundle->images.primary.header().observation_id().value(), "kf8");
  EXPECT_LT(bundle->corrected_time_delta_s, 0.05);
}

TEST(AcousticOpticBuffer, StereoWindowIsInclusiveAndRejectsJustOver) {
  AcousticOpticBuffer inclusive(TestBufferConfig(), TestRig());
  EXPECT_TRUE(AddNominal(&inclusive, 10.0, 10.002, 10.001).has_value());

  AcousticOpticBuffer over(TestBufferConfig(), TestRig());
  EXPECT_FALSE(AddNominal(&over, 10.0, 10.002001, 10.001).has_value());
  EXPECT_GT(over.Diagnostics().over_window_count, 0u);
}

TEST(AcousticOpticBuffer, SonarCameraWindowIsInclusiveAndRejectsJustOver) {
  AcousticOpticBuffer inclusive(TestBufferConfig(), TestRig());
  EXPECT_TRUE(AddNominal(&inclusive, 10.0, 10.0, 10.05).has_value());

  AcousticOpticBuffer over(TestBufferConfig(), TestRig());
  EXPECT_FALSE(AddNominal(&over, 10.0, 10.0, 10.050001).has_value());
  EXPECT_GT(over.Diagnostics().over_window_count, 0u);
}

TEST(AcousticOpticBuffer, CorrectedOffsetsAffectSelection) {
  AcousticOpticBuffer buffer(TestBufferConfig(), TestRig("rig_v1", 0.04, 0.039, 0.0, 0.0));
  AddBracket(&buffer, 10.05);
  buffer.AddImage(MakeImage("camera_left", "corrected", 10.01, 1));
  buffer.AddImage(MakeImage("camera_right", "corrected", 10.011, 2));
  const auto bundle = buffer.AddSonar(MakeSonar("sonar", 10.05));
  ASSERT_TRUE(bundle.has_value());
  EXPECT_LT(bundle->corrected_time_delta_s, 1e-6);
}

TEST(AcousticOpticBuffer, ObservationIdMismatchRejectsSelectedPairWithoutSearchingById) {
  AcousticOpticBuffer buffer(TestBufferConfig(), TestRig());
  AddBracket(&buffer, 10.0);
  buffer.AddImage(MakeImage("camera_left", "a", 10.0, 1));
  buffer.AddImage(MakeImage("camera_right", "b", 10.0, 2));
  buffer.AddImage(MakeImage("camera_right", "a", 10.001, 3));
  EXPECT_FALSE(buffer.AddSonar(MakeSonar("sonar", 10.0)).has_value());
  EXPECT_EQ(buffer.Diagnostics().integrity_rejection_count, 1u);
}

TEST(AcousticOpticBuffer, InterpolatesStateWithNormalizedShortestPathSlerp) {
  AcousticOpticBuffer buffer(TestBufferConfig(), TestRig());
  auto before = MakeVehicleState(9.95, 0.0, 1);
  auto after = MakeVehicleState(10.05, 0.2, 2);
  for (int i = 0; i < 4; ++i) after.set_orientation_xyzw(i, -after.orientation_xyzw(i));
  buffer.AddVehicleState(before);
  buffer.AddVehicleState(after);
  buffer.AddImage(MakeImage("camera_left", "kf", 10.0));
  buffer.AddImage(MakeImage("camera_right", "kf", 10.0));
  const auto bundle = buffer.AddSonar(MakeSonar("sonar", 10.0));
  ASSERT_TRUE(bundle.has_value());
  const auto& state = bundle->interpolated_vehicle_state;
  double norm2 = 0.0;
  for (double q : state.orientation_xyzw()) norm2 += q * q;
  EXPECT_NEAR(norm2, 1.0, 1e-12);
  EXPECT_NEAR(std::abs(state.orientation_xyzw(2)), std::sin(0.05), 1e-6);
  EXPECT_NEAR(state.depth_m(), 10.0, 1e-9);
  EXPECT_NEAR(state.angular_velocity_radps(0), 10.0, 1e-9);
  EXPECT_TRUE(state.device_health_valid());
  EXPECT_EQ(state.header().calibration_version().value(), "rig_v1");
}

TEST(AcousticOpticBuffer, ExactStateTimestampIsSupported) {
  AcousticOpticBuffer buffer(TestBufferConfig(), TestRig());
  buffer.AddVehicleState(MakeVehicleState(10.0, 0.2));
  buffer.AddImage(MakeImage("camera_left", "kf", 10.0));
  buffer.AddImage(MakeImage("camera_right", "kf", 10.0));
  EXPECT_TRUE(buffer.AddSonar(MakeSonar("sonar", 10.0)).has_value());
}

TEST(AcousticOpticBuffer, RequiresStateBracketWithinWindow) {
  AcousticOpticBuffer buffer(TestBufferConfig(), TestRig());
  buffer.AddVehicleState(MakeVehicleState(9.8, 0.0));
  buffer.AddVehicleState(MakeVehicleState(10.2, 0.1, 2));
  buffer.AddImage(MakeImage("camera_left", "kf", 10.0));
  buffer.AddImage(MakeImage("camera_right", "kf", 10.0));
  EXPECT_FALSE(buffer.AddSonar(MakeSonar("sonar", 10.0)).has_value());
  EXPECT_GT(buffer.Diagnostics().no_pair_count, 0u);
}

TEST(AcousticOpticBuffer, SortedInsertionAndEqualTimestampsUseSequenceTieBreak) {
  AcousticOpticBuffer buffer(TestBufferConfig(), TestRig());
  AddBracket(&buffer, 10.0);
  buffer.AddImage(MakeImage("camera_left", "later-sequence", 10.0, 20));
  buffer.AddImage(MakeImage("camera_left", "earlier-sequence", 10.0, 10));
  buffer.AddImage(MakeImage("camera_right", "earlier-sequence", 10.0, 10));
  const auto bundle = buffer.AddSonar(MakeSonar("sonar", 10.0));
  ASSERT_TRUE(bundle.has_value());
  EXPECT_EQ(bundle->images.primary.header().sequence_id().value(), 10u);
}

TEST(AcousticOpticBuffer, CapacityAndResidenceAreBounded) {
  auto config = TestBufferConfig();
  config.max_images_per_camera = 2;
  config.max_residence_s = 0.1;
  AcousticOpticBuffer buffer(config, TestRig());
  buffer.AddImage(MakeImage("camera_left", "one", 1.0, 1));
  buffer.AddImage(MakeImage("camera_left", "two", 1.01, 2));
  buffer.AddImage(MakeImage("camera_left", "three", 1.02, 3));
  EXPECT_EQ(buffer.Diagnostics().buffered_image_count, 2u);
  EXPECT_EQ(buffer.Diagnostics().capacity_drop_count, 1u);
  buffer.AddImage(MakeImage("camera_right", "new", 2.0, 4));
  EXPECT_EQ(buffer.Diagnostics().buffered_image_count, 1u);
  EXPECT_GT(buffer.Diagnostics().expiry_count, 0u);
}

TEST(AcousticOpticBuffer, InvalidStampsOffsetsAndStateFailClosed) {
  AcousticOpticBuffer buffer(TestBufferConfig(), TestRig());
  auto image = MakeImage("camera_left", "bad", 1.0);
  image.mutable_header()->mutable_capture_time()->set_nanos(-1);
  EXPECT_FALSE(buffer.AddImage(image).has_value());
  auto state = MakeVehicleState(1.0, 0.0);
  state.set_depth_m(std::numeric_limits<double>::infinity());
  EXPECT_FALSE(buffer.AddVehicleState(state).has_value());
  EXPECT_EQ(buffer.Diagnostics().invalid_time_count, 1u);
  EXPECT_GT(buffer.Diagnostics().invalid_input_count, 0u);
}

TEST(AcousticOpticBuffer, CalibrationChangeClearsPendingFrames) {
  AcousticOpticBuffer buffer(TestBufferConfig(), TestRig("rig_v1"));
  buffer.AddImage(MakeImage("camera_left", "a", 1.0));
  buffer.UpdateRig(TestRig("rig_v2"));
  EXPECT_EQ(buffer.Diagnostics().calibration_reset_count, 1u);
  EXPECT_EQ(buffer.Diagnostics().buffered_image_count, 0u);
}

TEST(AcousticOpticBuffer, SameVersionMutationIsRejectedButIdenticalUpdateIsNoOp) {
  AcousticOpticBuffer buffer(TestBufferConfig(), TestRig("rig_v1"));
  EXPECT_NO_THROW(buffer.UpdateRig(TestRig("rig_v1")));
  EXPECT_THROW(buffer.UpdateRig(TestRig("rig_v1", 0.001)), std::invalid_argument);
  EXPECT_EQ(buffer.Diagnostics().calibration_reset_count, 0u);
}

TEST(AcousticOpticBuffer, RejectsInvalidConfigAndIncompleteRigCalibration) {
  auto config = TestBufferConfig();
  config.max_stereo_delta_s = std::numeric_limits<double>::infinity();
  EXPECT_THROW(AcousticOpticBuffer(config, TestRig()), std::invalid_argument);

  auto rig = TestRig();
  rig.mutable_time_offset_provenance()->erase("camera_left");
  EXPECT_THROW(AcousticOpticBuffer(TestBufferConfig(), rig), std::invalid_argument);
  rig = TestRig();
  rig.clear_vehicle_state_sensors();
  EXPECT_THROW(AcousticOpticBuffer(TestBufferConfig(), rig), std::invalid_argument);
}

TEST(AcousticOpticBuffer, UndeclaredVehicleStateSensorFailsClosed) {
  AcousticOpticBuffer buffer(TestBufferConfig(), TestRig());
  auto state = MakeVehicleState(1.0, 0.0);
  state.mutable_header()->mutable_sensor_id()->set_value("other-state");
  EXPECT_FALSE(buffer.AddVehicleState(state).has_value());
  EXPECT_EQ(buffer.Diagnostics().invalid_input_count, 1u);
}
