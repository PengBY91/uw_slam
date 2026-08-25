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
  for (int i = 0; i < 49; ++i) state.add_covariance_7x7_row_major(0.0);
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
  buffer->AddVehicleState(MakeVehicleState(center - 0.04, 0.0, 1));
  buffer->AddVehicleState(MakeVehicleState(center + 0.04, 0.1, 2));
}

std::optional<uw::runtime::OnlineAcousticOpticBundle> AddNominal(
    AcousticOpticBuffer* buffer, double left, double right, double sonar,
    const std::string& left_id = "kf8", const std::string& right_id = "kf8") {
  AddBracket(buffer, 0.5 * (left + right));
  buffer->AddImage(MakeImage("camera_left", left_id, left, 10));
  buffer->AddImage(MakeImage("camera_right", right_id, right, 11));
  return buffer->AddSonar(MakeSonar("tick1001", sonar, 12));
}

}  // namespace

TEST(AcousticOpticBuffer, PairsNearestTimeNotObservationId) {
  AcousticOpticBuffer buffer(TestBufferConfig(), TestRig());
  buffer.AddVehicleState(MakeVehicleState(9.96, 0.0));
  buffer.AddVehicleState(MakeVehicleState(10.04, 0.1, 2));
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
  auto before = MakeVehicleState(9.96, 0.0, 1);
  auto after = MakeVehicleState(10.04, 0.2, 2);
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
  auto reordered = TestRig("rig_v1");
  reordered.mutable_time_offset_seconds()->clear();
  reordered.mutable_time_offset_provenance()->clear();
  for (const std::string sensor : {"rov-state", "sonar0", "camera_right", "camera_left"}) {
    (*reordered.mutable_time_offset_seconds())[sensor] = 0.0;
    (*reordered.mutable_time_offset_provenance())[sensor] = "measured:test";
  }
  EXPECT_NO_THROW(buffer.UpdateRig(reordered));
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

TEST(AcousticOpticBuffer, NewerEligibleSonarIsNotBlockedByOlderOverWindowSonar) {
  AcousticOpticBuffer buffer(TestBufferConfig(), TestRig());
  AddBracket(&buffer, 10.0);
  buffer.AddImage(MakeImage("camera_left", "kf", 10.0, 1));
  buffer.AddImage(MakeImage("camera_right", "kf", 10.0, 2));
  EXPECT_FALSE(buffer.AddSonar(MakeSonar("old", 9.9, 3)).has_value());
  const auto bundle = buffer.AddSonar(MakeSonar("eligible", 10.0, 4));
  ASSERT_TRUE(bundle.has_value());
  EXPECT_EQ(bundle->sonar.header().observation_id().value(), "eligible");
  EXPECT_EQ(buffer.Diagnostics().buffered_sonar_count, 1u);
}

TEST(AcousticOpticBuffer, BufferedSelectionIsIndependentOfInsertionPermutation) {
  auto run = [](bool reverse_sonars, bool reverse_images) {
    AcousticOpticBuffer buffer(TestBufferConfig(), TestRig());
    AddBracket(&buffer, 10.0);
    const auto old = MakeSonar("old", 9.9, 30);
    const auto best = MakeSonar("best", 10.0, 40);
    if (reverse_sonars) {
      buffer.AddSonar(best);
      buffer.AddSonar(old);
    } else {
      buffer.AddSonar(old);
      buffer.AddSonar(best);
    }
    std::optional<uw::runtime::OnlineAcousticOpticBundle> bundle;
    if (reverse_images) {
      buffer.AddImage(MakeImage("camera_right", "kf", 10.0, 20));
      bundle = buffer.AddImage(MakeImage("camera_left", "kf", 10.0, 10));
    } else {
      buffer.AddImage(MakeImage("camera_left", "kf", 10.0, 10));
      bundle = buffer.AddImage(MakeImage("camera_right", "kf", 10.0, 20));
    }
    EXPECT_TRUE(bundle.has_value());
    return bundle ? bundle->sonar.header().observation_id().value() : std::string{};
  };
  EXPECT_EQ(run(false, false), "best");
  EXPECT_EQ(run(false, true), "best");
  EXPECT_EQ(run(true, false), "best");
  EXPECT_EQ(run(true, true), "best");
}

TEST(AcousticOpticBuffer, RejectsNonCanonicalHeadersAndCalibrationMismatch) {
  AcousticOpticBuffer buffer(TestBufferConfig(), TestRig());
  auto missing_receive = MakeImage("camera_left", "bad-header", 1.0);
  missing_receive.mutable_header()->clear_receive_time();
  EXPECT_FALSE(buffer.AddImage(missing_receive).has_value());
  EXPECT_EQ(buffer.Diagnostics().buffered_image_count, 0u);

  auto missing_sequence = MakeImage("camera_left", "missing-sequence", 1.0, 10);
  missing_sequence.mutable_header()->clear_sequence_id();
  buffer.AddImage(missing_sequence);
  auto missing_capture = MakeImage("camera_left", "missing-capture", 1.0, 11);
  missing_capture.mutable_header()->clear_capture_time();
  buffer.AddImage(missing_capture);
  auto missing_frame = MakeSonar("missing-frame", 1.0, 12);
  missing_frame.mutable_header()->mutable_sensor_frame()->clear_value();
  buffer.AddSonar(missing_frame);
  auto unspecified_clock = MakeImage("camera_left", "unspecified-clock", 1.0, 13);
  unspecified_clock.mutable_header()->set_clock_domain(uw::domain::CLOCK_DOMAIN_UNSPECIFIED);
  buffer.AddImage(unspecified_clock);
  auto rejected_state = MakeVehicleState(1.0, 0.0, 14);
  rejected_state.mutable_header()->set_validity(
      uw::domain::ObservationHeader::VALIDITY_REJECTED);
  buffer.AddVehicleState(rejected_state);

  auto wrong_image = MakeImage("camera_left", "wrong-image", 1.0, 2);
  wrong_image.mutable_header()->mutable_calibration_version()->set_value("other-rig");
  auto wrong_sonar = MakeSonar("wrong-sonar", 1.0, 3);
  wrong_sonar.mutable_header()->mutable_calibration_version()->set_value("other-rig");
  auto wrong_state = MakeVehicleState(1.0, 0.0, 4);
  wrong_state.mutable_header()->mutable_calibration_version()->set_value("other-rig");
  buffer.AddImage(wrong_image);
  buffer.AddSonar(wrong_sonar);
  buffer.AddVehicleState(wrong_state);
  EXPECT_EQ(buffer.Diagnostics().invalid_input_count, 8u);
  EXPECT_EQ(buffer.Diagnostics().invalid_time_count, 1u);
  EXPECT_EQ(buffer.Diagnostics().buffered_image_count, 0u);
  EXPECT_EQ(buffer.Diagnostics().buffered_sonar_count, 0u);
  EXPECT_EQ(buffer.Diagnostics().buffered_vehicle_state_count, 0u);
}

TEST(AcousticOpticBuffer, RejectsStaleInputsAfterCalibrationUpdate) {
  AcousticOpticBuffer buffer(TestBufferConfig(), TestRig("rig_v1"));
  buffer.UpdateRig(TestRig("rig_v2"));
  EXPECT_FALSE(buffer.AddImage(MakeImage("camera_left", "stale", 1.0)).has_value());
  EXPECT_EQ(buffer.Diagnostics().invalid_input_count, 1u);
  EXPECT_EQ(buffer.Diagnostics().buffered_image_count, 0u);
}

TEST(AcousticOpticBuffer, RejectsInvalidCovarianceAndDeviceHealth) {
  AcousticOpticBuffer buffer(TestBufferConfig(), TestRig());
  auto missing_covariance = MakeVehicleState(1.0, 0.0, 1);
  missing_covariance.clear_covariance_7x7_row_major();
  buffer.AddVehicleState(missing_covariance);
  auto nonfinite_covariance = MakeVehicleState(1.0, 0.0, 2);
  nonfinite_covariance.set_covariance_7x7_row_major(
      10, std::numeric_limits<double>::quiet_NaN());
  buffer.AddVehicleState(nonfinite_covariance);
  auto invalid_link = MakeVehicleState(1.0, 0.0, 3);
  invalid_link.set_link_quality(1.01);
  buffer.AddVehicleState(invalid_link);
  EXPECT_EQ(buffer.Diagnostics().invalid_input_count, 3u);
  EXPECT_EQ(buffer.Diagnostics().buffered_vehicle_state_count, 0u);
}

TEST(AcousticOpticBuffer, StateBracketLimitAppliesToFullSpan) {
  AcousticOpticBuffer buffer(TestBufferConfig(), TestRig());
  buffer.AddVehicleState(MakeVehicleState(9.9, 0.0, 1));
  buffer.AddVehicleState(MakeVehicleState(10.1, 0.2, 2));
  buffer.AddImage(MakeImage("camera_left", "kf", 10.0));
  buffer.AddImage(MakeImage("camera_right", "kf", 10.0));
  EXPECT_FALSE(buffer.AddSonar(MakeSonar("sonar", 10.0)).has_value());
}

TEST(AcousticOpticBuffer, DoesNotApplyHiddenEpsilonToConfiguredWindowsOrExpiry) {
  auto stereo_config = TestBufferConfig();
  stereo_config.max_stereo_delta_s = 0.002 - 5e-13;
  AcousticOpticBuffer stereo(stereo_config, TestRig());
  EXPECT_FALSE(AddNominal(&stereo, 10.0, 10.002, 10.001).has_value());

  auto sonar_config = TestBufferConfig();
  sonar_config.max_sonar_camera_delta_s = 0.05 - 5e-13;
  AcousticOpticBuffer sonar(sonar_config, TestRig());
  EXPECT_FALSE(AddNominal(&sonar, 10.0, 10.0, 10.05).has_value());

  auto expiry_config = TestBufferConfig();
  expiry_config.max_residence_s = 0.1 - 5e-13;
  AcousticOpticBuffer expiry(expiry_config, TestRig());
  expiry.AddImage(MakeImage("camera_left", "old", 1.0, 1));
  expiry.AddImage(MakeImage("camera_left", "new", 1.1, 2));
  EXPECT_EQ(expiry.Diagnostics().buffered_image_count, 1u);
  EXPECT_EQ(expiry.Diagnostics().expiry_count, 1u);
}

TEST(AcousticOpticBuffer, StateAndExpiryWindowsAreInclusiveOnlyAtExactBoundary) {
  AcousticOpticBuffer exact_state(TestBufferConfig(), TestRig());
  exact_state.AddVehicleState(MakeVehicleState(0.0, 0.0, 1));
  exact_state.AddVehicleState(MakeVehicleState(0.1, 0.2, 2));
  exact_state.AddImage(MakeImage("camera_left", "kf", 0.05, 3));
  exact_state.AddImage(MakeImage("camera_right", "kf", 0.05, 4));
  EXPECT_TRUE(exact_state.AddSonar(MakeSonar("sonar", 0.05, 5)).has_value());

  auto just_over_config = TestBufferConfig();
  just_over_config.max_state_bracket_s = 0.1 - 5e-13;
  AcousticOpticBuffer just_over_state(just_over_config, TestRig());
  just_over_state.AddVehicleState(MakeVehicleState(0.0, 0.0, 1));
  just_over_state.AddVehicleState(MakeVehicleState(0.1, 0.2, 2));
  just_over_state.AddImage(MakeImage("camera_left", "kf", 0.05, 3));
  just_over_state.AddImage(MakeImage("camera_right", "kf", 0.05, 4));
  EXPECT_FALSE(just_over_state.AddSonar(MakeSonar("sonar", 0.05, 5)).has_value());

  auto expiry_config = TestBufferConfig();
  expiry_config.max_residence_s = 0.1;
  AcousticOpticBuffer exact_expiry(expiry_config, TestRig());
  exact_expiry.AddImage(MakeImage("camera_left", "old", 0.0, 1));
  exact_expiry.AddImage(MakeImage("camera_left", "boundary", 0.1, 2));
  EXPECT_EQ(exact_expiry.Diagnostics().buffered_image_count, 2u);
  EXPECT_EQ(exact_expiry.Diagnostics().expiry_count, 0u);
}

TEST(AcousticOpticBuffer, CalibrationChangeResetsVersionScopedDiagnosticsAndDeltaWindow) {
  AcousticOpticBuffer buffer(TestBufferConfig(), TestRig("rig_v1"));
  ASSERT_TRUE(AddNominal(&buffer, 10.0, 10.0, 10.01).has_value());
  auto before = buffer.Diagnostics();
  ASSERT_EQ(before.accepted_count, 1u);
  ASSERT_GT(before.corrected_delta_max_s, 0.0);
  buffer.UpdateRig(TestRig("rig_v2"));
  const auto after = buffer.Diagnostics();
  EXPECT_EQ(after.accepted_count, 0u);
  EXPECT_EQ(after.synchronization_candidate_count, 0u);
  EXPECT_EQ(after.corrected_delta_p95_s, 0.0);
  EXPECT_EQ(after.corrected_delta_max_s, 0.0);
  EXPECT_EQ(after.calibration_reset_count, 1u);
}

TEST(AcousticOpticBuffer, GlobalScoringConsidersEveryValidStereoEdge) {
  auto config = TestBufferConfig();
  config.max_sonar_camera_delta_s = 0.0005;
  AcousticOpticBuffer buffer(config, TestRig());
  AddBracket(&buffer, 10.0015);
  buffer.AddImage(MakeImage("camera_left", "kf", 10.0000, 10));
  buffer.AddImage(MakeImage("camera_left", "kf", 10.0015, 20));
  buffer.AddImage(MakeImage("camera_right", "kf", 10.00075, 30));
  const auto bundle = buffer.AddSonar(MakeSonar("sonar", 10.0015, 40));
  ASSERT_TRUE(bundle.has_value());
  EXPECT_EQ(bundle->images.primary.header().sequence_id().value(), 20u);
  EXPECT_NEAR(bundle->corrected_time_delta_s, 0.000375, 1e-12);
}

TEST(AcousticOpticBuffer, RejectsMultipleEnabledSonarsAtConstruction) {
  auto rig = TestRig();
  auto* second = rig.add_sonar_beam_models();
  second->mutable_sensor_id()->set_value("sonar1");
  second->set_sonar_enabled(true);
  (*rig.mutable_time_offset_seconds())["sonar1"] = 0.0;
  (*rig.mutable_time_offset_provenance())["sonar1"] = "measured:test";
  EXPECT_THROW(AcousticOpticBuffer(TestBufferConfig(), rig), std::invalid_argument);
}

TEST(AcousticOpticBuffer, IntegrityMismatchDiscardsOnlyCamerasAndRetainsSonar) {
  AcousticOpticBuffer buffer(TestBufferConfig(), TestRig());
  AddBracket(&buffer, 10.0);
  buffer.AddImage(MakeImage("camera_left", "left-bad", 10.0, 1));
  buffer.AddImage(MakeImage("camera_right", "right-bad", 10.0, 2));
  EXPECT_FALSE(buffer.AddSonar(MakeSonar("retained-sonar", 10.0, 3)).has_value());
  EXPECT_EQ(buffer.Diagnostics().buffered_sonar_count, 1u);
  EXPECT_EQ(buffer.Diagnostics().buffered_image_count, 0u);

  buffer.AddImage(MakeImage("camera_left", "valid", 10.001, 4));
  const auto bundle = buffer.AddImage(MakeImage("camera_right", "valid", 10.001, 5));
  ASSERT_TRUE(bundle.has_value());
  EXPECT_EQ(bundle->sonar.header().observation_id().value(), "retained-sonar");
}

TEST(AcousticOpticBuffer, PreservesNanosecondInterpolationNearInt64MaxSeconds) {
  AcousticOpticBuffer buffer(TestBufferConfig(), TestRig());
  const int64_t seconds = std::numeric_limits<int64_t>::max() - 1;
  auto set_exact_time = [&](uw::domain::ObservationHeader* header, int32_t nanos) {
    header->mutable_capture_time()->set_seconds(seconds);
    header->mutable_capture_time()->set_nanos(nanos);
    header->mutable_receive_time()->set_seconds(seconds);
    header->mutable_receive_time()->set_nanos(nanos);
  };
  auto before = MakeVehicleState(0.0, 0.0, 1);
  auto after = MakeVehicleState(0.0, 0.2, 2);
  before.set_depth_m(1.0);
  after.set_depth_m(2.0);
  set_exact_time(before.mutable_header(), 100'000'000);
  set_exact_time(after.mutable_header(), 200'000'000);
  buffer.AddVehicleState(before);
  buffer.AddVehicleState(after);
  auto left = MakeImage("camera_left", "kf", 0.0, 3);
  auto right = MakeImage("camera_right", "kf", 0.0, 4);
  auto sonar = MakeSonar("sonar", 0.0, 5);
  set_exact_time(left.mutable_header(), 150'000'000);
  set_exact_time(right.mutable_header(), 150'000'000);
  set_exact_time(sonar.mutable_header(), 150'000'000);
  buffer.AddImage(left);
  buffer.AddImage(right);
  const auto bundle = buffer.AddSonar(sonar);
  ASSERT_TRUE(bundle.has_value());
  EXPECT_EQ(bundle->interpolated_vehicle_state.header().capture_time().seconds(), seconds);
  EXPECT_EQ(bundle->interpolated_vehicle_state.header().capture_time().nanos(), 150'000'000);
  EXPECT_NEAR(bundle->interpolated_vehicle_state.depth_m(), 1.5, 1e-12);
  EXPECT_TRUE(uw::domain::ValidateObservationHeader(
                  bundle->interpolated_vehicle_state.header()).ok());
}

TEST(AcousticOpticBuffer, RejectsHugeFiniteTimeOffset) {
  auto rig = TestRig();
  (*rig.mutable_time_offset_seconds())["camera_left"] = 10.000001;
  EXPECT_THROW(AcousticOpticBuffer(TestBufferConfig(), rig), std::invalid_argument);
}
