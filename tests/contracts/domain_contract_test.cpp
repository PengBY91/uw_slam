// L0: input schema/unit/frame/time contract checks (platform architecture
// section 15's L0 row). Cross-cutting across uw_domain's message types,
// separate from any one module's unit tests.
#include <gtest/gtest.h>

#include "domain/domain.hpp"

TEST(DomainContract, ObservationHeaderRoundTrips) {
  uw::domain::ObservationHeader header;
  header.mutable_observation_id()->set_value("obs42");
  header.mutable_sensor_id()->set_value("sonar0");
  header.set_clock_domain(uw::domain::CLOCK_DOMAIN_SIMULATION);
  header.mutable_sensor_frame()->set_value("sonar_link");
  header.set_validity(uw::domain::ObservationHeader::VALIDITY_OK);

  std::string bytes;
  ASSERT_TRUE(header.SerializeToString(&bytes));

  uw::domain::ObservationHeader parsed;
  ASSERT_TRUE(parsed.ParseFromString(bytes));
  EXPECT_EQ(parsed.observation_id().value(), "obs42");
  EXPECT_EQ(parsed.sensor_id().value(), "sonar0");
  EXPECT_EQ(parsed.clock_domain(), uw::domain::CLOCK_DOMAIN_SIMULATION);
  EXPECT_EQ(parsed.validity(), uw::domain::ObservationHeader::VALIDITY_OK);
}

TEST(DomainContract, VehicleStateRoundTripsWithCanonicalHeaderAndDeviceHealth) {
  uw::domain::VehicleState state;
  state.mutable_header()->mutable_observation_id()->set_value("vehicle_0001");
  state.mutable_header()->mutable_sensor_id()->set_value("bluerov_state");
  state.mutable_header()->set_clock_domain(uw::domain::CLOCK_DOMAIN_SYSTEM_REALTIME);
  state.mutable_header()->set_receive_clock_domain(uw::domain::CLOCK_DOMAIN_SYSTEM_MONOTONIC);
  state.mutable_header()->mutable_sensor_frame()->set_value("base_link");
  state.mutable_header()->mutable_calibration_version()->set_value("rig_v1");
  state.mutable_header()->set_validity(uw::domain::ObservationHeader::VALIDITY_OK);
  for (double value : {0.0, 0.0, 0.0, 1.0}) state.add_orientation_xyzw(value);
  for (double value : {0.1, 0.2, 0.3}) state.add_angular_velocity_radps(value);
  for (int i = 0; i < 49; ++i) state.add_covariance_7x7_row_major(0.01 * (i + 1));
  state.set_depth_m(2.5);
  state.set_attitude_valid(true);
  state.set_depth_valid(true);
  state.set_leak_detected(false);
  state.set_supply_voltage_v(15.8);
  state.set_supply_current_a(8.2);
  state.set_link_quality(0.95);
  state.set_device_health_valid(true);

  std::string bytes;
  ASSERT_TRUE(state.SerializeToString(&bytes));

  uw::domain::VehicleState parsed;
  ASSERT_TRUE(parsed.ParseFromString(bytes));
  EXPECT_EQ(parsed.header().observation_id().value(), "vehicle_0001");
  EXPECT_EQ(parsed.header().clock_domain(), uw::domain::CLOCK_DOMAIN_SYSTEM_REALTIME);
  EXPECT_EQ(parsed.header().receive_clock_domain(), uw::domain::CLOCK_DOMAIN_SYSTEM_MONOTONIC);
  EXPECT_EQ(parsed.header().calibration_version().value(), "rig_v1");
  ASSERT_EQ(parsed.orientation_xyzw_size(), 4);
  EXPECT_DOUBLE_EQ(parsed.orientation_xyzw(3), 1.0);
  ASSERT_EQ(parsed.angular_velocity_radps_size(), 3);
  EXPECT_DOUBLE_EQ(parsed.angular_velocity_radps(2), 0.3);
  EXPECT_DOUBLE_EQ(parsed.depth_m(), 2.5);
  ASSERT_EQ(parsed.covariance_7x7_row_major_size(), 49);
  EXPECT_DOUBLE_EQ(parsed.covariance_7x7_row_major(48), 0.49);
  EXPECT_TRUE(parsed.attitude_valid());
  EXPECT_TRUE(parsed.depth_valid());
  EXPECT_FALSE(parsed.leak_detected());
  EXPECT_DOUBLE_EQ(parsed.supply_voltage_v(), 15.8);
  EXPECT_DOUBLE_EQ(parsed.supply_current_a(), 8.2);
  EXPECT_DOUBLE_EQ(parsed.link_quality(), 0.95);
  EXPECT_TRUE(parsed.device_health_valid());
}

TEST(DomainContract, SonarFrameAscendingAzimuthAccepted) {
  uw::domain::SonarFrame frame;
  frame.add_azimuth_angles(-0.5f);
  frame.add_azimuth_angles(0.0f);
  frame.add_azimuth_angles(0.5f);
  frame.set_operating_frequency_hz(750000.0);
  EXPECT_TRUE(uw::domain::IsAzimuthAscending(frame));

  std::string bytes;
  ASSERT_TRUE(frame.SerializeToString(&bytes));
  uw::domain::SonarFrame parsed;
  ASSERT_TRUE(parsed.ParseFromString(bytes));
  EXPECT_DOUBLE_EQ(parsed.operating_frequency_hz(), 750000.0);
}

TEST(DomainContract, SonarFrameNonAscendingAzimuthRejected) {
  uw::domain::SonarFrame frame;
  frame.add_azimuth_angles(0.5f);
  frame.add_azimuth_angles(-0.5f);
  frame.add_azimuth_angles(0.0f);
  EXPECT_FALSE(uw::domain::IsAzimuthAscending(frame));
}

TEST(DomainContract, MeasurementEvidencePayloadRoundTripsThroughOneof) {
  uw::domain::SonarRangeBearing measurement;
  measurement.set_range_m(4.2);
  measurement.set_bearing_rad(0.3);

  uw::domain::EvidenceId id;
  id.set_value("ev1");
  auto evidence = uw::domain::MakeEvidence<uw::domain::SonarRangeBearing>(id, {}, measurement, 1.0,
                                                                          "test_v1");

  std::string bytes;
  ASSERT_TRUE(evidence.SerializeToString(&bytes));
  uw::domain::MeasurementEvidence parsed;
  ASSERT_TRUE(parsed.ParseFromString(bytes));

  ASSERT_TRUE(uw::domain::HasPayload<uw::domain::SonarRangeBearing>(parsed));
  EXPECT_NEAR(uw::domain::GetPayload<uw::domain::SonarRangeBearing>(parsed).range_m(), 4.2, 1e-9);
  EXPECT_FALSE(uw::domain::HasPayload<uw::domain::PressureDepthMeasurement>(parsed));
}

TEST(DomainContract, ImageFrameRoundTripsWithCanonicalHeader) {
  uw::domain::ImageFrame frame;
  frame.mutable_header()->mutable_observation_id()->set_value("left_0001");
  frame.mutable_header()->mutable_sensor_id()->set_value("camera_left");
  frame.mutable_header()->mutable_sensor_frame()->set_value("camera_left_link");
  frame.set_width(2);
  frame.set_height(1);
  frame.set_row_stride_bytes(2);
  frame.set_encoding(uw::domain::ImageFrame::IMAGE_ENCODING_MONO8);
  frame.set_pixel_data(std::string{"\x10\x20", 2});
  frame.set_is_rectified(true);
  frame.set_exposure_seconds(0.004);

  std::string bytes;
  ASSERT_TRUE(frame.SerializeToString(&bytes));
  uw::domain::ImageFrame parsed;
  ASSERT_TRUE(parsed.ParseFromString(bytes));
  EXPECT_EQ(parsed.header().observation_id().value(), "left_0001");
  EXPECT_EQ(parsed.pixel_data(), std::string("\x10\x20", 2));
  EXPECT_TRUE(parsed.is_rectified());
}

TEST(DomainContract, ImuSampleRoundTripsWithAndWithoutBias) {
  uw::domain::ImuSample with_bias;
  with_bias.mutable_header()->mutable_observation_id()->set_value("imu_0001");
  with_bias.mutable_header()->mutable_sensor_id()->set_value("imu0");
  with_bias.mutable_header()->mutable_sensor_frame()->set_value("imu_link");
  for (double v : {0.1, 0.2, 9.81}) with_bias.add_linear_acceleration_mps2(v);
  for (double v : {0.01, -0.02, 0.03}) with_bias.add_angular_velocity_radps(v);
  with_bias.set_has_bias(true);
  for (double v : {0.001, 0.002, 0.003}) with_bias.add_bias_linear_acceleration_mps2(v);
  for (double v : {0.0001, 0.0002, 0.0003}) with_bias.add_bias_angular_velocity_radps(v);

  std::string bytes;
  ASSERT_TRUE(with_bias.SerializeToString(&bytes));
  uw::domain::ImuSample parsed;
  ASSERT_TRUE(parsed.ParseFromString(bytes));
  EXPECT_EQ(parsed.header().observation_id().value(), "imu_0001");
  ASSERT_EQ(parsed.linear_acceleration_mps2_size(), 3);
  EXPECT_NEAR(parsed.linear_acceleration_mps2(2), 9.81, 1e-9);
  ASSERT_EQ(parsed.angular_velocity_radps_size(), 3);
  EXPECT_TRUE(parsed.has_bias());
  ASSERT_EQ(parsed.bias_linear_acceleration_mps2_size(), 3);
  EXPECT_NEAR(parsed.bias_angular_velocity_radps(1), 0.0002, 1e-9);

  // Without bias: has_bias defaults false, bias fields stay empty — the
  // wire contract distinguishes "no bias reported" from "bias is zero".
  uw::domain::ImuSample no_bias;
  no_bias.add_linear_acceleration_mps2(1.0);
  EXPECT_FALSE(no_bias.has_bias());
  EXPECT_EQ(no_bias.bias_linear_acceleration_mps2_size(), 0);
}

TEST(DomainContract, DvlSampleRoundTripsWithAndWithoutBeamRanges) {
  uw::domain::DvlSample with_ranges;
  with_ranges.mutable_header()->mutable_observation_id()->set_value("dvl_0001");
  with_ranges.mutable_header()->mutable_sensor_id()->set_value("dvl0");
  for (double v : {0.5, -0.1, 0.02}) with_ranges.add_velocity_mps(v);
  with_ranges.set_has_beam_ranges(true);
  for (double v : {2.0, 2.1, 2.2, 2.3}) with_ranges.add_beam_ranges_m(v);

  std::string bytes;
  ASSERT_TRUE(with_ranges.SerializeToString(&bytes));
  uw::domain::DvlSample parsed;
  ASSERT_TRUE(parsed.ParseFromString(bytes));
  EXPECT_EQ(parsed.header().observation_id().value(), "dvl_0001");
  ASSERT_EQ(parsed.velocity_mps_size(), 3);
  EXPECT_NEAR(parsed.velocity_mps(0), 0.5, 1e-9);
  EXPECT_TRUE(parsed.has_beam_ranges());
  ASSERT_EQ(parsed.beam_ranges_m_size(), 4);
  EXPECT_NEAR(parsed.beam_ranges_m(3), 2.3, 1e-9);

  uw::domain::DvlSample no_ranges;
  no_ranges.add_velocity_mps(1.0);
  EXPECT_FALSE(no_ranges.has_beam_ranges());
  EXPECT_EQ(no_ranges.beam_ranges_m_size(), 0);
}

TEST(DomainContract, OpticalAndFusedDepthPayloadsRoundTripThroughEvidence) {
  uw::domain::OpticalDepthPriorMeasurement prior;
  prior.mutable_reference_camera_frame()->set_value("camera_left_link");
  prior.set_width(2);
  prior.set_height(1);
  prior.add_depth_m(2.0f);
  prior.add_depth_m(3.0f);
  prior.add_variance_m2(0.04f);
  prior.add_variance_m2(0.09f);
  prior.set_valid_mask(std::string{"\x01\x01", 2});
  prior.set_scale_status(uw::domain::OPTICAL_DEPTH_SCALE_STATUS_METRIC);
  prior.set_producer_type("stereo");

  uw::domain::EvidenceId prior_id;
  prior_id.set_value("optical_depth_1");
  uw::domain::ObservationId left_id;
  left_id.set_value("left_0001");
  uw::domain::ObservationId right_id;
  right_id.set_value("right_0001");
  auto prior_evidence = uw::domain::MakeEvidence(
      prior_id, {left_id, right_id}, prior, 1.0, "stereo_depth_frontend_v1");
  ASSERT_TRUE(uw::domain::HasPayload<uw::domain::OpticalDepthPriorMeasurement>(prior_evidence));
  EXPECT_EQ(prior_evidence.source_observations_size(), 2);

  uw::domain::FusedDepthMeasurement fused;
  fused.mutable_reference_camera_frame()->set_value("camera_left_link");
  fused.set_width(1);
  fused.set_height(1);
  fused.add_depth_m(2.1f);
  fused.add_variance_m2(0.01f);
  fused.set_valid_mask(std::string{"\x01", 1});
  fused.set_contribution_mask(std::string{"\x02", 1});
  auto* association = fused.add_associations();
  association->mutable_sonar_evidence_id()->set_value("sonar_cfar_1");
  association->set_status(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_ACCEPTED);
  association->set_has_selected_pixel(true);
  association->set_selected_pixel_index(0);
  association->set_reason(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_NONE);

  uw::domain::EvidenceId fused_id;
  fused_id.set_value("fused_depth_1");
  uw::domain::ObservationId sonar_id;
  sonar_id.set_value("sonar_0001");
  auto fused_evidence = uw::domain::MakeEvidence(
      fused_id, {left_id, right_id, sonar_id}, fused, 0.5,
      "acoustic_optic_depth_fusion_v1");
  ASSERT_TRUE(uw::domain::HasPayload<uw::domain::FusedDepthMeasurement>(fused_evidence));
  EXPECT_EQ(fused_evidence.source_observations_size(), 3);
  EXPECT_EQ(uw::domain::GetPayload<uw::domain::FusedDepthMeasurement>(fused_evidence)
                .associations(0)
                .selected_pixel_index(),
            0u);
}

TEST(DomainValidation, AcceptsWellFormedImageAndDepthGrids) {
  uw::domain::ImageFrame image;
  image.set_width(2);
  image.set_height(1);
  image.set_row_stride_bytes(2);
  image.set_encoding(uw::domain::ImageFrame::IMAGE_ENCODING_MONO8);
  image.set_pixel_data(std::string{"\x01\x02", 2});
  EXPECT_TRUE(uw::domain::ValidateImageFrame(image).ok());

  uw::domain::OpticalDepthPriorMeasurement prior;
  prior.set_width(2);
  prior.set_height(1);
  prior.add_depth_m(1.0f);
  prior.add_depth_m(2.0f);
  prior.add_variance_m2(0.01f);
  prior.add_variance_m2(0.04f);
  prior.set_valid_mask(std::string{"\x01\x01", 2});
  prior.set_scale_status(uw::domain::OPTICAL_DEPTH_SCALE_STATUS_METRIC);
  EXPECT_TRUE(uw::domain::ValidateOpticalDepthPrior(prior).ok());
}

TEST(DomainValidation, ConvertToMono8ConvertsColorAndPassesThroughMono8) {
  uw::domain::ImageFrame mono;
  mono.set_width(2);
  mono.set_height(1);
  mono.set_row_stride_bytes(2);
  mono.set_encoding(uw::domain::ImageFrame::IMAGE_ENCODING_MONO8);
  mono.set_pixel_data(std::string{"\x01\x02", 2});
  const auto mono_passthrough = uw::domain::ConvertToMono8(mono);
  ASSERT_TRUE(mono_passthrough.has_value());
  EXPECT_EQ(mono_passthrough->pixel_data(), mono.pixel_data());

  uw::domain::ImageFrame rgb;
  rgb.set_width(1);
  rgb.set_height(1);
  rgb.set_row_stride_bytes(3);
  rgb.set_encoding(uw::domain::ImageFrame::IMAGE_ENCODING_RGB8);
  rgb.set_pixel_data(std::string{"\xff\x00\x00", 3});  // pure red
  const auto rgb_gray = uw::domain::ConvertToMono8(rgb);
  ASSERT_TRUE(rgb_gray.has_value());
  EXPECT_EQ(rgb_gray->encoding(), uw::domain::ImageFrame::IMAGE_ENCODING_MONO8);
  EXPECT_EQ(rgb_gray->row_stride_bytes(), 1u);
  ASSERT_EQ(rgb_gray->pixel_data().size(), 1u);
  EXPECT_EQ(static_cast<unsigned char>(rgb_gray->pixel_data()[0]), 76);  // round(0.299*255)

  uw::domain::ImageFrame bgr;
  bgr.set_width(1);
  bgr.set_height(1);
  bgr.set_row_stride_bytes(3);
  bgr.set_encoding(uw::domain::ImageFrame::IMAGE_ENCODING_BGR8);
  bgr.set_pixel_data(std::string{"\x00\x00\xff", 3});  // same pure red, BGR channel order
  const auto bgr_gray = uw::domain::ConvertToMono8(bgr);
  ASSERT_TRUE(bgr_gray.has_value());
  EXPECT_EQ(static_cast<unsigned char>(bgr_gray->pixel_data()[0]), 76);
}

TEST(DomainValidation, ConvertToMono8RejectsInvalidInput) {
  uw::domain::ImageFrame malformed;
  malformed.set_width(2);
  malformed.set_height(1);
  malformed.set_row_stride_bytes(2);
  malformed.set_encoding(uw::domain::ImageFrame::IMAGE_ENCODING_RGB8);
  malformed.set_pixel_data(std::string{"\x01\x02", 2});  // too short for a 2-wide RGB8 row
  EXPECT_FALSE(uw::domain::ConvertToMono8(malformed).has_value());
}

TEST(DomainValidation, RejectsPayloadAndGridShapeMismatches) {
  uw::domain::ImageFrame image;
  image.set_width(2);
  image.set_height(2);
  image.set_row_stride_bytes(2);
  image.set_encoding(uw::domain::ImageFrame::IMAGE_ENCODING_MONO8);
  image.set_pixel_data(std::string{"\x01\x02", 2});
  EXPECT_EQ(uw::domain::ValidateImageFrame(image).code,
            uw::domain::ValidationCode::kImagePayloadSizeMismatch);

  uw::domain::OpticalDepthPriorMeasurement prior;
  prior.set_width(2);
  prior.set_height(1);
  prior.add_depth_m(1.0f);
  prior.add_variance_m2(0.01f);
  prior.add_variance_m2(0.01f);
  prior.set_valid_mask(std::string{"\x01\x01", 2});
  prior.set_scale_status(uw::domain::OPTICAL_DEPTH_SCALE_STATUS_METRIC);
  EXPECT_EQ(uw::domain::ValidateOpticalDepthPrior(prior).code,
            uw::domain::ValidationCode::kDepthGridSizeMismatch);

  uw::domain::FusedDepthMeasurement fused;
  fused.set_width(1);
  fused.set_height(1);
  fused.add_depth_m(1.0f);
  fused.add_variance_m2(0.01f);
  fused.set_valid_mask(std::string{"\x01", 1});
  EXPECT_EQ(uw::domain::ValidateFusedDepth(fused).code,
            uw::domain::ValidationCode::kContributionMaskSizeMismatch);
}

TEST(DomainValidation, RejectsInvalidMetricDepthValues) {
  uw::domain::OpticalDepthPriorMeasurement prior;
  prior.set_width(1);
  prior.set_height(1);
  prior.add_depth_m(-1.0f);
  prior.add_variance_m2(0.0f);
  prior.set_valid_mask(std::string{"\x01", 1});
  prior.set_scale_status(uw::domain::OPTICAL_DEPTH_SCALE_STATUS_METRIC);
  EXPECT_EQ(uw::domain::ValidateOpticalDepthPrior(prior).code,
            uw::domain::ValidationCode::kInvalidDepthValue);
}
