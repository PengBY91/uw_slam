// L0: input schema/unit/frame/time contract checks (platform architecture
// section 15's L0 row). Cross-cutting across uw_domain's message types,
// separate from any one module's unit tests.
#include <gtest/gtest.h>

#include "uw/domain/domain.hpp"

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

TEST(DomainContract, SonarFrameAscendingAzimuthAccepted) {
  uw::domain::SonarFrame frame;
  frame.add_azimuth_angles(-0.5f);
  frame.add_azimuth_angles(0.0f);
  frame.add_azimuth_angles(0.5f);
  EXPECT_TRUE(uw::domain::IsAzimuthAscending(frame));
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
