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
