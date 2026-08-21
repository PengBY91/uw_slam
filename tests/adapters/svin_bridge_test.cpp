#include "adapters/svin_bridge_local_odometry_provider.hpp"

#include <gtest/gtest.h>

using uw::adapters::SvinBridgeLocalOdometryProvider;

TEST(SvinBridgeLocalOdometryProvider, PollsInFifoOrderAndReportsHealth) {
  SvinBridgeLocalOdometryProvider provider;
  EXPECT_FALSE(provider.PollRelativePose().has_value());
  EXPECT_EQ(provider.Health().status(), uw::domain::HealthReport::STATUS_UNSPECIFIED);

  uw::domain::MeasurementEvidence first;
  first.mutable_evidence_id()->set_value("e1");
  uw::domain::MeasurementEvidence second;
  second.mutable_evidence_id()->set_value("e2");

  provider.PushRelativePoseEvidence(first);
  provider.PushRelativePoseEvidence(second);
  EXPECT_EQ(provider.Health().queue_depth(), 2u);

  auto polled_first = provider.PollRelativePose();
  ASSERT_TRUE(polled_first.has_value());
  EXPECT_EQ(polled_first->evidence_id().value(), "e1");

  auto polled_second = provider.PollRelativePose();
  ASSERT_TRUE(polled_second.has_value());
  EXPECT_EQ(polled_second->evidence_id().value(), "e2");

  EXPECT_FALSE(provider.PollRelativePose().has_value());
  EXPECT_EQ(provider.Health().status(), uw::domain::HealthReport::STATUS_HEALTHY);
}
