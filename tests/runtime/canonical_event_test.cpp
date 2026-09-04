#include "runtime/canonical_event.hpp"

#include <optional>

#include <gtest/gtest.h>

#include "domain/domain.hpp"
#include "runtime/canonical_topics.hpp"

namespace {

using uw::runtime::CanonicalEvent;
using uw::runtime::CanonicalEventKind;
using uw::runtime::CanonicalTopicRole;
using uw::runtime::LookupCanonicalTopic;
using uw::runtime::ResolveCanonicalTopic;

TEST(CanonicalTopics, KnownTopicMapsToUniqueEventKind) {
  const auto* left = LookupCanonicalTopic(uw::runtime::kTopicCameraLeft);
  ASSERT_NE(left, nullptr);
  EXPECT_EQ(left->kind, CanonicalEventKind::kImageFrame);

  const auto* sonar = LookupCanonicalTopic(uw::runtime::kTopicSonarFrame);
  ASSERT_NE(sonar, nullptr);
  EXPECT_EQ(sonar->kind, CanonicalEventKind::kSonarFrame);

  // Looking the same topic up twice must yield the same kind (a stable,
  // pure mapping -- not something that could drift call to call).
  const auto* left_again = LookupCanonicalTopic(uw::runtime::kTopicCameraLeft);
  ASSERT_NE(left_again, nullptr);
  EXPECT_EQ(left_again->kind, left->kind);

  EXPECT_EQ(LookupCanonicalTopic("/not/a/real/topic"), nullptr);
}

TEST(CanonicalTopics, TypeMismatchResolvesToNulloptNotAWrongEvent) {
  // /raw/camera/left is canonically ImageFrame; asking it to resolve against
  // SonarFrame's descriptor must fail closed, not silently hand back
  // kImageFrame (which would let a caller mis-parse the payload) or
  // kSonarFrame (which would be simply wrong).
  const auto mismatched =
      ResolveCanonicalTopic(uw::runtime::kTopicCameraLeft, uw::domain::SonarFrame::descriptor());
  EXPECT_EQ(mismatched, std::nullopt);

  const auto matched =
      ResolveCanonicalTopic(uw::runtime::kTopicCameraLeft, uw::domain::ImageFrame::descriptor());
  ASSERT_TRUE(matched.has_value());
  EXPECT_EQ(*matched, CanonicalEventKind::kImageFrame);

  EXPECT_EQ(ResolveCanonicalTopic("/not/a/real/topic", uw::domain::ImageFrame::descriptor()), std::nullopt);
}

TEST(CanonicalEventContract, PreservesOwnLogTimeAndSequenceIndependentOfCaptureTime) {
  uw::domain::ImageFrame frame_a;
  frame_a.mutable_header()->mutable_observation_id()->set_value("frame-a");
  *frame_a.mutable_header()->mutable_capture_time() = uw::domain::FromSeconds(10.0);

  uw::domain::ImageFrame frame_b;
  frame_b.mutable_header()->mutable_observation_id()->set_value("frame-b");
  *frame_b.mutable_header()->mutable_capture_time() = uw::domain::FromSeconds(10.0);  // same capture_time

  CanonicalEvent event_a;
  event_a.topic = uw::runtime::kTopicCameraLeft;
  event_a.log_time_ns = 1000;
  event_a.source_sequence = 0;
  event_a.payload = frame_a;

  CanonicalEvent event_b;
  event_b.topic = uw::runtime::kTopicCameraLeft;
  event_b.log_time_ns = 2000;
  event_b.source_sequence = 1;
  event_b.payload = frame_b;

  EXPECT_NE(event_a.log_time_ns, event_b.log_time_ns);
  EXPECT_NE(event_a.source_sequence, event_b.source_sequence);
  ASSERT_TRUE(std::holds_alternative<uw::domain::ImageFrame>(event_a.payload));
  ASSERT_TRUE(std::holds_alternative<uw::domain::ImageFrame>(event_b.payload));
  EXPECT_EQ(std::get<uw::domain::ImageFrame>(event_a.payload).header().observation_id().value(), "frame-a");
  EXPECT_EQ(std::get<uw::domain::ImageFrame>(event_b.payload).header().observation_id().value(), "frame-b");
}

TEST(CanonicalTopics, GtStateIsMarkedReferenceOnly) {
  const auto* gt_state = LookupCanonicalTopic(uw::runtime::kTopicGtState);
  ASSERT_NE(gt_state, nullptr);
  EXPECT_EQ(gt_state->kind, CanonicalEventKind::kStateSnapshot);
  EXPECT_EQ(gt_state->role, CanonicalTopicRole::kReferenceOnly);

  // Every other currently-registered topic must be algorithm input, not
  // reference-only, so a caller can safely default to "treat as input"
  // unless this specific role says otherwise.
  const auto* camera_left = LookupCanonicalTopic(uw::runtime::kTopicCameraLeft);
  ASSERT_NE(camera_left, nullptr);
  EXPECT_EQ(camera_left->role, CanonicalTopicRole::kAlgorithmInput);

  const auto* vehicle_state = LookupCanonicalTopic(uw::runtime::kTopicVehicleState);
  ASSERT_NE(vehicle_state, nullptr);
  EXPECT_EQ(vehicle_state->kind, CanonicalEventKind::kVehicleState);
  EXPECT_EQ(vehicle_state->role, CanonicalTopicRole::kAlgorithmInput);
}

TEST(CanonicalTopics, KeyframeBoundaryIsDistinctAlgorithmInputWithExactSchema) {
  const auto* boundary = LookupCanonicalTopic(uw::runtime::kTopicKeyframeBoundary);
  ASSERT_NE(boundary, nullptr);
  EXPECT_EQ(boundary->kind, CanonicalEventKind::kKeyframeBoundary);
  EXPECT_EQ(boundary->descriptor, uw::domain::KeyframeBoundary::descriptor());
  EXPECT_EQ(boundary->role, CanonicalTopicRole::kAlgorithmInput);

  EXPECT_EQ(ResolveCanonicalTopic(uw::runtime::kTopicKeyframeBoundary,
                                  uw::domain::ImageFrame::descriptor()),
            std::nullopt);
  EXPECT_EQ(ResolveCanonicalTopic(uw::runtime::kTopicKeyframeBoundary,
                                  uw::domain::KeyframeBoundary::descriptor()),
            CanonicalEventKind::kKeyframeBoundary);

  const auto* gt_state = LookupCanonicalTopic(uw::runtime::kTopicGtState);
  ASSERT_NE(gt_state, nullptr);
  EXPECT_EQ(gt_state->role, CanonicalTopicRole::kReferenceOnly);
}

}  // namespace

TEST(ControlTopics, CommandTopicsAreRegisteredSeparatelyFromCanonicalInputs) {
  // PREP-C-02: /cmd/* topics carry setpoint-level commands to the vehicle.
  // They resolve through the control registry with the right descriptor and
  // are NOT canonical algorithm inputs -- structurally they can never reach a
  // lane.
  const auto* pilot = uw::runtime::LookupControlTopic(uw::runtime::kTopicCmdPilot);
  ASSERT_NE(pilot, nullptr);
  EXPECT_EQ(pilot->descriptor, uw::domain::PilotCommand::descriptor());
  const auto* setpoint = uw::runtime::LookupControlTopic(uw::runtime::kTopicCmdSetpoint);
  ASSERT_NE(setpoint, nullptr);
  EXPECT_EQ(setpoint->descriptor, uw::domain::MotionSetpoint::descriptor());
  const auto* mode = uw::runtime::LookupControlTopic(uw::runtime::kTopicCmdMode);
  ASSERT_NE(mode, nullptr);
  EXPECT_EQ(mode->descriptor, uw::domain::FlightModeRequest::descriptor());

  EXPECT_EQ(LookupCanonicalTopic(uw::runtime::kTopicCmdPilot), nullptr);
  EXPECT_EQ(LookupCanonicalTopic(uw::runtime::kTopicCmdSetpoint), nullptr);
  EXPECT_EQ(LookupCanonicalTopic(uw::runtime::kTopicCmdMode), nullptr);
  EXPECT_EQ(uw::runtime::LookupControlTopic(uw::runtime::kTopicCameraLeft), nullptr);
  EXPECT_EQ(uw::runtime::LookupControlTopic(uw::runtime::kTopicGtState), nullptr);
}
