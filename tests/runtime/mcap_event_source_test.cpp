#include "runtime/mcap_event_source.hpp"

#include <cstdio>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "domain/domain.hpp"
#include "runtime/canonical_topics.hpp"
#include "runtime/mcap_io.hpp"

namespace {

using uw::runtime::CanonicalEvent;
using uw::runtime::EventSourceStatus;
using uw::runtime::McapEventSource;
using uw::runtime::McapProtobufWriter;

class McapEventSourceTest : public ::testing::Test {
 protected:
  void TearDown() override { std::remove(path_.c_str()); }

  std::string path_ = "uw_runtime_mcap_event_source_test.mcap";
};

uw::domain::ImageFrame MakeImageFrame(const std::string& observation_id) {
  uw::domain::ImageFrame frame;
  frame.mutable_header()->mutable_observation_id()->set_value(observation_id);
  frame.set_width(2);
  frame.set_height(2);
  return frame;
}

uw::domain::SonarFrame MakeSonarFrame(const std::string& observation_id) {
  uw::domain::SonarFrame frame;
  frame.mutable_header()->mutable_observation_id()->set_value(observation_id);
  frame.set_num_ranges(1);
  frame.set_num_beams(1);
  return frame;
}

TEST_F(McapEventSourceTest, OrdersEventsByLogTimeNotWriteOrder) {
  {
    McapProtobufWriter writer;
    ASSERT_TRUE(writer.Open(path_));
    // Deliberately write the LATER-timestamped sonar message first, and the
    // EARLIER-timestamped image message second. If the reader used file
    // (write) order instead of logTime order, this would come back sonar
    // first -- this is the exact trap called out in the plan: writing in
    // timestamp order would make FileOrder and LogTimeOrder coincide and
    // hide a broken sort.
    ASSERT_TRUE(writer.WriteMessage(uw::runtime::kTopicSonarFrame, /*log_time_ns=*/2000,
                                     MakeSonarFrame("kf0")));
    ASSERT_TRUE(writer.WriteMessage(uw::runtime::kTopicCameraLeft, /*log_time_ns=*/1000,
                                     MakeImageFrame("kf0")));
    writer.Close();
  }

  McapEventSource source(path_);
  std::vector<CanonicalEvent> events;
  const auto report = source.Run([&](const CanonicalEvent& event) {
    events.push_back(event);
    return true;
  });

  EXPECT_EQ(report.status, EventSourceStatus::kCompleted);
  EXPECT_EQ(report.messages_seen, 2u);
  EXPECT_EQ(report.events_emitted, 2u);
  EXPECT_EQ(report.unknown_topic_count, 0u);
  EXPECT_EQ(report.parse_failure_count, 0u);

  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0].topic, uw::runtime::kTopicCameraLeft);
  EXPECT_EQ(events[0].log_time_ns, 1000u);
  EXPECT_EQ(events[1].topic, uw::runtime::kTopicSonarFrame);
  EXPECT_EQ(events[1].log_time_ns, 2000u);
  // Stable tie-break sequencing: strictly increasing regardless of topic.
  EXPECT_LT(events[0].source_sequence, events[1].source_sequence);
}

TEST_F(McapEventSourceTest, CountsUnknownTopicsWithoutEmittingEvents) {
  {
    McapProtobufWriter writer;
    ASSERT_TRUE(writer.Open(path_));
    ASSERT_TRUE(
        writer.WriteMessage(uw::runtime::kTopicCameraLeft, 1000, MakeImageFrame("kf0")));
    // Not a registered canonical topic.
    uw::domain::HealthReport health;
    health.set_component_id("scratch");
    ASSERT_TRUE(writer.WriteMessage("/scratch/not_canonical", 1500, health));
    writer.Close();
  }

  McapEventSource source(path_);
  std::vector<CanonicalEvent> events;
  const auto report = source.Run([&](const CanonicalEvent& event) {
    events.push_back(event);
    return true;
  });

  EXPECT_EQ(report.status, EventSourceStatus::kCompleted);
  EXPECT_EQ(report.messages_seen, 2u);
  EXPECT_EQ(report.events_emitted, 1u);
  EXPECT_EQ(report.unknown_topic_count, 1u);
  EXPECT_EQ(report.parse_failure_count, 0u);
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].topic, uw::runtime::kTopicCameraLeft);
}

TEST_F(McapEventSourceTest, CountsSchemaMismatchAsParseFailureNotAWrongEvent) {
  {
    McapProtobufWriter writer;
    ASSERT_TRUE(writer.Open(path_));
    // /raw/camera/left is canonically ImageFrame; write a SonarFrame there
    // instead (e.g. a topic reused across schema versions/producers).
    ASSERT_TRUE(
        writer.WriteMessage(uw::runtime::kTopicCameraLeft, 1000, MakeSonarFrame("kf0")));
    writer.Close();
  }

  McapEventSource source(path_);
  std::vector<CanonicalEvent> events;
  const auto report = source.Run([&](const CanonicalEvent& event) {
    events.push_back(event);
    return true;
  });

  EXPECT_EQ(report.status, EventSourceStatus::kCompleted);
  EXPECT_EQ(report.messages_seen, 1u);
  EXPECT_EQ(report.events_emitted, 0u);
  EXPECT_EQ(report.unknown_topic_count, 0u);
  EXPECT_EQ(report.parse_failure_count, 1u);
  EXPECT_TRUE(events.empty());
}

TEST_F(McapEventSourceTest, ReturnsOpenFailedForMissingFile) {
  McapEventSource source("/nonexistent/path/does_not_exist.mcap");
  const auto report = source.Run([](const CanonicalEvent&) { return true; });
  EXPECT_EQ(report.status, EventSourceStatus::kOpenFailed);
  EXPECT_EQ(report.messages_seen, 0u);
  EXPECT_EQ(report.events_emitted, 0u);
}

TEST_F(McapEventSourceTest, StoppedByConsumerStopsEarlyAndClosesCleanly) {
  {
    McapProtobufWriter writer;
    ASSERT_TRUE(writer.Open(path_));
    ASSERT_TRUE(writer.WriteMessage(uw::runtime::kTopicCameraLeft, 1000, MakeImageFrame("kf0")));
    ASSERT_TRUE(writer.WriteMessage(uw::runtime::kTopicCameraLeft, 2000, MakeImageFrame("kf1")));
    ASSERT_TRUE(writer.WriteMessage(uw::runtime::kTopicCameraLeft, 3000, MakeImageFrame("kf2")));
    writer.Close();
  }

  McapEventSource source(path_);
  int seen = 0;
  const auto report = source.Run([&](const CanonicalEvent&) {
    ++seen;
    return seen < 1;  // stop after the first event
  });

  EXPECT_EQ(report.status, EventSourceStatus::kStoppedByConsumer);
  EXPECT_EQ(report.events_emitted, 1u);
  EXPECT_EQ(seen, 1);
}

}  // namespace

TEST_F(McapEventSourceTest, CountsRecordedControlCommandsWithoutEmittingEvents) {
  // PREP-C-02: a bag may carry the pilot's /cmd/pilot commands for 复盘.
  // They are neither algorithm inputs nor unknown topics.
  {
    McapProtobufWriter writer;
    ASSERT_TRUE(writer.Open(path_));
    ASSERT_TRUE(writer.WriteMessage(uw::runtime::kTopicCameraLeft, 1000, MakeImageFrame("kf0")));
    uw::domain::PilotCommand command;
    command.mutable_header()->mutable_observation_id()->set_value("cmd0");
    command.set_surge(0.5);
    command.set_source(uw::domain::PilotCommand::SOURCE_PILOT);
    ASSERT_TRUE(writer.WriteMessage(uw::runtime::kTopicCmdPilot, 1200, command));
    writer.Close();
  }

  McapEventSource source(path_);
  std::vector<CanonicalEvent> events;
  const auto report = source.Run([&](const CanonicalEvent& event) {
    events.push_back(event);
    return true;
  });

  EXPECT_EQ(report.status, EventSourceStatus::kCompleted);
  EXPECT_EQ(report.messages_seen, 2u);
  EXPECT_EQ(report.events_emitted, 1u);
  EXPECT_EQ(report.unknown_topic_count, 0u);
  EXPECT_EQ(report.control_message_count, 1u);
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].topic, uw::runtime::kTopicCameraLeft);
}
