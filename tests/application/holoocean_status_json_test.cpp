// docs/archive/rov-realtime-closed-loop-code-review-2026-08-27.md finding B2: this
// JSON-building logic previously lived anonymous-namespace-private inside
// holoocean_realtime_sink.cpp with zero test coverage (that whole file was
// only ever compile-verified against a real ROS2 toolchain, never unit
// tested -- see adapters/ros2/README.md). Extracted into
// application/holoocean_status_json.hpp specifically so it could be tested
// directly; no JSON parsing library is available in this repo's test
// dependencies, so assertions are substring/structural checks, matching
// this codebase's existing pragmatic testing style for hand-rolled
// serializers (see e.g. tests/adapters/holoocean_live_conversion_test.cpp).
#include "application/holoocean_status_json.hpp"

#include <array>
#include <string>

#include <gtest/gtest.h>

using uw::application::BuildOnlineAssistStatusJson;

namespace {

uw::domain::HealthReport MakeQueueHealth(const std::string& component_id, uint32_t queue_depth,
                                         uint32_t high_watermark, uint32_t dropped, uint32_t rejected) {
  uw::domain::HealthReport report;
  report.set_component_id(component_id);
  report.set_status(uw::domain::HealthReport::STATUS_HEALTHY);
  report.set_queue_depth(queue_depth);
  report.set_queue_high_watermark(high_watermark);
  report.set_dropped_frame_count(dropped);
  report.set_rejected_frame_count(rejected);
  report.set_oldest_message_age_ms(12.5);
  return report;
}

std::array<uw::domain::HealthReport, 4> MakeQueueHealthArray() {
  return {
      MakeQueueHealth("live_source.localization", 3, 10, 0, 2),
      MakeQueueHealth("live_source.correction", 1, 5, 4, 0),
      MakeQueueHealth("live_source.mapping", 0, 2, 0, 0),
      MakeQueueHealth("live_source.evidence", 0, 1, 0, 0),
  };
}

uw::domain::OperatorAssistState MakeState() {
  uw::domain::OperatorAssistState state;
  state.set_guidance_valid(true);
  state.set_degradation_reason("");
  state.set_data_age_ms(87.0);
  state.mutable_system_health()->set_component_id("online_assist_pipeline");
  state.mutable_system_health()->set_status(uw::domain::HealthReport::STATUS_HEALTHY);
  return state;
}

// Every '{' must have a matching '}' -- a cheap, parser-free structural
// sanity check for hand-built JSON.
bool BracesBalance(const std::string& json) {
  int depth = 0;
  for (const char c : json) {
    if (c == '{') ++depth;
    if (c == '}') --depth;
    if (depth < 0) return false;
  }
  return depth == 0;
}

}  // namespace

TEST(HolooceanStatusJson, IncludesQueueHealthSectionWithBackpressureStats) {
  const auto json = BuildOnlineAssistStatusJson(MakeState(), MakeQueueHealthArray());

  EXPECT_TRUE(BracesBalance(json));
  EXPECT_NE(json.find("\"queue_health\":{"), std::string::npos);

  // The correction lane's dropped_frame_count=4 must actually be visible --
  // this is the exact "operator has zero signal a lane is backpressuring"
  // gap finding B2 exists to close.
  EXPECT_NE(json.find("\"correction\":{"), std::string::npos);
  const auto correction_pos = json.find("\"correction\":{");
  const auto correction_section = json.substr(correction_pos, 300);
  EXPECT_NE(correction_section.find("\"dropped_frame_count\":4"), std::string::npos);
  EXPECT_NE(correction_section.find("\"queue_depth\":1"), std::string::npos);
  EXPECT_NE(correction_section.find("\"queue_high_watermark\":5"), std::string::npos);
}

TEST(HolooceanStatusJson, IncludesAllFourLanesByName) {
  const auto json = BuildOnlineAssistStatusJson(MakeState(), MakeQueueHealthArray());

  for (const char* lane : {"localization", "correction", "mapping", "evidence"}) {
    EXPECT_NE(json.find(std::string("\"") + lane + "\":{"), std::string::npos) << lane;
  }
}

TEST(HolooceanStatusJson, StillCarriesTopLevelGuidanceAndDataAgeFields) {
  const auto json = BuildOnlineAssistStatusJson(MakeState(), MakeQueueHealthArray());

  EXPECT_NE(json.find("\"guidance_valid\":true"), std::string::npos);
  EXPECT_NE(json.find("\"data_age_ms\":87"), std::string::npos);
  EXPECT_NE(json.find("\"system_health\":{\"component_id\":\"online_assist_pipeline\""),
            std::string::npos);
}

TEST(HolooceanStatusJson, EscapesControlCharactersInDegradationReason) {
  auto state = MakeState();
  state.set_degradation_reason("line1\nline2\"quoted\"");
  const auto json = BuildOnlineAssistStatusJson(state, MakeQueueHealthArray());

  EXPECT_TRUE(BracesBalance(json));
  EXPECT_NE(json.find("line1\\nline2\\\"quoted\\\""), std::string::npos);
}

TEST(HolooceanStatusJson, RejectedFrameCountIsVisiblePerLane) {
  const auto json = BuildOnlineAssistStatusJson(MakeState(), MakeQueueHealthArray());

  const auto localization_pos = json.find("\"localization\":{");
  ASSERT_NE(localization_pos, std::string::npos);
  const auto localization_section = json.substr(localization_pos, 300);
  EXPECT_NE(localization_section.find("\"rejected_frame_count\":2"), std::string::npos);
}
