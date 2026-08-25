#include "application/event_pump.hpp"

#include <cstdio>
#include <functional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "application/pipeline_input_port.hpp"
#include "domain/domain.hpp"
#include "runtime/canonical_topics.hpp"
#include "runtime/event_source.hpp"
#include "runtime/mcap_event_source.hpp"
#include "runtime/mcap_io.hpp"

namespace {

using uw::application::PipelineInputPort;
using uw::application::PumpEvents;
using uw::runtime::CanonicalEvent;
using uw::runtime::EventConsumer;
using uw::runtime::EventSource;
using uw::runtime::EventSourceReport;
using uw::runtime::EventSourceStatus;

// Minimal test double: replays a fixed event list, mirroring
// McapEventSource's own accounting (messages_seen/events_emitted
// incremented before the consumer is invoked, so a false return still
// counts that event as processed).
class InMemoryEventSource final : public EventSource {
 public:
  explicit InMemoryEventSource(std::vector<CanonicalEvent> events) : events_(std::move(events)) {}

  EventSourceReport Run(const EventConsumer& consumer) override {
    EventSourceReport report;
    for (const auto& event : events_) {
      ++report.messages_seen;
      ++report.events_emitted;
      if (!consumer(event)) {
        report.status = EventSourceStatus::kStoppedByConsumer;
        return report;
      }
    }
    report.status = EventSourceStatus::kCompleted;
    return report;
  }

 private:
  std::vector<CanonicalEvent> events_;
};

struct CallRecord {
  std::string method;
  std::string topic;
  uint64_t log_time_ns = 0;
};

class SpyInputPort final : public PipelineInputPort {
 public:
  std::vector<CallRecord> calls;
  int vehicle_state_count = 0;
  int reference_state_count = 0;
  int flush_count = 0;
  std::function<bool(const std::string&)> should_accept = [](const std::string&) { return true; };

  bool OnImageFrame(const CanonicalEvent& event) override { return Record("OnImageFrame", event); }
  bool OnSonarFrame(const CanonicalEvent& event) override { return Record("OnSonarFrame", event); }
  bool OnImuSample(const CanonicalEvent& event) override { return Record("OnImuSample", event); }
  bool OnDvlSample(const CanonicalEvent& event) override { return Record("OnDvlSample", event); }
  bool OnVehicleState(const CanonicalEvent& event) override {
    ++vehicle_state_count;
    return Record("OnVehicleState", event);
  }
  bool OnMeasurementEvidence(const CanonicalEvent& event) override {
    return Record("OnMeasurementEvidence", event);
  }
  bool OnReferenceState(const CanonicalEvent& event) override {
    ++reference_state_count;
    return Record("OnReferenceState", event);
  }
  bool OnHealthReport(const CanonicalEvent& event) override { return Record("OnHealthReport", event); }
  bool OnMapEvidence(const CanonicalEvent& event) override { return Record("OnMapEvidence", event); }
  bool Flush() override {
    ++flush_count;
    return true;
  }

 private:
  bool Record(const std::string& method, const CanonicalEvent& event) {
    calls.push_back({method, event.topic, event.log_time_ns});
    return should_accept(method);
  }
};

std::vector<CanonicalEvent> MakeOneOfEachEvent() {
  std::vector<CanonicalEvent> events;

  uw::domain::ImageFrame image;
  image.mutable_header()->mutable_observation_id()->set_value("kf0");
  events.push_back({uw::runtime::kTopicCameraLeft, 1000, 0, image});

  uw::domain::SonarFrame sonar;
  sonar.mutable_header()->mutable_observation_id()->set_value("kf0");
  events.push_back({uw::runtime::kTopicSonarFrame, 2000, 1, sonar});

  uw::domain::ImuSample imu;
  imu.mutable_header()->mutable_observation_id()->set_value("imu0");
  events.push_back({uw::runtime::kTopicImu, 3000, 2, imu});

  uw::domain::DvlSample dvl;
  dvl.mutable_header()->mutable_observation_id()->set_value("dvl0");
  events.push_back({uw::runtime::kTopicDvl, 4000, 3, dvl});

  uw::domain::VehicleState vehicle_state;
  vehicle_state.mutable_header()->mutable_observation_id()->set_value("vehicle0");
  events.push_back({uw::runtime::kTopicVehicleState, 4500, 4, vehicle_state});

  uw::domain::MeasurementEvidence evidence;
  evidence.mutable_evidence_id()->set_value("ev0");
  events.push_back({uw::runtime::kTopicEvidenceDepth, 5000, 5, evidence});

  uw::domain::StateSnapshot state;
  state.mutable_state_id()->set_value("gt0");
  events.push_back({uw::runtime::kTopicGtState, 6000, 6, state});

  uw::domain::HealthReport health;
  health.set_component_id("comp0");
  events.push_back({uw::runtime::kTopicHealth, 7000, 7, health});

  uw::domain::MapEvidence map_evidence;
  map_evidence.mutable_evidence_id()->set_value("map0");
  events.push_back({uw::runtime::kTopicEvidenceMap, 8000, 8, map_evidence});

  return events;
}

TEST(EventPump, DispatchesEachPayloadKindToItsOwnMethod) {
  InMemoryEventSource source(MakeOneOfEachEvent());
  SpyInputPort spy;

  const auto report = PumpEvents(source, spy);

  EXPECT_EQ(report.status, EventSourceStatus::kCompleted);
  ASSERT_EQ(spy.calls.size(), 9u);
  EXPECT_EQ(spy.calls[0].method, "OnImageFrame");
  EXPECT_EQ(spy.calls[1].method, "OnSonarFrame");
  EXPECT_EQ(spy.calls[2].method, "OnImuSample");
  EXPECT_EQ(spy.calls[3].method, "OnDvlSample");
  EXPECT_EQ(spy.calls[4].method, "OnVehicleState");
  EXPECT_EQ(spy.calls[5].method, "OnMeasurementEvidence");
  EXPECT_EQ(spy.calls[6].method, "OnReferenceState");
  EXPECT_EQ(spy.calls[7].method, "OnHealthReport");
  EXPECT_EQ(spy.calls[8].method, "OnMapEvidence");
  EXPECT_EQ(spy.flush_count, 1);
}

TEST(EventPump, VehicleStateAndReferenceStateUseSeparateCallbacks) {
  std::vector<CanonicalEvent> events;
  uw::domain::VehicleState vehicle_state;
  vehicle_state.mutable_header()->mutable_observation_id()->set_value("vehicle0");
  events.push_back({uw::runtime::kTopicVehicleState, 500, 0, vehicle_state});

  uw::domain::StateSnapshot state;
  state.mutable_state_id()->set_value("gt0");
  events.push_back({uw::runtime::kTopicGtState, 1000, 1, state});

  InMemoryEventSource source(events);
  SpyInputPort spy;
  PumpEvents(source, spy);

  ASSERT_EQ(spy.calls.size(), 2u);
  EXPECT_EQ(spy.vehicle_state_count, 1);
  EXPECT_EQ(spy.reference_state_count, 1);
  EXPECT_EQ(spy.calls[0].method, "OnVehicleState");
  EXPECT_EQ(spy.calls[0].topic, uw::runtime::kTopicVehicleState);
  EXPECT_EQ(spy.calls[1].method, "OnReferenceState");
  EXPECT_EQ(spy.calls[1].topic, uw::runtime::kTopicGtState);
}

TEST(EventPump, PortReturningFalseStopsSourceAndPreservesProcessedCount) {
  InMemoryEventSource source(MakeOneOfEachEvent());
  SpyInputPort spy;
  spy.should_accept = [](const std::string& method) { return method != "OnImuSample"; };

  const auto report = PumpEvents(source, spy);

  EXPECT_EQ(report.status, EventSourceStatus::kStoppedByConsumer);
  EXPECT_EQ(report.events_emitted, 3u);  // image, sonar, imu (the one that stopped it)
  ASSERT_EQ(spy.calls.size(), 3u);
  EXPECT_EQ(spy.calls.back().method, "OnImuSample");
  EXPECT_EQ(spy.flush_count, 0);  // must not Flush() on a non-kCompleted run
}

TEST(EventPump, McapSourceAndMemorySourceProduceIdenticalDispatchSequence) {
  const std::string path = "uw_application_event_pump_test.mcap";
  const auto events = MakeOneOfEachEvent();
  {
    uw::runtime::McapProtobufWriter writer;
    ASSERT_TRUE(writer.Open(path));
    for (const auto& event : events) {
      std::visit(
          [&](const auto& payload) {
            ASSERT_TRUE(writer.WriteMessage(event.topic, event.log_time_ns, payload));
          },
          event.payload);
    }
    writer.Close();
  }

  InMemoryEventSource memory_source(events);
  SpyInputPort memory_spy;
  PumpEvents(memory_source, memory_spy);

  uw::runtime::McapEventSource mcap_source(path);
  SpyInputPort mcap_spy;
  PumpEvents(mcap_source, mcap_spy);
  std::remove(path.c_str());

  ASSERT_EQ(memory_spy.calls.size(), mcap_spy.calls.size());
  for (std::size_t i = 0; i < memory_spy.calls.size(); ++i) {
    EXPECT_EQ(memory_spy.calls[i].method, mcap_spy.calls[i].method) << "index " << i;
    EXPECT_EQ(memory_spy.calls[i].topic, mcap_spy.calls[i].topic) << "index " << i;
    EXPECT_EQ(memory_spy.calls[i].log_time_ns, mcap_spy.calls[i].log_time_ns) << "index " << i;
  }
}

}  // namespace
