// Standalone integration check (docs/superpowers/plans/2026-08-24-live-
// replay-unified-ingress.md Task 6): the same ordered CanonicalEvent
// sequence, delivered through McapEventSource (replay) and a trivial
// in-memory EventSource (standing in for a future vendor SDK live source),
// must produce an IDENTICAL normalized dispatch summary through the same
// PipelineInputPort. This is what proves the input main chain is source-
// agnostic -- it does not claim live thread scheduling exists yet (see the
// plan's section 1.2/9).
//
// The MCAP fixture is deliberately written in the REVERSE of log_time_ns
// order (see mcap_event_source_test.cpp's own comment on this trap): if
// this test's fixture were written in already-increasing time order,
// McapEventSource's LogTimeOrder sort and the SDK's default FileOrder would
// coincide, and a regression to FileOrder would go undetected here.
#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

#include "application/event_pump.hpp"
#include "application/pipeline_input_port.hpp"
#include "domain/domain.hpp"
#include "runtime/canonical_topics.hpp"
#include "runtime/event_source.hpp"
#include "runtime/mcap_event_source.hpp"
#include "runtime/mcap_io.hpp"

namespace {

using uw::runtime::CanonicalEvent;
using uw::runtime::EventConsumer;
using uw::runtime::EventSource;
using uw::runtime::EventSourceReport;
using uw::runtime::EventSourceStatus;

// Replays a fixed, already-ordered event list verbatim -- the simplest
// possible EventSource, standing in for a future vendor SDK live source.
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

std::string IdentityOf(const CanonicalEvent& event) {
  return std::visit(
      [](const auto& payload) -> std::string {
        using T = std::decay_t<decltype(payload)>;
        if constexpr (std::is_same_v<T, uw::domain::MeasurementEvidence>) {
          return payload.evidence_id().value();
        } else if constexpr (std::is_same_v<T, uw::domain::StateSnapshot>) {
          return payload.state_id().value();
        } else if constexpr (std::is_same_v<T, uw::domain::HealthReport>) {
          return payload.component_id();
        } else if constexpr (std::is_same_v<T, uw::domain::MapEvidence>) {
          return payload.evidence_id().value();
        } else {
          // ImageFrame, SonarFrame, ImuSample, DvlSample: all carry a plain
          // ObservationHeader.
          return payload.header().observation_id().value();
        }
      },
      event.payload);
}

std::string ProtobufFullNameOf(const CanonicalEvent& event) {
  return std::visit(
      [](const auto& payload) -> std::string {
        return std::string(std::decay_t<decltype(payload)>::descriptor()->full_name());
      },
      event.payload);
}

std::string SummaryLine(const CanonicalEvent& event) {
  std::ostringstream line;
  line << event.source_sequence << ',' << event.log_time_ns << ',' << event.topic << ','
       << ProtobufFullNameOf(event) << ',' << IdentityOf(event);
  return line.str();
}

class SpyInputPort final : public uw::application::PipelineInputPort {
 public:
  std::vector<std::string> summary;

  bool OnImageFrame(const CanonicalEvent& event) override { return Record(event); }
  bool OnSonarFrame(const CanonicalEvent& event) override { return Record(event); }
  bool OnImuSample(const CanonicalEvent& event) override { return Record(event); }
  bool OnDvlSample(const CanonicalEvent& event) override { return Record(event); }
  bool OnVehicleState(const CanonicalEvent& event) override { return Record(event); }
  bool OnMeasurementEvidence(const CanonicalEvent& event) override { return Record(event); }
  bool OnReferenceState(const CanonicalEvent& event) override { return Record(event); }
  bool OnHealthReport(const CanonicalEvent& event) override { return Record(event); }
  bool OnMapEvidence(const CanonicalEvent& event) override { return Record(event); }
  bool Flush() override { return true; }

 private:
  bool Record(const CanonicalEvent& event) {
    summary.push_back(SummaryLine(event));
    return true;
  }
};

// Built already sorted by log_time_ns, with source_sequence assigned 0..N-1
// in that same order -- this is what BOTH sources must reproduce: the
// in-memory one trivially (it just replays this vector), McapEventSource
// only if its LogTimeOrder read is actually wired up correctly.
std::vector<CanonicalEvent> BuildOrderedFixture() {
  std::vector<CanonicalEvent> events;
  uint64_t sequence = 0;

  uw::domain::ImageFrame image;
  image.mutable_header()->mutable_observation_id()->set_value("kf0");
  events.push_back({uw::runtime::kTopicCameraLeft, 1000, sequence++, image});

  uw::domain::SonarFrame sonar;
  sonar.mutable_header()->mutable_observation_id()->set_value("kf0");
  events.push_back({uw::runtime::kTopicSonarFrame, 2000, sequence++, sonar});

  uw::domain::ImuSample imu;
  imu.mutable_header()->mutable_observation_id()->set_value("tick0");
  events.push_back({uw::runtime::kTopicImu, 3000, sequence++, imu});

  uw::domain::DvlSample dvl;
  dvl.mutable_header()->mutable_observation_id()->set_value("tick0");
  events.push_back({uw::runtime::kTopicDvl, 4000, sequence++, dvl});

  uw::domain::MeasurementEvidence evidence;
  evidence.mutable_evidence_id()->set_value("depth_kf0");
  events.push_back({uw::runtime::kTopicEvidenceDepth, 5000, sequence++, evidence});

  uw::domain::StateSnapshot state;
  state.mutable_state_id()->set_value("kf0");
  events.push_back({uw::runtime::kTopicGtState, 6000, sequence++, state});

  uw::domain::HealthReport health;
  health.set_component_id("sonar_cfar_frontend");
  events.push_back({uw::runtime::kTopicHealth, 7000, sequence++, health});

  uw::domain::MapEvidence map_evidence;
  map_evidence.mutable_evidence_id()->set_value("landmark_0");
  events.push_back({uw::runtime::kTopicEvidenceMap, 8000, sequence++, map_evidence});

  return events;
}

bool WriteFixtureInReverseOrder(const std::string& path, const std::vector<CanonicalEvent>& events) {
  uw::runtime::McapProtobufWriter writer;
  if (!writer.Open(path)) return false;
  for (auto it = events.rbegin(); it != events.rend(); ++it) {
    const bool ok = std::visit(
        [&](const auto& payload) { return writer.WriteMessage(it->topic, it->log_time_ns, payload); },
        it->payload);
    if (!ok) return false;
  }
  writer.Close();
  return true;
}

}  // namespace

int main() {
  const std::string path = "uw_integration_event_source_parity_test.mcap";
  const auto events = BuildOrderedFixture();

  if (!WriteFixtureInReverseOrder(path, events)) {
    std::fprintf(stderr, "event source parity FAILED: could not write MCAP fixture\n");
    return 1;
  }

  InMemoryEventSource memory_source(events);
  SpyInputPort memory_spy;
  const auto memory_report = uw::application::PumpEvents(memory_source, memory_spy);

  uw::runtime::McapEventSource mcap_source(path);
  SpyInputPort mcap_spy;
  const auto mcap_report = uw::application::PumpEvents(mcap_source, mcap_spy);
  std::remove(path.c_str());

  bool ok = true;
  if (mcap_report.status != EventSourceStatus::kCompleted) {
    std::fprintf(stderr, "event source parity FAILED: McapEventSource did not complete\n");
    ok = false;
  }
  if (memory_spy.summary.size() != events.size() || mcap_spy.summary.size() != events.size()) {
    std::fprintf(stderr, "event source parity FAILED: expected %zu events, got %zu (memory) / %zu (mcap)\n",
                events.size(), memory_spy.summary.size(), mcap_spy.summary.size());
    ok = false;
  }
  if (ok && memory_spy.summary != mcap_spy.summary) {
    std::fprintf(stderr, "event source parity FAILED: summaries differ\n");
    for (std::size_t i = 0; i < memory_spy.summary.size() && i < mcap_spy.summary.size(); ++i) {
      if (memory_spy.summary[i] != mcap_spy.summary[i]) {
        std::fprintf(stderr, "  [%zu] memory=%s mcap=%s\n", i, memory_spy.summary[i].c_str(),
                    mcap_spy.summary[i].c_str());
      }
    }
    ok = false;
  }

  if (!ok) return 1;
  std::printf("event source parity OK: %zu events matched (source_sequence,log_time_ns,topic,"
              "protobuf_full_name,identity)\n",
              memory_spy.summary.size());
  return 0;
}
