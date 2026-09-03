#include "runtime/mcap_event_source.hpp"

#include <optional>
#include <utility>

#include <mcap/reader.hpp>

#include "runtime/canonical_topics.hpp"

namespace uw::runtime {

namespace {

std::optional<CanonicalPayload> ParsePayload(CanonicalEventKind kind, const std::byte* data,
                                              uint64_t size) {
  const auto parse_size = static_cast<int>(size);
  switch (kind) {
    case CanonicalEventKind::kImageFrame: {
      uw::domain::ImageFrame message;
      if (!message.ParseFromArray(data, parse_size)) return std::nullopt;
      return CanonicalPayload(std::move(message));
    }
    case CanonicalEventKind::kSonarFrame: {
      uw::domain::SonarFrame message;
      if (!message.ParseFromArray(data, parse_size)) return std::nullopt;
      return CanonicalPayload(std::move(message));
    }
    case CanonicalEventKind::kImuSample: {
      uw::domain::ImuSample message;
      if (!message.ParseFromArray(data, parse_size)) return std::nullopt;
      return CanonicalPayload(std::move(message));
    }
    case CanonicalEventKind::kDvlSample: {
      uw::domain::DvlSample message;
      if (!message.ParseFromArray(data, parse_size)) return std::nullopt;
      return CanonicalPayload(std::move(message));
    }
    case CanonicalEventKind::kVehicleState: {
      uw::domain::VehicleState message;
      if (!message.ParseFromArray(data, parse_size)) return std::nullopt;
      return CanonicalPayload(std::move(message));
    }
    case CanonicalEventKind::kMeasurementEvidence: {
      uw::domain::MeasurementEvidence message;
      if (!message.ParseFromArray(data, parse_size)) return std::nullopt;
      return CanonicalPayload(std::move(message));
    }
    case CanonicalEventKind::kStateSnapshot: {
      uw::domain::StateSnapshot message;
      if (!message.ParseFromArray(data, parse_size)) return std::nullopt;
      return CanonicalPayload(std::move(message));
    }
    case CanonicalEventKind::kHealthReport: {
      uw::domain::HealthReport message;
      if (!message.ParseFromArray(data, parse_size)) return std::nullopt;
      return CanonicalPayload(std::move(message));
    }
    case CanonicalEventKind::kMapEvidence: {
      uw::domain::MapEvidence message;
      if (!message.ParseFromArray(data, parse_size)) return std::nullopt;
      return CanonicalPayload(std::move(message));
    }
  }
  return std::nullopt;
}

}  // namespace

McapEventSource::McapEventSource(std::string path) : path_(std::move(path)) {}

EventSourceReport McapEventSource::Run(const EventConsumer& consumer) {
  EventSourceReport report;

  mcap::McapReader reader;
  if (!reader.open(path_).ok()) {
    report.status = EventSourceStatus::kOpenFailed;
    return report;
  }

  // LogTimeOrder is not the SDK default (FileOrder, i.e. write order) --
  // explicit here because completion standard #2 requires event order to be
  // defined by logTime, not by whatever order a producer happened to call
  // WriteMessage() in.
  mcap::ReadMessageOptions options;
  options.readOrder = mcap::ReadMessageOptions::ReadOrder::LogTimeOrder;

  uint64_t sequence = 0;
  bool stopped_by_consumer = false;
  for (const auto& view : reader.readMessages([](const mcap::Status&) {}, options)) {
    ++report.messages_seen;
    if (view.channel == nullptr) {
      ++report.unknown_topic_count;
      continue;
    }

    if (LookupControlTopic(view.channel->topic) != nullptr) {
      ++report.control_message_count;
      continue;
    }
    const auto* topic_info = LookupCanonicalTopic(view.channel->topic);
    if (topic_info == nullptr) {
      ++report.unknown_topic_count;
      continue;
    }

    if (view.schema == nullptr || view.schema->name != topic_info->descriptor->full_name()) {
      ++report.parse_failure_count;
      continue;
    }

    auto payload = ParsePayload(topic_info->kind, view.message.data, view.message.dataSize);
    if (!payload.has_value()) {
      ++report.parse_failure_count;
      continue;
    }

    CanonicalEvent event;
    event.topic = view.channel->topic;
    event.log_time_ns = view.message.logTime;
    event.source_sequence = sequence++;
    event.payload = std::move(*payload);

    ++report.events_emitted;
    if (!consumer(event)) {
      stopped_by_consumer = true;
      break;
    }
  }

  reader.close();
  report.status =
      stopped_by_consumer ? EventSourceStatus::kStoppedByConsumer : EventSourceStatus::kCompleted;
  return report;
}

}  // namespace uw::runtime
