#include "runtime/live_event_source.hpp"

#include <array>
#include <stdexcept>
#include <utility>

#include "domain/domain.hpp"
#include "runtime/canonical_event_validation.hpp"
#include "runtime/canonical_topics.hpp"

namespace uw::runtime {
namespace {

constexpr std::array<Lane, 15> kWeightedSchedule{
    Lane::kLocalization, Lane::kLocalization, Lane::kLocalization,
    Lane::kLocalization, Lane::kLocalization, Lane::kLocalization,
    Lane::kLocalization, Lane::kLocalization, Lane::kCorrection,
    Lane::kCorrection,   Lane::kCorrection,   Lane::kCorrection,
    Lane::kMapping,      Lane::kMapping,      Lane::kEvidence,
};

Lane LaneForKind(CanonicalEventKind kind) {
  switch (kind) {
    case CanonicalEventKind::kImuSample:
    case CanonicalEventKind::kDvlSample:
    case CanonicalEventKind::kVehicleState:
      return Lane::kLocalization;
    case CanonicalEventKind::kSonarFrame:
      return Lane::kCorrection;
    case CanonicalEventKind::kImageFrame:
      return Lane::kMapping;
    case CanonicalEventKind::kMeasurementEvidence:
    case CanonicalEventKind::kHealthReport:
    case CanonicalEventKind::kMapEvidence:
      return Lane::kEvidence;
    case CanonicalEventKind::kStateSnapshot:
      throw std::logic_error("reference-only event kind has no algorithm lane");
  }
  throw std::logic_error("canonical event kind has no explicit live-source lane");
}

const uw::domain::ObservationHeader* RawHeader(CanonicalEventKind kind,
                                                const CanonicalPayload& payload) {
  switch (kind) {
    case CanonicalEventKind::kImageFrame:
      return &std::get<uw::domain::ImageFrame>(payload).header();
    case CanonicalEventKind::kSonarFrame:
      return &std::get<uw::domain::SonarFrame>(payload).header();
    case CanonicalEventKind::kImuSample:
      return &std::get<uw::domain::ImuSample>(payload).header();
    case CanonicalEventKind::kDvlSample:
      return &std::get<uw::domain::DvlSample>(payload).header();
    case CanonicalEventKind::kVehicleState:
      return &std::get<uw::domain::VehicleState>(payload).header();
    case CanonicalEventKind::kMeasurementEvidence:
    case CanonicalEventKind::kStateSnapshot:
    case CanonicalEventKind::kHealthReport:
    case CanonicalEventKind::kMapEvidence:
      return nullptr;
  }
  throw std::logic_error("canonical event kind has no explicit header classification");
}

}  // namespace

LiveSourceConfig LiveSourceConfig::ForTest() { return {}; }

LiveEventSource::LiveEventSource(LiveSourceConfig config)
    : localization_(config.localization.capacity, config.localization.overflow_policy),
      correction_(config.correction.capacity, config.correction.overflow_policy),
      mapping_(config.mapping.capacity, config.mapping.overflow_policy),
      evidence_(config.evidence.capacity, config.evidence.overflow_policy) {}

LiveSubmitStatus LiveEventSource::Submit(CanonicalEvent event) {
  std::unique_lock<std::mutex> lock(mutex_);
  ++stats_.submit_attempt_count;
  if (closed_) {
    ++stats_.closed_rejected_count;
    return LiveSubmitStatus::kClosed;
  }

  const auto* topic_info = LookupCanonicalTopic(event.topic);
  if (topic_info == nullptr || !ValidateCanonicalEvent(event).ok()) {
    ++stats_.semantic_rejected_count;
    return LiveSubmitStatus::kSemanticRejected;
  }
  if (topic_info->role == CanonicalTopicRole::kReferenceOnly) {
    ++stats_.reference_rejected_count;
    return LiveSubmitStatus::kReferenceRejected;
  }

  const Lane lane = LaneForKind(topic_info->kind);
  const auto* header = RawHeader(topic_info->kind, event.payload);
  std::string sensor_id;
  std::string calibration_version;
  uint64_t sequence_id = 0;
  uint64_t pending_gap = 0;
  if (header != nullptr) {
    sensor_id = header->sensor_id().value();
    calibration_version = header->calibration_version().value();
    sequence_id = header->sequence_id().value();
    const auto previous = sequence_by_sensor_.find(sensor_id);
    if (previous != sequence_by_sensor_.end() &&
        previous->second.calibration_version == calibration_version) {
      if (sequence_id <= previous->second.sequence_id) {
        ++stats_.duplicate_or_out_of_order_rejected_count;
        return LiveSubmitStatus::kDuplicateOrOutOfOrderRejected;
      }
      pending_gap = sequence_id - previous->second.sequence_id - 1;
    }
  }

  const PushResult push_result = PushLocked(lane, std::move(event));
  LiveSubmitStatus status = LiveSubmitStatus::kAccepted;
  switch (push_result) {
    case PushResult::kEnqueued:
      ++queued_count_;
      ++stats_.accepted_count;
      status = LiveSubmitStatus::kAccepted;
      break;
    case PushResult::kDroppedOldestAndEnqueued:
      ++stats_.accepted_count;
      ++stats_.accepted_after_dropping_oldest_count;
      status = LiveSubmitStatus::kAcceptedAfterDroppingOldest;
      break;
    case PushResult::kDroppedNewest:
      ++stats_.dropped_newest_count;
      return LiveSubmitStatus::kDroppedNewest;
    case PushResult::kRejected:
      ++stats_.overflow_rejected_count;
      return LiveSubmitStatus::kOverflowRejected;
  }

  if (header != nullptr) {
    sequence_by_sensor_[sensor_id] = SequenceState{calibration_version, sequence_id};
    stats_.sequence_gap_count += pending_gap;
  }
  lock.unlock();
  cv_.notify_one();
  return status;
}

void LiveEventSource::Close() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    closed_ = true;
  }
  cv_.notify_all();
}

EventSourceReport LiveEventSource::Run(const EventConsumer& consumer) {
  EventSourceReport report;
  std::unique_lock<std::mutex> lock(mutex_);
  if (run_started_) {
    throw std::logic_error("LiveEventSource::Run may only be called once");
  }
  run_started_ = true;

  while (true) {
    cv_.wait(lock, [this] { return queued_count_ != 0 || closed_; });
    if (queued_count_ == 0 && closed_) {
      report.status = EventSourceStatus::kCompleted;
      report.reference_rejected_count = stats_.reference_rejected_count;
      return report;
    }

    auto event = PopNextLocked();
    if (!event.has_value()) {
      throw std::logic_error("live-source queue depth is inconsistent");
    }
    lock.unlock();

    ++report.messages_seen;
    ++report.events_emitted;
    if (!consumer(*event)) {
      lock.lock();
      report.status = EventSourceStatus::kStoppedByConsumer;
      report.reference_rejected_count = stats_.reference_rejected_count;
      return report;
    }

    lock.lock();
  }
}

LiveSourceStats LiveEventSource::Stats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  LiveSourceStats snapshot = stats_;
  snapshot.localization = localization_.Stats();
  snapshot.correction = correction_.Stats();
  snapshot.mapping = mapping_.Stats();
  snapshot.evidence = evidence_.Stats();
  return snapshot;
}

PushResult LiveEventSource::PushLocked(Lane lane, CanonicalEvent event) {
  switch (lane) {
    case Lane::kLocalization:
      return localization_.Push(std::move(event));
    case Lane::kCorrection:
      return correction_.Push(std::move(event));
    case Lane::kMapping:
      return mapping_.Push(std::move(event));
    case Lane::kEvidence:
      return evidence_.Push(std::move(event));
  }
  throw std::logic_error("unknown live-source lane");
}

std::optional<CanonicalEvent> LiveEventSource::PopNextLocked() {
  for (std::size_t offset = 0; offset < kWeightedSchedule.size(); ++offset) {
    const std::size_t index = (schedule_cursor_ + offset) % kWeightedSchedule.size();
    std::optional<CanonicalEvent> event;
    switch (kWeightedSchedule[index]) {
      case Lane::kLocalization:
        event = localization_.TryPop();
        break;
      case Lane::kCorrection:
        event = correction_.TryPop();
        break;
      case Lane::kMapping:
        event = mapping_.TryPop();
        break;
      case Lane::kEvidence:
        event = evidence_.TryPop();
        break;
    }
    if (event.has_value()) {
      schedule_cursor_ = (index + 1) % kWeightedSchedule.size();
      --queued_count_;
      return event;
    }
  }
  return std::nullopt;
}

}  // namespace uw::runtime
