#include "runtime/live_event_source.hpp"

#include <algorithm>
#include <array>
#include <limits>
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

constexpr std::array<Lane, 4> kLaneOrder{
    Lane::kLocalization, Lane::kCorrection, Lane::kMapping, Lane::kEvidence};

constexpr std::array<const char*, 4> kComponentIds{
    "live_source.localization", "live_source.correction", "live_source.mapping",
    "live_source.evidence"};

std::size_t LaneIndex(Lane lane) {
  switch (lane) {
    case Lane::kLocalization:
      return 0;
    case Lane::kCorrection:
      return 1;
    case Lane::kMapping:
      return 2;
    case Lane::kEvidence:
      return 3;
  }
  throw std::logic_error("unknown live-source lane");
}

uint32_t SaturatingUint32(std::size_t value) {
  constexpr auto kMax = std::numeric_limits<uint32_t>::max();
  return value > static_cast<std::size_t>(kMax) ? kMax : static_cast<uint32_t>(value);
}

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
      evidence_(config.evidence.capacity, config.evidence.overflow_policy),
      monotonic_now_(std::move(config.monotonic_now)),
      wall_now_(std::move(config.wall_now)) {
  if (!monotonic_now_ || !wall_now_) {
    throw std::invalid_argument("live-source clock callbacks must be provided");
  }
}

LiveSubmitStatus LiveEventSource::Submit(CanonicalEvent event) {
  // User-provided clocks are always invoked outside mutex_, so a callback
  // may safely query Stats() without causing a re-entrant lock deadlock.
  const MonotonicTime ingress_time = monotonic_now_();
  std::unique_lock<std::mutex> lock(mutex_);
  ++stats_.submit_attempt_count;
  if (closed_) {
    ++stats_.closed_rejected_count;
    return LiveSubmitStatus::kClosed;
  }

  const auto* topic_info = LookupCanonicalTopic(event.topic);
  if (topic_info == nullptr) {
    ++stats_.semantic_rejected_count;
    return LiveSubmitStatus::kSemanticRejected;
  }
  if (!ValidateCanonicalEvent(event).ok()) {
    ++stats_.semantic_rejected_count;
    if (topic_info->role == CanonicalTopicRole::kAlgorithmInput) {
      ++rejected_frame_counts_[LaneIndex(LaneForKind(topic_info->kind))];
    }
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
  std::optional<uw::domain::Stamp> capture_time;
  std::optional<uw::domain::Stamp> receive_time;
  if (header != nullptr) {
    sensor_id = header->sensor_id().value();
    calibration_version = header->calibration_version().value();
    sequence_id = header->sequence_id().value();
    capture_time = header->capture_time();
    receive_time = header->receive_time();
    const auto previous = sequence_by_sensor_.find(sensor_id);
    if (previous != sequence_by_sensor_.end() &&
        previous->second.calibration_version == calibration_version) {
      if (sequence_id <= previous->second.sequence_id) {
        ++stats_.duplicate_or_out_of_order_rejected_count;
        ++rejected_frame_counts_[LaneIndex(lane)];
        return LiveSubmitStatus::kDuplicateOrOutOfOrderRejected;
      }
      pending_gap = sequence_id - previous->second.sequence_id - 1;
    }
  }

  // Allocate the oldest-age tracking node before mutating the bounded queue.
  // Once PushLocked succeeds, linking/removing shared_ptr nodes is noexcept.
  const auto ingress_node = std::make_shared<IngressNode>(ingress_time);
  const PushResult push_result =
      PushLocked(lane, QueuedEvent{std::move(event), ingress_time});
  LiveSubmitStatus status = LiveSubmitStatus::kAccepted;
  switch (push_result) {
    case PushResult::kEnqueued:
      AppendIngressLocked(lane, ingress_node);
      ++queued_count_;
      ++stats_.accepted_count;
      status = LiveSubmitStatus::kAccepted;
      break;
    case PushResult::kDroppedOldestAndEnqueued:
      RemoveOldestIngressLocked(lane);
      AppendIngressLocked(lane, ingress_node);
      ++stats_.accepted_count;
      ++stats_.accepted_after_dropping_oldest_count;
      status = LiveSubmitStatus::kAcceptedAfterDroppingOldest;
      break;
    case PushResult::kDroppedNewest:
      ++stats_.dropped_newest_count;
      return LiveSubmitStatus::kDroppedNewest;
    case PushResult::kRejected:
      ++stats_.overflow_rejected_count;
      ++rejected_frame_counts_[LaneIndex(lane)];
      return LiveSubmitStatus::kOverflowRejected;
  }

  if (capture_time.has_value()) {
    sequence_by_sensor_[sensor_id] = SequenceState{calibration_version, sequence_id};
    stats_.sequence_gap_count += pending_gap;
    sequence_gap_counts_[LaneIndex(lane)] += pending_gap;
    last_valid_capture_times_[LaneIndex(lane)] = std::move(capture_time);
    last_valid_receive_times_[LaneIndex(lane)] = std::move(receive_time);
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

    lock.unlock();
    const MonotonicTime pop_time = monotonic_now_();
    const auto processed_time = wall_now_();
    lock.lock();

    auto popped = PopNextLocked();
    if (!popped.has_value()) {
      throw std::logic_error("live-source queue depth is inconsistent");
    }
    const double residence_ms = std::max(
        0.0, std::chrono::duration<double, std::milli>(pop_time - popped->ingress_time)
                 .count());
    LatencyForLane(popped->lane).ObserveMs(residence_ms);
    last_processed_times_[LaneIndex(popped->lane)] = uw::domain::ToStamp(processed_time);
    lock.unlock();

    ++report.messages_seen;
    ++report.events_emitted;
    if (!consumer(popped->event)) {
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

std::array<uw::domain::HealthReport, 4> LiveEventSource::HealthReports() const {
  // The monotonic callback is outside mutex_ by contract, including when a
  // test or embedding supplies a callback that queries this source.
  const MonotonicTime now = monotonic_now_();
  std::lock_guard<std::mutex> lock(mutex_);
  std::array<uw::domain::HealthReport, 4> reports;
  for (std::size_t index = 0; index < kLaneOrder.size(); ++index) {
    const Lane lane = kLaneOrder[index];
    const QueueStats queue_stats = QueueStatsForLane(lane);
    const RollingLatencySnapshot latency = LatencyForLane(lane).Snapshot();
    auto& report = reports[index];
    report.set_component_id(kComponentIds[index]);
    report.set_status(uw::domain::HealthReport::STATUS_HEALTHY);
    report.set_queue_depth(SaturatingUint32(queue_stats.current_depth));
    report.set_queue_high_watermark(SaturatingUint32(queue_stats.high_watermark));
    report.set_dropped_frame_count(queue_stats.dropped_oldest_count +
                                   queue_stats.dropped_newest_count);
    report.set_rejected_frame_count(rejected_frame_counts_[index]);
    report.set_sequence_gap_count(sequence_gap_counts_[index]);
    report.set_latency_p50_ms(latency.p50_ms);
    report.set_latency_p95_ms(latency.p95_ms);
    report.set_latency_p99_ms(latency.p99_ms);
    if (oldest_ingress_[index]) {
      report.set_oldest_message_age_ms(std::max(
          0.0, std::chrono::duration<double, std::milli>(
                   now - oldest_ingress_[index]->ingress_time)
                   .count()));
    }
    if (last_valid_capture_times_[index]) {
      *report.mutable_last_valid_capture_time() = *last_valid_capture_times_[index];
    }
    if (last_valid_receive_times_[index]) {
      *report.mutable_last_valid_receive_time() = *last_valid_receive_times_[index];
    }
    if (last_processed_times_[index]) {
      *report.mutable_last_processed_time() = *last_processed_times_[index];
    }
  }
  return reports;
}

PushResult LiveEventSource::PushLocked(Lane lane, QueuedEvent event) {
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

std::optional<LiveEventSource::PoppedEvent> LiveEventSource::PopNextLocked() {
  for (std::size_t offset = 0; offset < kWeightedSchedule.size(); ++offset) {
    const std::size_t index = (schedule_cursor_ + offset) % kWeightedSchedule.size();
    const Lane lane = kWeightedSchedule[index];
    std::optional<QueuedEvent> event;
    switch (lane) {
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
      RemoveOldestIngressLocked(lane);
      return PoppedEvent{std::move(event->event), lane, event->ingress_time};
    }
  }
  return std::nullopt;
}

void LiveEventSource::AppendIngressLocked(
    Lane lane, const std::shared_ptr<IngressNode>& node) {
  const std::size_t index = LaneIndex(lane);
  if (newest_ingress_[index]) {
    newest_ingress_[index]->next = node;
  } else {
    oldest_ingress_[index] = node;
  }
  newest_ingress_[index] = node;
}

void LiveEventSource::RemoveOldestIngressLocked(Lane lane) {
  const std::size_t index = LaneIndex(lane);
  if (!oldest_ingress_[index]) {
    throw std::logic_error("live-source ingress index is inconsistent");
  }
  oldest_ingress_[index] = oldest_ingress_[index]->next;
  if (!oldest_ingress_[index]) newest_ingress_[index].reset();
}

RollingLatency& LiveEventSource::LatencyForLane(Lane lane) {
  switch (lane) {
    case Lane::kLocalization:
      return localization_latency_;
    case Lane::kCorrection:
      return correction_latency_;
    case Lane::kMapping:
      return mapping_latency_;
    case Lane::kEvidence:
      return evidence_latency_;
  }
  throw std::logic_error("unknown live-source lane");
}

const RollingLatency& LiveEventSource::LatencyForLane(Lane lane) const {
  switch (lane) {
    case Lane::kLocalization:
      return localization_latency_;
    case Lane::kCorrection:
      return correction_latency_;
    case Lane::kMapping:
      return mapping_latency_;
    case Lane::kEvidence:
      return evidence_latency_;
  }
  throw std::logic_error("unknown live-source lane");
}

QueueStats LiveEventSource::QueueStatsForLane(Lane lane) const {
  switch (lane) {
    case Lane::kLocalization:
      return localization_.Stats();
    case Lane::kCorrection:
      return correction_.Stats();
    case Lane::kMapping:
      return mapping_.Stats();
    case Lane::kEvidence:
      return evidence_.Stats();
  }
  throw std::logic_error("unknown live-source lane");
}

}  // namespace uw::runtime
