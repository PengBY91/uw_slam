#pragma once

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include "runtime/bounded_queue.hpp"
#include "runtime/event_source.hpp"
#include "runtime/rolling_latency.hpp"

namespace uw::runtime {

struct LaneQueueConfig {
  std::size_t capacity;
  OverflowPolicy overflow_policy;
  // Maximum time (seconds) an event may sit in this lane before it is
  // dropped WITHOUT being handed to the consumer -- nullopt disables this
  // (capacity + overflow policy only, the historical behavior). Checked at
  // pop time, before the (potentially expensive) consumer callback runs:
  // per FUS-Q-004, a message that already exceeds the system's own data-
  // age budget must be dropped before expensive processing, not paid for
  // and then discarded downstream anyway. See
  // docs/archive/rov-realtime-closed-loop-code-review-2026-08-27.md findings B3/C3.
  std::optional<double> max_residence_s;
};

struct LiveSourceConfig {
  LaneQueueConfig localization{64, OverflowPolicy::kReject, std::nullopt};
  // correction (sonar -> CFAR/clustering) and mapping (camera -> visual
  // detection) are this source's two expensive-to-process lanes -- their
  // default budget matches FUS-RT-002's hard-expire ceiling (500ms): a
  // message already older than the system's own hard result-staleness
  // bound cannot produce a usable result no matter how fast it's
  // processed, so there is no point paying for the processing at all.
  // localization/evidence are left unbounded by default: their processing
  // cost is cheap regardless of staleness, and localization additionally
  // uses kReject (explicit backpressure) rather than silent dropping per
  // FUS-Q-003 -- time-dropping it by default would work against that.
  LaneQueueConfig correction{32, OverflowPolicy::kDropOldest, 0.5};
  LaneQueueConfig mapping{16, OverflowPolicy::kDropOldest, 0.5};
  LaneQueueConfig evidence{256, OverflowPolicy::kDropOldest, std::nullopt};
  std::function<std::chrono::steady_clock::time_point()> monotonic_now = [] {
    return std::chrono::steady_clock::now();
  };
  std::function<std::chrono::system_clock::time_point()> wall_now = [] {
    return std::chrono::system_clock::now();
  };

  static LiveSourceConfig ForTest();
};

enum class LiveSubmitStatus {
  kAccepted,
  kAcceptedAfterDroppingOldest,
  kDroppedNewest,
  kOverflowRejected,
  kSemanticRejected,
  kDuplicateOrOutOfOrderRejected,
  kReferenceRejected,
  kClosed,
};

struct LiveSourceStats {
  uint64_t submit_attempt_count = 0;
  // Includes events accepted after dropping the previous oldest event.
  uint64_t accepted_count = 0;
  uint64_t accepted_after_dropping_oldest_count = 0;
  uint64_t dropped_newest_count = 0;
  uint64_t overflow_rejected_count = 0;
  uint64_t semantic_rejected_count = 0;
  uint64_t duplicate_or_out_of_order_rejected_count = 0;
  uint64_t reference_rejected_count = 0;
  uint64_t closed_rejected_count = 0;
  uint64_t sequence_gap_count = 0;
  // Dropped at pop time for exceeding the lane's max_residence_s -- these
  // never reached the consumer, unlike accepted_after_dropping_oldest_count
  // (an overflow-time drop of a DIFFERENT, older queued event) or
  // dropped_newest_count (an overflow-time rejection of the incoming
  // event itself).
  uint64_t stale_dropped_count = 0;

  QueueStats localization;
  QueueStats correction;
  QueueStats mapping;
  QueueStats evidence;
};

// Four bounded input lanes with a weighted scheduler. Run() is intentionally
// single-shot, like every EventSource traversal; a second or concurrent call
// throws std::logic_error instead of racing another consumer for queued events.
class LiveEventSource final : public EventSource {
 public:
  explicit LiveEventSource(LiveSourceConfig config);

  LiveSubmitStatus Submit(CanonicalEvent event);
  void Close();
  EventSourceReport Run(const EventConsumer& consumer) override;
  LiveSourceStats Stats() const;
  // Fixed order: localization, correction, mapping, evidence.
  std::array<uw::domain::HealthReport, 4> HealthReports() const;

 private:
  using MonotonicTime = std::chrono::steady_clock::time_point;

  struct SequenceState {
    std::string calibration_version;
    uint64_t sequence_id = 0;
  };

  struct PoppedEvent {
    CanonicalEvent event;
    Lane lane;
    MonotonicTime ingress_time;
  };

  struct QueuedEvent {
    CanonicalEvent event;
    MonotonicTime ingress_time;
  };

  struct IngressNode {
    explicit IngressNode(MonotonicTime ingress) : ingress_time(ingress) {}
    MonotonicTime ingress_time;
    std::shared_ptr<IngressNode> next;
  };

  PushResult PushLocked(Lane lane, QueuedEvent event);
  std::optional<QueuedEvent> TryPopLocked(Lane lane);
  std::optional<double> MaxResidenceSForLane(Lane lane) const;
  std::optional<PoppedEvent> PopNextLocked();
  void AppendIngressLocked(Lane lane, const std::shared_ptr<IngressNode>& node);
  void RemoveOldestIngressLocked(Lane lane);
  RollingLatency& LatencyForLane(Lane lane);
  const RollingLatency& LatencyForLane(Lane lane) const;
  QueueStats QueueStatsForLane(Lane lane) const;

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  BoundedQueue<QueuedEvent> localization_;
  BoundedQueue<QueuedEvent> correction_;
  BoundedQueue<QueuedEvent> mapping_;
  BoundedQueue<QueuedEvent> evidence_;
  std::array<std::shared_ptr<IngressNode>, 4> oldest_ingress_;
  std::array<std::shared_ptr<IngressNode>, 4> newest_ingress_;
  RollingLatency localization_latency_{128};
  RollingLatency correction_latency_{128};
  RollingLatency mapping_latency_{128};
  RollingLatency evidence_latency_{128};
  std::array<uint64_t, 4> rejected_frame_counts_{};
  std::array<uint64_t, 4> stale_dropped_counts_{};
  std::array<std::optional<double>, 4> max_residence_s_by_lane_{};
  std::array<uint64_t, 4> sequence_gap_counts_{};
  std::array<std::optional<uw::domain::Stamp>, 4> last_valid_capture_times_;
  std::array<std::optional<uw::domain::Stamp>, 4> last_valid_receive_times_;
  std::array<std::optional<uw::domain::Stamp>, 4> last_processed_times_;
  std::unordered_map<std::string, SequenceState> sequence_by_sensor_;
  std::function<MonotonicTime()> monotonic_now_;
  std::function<std::chrono::system_clock::time_point()> wall_now_;
  LiveSourceStats stats_;
  std::size_t queued_count_ = 0;
  std::size_t schedule_cursor_ = 0;
  bool closed_ = false;
  bool run_started_ = false;
};

}  // namespace uw::runtime
