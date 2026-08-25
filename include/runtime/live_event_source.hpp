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
};

struct LiveSourceConfig {
  LaneQueueConfig localization{64, OverflowPolicy::kReject};
  LaneQueueConfig correction{32, OverflowPolicy::kDropOldest};
  LaneQueueConfig mapping{16, OverflowPolicy::kDropOldest};
  LaneQueueConfig evidence{256, OverflowPolicy::kDropOldest};
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
