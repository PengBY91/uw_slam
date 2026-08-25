#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include "runtime/bounded_queue.hpp"
#include "runtime/event_source.hpp"

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

 private:
  struct SequenceState {
    std::string calibration_version;
    uint64_t sequence_id = 0;
  };

  PushResult PushLocked(Lane lane, CanonicalEvent event);
  std::optional<CanonicalEvent> PopNextLocked();

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  BoundedQueue<CanonicalEvent> localization_;
  BoundedQueue<CanonicalEvent> correction_;
  BoundedQueue<CanonicalEvent> mapping_;
  BoundedQueue<CanonicalEvent> evidence_;
  std::unordered_map<std::string, SequenceState> sequence_by_sensor_;
  LiveSourceStats stats_;
  std::size_t queued_count_ = 0;
  std::size_t schedule_cursor_ = 0;
  bool closed_ = false;
  bool run_started_ = false;
};

}  // namespace uw::runtime
