#pragma once

#include <algorithm>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>

namespace uw::runtime {

// Per platform architecture section 13: ROS callbacks / adapters only do
// bounded-queue enqueue; a scheduler decides processing order. This is the
// queue primitive shared by all four lanes (Localization/Correction/
// Mapping/Evidence) — the per-lane overflow POLICY differs (IMU must not
// randomly drop, camera/mapping may drop old/low-value items), so it is a
// constructor parameter rather than hard-coded.
enum class OverflowPolicy {
  kDropOldest,  // camera: keep most recent frames
  kDropNewest,  // rarely appropriate; kept for completeness
  kReject,      // IMU: never silently drop — caller must handle failure/backpressure
};

enum class PushResult {
  kEnqueued,
  kDroppedOldestAndEnqueued,
  kDroppedNewest,
  kRejected,
};

struct QueueStats {
  uint64_t enqueued_count = 0;
  uint64_t dequeued_count = 0;
  uint64_t dropped_oldest_count = 0;
  uint64_t dropped_newest_count = 0;
  uint64_t rejected_count = 0;
  std::size_t current_depth = 0;
  std::size_t high_watermark = 0;
};

template <typename T>
class BoundedQueue {
 public:
  BoundedQueue(std::size_t capacity, OverflowPolicy policy)
      : capacity_(capacity), policy_(policy) {
    if (capacity_ == 0) throw std::runtime_error("bounded queue capacity must be positive");
  }

  PushResult Push(T item) {
    std::lock_guard<std::mutex> lock(mutex_);
    PushResult result = PushResult::kEnqueued;
    if (items_.size() >= capacity_) {
      switch (policy_) {
        case OverflowPolicy::kDropOldest:
          items_.pop_front();
          ++stats_.dropped_oldest_count;
          result = PushResult::kDroppedOldestAndEnqueued;
          break;
        case OverflowPolicy::kDropNewest:
          ++stats_.dropped_newest_count;
          return PushResult::kDroppedNewest;
        case OverflowPolicy::kReject:
          ++stats_.rejected_count;
          return PushResult::kRejected;
      }
    }
    items_.push_back(std::move(item));
    ++stats_.enqueued_count;
    stats_.current_depth = items_.size();
    stats_.high_watermark = std::max(stats_.high_watermark, stats_.current_depth);
    cv_.notify_one();
    return result;
  }

  std::optional<T> TryPop() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (items_.empty()) return std::nullopt;
    T item = std::move(items_.front());
    items_.pop_front();
    ++stats_.dequeued_count;
    stats_.current_depth = items_.size();
    return item;
  }

  std::size_t Size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return items_.size();
  }

  uint64_t DroppedCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_.dropped_oldest_count + stats_.dropped_newest_count;
  }

  QueueStats Stats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    QueueStats snapshot = stats_;
    snapshot.current_depth = items_.size();
    return snapshot;
  }

 private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<T> items_;
  std::size_t capacity_;
  OverflowPolicy policy_;
  QueueStats stats_;
};

enum class Lane {
  kLocalization,  // highest priority: IMU, local estimator, StateStore, TF
  kCorrection,    // sonar registration, graph, relocalization
  kMapping,       // stereo/sonar integration, submap, mesh
  kEvidence,      // recorder, metrics, visualization, model log — lowest priority
};

}  // namespace uw::runtime
