#pragma once

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>

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

template <typename T>
class BoundedQueue {
 public:
  BoundedQueue(std::size_t capacity, OverflowPolicy policy)
      : capacity_(capacity), policy_(policy) {}

  // Returns false if the item was rejected (only possible with kReject).
  bool Push(T item) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (items_.size() >= capacity_) {
      switch (policy_) {
        case OverflowPolicy::kDropOldest:
          items_.pop_front();
          ++dropped_count_;
          break;
        case OverflowPolicy::kDropNewest:
          ++dropped_count_;
          return true;  // silently drop the incoming item
        case OverflowPolicy::kReject:
          return false;
      }
    }
    items_.push_back(std::move(item));
    cv_.notify_one();
    return true;
  }

  std::optional<T> TryPop() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (items_.empty()) return std::nullopt;
    T item = std::move(items_.front());
    items_.pop_front();
    return item;
  }

  std::size_t Size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return items_.size();
  }

  uint64_t DroppedCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return dropped_count_;
  }

 private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<T> items_;
  std::size_t capacity_;
  OverflowPolicy policy_;
  uint64_t dropped_count_ = 0;
};

enum class Lane {
  kLocalization,  // highest priority: IMU, local estimator, StateStore, TF
  kCorrection,    // sonar registration, graph, relocalization
  kMapping,       // stereo/sonar integration, submap, mesh
  kEvidence,      // recorder, metrics, visualization, model log — lowest priority
};

}  // namespace uw::runtime
