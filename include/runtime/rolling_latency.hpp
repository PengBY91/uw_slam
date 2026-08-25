#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <vector>

namespace uw::runtime {

struct RollingLatencySnapshot {
  uint64_t sample_count = 0;
  double p50_ms = 0.0;
  double p95_ms = 0.0;
  double p99_ms = 0.0;
};

// A fixed-capacity, thread-safe window of non-negative finite latencies.
class RollingLatency {
 public:
  explicit RollingLatency(std::size_t window_size);

  void ObserveMs(double latency_ms);
  RollingLatencySnapshot Snapshot() const;

 private:
  mutable std::mutex mutex_;
  std::vector<double> samples_;
  std::size_t window_size_;
  std::size_t next_index_ = 0;
};

}  // namespace uw::runtime
