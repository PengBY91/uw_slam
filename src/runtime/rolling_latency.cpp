#include "runtime/rolling_latency.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace uw::runtime {
namespace {

double NearestRank(const std::vector<double>& sorted_samples, double percentile) {
  if (sorted_samples.empty()) return 0.0;
  const auto rank = static_cast<std::size_t>(
      std::ceil(percentile * static_cast<double>(sorted_samples.size())));
  return sorted_samples[rank - 1];
}

}  // namespace

RollingLatency::RollingLatency(std::size_t window_size) : window_size_(window_size) {
  if (window_size_ == 0) {
    throw std::invalid_argument("rolling latency window size must be positive");
  }
  samples_.reserve(window_size_);
}

void RollingLatency::ObserveMs(double latency_ms) {
  if (!std::isfinite(latency_ms) || latency_ms < 0.0) {
    throw std::invalid_argument("rolling latency sample must be finite and non-negative");
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (samples_.size() < window_size_) {
    samples_.push_back(latency_ms);
    return;
  }
  samples_[next_index_] = latency_ms;
  next_index_ = (next_index_ + 1) % window_size_;
}

RollingLatencySnapshot RollingLatency::Snapshot() const {
  std::vector<double> sorted_samples;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    sorted_samples = samples_;
  }
  std::sort(sorted_samples.begin(), sorted_samples.end());
  return {static_cast<uint64_t>(sorted_samples.size()),
          NearestRank(sorted_samples, 0.50), NearestRank(sorted_samples, 0.95),
          NearestRank(sorted_samples, 0.99)};
}

}  // namespace uw::runtime
