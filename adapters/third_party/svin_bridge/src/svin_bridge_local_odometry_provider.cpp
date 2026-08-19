#include "uw/adapters/svin_bridge_local_odometry_provider.hpp"

namespace uw::adapters {

void SvinBridgeLocalOdometryProvider::PushRelativePoseEvidence(
    uw::domain::MeasurementEvidence evidence) {
  std::lock_guard<std::mutex> lock(mutex_);
  pending_.push_back(std::move(evidence));
  ++total_pushed_;
}

std::optional<uw::domain::MeasurementEvidence> SvinBridgeLocalOdometryProvider::PollRelativePose() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (pending_.empty()) return std::nullopt;
  auto evidence = std::move(pending_.front());
  pending_.pop_front();
  ++total_polled_;
  return evidence;
}

uw::domain::HealthReport SvinBridgeLocalOdometryProvider::Health() const {
  std::lock_guard<std::mutex> lock(mutex_);
  uw::domain::HealthReport report;
  report.set_component_id("svin_bridge_local_odometry_provider");
  report.set_status(total_pushed_ == 0 ? uw::domain::HealthReport::STATUS_UNSPECIFIED
                                        : uw::domain::HealthReport::STATUS_HEALTHY);
  report.set_queue_depth(static_cast<uint32_t>(pending_.size()));
  return report;
}

}  // namespace uw::adapters
