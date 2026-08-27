#include "adapters/sim_wall_clock_estimator.hpp"

#include <chrono>
#include <utility>

namespace uw::adapters {
namespace {

double DefaultWallNowSeconds() {
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

}  // namespace

SimWallClockEstimator::SimWallClockEstimator(std::function<double()> wall_now_s)
    : wall_now_s_(wall_now_s ? std::move(wall_now_s) : std::function<double()>(&DefaultWallNowSeconds)) {}

void SimWallClockEstimator::Observe(const uw::domain::ObservationHeader& header) {
  if (header.clock_domain() != uw::domain::CLOCK_DOMAIN_SIMULATION) return;
  const double capture_s = uw::domain::ToSeconds(header.capture_time());
  const double wall_s = wall_now_s_();
  std::lock_guard<std::mutex> lock(mutex_);
  anchor_sim_s_ = capture_s;
  anchor_wall_s_ = wall_s;
  has_anchor_ = true;
}

uw::domain::Stamp SimWallClockEstimator::EstimateNow() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_anchor_) return uw::domain::FromSeconds(wall_now_s_());
  const double elapsed_wall_s = wall_now_s_() - anchor_wall_s_;
  return uw::domain::FromSeconds(anchor_sim_s_ + elapsed_wall_s);
}

}  // namespace uw::adapters
