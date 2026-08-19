// LocalOdometryProvider wrapper for SVIn (AutonomousFieldRoboticsLab/SVIn),
// per platform architecture section 19 ("SVIn: 第一阶段
// LocalOdometryProvider adapter") and section 22.5's minimal integration
// point: consume SVIn's ROS2 `okvis_odometry` topic (nav_msgs/Odometry) as
// black-box relative-pose evidence, and self-estimate a covariance proxy
// since that topic does not populate usable pose covariance (documented
// finding, platform architecture section 22.4).
//
// KNOWN LIMITATION (this machine, this repo state): there is no sourced
// ROS2 install here (see cmake/../CMakeLists.txt's UW_BUILD_ROS2 option),
// so this class does NOT itself subscribe to a ROS2 topic. It exposes
// PushRelativePoseEvidence() as an explicit injection point instead — the
// production path is: adapters/ros2 (built only when UW_BUILD_ROS2=ON)
// subscribes to okvis_odometry, converts to RelativePoseMeasurement, and
// calls PushRelativePoseEvidence() on an instance of this class. This
// keeps the SVIn integration point compiling and unit-testable everywhere,
// with the actual ROS2 wiring isolated to the one place that needs it.
#pragma once

#include <deque>
#include <mutex>

#include "uw/measurement_api/providers.hpp"

namespace uw::adapters {

class SvinBridgeLocalOdometryProvider : public uw::measurement_api::LocalOdometryProvider {
 public:
  void PushRelativePoseEvidence(uw::domain::MeasurementEvidence evidence);

  std::optional<uw::domain::MeasurementEvidence> PollRelativePose() override;
  uw::domain::HealthReport Health() const override;

 private:
  mutable std::mutex mutex_;
  std::deque<uw::domain::MeasurementEvidence> pending_;
  uint64_t total_pushed_ = 0;
  uint64_t total_polled_ = 0;
};

}  // namespace uw::adapters
