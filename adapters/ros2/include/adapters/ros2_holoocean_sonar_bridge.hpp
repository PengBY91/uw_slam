// ROS2 adapter — the ONLY place ROS2 headers may appear (platform
// architecture section 5 invariant #4), together with
// ros2_svin_odometry_bridge.hpp. Built only with -DUW_BUILD_ROS2=ON on a
// machine with a sourced ROS2 Jazzy environment AND `holoocean_interfaces`
// (external_repos/holoocean-ros/holoocean_interfaces) colcon-built onto
// CMAKE_PREFIX_PATH — see adapters/ros2/README.md for the exact workspace
// setup this was written and compiled against.
//
// Production role: subscribe to holoocean_main's ImagingSonar topic and
// push it into a uw::adapters::HoloOceanRosBridgeSonarFrameProvider (see
// include/adapters/holoocean_ros_bridge_sonar_frame_provider.hpp) — that class does the actual
// HoloOcean-\>SonarFrame field conversion and is usable without ROS2 (e.g.
// fed by a test or a non-ROS2 replay path). This class is the thin
// translation layer, nothing more; it must not carry any estimation or
// signal-processing logic itself (section 21 invariant #4: "ROS2 拥有传输，
// 不拥有算法语义").
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <holoocean_interfaces/msg/imaging_sonar.hpp>

#include "adapters/holoocean_ros_bridge_sonar_frame_provider.hpp"

namespace uw::adapters {

class Ros2HoloOceanSonarBridge : public rclcpp::Node {
 public:
  explicit Ros2HoloOceanSonarBridge(HoloOceanRosBridgeSonarFrameProvider& provider)
      : rclcpp::Node("uw_holoocean_sonar_bridge"), provider_(provider) {
    const std::string input_topic =
        declare_parameter<std::string>("input_topic", "/holoocean/auv0/ImagingSonar");
    subscription_ = create_subscription<holoocean_interfaces::msg::ImagingSonar>(
        input_topic, rclcpp::SensorDataQoS(),
        [this](const holoocean_interfaces::msg::ImagingSonar::SharedPtr msg) { OnImagingSonar(*msg); });
  }

 private:
  void OnImagingSonar(const holoocean_interfaces::msg::ImagingSonar& msg) {
    // No estimation/signal-processing here — msg.image_range is handed to
    // the provider's conversion code as-is; see that class's header comment
    // for the field-mapping rationale (image_range vs image_azimuth, the
    // mirror flip, FOV/range coming from HoloOceanImagingSonarParams rather
    // than the message).
    const auto observation_id = "holoocean_sonar_" + std::to_string(sequence_++);
    provider_.PushImagingSonar(msg.timestamp, static_cast<uint32_t>(msg.bins_range),
                                static_cast<uint32_t>(msg.bins_azimuth), msg.image_range,
                                observation_id);
  }

  HoloOceanRosBridgeSonarFrameProvider& provider_;
  rclcpp::Subscription<holoocean_interfaces::msg::ImagingSonar>::SharedPtr subscription_;
  uint64_t sequence_ = 0;
};

}  // namespace uw::adapters
