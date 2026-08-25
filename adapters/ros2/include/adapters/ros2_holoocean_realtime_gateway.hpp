// ROS2 adapter -- the ONLY place ROS2 headers may appear (platform
// architecture section 5 invariant #4), together with
// ros2_svin_odometry_bridge.hpp / ros2_holoocean_sonar_bridge.hpp. Built
// only with -DUW_BUILD_ROS2=ON on a machine with a sourced ROS2 Jazzy
// environment AND `holoocean_interfaces` colcon-built onto
// CMAKE_PREFIX_PATH -- see adapters/ros2/README.md.
//
// Production role: subscribes Task 3's (adapters/holoocean/
// uw_holoocean_adapter/realtime_ros_session.py) four algorithm-input
// topics (AI-D left/right camera, sonar, noisy vehicle state) plus the
// independent PilotCamera presentation topic, converts each into a
// uw::domain message via include/adapters/holoocean_live_conversion.hpp
// (portable, no ROS2/HoloOcean types), and forwards them into a
// uw::adapters::HoloOceanRealtimeSink (include/adapters/
// holoocean_realtime_sink.hpp) -- the dependency-inversion seam that keeps
// the real LiveEventSource/OnlineAssistPipeline machinery (application/
// runtime role) out of this ROS-header-including translation unit
// entirely, per tools/lint/check_layer_dependencies.py's `ros2` role,
// which may only depend on {adapters, measurement_api, sensor_models,
// domain, domain_proto}. This class implements HoloOceanRealtimeOutput
// itself to publish the sink's operator overlay/status back onto
// `/uw/hmi/overlay` (sensor_msgs/Image) and `/uw/hmi/status`
// (std_msgs/String). It never subscribes /uw/sim/ground_truth -- only the
// four algorithm-input topics and PilotCamera are ever subscribed -- and
// never touches /uw/pilot/thrusters (Task 3's realtime_ros_session.py owns
// that entirely). Must not carry estimation/signal-processing logic of its
// own beyond the thin conversion in holoocean_live_conversion.hpp (section
// 21 invariant #4: "ROS2 拥有传输，不拥有算法语义").
#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include <holoocean_interfaces/msg/imaging_sonar.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include "adapters/holoocean_live_conversion.hpp"
#include "adapters/holoocean_realtime_sink.hpp"
#include "domain/domain.hpp"

namespace uw::adapters {

struct HoloOceanRealtimeGatewayOptions {
  std::string agent_name = "auv0";
  std::string calibration_version = "holoocean_realtime_v1";
  HoloOceanSonarCalibration sonar_calibration;
};

// Minimal, identity-extrinsic RigCalibrationSnapshot for the four algorithm
// sensors -- real calibrated intrinsics/extrinsics are not carried by
// Task 1's manifest in a directly-usable form and wiring them through is
// out of this task's scope (topic/conversion/overlay wiring only, per the
// plan text); matches apps/online_assist_smoke.cpp's own BuildRig()
// placeholder shape. Only depends on sensor_models (Pose3) + domain, both
// allowed for the `ros2` role.
uw::domain::RigCalibrationSnapshot BuildIdentityRig(const HoloOceanRealtimeGatewayOptions& options);

class HoloOceanRealtimeGatewayNode : public rclcpp::Node, private HoloOceanRealtimeOutput {
 public:
  // Reads HoloOceanRealtimeGatewayOptions from this node's own ROS2
  // parameters (declare_parameter, defaulted to mirror Task 1's
  // blue_rov_aid_sv1213_base.json manifest -- see
  // ReadOptionsFromParameters in the .cpp) rather than taking them as a
  // constructor argument, since declare_parameter needs the rclcpp::Node
  // base already constructed.
  HoloOceanRealtimeGatewayNode();

 private:
  // HoloOceanRealtimeOutput
  void PublishOverlay(uw::domain::ImageFrame frame) override;
  void PublishStatus(std::string json_status) override;

  void OnLeftCamera(const sensor_msgs::msg::Image::SharedPtr msg);
  void OnRightCamera(const sensor_msgs::msg::Image::SharedPtr msg);
  void OnPilotCamera(const sensor_msgs::msg::Image::SharedPtr msg);
  void OnSonar(const holoocean_interfaces::msg::ImagingSonar::SharedPtr msg);
  void OnVehicleState(const nav_msgs::msg::Odometry::SharedPtr msg);

  int64_t NowMonotonicNs() const;

  HoloOceanRealtimeGatewayOptions options_;
  std::unique_ptr<HoloOceanRealtimeSink> sink_;

  std::atomic<uint64_t> left_sequence_{0};
  std::atomic<uint64_t> right_sequence_{0};
  std::atomic<uint64_t> pilot_sequence_{0};
  std::atomic<uint64_t> sonar_sequence_{0};
  std::atomic<uint64_t> state_sequence_{0};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr pilot_sub_;
  rclcpp::Subscription<holoocean_interfaces::msg::ImagingSonar>::SharedPtr sonar_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
};

}  // namespace uw::adapters
