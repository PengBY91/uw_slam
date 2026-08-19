// ROS2 adapter skeleton — the ONLY place ROS2 headers may appear (platform
// architecture section 5 invariant #4). Not compiled by default: this
// machine has no sourced ROS2 install, so this directory is excluded from
// the build unless -DUW_BUILD_ROS2=ON with a sourced ROS2 Jazzy
// environment (see top-level CMakeLists.txt). Structure and intended
// wiring are documented here so the integration point is unambiguous once
// ROS2 is available; nothing here has been compiled or tested in this
// repo state.
//
// Production role: subscribe to SVIn's `okvis_odometry` topic
// (nav_msgs/Odometry), convert to RelativePoseMeasurement, and push it
// into a uw::adapters::SvinBridgeLocalOdometryProvider instance (see
// adapters/third_party/svin_bridge) — that class is what actually
// implements uw::measurement_api::LocalOdometryProvider and is usable
// without ROS2 (e.g. fed by a test or a non-ROS2 replay path). This class
// is the thin translation layer, nothing more; it must not carry any
// estimation logic itself (section 21 invariant #4: "ROS2 拥有传输，不拥有
// 算法语义").
#pragma once

// #include <rclcpp/rclcpp.hpp>
// #include <nav_msgs/msg/odometry.hpp>
#include "uw/adapters/svin_bridge_local_odometry_provider.hpp"

namespace uw::adapters {

// class Ros2SvinOdometryBridge : public rclcpp::Node {
//  public:
//   Ros2SvinOdometryBridge(SvinBridgeLocalOdometryProvider& provider)
//       : rclcpp::Node("uw_svin_odometry_bridge"), provider_(provider) {
//     subscription_ = create_subscription<nav_msgs::msg::Odometry>(
//         "okvis_odometry", rclcpp::SensorDataQoS(),
//         [this](const nav_msgs::msg::Odometry::SharedPtr msg) { OnOdometry(*msg); });
//   }
//
//  private:
//   void OnOdometry(const nav_msgs::msg::Odometry& msg);  // -> RelativePoseMeasurement,
//                                                          //    self-estimated covariance proxy
//                                                          //    (SVIn does not populate one —
//                                                          //    see platform architecture 22.4)
//   SvinBridgeLocalOdometryProvider& provider_;
//   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
// };

}  // namespace uw::adapters
