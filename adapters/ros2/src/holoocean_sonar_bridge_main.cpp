// Standalone executable: spins Ros2HoloOceanSonarBridge against a live
// holoocean_main ImagingSonar topic. This is the transport-only half of the
// SonarFrameProvider wiring — nothing downstream (SonarFrontend) is driven
// from here yet; see adapters/ros2/README.md for what is/isn't validated.
#include <rclcpp/rclcpp.hpp>

#include "uw/adapters/ros2_holoocean_sonar_bridge.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  uw::adapters::HoloOceanImagingSonarParams params;
  params.horizontal_fov_rad = 2.0943951f;  // 120 deg, matches holoocean_bridge's default scenario FOV
  params.min_range_m = 0.5f;
  params.max_range_m = 30.0f;
  uw::adapters::HoloOceanRosBridgeSonarFrameProvider provider(params);
  auto node = std::make_shared<uw::adapters::Ros2HoloOceanSonarBridge>(provider);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
