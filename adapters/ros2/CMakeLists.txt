# Only reached when configured with -DUW_BUILD_ROS2=ON on a machine with a
# sourced ROS2 Jazzy environment (ament_cmake on CMAKE_PREFIX_PATH). NOT
# exercised by this repo's CI/test runs in its current state — there is no
# ROS2 install on the machine this was developed on. Kept as a real
# find_package()-based target (not just commented out) so the first person
# with a ROS2 environment has a concrete starting point rather than an
# empty shell.
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

add_library(uw_ros2_svin_bridge INTERFACE)
target_include_directories(uw_ros2_svin_bridge INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(uw_ros2_svin_bridge INTERFACE uw_svin_bridge rclcpp::rclcpp ${nav_msgs_TARGETS})

message(STATUS "adapters/ros2: configured against a sourced ROS2 install — "
               "untested in this repo's current CI, see the header's doc comment")

# holoocean_interfaces is not on the public ROS2 package index — it is the
# message package from external_repos/holoocean-ros/holoocean_interfaces,
# and must be colcon-built onto CMAKE_PREFIX_PATH before this find_package()
# succeeds (see adapters/ros2/README.md and
# adapters/third_party/holoocean_ros_bridge/README.md for the workspace
# setup and field-mapping this was written against).
find_package(holoocean_interfaces REQUIRED)

add_executable(uw_holoocean_sonar_bridge_node src/holoocean_sonar_bridge_main.cpp)
target_include_directories(uw_holoocean_sonar_bridge_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(uw_holoocean_sonar_bridge_node PRIVATE
  uw_holoocean_ros_bridge rclcpp::rclcpp ${holoocean_interfaces_TARGETS})
