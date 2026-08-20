cmake_minimum_required(VERSION 3.22)
project(uw_slam CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

option(UW_BUILD_ROS2 "Build the ROS2 adapter (requires a sourced ROS2 install)" OFF)
option(UW_BUILD_TESTS "Build the L0/L1/L2 test suites" ON)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)

include(UwProtobuf)   # -> uw_domain_proto
include(UwMcap)        # -> mcap

if(UW_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
endif()

# Dependency order mirrors platform architecture section 5's DAG: domain has
# no deps; sensor_models/measurement_api depend only on domain; algorithms
# depend on measurement_api; runtime/adapters/apps sit on top. Each
# add_subdirectory below only depends on ones already added.
add_subdirectory(core/domain)
add_subdirectory(core/sensor_models)
add_subdirectory(core/measurement_api)

add_subdirectory(algorithms/factor_builders/sonar_range_factor)
add_subdirectory(algorithms/factor_builders/relative_pose_factor)
add_subdirectory(algorithms/factor_builders/depth_factor)
add_subdirectory(algorithms/frontends/sonar_cfar_frontend)
add_subdirectory(algorithms/frontends/stereo_optical_depth_frontend)
add_subdirectory(algorithms/frontends/acoustic_optic_associator)
add_subdirectory(algorithms/frontends/acoustic_optic_depth_fusion)
add_subdirectory(algorithms/estimation)
add_subdirectory(algorithms/mapping/submap_manager)

add_subdirectory(runtime)
add_subdirectory(evaluation)

if(UW_BUILD_ROS2)
  add_subdirectory(adapters/ros2)
else()
  message(STATUS "UW_BUILD_ROS2=OFF: skipping adapters/ros2 (needs a sourced ROS2 Jazzy install)")
endif()
add_subdirectory(adapters/third_party/svin_bridge)
add_subdirectory(adapters/third_party/holoocean_ros_bridge)

add_subdirectory(apps/tools/synth_bag_gen)
add_subdirectory(apps/tools/synth_stereo_gen)
add_subdirectory(apps/replay_demo)
add_subdirectory(apps/tools/optical_baseline_eval)
add_subdirectory(apps/tools/acoustic_optic_scenario_matrix)

if(UW_BUILD_TESTS)
  add_subdirectory(tests)
endif()
