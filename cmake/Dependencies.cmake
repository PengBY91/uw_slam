option(UW_BUILD_ROS2 "Build the ROS2 adapter (requires a sourced ROS2 install)" OFF)
option(UW_BUILD_TESTS "Build the contract, unit, and integration tests" ON)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core calib3d imgproc)

include(UwProtobuf)
include(UwMcap)

if(UW_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  find_package(Threads REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
endif()

if(UW_BUILD_ROS2)
  find_package(rclcpp REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(holoocean_interfaces REQUIRED)
else()
  message(STATUS "UW_BUILD_ROS2=OFF: skipping ROS2 adapters")
endif()
