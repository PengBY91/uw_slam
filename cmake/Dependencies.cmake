option(UW_BUILD_ROS2 "Build the ROS2 adapter (requires a sourced ROS2 install)" OFF)
option(UW_BUILD_TESTS "Build the contract, unit, and integration tests" ON)
# Default OFF, unlike UwNanoflann's always-on FetchContent: Ceres is a heavy
# external dependency (SuiteSparse, glog, gflags), and per
# docs/superpowers/specs/2026-08-23-solver-and-mapping-oss-adoption.md this
# is a benchmark-decision-gate candidate, not (yet) the default solver — see
# that doc's §5.1/§11.1 for why find_package (conda-forge), not FetchContent.
option(UW_BUILD_CERES_SOLVER "Build the Ceres pose-graph solver adapter (requires Ceres on CMAKE_PREFIX_PATH)" OFF)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)

include(UwProtobuf)
include(UwMcap)
include(UwNanoflann)

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

if(UW_BUILD_CERES_SOLVER)
  # Ceres's CMake config transitively requires SuiteSparse_config, which
  # probes for OpenMP's C component (find_dependency(OpenMP COMPONENTS C))
  # regardless of whether Ceres itself uses OpenMP from C++ — this repo
  # otherwise has no C translation units (project(... LANGUAGES CXX) only),
  # so FindOpenMP has no C compiler to test-compile against without this.
  # Scoped to this option only, not added to the top-level project()
  # languages, since nothing else here needs it.
  enable_language(C)
  find_package(Ceres CONFIG REQUIRED)
else()
  message(STATUS "UW_BUILD_CERES_SOLVER=OFF: skipping the Ceres solver adapter")
endif()
