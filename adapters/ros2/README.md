# `adapters/ros2`

The only place ROS2 headers may appear in this repository (platform
architecture section 5 invariant #4). Built only with `-DUW_BUILD_ROS2=ON`
on a machine with a sourced ROS2 install — see the top-level `CMakeLists.txt`.

| Target | Wraps | Depends on (beyond rclcpp) | Status |
|---|---|---|---|
| `uw::ros2_adapters` | portable `include/adapters/` providers (SVIn `LocalOdometryProvider`, HoloOcean `SonarFrameProvider`, see `adapters/svin_bridge.md`/`adapters/holoocean_ros_bridge.md`) | `nav_msgs`, `holoocean_interfaces` | The SVIn side (`ros2_svin_odometry_bridge.hpp`) is header content fully commented out — documented skeleton, never compiled (see the header's doc comment). |
| `holoocean_sonar_bridge_node` | `include/adapters/holoocean_ros_bridge_sonar_frame_provider.hpp` via `uw::ros2_adapters` | `holoocean_interfaces` (from `external_repos/holoocean-ros/holoocean_interfaces`, colcon-built) | Real `rclcpp::Node`, compiled and run against ROS2 Jazzy on this machine — see below for exactly what was/wasn't verified. |

## `holoocean_interfaces` is not on the ROS2 package index

It is HoloOcean's own message package (MIT, `external_repos/holoocean-ros/holoocean_interfaces`),
not something `rosdep`/apt can resolve. `find_package(holoocean_interfaces REQUIRED)`
in this directory's `CMakeLists.txt` only succeeds once it is colcon-built and
its `install/` is on `CMAKE_PREFIX_PATH` (the same one used to source the ROS2
environment itself works, since `colcon build --merge-install` installs into
it). This repo does not vendor or build that workspace — it lives in a
separate colcon workspace outside this repo (see the setup notes this session
left in `external_repos/holoocean-ros`'s own README for prerequisites), kept
distinct because `holoocean_interfaces` is a colcon package with its own
build/install/log artifacts that do not belong inside a CMake-only C++ repo.

## What has and hasn't been verified

- `holoocean_sonar_bridge_node` compiles and links against a real
  `rclcpp`/`holoocean_interfaces` on this machine.
- It has **not** been run against a live `holoocean_main` process — that
  needs the actual HoloOcean simulator (Unreal Engine binary, Epic Games EULA,
  GPU), which was out of scope for this pass (see
  `external_repos/holoocean-ros/docker/README.md` for what that requires).
  `HoloOceanRosBridgeSonarFrameProvider`'s conversion logic is the part that's
  actually tested, via its own unit tests
  (`tests/adapters/holoocean_ros_bridge_sonar_frame_provider_test.cpp`), which
  don't need ROS2 or the simulator at all.
- `holoocean_sonar_bridge_node` is a thin transport layer only — nothing
  downstream (`SonarFrontend::ProcessSonarFrame`) is wired to it yet; that's
  a separate integration this repo's own `sonar_cfar_frontend` hasn't had
  wired into `apps/replay_demo.cpp` either (see that app's `README`/source).
