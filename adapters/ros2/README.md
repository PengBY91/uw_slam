# `adapters/ros2`

The only place ROS2 headers may appear in this repository (platform
architecture section 5 invariant #4). Built only with `-DUW_BUILD_ROS2=ON`
on a machine with a sourced ROS2 install — see the top-level `CMakeLists.txt`.

| Target | Wraps | Depends on (beyond rclcpp) | Status |
|---|---|---|---|
| `uw::ros2_adapters` | portable `include/adapters/` providers (SVIn `LocalOdometryProvider`, HoloOcean `SonarFrameProvider`, see `adapters/svin_bridge.md`/`adapters/holoocean_ros_bridge.md`) | `nav_msgs`, `sensor_msgs`, `std_msgs`, `rosgraph_msgs`, `holoocean_interfaces` | The SVIn side (`ros2_svin_odometry_bridge.hpp`) is header content fully commented out — documented skeleton, never compiled (see the header's doc comment). |
| `holoocean_sonar_bridge_node` | `include/adapters/holoocean_ros_bridge_sonar_frame_provider.hpp` via `uw::ros2_adapters` | `holoocean_interfaces` (from `external_repos/holoocean-ros/holoocean_interfaces`, colcon-built) | Real `rclcpp::Node`, compiled and run against ROS2 Jazzy on this machine — see below for exactly what was/wasn't verified. |
| `holoocean_realtime_node` | `include/adapters/holoocean_live_conversion.hpp` (portable raw-HoloOcean→`uw::domain` conversion) + `include/adapters/holoocean_realtime_sink.hpp` (dependency-inversion seam into the real `LiveEventSource`/`OnlineAssistPipeline` machinery, implemented at `application` role in `src/application/holoocean_realtime_sink.cpp` — see that header's doc comment for why the seam exists: the `ros2` role may only depend on `{adapters, measurement_api, sensor_models, domain, domain_proto}`, so this ROS-header-including translation unit never itself includes `application`/`runtime`/`opencv_adapters`) | `nav_msgs`, `sensor_msgs`, `std_msgs`, `rosgraph_msgs`, `holoocean_interfaces` | Real `rclcpp::Node`, compiled and linked against a real `rclcpp`/`holoocean_interfaces` on this machine — see below. |

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
- `holoocean_realtime_node` compiles and links against a real
  `rclcpp`/`nav_msgs`/`sensor_msgs`/`std_msgs`/`rosgraph_msgs`/
  `holoocean_interfaces` on this machine — verified with:
  ```bash
  export PATH="$HOME/miniconda3/envs/uw_slam_build/bin:$PATH"   # conda's cmake (4.x), needed for the configure step
  source /opt/ros/jazzy/setup.bash
  source ~/ros2_ws/install/setup.bash
  cmake -S . -B build_ros2 -DUW_BUILD_ROS2=ON -DUW_BUILD_TESTS=ON \
    -DCMAKE_PREFIX_PATH="$HOME/miniconda3/envs/uw_slam_build;$AMENT_PREFIX_PATH"
  cmake --build build_ros2 --target holoocean_realtime_node -j"$(nproc)"
  ```
  Its portable half (`ConvertHoloImage`/`ConvertHoloSonar`/`ConvertHoloVehicleState`, unit-tested via
  `tests/adapters/holoocean_live_conversion_test.cpp`, and the pipeline-owning
  `MakeOnlineAssistRealtimeSink` in `src/application/holoocean_realtime_sink.cpp`, exercised
  indirectly through `application_tests`' `OnlineAssistPipeline` coverage) needs neither
  ROS2 nor a real HoloOcean install, matching `holoocean_sonar_bridge_node`'s own split.
- It has **not** been run against a live Task 3 `realtime_ros_session.py` process publishing
  real HoloOcean sensor topics — that needs the actual HoloOcean simulator (native-Windows-only
  in this repo's current setup, see the HoloOcean deployment notes), which was out of scope for
  this pass. The four algorithm-input subscriptions (`LeftCamera`/`RightCamera`/`ImagingSonar`/
  `VehicleState`), the independent `PilotCamera` presentation path, and the
  `/uw/hmi/overlay`/`/uw/hmi/status` publishers all follow the exact wiring
  `apps/online_assist_smoke.cpp` already exercises end-to-end against synthetic data (real
  `OnlineAssistPipeline`, real `LiveEventSource`, real `PumpEvents` thread) — only the ROS2
  transport at the edges (subscribing real topics instead of a synthetic generator, publishing
  real topics instead of a `ReportSink`) is new and untested against a live simulator here.
- Sonar calibration (`HoloOceanSonarCalibration` — FOV/range/sound-speed/operating-frequency) is
  read from ROS2 node parameters (`declare_parameter`, same pattern
  `Ros2HoloOceanSonarBridge` already uses for `input_topic`), defaulted to mirror Task 1's
  `adapters/holoocean/scenarios/blue_rov_aid_sv1213_base.json` manifest exactly so an
  unconfigured launch already matches the BlueROV/AI-D/SV1213 profile — a real launch file only
  needs to override these if a different scenario manifest is in use. There is still no automatic
  single-source-of-truth link between the Python manifest and these C++ parameter defaults (they
  can drift if one is edited without the other) — that would need either a generated launch file
  or a shared config format, out of this task's scope (topic/conversion/overlay wiring only).
  The rig (`BuildIdentityRig`) stays a placeholder identity-extrinsic rig — real calibrated
  intrinsics/extrinsics are not carried by Task 1's manifest in a directly-usable form.
