# HoloOcean ROS 2 `ImagingSonar` bridge

Per platform architecture section 19/22.5's adapter pattern (same shape as
`adapters/third_party/svin_bridge`): wraps HoloOcean's official ROS 2
interface (`external_repos/holoocean-ros`, byu-holoocean/holoocean-ros, MIT)
as a `SonarFrameProvider` (see `core/measurement_api/include/uw/measurement_api/providers.hpp`),
converting `holoocean_interfaces/msg/ImagingSonar` into this platform's native
`uw::domain::SonarFrame` — NOT into `sonar_oculus/OculusPing`, which is what
the external baseline path (`adapters/third_party/sonar_camera_reconstruction_baseline`)
consumes instead.

`HoloOceanRosBridgeSonarFrameProvider` (see
[`include/uw/adapters/holoocean_ros_bridge_sonar_frame_provider.hpp`](./include/uw/adapters/holoocean_ros_bridge_sonar_frame_provider.hpp))
does the actual field conversion and is unit-testable everywhere via its
`PushImagingSonar()` injection point; the real ROS2 subscription is isolated
to `adapters/ros2` (built only when `UW_BUILD_ROS2=ON`).

## Two non-obvious conversion details

`ImagingSonar.msg` has no per-beam angle array or range calibration, and its
`image_range`/`image_azimuth` field split is not self-explanatory from the
`.msg` file alone. Both were resolved by reading
`external_repos/holoocean_bridge/holoocean_bridge/sonar_adapter_node.py` — a
colleague's separate ROS2 bridge package (also dropped into `external_repos/`,
**not vendored into this repo**, different provenance chain) that already
converts the same topic to `sonar_oculus/OculusPing` and has been run against
real HoloOcean output:

- `image_range` is the flattened row-major `[num_ranges, num_beams]`
  intensity image, float32 in `[0, 1]`; `image_azimuth` is unused by that
  reference adapter and is likewise ignored here.
- HoloOcean's column order runs opposite this platform's ascending-bearing
  convention (`uw::domain::IsAzimuthAscending`), so each row is mirrored when
  built into `SonarFrame` — the same correction the reference adapter applies
  via `np.fliplr`.
- Horizontal FOV and min/max range are not in the message; both adapters read
  them from the scenario JSON's `agents[0].sensors[].configuration.{Azimuth,
  RangeMin,RangeMax}` instead (`HoloOceanImagingSonarParams` here, ROS2 node
  parameters in the reference).

## Known blocker

This machine had no sourced ROS2 install until it was set up alongside this
adapter (see `external_repos/holoocean-ros/README.md` for prerequisites); the
provider class above has been unit-tested, but the ROS2 subscription side
(`adapters/ros2`) depends on `holoocean_interfaces` being colcon-built and on
`CMAKE_PREFIX_PATH`/`AMENT_PREFIX_PATH`, which is a heavier, separate
environment step — see `adapters/ros2/README.md` for status.
