# SVIn `LocalOdometryProvider` bridge

Per platform architecture section 19 ("SVIn: 第一阶段 LocalOdometryProvider adapter") and section
22.5: this wraps SVIn's ROS2 `okvis_odometry` topic (`nav_msgs/Odometry`) as black-box relative-pose
evidence. `SvinBridgeLocalOdometryProvider` (see
[`include/uw/adapters/svin_bridge_local_odometry_provider.hpp`](./include/uw/adapters/svin_bridge_local_odometry_provider.hpp))
is unit-testable everywhere via its `PushRelativePoseEvidence()` injection point; the actual ROS2
subscription is isolated to `adapters/ros2` (built only when `UW_BUILD_ROS2=ON`).

## Known blocker

This machine has no sourced ROS2 install, so the ROS2 subscription side has never been compiled or
run against a real SVIn process — only the injection-point unit test
(`test/svin_bridge_test.cpp`) has been verified.

## A working reference for closing that gap

A colleague's separate deployment package (`workfiles_02`, outside this repo) runs SVIn under ROS2
Humble end-to-end against HoloOcean (`ros2 launch holoocean_bridge pipeline.launch.py`), including a
working `eval_svin_holoocean.py` for checking `okvis_odometry` output against ground truth. On a
machine with ROS2 available, it's a ready-made way to sanity-check this bridge's field mapping
(pose → `RelativePoseMeasurement`, the covariance-proxy fallback documented in the header — SVIn's
`okvis_odometry` doesn't populate usable pose covariance, platform architecture section 22.4) against
real SVIn output before trusting it on this platform's own data. It is **not** vendored into this
repo (separate license/provenance chain), so treat it as something to run and compare against, not
code to copy in wholesale.
