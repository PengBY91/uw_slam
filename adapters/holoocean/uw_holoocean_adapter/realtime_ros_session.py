"""Runs a real HoloOcean session at its manifest tick rate, converts each
tick's sensor-only output into ROS2 messages, and publishes them —
`record_session.py`'s realtime counterpart, publishing live topics instead
of writing an MCAP bag. Also applies `/uw/pilot/thrusters` commands through
`PilotCommandModel` before stepping the session.

Follows `record_session.py`'s split exactly: `build_realtime_messages`
below is the portable, fully unit-testable core (no rclpy or HoloOcean
dependency — it only consumes an already-obtained `RawSensorFrame`); the
real ROS2 node class at the bottom is a thin wrapper only reachable through
`main()`, with `rclpy` imported lazily inside it (same pattern as
`holoocean_driver.py`'s `_import_holoocean()`), since this machine has
neither rclpy nor a HoloOcean install.
"""
from __future__ import annotations

import argparse
from typing import Any, List, Tuple

import numpy as np

from uw_holoocean_adapter.holoocean_driver import HoloOceanSession, RawSensorFrame
from uw_holoocean_adapter.pilot_command_model import PilotCommandModel
from uw_holoocean_adapter.scenario_randomization import ScenarioRandomization
from uw_holoocean_adapter.ros_message_conversion import (
    RosMessageTypes,
    StateNoise,
    TopicMap,
    build_topic_map,
    holoocean_camera_to_ros_image,
    holoocean_sonar_to_imaging_sonar_msg,
    sim_time_to_clock_msg,
    vehicle_state_to_odometry,
)
from uw_holoocean_adapter.scenario_manifest import RealtimeScenarioManifest, load_realtime_manifest

_LEFT_CAMERA_KEY = "LeftCamera"
_RIGHT_CAMERA_KEY = "RightCamera"
_PILOT_CAMERA_KEY = "PilotCamera"
_SONAR_KEY = "ImagingSonar"
_ORIENTATION_KEY = "VehicleOrientation"
_IMU_KEY = "IMUSensor"
_DEPTH_KEY = "DepthSensor"

_THRUSTER_COUNT = 8

_DEFAULT_STATE_NOISE = StateNoise(orientation_sigma=0.01, depth_sigma=0.02)


def build_realtime_messages(
    frame: RawSensorFrame,
    topics: TopicMap,
    message_types: RosMessageTypes,
    *,
    state_noise: StateNoise = _DEFAULT_STATE_NOISE,
    rng: np.random.Generator,
) -> List[Tuple[str, Any]]:
    """Converts one HoloOcean tick's `RawSensorFrame` into the (topic,
    message) pairs that tick should publish. Only builds a message for a
    sensor that actually appears in `frame.sensors` this tick — HoloOcean
    publishes each sensor at its own configured rate (see
    `holoocean_driver.py`'s `RawSensorFrame` docstring and
    `record_session.py`'s identical independent-rate handling), so a tick
    where only the 50 Hz state sensors fired yields no camera/sonar
    messages, and vice versa. `/clock` is published every tick
    unconditionally (it tracks simulation time itself, not any one
    sensor)."""
    sensors = frame.sensors
    messages: List[Tuple[str, Any]] = []

    if _LEFT_CAMERA_KEY in sensors:
        messages.append((
            topics.left_camera,
            holoocean_camera_to_ros_image(
                np.asarray(sensors[_LEFT_CAMERA_KEY]),
                capture_time_s=frame.sim_time_s,
                frame_id="camera_left_link",
                message_types=message_types,
            ),
        ))
    if _RIGHT_CAMERA_KEY in sensors:
        messages.append((
            topics.right_camera,
            holoocean_camera_to_ros_image(
                np.asarray(sensors[_RIGHT_CAMERA_KEY]),
                capture_time_s=frame.sim_time_s,
                frame_id="camera_right_link",
                message_types=message_types,
            ),
        ))
    if _PILOT_CAMERA_KEY in sensors:
        messages.append((
            topics.pilot_camera,
            holoocean_camera_to_ros_image(
                np.asarray(sensors[_PILOT_CAMERA_KEY]),
                capture_time_s=frame.sim_time_s,
                frame_id="camera_pilot_link",
                message_types=message_types,
            ),
        ))
    if _SONAR_KEY in sensors:
        messages.append((
            topics.imaging_sonar,
            holoocean_sonar_to_imaging_sonar_msg(
                np.asarray(sensors[_SONAR_KEY]),
                capture_time_ns=int(frame.sim_time_s * 1e9),
                message_types=message_types,
            ),
        ))
    if _ORIENTATION_KEY in sensors and _IMU_KEY in sensors and _DEPTH_KEY in sensors:
        imu = np.asarray(sensors[_IMU_KEY])
        depth = np.asarray(sensors[_DEPTH_KEY]).reshape(-1)
        messages.append((
            topics.vehicle_state,
            vehicle_state_to_odometry(
                np.asarray(sensors[_ORIENTATION_KEY]),
                imu[1],
                float(depth[0]),
                frame.sim_time_s,
                state_noise,
                rng,
                message_types,
            ),
        ))

    messages.append((topics.clock, sim_time_to_clock_msg(frame.sim_time_s, message_types)))
    return messages


def _validate_thruster_command(values) -> List[float] | None:
    """`/uw/pilot/thrusters` must carry exactly `_THRUSTER_COUNT` floats —
    any other length is rejected (logged, not applied) rather than crashing
    the realtime loop or silently padding/truncating it, per the plan's
    'Reject any other length' requirement."""
    values = list(values)
    if len(values) != _THRUSTER_COUNT:
        return None
    return [float(v) for v in values]


class RealtimeRosSession:
    """Thin real-ROS2/HoloOcean wrapper — only reachable through `main()`,
    never unit tested directly (same status as `HoloOceanSession` itself
    and `record_session.py`'s `record_session()`: this needs a real
    HoloOcean+ROS2 host, exercised only by the plan's own Step 4 native-host
    command). `build_realtime_messages` above is the tested core this
    class simply drives."""

    def __init__(self, manifest: RealtimeScenarioManifest, seed: int, rng: np.random.Generator):
        self._manifest = manifest
        self._topics = build_topic_map(manifest.agent_name)
        self._rng = rng
        self._session = HoloOceanSession(
            manifest.holoocean_scenario_cfg(), seed, randomization=ScenarioRandomization()
        )
        for prop in manifest.task.props:
            self._session.spawn_prop(
                prop.prop_type,
                location=list(prop.location_m),
                material=prop.visual_material,
                tag=prop.tag,
            )
        actuator = manifest.actuator_model
        self._pilot_command_model = PilotCommandModel(
            limit=actuator.limit, deadzone=actuator.deadzone, time_constant_s=actuator.time_constant_s
        )
        self._dt_s = 1.0 / manifest.ticks_per_sec
        self._last_thruster_command = [0.0] * _THRUSTER_COUNT

    def on_thruster_command(self, values) -> None:
        """Callback for the `/uw/pilot/thrusters` subscription. Never
        directly moves/teleports the vehicle — the filtered command only
        ever reaches HoloOcean through `step()` below, on the next tick."""
        validated = _validate_thruster_command(values)
        if validated is not None:
            self._last_thruster_command = validated

    def tick(self, message_types: RosMessageTypes) -> List[Tuple[str, Any]]:
        shaped_command = self._pilot_command_model.step(self._last_thruster_command, self._dt_s)
        frame = self._session.step(shaped_command)
        return build_realtime_messages(frame, self._topics, message_types, rng=self._rng)

    def close(self) -> None:
        self._session.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--task", required=True)
    parser.add_argument("--seed", type=int, default=42)
    # A separate process (a fixed-step task control script, ScriptedPilot,
    # or an interactive pilot station) publishes /uw/pilot/thrusters — this
    # gateway only subscribes it, per the plan: "A deterministic task
    # control script may publish this topic for unattended gates, while a
    # pilot station can publish the same topic for interactive runs." This
    # module has no --pilot flag because it never runs one itself.
    args = parser.parse_args()

    import rclpy  # noqa: E402  (lazy: no rclpy install outside a sourced ROS2 distro)
    from rclpy.node import Node  # noqa: E402
    from rclpy.qos import qos_profile_sensor_data  # noqa: E402
    from nav_msgs.msg import Odometry  # noqa: E402
    from rosgraph_msgs.msg import Clock  # noqa: E402
    from sensor_msgs.msg import Image  # noqa: E402
    from std_msgs.msg import Float32MultiArray  # noqa: E402
    from holoocean_interfaces.msg import ImagingSonar  # noqa: E402

    manifest = load_realtime_manifest(args.scenario, args.task)
    rng = np.random.default_rng(args.seed)
    session = RealtimeRosSession(manifest, args.seed, rng)
    message_types = RosMessageTypes(Image=Image, Odometry=Odometry, ImagingSonar=ImagingSonar, Clock=Clock)

    rclpy.init()
    node = Node("uw_holoocean_realtime_gateway")
    topics = build_topic_map(manifest.agent_name)
    publishers = {
        topic: node.create_publisher(getattr(message_types, attr), topic, qos_profile_sensor_data)
        for topic, attr in (
            (topics.left_camera, "Image"),
            (topics.right_camera, "Image"),
            (topics.pilot_camera, "Image"),
            (topics.imaging_sonar, "ImagingSonar"),
            (topics.vehicle_state, "Odometry"),
            (topics.clock, "Clock"),
        )
    }
    node.create_subscription(
        Float32MultiArray, "/uw/pilot/thrusters", lambda msg: session.on_thruster_command(msg.data),
        qos_profile_sensor_data,
    )

    try:
        rate = node.create_rate(manifest.ticks_per_sec)
        while rclpy.ok():
            for topic, message in session.tick(message_types):
                publishers[topic].publish(message)
            rclpy.spin_once(node, timeout_sec=0.0)
            rate.sleep()
    finally:
        session.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
