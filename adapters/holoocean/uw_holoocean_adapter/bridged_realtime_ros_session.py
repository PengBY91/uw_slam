"""WSL2-side counterpart to `holoocean_bridge_sensor_host.py` (which runs on
the native Windows host that can actually launch real HoloOcean -- see
`raw_frame_wire.py`'s module docstring for why this two-host split exists
and what it deliberately avoids: installing ROS2/rclpy on Windows, or
bridging cross-host DDS discovery).

`BridgedRealtimeRosSession` plays the exact same role `RealtimeRosSession`
(`realtime_ros_session.py`) does -- pilot-command shaping, degradation/
fault-injector wiring, `build_realtime_messages()` -- except it sources each
tick's `RawSensorFrame` from a TCP socket connected to the Windows-side
sensor host instead of a local `HoloOceanSession`. Kept as a separate class
rather than a `RealtimeRosSession` subclass: `RealtimeRosSession.__init__`
unconditionally constructs a real `HoloOceanSession`, which would try (and
fail or hang, no Vulkan RT here) to launch HoloOcean locally in this WSL2
sandbox -- there is no clean partial-construction seam to reuse without
changing that already-tested class's contract.

`main()` below is the real ROS2/network entrypoint -- never unit tested
directly, same status as `realtime_ros_session.main()`/`RealtimeRosSession`
itself (needs a real socket peer and a sourced ROS2 install). The portable
`build_realtime_messages()` it drives is what actually carries this
module's test coverage, already exercised by `test_realtime_ros_session.py`.
"""
from __future__ import annotations

import argparse
import socket
import time
from typing import Any, List, Optional, Tuple

import numpy as np

from uw_holoocean_adapter.fault_injector import (
    FaultInjector,
    SensorDegradationSchedule,
    ThrusterFaultConfig,
    apply_thruster_fault,
    resolve_active_degradation,
)
from uw_holoocean_adapter.pilot_command_model import PilotCommandModel
from uw_holoocean_adapter.raw_frame_wire import recv_raw_sensor_frame, send_thruster_command
from uw_holoocean_adapter.realtime_ros_session import (
    _pilot_axes_to_thrusters,
    _validate_thruster_command,
    build_realtime_messages,
)
from uw_holoocean_adapter.ros_message_conversion import (
    HeadingNoise,
    RosMessageTypes,
    TopicMap,
    build_topic_map,
    thrust_fraction_of,
)
from uw_holoocean_adapter.scenario_manifest import RealtimeScenarioManifest, load_realtime_manifest
from uw_holoocean_adapter.scenario_randomization import SonarDegradation, VisualDegradation

_THRUSTER_COUNT = 8


class BridgedRealtimeRosSession:
    def __init__(
        self,
        manifest: RealtimeScenarioManifest,
        seed: int,
        rng: np.random.Generator,
        sock: socket.socket,
        *,
        fault_injector: Optional[FaultInjector] = None,
        thruster_fault: Optional[ThrusterFaultConfig] = None,
        perturbation_rng: Optional[np.random.Generator] = None,
        sensor_degradation_schedule: Optional[SensorDegradationSchedule] = None,
        visual_degradation_profile: Optional[VisualDegradation] = None,
        sonar_degradation_profile: Optional[SonarDegradation] = None,
    ):
        self._manifest = manifest
        self._topics: TopicMap = build_topic_map(manifest.agent_name, manifest.algorithm_topics)
        self._heading_noise = HeadingNoise.from_manifest_metadata(manifest.uw_metadata.get("heading_noise_model"))
        self._last_angular_velocity: Optional[np.ndarray] = None
        self._last_thrust_fraction = 0.0
        self._rng = rng
        self._sock = sock
        actuator = manifest.actuator_model
        self._pilot_command_model = PilotCommandModel(
            limit=actuator.limit, deadzone=actuator.deadzone, time_constant_s=actuator.time_constant_s
        )
        self._dt_s = 1.0 / manifest.ticks_per_sec
        self._last_thruster_command = [0.0] * _THRUSTER_COUNT
        self._fault_injector = fault_injector
        self._thruster_fault = thruster_fault
        self._perturbation_rng = perturbation_rng
        self._sonar_min_range_m = manifest.sensor("ImagingSonar").configuration.get("RangeMin")
        self._sonar_max_range_m = manifest.sensor("ImagingSonar").configuration.get("RangeMax")
        self._sensor_degradation_schedule = sensor_degradation_schedule
        self._visual_degradation_profile = visual_degradation_profile
        self._sonar_degradation_profile = sonar_degradation_profile

    def on_pilot_command(self, values) -> None:
        """Callback for the `/uw/pilot/command` subscription (PREP-C-02
        setpoint-level contract) -- identical to
        `RealtimeRosSession.on_pilot_command`."""
        allocated = _pilot_axes_to_thrusters(values, self._pilot_command_model.limit)
        if allocated is not None:
            self._last_thruster_command = allocated

    def on_thruster_command(self, values) -> None:
        """Callback for the legacy `/uw/pilot/thrusters` subscription -- identical
        contract to `RealtimeRosSession.on_thruster_command`: never directly
        touches HoloOcean, only updates the value `tick()` sends to the
        Windows host on its next call."""
        validated = _validate_thruster_command(values)
        if validated is not None:
            self._last_thruster_command = validated

    def set_socket(self, sock: socket.socket) -> None:
        """Swaps in a freshly-accepted socket after a reconnect -- see
        `main()`'s reconnect loop. This dev machine's WSL2<->Windows TCP
        path has been observed to drop long-lived connections mid-run
        (network/security-policy flakiness, not a pipeline bug -- confirmed
        against this exact bridge: 476 real ticks flowed correctly before
        one `ConnectionResetError`). Node/publishers/pilot-command-model
        state all stay alive across a reconnect; only the socket changes."""
        self._sock = sock

    def tick(
        self,
        message_types: RosMessageTypes,
        *,
        visual_degradation: Optional[VisualDegradation] = None,
        sonar_degradation: Optional[SonarDegradation] = None,
    ) -> List[Tuple[str, Any]]:
        shaped_command = self._pilot_command_model.step(self._last_thruster_command, self._dt_s)
        shaped_command = apply_thruster_fault(shaped_command, self._thruster_fault)
        # One request/response pair per tick: send the command the Windows
        # host should apply on its OWN next env.step() before waiting for
        # the frame that command produces -- keeps both hosts' understanding
        # of "which command produced this frame" in lockstep without a
        # separate acknowledgement message.
        self._last_thrust_fraction = thrust_fraction_of(shaped_command, self._pilot_command_model.limit)
        send_thruster_command(self._sock, shaped_command)
        frame = recv_raw_sensor_frame(self._sock)
        if "IMUSensor" in frame.sensors:
            self._last_angular_velocity = np.asarray(frame.sensors["IMUSensor"], dtype=float)[1].copy()
        visual_degradation, sonar_degradation = resolve_active_degradation(
            self._sensor_degradation_schedule,
            self._visual_degradation_profile,
            self._sonar_degradation_profile,
            frame.sim_time_s,
            baseline_visual=visual_degradation,
            baseline_sonar=sonar_degradation,
        )
        return build_realtime_messages(
            frame, self._topics, message_types, rng=self._rng,
            visual_degradation=visual_degradation,
            sonar_degradation=sonar_degradation,
            perturbation_rng=self._perturbation_rng,
            sonar_min_range_m=self._sonar_min_range_m,
            sonar_max_range_m=self._sonar_max_range_m,
            fault_injector=self._fault_injector,
            heading_noise=self._heading_noise,
            thrust_fraction=self._last_thrust_fraction,
            fallback_angular_velocity=self._last_angular_velocity,
        )

    def close(self) -> None:
        self._sock.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--task", required=True)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--bridge-host", default="0.0.0.0")
    parser.add_argument("--bridge-port", type=int, default=5599)
    parser.add_argument("--profile", default=None, help="fidelity | realtime (PREP-A-03 uw_profiles overlay)")
    parser.add_argument("--sonar-mode", default=None, help="sonar_1200khz | sonar_750khz")
    parser.add_argument("--fault-profile", choices=("none", "critical"), default="none")
    parser.add_argument("--fault-seed", type=int, default=None)
    parser.add_argument("--fault-duration-s", type=float, default=None)
    parser.add_argument("--visual-degradation", choices=("clear", "critical"), default="clear")
    parser.add_argument("--sonar-degradation", choices=("clear", "critical"), default="clear")
    parser.add_argument("--thruster-fault-index", type=int, default=None)
    parser.add_argument("--thruster-fault-effectiveness", type=float, default=1.0)
    parser.add_argument("--sensor-fault-schedule", choices=("none", "scheduled"), default="none")
    parser.add_argument("--visual-fault-window-count", type=int, default=1)
    parser.add_argument("--visual-fault-window-duration-s", type=float, default=30.0)
    parser.add_argument("--sonar-fault-window-count", type=int, default=1)
    parser.add_argument("--sonar-fault-window-duration-s", type=float, default=30.0)
    args = parser.parse_args()

    import rclpy  # noqa: E402  (lazy: no rclpy install outside a sourced ROS2 distro)
    from rclpy.node import Node  # noqa: E402
    from rclpy.qos import qos_profile_sensor_data  # noqa: E402
    from nav_msgs.msg import Odometry  # noqa: E402
    from rosgraph_msgs.msg import Clock  # noqa: E402
    from sensor_msgs.msg import Image, Imu  # noqa: E402
    from std_msgs.msg import Float32MultiArray  # noqa: E402
    from holoocean_interfaces.msg import ImagingSonar  # noqa: E402

    from uw_holoocean_adapter.fault_injector import build_fault_schedule, build_sensor_degradation_schedule
    from uw_holoocean_adapter.scenario_randomization import PRESET_CRITICAL_DEGRADED

    manifest = load_realtime_manifest(args.scenario, args.task, profile=args.profile, sonar_mode=args.sonar_mode)
    rng = np.random.default_rng(args.seed)
    perturbation_rng = np.random.default_rng(args.seed + 1)
    topics = build_topic_map(manifest.agent_name, manifest.algorithm_topics)

    injector: Optional[FaultInjector] = None
    if args.fault_profile == "critical":
        from uw_holoocean_adapter.realtime_ros_session import _build_critical_fault_profile

        fault_seed = args.fault_seed if args.fault_seed is not None else args.seed
        duration_s = args.fault_duration_s if args.fault_duration_s is not None else manifest.task.max_duration_s
        profile = _build_critical_fault_profile(topics)
        schedule = build_fault_schedule(fault_seed, profile, duration_s)
        injector = FaultInjector(schedule, profile, np.random.default_rng(fault_seed))

    thruster_fault = None
    if args.thruster_fault_index is not None:
        thruster_fault = ThrusterFaultConfig(
            thruster_index=args.thruster_fault_index,
            effectiveness_multiplier=args.thruster_fault_effectiveness,
        )

    visual_degradation = PRESET_CRITICAL_DEGRADED.visual if args.visual_degradation == "critical" else None
    sonar_degradation = PRESET_CRITICAL_DEGRADED.sonar if args.sonar_degradation == "critical" else None

    sensor_degradation_schedule = None
    visual_degradation_profile = None
    sonar_degradation_profile = None
    if args.sensor_fault_schedule == "scheduled":
        fault_seed = args.fault_seed if args.fault_seed is not None else args.seed
        duration_s = args.fault_duration_s if args.fault_duration_s is not None else manifest.task.max_duration_s
        visual_degradation_profile = visual_degradation
        sonar_degradation_profile = sonar_degradation
        sensor_degradation_schedule = build_sensor_degradation_schedule(
            fault_seed, duration_s,
            visual_window_count=args.visual_fault_window_count if visual_degradation_profile is not None else 0,
            visual_window_duration_s=args.visual_fault_window_duration_s,
            sonar_window_count=args.sonar_fault_window_count if sonar_degradation_profile is not None else 0,
            sonar_window_duration_s=args.sonar_fault_window_duration_s,
        )
        visual_degradation = None
        sonar_degradation = None

    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind((args.bridge_host, args.bridge_port))
    listener.listen(1)
    print(
        f"bridged_realtime_ros_session: listening on {args.bridge_host}:{args.bridge_port} -- "
        "start holoocean_bridge_sensor_host.py on the Windows host now (it connects out to "
        "this port)."
    )
    sock, peer = listener.accept()
    print(f"bridged_realtime_ros_session: sensor host connected from {peer}")

    session = BridgedRealtimeRosSession(
        manifest, args.seed, rng, sock,
        fault_injector=injector, thruster_fault=thruster_fault, perturbation_rng=perturbation_rng,
        sensor_degradation_schedule=sensor_degradation_schedule,
        visual_degradation_profile=visual_degradation_profile,
        sonar_degradation_profile=sonar_degradation_profile,
    )
    message_types = RosMessageTypes(Image=Image, Odometry=Odometry, ImagingSonar=ImagingSonar, Clock=Clock, Imu=Imu)

    rclpy.init()
    # Deliberately NOT "uw_holoocean_realtime_gateway" -- that name is
    # already used by the C++ adapters/ros2/src/holoocean_realtime_node.cpp
    # gateway this session's topics feed into (realtime_ros_session.py's
    # own main() has the same collision -- a pre-existing issue, not
    # something this bridging work should silently carry into new code).
    node = Node("uw_holoocean_bridged_sensor_session")
    from uw_holoocean_adapter.realtime_ros_session import build_publisher_table

    publishers = {
        topic: node.create_publisher(getattr(message_types, attr), topic, qos_profile_sensor_data)
        for topic, attr in build_publisher_table(manifest, topics)
    }
    node.create_subscription(
        Float32MultiArray, topics.pilot_command, lambda msg: session.on_pilot_command(msg.data),
        qos_profile_sensor_data,
    )
    node.create_subscription(
        Float32MultiArray, topics.pilot_thrusters, lambda msg: session.on_thruster_command(msg.data),
        qos_profile_sensor_data,
    )

    try:
        # See realtime_ros_session.main()'s identical comment: NOT
        # node.create_rate(...).sleep() -- confirmed against a real sourced
        # ROS2 install that it deadlocks in this single-threaded loop shape.
        dt_s = 1.0 / manifest.ticks_per_sec
        # Reconnect loop: this dev machine's WSL2<->Windows TCP path has
        # been observed to drop long-lived connections mid-run (confirmed
        # directly against this bridge -- 476 real ticks flowed correctly,
        # then one ConnectionResetError; network/security-policy flakiness,
        # not a pipeline bug). session/node/publishers/subscriptions all
        # stay alive across a reconnect -- only the socket is replaced.
        while rclpy.ok():
            try:
                while rclpy.ok():
                    for topic, message in session.tick(
                        message_types, visual_degradation=visual_degradation, sonar_degradation=sonar_degradation
                    ):
                        publishers[topic].publish(message)
                    rclpy.spin_once(node, timeout_sec=0.0)
                    time.sleep(dt_s)
            except (ConnectionError, OSError) as error:
                print(f"bridged_realtime_ros_session: connection dropped ({error!r}), waiting for a new one")
                sock.close()
                sock, peer = listener.accept()
                print(f"bridged_realtime_ros_session: sensor host reconnected from {peer}")
                session.set_socket(sock)
    finally:
        session.close()
        listener.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
