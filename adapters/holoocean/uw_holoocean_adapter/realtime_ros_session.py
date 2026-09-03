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

Task 5 additions: `build_realtime_messages` optionally perturbs camera/sonar
arrays (`sensor_perturbation.py`) before conversion and/or runs the
resulting (topic, message) pairs through a `FaultInjector`
(`fault_injector.py`) — both default to `None`/no-op, so every call site
that predates Task 5 (including this module's own 33 pre-existing tests)
keeps its exact original behavior. `RealtimeRosSession` similarly grows
optional `randomization`/`fault_injector`/`thruster_fault` constructor
parameters, all defaulted to today's no-fault behavior. Ocean-current fault
delivery is NOT wired here despite the plan text mentioning
`set_ocean_currents` under this task — `HoloOceanSession.apply_randomization()`
(Task 2) is the one sanctioned way to reach it, but `ScenarioRandomization`
has no current-velocity field to update, and adding one is a Task 2 file
change out of this task's scope; left as a documented gap rather than
reaching into `HoloOceanSession`'s private `_env` to bypass it.
"""
from __future__ import annotations

import argparse
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
from uw_holoocean_adapter.holoocean_driver import HoloOceanSession, RawSensorFrame
from uw_holoocean_adapter.pilot_command_model import PilotCommandModel
from uw_holoocean_adapter.scenario_randomization import ScenarioRandomization, SonarDegradation, VisualDegradation
from uw_holoocean_adapter.sensor_perturbation import perturb_sonar, perturb_stereo_pair
from uw_holoocean_adapter.coordinates import pose_sensor_to_pose
from uw_holoocean_adapter.ros_message_conversion import (
    RosMessageTypes,
    StateNoise,
    TopicMap,
    build_topic_map,
    holoocean_camera_to_ros_image,
    holoocean_sonar_to_imaging_sonar_msg,
    sim_time_to_clock_msg,
    truth_pose_to_odometry,
    vehicle_state_to_odometry,
)
from uw_holoocean_adapter.scenario_manifest import RealtimeScenarioManifest, load_realtime_manifest
from uw_holoocean_adapter.thrust_allocation import allocate, parse_pilot_axes

_LEFT_CAMERA_KEY = "LeftCamera"
_RIGHT_CAMERA_KEY = "RightCamera"
_PILOT_CAMERA_KEY = "PilotCamera"
_SONAR_KEY = "ImagingSonar"
_ORIENTATION_KEY = "VehicleOrientation"
_IMU_KEY = "IMUSensor"
_DEPTH_KEY = "DepthSensor"
_POSE_SENSOR_KEY = "PoseSensor"

_THRUSTER_COUNT = 8

_DEFAULT_STATE_NOISE = StateNoise(orientation_sigma=0.01, depth_sigma=0.02)


def build_realtime_messages(
    frame: RawSensorFrame,
    topics: TopicMap,
    message_types: RosMessageTypes,
    *,
    state_noise: StateNoise = _DEFAULT_STATE_NOISE,
    rng: np.random.Generator,
    visual_degradation: Optional[VisualDegradation] = None,
    sonar_degradation: Optional[SonarDegradation] = None,
    perturbation_rng: Optional[np.random.Generator] = None,
    sonar_min_range_m: Optional[float] = None,
    sonar_max_range_m: Optional[float] = None,
    fault_injector: Optional[FaultInjector] = None,
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
    sensor).

    `visual_degradation`/`sonar_degradation` (Task 5's `sensor_perturbation.py`)
    and `fault_injector` (Task 5's `fault_injector.py`) are all optional and
    default to `None` — omitting them reproduces this function's exact
    pre-Task-5 behavior. Supplying `visual_degradation`/`sonar_degradation`
    without a `perturbation_rng` (or supplying `sonar_degradation` without
    both range bounds) is a caller error, not a silent no-op — raises
    `ValueError`, since a perturbation config the caller explicitly asked
    for silently not being applied would be worse than failing loudly."""
    if visual_degradation is not None and perturbation_rng is None:
        raise ValueError("visual_degradation requires perturbation_rng")
    if sonar_degradation is not None and (
        perturbation_rng is None or sonar_min_range_m is None or sonar_max_range_m is None
    ):
        raise ValueError("sonar_degradation requires perturbation_rng and sonar_min_range_m/sonar_max_range_m")

    sensors = frame.sensors
    messages: List[Tuple[str, Any]] = []

    left_pixels = np.asarray(sensors[_LEFT_CAMERA_KEY]) if _LEFT_CAMERA_KEY in sensors else None
    right_pixels = np.asarray(sensors[_RIGHT_CAMERA_KEY]) if _RIGHT_CAMERA_KEY in sensors else None
    if visual_degradation is not None and left_pixels is not None and right_pixels is not None:
        left_pixels, right_pixels, _active = perturb_stereo_pair(
            perturbation_rng, visual_degradation, left_pixels, right_pixels
        )

    if left_pixels is not None:
        messages.append((
            topics.left_camera,
            holoocean_camera_to_ros_image(
                left_pixels,
                capture_time_s=frame.sim_time_s,
                frame_id="camera_left_link",
                message_types=message_types,
            ),
        ))
    if right_pixels is not None:
        messages.append((
            topics.right_camera,
            holoocean_camera_to_ros_image(
                right_pixels,
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
        sonar_intensity = np.asarray(sensors[_SONAR_KEY])
        if sonar_degradation is not None:
            sonar_intensity, _active = perturb_sonar(
                perturbation_rng, sonar_degradation, sonar_intensity,
                min_range_m=sonar_min_range_m, max_range_m=sonar_max_range_m,
            )
        messages.append((
            topics.imaging_sonar,
            holoocean_sonar_to_imaging_sonar_msg(
                sonar_intensity,
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

    if fault_injector is not None:
        messages = fault_injector.apply(frame.sim_time_s, messages)

    # Ground truth is appended AFTER fault injection, not before -- it must
    # never be dropped/duplicated/reordered/delayed like an algorithm-facing
    # topic, and no `per_topic` fault profile should even be able to target
    # it by construction (SIM-ARCH-002/SYS-ARCH-003: only TaskScorer may see
    # this topic, and only as genuine, undistorted truth).
    if _POSE_SENSOR_KEY in sensors:
        truth_pose = pose_sensor_to_pose(np.asarray(sensors[_POSE_SENSOR_KEY]))
        messages.append((
            topics.scoring_truth,
            truth_pose_to_odometry(truth_pose, frame.sim_time_s, message_types),
        ))

    return messages


def _pilot_axes_to_thrusters(values, limit: float) -> List[float] | None:
    """`/uw/pilot/command` (PREP-C-02 setpoint-level contract: exactly four
    floats [surge, sway, heave, yaw_rate] in [-1, 1]) allocated to the 8
    HoloOcean thruster forces this backend actually drives. Any other length
    is rejected (None), never padded."""
    axes = parse_pilot_axes(values)
    if axes is None:
        return None
    return allocate(axes, limit=limit)


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

    def __init__(
        self,
        manifest: RealtimeScenarioManifest,
        seed: int,
        rng: np.random.Generator,
        *,
        randomization: Optional[ScenarioRandomization] = None,
        fault_injector: Optional[FaultInjector] = None,
        thruster_fault: Optional[ThrusterFaultConfig] = None,
        perturbation_rng: Optional[np.random.Generator] = None,
        sensor_degradation_schedule: Optional[SensorDegradationSchedule] = None,
        visual_degradation_profile: Optional[VisualDegradation] = None,
        sonar_degradation_profile: Optional[SonarDegradation] = None,
    ):
        self._manifest = manifest
        self._topics = build_topic_map(manifest.agent_name)
        self._rng = rng
        self._session = HoloOceanSession(
            manifest.holoocean_scenario_cfg(), seed,
            randomization=randomization if randomization is not None else ScenarioRandomization(),
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
        self._fault_injector = fault_injector
        self._thruster_fault = thruster_fault
        self._perturbation_rng = perturbation_rng
        self._sonar_min_range_m = manifest.sensor("ImagingSonar").configuration.get("RangeMin")
        self._sonar_max_range_m = manifest.sensor("ImagingSonar").configuration.get("RangeMax")
        self._sensor_degradation_schedule = sensor_degradation_schedule
        self._visual_degradation_profile = visual_degradation_profile
        self._sonar_degradation_profile = sonar_degradation_profile

    def on_pilot_command(self, values) -> None:
        """Callback for the `/uw/pilot/command` subscription -- the
        setpoint-level contract (PREP-C-02). Allocated to thrusters here,
        inside the simulation backend, then shaped by PilotCommandModel and
        applied on the next `step()` exactly like the legacy thruster topic."""
        allocated = _pilot_axes_to_thrusters(values, self._pilot_command_model.limit)
        if allocated is not None:
            self._last_thruster_command = allocated

    def on_thruster_command(self, values) -> None:
        """Callback for the legacy sim-internal `/uw/pilot/thrusters`
        subscription (8 raw forces). Never directly moves/teleports the
        vehicle — the filtered command only ever reaches HoloOcean through
        `step()` below, on the next tick."""
        validated = _validate_thruster_command(values)
        if validated is not None:
            self._last_thruster_command = validated

    def tick(
        self,
        message_types: RosMessageTypes,
        *,
        visual_degradation: Optional[VisualDegradation] = None,
        sonar_degradation: Optional[SonarDegradation] = None,
    ) -> List[Tuple[str, Any]]:
        shaped_command = self._pilot_command_model.step(self._last_thruster_command, self._dt_s)
        shaped_command = apply_thruster_fault(shaped_command, self._thruster_fault)
        frame = self._session.step(shaped_command)
        # resolve_active_degradation is the ONLY scheduling decision made
        # here (fully unit-tested in isolation, see test_fault_injector.py)
        # -- this class itself is real-HoloOcean-only and never unit tested
        # directly. With no schedule configured it returns
        # (visual_degradation, sonar_degradation) unchanged, so this is a
        # no-op for every pre-B4 caller.
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
        )

    def close(self) -> None:
        self._session.close()


def _build_critical_fault_profile(topics: TopicMap) -> "FaultInjectionProfile":
    from uw_holoocean_adapter.fault_injector import FaultInjectionProfile, TopicFaultConfig

    return FaultInjectionProfile(
        per_topic={
            topic: TopicFaultConfig(
                clock_offset_s=0.01, jitter_sigma_s=0.005, drop_probability=0.05,
                duplicate_probability=0.02, reorder_probability=0.03, reorder_max_distance=2,
                outage_count=1, outage_duration_s=2.0,
            )
            for topic in topics.algorithm_inputs
        },
        check_interval_s=1.0,
    )


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
    #
    # All flags below are additive Task 5 options, every one defaulted to
    # today's no-fault/no-perturbation behavior.
    parser.add_argument("--fault-profile", choices=("none", "critical"), default="none")
    parser.add_argument("--fault-seed", type=int, default=None)
    parser.add_argument("--fault-duration-s", type=float, default=None)
    parser.add_argument("--visual-degradation", choices=("clear", "critical"), default="clear")
    parser.add_argument("--sonar-degradation", choices=("clear", "critical"), default="clear")
    parser.add_argument("--thruster-fault-index", type=int, default=None)
    parser.add_argument("--thruster-fault-effectiveness", type=float, default=1.0)
    # Finding B4 additions (docs/rov-realtime-closed-loop-code-review-2026-
    # 08-27.md): --sensor-fault-schedule scheduled turns --visual-degradation/
    # --sonar-degradation critical from "on for the whole run" into
    # scheduled windows with an actual start/duration/recovery, matching
    # the timing/outage faults above -- default "none" keeps every existing
    # invocation's exact original (permanently-on-or-absent) behavior.
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
    from sensor_msgs.msg import Image  # noqa: E402
    from std_msgs.msg import Float32MultiArray  # noqa: E402
    from holoocean_interfaces.msg import ImagingSonar  # noqa: E402

    from uw_holoocean_adapter.fault_injector import build_fault_schedule, build_sensor_degradation_schedule
    from uw_holoocean_adapter.scenario_randomization import PRESET_CRITICAL_DEGRADED

    manifest = load_realtime_manifest(args.scenario, args.task)
    rng = np.random.default_rng(args.seed)
    perturbation_rng = np.random.default_rng(args.seed + 1)
    topics = build_topic_map(manifest.agent_name)

    injector: Optional[FaultInjector] = None
    if args.fault_profile == "critical":
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
        # Under scheduling, --visual-degradation/--sonar-degradation choose
        # WHICH profile applies inside a window, not a permanently-on value
        # -- outside every window the baseline passed to tick() reverts to
        # clear (None), which is what actually gives the fault a recovery.
        visual_degradation = None
        sonar_degradation = None

    session = RealtimeRosSession(
        manifest, args.seed, rng,
        fault_injector=injector, thruster_fault=thruster_fault, perturbation_rng=perturbation_rng,
        sensor_degradation_schedule=sensor_degradation_schedule,
        visual_degradation_profile=visual_degradation_profile,
        sonar_degradation_profile=sonar_degradation_profile,
    )
    message_types = RosMessageTypes(Image=Image, Odometry=Odometry, ImagingSonar=ImagingSonar, Clock=Clock)

    rclpy.init()
    node = Node("uw_holoocean_realtime_gateway")
    publishers = {
        topic: node.create_publisher(getattr(message_types, attr), topic, qos_profile_sensor_data)
        for topic, attr in (
            (topics.left_camera, "Image"),
            (topics.right_camera, "Image"),
            (topics.pilot_camera, "Image"),
            (topics.imaging_sonar, "ImagingSonar"),
            (topics.vehicle_state, "Odometry"),
            (topics.clock, "Clock"),
            (topics.scoring_truth, "Odometry"),
        )
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
        # NOT node.create_rate(...).sleep() -- confirmed against a real
        # sourced ROS2 install (see docs/rov-realtime-closed-loop-code-
        # review-2026-08-27.md) that it deadlocks here: rclpy's Rate.sleep()
        # blocks on a timer callback that only fires while something spins
        # this node, and nothing spins it while sleep() itself is blocking
        # in this single-threaded loop (spin_once already returned before
        # sleep starts). A plain time.sleep() has no such dependency.
        dt_s = 1.0 / manifest.ticks_per_sec
        while rclpy.ok():
            for topic, message in session.tick(
                message_types, visual_degradation=visual_degradation, sonar_degradation=sonar_degradation
            ):
                publishers[topic].publish(message)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(dt_s)
    finally:
        session.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
