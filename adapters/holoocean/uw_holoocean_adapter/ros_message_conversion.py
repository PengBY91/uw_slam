"""Portable (no rclpy import) conversion from raw HoloOcean sensor readings
to ROS2 message shapes, plus the one place that defines every realtime
closed-loop topic name — so the truth-topic deny list
(`scenario_manifest.py`'s `_TRUTH_TOPIC`) has a single counterpart on the
ROS side instead of being re-typed at each publisher/subscriber call site.

Message-constructing functions here take a `message_types` bundle (an
object exposing `.Odometry`, a zero-arg constructor) rather than importing
`nav_msgs.msg`/`geometry_msgs.msg` directly — same "pass the generated/
vendor types in" principle `camera_conversion.py`/`sonar_conversion.py`
already use for the protobuf schema modules, for the same reason: this
module has no hard dependency on rclpy or any ROS2 message package being
installed, so it is fully unit-testable on a machine with neither (see
`holoocean_driver.py`'s `_import_holoocean()` for the matching pattern on
the HoloOcean side).
"""
from __future__ import annotations

import dataclasses
from typing import Any

import numpy as np

from uw_holoocean_adapter.coordinates import Pose, matrix_to_quaternion, pose_sensor_to_pose

_TRUTH_TOPIC = "/uw/sim/ground_truth"


@dataclasses.dataclass(frozen=True)
class RosMessageTypes:
    """Bundles the message constructor callables this module's conversion
    functions need, so call sites thread one object instead of an
    ever-growing set of positional type parameters — same bundling
    reasoning as `record_session.py`'s `SchemaModules`. A real caller
    passes the actual `sensor_msgs.msg.Image` / `nav_msgs.msg.Odometry` /
    `holoocean_interfaces.msg.ImagingSonar` / `rosgraph_msgs.msg.Clock`
    classes; a test passes hand-rolled fakes implementing only the
    attributes each conversion function actually sets."""

    Image: Any
    Odometry: Any
    ImagingSonar: Any
    Clock: Any


@dataclasses.dataclass(frozen=True)
class StateNoise:
    """Additive Gaussian noise applied when building the noisy `VehicleState`
    odometry published to algorithm consumers (never the raw, noise-free
    `PoseSensor`/scoring truth channel)."""

    orientation_sigma: float
    depth_sigma: float


@dataclasses.dataclass(frozen=True)
class TopicMap:
    left_camera: str
    right_camera: str
    pilot_camera: str
    imaging_sonar: str
    vehicle_state: str
    clock: str
    scoring_truth: str
    algorithm_inputs: tuple[str, ...]


def build_topic_map(agent_name: str = "auv0") -> TopicMap:
    """The complete realtime closed-loop topic set. `algorithm_inputs` is
    exactly the four topics `OnlineAssistPipeline`/algorithm-side code may
    subscribe — never `pilot_camera` (a separate, presentation-only path),
    never `clock`, and never `scoring_truth` (only the scorer may
    subscribe that one)."""
    left_camera = f"/holoocean/{agent_name}/LeftCamera"
    right_camera = f"/holoocean/{agent_name}/RightCamera"
    pilot_camera = f"/holoocean/{agent_name}/PilotCamera"
    imaging_sonar = f"/holoocean/{agent_name}/ImagingSonar"
    vehicle_state = f"/holoocean/{agent_name}/VehicleState"
    return TopicMap(
        left_camera=left_camera,
        right_camera=right_camera,
        pilot_camera=pilot_camera,
        imaging_sonar=imaging_sonar,
        vehicle_state=vehicle_state,
        clock="/clock",
        scoring_truth=_TRUTH_TOPIC,
        algorithm_inputs=(left_camera, right_camera, imaging_sonar, vehicle_state),
    )


def _small_angle_quaternion(rotation_vector_rad: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(rotation_vector_rad))
    if angle < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0])
    axis = rotation_vector_rad / angle
    half = angle / 2.0
    return np.array([axis[0] * np.sin(half), axis[1] * np.sin(half), axis[2] * np.sin(half), np.cos(half)])


def vehicle_state_to_odometry(
    orientation_matrix: np.ndarray,
    angular_velocity: np.ndarray,
    raw_depth_z: float,
    capture_time_s: float,
    noise: StateNoise,
    rng: np.random.Generator,
    message_types,
):
    """Builds the noisy `VehicleState` `nav_msgs/Odometry` published to
    algorithm consumers, from exactly the three sensors the plan specifies
    (`VehicleOrientation`, `IMUSensor` angular velocity, `DepthSensor`) —
    deliberately NOT the ground-truth `ScoringPose`/`PoseSensor` reading,
    and deliberately carrying no x/y position: this vehicle has no onboard
    absolute-position sensor, matching real hardware (x/y is what
    SLAM/estimation is for).

    `raw_depth_z` is HoloOcean's own raw world-frame z (the same convention
    documented in `state_conversion.py`'s `depth_sensor_to_evidence`
    docstring) — passed through close to unchanged (only depth-noise
    perturbed), NOT negated into the positive-down `depth_m` wire
    convention used elsewhere; that convention belongs to the canonical
    protobuf schema, not this ROS-side odometry message.

    `rng` must be an explicitly-owned, seeded `numpy.random.Generator` (see
    this repo's determinism rule in CLAUDE.md) — never a bare
    `np.random.normal()` call against global state.
    """
    orientation_matrix = np.asarray(orientation_matrix, dtype=float)
    if orientation_matrix.shape != (3, 3):
        raise ValueError(f"expected a (3, 3) orientation matrix, got shape {orientation_matrix.shape}")
    angular_velocity = np.asarray(angular_velocity, dtype=float).reshape(3)

    base_pose = Pose(translation=np.zeros(3), quaternion_xyzw=matrix_to_quaternion(orientation_matrix))
    noise_pose = Pose(
        translation=np.zeros(3),
        quaternion_xyzw=_small_angle_quaternion(rng.normal(0.0, noise.orientation_sigma, size=3)),
    )
    noisy_quat = noise_pose.compose(base_pose).quaternion_xyzw
    noisy_depth_z = raw_depth_z + float(rng.normal(0.0, noise.depth_sigma))
    # Angular-rate noise reuses the same orientation_sigma knob (one shared
    # rotational-uncertainty budget) rather than inventing a third
    # StateNoise field beyond the two the plan's own constructor example
    # shows (StateNoise(0.01, 0.02)).
    noisy_angular_velocity = angular_velocity + rng.normal(0.0, noise.orientation_sigma, size=3)

    msg = message_types.Odometry()
    _stamp_from_seconds(msg.header.stamp, capture_time_s)
    msg.pose.pose.position.z = noisy_depth_z
    msg.pose.pose.orientation.x = float(noisy_quat[0])
    msg.pose.pose.orientation.y = float(noisy_quat[1])
    msg.pose.pose.orientation.z = float(noisy_quat[2])
    msg.pose.pose.orientation.w = float(noisy_quat[3])
    msg.twist.twist.angular.x = float(noisy_angular_velocity[0])
    msg.twist.twist.angular.y = float(noisy_angular_velocity[1])
    msg.twist.twist.angular.z = float(noisy_angular_velocity[2])
    return msg


def truth_pose_to_odometry(pose: Pose, capture_time_s: float, message_types):
    """Builds the ground-truth `nav_msgs/Odometry` published on
    `TopicMap.scoring_truth` ONLY -- the raw, noise-free `PoseSensor` pose
    (full x/y/z + orientation), unlike `vehicle_state_to_odometry`'s noisy,
    z-only, algorithm-facing message. Only `TaskScorer` may subscribe this
    topic (see `build_topic_map`'s docstring and `scenario_manifest.py`'s
    `algorithm_topics` deny-list, which SIM-ARCH-002/SYS-ARCH-003 require)."""
    msg = message_types.Odometry()
    _stamp_from_seconds(msg.header.stamp, capture_time_s)
    msg.pose.pose.position.x = float(pose.translation[0])
    msg.pose.pose.position.y = float(pose.translation[1])
    msg.pose.pose.position.z = float(pose.translation[2])
    msg.pose.pose.orientation.x = float(pose.quaternion_xyzw[0])
    msg.pose.pose.orientation.y = float(pose.quaternion_xyzw[1])
    msg.pose.pose.orientation.z = float(pose.quaternion_xyzw[2])
    msg.pose.pose.orientation.w = float(pose.quaternion_xyzw[3])
    return msg


def _stamp_from_seconds(stamp, seconds: float) -> None:
    whole = int(seconds // 1)
    stamp.sec = whole
    stamp.nanosec = int(round((seconds - whole) * 1e9))


def holoocean_camera_to_ros_image(pixels: np.ndarray, *, capture_time_s: float, frame_id: str, message_types):
    """Builds a `sensor_msgs/Image` from one HoloOcean RGBCamera reading.
    Reuses the same real-camera-verified BGR(A)->RGB channel order already
    documented in `camera_conversion.py`'s module docstring — that module
    targets the protobuf `ImageFrame` schema for MCAP bags
    (`record_session.py`); this one targets `sensor_msgs/Image` for live
    ROS2 publishing instead, but it's the same underlying HoloOcean pixel
    quirk either way, so the fix is the same (drop alpha, reverse the
    remaining three channels)."""
    if pixels.ndim != 3 or pixels.shape[2] not in (3, 4):
        raise ValueError(f"expected a (H, W, 3-or-4) camera array, got shape {pixels.shape}")
    if pixels.dtype != np.uint8:
        raise ValueError(f"expected uint8 pixel data, got dtype {pixels.dtype}")

    height, width = pixels.shape[:2]
    rgb = np.ascontiguousarray(pixels[:, :, :3][:, :, ::-1])

    msg = message_types.Image()
    _stamp_from_seconds(msg.header.stamp, capture_time_s)
    msg.header.frame_id = frame_id
    msg.height = height
    msg.width = width
    msg.encoding = "rgb8"
    msg.is_bigendian = 0
    msg.step = width * 3
    msg.data = rgb.tobytes()
    return msg


def holoocean_sonar_to_imaging_sonar_msg(intensity_array: np.ndarray, *, capture_time_ns: int, message_types):
    """Builds a `holoocean_interfaces/msg/ImagingSonar` — this repo's own
    ROS2 sonar bridge (`adapters/ros2/include/adapters/
    ros2_holoocean_sonar_bridge.hpp`) already consumes exactly this real
    message shape (`timestamp`/`bins_range`/`bins_azimuth`/`image_range`,
    confirmed straight from that header and
    `holoocean_ros_bridge_sonar_frame_provider.hpp`'s field-mapping
    comment) — from a raw HoloOcean ImagingSonar reading.

    Publishes the raw `[0, 1]` float32 intensities UNCHANGED: no mirror
    flip, no uint8 quantization. Both of those belong to
    `sonar_conversion.py`'s protobuf `SonarFrame` target and to
    `HoloOceanRosBridgeSonarFrameProvider::PushImagingSonar`'s own
    downstream conversion (its header comment documents applying the
    mirror flip itself, since HoloOcean's raw column order runs opposite
    this platform's ascending-bearing convention) — flipping/quantizing
    here too would double-apply it or corrupt the wire format real
    `holoocean_main` itself publishes.
    """
    if intensity_array.ndim != 2:
        raise ValueError(f"expected a 2D (num_ranges, num_beams) array, got shape {intensity_array.shape}")
    num_ranges, num_beams = intensity_array.shape
    if num_ranges == 0 or num_beams == 0:
        raise ValueError(f"num_ranges/num_beams must both be non-zero, got shape {intensity_array.shape}")

    msg = message_types.ImagingSonar()
    msg.timestamp = int(capture_time_ns)
    msg.bins_range = num_ranges
    msg.bins_azimuth = num_beams
    msg.image_range = np.asarray(intensity_array, dtype=np.float32).reshape(-1).tolist()
    return msg


def sim_time_to_clock_msg(capture_time_s: float, message_types):
    """Builds a `rosgraph_msgs/Clock` from simulation time."""
    msg = message_types.Clock()
    _stamp_from_seconds(msg.clock, capture_time_s)
    return msg
