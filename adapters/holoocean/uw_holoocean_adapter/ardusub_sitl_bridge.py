"""ArduSub SITL <-> HoloOcean physics bridge (PREP-A-05).

ArduPilot's SITL can delegate its physics to an external process over the
"JSON backend": every physics step SITL sends a small binary packet of servo
PWM values to UDP 9002 and blocks until it receives a JSON line describing
the resulting vehicle state. This module is that external process, with
HoloOcean standing in for the physics.

Scheme (a) of the spec: the 8 PWM channels ArduSub already allocated for the
``vectored_6dof`` frame are turned into 8 per-thruster forces and handed to
HoloOcean's own BlueROV2 dynamics (``control_scheme: 0`` = AUV_THRUSTERS);
the resulting IMU/pose readings are converted back into the JSON reply. We do
NOT re-derive an allocation from pilot axes here -- ArduSub owns the
allocation on the real vehicle, and re-doing it would test our matrix instead
of ArduSub's.

Where each piece runs::

    ArduSub SITL (WSL2)  --UDP 9002-->  this bridge (WSL2)  --TCP-->  holoocean_sitl_physics_host.py (Windows)  -->  HoloOcean/UE5
                         <--JSON line--                     <--frame--

The bridge lives on the WSL2 side, and the Windows side is a dumb shuttle,
for the same reason ``bridged_realtime_ros_session.py`` is arranged that
way: everything interesting (PWM decoding, the thruster correspondence, the
frame conversions) is then in the repo, under test, and editable without
re-copying files to the Windows harness directory.

``--physics mock`` swaps HoloOcean for an in-process rigid-body model in the
same frame conventions. That is not a toy: it makes the entire SITL-side
protocol -- packet decode, thruster mapping signs, JSON reply, QGroundControl
arming and DEPTH_HOLD -- verifiable in WSL2 alone, so that when HoloOcean is
attached the only new variable is HoloOcean.

Frame conventions::

    HoloOcean world : x, y, z-up, right-handed
    HoloOcean body  : FLU (x forward, y left, z up)
    ArduPilot world : NED       ArduPilot body : FRD
    C_frd_flu = diag(1, -1, -1)     C_ned_world = diag(1, -1, -1)

The body conversion is the usual FLU->FRD rotation about x and matches
``include/sensor_models/ned_conversion.hpp`` (PREP-C-04) exactly.

The WORLD conversion deliberately does NOT match that header, which maps
ENU->NED with ``[[0,1,0],[1,0,0],[0,0,-1]]`` -- i.e. it calls HoloOcean's
world x "east". That header says in its own comment that its world
convention is a don't-care for its use case (the MAVLink adapter only ever
sends body-frame deltas), and here it is not: it decides the vehicle's
absolute heading. Calling HoloOcean's world x "east" would put a vehicle
facing +x in UE5 at a 90-degree heading in QGroundControl, so this bridge
calls world x NORTH instead, which makes ``diag(1,-1,-1)`` the world
conversion too and leaves a vehicle spawned with identity attitude at
heading 0. Both are proper rotations and each is its own inverse; nothing
downstream of the bridge depends on the other choice.
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import math
import socket
import struct
import sys
import time
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# SITL JSON backend wire format (ArduPilot libraries/SITL/SIM_JSON.{h,cpp}
# and libraries/SITL/examples/JSON/readme.md).
# ---------------------------------------------------------------------------
SERVO_MAGIC_16 = 18458
SERVO_MAGIC_32 = 29569
# uint16 magic, uint16 frame_rate, uint32 frame_count, uint16 pwm[N].
# '<' (standard size, no padding) matches the C struct exactly here because
# the uint32 already lands on a 4-byte boundary.
_SERVO_PACKET_16 = struct.Struct("<HHI16H")
_SERVO_PACKET_32 = struct.Struct("<HHI32H")
DEFAULT_CONTROL_PORT = 9002


@dataclasses.dataclass(frozen=True)
class ServoPacket:
    frame_rate_hz: int
    frame_count: int
    pwm_us: Tuple[int, ...]


def decode_servo_packet(datagram: bytes) -> Optional[ServoPacket]:
    """Decodes one SITL output packet, or None if it is not one.

    Returning None rather than raising is deliberate: UDP 9002 is a public
    socket and the bridge must not die because something else on the host
    sent it a stray datagram.
    """
    if len(datagram) == _SERVO_PACKET_16.size:
        fields = _SERVO_PACKET_16.unpack(datagram)
        if fields[0] != SERVO_MAGIC_16:
            return None
    elif len(datagram) == _SERVO_PACKET_32.size:
        fields = _SERVO_PACKET_32.unpack(datagram)
        if fields[0] != SERVO_MAGIC_32:
            return None
    else:
        return None
    return ServoPacket(frame_rate_hz=fields[1], frame_count=fields[2], pwm_us=tuple(fields[3:]))


# ---------------------------------------------------------------------------
# ArduSub motor outputs -> HoloOcean thruster forces.
# ---------------------------------------------------------------------------
# ArduSub's SUB_FRAME_VECTORED_6DOF table (AP_Motors6DOF.cpp) and HoloOcean's
# BlueROV2 thruster table describe the same eight physical thrusters in
# different orders and in different body frames. The correspondence below is
# derived NUMERICALLY from both tables (tests/test_ardusub_sitl_bridge.py
# re-derives it on every run) rather than read off the thruster names, which
# do not match either table's geometry.
#
# IMPORTANT: the HoloOcean side of that derivation must use the ENGINE's
# thruster geometry, which is what thrust_allocation.py now carries. The
# first version of this constant was derived from the table in the Python
# client's holoocean/agents.py, which upstream marks "may not be correct --
# check the C++", and it is not: with that table the mapping came out as
# (6, 7, 4, 5, 3, 2, 0, 1) with a uniform -1 sign, which in the engine
# produces ZERO force for a pure ArduSub surge demand and inverts both yaw
# and pitch. The inverted yaw is what turned ArduSub's heading hold into a
# positive feedback loop and spun the vehicle to 60+ rad/s in the first
# HoloOcean run -- see docs/ardusub-sitl-bridge-feasibility.md section 2.4.
#
# ARDUSUB_TO_HOLOOCEAN[k] is the HoloOcean thruster index driven by ArduSub
# motor k+1 (MOT_1..MOT_8); MOT_1..4 are the four angled/horizontal
# thrusters, MOT_5..8 the verticals.
ARDUSUB_TO_HOLOOCEAN: Tuple[int, ...] = (4, 5, 7, 6, 0, 1, 3, 2)

# Per-motor sense of that correspondence. NOT a single global constant: the
# engine's angled thrusters all push forward, so the two ArduSub motors whose
# forward factor is positive drive their thruster positively while the two
# whose factor is negative drive theirs negatively. (The earlier uniform -1
# was an artifact of the wrong geometry table.)
ARDUSUB_THRUSTER_SIGN: Tuple[float, ...] = (-1.0, -1.0, 1.0, 1.0, -1.0, -1.0, -1.0, -1.0)

# BlueROV2 T200 at 16 V: 5.25 kgf forward, 4.1 kgf reverse (Blue Robotics
# published thrust curve). Asymmetric because the propeller is not
# symmetric; ArduSub itself knows nothing about this, it just outputs PWM.
# These are the REAL vehicle's numbers, used by the mock backend.
DEFAULT_MAX_THRUST_FORWARD_N = 51.5
DEFAULT_MAX_THRUST_REVERSE_N = 40.2

# HoloOcean's BlueROV2 action space is narrower than the real thruster, and
# the engine CLAMPS to it rather than scaling (BlueROV2.cpp ApplyThrusters:
# FMath::Clamp(..., -BR_MAX_THRUST, BR_MAX_THRUST)). Sending the real T200's
# 51.5 N therefore just saturates asymmetrically. The holoocean backend
# defaults to the engine's own limit; --max-thrust-forward-n overrides it.
HOLOOCEAN_MAX_THRUST_N = 28.75

DEFAULT_PWM_MIN_US = 1100
DEFAULT_PWM_TRIM_US = 1500
DEFAULT_PWM_MAX_US = 1900


def pwm_to_normalized(
    pwm_us: float,
    pwm_min_us: float = DEFAULT_PWM_MIN_US,
    pwm_trim_us: float = DEFAULT_PWM_TRIM_US,
    pwm_max_us: float = DEFAULT_PWM_MAX_US,
) -> float:
    """PWM microseconds -> motor output in [-1, 1].

    A zero/absent PWM value (SITL emits 0 on channels with no assigned
    output function) reads as neutral rather than as full reverse, which is
    what an ESC that never sees a pulse actually does.
    """
    if not math.isfinite(pwm_us) or pwm_us <= 0.0:
        return 0.0
    if pwm_us >= pwm_trim_us:
        span = max(pwm_max_us - pwm_trim_us, 1e-9)
        return min(1.0, (pwm_us - pwm_trim_us) / span)
    span = max(pwm_trim_us - pwm_min_us, 1e-9)
    return max(-1.0, (pwm_us - pwm_trim_us) / span)


def pwm_to_thruster_forces(
    pwm_us: Sequence[float],
    max_forward_n: float = DEFAULT_MAX_THRUST_FORWARD_N,
    max_reverse_n: float = DEFAULT_MAX_THRUST_REVERSE_N,
    pwm_min_us: float = DEFAULT_PWM_MIN_US,
    pwm_trim_us: float = DEFAULT_PWM_TRIM_US,
    pwm_max_us: float = DEFAULT_PWM_MAX_US,
) -> List[float]:
    """The 8 HoloOcean thruster forces (N) for one SITL servo packet."""
    forces = [0.0] * 8
    for motor_index in range(8):
        output = pwm_to_normalized(pwm_us[motor_index], pwm_min_us, pwm_trim_us, pwm_max_us)
        newtons = output * (max_forward_n if output >= 0.0 else max_reverse_n)
        forces[ARDUSUB_TO_HOLOOCEAN[motor_index]] = ARDUSUB_THRUSTER_SIGN[motor_index] * newtons
    return forces


# ---------------------------------------------------------------------------
# Frame conversions (see the module docstring).
# ---------------------------------------------------------------------------
C_FRD_FLU = np.diag([1.0, -1.0, -1.0])
# HoloOcean world x is taken as NORTH -- see the module docstring for why
# this differs from ned_conversion.hpp's ENU->NED matrix.
C_NED_WORLD = np.diag([1.0, -1.0, -1.0])


def matrix_to_quaternion_wxyz(rotation: np.ndarray) -> Tuple[float, float, float, float]:
    """Rotation matrix -> (w, x, y, z), the order ArduPilot's JSON backend
    parses `quaternion` in (its Quaternion is q1=w, q2=x, q3=y, q4=z)."""
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rotation[2, 1] - rotation[1, 2]) / s
        y = (rotation[0, 2] - rotation[2, 0]) / s
        z = (rotation[1, 0] - rotation[0, 1]) / s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        w = (rotation[2, 1] - rotation[1, 2]) / s
        x = 0.25 * s
        y = (rotation[0, 1] + rotation[1, 0]) / s
        z = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        w = (rotation[0, 2] - rotation[2, 0]) / s
        x = (rotation[0, 1] + rotation[1, 0]) / s
        y = 0.25 * s
        z = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        w = (rotation[1, 0] - rotation[0, 1]) / s
        x = (rotation[0, 2] + rotation[2, 0]) / s
        y = (rotation[1, 2] + rotation[2, 1]) / s
        z = 0.25 * s
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    return (w / norm, x / norm, y / norm, z / norm)


def quaternion_xyzw_to_matrix(q_xyzw: Sequence[float]) -> np.ndarray:
    x, y, z, w = (float(v) for v in q_xyzw)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


@dataclasses.dataclass
class VehicleState:
    """One physics step's result, in HoloOcean's own conventions.

    Kept in HoloOcean's frames (not ArduPilot's) so that the conversion
    happens in exactly one place, `build_json_state`, and both physics
    backends feed it identically.
    """

    sim_time_s: float
    position_enu_m: np.ndarray
    velocity_enu_mps: np.ndarray
    # Body -> world rotation, i.e. columns are the FLU body axes in ENU.
    rotation_enu_flu: np.ndarray
    angular_velocity_flu_radps: np.ndarray
    # Specific force (what an accelerometer reads: includes gravity), FLU.
    specific_force_flu_mps2: np.ndarray


def build_json_state(state: VehicleState, position_offset_enu_m: np.ndarray) -> dict:
    """VehicleState -> the dict SITL's JSON backend parses.

    `position_offset_enu_m` is subtracted before the NED conversion. It
    normally zeroes only the horizontal start position: the vertical
    component must NOT be zeroed, because SITL derives the simulated
    barometer (and therefore ArduSub's depth, and therefore DEPTH_HOLD)
    from this position's down component. Zeroing z would tell ArduSub it
    starts at the surface no matter where the vehicle was spawned.
    """
    position_ned = C_NED_WORLD @ (np.asarray(state.position_enu_m, dtype=float) - position_offset_enu_m)
    velocity_ned = C_NED_WORLD @ np.asarray(state.velocity_enu_mps, dtype=float)
    rotation_ned_frd = C_NED_WORLD @ np.asarray(state.rotation_enu_flu, dtype=float) @ C_FRD_FLU
    gyro_frd = C_FRD_FLU @ np.asarray(state.angular_velocity_flu_radps, dtype=float)
    accel_frd = C_FRD_FLU @ np.asarray(state.specific_force_flu_mps2, dtype=float)
    quaternion = matrix_to_quaternion_wxyz(rotation_ned_frd)
    return {
        "timestamp": float(state.sim_time_s),
        "imu": {"gyro": [float(v) for v in gyro_frd], "accel_body": [float(v) for v in accel_frd]},
        "position": [float(v) for v in position_ned],
        "quaternion": [float(v) for v in quaternion],
        "velocity": [float(v) for v in velocity_ned],
    }


def encode_json_reply(payload: dict) -> bytes:
    """SITL wants each frame preceded and terminated by a newline."""
    return ("\n" + json.dumps(payload, separators=(",", ":")) + "\n").encode("ascii")


# ---------------------------------------------------------------------------
# Physics backends.
# ---------------------------------------------------------------------------
class MockPhysics:
    """In-process 6-DOF rigid body in HoloOcean's own frames.

    Exists so the SITL-facing half of this bridge -- packet decode, the
    thruster correspondence and its signs, the JSON reply, and everything
    ArduSub then does with it (arming, STABILIZE, DEPTH_HOLD) -- can be
    verified in WSL2 with no simulator attached. When HoloOcean is then
    plugged in, HoloOcean is the only new variable.

    The model is coarse (diagonal added mass, quadratic drag, no coupling)
    but its FRAMES AND SIGNS are exactly HoloOcean's, which is the part the
    bridge is being checked against. It is not a substitute for HoloOcean's
    own dynamics and must never be used to draw a conclusion about vehicle
    behaviour -- only about plumbing.
    """

    def __init__(
        self,
        start_position_enu_m: Sequence[float] = (0.0, 0.0, -3.0),
        # A floor, so a runaway dive terminates instead of integrating to
        # kilometres. HoloOcean's worlds have their own seabed; this exists
        # so the mock behaves comparably.
        seabed_depth_m: float = 30.0,
        mass_kg: float = 11.5,
        # BlueROV2 Heavy is trimmed slightly positively buoyant so it
        # surfaces on power loss; ~2 N is the usual ballpark.
        net_buoyancy_n: float = 2.0,
        # Vertical separation between the centre of buoyancy and the centre
        # of gravity. Small (~1 cm on a BlueROV2) but it is the ONLY thing
        # keeping the vehicle upright in roll and pitch, and leaving it out
        # makes the model tumble freely under any asymmetric thrust -- which
        # then contaminates every axis measurement taken from it. Yaw
        # deliberately gets no restoring term: a submerged body really is
        # free in yaw, which is why ArduSub has to hold heading itself.
        bg_separation_m: float = 0.010,
        added_mass_factor: Sequence[float] = (1.2, 1.6, 2.4),
        inertia_kgm2: Sequence[float] = (0.26, 0.23, 0.37),
        linear_drag_ns_m: Sequence[float] = (18.0, 32.0, 55.0),
        quadratic_drag_ns2_m2: Sequence[float] = (28.0, 60.0, 90.0),
        angular_drag_nms: Sequence[float] = (0.8, 0.8, 1.2),
        gravity_mps2: float = 9.80665,
    ) -> None:
        from uw_holoocean_adapter.thrust_allocation import wrench_matrix

        self._wrench_matrix = wrench_matrix()
        self._mass = mass_kg
        self._effective_mass = mass_kg * np.asarray(added_mass_factor, dtype=float)
        self._inertia = np.asarray(inertia_kgm2, dtype=float)
        self._linear_drag = np.asarray(linear_drag_ns_m, dtype=float)
        self._quadratic_drag = np.asarray(quadratic_drag_ns2_m2, dtype=float)
        self._angular_drag = np.asarray(angular_drag_nms, dtype=float)
        self._net_buoyancy_n = net_buoyancy_n
        self._gravity = gravity_mps2
        # Total (not net) buoyancy is what generates the righting moment:
        # the couple is between the full buoyant force at the CB and the
        # full weight at the CG, and the two very nearly cancel in
        # magnitude on a trimmed vehicle.
        self._buoyancy_n = mass_kg * gravity_mps2 + net_buoyancy_n
        self._bg_separation_m = bg_separation_m

        self._seabed_z_m = -abs(seabed_depth_m)
        self._position = np.asarray(start_position_enu_m, dtype=float)
        self._velocity = np.zeros(3)
        self._rotation = np.eye(3)
        self._angular_velocity = np.zeros(3)
        self._specific_force = np.array([0.0, 0.0, gravity_mps2])
        self._sim_time_s = 0.0

    @property
    def sim_time_s(self) -> float:
        return self._sim_time_s

    def step(self, thruster_forces_n: Sequence[float], dt_s: float) -> VehicleState:
        wrench = self._wrench_matrix @ np.asarray(thruster_forces_n, dtype=float)
        force_body = wrench[0:3]
        torque_body = wrench[3:6]

        # Net buoyancy acts along world +z regardless of attitude.
        buoyancy_body = self._rotation.T @ np.array([0.0, 0.0, self._net_buoyancy_n])
        accel_world = self._rotation @ ((force_body + buoyancy_body) / self._effective_mass)

        previous_velocity = self._velocity.copy()
        velocity_world = self._velocity + accel_world * dt_s

        # Drag is applied SEMI-IMPLICITLY (backward Euler), not as another
        # explicit force term. Explicit quadratic drag is unstable at this
        # timestep and it bit hard: at 10 m/s the lateral drag deceleration
        # is 343 m/s^2, so one 20 ms step overshoots by 6.9 m/s, reverses
        # the sign and grows -- the vehicle ended up oscillating at
        # +/-10 m/s no matter what was commanded, which read downstream as
        # "the lateral axis is inverted". Solving v1 = v0 - (c1 + c2|v0|)
        # v1 dt / m for v1 gives a division that is unconditionally stable
        # and can never flip the sign.
        velocity_body = self._rotation.T @ velocity_world
        damping = 1.0 + (self._linear_drag + self._quadratic_drag * np.abs(velocity_body)) * dt_s / self._effective_mass
        velocity_body = velocity_body / damping
        self._velocity = self._rotation @ velocity_body

        self._position = self._position + self._velocity * dt_s
        # A real ROV stops at the water surface, and HoloOcean's worlds
        # have a seabed. Without either, the slightly positive buoyancy
        # sends the mock vehicle "flying" and every depth check downstream
        # becomes meaningless.
        if self._position[2] > 0.0:
            self._position[2] = 0.0
            self._velocity[2] = min(self._velocity[2], 0.0)
        elif self._position[2] < self._seabed_z_m:
            self._position[2] = self._seabed_z_m
            self._velocity[2] = max(self._velocity[2], 0.0)

        # Righting couple: the buoyant force acts at the CB, which sits
        # `bg_separation_m` above the CG along body +z.
        buoyant_force_body = self._rotation.T @ np.array([0.0, 0.0, self._buoyancy_n])
        righting_torque = np.cross(np.array([0.0, 0.0, self._bg_separation_m]), buoyant_force_body)
        angular_velocity = self._angular_velocity + (torque_body + righting_torque) / self._inertia * dt_s
        angular_velocity = angular_velocity / (1.0 + self._angular_drag * dt_s / self._inertia)
        self._angular_velocity = angular_velocity
        self._rotation = _orthonormalize(self._rotation @ _exp_so3(self._angular_velocity * dt_s))

        # An accelerometer reads specific force: the total kinematic
        # acceleration (thrust AND drag, so it is differenced from the
        # actual velocity change rather than recomputed) minus gravity. At
        # rest and level this is (0, 0, +g) in FLU, the convention
        # HoloOcean's IMUSensor also uses (see imu.proto).
        total_accel_world = (self._velocity - previous_velocity) / dt_s
        self._specific_force = self._rotation.T @ (total_accel_world + np.array([0.0, 0.0, self._gravity]))
        self._sim_time_s += dt_s

        return VehicleState(
            sim_time_s=self._sim_time_s,
            position_enu_m=self._position.copy(),
            velocity_enu_mps=self._velocity.copy(),
            rotation_enu_flu=self._rotation.copy(),
            angular_velocity_flu_radps=self._angular_velocity.copy(),
            specific_force_flu_mps2=self._specific_force.copy(),
        )


def _exp_so3(phi: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(phi))
    if angle < 1e-12:
        return np.eye(3)
    axis = phi / angle
    skew = np.array([[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]])
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)


def _orthonormalize(rotation: np.ndarray) -> np.ndarray:
    u, _, vt = np.linalg.svd(rotation)
    result = u @ vt
    if np.linalg.det(result) < 0.0:
        u[:, -1] *= -1.0
        result = u @ vt
    return result


class HoloOceanPhysics:
    """HoloOcean over the raw_frame_wire TCP protocol.

    Listens for `holoocean_sitl_physics_host.py` (Windows) to connect, then
    runs the same request/response loop the existing bridged realtime
    session uses: send this step's 8 thruster forces, receive the resulting
    RawSensorFrame. Reconnects rather than exiting, because this dev
    machine's WSL2<->Windows TCP path is known to drop long-lived
    connections mid-run (documented in holoocean_bridge_sensor_host.py).
    """

    def __init__(
        self,
        listen_host: str = "0.0.0.0",
        listen_port: int = 5601,
        imu_sensor: str = "IMUSensor",
        pose_sensor: str = "PoseSensor",
        velocity_sensor: Optional[str] = "DynamicsSensor",
        gravity_mode: str = "auto",
        gravity_mps2: float = 9.80665,
    ) -> None:
        self._imu_sensor = imu_sensor
        self._pose_sensor = pose_sensor
        self._velocity_sensor = velocity_sensor
        self._gravity_mode = gravity_mode
        self._gravity = gravity_mps2
        self._imu_includes_gravity: Optional[bool] = None if gravity_mode == "auto" else (
            gravity_mode == "included"
        )
        self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listener.bind((listen_host, listen_port))
        self._listener.listen(1)
        print(f"ardusub_sitl_bridge: waiting for the HoloOcean host on {listen_host}:{listen_port} ...",
              flush=True)
        self._socket, peer = self._listener.accept()
        print(f"ardusub_sitl_bridge: HoloOcean host connected from {peer}", flush=True)
        self._reconnects = 0
        self._previous_position: Optional[np.ndarray] = None
        self._previous_time_s: Optional[float] = None

    def _reconnect(self) -> None:
        self._reconnects += 1
        print(f"ardusub_sitl_bridge: HoloOcean link lost, waiting for reconnect (#{self._reconnects}) ...",
              flush=True)
        try:
            self._socket.close()
        except OSError:
            pass
        self._socket, peer = self._listener.accept()
        print(f"ardusub_sitl_bridge: HoloOcean host reconnected from {peer}", flush=True)

    def step(self, thruster_forces_n: Sequence[float], dt_s: float) -> VehicleState:
        from uw_holoocean_adapter.raw_frame_wire import recv_raw_sensor_frame, send_thruster_command

        while True:
            try:
                send_thruster_command(self._socket, list(thruster_forces_n))
                frame = recv_raw_sensor_frame(self._socket)
                break
            except OSError as error:
                print(f"ardusub_sitl_bridge: link error {error!r}", flush=True)
                self._reconnect()
        return self.frame_to_state(frame.sim_time_s, frame.sensors, dt_s)

    def frame_to_state(self, sim_time_s: float, sensors: dict, dt_s: float) -> VehicleState:
        state = holoocean_frame_to_state(
            sim_time_s,
            sensors,
            dt_s,
            imu_sensor=self._imu_sensor,
            pose_sensor=self._pose_sensor,
            velocity_sensor=self._velocity_sensor,
            gravity_state=self,
            previous=(
                None
                if self._previous_position is None
                else (self._previous_time_s, self._previous_position)
            ),
        )
        self._previous_position = np.asarray(state.position_enu_m, dtype=float).copy()
        self._previous_time_s = sim_time_s
        return state

    # -- gravity convention detection ------------------------------------
    def resolve_specific_force(self, accel_flu: np.ndarray, rotation: np.ndarray) -> np.ndarray:
        """HoloOcean's IMUSensor is documented (imu.proto) as reporting
        acceleration INCLUDING gravity, i.e. specific force -- which is
        exactly what ArduPilot's JSON `accel_body` wants. That is an
        assumption written down in this repo, though, not something anyone
        has measured, and getting it wrong is a silent 9.8 m/s^2 bias into
        ArduSub's EKF. So on the first frame the magnitude is checked
        against g and the answer is logged; `--imu-gravity` overrides."""
        if self._imu_includes_gravity is None:
            magnitude = float(np.linalg.norm(accel_flu))
            self._imu_includes_gravity = magnitude > 0.5 * self._gravity
            print(f"ardusub_sitl_bridge: first IMU sample |accel| = {magnitude:.3f} m/s^2 -> treating "
                  f"HoloOcean IMU as {'INCLUDING' if self._imu_includes_gravity else 'EXCLUDING'} gravity",
                  flush=True)
        if self._imu_includes_gravity:
            return accel_flu
        return accel_flu + rotation.T @ np.array([0.0, 0.0, self._gravity])

    def close(self) -> None:
        try:
            self._socket.close()
        finally:
            self._listener.close()


def holoocean_frame_to_state(
    sim_time_s: float,
    sensors: dict,
    dt_s: float,
    *,
    imu_sensor: str = "IMUSensor",
    pose_sensor: str = "PoseSensor",
    velocity_sensor: Optional[str] = "DynamicsSensor",
    gravity_state=None,
    previous: Optional[Tuple[float, np.ndarray]] = None,
) -> VehicleState:
    """One HoloOcean tick's sensors -> VehicleState.

    Velocity comes from a DynamicsSensor when the scenario has one and its
    shape is recognised, otherwise from differencing consecutive PoseSensor
    positions. Differencing is the fallback rather than the default because
    it lags half a step and quantises at the tick rate; it is good enough
    here because ArduSub without a GPS uses this velocity only for SITL's
    own state reporting, not in the EKF.
    """
    if pose_sensor not in sensors:
        raise KeyError(f"HoloOcean frame has no {pose_sensor!r}; sensors present: {sorted(sensors)}")
    pose_matrix = np.asarray(sensors[pose_sensor], dtype=float)
    if pose_matrix.shape != (4, 4):
        raise ValueError(f"{pose_sensor} must be a 4x4 matrix, got shape {pose_matrix.shape}")
    position = pose_matrix[:3, 3].copy()
    rotation = _orthonormalize(pose_matrix[:3, :3].copy())

    if imu_sensor not in sensors:
        raise KeyError(f"HoloOcean frame has no {imu_sensor!r}; sensors present: {sorted(sensors)}")
    imu = np.asarray(sensors[imu_sensor], dtype=float)
    if imu.ndim != 2 or imu.shape[1] != 3 or imu.shape[0] not in (2, 4):
        raise ValueError(f"{imu_sensor} must be a (2,3) or (4,3) array, got shape {imu.shape}")
    accel_flu = imu[0].copy()
    gyro_flu = imu[1].copy()

    velocity = _extract_velocity(sensors, velocity_sensor, rotation)
    if velocity is None:
        if previous is not None and dt_s > 0.0:
            velocity = (position - previous[1]) / dt_s
        else:
            velocity = np.zeros(3)

    if gravity_state is not None:
        specific_force = gravity_state.resolve_specific_force(accel_flu, rotation)
    else:
        specific_force = accel_flu

    return VehicleState(
        sim_time_s=sim_time_s,
        position_enu_m=position,
        velocity_enu_mps=np.asarray(velocity, dtype=float),
        rotation_enu_flu=rotation,
        angular_velocity_flu_radps=gyro_flu,
        specific_force_flu_mps2=specific_force,
    )


def _extract_velocity(sensors: dict, velocity_sensor: Optional[str], rotation: np.ndarray):
    """HoloOcean's DynamicsSensor packs [accel, vel, pos, ang_accel, ang_vel,
    rpy] as an (n, 3) array whose row count depends on its own
    `UseCOM`/`UseRPY` options. Only the shapes whose velocity row is
    unambiguous are accepted; anything else falls back to differencing
    rather than guessing which row is which."""
    if velocity_sensor is None or velocity_sensor not in sensors:
        return None
    array = np.asarray(sensors[velocity_sensor], dtype=float)
    if array.ndim == 1 and array.size >= 6:
        return array[3:6].copy()
    if array.ndim == 2 and array.shape[1] == 3 and array.shape[0] >= 2:
        return array[1].copy()
    return None


# ---------------------------------------------------------------------------
# Main loop.
# ---------------------------------------------------------------------------
@dataclasses.dataclass
class BridgeStats:
    frames: int = 0
    stale_frames: int = 0        # SITL resent a frame_count we already served
    dropped_frames: int = 0      # frame_count jumped by more than 1
    foreign_datagrams: int = 0
    first_wall_time_s: float = 0.0
    last_wall_time_s: float = 0.0
    last_sim_time_s: float = 0.0
    # (wall, sim) of the frame `recent_window` frames ago, for the steady-state
    # rate. The cumulative figures include SITL's startup and EKF
    # initialisation, during which the loop runs well below rate -- measured
    # here, the cumulative RTF of a settled 50 Hz loop still read 0.73 after
    # 30 s and only reached 0.87 after 80 s while the instantaneous rate was
    # already at target the whole time. Reporting only the cumulative figure
    # would understate whatever HoloOcean can actually sustain.
    recent_window: int = 500
    _window: List[Tuple[float, float]] = dataclasses.field(default_factory=list)

    def observe(self, wall_time_s: float, sim_time_s: float) -> None:
        self._window.append((wall_time_s, sim_time_s))
        if len(self._window) > self.recent_window:
            del self._window[0]

    def realtime_factor(self) -> float:
        wall = self.last_wall_time_s - self.first_wall_time_s
        return self.last_sim_time_s / wall if wall > 1e-9 else 0.0

    def frame_rate_hz(self) -> float:
        wall = self.last_wall_time_s - self.first_wall_time_s
        return self.frames / wall if wall > 1e-9 else 0.0

    def recent_frame_rate_hz(self) -> float:
        if len(self._window) < 2:
            return 0.0
        wall = self._window[-1][0] - self._window[0][0]
        return (len(self._window) - 1) / wall if wall > 1e-9 else 0.0

    def recent_realtime_factor(self) -> float:
        if len(self._window) < 2:
            return 0.0
        wall = self._window[-1][0] - self._window[0][0]
        sim = self._window[-1][1] - self._window[0][1]
        return sim / wall if wall > 1e-9 else 0.0


def run_bridge(
    physics,
    *,
    control_port: int = DEFAULT_CONTROL_PORT,
    bind_host: str = "127.0.0.1",
    step_hz: float = 50.0,
    max_frames: Optional[int] = None,
    max_seconds: Optional[float] = None,
    max_thrust_forward_n: float = DEFAULT_MAX_THRUST_FORWARD_N,
    max_thrust_reverse_n: float = DEFAULT_MAX_THRUST_REVERSE_N,
    zero_horizontal_origin: bool = True,
    report_every: int = 200,
    rate_check_after_frames: int = 100,
    trace_csv_path: Optional[str] = None,
    clock: Callable[[], float] = time.monotonic,
    socket_factory: Optional[Callable[[], socket.socket]] = None,
) -> BridgeStats:
    """Serves SITL's JSON physics interface until stopped.

    `step_hz` is the physics timestep the bridge advances per SITL frame.
    It MUST match ArduPilot's SIM_RATE_HZ and, when HoloOcean is the
    backend, HoloOcean's own `ticks_per_sec` -- one HoloOcean tick advances
    HoloOcean's clock by 1/ticks_per_sec, so any other value makes the two
    simulators' clocks drift apart while both believe they are in sync.
    The bridge cannot read either value, so it is reported at startup and
    checked against the frame_rate SITL advertises in every packet.
    """
    if socket_factory is None:
        udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        udp.bind((bind_host, control_port))
    else:
        udp = socket_factory()
    udp.settimeout(1.0)
    dt_s = 1.0 / step_hz
    print(f"ardusub_sitl_bridge: serving SITL JSON physics on udp://{bind_host}:{control_port} "
          f"at {step_hz:g} Hz (dt={dt_s * 1000:.2f} ms)", flush=True)

    # A per-frame trace of what the PHYSICS actually produced, independent
    # of anything ArduSub's EKF believes. Indispensable when a closed-loop
    # check disagrees with the model: it separates "the vehicle did not
    # move" from "the estimator says it did not move".
    trace = open(trace_csv_path, "w", encoding="ascii") if trace_csv_path else None
    if trace is not None:
        trace.write("frame,sim_time_s,wall_s,px,py,pz,vx,vy,vz,yaw_deg,"
                    + ",".join(f"f{i}" for i in range(8)) + "\n")

    stats = BridgeStats()
    position_offset = np.zeros(3)
    offset_captured = not zero_horizontal_origin
    # The physics backend's clock is rebased to zero at the first frame.
    # It matters when the backend outlives the bridge: the Windows
    # HoloOcean host reconnects automatically after a bridge restart, but
    # its session clock keeps counting, so without this SITL's first
    # timestamp would jump by however long the previous run lasted (seen
    # here: a restarted SITL was handed timestamp 265.66 s).
    sim_time_offset: Optional[float] = None
    last_frame_count: Optional[int] = None
    last_state: Optional[VehicleState] = None
    last_forces: List[float] = [0.0] * 8
    warned_rate = False

    try:
        while True:
            try:
                datagram, peer = udp.recvfrom(2048)
            except socket.timeout:
                if stats.frames == 0:
                    print("ardusub_sitl_bridge: still waiting for SITL ...", flush=True)
                continue

            packet = decode_servo_packet(datagram)
            if packet is None:
                stats.foreign_datagrams += 1
                continue

            # Checked a little way in, not on the first packet: with a
            # fresh eeprom SITL emits a few frames at its built-in default
            # (1200 Hz on SITL builds) before --add-param-file's values are
            # applied, and warning on those is a false alarm every time.
            if (not warned_rate and stats.frames >= rate_check_after_frames
                    and packet.frame_rate_hz > 0 and abs(packet.frame_rate_hz - step_hz) > 1.0):
                warned_rate = True
                print(f"ardusub_sitl_bridge: WARNING SITL advertises SIM_RATE_HZ={packet.frame_rate_hz} "
                      f"but this bridge steps at {step_hz:g} Hz -- the two clocks will diverge. "
                      f"Set SIM_RATE_HZ={step_hz:g} (and HoloOcean ticks_per_sec to match).", flush=True)

            # ArduPilot resends the same frame_count, without incrementing
            # it, when it has heard nothing back for 10 s (SIM_JSON.cpp's
            # "resending servos" path). Stepping the physics again for such
            # a resend would advance the simulation twice for one control
            # frame, so the cached state is re-served instead.
            is_resend = last_frame_count is not None and packet.frame_count == last_frame_count
            if last_frame_count is not None and packet.frame_count > last_frame_count + 1:
                stats.dropped_frames += packet.frame_count - last_frame_count - 1
            last_frame_count = packet.frame_count

            if is_resend and last_state is not None:
                stats.stale_frames += 1
                state = last_state
            else:
                forces = pwm_to_thruster_forces(
                    packet.pwm_us, max_thrust_forward_n, max_thrust_reverse_n
                )
                state = physics.step(forces, dt_s)
                last_forces = forces

            if not offset_captured:
                # Horizontal only: the vertical component is what SITL turns
                # into the simulated barometer, i.e. ArduSub's depth.
                position_offset = np.array([state.position_enu_m[0], state.position_enu_m[1], 0.0])
                offset_captured = True

            if sim_time_offset is None:
                # Rebased so the FIRST frame reports one timestep rather
                # than zero: SITL computes deltat against its own previous
                # timestamp, which starts at 0, and a first frame of
                # timestamp 0 would advance its clock by nothing.
                sim_time_offset = state.sim_time_s - dt_s
            state = dataclasses.replace(state, sim_time_s=state.sim_time_s - sim_time_offset)

            last_state = state
            if trace is not None:
                rotation = np.asarray(state.rotation_enu_flu, dtype=float)
                yaw_deg = math.degrees(math.atan2(rotation[1, 0], rotation[0, 0]))
                trace.write(
                    f"{stats.frames},{state.sim_time_s:.4f},{clock():.4f},"
                    + ",".join(f"{v:.4f}" for v in state.position_enu_m)
                    + "," + ",".join(f"{v:.4f}" for v in state.velocity_enu_mps)
                    + f",{yaw_deg:.2f},"
                    + ",".join(f"{v:.2f}" for v in last_forces) + "\n"
                )
            udp.sendto(encode_json_reply(build_json_state(state, position_offset)), peer)

            now = clock()
            if stats.frames == 0:
                stats.first_wall_time_s = now
            stats.frames += 1
            stats.last_wall_time_s = now
            stats.last_sim_time_s = state.sim_time_s
            stats.observe(now, state.sim_time_s)

            if report_every > 0 and stats.frames % report_every == 0:
                print(f"ardusub_sitl_bridge: {stats.frames} frames, sim_time={state.sim_time_s:.2f}s, "
                      f"recent {stats.recent_frame_rate_hz():.1f} frames/s "
                      f"(RTF {stats.recent_realtime_factor():.2f}), "
                      f"cumulative {stats.frame_rate_hz():.1f} frames/s "
                      f"(RTF {stats.realtime_factor():.2f}), "
                      f"stale={stats.stale_frames} dropped={stats.dropped_frames}", flush=True)

            if max_frames is not None and stats.frames >= max_frames:
                break
            if max_seconds is not None and stats.last_wall_time_s - stats.first_wall_time_s >= max_seconds:
                break
    except KeyboardInterrupt:
        print("ardusub_sitl_bridge: interrupted", flush=True)
    finally:
        if trace is not None:
            trace.close()
        udp.close()
        close = getattr(physics, "close", None)
        if close is not None:
            close()

    print(f"ardusub_sitl_bridge: {stats.frames} frames served, sim_time={stats.last_sim_time_s:.2f}s; "
          f"steady state {stats.recent_frame_rate_hz():.1f} frames/s "
          f"(RTF {stats.recent_realtime_factor():.2f}); "
          f"cumulative {stats.frame_rate_hz():.1f} frames/s "
          f"(RTF {stats.realtime_factor():.2f}); "
          f"stale={stats.stale_frames}, dropped={stats.dropped_frames}, "
          f"foreign_datagrams={stats.foreign_datagrams}", flush=True)
    return stats


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--physics", choices=("mock", "holoocean"), default="mock")
    parser.add_argument("--control-port", type=int, default=DEFAULT_CONTROL_PORT)
    parser.add_argument("--bind-host", default="127.0.0.1")
    parser.add_argument("--step-hz", type=float, default=50.0,
                        help="physics step rate; must equal ArduPilot SIM_RATE_HZ and, for the "
                             "holoocean backend, the scenario's ticks_per_sec")
    parser.add_argument("--listen-host", default="0.0.0.0", help="holoocean backend: TCP bind host")
    parser.add_argument("--listen-port", type=int, default=5601, help="holoocean backend: TCP bind port")
    parser.add_argument("--imu-gravity", choices=("auto", "included", "excluded"), default="auto")
    parser.add_argument("--velocity-sensor", default="DynamicsSensor",
                        help="empty string to always difference PoseSensor instead")
    parser.add_argument("--max-thrust-forward-n", type=float, default=None,
                        help=f"default {DEFAULT_MAX_THRUST_FORWARD_N} N (real T200) for --physics "
                             f"mock, {HOLOOCEAN_MAX_THRUST_N} N (HoloOcean's own action-space "
                             f"limit) for --physics holoocean")
    parser.add_argument("--max-thrust-reverse-n", type=float, default=None)
    parser.add_argument("--start-depth-m", type=float, default=3.0,
                        help="mock backend only: spawn depth below the surface")
    parser.add_argument("--seabed-depth-m", type=float, default=30.0,
                        help="mock backend only: depth of the floor the vehicle cannot pass")
    parser.add_argument("--trace-csv", default=None,
                        help="per-frame CSV of the physics state and applied thruster forces")
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--max-seconds", type=float, default=None)
    args = parser.parse_args(argv)

    max_forward_n = args.max_thrust_forward_n
    max_reverse_n = args.max_thrust_reverse_n
    if args.physics == "holoocean":
        if max_forward_n is None:
            max_forward_n = HOLOOCEAN_MAX_THRUST_N
        if max_reverse_n is None:
            max_reverse_n = HOLOOCEAN_MAX_THRUST_N
    else:
        if max_forward_n is None:
            max_forward_n = DEFAULT_MAX_THRUST_FORWARD_N
        if max_reverse_n is None:
            max_reverse_n = DEFAULT_MAX_THRUST_REVERSE_N
    print(f"ardusub_sitl_bridge: per-thruster limits {max_forward_n:g} N forward / "
          f"{max_reverse_n:g} N reverse", flush=True)

    if args.physics == "mock":
        physics = MockPhysics(
            start_position_enu_m=(0.0, 0.0, -abs(args.start_depth_m)),
            seabed_depth_m=args.seabed_depth_m,
        )
    else:
        physics = HoloOceanPhysics(
            listen_host=args.listen_host,
            listen_port=args.listen_port,
            velocity_sensor=args.velocity_sensor or None,
            gravity_mode=args.imu_gravity,
        )

    stats = run_bridge(
        physics,
        control_port=args.control_port,
        bind_host=args.bind_host,
        step_hz=args.step_hz,
        max_frames=args.max_frames,
        max_seconds=args.max_seconds,
        max_thrust_forward_n=max_forward_n,
        max_thrust_reverse_n=max_reverse_n,
        trace_csv_path=args.trace_csv,
    )
    return 0 if stats.frames > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
