"""Offline tests for the ArduSub SITL <-> HoloOcean bridge (PREP-A-05).

Everything here runs with no ArduPilot, no HoloOcean and no network: the
SITL wire format is a struct, the thruster correspondence is derivable from
the two published thruster tables, and the frame conversions are matrix
algebra. The thruster geometry these tests derive from is the ENGINE's
(holoocean-engine BlueROV2.h/.cpp), pinned by
test_thruster_geometry_matches_the_engine_source. It is deliberately NOT the
table in the Python client's holoocean/agents.py, which upstream marks "may
not be correct -- check the C++" and which is in fact wrong; building on it
gave a mapping under which a pure surge demand produced zero net force and
yaw/pitch were inverted (docs/ardusub-sitl-bridge-feasibility.md 2.4).

What these tests still cannot establish is that the running engine matches
its own source -- that is what adapters/holoocean/tools/sitl_bridge_check.py
does against a live simulator.
"""
from __future__ import annotations

import json
import math
import struct

import numpy as np
import pytest

from uw_holoocean_adapter import ardusub_sitl_bridge as bridge
from uw_holoocean_adapter.thrust_allocation import _THRUSTER_D, _THRUSTER_P, wrench_matrix

# ArduSub SUB_FRAME_VECTORED_6DOF, AP_Motors6DOF.cpp:158-165.
# Columns: roll, pitch, yaw, throttle, forward, lateral.
ARDUSUB_VECTORED_6DOF = np.array(
    [
        [0, 0, 1, 0, -1, 1],
        [0, 0, -1, 0, -1, -1],
        [0, 0, -1, 0, 1, 1],
        [0, 0, 1, 0, 1, -1],
        [1, -1, 0, -1, 0, 0],
        [-1, -1, 0, -1, 0, 0],
        [1, 1, 0, -1, 0, 0],
        [-1, 1, 0, -1, 0, 0],
    ],
    dtype=float,
)


def make_servo_packet(pwm, frame_rate=50, frame_count=7, channels=16):
    magic = bridge.SERVO_MAGIC_16 if channels == 16 else bridge.SERVO_MAGIC_32
    values = list(pwm) + [0] * (channels - len(pwm))
    return struct.pack(f"<HHI{channels}H", magic, frame_rate, frame_count, *values)


def ardusub_outputs(**axes) -> list:
    """Per-motor outputs ArduSub would produce for the given axis demands."""
    demand = np.array([axes.get(name, 0.0) for name in
                       ("roll", "pitch", "yaw", "throttle", "forward", "lateral")])
    return list(ARDUSUB_VECTORED_6DOF @ demand)


def outputs_to_pwm(outputs) -> list:
    return [
        bridge.DEFAULT_PWM_TRIM_US + o * (bridge.DEFAULT_PWM_MAX_US - bridge.DEFAULT_PWM_TRIM_US)
        for o in outputs
    ]


# --------------------------------------------------------------------------
# SITL wire format
# --------------------------------------------------------------------------
def test_servo_packet_sizes_match_the_c_struct():
    # uint16 + uint16 + uint32 + uint16[N], no padding (the uint32 already
    # lands 4-byte aligned).
    assert bridge._SERVO_PACKET_16.size == 40
    assert bridge._SERVO_PACKET_32.size == 72


def test_decodes_a_16_channel_packet():
    packet = bridge.decode_servo_packet(make_servo_packet([1600, 1400], frame_rate=50, frame_count=9))
    assert packet is not None
    assert packet.frame_rate_hz == 50
    assert packet.frame_count == 9
    assert packet.pwm_us[:2] == (1600, 1400)
    assert len(packet.pwm_us) == 16


def test_decodes_a_32_channel_packet():
    packet = bridge.decode_servo_packet(make_servo_packet([1700], channels=32))
    assert packet is not None
    assert len(packet.pwm_us) == 32


def test_rejects_foreign_datagrams_instead_of_raising():
    # UDP 9002 is a public socket; a stray datagram must not kill the bridge.
    assert bridge.decode_servo_packet(b"") is None
    assert bridge.decode_servo_packet(b"hello from something else") is None
    wrong_magic = struct.pack("<HHI16H", 1234, 50, 1, *([1500] * 16))
    assert bridge.decode_servo_packet(wrong_magic) is None


# --------------------------------------------------------------------------
# PWM -> thruster force
# --------------------------------------------------------------------------
def test_pwm_trim_is_zero_and_the_ends_are_full_scale():
    assert bridge.pwm_to_normalized(1500) == pytest.approx(0.0)
    assert bridge.pwm_to_normalized(1900) == pytest.approx(1.0)
    assert bridge.pwm_to_normalized(1100) == pytest.approx(-1.0)
    assert bridge.pwm_to_normalized(1700) == pytest.approx(0.5)


def test_unassigned_channel_reads_as_neutral_not_full_reverse():
    # SITL emits 0 on channels with no output function assigned, and an ESC
    # that never sees a pulse does nothing rather than reversing.
    assert bridge.pwm_to_normalized(0) == 0.0


def test_pwm_is_clamped_outside_the_configured_range():
    assert bridge.pwm_to_normalized(2500) == pytest.approx(1.0)
    assert bridge.pwm_to_normalized(700) == pytest.approx(-1.0)


def test_forward_and_reverse_thrust_limits_are_asymmetric():
    forces = bridge.pwm_to_thruster_forces([1900] + [1500] * 15)
    assert min(forces) == pytest.approx(-bridge.DEFAULT_MAX_THRUST_FORWARD_N)
    forces = bridge.pwm_to_thruster_forces([1100] + [1500] * 15)
    assert max(forces) == pytest.approx(bridge.DEFAULT_MAX_THRUST_REVERSE_N)


def test_neutral_pwm_produces_no_thrust():
    assert bridge.pwm_to_thruster_forces([1500] * 16) == [0.0] * 8


# --------------------------------------------------------------------------
# The thruster correspondence, re-derived rather than remembered
# --------------------------------------------------------------------------
def test_ardusub_to_holoocean_is_a_bijection():
    assert sorted(bridge.ARDUSUB_TO_HOLOOCEAN) == list(range(8))
    assert len(bridge.ARDUSUB_THRUSTER_SIGN) == 8
    assert set(bridge.ARDUSUB_THRUSTER_SIGN) <= {-1.0, 1.0}


def test_thruster_geometry_matches_the_engine_source():
    """Pins thrust_allocation.py's table to holoocean-engine's C++.

    The table this repo used until 2026-09-03 came from the Python client's
    `holoocean/agents.py`, which upstream marks "may not be correct -- check
    the C++". It was not correct, and because every other test here checks
    allocation SELF-consistency (allocate -> commanded_wrench), none of them
    could see it. This test is the one that would have.

    Engine source: `Source/Holodeck/Agents/Public/BlueROV2.h`
    (`thrusterLocations`, UE cm, left-handed) and `Private/BlueROV2.cpp`
    (`ApplyThrusters` force directions; `InitializeAgent` subtracts a
    CenterMass of (0, 0, -1.00) cm when `Perfect` is true). UE -> client is
    a y negation, cm -> m a factor of 100.
    """
    engine_locations_ue_cm = np.array(
        [
            [12.00, 21.81, 7.09],
            [12.00, -21.81, 7.09],
            [-12.00, -21.81, 7.09],
            [-12.00, 21.81, 7.09],
            [15.62, 9.88, -1.00],
            [15.62, -9.88, -1.00],
            [-15.62, -9.88, -1.00],
            [-15.62, 9.88, -1.00],
        ]
    )
    center_mass_ue_cm = np.array([0.0, 0.0, -1.00])
    relative = engine_locations_ue_cm - center_mass_ue_cm
    expected_p = np.stack([relative[:, 0], -relative[:, 1], relative[:, 2]], axis=1) / 100.0
    assert np.allclose(_THRUSTER_P, expected_p, atol=1e-4)

    root_half = math.sqrt(0.5)
    expected_d = np.array(
        [[0.0, 0.0, 1.0]] * 4
        + [
            [root_half, root_half, 0.0],   # i == 4, i % 2 == 0
            [root_half, -root_half, 0.0],  # i == 5
            [root_half, root_half, 0.0],   # i == 6
            [root_half, -root_half, 0.0],  # i == 7
        ]
    )
    assert np.allclose(_THRUSTER_D, expected_d)


def test_engine_geometry_is_yaw_symmetric():
    """All four angled thrusters must have the same yaw moment arm.

    The old Python-client table gave the back pair 0.032 m against the front
    pair's 0.189 m, which read as "HoloOcean's vehicle has a parasitic yaw
    moment". The engine's real geometry is symmetric; the asymmetry was an
    artifact of the wrong table.
    """
    arms = [
        _THRUSTER_P[i][0] * _THRUSTER_D[i][1] - _THRUSTER_P[i][1] * _THRUSTER_D[i][0]
        for i in range(4, 8)
    ]
    assert all(abs(abs(arm) - abs(arms[0])) < 1e-9 for arm in arms)
    assert abs(arms[0]) == pytest.approx(0.1803, abs=1e-3)


def test_correspondence_matches_both_published_thruster_tables():
    """Re-derives ARDUSUB_TO_HOLOOCEAN from HoloOcean's thruster geometry and
    ArduSub's motor table, so the constant cannot silently drift from either.

    Each HoloOcean thruster's unit wrench is expressed in ArduSub's own axis
    coordinates (roll, pitch, yaw, throttle, forward, lateral) -- moments and
    forces rotated FLU->FRD, with throttle positive up -- and matched against
    ArduSub's factor rows. The match is by DIRECTION, not magnitude: ArduSub's
    table uses unit factors while the real moment arms are 0.03-0.22 m.
    """
    signatures = []
    for i in range(8):
        force = bridge.C_FRD_FLU @ _THRUSTER_D[i]
        moment = bridge.C_FRD_FLU @ np.cross(_THRUSTER_P[i], _THRUSTER_D[i])
        signatures.append(np.array([moment[0], moment[1], moment[2], -force[2], force[0], force[1]]))

    for motor_index in range(8):
        wanted = ARDUSUB_VECTORED_6DOF[motor_index]
        wanted = wanted / np.linalg.norm(wanted)
        best_index, best_cosine = None, 0.0
        for thruster_index, signature in enumerate(signatures):
            unit = signature / np.linalg.norm(signature)
            cosine = float(wanted @ unit)
            if abs(cosine) > abs(best_cosine):
                best_index, best_cosine = thruster_index, cosine
        assert best_index == bridge.ARDUSUB_TO_HOLOOCEAN[motor_index], (
            f"MOT_{motor_index + 1} should drive HoloOcean thruster "
            f"{bridge.ARDUSUB_TO_HOLOOCEAN[motor_index]}, derivation says {best_index}"
        )
        assert math.copysign(1.0, best_cosine) == bridge.ARDUSUB_THRUSTER_SIGN[motor_index]


@pytest.mark.parametrize(
    "axis, expected_axis, expected_sign, label",
    [
        ("forward", 0, +1, "+x is forward"),
        ("lateral", 1, -1, "+right is -y, because y points to port"),
        ("throttle", 2, +1, "+throttle is up, and z is up"),
    ],
)
def test_each_axis_demand_produces_a_pure_force_in_the_engine_geometry(
    axis, expected_axis, expected_sign, label
):
    """The regression that the whole engine-geometry correction exists for.

    Under the old (Python-client) table, `forward` produced exactly ZERO net
    force in the engine -- the pattern that is "forward" under those
    directions is a null vector under the engine's, because the engine
    pushes all four angled thrusters forward. This asserts real force on the
    commanded axis and nothing on the others.
    """
    pwm = outputs_to_pwm(ardusub_outputs(**{axis: 1.0}))
    forces = bridge.pwm_to_thruster_forces(pwm, max_forward_n=50.0, max_reverse_n=50.0)
    wrench = wrench_matrix() @ np.array(forces)
    force = wrench[0:3]
    assert math.copysign(1.0, force[expected_axis]) == expected_sign, label
    assert abs(force[expected_axis]) > 100.0
    for other in range(3):
        if other != expected_axis:
            assert abs(force[other]) < 1e-6
    # And no parasitic moment: a translation demand must not rotate the
    # vehicle. This is what the old table violated most damagingly.
    assert np.max(np.abs(wrench[3:6])) < 1e-6


@pytest.mark.parametrize(
    "axis, moment_index, expected_sign, label",
    [
        ("yaw", 5, -1, "ArduSub yaw-right is a nose-right, i.e. -z, moment in FLU"),
        ("roll", 3, +1, "ArduSub roll-right is +x moment in FLU"),
        ("pitch", 4, -1, "ArduSub pitch demand maps to -y moment in FLU"),
    ],
)
def test_each_moment_demand_produces_a_pure_moment(axis, moment_index, expected_sign, label):
    pwm = outputs_to_pwm(ardusub_outputs(**{axis: 1.0}))
    forces = bridge.pwm_to_thruster_forces(pwm, max_forward_n=50.0, max_reverse_n=50.0)
    wrench = wrench_matrix() @ np.array(forces)
    assert math.copysign(1.0, wrench[moment_index]) == expected_sign, label
    assert abs(wrench[moment_index]) > 1.0
    assert np.max(np.abs(wrench[0:3])) < 1e-6
    for other in (3, 4, 5):
        if other != moment_index:
            assert abs(wrench[other]) < 1e-6


# --------------------------------------------------------------------------
# Frame conversions
# --------------------------------------------------------------------------
def test_frame_matrices_are_proper_rotations_and_self_inverse():
    for matrix in (bridge.C_FRD_FLU, bridge.C_NED_WORLD):
        assert np.linalg.det(matrix) == pytest.approx(1.0)
        assert np.allclose(matrix @ matrix, np.eye(3))


def make_state(**overrides):
    defaults = dict(
        sim_time_s=1.0,
        position_enu_m=np.zeros(3),
        velocity_enu_mps=np.zeros(3),
        rotation_enu_flu=np.eye(3),
        angular_velocity_flu_radps=np.zeros(3),
        specific_force_flu_mps2=np.array([0.0, 0.0, 9.80665]),
    )
    defaults.update(overrides)
    return bridge.VehicleState(**defaults)


def test_level_vehicle_at_rest_reports_identity_attitude_and_accelerometer_up():
    payload = bridge.build_json_state(make_state(), np.zeros(3))
    assert payload["quaternion"] == pytest.approx([1.0, 0.0, 0.0, 0.0])
    # An accelerometer at rest reads +1 g upward, which in FRD is -z.
    assert payload["imu"]["accel_body"] == pytest.approx([0.0, 0.0, -9.80665])


def test_depth_becomes_positive_down():
    payload = bridge.build_json_state(
        make_state(position_enu_m=np.array([0.0, 0.0, -3.0])), np.zeros(3)
    )
    assert payload["position"][2] == pytest.approx(3.0)


def test_horizontal_origin_is_removed_but_depth_is_not():
    """The vertical offset must survive: SITL turns the down component into
    the simulated barometer, which is ArduSub's only depth source."""
    state = make_state(position_enu_m=np.array([10.0, -4.0, -3.0]))
    payload = bridge.build_json_state(state, np.array([10.0, -4.0, 0.0]))
    assert payload["position"][0] == pytest.approx(0.0)
    assert payload["position"][1] == pytest.approx(0.0)
    assert payload["position"][2] == pytest.approx(3.0)


def test_world_x_is_north_so_identity_attitude_is_heading_zero():
    # Deliberately different from ned_conversion.hpp's ENU->NED, which would
    # report a 90-degree heading here. See the module docstring.
    payload = bridge.build_json_state(
        make_state(velocity_enu_mps=np.array([1.0, 0.0, 0.0])), np.zeros(3)
    )
    assert payload["velocity"] == pytest.approx([1.0, 0.0, 0.0])
    assert payload["quaternion"] == pytest.approx([1.0, 0.0, 0.0, 0.0])


def test_body_left_velocity_becomes_east_negative():
    payload = bridge.build_json_state(
        make_state(velocity_enu_mps=np.array([0.0, 1.0, 0.0])), np.zeros(3)
    )
    assert payload["velocity"] == pytest.approx([0.0, -1.0, 0.0])


def test_yaw_left_in_world_becomes_negative_yaw_in_ned():
    yaw = math.radians(30.0)
    rotation = np.array([[math.cos(yaw), -math.sin(yaw), 0.0],
                         [math.sin(yaw), math.cos(yaw), 0.0],
                         [0.0, 0.0, 1.0]])
    payload = bridge.build_json_state(make_state(rotation_enu_flu=rotation), np.zeros(3))
    w, x, y, z = payload["quaternion"]
    assert 2.0 * math.atan2(z, w) == pytest.approx(-yaw, abs=1e-9)


def test_gyro_and_accel_are_rotated_into_frd():
    state = make_state(
        angular_velocity_flu_radps=np.array([0.1, 0.2, 0.3]),
        specific_force_flu_mps2=np.array([1.0, 2.0, 3.0]),
    )
    payload = bridge.build_json_state(state, np.zeros(3))
    assert payload["imu"]["gyro"] == pytest.approx([0.1, -0.2, -0.3])
    assert payload["imu"]["accel_body"] == pytest.approx([1.0, -2.0, -3.0])


def test_quaternion_round_trips_through_the_matrix_conversion():
    rng = np.random.default_rng(20260903)
    for _ in range(20):
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        angle = rng.uniform(-math.pi, math.pi)
        skew = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        rotation = np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * (skew @ skew)
        w, x, y, z = bridge.matrix_to_quaternion_wxyz(rotation)
        recovered = bridge.quaternion_xyzw_to_matrix([x, y, z, w])
        assert np.allclose(recovered, rotation, atol=1e-9)


def test_json_reply_is_newline_framed_and_parses():
    payload = bridge.build_json_state(make_state(), np.zeros(3))
    blob = bridge.encode_json_reply(payload)
    assert blob.startswith(b"\n") and blob.endswith(b"\n")
    assert json.loads(blob.decode("ascii").strip()) == payload
    # Every field ArduPilot's keytable marks required must be present.
    for key in ("timestamp", "imu", "position", "velocity"):
        assert key in payload
    assert set(payload["imu"]) == {"gyro", "accel_body"}


# --------------------------------------------------------------------------
# HoloOcean frame -> VehicleState
# --------------------------------------------------------------------------
def holoocean_sensors(position=(1.0, 2.0, -3.0), accel=(0.0, 0.0, 9.8), gyro=(0.0, 0.0, 0.1)):
    pose = np.eye(4)
    pose[:3, 3] = position
    return {"PoseSensor": pose, "IMUSensor": np.array([list(accel), list(gyro)])}


def test_holoocean_frame_reads_pose_and_imu():
    state = bridge.holoocean_frame_to_state(0.5, holoocean_sensors(), 0.02, velocity_sensor=None)
    assert state.position_enu_m == pytest.approx([1.0, 2.0, -3.0])
    assert state.angular_velocity_flu_radps == pytest.approx([0.0, 0.0, 0.1])
    assert state.sim_time_s == 0.5


def test_holoocean_velocity_falls_back_to_differencing_pose():
    previous = (0.0, np.array([1.0, 2.0, -3.0]))
    state = bridge.holoocean_frame_to_state(
        0.02, holoocean_sensors(position=(1.02, 2.0, -3.0)), 0.02,
        velocity_sensor=None, previous=previous,
    )
    assert state.velocity_enu_mps == pytest.approx([1.0, 0.0, 0.0])


def test_holoocean_velocity_prefers_a_dynamics_sensor_when_the_shape_is_recognised():
    sensors = holoocean_sensors()
    # [accel, velocity, position, ...] -- the velocity row is row 1.
    sensors["DynamicsSensor"] = np.array([[0.0, 0.0, 0.0], [0.5, -0.25, 0.1], [1.0, 2.0, -3.0]])
    state = bridge.holoocean_frame_to_state(0.02, sensors, 0.02)
    assert state.velocity_enu_mps == pytest.approx([0.5, -0.25, 0.1])


def test_holoocean_frame_rejects_missing_or_malformed_sensors():
    with pytest.raises(KeyError):
        bridge.holoocean_frame_to_state(0.0, {"IMUSensor": np.zeros((2, 3))}, 0.02)
    with pytest.raises(KeyError):
        bridge.holoocean_frame_to_state(0.0, {"PoseSensor": np.eye(4)}, 0.02)
    bad_pose = dict(holoocean_sensors())
    bad_pose["PoseSensor"] = np.eye(3)
    with pytest.raises(ValueError):
        bridge.holoocean_frame_to_state(0.0, bad_pose, 0.02)
    bad_imu = dict(holoocean_sensors())
    bad_imu["IMUSensor"] = np.zeros((3, 3))
    with pytest.raises(ValueError):
        bridge.holoocean_frame_to_state(0.0, bad_imu, 0.02)


# --------------------------------------------------------------------------
# Mock physics
# --------------------------------------------------------------------------
def test_mock_at_rest_reads_one_g_upward():
    state = bridge.MockPhysics().step([0.0] * 8, 0.02)
    assert np.linalg.norm(state.specific_force_flu_mps2) == pytest.approx(9.8, abs=0.2)
    assert state.specific_force_flu_mps2[2] > 9.0


def test_mock_quadratic_drag_is_stable_at_full_thrust():
    """The regression this integrator was rewritten for.

    Explicit-Euler quadratic drag diverges at this timestep: at 10 m/s the
    lateral drag deceleration is ~343 m/s^2, so one 20 ms step overshoots by
    ~7 m/s, flips the sign and grows. The vehicle then oscillated at
    +/-10 m/s regardless of command, which read through ArduSub's EKF as an
    inverted lateral axis. The semi-implicit form must stay bounded at a
    physically plausible speed.
    """
    physics = bridge.MockPhysics()
    pwm = outputs_to_pwm(ardusub_outputs(lateral=1.0))
    forces = bridge.pwm_to_thruster_forces(pwm)
    speeds = []
    for _ in range(1000):
        state = physics.step(forces, 0.02)
        speeds.append(float(np.linalg.norm(state.velocity_enu_mps)))
    assert max(speeds) < 3.0
    assert speeds[-1] == pytest.approx(speeds[-2], abs=1e-3)  # settled, not oscillating


def test_mock_stops_at_the_surface_and_at_the_seabed():
    physics = bridge.MockPhysics(start_position_enu_m=(0.0, 0.0, -0.2), seabed_depth_m=5.0)
    for _ in range(500):
        state = physics.step([0.0] * 8, 0.02)  # positively buoyant, rises
    assert state.position_enu_m[2] == pytest.approx(0.0)

    physics = bridge.MockPhysics(start_position_enu_m=(0.0, 0.0, -4.5), seabed_depth_m=5.0)
    pwm = outputs_to_pwm(ardusub_outputs(throttle=-1.0))
    forces = bridge.pwm_to_thruster_forces(pwm)
    for _ in range(500):
        state = physics.step(forces, 0.02)
    assert state.position_enu_m[2] == pytest.approx(-5.0)


def test_mock_righting_moment_returns_the_vehicle_to_level():
    physics = bridge.MockPhysics()
    physics._rotation = np.array([[1.0, 0.0, 0.0],
                                  [0.0, math.cos(0.4), -math.sin(0.4)],
                                  [0.0, math.sin(0.4), math.cos(0.4)]])
    for _ in range(2000):
        state = physics.step([0.0] * 8, 0.02)
    # Body +z should end up pointing back along world +z.
    assert state.rotation_enu_flu[2, 2] > 0.99


def test_mock_is_deterministic():
    forces = bridge.pwm_to_thruster_forces(outputs_to_pwm(ardusub_outputs(forward=0.7, yaw=0.3)))
    def run():
        physics = bridge.MockPhysics()
        for _ in range(300):
            state = physics.step(forces, 0.02)
        return state
    a, b = run(), run()
    assert a.position_enu_m == pytest.approx(b.position_enu_m)
    assert a.velocity_enu_mps == pytest.approx(b.velocity_enu_mps)


# --------------------------------------------------------------------------
# Bridge loop
# --------------------------------------------------------------------------
class FakeSocket:
    """A UDP socket stand-in that replays a fixed script of datagrams."""

    def __init__(self, datagrams):
        self._datagrams = list(datagrams)
        self.sent = []
        self.closed = False

    def settimeout(self, _):
        pass

    def recvfrom(self, _):
        if not self._datagrams:
            raise KeyboardInterrupt
        return self._datagrams.pop(0), ("127.0.0.1", 9002)

    def sendto(self, data, peer):
        self.sent.append((data, peer))

    def close(self):
        self.closed = True


def run_with(datagrams, **kwargs):
    holder = {}
    def factory():
        holder["socket"] = FakeSocket(datagrams)
        return holder["socket"]
    stats = bridge.run_bridge(bridge.MockPhysics(), step_hz=50.0, report_every=0,
                              socket_factory=factory, **kwargs)
    return stats, holder["socket"]


def test_bridge_replies_once_per_servo_packet_to_the_sender():
    packets = [make_servo_packet([1500] * 8, frame_count=n) for n in range(5)]
    stats, sock = run_with(packets)
    assert stats.frames == 5
    assert len(sock.sent) == 5
    assert all(peer == ("127.0.0.1", 9002) for _, peer in sock.sent)
    payload = json.loads(sock.sent[0][0].decode("ascii").strip())
    assert payload["timestamp"] == pytest.approx(0.02)


def test_bridge_ignores_foreign_datagrams_without_replying():
    packets = [b"not a servo packet", make_servo_packet([1500] * 8, frame_count=1)]
    stats, sock = run_with(packets)
    assert stats.foreign_datagrams == 1
    assert stats.frames == 1
    assert len(sock.sent) == 1


def test_bridge_does_not_double_step_a_resent_frame():
    """ArduPilot resends the same frame_count when it has heard nothing for
    10 s. Stepping the physics again would advance the simulation twice for
    one control frame."""
    packets = [
        make_servo_packet([1500] * 8, frame_count=1),
        make_servo_packet([1500] * 8, frame_count=1),  # resend
        make_servo_packet([1500] * 8, frame_count=2),
    ]
    stats, sock = run_with(packets)
    assert stats.stale_frames == 1
    timestamps = [json.loads(blob.decode("ascii").strip())["timestamp"] for blob, _ in sock.sent]
    assert timestamps == pytest.approx([0.02, 0.02, 0.04])


def test_bridge_counts_dropped_frames():
    packets = [
        make_servo_packet([1500] * 8, frame_count=1),
        make_servo_packet([1500] * 8, frame_count=5),
    ]
    stats, _ = run_with(packets)
    assert stats.dropped_frames == 3


def test_bridge_zeroes_the_horizontal_origin_only():
    physics = bridge.MockPhysics(start_position_enu_m=(25.0, -8.0, -3.0))
    holder = {}
    def factory():
        holder["socket"] = FakeSocket([make_servo_packet([1500] * 8, frame_count=1)])
        return holder["socket"]
    bridge.run_bridge(physics, step_hz=50.0, report_every=0, socket_factory=factory)
    payload = json.loads(holder["socket"].sent[0][0].decode("ascii").strip())
    assert payload["position"][0] == pytest.approx(0.0, abs=1e-6)
    assert payload["position"][1] == pytest.approx(0.0, abs=1e-6)
    assert payload["position"][2] == pytest.approx(3.0, abs=0.01)


def test_bridge_reports_a_steady_state_rate_separate_from_the_cumulative_one():
    stats = bridge.BridgeStats(recent_window=3)
    for i in range(10):
        stats.observe(float(i), 0.02 * i)
    assert stats.recent_frame_rate_hz() == pytest.approx(1.0)
    assert stats.recent_realtime_factor() == pytest.approx(0.02)


def test_bridge_writes_a_trace_when_asked(tmp_path):
    trace = tmp_path / "trace.csv"
    packets = [make_servo_packet([1900] + [1500] * 7, frame_count=n) for n in range(3)]
    run_with(packets, trace_csv_path=str(trace))
    lines = trace.read_text(encoding="ascii").strip().splitlines()
    assert lines[0].startswith("frame,sim_time_s,wall_s,px,py,pz,vx,vy,vz,yaw_deg,f0")
    assert len(lines) == 4
    fields = lines[1].split(",")
    assert len(fields) == 10 + 8
