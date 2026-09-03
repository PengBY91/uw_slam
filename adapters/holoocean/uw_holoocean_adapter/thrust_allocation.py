"""Setpoint-level pilot axes -> HoloOcean BlueROV2 Heavy thruster forces
(PREP-C-02, docs/ROV平台到货前准备工作规格-2026-09-02.md D-3).

On the real vehicle ArduSub owns the thrusters; the only command surface is
MAVLink MANUAL_CONTROL / SET_POSITION_TARGET_*. So the cross-language
command contract (`schemas/proto/uw/domain/command.proto` PilotCommand) is
defined at the axis level -- surge / sway / heave / yaw_rate in [-1, 1] --
and turning that into 8 per-thruster forces is a SIMULATION-BACKEND detail
that lives here, not something any pilot or assist code should do itself.

Thruster geometry is read from HoloOcean's ENGINE source
(holoocean-engine `Source/Holodeck/Agents/{Public/BlueROV2.h,
Private/BlueROV2.cpp}`), NOT from the `thruster_d`/`thruster_p` table in the
Python client's `holoocean/agents.py`.

That distinction is load-bearing, not pedantry. The Python table is marked
upstream as "provided for convenience, may not be correct -- check the C++",
and checking the C++ (2026-09-03, PREP-A-05) showed it is in fact wrong:

  * every position has its x sign flipped relative to the engine;
  * the angled thrusters' directions are wrong -- the Python table has the
    front pair pushing aft (-x) and the back pair forward (+x), while the
    engine pushes ALL FOUR forward (+x), with the port/starboard split
    carried entirely by y;
  * as a result the Python table is not even yaw-symmetric: it gives the
    back pair a sixth of the front pair's yaw moment arm (0.032 vs 0.189 m),
    where the engine gives all four the same 0.180 m.

Since `allocate()` is the pseudo-inverse of this matrix, using the Python
table produced an allocation that did not match what the engine integrates
-- most visibly, a pure surge command came out as ZERO net force in the
engine, because the pattern that is "forward" under the Python directions is
a null vector under the engine's. See
adapters/holoocean/docs/ardusub-sitl-bridge-feasibility.md.

The engine works in UE's left-handed frame (x forward, y right, z up) in
centimetres, and converts forces with `ConvertLinearVector(v, ClientToUE)`,
which reverses y. The table below is therefore already converted to the
client frame this repo uses everywhere: x forward, y PORT (left), z up,
metres -- so a "+right" sway is a -y force and a "+clockwise-from-above" yaw
is a -z torque, which is what `_AXIS_SIGN` encodes.

Thruster order is HoloOcean's own (agents.py BlueROV2 docstring):
  [Vertical Front Starboard, Vertical Front Port, Vertical Back Port,
   Vertical Back Starboard, Angled Front Starboard, Angled Front Port,
   Angled Back Port, Angled Back Starboard]
Note that the engine's positions do not match those names either (its
"front" verticals sit at +x while the Python table put them at -x); the
names are kept only because they are the published action-space order.
"""
from __future__ import annotations

import dataclasses
from typing import List, Sequence

import numpy as np

THRUSTER_COUNT = 8
THRUSTER_NAMES = (
    "vertical_front_starboard",
    "vertical_front_port",
    "vertical_back_port",
    "vertical_back_starboard",
    "angled_front_starboard",
    "angled_front_port",
    "angled_back_port",
    "angled_back_starboard",
)
_VERTICAL = (0, 1, 2, 3)
_ANGLED = (4, 5, 6, 7)

# Unit thrust directions and positions (m) per thruster, in the client
# frame (x forward, y port, z up), transcribed from the engine source.
#
# BlueROV2.cpp ApplyThrusters(): thrusters 0-3 push +z; 4 and 6 push
# (+x, +y)/sqrt2 and 5 and 7 push (+x, -y)/sqrt2, all in the CLIENT frame
# (the code builds LocalForce there, then calls ConvertLinearVector(...,
# ClientToUE)). All four angled thrusters therefore push FORWARD.
_SQRT_HALF = float(np.sqrt(0.5))
_THRUSTER_D = np.array(
    [
        [0.0, 0.0, 1.0],
        [0.0, 0.0, 1.0],
        [0.0, 0.0, 1.0],
        [0.0, 0.0, 1.0],
        [_SQRT_HALF, _SQRT_HALF, 0.0],
        [_SQRT_HALF, -_SQRT_HALF, 0.0],
        [_SQRT_HALF, _SQRT_HALF, 0.0],
        [_SQRT_HALF, -_SQRT_HALF, 0.0],
    ]
)

# BlueROV2.h `thrusterLocations` (UE cm, left-handed), minus the CenterMass
# that BlueROV2.cpp InitializeAgent() subtracts when `Perfect` is true --
# CenterMass = ((t0 + t2) / 2) with Z taken from t7, i.e. (0, 0, -1.00) cm --
# then converted to the client frame (y negated) and to metres.
_THRUSTER_P = np.array(
    [
        [0.1200, -0.2181, 0.0809],
        [0.1200, 0.2181, 0.0809],
        [-0.1200, 0.2181, 0.0809],
        [-0.1200, -0.2181, 0.0809],
        [0.1562, -0.0988, 0.0000],
        [0.1562, 0.0988, 0.0000],
        [-0.1562, 0.0988, 0.0000],
        [-0.1562, -0.0988, 0.0000],
    ]
)

# The engine clamps every thruster to +/-BR_MAX_THRUST
# (BlueROV2.h: BR_MAX_LIN_ACCEL * 11.5 / 4 = 10 * 11.5 / 4), i.e. its model
# is parameterised by a maximum linear acceleration rather than by T200
# thrust curves. A real T200 at 16 V does 51.5 N forward; commands above
# this limit are silently clipped by the engine, not scaled.
ENGINE_MAX_THRUST_N = 28.75

# PilotCommand axis sense -> HoloOcean-frame wrench sense.
_AXIS_SIGN = {
    "surge": +1.0,  # +forward == +x
    "sway": -1.0,  # +right == -y (y points to port)
    "heave": -1.0,  # +down == -z (z up)
    "yaw_rate": -1.0,  # +clockwise from above == -z torque
}


def wrench_matrix() -> np.ndarray:
    """6x8 matrix mapping per-thruster force (N) to body wrench
    [Fx, Fy, Fz, Mx, My, Mz] in the HoloOcean body frame."""
    B = np.zeros((6, THRUSTER_COUNT))
    for i in range(THRUSTER_COUNT):
        B[0:3, i] = _THRUSTER_D[i]
        B[3:6, i] = np.cross(_THRUSTER_P[i], _THRUSTER_D[i])
    return B


def _clamp(value: float) -> float:
    return max(-1.0, min(1.0, float(value)))


@dataclasses.dataclass(frozen=True)
class PilotAxes:
    """One PilotCommand's four axes, each already clamped to [-1, 1]. Sense
    matches command.proto: +surge forward, +sway right, +heave down,
    +yaw_rate clockwise viewed from above."""

    surge: float = 0.0
    sway: float = 0.0
    heave: float = 0.0
    yaw_rate: float = 0.0

    @staticmethod
    def clamped(surge: float, sway: float, heave: float, yaw_rate: float) -> "PilotAxes":
        return PilotAxes(_clamp(surge), _clamp(sway), _clamp(heave), _clamp(yaw_rate))

    @staticmethod
    def zero() -> "PilotAxes":
        return PilotAxes()

    def as_list(self) -> List[float]:
        return [self.surge, self.sway, self.heave, self.yaw_rate]


def parse_pilot_axes(values: Sequence[float]) -> PilotAxes | None:
    """`/uw/pilot/command` wire shape: exactly four floats
    [surge, sway, heave, yaw_rate]. Any other length is rejected (None) --
    same fail-closed contract as the old thruster topic. Out-of-range values
    are clamped, not rejected: a stick can overshoot 1.0 by a rounding error
    and that must not drop the whole command."""
    values = list(values)
    if len(values) != 4:
        return None
    try:
        return PilotAxes.clamped(*[float(v) for v in values])
    except (TypeError, ValueError):
        return None


# Horizontal allocation: least-squares over the four angled thrusters for the
# planar wrench [Fx, Fy, Mz]; each axis column is normalised so that a
# full-scale single-axis command drives its most-loaded thruster to exactly
# `limit`, which is the natural "100% stick == 100% thruster" scaling.
_B_PLANAR = wrench_matrix()[[0, 1, 5], :][:, list(_ANGLED)]  # 3x4
_PINV_PLANAR = np.linalg.pinv(_B_PLANAR)  # 4x3, columns: unit Fx, unit Fy, unit Mz
_PLANAR_COLUMN_SCALE = 1.0 / np.max(np.abs(_PINV_PLANAR), axis=0)  # per axis


def allocate(axes: PilotAxes, *, limit: float) -> List[float]:
    """Maps PilotAxes to 8 HoloOcean thruster forces in [-limit, limit],
    HoloOcean order. A combined command that would exceed `limit` on any
    thruster is scaled down uniformly (direction preserved) rather than
    clipped per thruster (which would distort the requested wrench)."""
    if limit <= 0:
        raise ValueError(f"limit must be positive, got {limit}")
    planar_request = np.array(
        [
            _AXIS_SIGN["surge"] * axes.surge,
            _AXIS_SIGN["sway"] * axes.sway,
            _AXIS_SIGN["yaw_rate"] * axes.yaw_rate,
        ]
    )
    angled = (_PINV_PLANAR * _PLANAR_COLUMN_SCALE) @ planar_request * limit
    vertical = np.full(4, _AXIS_SIGN["heave"] * axes.heave * limit)

    forces = np.zeros(THRUSTER_COUNT)
    forces[list(_VERTICAL)] = vertical
    forces[list(_ANGLED)] = angled
    peak = float(np.max(np.abs(forces)))
    if peak > limit:
        forces *= limit / peak
    return [float(f) for f in forces]


def commanded_wrench(forces: Sequence[float]) -> np.ndarray:
    """Body wrench [Fx, Fy, Fz, Mx, My, Mz] produced by `forces` -- the
    self-consistency check the tests use."""
    return wrench_matrix() @ np.asarray(forces, dtype=float)
