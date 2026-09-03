"""Setpoint-level pilot axes -> HoloOcean BlueROV2 Heavy thruster forces
(PREP-C-02, docs/ROV平台到货前准备工作规格-2026-09-02.md D-3).

On the real vehicle ArduSub owns the thrusters; the only command surface is
MAVLink MANUAL_CONTROL / SET_POSITION_TARGET_*. So the cross-language
command contract (`schemas/proto/uw/domain/command.proto` PilotCommand) is
defined at the axis level -- surge / sway / heave / yaw_rate in [-1, 1] --
and turning that into 8 per-thruster forces is a SIMULATION-BACKEND detail
that lives here, not something any pilot or assist code should do itself.

Thruster geometry is HoloOcean's own BlueROV2 table (holoocean/agents.py,
`thruster_d` / `thruster_p`, marked there as "provided for convenience, may
not be correct -- check the C++"). Because the allocation is computed from
that same table (pseudo-inverse of the wrench matrix), the commanded wrench
is self-consistent with whatever HoloOcean actually integrates as long as the
table matches the engine. The absolute SIGN of each axis as seen on screen
(does +sway really move the vehicle to starboard in the rendered world?) is
exactly what PREP-A-03 / A-12 verify in the running simulator -- treat the
`_AXIS_SIGN` constants as the one place to flip if that check fails.

HoloOcean thruster order (agents.py BlueROV2 docstring):
  [Vertical Front Starboard, Vertical Front Port, Vertical Back Port,
   Vertical Back Starboard, Angled Front Starboard, Angled Front Port,
   Angled Back Port, Angled Back Starboard]
In that table starboard positions have negative y, i.e. the body frame is
x forward, y port (left), z up -- so a "+right" sway is a -y force and a
"+clockwise-from-above" yaw is a -z torque.
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

# Unit thrust directions and positions (m) per thruster, HoloOcean frame.
_THRUSTER_D = np.array(
    [
        [0.0, 0.0, 1.0],
        [0.0, 0.0, 1.0],
        [0.0, 0.0, 1.0],
        [0.0, 0.0, 1.0],
        [np.cos(3 * np.pi / 4), np.sin(3 * np.pi / 4), 0.0],
        [np.cos(-3 * np.pi / 4), np.sin(-3 * np.pi / 4), 0.0],
        [np.cos(np.pi / 4), np.sin(np.pi / 4), 0.0],
        [np.cos(-np.pi / 4), np.sin(-np.pi / 4), 0.0],
    ]
)
_THRUSTER_P = np.array(
    [
        [-0.12, -0.218, 0.0],
        [-0.12, 0.218, 0.0],
        [0.12, 0.218, 0.0],
        [0.12, -0.218, 0.0],
        [-0.156, -0.111, 0.085],
        [-0.156, 0.111, 0.085],
        [0.156, 0.111, 0.085],
        [0.156, -0.111, 0.085],
    ]
)

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
