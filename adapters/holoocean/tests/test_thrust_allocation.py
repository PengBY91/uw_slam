import numpy as np
import pytest

from uw_holoocean_adapter.thrust_allocation import (
    THRUSTER_COUNT,
    PilotAxes,
    allocate,
    commanded_wrench,
    parse_pilot_axes,
)

LIMIT = 100.0


def _wrench(axes: PilotAxes):
    return commanded_wrench(allocate(axes, limit=LIMIT))


def test_zero_axes_give_zero_thrust():
    assert allocate(PilotAxes.zero(), limit=LIMIT) == [0.0] * THRUSTER_COUNT


def test_surge_is_a_pure_forward_force_from_the_angled_thrusters():
    forces = allocate(PilotAxes(surge=1.0), limit=LIMIT)
    assert forces[0:4] == [0.0] * 4  # verticals untouched
    fx, fy, fz, mx, my, mz = _wrench(PilotAxes(surge=1.0))
    assert fx > 0.0
    assert abs(fy) < 1e-9 and abs(mz) < 1e-9
    assert max(abs(f) for f in forces[4:8]) == pytest.approx(LIMIT)


def test_sway_right_is_a_starboard_force_without_yaw():
    # HoloOcean's BlueROV2 table has starboard at negative y, so "+right"
    # must come out as a -y force with no net yaw or surge.
    fx, fy, fz, mx, my, mz = _wrench(PilotAxes(sway=1.0))
    assert fy < 0.0
    assert abs(fx) < 1e-9 and abs(mz) < 1e-9


def test_yaw_right_is_a_clockwise_torque_without_translation():
    fx, fy, fz, mx, my, mz = _wrench(PilotAxes(yaw_rate=1.0))
    assert mz < 0.0  # clockwise from above == -z torque in a z-up frame
    assert abs(fx) < 1e-9 and abs(fy) < 1e-9


def test_heave_down_drives_all_verticals_equally_downward():
    forces = allocate(PilotAxes(heave=1.0), limit=LIMIT)
    assert forces[4:8] == [0.0] * 4
    assert forces[0:4] == pytest.approx([-LIMIT] * 4)
    assert _wrench(PilotAxes(heave=1.0))[2] < 0.0


def test_axes_are_sign_symmetric():
    plus = allocate(PilotAxes(surge=0.4, sway=-0.2, heave=0.3, yaw_rate=0.1), limit=LIMIT)
    minus = allocate(PilotAxes(surge=-0.4, sway=0.2, heave=-0.3, yaw_rate=-0.1), limit=LIMIT)
    assert plus == pytest.approx([-m for m in minus])


def test_combined_command_never_exceeds_limit_and_keeps_direction():
    axes = PilotAxes(surge=1.0, sway=1.0, heave=1.0, yaw_rate=1.0)
    forces = allocate(axes, limit=LIMIT)
    assert max(abs(f) for f in forces) <= LIMIT + 1e-9
    # Direction preserved: the wrench is a positive scaling of the unsaturated one.
    unsat = commanded_wrench(allocate(axes, limit=1e9))
    sat = commanded_wrench(forces)
    ratio = sat[np.abs(unsat) > 1e-9] / unsat[np.abs(unsat) > 1e-9]
    assert np.allclose(ratio, ratio[0]) and ratio[0] > 0.0


def test_parse_pilot_axes_rejects_wrong_length_and_clamps_overshoot():
    assert parse_pilot_axes([1.0, 0.0, 0.0]) is None
    assert parse_pilot_axes([0.0] * 5) is None
    assert parse_pilot_axes(["x", 0.0, 0.0, 0.0]) is None
    axes = parse_pilot_axes([1.2, -1.5, 0.25, 0.0])
    assert axes == PilotAxes(1.0, -1.0, 0.25, 0.0)


def test_limit_must_be_positive():
    with pytest.raises(ValueError):
        allocate(PilotAxes.zero(), limit=0.0)
