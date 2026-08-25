import pytest

from uw_holoocean_adapter.pilot_command_model import PilotCommandModel


def test_pilot_thrusters_apply_deadzone_saturation_and_delay():
    model = PilotCommandModel(limit=100.0, deadzone=5.0, time_constant_s=0.15)
    assert model.step([4.0] * 8, dt_s=0.01) == pytest.approx([0.0] * 8)
    first = model.step([120.0] * 8, dt_s=0.01)
    assert all(0.0 < value < 100.0 for value in first)
    assert all(value <= 100.0 for value in model.step([120.0] * 8, dt_s=2.0))


def test_deadzone_boundary_is_exclusive():
    model = PilotCommandModel(limit=100.0, deadzone=5.0, time_constant_s=0.15)
    below = model.step([4.999], dt_s=10.0)
    assert below == pytest.approx([0.0])

    model = PilotCommandModel(limit=100.0, deadzone=5.0, time_constant_s=0.15)
    at_or_above = model.step([5.0], dt_s=10.0)
    assert at_or_above[0] == pytest.approx(5.0)


def test_negative_commands_saturate_symmetrically():
    model = PilotCommandModel(limit=100.0, deadzone=5.0, time_constant_s=0.15)
    result = model.step([-500.0], dt_s=10.0)
    assert result[0] == pytest.approx(-100.0)


def test_channel_count_can_change_between_calls():
    model = PilotCommandModel(limit=100.0, deadzone=5.0, time_constant_s=0.15)
    model.step([50.0] * 8, dt_s=0.5)
    result = model.step([50.0] * 4, dt_s=10.0)
    assert len(result) == 4


@pytest.mark.parametrize("bad_kwargs", [
    {"limit": 0.0, "deadzone": 0.0, "time_constant_s": 0.1},
    {"limit": 10.0, "deadzone": 10.0, "time_constant_s": 0.1},
    {"limit": 10.0, "deadzone": -1.0, "time_constant_s": 0.1},
    {"limit": 10.0, "deadzone": 1.0, "time_constant_s": 0.0},
])
def test_rejects_invalid_construction(bad_kwargs):
    with pytest.raises(ValueError):
        PilotCommandModel(**bad_kwargs)
