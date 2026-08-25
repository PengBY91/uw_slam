import pathlib

import pytest

from uw_holoocean_adapter.scenario_manifest import load_realtime_manifest
from uw_holoocean_adapter.scripted_pilot import AssistGuidanceStatus, ScriptedPilot

REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
SCENARIOS_DIR = REPO_ROOT / "adapters" / "holoocean" / "scenarios"
BASE_SCENARIO = SCENARIOS_DIR / "blue_rov_aid_sv1213_base.json"
SEARCH_TASK = SCENARIOS_DIR / "aquaculture_search.yaml"
STRUCTURE_TASK = SCENARIOS_DIR / "structure_inspection.yaml"


def search_task_spec():
    return load_realtime_manifest(BASE_SCENARIO, SEARCH_TASK).task


def structure_task_spec():
    return load_realtime_manifest(BASE_SCENARIO, STRUCTURE_TASK).task


def make_assist_status(**overrides):
    defaults = dict(
        guidance_valid=True,
        source="ACOUSTIC_OPTIC",
        age_s=0.0,
        bearing_rad=None,
        range_m=None,
        path_lateral_offset_m=None,
    )
    defaults.update(overrides)
    return AssistGuidanceStatus(**defaults)


def test_scripted_pilot_uses_assist_only_and_stops_on_invalid_guidance():
    pilot = ScriptedPilot(search_task_spec())
    assert "/uw/sim/ground_truth" not in pilot.subscriptions()
    assert pilot.command(make_assist_status(guidance_valid=False)) == pytest.approx([0.0] * 8)
    assert pilot.command(make_assist_status(bearing_rad=0.3, range_m=6.0))[4] != 0.0


def test_subscriptions_is_exactly_hmi_status():
    pilot = ScriptedPilot(search_task_spec())
    assert pilot.subscriptions() == ("/uw/hmi/status",)


def test_stale_guidance_commands_zero():
    pilot = ScriptedPilot(search_task_spec())
    stale = make_assist_status(bearing_rad=0.3, range_m=6.0, age_s=0.6)
    assert pilot.command(stale) == pytest.approx([0.0] * 8)


def test_no_track_data_at_all_commands_zero():
    pilot = ScriptedPilot(search_task_spec())
    empty = make_assist_status()
    assert pilot.command(empty) == pytest.approx([0.0] * 8)


def test_negative_bearing_turns_the_opposite_way():
    pilot = ScriptedPilot(search_task_spec())
    positive = pilot.command(make_assist_status(bearing_rad=0.3, range_m=6.0))
    negative = pilot.command(make_assist_status(bearing_rad=-0.3, range_m=6.0))
    # Same range (so the same forward-thrust term on both), opposite-sign
    # bearing error -> the yaw-differential term flips sign, so the
    # negative-bearing command's first horizontal channel must be strictly
    # less than the positive-bearing one's.
    assert negative[4] < positive[4]


def test_sonar_only_search_is_more_conservative_than_visual():
    pilot = ScriptedPilot(search_task_spec())
    visual = pilot.command(make_assist_status(source="VISUAL", bearing_rad=0.3, range_m=6.0))
    sonar = pilot.command(make_assist_status(source="SONAR", bearing_rad=0.3, range_m=6.0))
    assert abs(sonar[4]) < abs(visual[4])


def test_path_lateral_offset_drives_yaw_correction():
    pilot = ScriptedPilot(structure_task_spec())
    status = make_assist_status(source="VISUAL", path_lateral_offset_m=0.8)
    command = pilot.command(status)
    assert command[4] != 0.0
    assert command != [0.0] * 8
