"""Parses the exact JSON shape `BuildStatusJson`
(src/application/holoocean_realtime_sink.cpp) produces -- these fixtures are
hand-built to match that function field-for-field, not derived from a real
run (this repo has never run the realtime gateway against a real
simulator)."""
import json

import pytest

from uw_holoocean_adapter.hmi_status_bridge import parse_guidance_status, parse_track_observation


def _status_json(**overrides):
    payload = {
        "guidance_valid": True,
        "degradation_reason": "",
        "data_age_ms": 120.0,
        "system_health": {"component_id": "online_assist_pipeline", "status": 0, "reason_code": ""},
        "has_path_lateral_offset": False,
        "path_lateral_offset_m": 0.0,
        "path_offset_sigma_m": 0.0,
        "sensor_health": [],
        "target_tracks": [],
    }
    payload.update(overrides)
    return json.dumps(payload)


def _track(status=2, confidence=0.8, bearing_rad=0.1, has_range=True, range_m=4.5, sources=(1, 2)):
    return {
        "track_id": "t0",
        "class_label": "aquaculture_zone",
        "confidence": confidence,
        "bearing_rad": bearing_rad,
        "has_range": has_range,
        "range_m": range_m,
        "status": status,
        "sources": list(sources),
    }


def test_parse_guidance_status_maps_top_level_fields():
    status = parse_guidance_status(_status_json(guidance_valid=True, data_age_ms=250.0))
    assert status.guidance_valid is True
    assert status.age_s == pytest.approx(0.25)


def test_fused_track_reports_acoustic_optic_source():
    payload = _status_json(target_tracks=[_track(sources=(1, 2))])
    status = parse_guidance_status(payload)
    assert status.source == "ACOUSTIC_OPTIC"
    assert status.bearing_rad == pytest.approx(0.1)
    assert status.range_m == pytest.approx(4.5)


def test_visual_only_track_reports_visual_source():
    payload = _status_json(target_tracks=[_track(sources=(1,))])
    status = parse_guidance_status(payload)
    assert status.source == "VISUAL"


def test_sonar_only_track_reports_sonar_source():
    payload = _status_json(target_tracks=[_track(sources=(2,))])
    status = parse_guidance_status(payload)
    assert status.source == "SONAR"


def test_track_without_range_leaves_range_none():
    payload = _status_json(target_tracks=[_track(has_range=False)])
    status = parse_guidance_status(payload)
    assert status.range_m is None
    assert status.bearing_rad == pytest.approx(0.1)


def test_no_actionable_track_yields_no_bearing_range_or_source():
    # Only a TENTATIVE (status=1) track present -- not CONFIRMED/DEGRADED,
    # must not be steered on.
    payload = _status_json(target_tracks=[_track(status=1)])
    status = parse_guidance_status(payload)
    assert status.source == ""
    assert status.bearing_rad is None
    assert status.range_m is None


def test_highest_confidence_actionable_track_wins():
    low = _track(confidence=0.3, bearing_rad=0.0, sources=(2,))
    high = _track(confidence=0.9, bearing_rad=1.0, sources=(1,))
    payload = _status_json(target_tracks=[low, high])
    status = parse_guidance_status(payload)
    assert status.bearing_rad == pytest.approx(1.0)
    assert status.source == "VISUAL"


def test_path_lateral_offset_only_populated_when_flagged():
    payload = _status_json(has_path_lateral_offset=True, path_lateral_offset_m=0.42)
    status = parse_guidance_status(payload)
    assert status.path_lateral_offset_m == pytest.approx(0.42)

    unset = parse_guidance_status(_status_json(has_path_lateral_offset=False, path_lateral_offset_m=99.0))
    assert unset.path_lateral_offset_m is None


def test_parse_track_observation_carries_confidence_for_scoring():
    payload = _status_json(target_tracks=[_track(confidence=0.77, sources=(1, 2))])
    observation = parse_track_observation(payload)
    assert observation.confidence == pytest.approx(0.77)
    assert observation.source == "ACOUSTIC_OPTIC"
    assert observation.guidance_valid is True
