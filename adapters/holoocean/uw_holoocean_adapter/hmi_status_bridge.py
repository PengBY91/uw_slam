"""Portable (no rclpy) parsing of the `/uw/hmi/status` JSON payload
(`std_msgs/String`, built by `BuildStatusJson` in
`src/application/holoocean_realtime_sink.cpp`) into the two dataclasses
`ScriptedPilot`/`TaskScorer` already consume --
`scripted_pilot.AssistGuidanceStatus` and `task_scorer.AssistTrackObservation`.

Both `scripted_pilot.py` and `task_scorer.py` were written to consume an
already-parsed status ("Task 4's real ROS2 gateway is what will eventually
build this from live JSON; this module only consumes it" -- scripted_pilot.py's
own docstring); this module is that missing piece, factored out on its own
so `pilot_ros_bridge.py` and `scorer_ros_bridge.py` (the two rclpy-owning
wrappers, see their own docstrings for why they're separate) share one
parser instead of two independently-drifting copies.

`AssistSource` values below (1=VISUAL, 2=SONAR) mirror
schemas/proto/uw/domain/target.proto exactly. ASSIST_SOURCE_ACOUSTIC_OPTIC
(3) is declared in the proto but the C++ tracker never actually emits it --
a fused track instead carries both VISUAL and SONAR in its `sources` list
(see src/frontends/target_tracker.cpp's `add_sources` loop) -- so a fused
"ACOUSTIC_OPTIC" label here is derived from that combination, not read off
the wire directly.
"""
from __future__ import annotations

import dataclasses
import json
from typing import Any, Dict, List, Optional

from uw_holoocean_adapter.scripted_pilot import AssistGuidanceStatus
from uw_holoocean_adapter.task_scorer import AssistTrackObservation

_ASSIST_SOURCE_VISUAL = 1
_ASSIST_SOURCE_SONAR = 2

# TARGET_TRACK_STATUS_CONFIRMED / _DEGRADED (schemas/proto/uw/domain/target.proto)
# -- the only statuses worth steering on; TENTATIVE hasn't earned trust yet
# and STALE has already aged out (target_tracker.cpp still reports it so the
# HMI can show it, but a pilot/scorer must not act on it).
_ACTIONABLE_TRACK_STATUSES = (2, 3)


@dataclasses.dataclass(frozen=True)
class ParsedHmiStatus:
    """Every field `AssistGuidanceStatus`/`AssistTrackObservation` need,
    parsed once so `parse_guidance_status`/`parse_track_observation` don't
    each re-walk the JSON and risk picking a different "primary" track."""

    guidance_valid: bool
    age_s: float
    source: str  # "ACOUSTIC_OPTIC" | "VISUAL" | "SONAR" | ""
    confidence: float
    bearing_rad: Optional[float]
    range_m: Optional[float]
    path_lateral_offset_m: Optional[float]


def _source_label(sources: List[int]) -> str:
    has_visual = _ASSIST_SOURCE_VISUAL in sources
    has_sonar = _ASSIST_SOURCE_SONAR in sources
    if has_visual and has_sonar:
        return "ACOUSTIC_OPTIC"
    if has_visual:
        return "VISUAL"
    if has_sonar:
        return "SONAR"
    return ""


def _select_primary_track(tracks: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    """Highest-confidence track among CONFIRMED/DEGRADED candidates. A
    single scalar bearing/range is all `ScriptedPilot`/`TaskScorer` were
    designed to consume (see scripted_pilot.py's AssistGuidanceStatus
    docstring) -- multi-target steering is out of scope for both."""
    candidates = [t for t in tracks if t.get("status") in _ACTIONABLE_TRACK_STATUSES]
    if not candidates:
        return None
    return max(candidates, key=lambda t: float(t.get("confidence", 0.0)))


def parse_hmi_status(json_text: str) -> ParsedHmiStatus:
    payload = json.loads(json_text)
    guidance_valid = bool(payload.get("guidance_valid", False))
    age_s = float(payload.get("data_age_ms", 0.0)) / 1000.0
    path_lateral_offset_m = (
        float(payload["path_lateral_offset_m"]) if payload.get("has_path_lateral_offset") else None
    )

    track = _select_primary_track(payload.get("target_tracks", []))
    if track is None:
        return ParsedHmiStatus(
            guidance_valid=guidance_valid,
            age_s=age_s,
            source="",
            confidence=0.0,
            bearing_rad=None,
            range_m=None,
            path_lateral_offset_m=path_lateral_offset_m,
        )

    return ParsedHmiStatus(
        guidance_valid=guidance_valid,
        age_s=age_s,
        source=_source_label(track.get("sources", [])),
        confidence=float(track.get("confidence", 0.0)),
        bearing_rad=float(track["bearing_rad"]) if "bearing_rad" in track else None,
        range_m=float(track["range_m"]) if track.get("has_range") else None,
        path_lateral_offset_m=path_lateral_offset_m,
    )


def parse_guidance_status(json_text: str) -> AssistGuidanceStatus:
    parsed = parse_hmi_status(json_text)
    return AssistGuidanceStatus(
        guidance_valid=parsed.guidance_valid,
        source=parsed.source,
        age_s=parsed.age_s,
        bearing_rad=parsed.bearing_rad,
        range_m=parsed.range_m,
        path_lateral_offset_m=parsed.path_lateral_offset_m,
    )


def parse_track_observation(json_text: str) -> AssistTrackObservation:
    parsed = parse_hmi_status(json_text)
    return AssistTrackObservation(
        guidance_valid=parsed.guidance_valid,
        source=parsed.source,
        confidence=parsed.confidence,
        bearing_rad=parsed.bearing_rad,
        range_m=parsed.range_m,
        path_lateral_offset_m=parsed.path_lateral_offset_m,
    )
