"""A bounded, deterministic pilot driver for closing the realtime loop in
simulation-only gate runs (`realtime_gate.py`, later tasks) — explicitly
NOT a production autonomy controller. It only ever reacts to
`/uw/hmi/status` (the operator-assist output this repo already produces —
see `adapters/opencv/src/operator_overlay_renderer.cpp` for the
source/status vocabulary this module's `AssistGuidanceStatus` mirrors), and
never subscribes truth: real search/align/approach behavior is driven by
`OnlineAssistPipeline`'s guidance, same signal a human pilot would watch on
the operator overlay.

Task-sourced gain/bound overrides are a natural future extension (the plan
text says "the task YAML supplies gains, bounds and success hold times") —
`scenario_manifest.py`'s `TaskSpec.success_conditions` today only carries
detection/coverage acceptance thresholds, not pilot control gains, so this
module uses its own documented in-module defaults rather than inventing new
TaskSpec fields outside this task's scope.
"""
from __future__ import annotations

import dataclasses
from typing import List, Optional

from uw_holoocean_adapter.scenario_manifest import TaskSpec

_THRUSTER_COUNT = 8  # [4 vertical, 4 horizontal], matching record_session.py's _default_command() layout

_MAX_STALENESS_S = 0.5

_YAW_GAIN = 60.0  # raw thruster units per radian of bearing/lateral-offset error
_FORWARD_GAIN = 8.0  # raw thruster units per meter of range/advance error
_STANDOFF_RANGE_M = 3.0  # point-target: hold this far back once aligned, rather than colliding with it
_CRUISE_ADVANCE = 15.0  # path-following: steady forward thrust while tracking a path
_SONAR_ONLY_GAIN_SCALE = 0.4  # conservative search mode during visual loss (source == "SONAR")


@dataclasses.dataclass(frozen=True)
class AssistGuidanceStatus:
    """One `/uw/hmi/status` snapshot, already parsed — Task 4's real ROS2
    gateway is what will eventually build this from live JSON; this module
    only consumes it."""

    guidance_valid: bool
    source: str  # "ACOUSTIC_OPTIC" | "VISUAL" | "SONAR" | "" (no active track)
    age_s: float
    bearing_rad: Optional[float] = None
    range_m: Optional[float] = None
    path_lateral_offset_m: Optional[float] = None


class ScriptedPilot:
    def __init__(self, task: TaskSpec):
        self._task = task

    def subscriptions(self) -> tuple:
        return ("/uw/hmi/status",)

    def command(self, status: AssistGuidanceStatus) -> List[float]:
        if not status.guidance_valid or status.age_s > _MAX_STALENESS_S:
            return [0.0] * _THRUSTER_COUNT

        gain_scale = _SONAR_ONLY_GAIN_SCALE if status.source == "SONAR" else 1.0

        if status.bearing_rad is not None and status.range_m is not None:
            yaw_correction = _YAW_GAIN * gain_scale * status.bearing_rad
            forward_thrust = _FORWARD_GAIN * gain_scale * (status.range_m - _STANDOFF_RANGE_M)
        elif status.path_lateral_offset_m is not None:
            yaw_correction = _YAW_GAIN * gain_scale * status.path_lateral_offset_m
            forward_thrust = _CRUISE_ADVANCE * gain_scale
        else:
            return [0.0] * _THRUSTER_COUNT

        vertical = [0.0, 0.0, 0.0, 0.0]
        # Differential yaw across a [front-right, front-left, rear-right,
        # rear-left]-style 4-thruster horizontal layout: same-side pair
        # gets +correction, opposite-side pair gets -correction, so net
        # forward thrust is preserved while the vehicle also yaws.
        horizontal = [
            forward_thrust + yaw_correction,
            forward_thrust - yaw_correction,
            forward_thrust + yaw_correction,
            forward_thrust - yaw_correction,
        ]
        return vertical + horizontal
