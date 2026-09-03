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
from uw_holoocean_adapter.thrust_allocation import PilotAxes, allocate

# Full-scale thrust a PilotAxes value of 1.0 maps to when the caller asks for
# raw thruster forces via command(); matches blue_rov_aid_sv1213_base.json's
# pilot_command_model.limit. The gains below are still expressed in those raw
# units (their original calibration) and are normalised by this on the way
# out, so PilotAxes stays in [-1, 1] (PREP-C-02 setpoint-level contract).
_AXIS_FULL_SCALE = 100.0

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

    def pilot_axes(self, status: AssistGuidanceStatus) -> PilotAxes:
        """The setpoint-level command (PREP-C-02): what a pilot station or
        the MAVLink adapter consumes. Bearing is body-FLU (positive = target
        to the left), so steering toward it is a NEGATIVE yaw_rate
        (+yaw_rate == clockwise from above == turn right)."""
        if not status.guidance_valid or status.age_s > _MAX_STALENESS_S:
            return PilotAxes.zero()

        gain_scale = _SONAR_ONLY_GAIN_SCALE if status.source == "SONAR" else 1.0

        if status.bearing_rad is not None and status.range_m is not None:
            yaw_correction = _YAW_GAIN * gain_scale * status.bearing_rad
            forward_thrust = _FORWARD_GAIN * gain_scale * (status.range_m - _STANDOFF_RANGE_M)
        elif status.path_lateral_offset_m is not None:
            yaw_correction = _YAW_GAIN * gain_scale * status.path_lateral_offset_m
            forward_thrust = _CRUISE_ADVANCE * gain_scale
        else:
            return PilotAxes.zero()

        return PilotAxes.clamped(
            surge=forward_thrust / _AXIS_FULL_SCALE,
            sway=0.0,
            heave=0.0,
            yaw_rate=-yaw_correction / _AXIS_FULL_SCALE,
        )

    def command(self, status: AssistGuidanceStatus) -> List[float]:
        """Raw HoloOcean thruster forces for the simulation backend --
        `pilot_axes()` run through the BlueROV2 Heavy allocation. Kept for
        sim-only callers; production paths carry PilotAxes / PilotCommand."""
        return allocate(self.pilot_axes(status), limit=_AXIS_FULL_SCALE)
