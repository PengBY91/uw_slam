"""Per-channel thruster command shaping shared by the realtime ROS session
and any pilot (scripted or interactive) publishing to `/uw/pilot/thrusters`.

Models the actuator characteristics `scenario_manifest.py`'s
`ActuatorModelSpec` already validates as scenario metadata (`limit`,
`deadzone`, `time_constant_s`): a raw commanded value below `deadzone`
magnitude is treated as zero, the target is then saturated to
`[-limit, limit]`, and the actual output only approaches that target with a
first-order (exponential) lag — real thrusters do not jump instantaneously
to a new command.
"""
from __future__ import annotations

import math
from typing import List, Sequence


class PilotCommandModel:
    def __init__(self, limit: float, deadzone: float, time_constant_s: float):
        if limit <= 0:
            raise ValueError(f"limit must be positive, got {limit}")
        if not (0.0 <= deadzone < limit):
            raise ValueError(f"deadzone must be within [0, limit), got {deadzone}")
        if time_constant_s <= 0:
            raise ValueError(f"time_constant_s must be positive, got {time_constant_s}")
        self._limit = limit
        self._deadzone = deadzone
        self._time_constant_s = time_constant_s
        self._output: List[float] = []

    @property
    def limit(self) -> float:
        """Saturation limit in raw thruster units -- also the full-scale
        thrust a PilotAxes value of 1.0 allocates to (thrust_allocation.py)."""
        return self._limit

    def _target(self, commanded: float) -> float:
        if abs(commanded) < self._deadzone:
            return 0.0
        return max(-self._limit, min(self._limit, commanded))

    def step(self, commands: Sequence[float], dt_s: float) -> List[float]:
        if dt_s <= 0:
            raise ValueError(f"dt_s must be positive, got {dt_s}")
        if len(self._output) != len(commands):
            self._output = [0.0] * len(commands)

        alpha = 1.0 - math.exp(-dt_s / self._time_constant_s)
        for index, commanded in enumerate(commands):
            target = self._target(commanded)
            self._output[index] += (target - self._output[index]) * alpha
        return list(self._output)
