"""HoloOcean session driver — the actual replacement for ocean_t/src/main.py.

KNOWN LIMITATION (this repo state): the `holoocean` Python package is not
installed on the machine this was developed on (no HoloOcean/Unreal
environment available), so `HoloOceanSession` below cannot be exercised
end-to-end here. The import is deferred and guarded (see `_import_holoocean`)
so the rest of this package (coordinates, canonical_writer,
scenario_randomization, and their tests) work and are tested without
HoloOcean installed. What IS real and tested: the deterministic seeding
discipline, the capture/receive time separation, and the canonical bag
output format — the parts of the ocean_t audit findings (platform
architecture section 22.3) that don't require an actual simulator.

Determinism fix vs ocean_t/src/svin2_pipeline.py: this module accepts a
single `numpy.random.Generator` seeded once from the scenario config and
threads it through explicitly; nothing here calls `numpy.random.seed()` at
runtime (that was the specific bug that broke L2 replay determinism in the
audited code).
"""
from __future__ import annotations

import dataclasses
from typing import Any, Dict, Optional

import numpy as np

from uw_holoocean_adapter.scenario_randomization import ScenarioRandomization


def _import_holoocean():
    try:
        import holoocean  # type: ignore

        return holoocean
    except ImportError as exc:
        raise RuntimeError(
            "the 'holoocean' package is not installed in this environment. "
            "HoloOceanSession requires it; everything else in "
            "uw_holoocean_adapter (coordinates, canonical_writer, "
            "scenario_randomization) works without it."
        ) from exc


@dataclasses.dataclass(frozen=True)
class RawSensorFrame:
    """One tick's worth of raw HoloOcean sensor output, before conversion to
    the canonical schema. Field names match HoloOcean's own sensor keys
    where practical, so the boundary between "what HoloOcean gave us" and
    "what we derived" stays legible."""

    sim_time_s: float
    receive_time_s: float
    sensors: Dict[str, Any]  # raw values keyed by HoloOcean sensor name


class HoloOceanSession:
    """Thin wrapper around a HoloOcean environment: owns the deterministic
    RNG and scenario randomization, exposes `step()` returning a
    RawSensorFrame. Conversion of a RawSensorFrame into canonical
    ObservationHeader-wrapped protobuf messages is intentionally NOT this
    class's job (single-responsibility: this class only knows about
    HoloOcean's API; schema conversion lives in canonical_writer.py call
    sites, which don't need to know anything about HoloOcean)."""

    def __init__(
        self,
        scenario_name: str,
        seed: int,
        randomization: Optional[ScenarioRandomization] = None,
    ):
        self._holoocean = _import_holoocean()
        self._rng = np.random.default_rng(seed)
        self._randomization = randomization or ScenarioRandomization()
        self._env = self._holoocean.make(scenario_name)
        self._tick = 0

    def apply_randomization(self) -> None:
        """Pushes this session's ScenarioRandomization into the HoloOcean
        environment. NOT implemented/tested here (see module docstring) —
        the exact HoloOcean API calls for water/sonar parameter injection
        depend on the HoloOcean version and scene, and must be filled in
        against a real HoloOcean install."""
        raise NotImplementedError(
            "HoloOceanSession.apply_randomization: fill in against a real "
            "HoloOcean environment; see ScenarioRandomization for the "
            "parameter space this must cover."
        )

    def step(self, action: Optional[Any] = None) -> RawSensorFrame:
        import time as _time

        state = self._env.tick(action) if action is not None else self._env.tick()
        sim_time_s = self._tick * (1.0 / self._env.ticks_per_sec)
        self._tick += 1
        return RawSensorFrame(sim_time_s=sim_time_s, receive_time_s=_time.time(), sensors=state)

    def close(self) -> None:
        self._env.close()
