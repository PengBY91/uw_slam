"""HoloOcean session driver — the actual replacement for ocean_t/src/main.py.

UPDATE: exercised against a real HoloOcean 2.3.0 install (native Windows —
WSL2 can't render, see the platform architecture notes on that) via a
throwaway test script, not yet through this class directly. That test run
is what caught the two real bugs fixed here (see below); this class itself
still hasn't been driven end-to-end, so treat it as "fixed against known
issues, not yet proven" rather than fully verified. `_import_holoocean`'s
deferred/guarded import still means the rest of this package (coordinates,
canonical_writer, scenario_randomization, and their tests) works and is
tested without HoloOcean installed, on any machine including this one.

Four real bugs found and fixed by that real-install test run (the first
two via a throwaway script, the last two via record_session.py's first
actual recording attempt), none caught by this module's own
(HoloOcean-absent) test coverage:
  - `step()` called `self._env.tick(action)` when an action was supplied.
    HoloOcean's `tick(n)` takes a TICK COUNT, not an action —
    `TypeError: 'list' object cannot be interpreted as an integer` the
    moment a real action list was passed. The action-applying call is
    `env.step(action)`; `env.tick()` (no action) is for advancing without
    supplying a new command. Fixed below.
  - `__init__` never called `self._env.reset()`. HoloOcean requires
    `reset()` on a freshly created environment before the first
    tick/step — per HoloOcean's own docs, confirmed by hitting exactly
    that failure mode against the real install. Fixed below.
  - `step()` read `self._env.ticks_per_sec`, which doesn't exist —
    `HoloOceanEnvironment` only has the private `_ticks_per_sec` (no
    public getter; `set_ticks_per_sec()` is a setter with no matching
    getter). Fixed below by reading the private attribute directly, with
    a comment explaining why there's no cleaner option.
  - `close()` called `self._env.close()`, which also doesn't exist.
    Checking external_repos/HoloOcean/client/src/holoocean/environments.py
    (read-only reference) directly: `HoloOceanEnvironment` only exposes
    cleanup through the context-manager protocol
    (`__enter__`/`__exit__`, its own source comments call this "Context
    manager APIs") — no public `close()`. Fixed below by calling
    `self._env.__exit__(None, None, None)`.

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
        self._env.reset()
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

        state = self._env.step(action) if action is not None else self._env.tick()
        # HoloOceanEnvironment has no public ticks_per_sec accessor — only
        # the private `_ticks_per_sec` (set from the scenario config or
        # defaulted to 30) and a set_ticks_per_sec() *setter* with no
        # matching getter. Confirmed by hitting `AttributeError:
        # 'HoloOceanEnvironment' object has no attribute 'ticks_per_sec'`
        # against the real install.
        sim_time_s = self._tick * (1.0 / self._env._ticks_per_sec)
        self._tick += 1
        return RawSensorFrame(sim_time_s=sim_time_s, receive_time_s=_time.time(), sensors=state)

    def close(self) -> None:
        # HoloOceanEnvironment has no public close() either (confirmed the
        # same way as ticks_per_sec above) — cleanup is only exposed
        # through the context-manager protocol (`with holoocean.make(...)
        # as env:`), which is what its own source calls "Context manager
        # APIs" in a comment, i.e. the one bit of the exit path actually
        # meant to be called from outside the class.
        self._env.__exit__(None, None, None)

    def spawn_prop(
        self,
        prop_type: str,
        location: Any,
        rotation: Any = (0.0, 0.0, 0.0),
        scale: Any = (1.0, 1.0, 1.0),
        material: str = "white",
        tag: Optional[str] = None,
    ) -> None:
        """Forwards to HoloOceanEnvironment.spawn_prop() (see
        external_repos/HoloOcean/client/src/holoocean/environments.py,
        read-only reference). Used by calibrate_camera.py to place a
        known-size calibration target; not exercised by record_session.py.
        Spawned props do not persist across env.reset(), so callers must
        spawn after this session's __init__ (which already reset once) and
        before closing."""
        self._env.spawn_prop(
            prop_type,
            location=location,
            rotation=rotation,
            scale=scale,
            material=material,
            tag=tag,
        )

    def teleport_agent(self, location: Any, rotation: Any, agent_name: Optional[str] = None) -> None:
        """Forwards to Agent.set_physics_state() (NOT plain teleport()) for
        the named agent (or the only agent in the scenario if `agent_name`
        is omitted — most single-AUV scenarios like OpenWater-HoveringCamera
        only have one). Deliberately zeroes velocity/angular_velocity too —
        confirmed the hard way against a real HoloOcean install: plain
        teleport() only sets position/rotation and leaves any existing
        velocity untouched, so a HoveringAUV that picked up drift from
        gravity/buoyancy between two calibrate_camera.py poses (no thruster
        command is ever sent — session.step() is called with no action)
        carried that momentum straight through the next teleport, and by
        the ~4th-9th pose had drifted meters off target during the
        settle/wait ticks, taking the calibration checkerboard clean out of
        frame. HoloOcean has no public API to move a sensor independently
        of its agent, so this moves the whole rig; the camera moves with it
        via its fixed mounting extrinsic."""
        agents = self._env.agents
        if agent_name is None:
            if len(agents) != 1:
                raise ValueError(
                    f"scenario has {len(agents)} agents ({sorted(agents)}); "
                    "pass agent_name explicitly to disambiguate"
                )
            agent_name = next(iter(agents))
        agents[agent_name].set_physics_state(location, rotation, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
