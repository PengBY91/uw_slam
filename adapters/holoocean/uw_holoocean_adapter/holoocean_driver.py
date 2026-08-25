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

Realtime closed-loop Task 2 update: `__init__` no longer calls
`self._env.reset()` — reading `HoloOceanEnvironment.__init__`'s own source
directly confirmed it already calls `self.reset()` before returning from
`holoocean.make()`, so the old explicit reset was a real double-reset bug,
just one the four bugs above didn't happen to expose. `__init__` also now
accepts either a `scenario_cfg` dict (the realtime BlueROV manifest path,
see `scenario_manifest.py`) or a legacy bare `scenario_name` string (the
path `record_session.py`/`calibrate_camera.py` already use against
hardware-verified HoloOcean built-in scenarios) — dispatched by
`isinstance`, both are real independently-supported forms of
`holoocean.make()`. `apply_randomization()` is implemented for real and
called automatically at construction time instead of raising
`NotImplementedError`. And Python's global `random` module state is now
saved/seeded/restored around the session's lifetime in addition to the
owned `numpy.random.Generator` above — HoloOcean's own internals may reach
into the global `random` module in ways this repo doesn't control, so
leaving it unseeded would reintroduce the same class of non-determinism
the numpy Generator was already threaded through to avoid. This is
deliberately scoped to construction→close (seed once, restore once), not a
mid-run reseed — the pattern CLAUDE.md's "已经踩过的坑" section flags as the
actual bug class to avoid.
"""
from __future__ import annotations

import copy
import dataclasses
import random
from typing import Any, Dict, Optional

import numpy as np

from uw_holoocean_adapter.scenario_randomization import ScenarioRandomization

_NOMINAL_FLASHLIGHT_INTENSITY = 5000.0  # HoloOcean's own turn_on_flashlight() default


def _prepare_scenario_cfg(scenario_cfg: Dict[str, Any], randomization: ScenarioRandomization) -> Dict[str, Any]:
    """Returns a deep copy of `scenario_cfg` with `randomization.sonar`'s
    speckle/range-noise/sound-speed axes applied to every ImagingSonar
    sensor's `configuration` block. These are construction-time HoloOcean
    sensor parameters (baked into the scenario the server loads), not
    runtime commands, so they must land in the dict passed to
    `holoocean.make()` rather than be pushed via a post-construction API
    call. `AddSigma`/`MultiPath` are left untouched (no corresponding
    ScenarioRandomization field exists yet) rather than overwritten with a
    fabricated value."""
    prepared = copy.deepcopy(scenario_cfg)
    sonar = randomization.sonar
    for agent in prepared.get("agents", []):
        for sensor in agent.get("sensors", []):
            if sensor.get("sensor_type") != "ImagingSonar":
                continue
            config = sensor.setdefault("configuration", {})
            config["MultSigma"] = sonar.speckle_sigma
            config["RangeSigma"] = sonar.range_noise_sigma_m
            if "WaterSpeedSound" in config:
                config["WaterSpeedSound"] = config["WaterSpeedSound"] * sonar.sound_speed_scale
    return prepared


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
        scenario_cfg: Dict[str, Any] | str,
        seed: int,
        randomization: ScenarioRandomization = ScenarioRandomization(),
    ):
        self._holoocean = _import_holoocean()
        self._saved_random_state = random.getstate()
        random.seed(seed)
        self._rng = np.random.default_rng(seed)
        self._randomization = randomization
        if isinstance(scenario_cfg, dict):
            prepared_cfg = _prepare_scenario_cfg(scenario_cfg, randomization)
            self._env = self._holoocean.make(scenario_cfg=prepared_cfg)
        else:
            self._env = self._holoocean.make(scenario_cfg)
        self._tick = 0
        self.apply_randomization()

    def apply_randomization(self) -> None:
        """Pushes this session's ScenarioRandomization into the HoloOcean
        environment via real, post-construction command APIs (sonar
        construction-time parameters are handled separately, before
        `make()`, by `_prepare_scenario_cfg` — see its docstring). Called
        automatically at the end of `__init__`; exposed as a public method
        too in case a caller wants to re-push after mutating
        `self._randomization` mid-session (not currently exercised by any
        caller in this repo, but harmless — every call here is idempotent,
        not incremental)."""
        visual = self._randomization.visual
        self._env.water_fog(visual.turbidity)
        # No ScenarioRandomization field maps directly onto an RGB water
        # tint; this is a coarse, undocumented-by-spec placeholder (murkier
        # water skews darker/greener as backscatter rises) rather than a
        # value any test pins down numerically.
        self._env.water_color(
            0.05,
            max(0.0, 0.55 - 0.25 * visual.backscatter_gain),
            max(0.0, 0.65 - 0.15 * visual.backscatter_gain),
        )

        agent_name = self._try_resolve_agent_name()
        if agent_name is not None:
            # Baseline neutral current — ScenarioRandomization has no
            # current-velocity axis yet; Task 5's fault injector is where
            # `set_ocean_currents` gets driven with real, non-zero values
            # (see the realtime closed-loop plan's Task 5: "route current
            # changes through set_ocean_currents"). Calling it here with a
            # zero vector still satisfies this task's "apply ... after
            # creation" requirement without fabricating a randomization
            # axis that doesn't exist yet.
            self._env.set_ocean_currents(agent_name, [0.0, 0.0, 0.0])

        scenario = getattr(self._env, "_scenario", None)
        if isinstance(scenario, dict):
            for flashlight in scenario.get("flashlight", []):
                name = flashlight.get("flashlight_name", "flashlight1")
                base_intensity = flashlight.get("intensity", _NOMINAL_FLASHLIGHT_INTENSITY)
                self._env.turn_on_flashlight(name, intensity=base_intensity * visual.illumination_scale)

    def _try_resolve_agent_name(self) -> Optional[str]:
        try:
            return self._resolve_agent_name()
        except ValueError:
            return None

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
        try:
            # HoloOceanEnvironment has no public close() either (confirmed
            # the same way as ticks_per_sec above) — cleanup is only
            # exposed through the context-manager protocol (`with
            # holoocean.make(...) as env:`), which is what its own source
            # calls "Context manager APIs" in a comment, i.e. the one bit
            # of the exit path actually meant to be called from outside the
            # class.
            self._env.__exit__(None, None, None)
        finally:
            random.setstate(self._saved_random_state)

    def spawn_prop(
        self,
        prop_type: str,
        location: Any,
        rotation: Any = (0.0, 0.0, 0.0),
        scale: Any = (1.0, 1.0, 1.0),
        sim_physics: bool = False,
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
            sim_physics=sim_physics,
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
        agent_name = self._resolve_agent_name(agent_name)
        self._env.agents[agent_name].set_physics_state(location, rotation, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

    def _resolve_agent_name(self, agent_name: Optional[str] = None) -> str:
        agents = self._env.agents
        if agent_name is not None:
            return agent_name
        if len(agents) != 1:
            raise ValueError(
                f"scenario has {len(agents)} agents ({sorted(agents)}); "
                "pass agent_name explicitly to disambiguate"
            )
        return next(iter(agents))
