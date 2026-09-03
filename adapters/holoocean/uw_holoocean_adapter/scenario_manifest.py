"""Versioned, validated BlueROV2/AI-D/SV1213 realtime scenario manifests.

Loads a repo-owned scenario JSON (HoloOcean's own `scenario_cfg` shape, plus
two repo-only keys stripped before the file reaches `holoocean.make` — see
`holoocean_scenario_cfg()`) together with a task YAML (arena/structure,
props, target/path truth, success conditions) into one typed, validated
`RealtimeScenarioManifest`. Nothing here talks to HoloOcean or ROS2; this is
pure data loading/validation so it can be unit tested on any machine (see
`adapters/holoocean/README.md`'s "what's real vs not tested here" section).

PREP-A-03 (docs/ROV平台到货前准备工作规格-2026-09-02.md) additions -- all
repo-only keys, all stripped before the dict reaches `holoocean.make`:

- `uw_extends`: a derived scenario file names its base file (path relative
  to the derived file) and only carries the keys it overrides. Top-level
  scalars/lists replace, `uw_metadata` is shallow-merged, and
  `uw_sensor_subset` (a list of sensor names) keeps only those sensors of
  the base agent -- that is how `blue_rov_contract_mono.json` drops the
  future-stereo cameras without duplicating the whole baseline.
- `uw_profiles`: named overlays (`fidelity` / `realtime`, see
  `SIM-PERF-002/003`) of `ticks_per_sec`, `frames_per_sec` and per-sensor
  `Hz`/`configuration` fields. Selected by `load_realtime_manifest(...,
  profile=...)`; `uw_metadata.default_profile` picks the default when the
  caller passes none. A file without `uw_profiles` (the legacy
  `blue_rov_aid_sv1213_base.json`) is used as-is.
- `uw_sonar_modes`: named overlays of the `ImagingSonar` `configuration`
  (the contract sonar's 750 kHz / 1.2 MHz modes); same selection rules via
  `sonar_mode=` and `uw_metadata.default_sonar_mode`.
"""
from __future__ import annotations

import dataclasses
import json
import pathlib
from typing import Any

import yaml

_TRUTH_TOPIC = "/uw/sim/ground_truth"

DEFAULT_ALGORITHM_TOPICS = (
    "/holoocean/auv0/LeftCamera",
    "/holoocean/auv0/RightCamera",
    "/holoocean/auv0/ImagingSonar",
    "/holoocean/auv0/VehicleState",
)

# Sensors every realtime scenario must carry regardless of camera fit-out.
# Cameras are handled separately below: the contract vehicle (PREP-A-03)
# ships with a single gimbal camera (`MainCamera`), the stereo pair
# (`LeftCamera`/`RightCamera`) arrives later, and the legacy AI-D baseline
# has only the pair -- so the rule is "at least one algorithm camera, and
# Left/Right either both present or both absent", never a fixed set.
_REQUIRED_SENSOR_TYPES = {
    "ImagingSonar": "ImagingSonar",
    "VehicleOrientation": "OrientationSensor",
    "IMUSensor": "IMUSensor",
    "DepthSensor": "DepthSensor",
}
ALGORITHM_CAMERA_NAMES = ("MainCamera", "LeftCamera", "RightCamera")
_STEREO_CAMERA_NAMES = ("LeftCamera", "RightCamera")
# Keys this repo adds on top of HoloOcean's own scenario_cfg shape. Every
# one of them must be stripped by `holoocean_scenario_cfg()`.
REPO_ONLY_SCENARIO_KEYS = (
    "uw_metadata",
    "algorithm_topics",
    "uw_extends",
    "uw_sensor_subset",
    "uw_profiles",
    "uw_sonar_modes",
    "_comment",
)
# Fields a profile overlay may touch on a sensor entry.
_PROFILE_SENSOR_KEYS = ("Hz", "configuration")

_REQUIRED_SONAR_CONFIG_KEYS = (
    "Azimuth",
    "Elevation",
    "RangeMin",
    "RangeMax",
    "RangeBins",
    "AzimuthBins",
    "AddSigma",
    "MultSigma",
    "RangeSigma",
    "WaterSpeedSound",
)

_ALLOWED_PROP_TYPES = frozenset({"box", "sphere", "cylinder", "cone"})
# HoloOcean's own HoloOceanEnvironment.spawn_prop() material palette (confirmed
# against a real HoloOcean 2.3.0 install, not guessed — see holoocean_driver.py's
# module docstring for this repo's other real-install-verified findings). Any
# other value raises holoocean.exceptions.HoloOceanException at spawn time;
# earlier drafts of this repo's task YAML files used invented names like
# "orange_buoy_marker" that unit tests never catch because HoloOcean itself
# isn't installed in the sandbox that writes/tests them.
_ALLOWED_VISUAL_MATERIALS = frozenset(
    {"white", "gold", "cobblestone", "brick", "wood", "grass", "steel", "black"}
)
_ALLOWED_ACOUSTIC_REFLECTIVITY_CLASSES = frozenset({"weak", "moderate", "strong"})
_ALLOWED_DYNAMICS_CALIBRATION_STATUSES = frozenset(
    {"nominal_not_pool_calibrated", "pool_calibrated"}
)


@dataclasses.dataclass(frozen=True)
class SensorSpec:
    sensor_name: str
    sensor_type: str
    socket: str
    hz: float
    location_m: tuple[float, ...] | None
    configuration: dict[str, Any]


@dataclasses.dataclass(frozen=True)
class PropSpec:
    tag: str
    prop_type: str
    dimensions_m: tuple[float, ...]
    location_m: tuple[float, float, float]
    visual_material: str
    acoustic_reflectivity_class: str


@dataclasses.dataclass(frozen=True)
class ActuatorModelSpec:
    limit: float
    deadzone: float
    time_constant_s: float
    thruster_count: int


@dataclasses.dataclass(frozen=True)
class TaskSpec:
    task_id: str
    version: int
    max_duration_s: float
    arena_dimensions_m: tuple[float, float, float]
    start_translation_m: tuple[float, float, float]
    start_quaternion_xyzw: tuple[float, float, float, float]
    props: tuple[PropSpec, ...]
    target: dict[str, Any]
    success_conditions: dict[str, float]


@dataclasses.dataclass(frozen=True)
class RealtimeScenarioManifest:
    name: str
    package_name: str
    world: str
    main_agent: str
    ticks_per_sec: int
    frames_per_sec: int
    agent_name: str
    agent_type: str
    control_scheme: int
    start_location_m: tuple[float, float, float]
    start_rotation_deg: tuple[float, float, float]
    sensors: tuple[SensorSpec, ...]
    actuator_model: ActuatorModelSpec
    dynamics_calibration_status: str
    sonar_operating_frequency_hz: float
    sonar_calibration_status: str
    algorithm_topics: tuple[str, ...]
    task: TaskSpec
    uw_metadata: dict[str, Any]
    _raw_scenario: dict[str, Any]
    _raw_task: dict[str, Any]
    # Which `uw_profiles` / `uw_sonar_modes` overlay was applied (None when
    # the file defines none). SIM-PERF-002 requires the profile name to be
    # recorded in the run manifest; this is where a session reads it from.
    profile: str | None = None
    sonar_mode: str | None = None

    def sensor(self, sensor_name: str) -> SensorSpec:
        for spec in self.sensors:
            if spec.sensor_name == sensor_name:
                return spec
        raise KeyError(f"no sensor named {sensor_name!r} in this manifest")

    def has_sensor(self, sensor_name: str) -> bool:
        return any(spec.sensor_name == sensor_name for spec in self.sensors)

    def holoocean_scenario_cfg(self) -> dict[str, Any]:
        """The HoloOcean-clean `scenario_cfg` dict, with every repo-only key
        (`REPO_ONLY_SCENARIO_KEYS`) stripped — safe to pass directly to
        `holoocean.make(scenario_cfg=...)`. Profile/sonar-mode overlays
        have already been applied to `_raw_scenario` by the loader, so
        what HoloOcean sees is exactly the resolved sensor set/rates."""
        cfg = json.loads(json.dumps(self._raw_scenario))
        for key in REPO_ONLY_SCENARIO_KEYS:
            cfg.pop(key, None)
        return cfg

    def validation_data(self) -> dict[str, Any]:
        """The combined dict shape `validate_realtime_manifest` accepts —
        exposed so tests can mutate a real, valid manifest instead of
        hand-rolling one field at a time."""
        return {
            "scenario": self._raw_scenario,
            "task": self._raw_task,
            "algorithm_topics": list(self.algorithm_topics),
        }


def load_realtime_manifest(
    scenario_path: str | pathlib.Path,
    task_path: str | pathlib.Path,
    algorithm_topics: list[str] | None = None,
    *,
    profile: str | None = None,
    sonar_mode: str | None = None,
) -> RealtimeScenarioManifest:
    """Loads a scenario JSON file and a task YAML file, validates the
    combination, and returns a typed manifest. Always takes real file paths
    (or at minimum path-like objects that resolve to real files) — never
    accepts a bare scenario name, so there is no hidden name->path registry
    to keep in sync.

    `profile`/`sonar_mode` select a `uw_profiles`/`uw_sonar_modes` overlay
    (see the module docstring). Passing one for a file that defines no such
    block is an error, not a silent no-op; passing none picks the file's
    `uw_metadata.default_profile`/`default_sonar_mode` when the block
    exists (and `fidelity`/`sonar_1200khz` when those defaults are absent)."""
    if not isinstance(scenario_path, (str, pathlib.Path)) or not isinstance(
        task_path, (str, pathlib.Path)
    ):
        raise TypeError("scenario_path/task_path must be real file paths")

    scenario_path = pathlib.Path(scenario_path)
    task_path = pathlib.Path(task_path)
    scenario = load_scenario_json(scenario_path)
    task = yaml.safe_load(task_path.read_text())

    scenario, resolved_profile, resolved_sonar_mode = resolve_scenario_overlays(
        scenario, profile=profile, sonar_mode=sonar_mode
    )
    data = _build_validation_data(scenario, task, algorithm_topics)
    validate_realtime_manifest(data)
    return dataclasses.replace(
        _manifest_from_validation_data(data), profile=resolved_profile, sonar_mode=resolved_sonar_mode
    )


def load_scenario_json(scenario_path: pathlib.Path) -> dict[str, Any]:
    """Reads a scenario JSON and resolves its `uw_extends` chain (base file
    path relative to the derived file's directory). Cycles and missing
    bases raise; the returned dict carries no `uw_extends`/`uw_sensor_subset`
    keys any more (they are consumed here)."""
    return _load_scenario_json(scenario_path.resolve(), seen=())


def _load_scenario_json(scenario_path: pathlib.Path, seen: tuple[pathlib.Path, ...]) -> dict[str, Any]:
    if scenario_path in seen:
        raise ValueError(f"uw_extends cycle: {' -> '.join(str(p) for p in (*seen, scenario_path))}")
    derived = json.loads(scenario_path.read_text())
    base_ref = derived.pop("uw_extends", None)
    if base_ref is None:
        if "uw_sensor_subset" in derived:
            raise ValueError(f"{scenario_path}: uw_sensor_subset is only meaningful together with uw_extends")
        return derived
    base_path = (scenario_path.parent / base_ref).resolve()
    if not base_path.is_file():
        raise FileNotFoundError(f"{scenario_path}: uw_extends base {base_ref!r} not found at {base_path}")
    base = _load_scenario_json(base_path, seen=(*seen, scenario_path))
    return _apply_extends(base, derived, scenario_path)


def _apply_extends(base: dict[str, Any], derived: dict[str, Any], derived_path: pathlib.Path) -> dict[str, Any]:
    merged = json.loads(json.dumps(base))
    subset = derived.pop("uw_sensor_subset", None)
    for key, value in derived.items():
        if key == "uw_metadata" and isinstance(merged.get(key), dict) and isinstance(value, dict):
            merged[key] = {**merged[key], **value}
        else:
            merged[key] = value
    if subset is not None:
        if not merged.get("agents"):
            raise ValueError(f"{derived_path}: uw_sensor_subset needs a base with at least one agent")
        available = {sensor["sensor_name"] for sensor in merged["agents"][0]["sensors"]}
        unknown = [name for name in subset if name not in available]
        if unknown:
            raise ValueError(f"{derived_path}: uw_sensor_subset names sensors the base does not define: {unknown}")
        merged["agents"][0]["sensors"] = [
            sensor for sensor in merged["agents"][0]["sensors"] if sensor["sensor_name"] in set(subset)
        ]
    return merged


_DEFAULT_PROFILE = "fidelity"
_DEFAULT_SONAR_MODE = "sonar_1200khz"


def resolve_scenario_overlays(
    scenario: dict[str, Any],
    *,
    profile: str | None = None,
    sonar_mode: str | None = None,
) -> tuple[dict[str, Any], str | None, str | None]:
    """Applies the selected `uw_profiles` and `uw_sonar_modes` overlays and
    returns (resolved scenario, profile name, sonar mode name). Pure: never
    mutates its input. A scenario without the corresponding block yields
    `None` for that name (and rejects an explicit selection)."""
    scenario = json.loads(json.dumps(scenario))
    uw_metadata = scenario.get("uw_metadata", {})

    profiles = scenario.get("uw_profiles")
    resolved_profile: str | None = None
    if profiles is None:
        if profile is not None:
            raise ValueError(f"scenario defines no uw_profiles block, cannot select profile {profile!r}")
    else:
        resolved_profile = profile or uw_metadata.get("default_profile", _DEFAULT_PROFILE)
        if resolved_profile not in profiles:
            raise ValueError(
                f"unknown profile {resolved_profile!r}; this scenario defines {sorted(profiles)}"
            )
        _apply_profile(scenario, profiles[resolved_profile], resolved_profile)

    sonar_modes = scenario.get("uw_sonar_modes")
    resolved_sonar_mode: str | None = None
    if sonar_modes is None:
        if sonar_mode is not None:
            raise ValueError(f"scenario defines no uw_sonar_modes block, cannot select sonar mode {sonar_mode!r}")
    else:
        resolved_sonar_mode = sonar_mode or uw_metadata.get("default_sonar_mode", _DEFAULT_SONAR_MODE)
        if resolved_sonar_mode not in sonar_modes:
            raise ValueError(
                f"unknown sonar mode {resolved_sonar_mode!r}; this scenario defines {sorted(sonar_modes)}"
            )
        _apply_sonar_mode(scenario, sonar_modes[resolved_sonar_mode], resolved_sonar_mode)

    return scenario, resolved_profile, resolved_sonar_mode


def _apply_profile(scenario: dict[str, Any], overlay: dict[str, Any], name: str) -> None:
    for key in ("ticks_per_sec", "frames_per_sec"):
        if key in overlay:
            scenario[key] = overlay[key]
    sensor_overlays = overlay.get("sensors", {})
    if not scenario.get("agents"):
        raise ValueError(f"profile {name!r}: scenario has no agent to apply sensor overlays to")
    by_name = {sensor["sensor_name"]: sensor for sensor in scenario["agents"][0]["sensors"]}
    for sensor_name, fields in sensor_overlays.items():
        if sensor_name not in by_name:
            # A subset-derived file (mono) legitimately lacks sensors the
            # shared base profile still names -- skip, do not fail.
            continue
        unknown = [key for key in fields if key not in _PROFILE_SENSOR_KEYS]
        if unknown:
            raise ValueError(
                f"profile {name!r} sensor {sensor_name!r} overrides unsupported keys {unknown}; "
                f"only {list(_PROFILE_SENSOR_KEYS)} may be overlaid"
            )
        target = by_name[sensor_name]
        if "Hz" in fields:
            target["Hz"] = fields["Hz"]
        if "configuration" in fields:
            target["configuration"] = {**target.get("configuration", {}), **fields["configuration"]}


def _apply_sonar_mode(scenario: dict[str, Any], overlay: dict[str, Any], name: str) -> None:
    sonars = [s for s in scenario["agents"][0]["sensors"] if s["sensor_type"] == "ImagingSonar"]
    if len(sonars) != 1:
        raise ValueError(f"sonar mode {name!r}: expected exactly one ImagingSonar sensor, found {len(sonars)}")
    sonars[0]["configuration"] = {**sonars[0].get("configuration", {}), **overlay.get("configuration", {})}
    sonar_model = scenario.setdefault("uw_metadata", {}).setdefault("sonar_model", {})
    if "operating_frequency_hz" in overlay:
        sonar_model["operating_frequency_hz"] = overlay["operating_frequency_hz"]


def validate_realtime_manifest(data: dict[str, Any]) -> None:
    """Validates the combined `{"scenario": ..., "task": ..., "algorithm_topics": [...]}`
    dict produced by `_build_validation_data`/`RealtimeScenarioManifest.validation_data()`.
    Raises `ValueError` with a message naming the specific violation."""
    scenario = data["scenario"]
    task = data["task"]
    algorithm_topics = data["algorithm_topics"]

    if _TRUTH_TOPIC in algorithm_topics:
        raise ValueError(
            f"algorithm_topics must never include the ground truth topic {_TRUTH_TOPIC!r}"
        )

    ticks_per_sec = scenario["ticks_per_sec"]
    if not scenario.get("agents"):
        raise ValueError("scenario must define at least one agent")
    agent = scenario["agents"][0]
    sensors = agent["sensors"]

    seen_names: set[str] = set()
    seen_types: dict[str, str] = {}
    for sensor in sensors:
        sensor_name = sensor["sensor_name"]
        if sensor_name in seen_names:
            raise ValueError(f"duplicate sensor name {sensor_name!r} in scenario agent sensors")
        seen_names.add(sensor_name)
        seen_types[sensor_name] = sensor["sensor_type"]

        hz = sensor["Hz"]
        if hz <= 0 or not _rate_divides_ticks(hz, ticks_per_sec):
            raise ValueError(
                f"sensor {sensor_name!r} rate {hz} Hz must evenly divide ticks_per_sec ({ticks_per_sec})"
            )

        if sensor["sensor_type"] == "RGBCamera":
            config = sensor.get("configuration", {})
            if config.get("CaptureWidth", 0) <= 0 or config.get("CaptureHeight", 0) <= 0:
                raise ValueError(f"camera {sensor_name!r} must have positive CaptureWidth/CaptureHeight")

        if sensor["sensor_type"] == "ImagingSonar":
            config = sensor.get("configuration", {})
            missing = [key for key in _REQUIRED_SONAR_CONFIG_KEYS if key not in config]
            if missing:
                raise ValueError(
                    f"ImagingSonar is missing required sonar calibration fields: {missing}"
                )

    for required_name, required_type in _REQUIRED_SENSOR_TYPES.items():
        if required_name not in seen_names:
            raise ValueError(f"scenario is missing required sensor {required_name!r}")
        if seen_types[required_name] != required_type:
            raise ValueError(
                f"sensor {required_name!r} must be sensor_type {required_type!r}, "
                f"got {seen_types[required_name]!r}"
            )

    present_cameras = [name for name in ALGORITHM_CAMERA_NAMES if name in seen_names]
    if not present_cameras:
        raise ValueError(
            f"scenario must define at least one algorithm camera among {list(ALGORITHM_CAMERA_NAMES)}"
        )
    for name in present_cameras + (["PilotCamera"] if "PilotCamera" in seen_names else []):
        if seen_types[name] != "RGBCamera":
            raise ValueError(f"sensor {name!r} must be sensor_type 'RGBCamera', got {seen_types[name]!r}")
    stereo_present = [name in seen_names for name in _STEREO_CAMERA_NAMES]
    if any(stereo_present) and not all(stereo_present):
        raise ValueError(
            "LeftCamera and RightCamera must be both present (stereo) or both absent (monocular); "
            f"found {[n for n, p in zip(_STEREO_CAMERA_NAMES, stereo_present) if p]}"
        )

    uw_metadata = scenario.get("uw_metadata", {})
    thruster_count = uw_metadata.get("thruster_count")
    if thruster_count != 8:
        raise ValueError(f"uw_metadata.thruster_count must be 8 (eight-thruster BlueROV2), got {thruster_count!r}")

    actuator = uw_metadata.get("pilot_command_model", {})
    limit = actuator.get("limit")
    deadzone = actuator.get("deadzone")
    time_constant_s = actuator.get("time_constant_s")
    if limit is None or limit <= 0:
        raise ValueError("actuator model pilot_command_model.limit must be positive")
    if deadzone is None or not (0.0 <= deadzone < limit):
        raise ValueError("actuator model pilot_command_model.deadzone must be within [0, limit)")
    if time_constant_s is None or time_constant_s <= 0:
        raise ValueError("actuator model pilot_command_model.time_constant_s must be positive")

    dynamics_status = uw_metadata.get("dynamics_calibration_status")
    if dynamics_status not in _ALLOWED_DYNAMICS_CALIBRATION_STATUSES:
        raise ValueError(
            f"uw_metadata.dynamics_calibration_status must be one of "
            f"{sorted(_ALLOWED_DYNAMICS_CALIBRATION_STATUSES)}, got {dynamics_status!r}"
        )

    props = task.get("props", [])
    seen_prop_tags: set[str] = set()
    for prop in props:
        tag = prop["tag"]
        if tag in seen_prop_tags:
            raise ValueError(f"duplicate prop tag {tag!r} in task props")
        seen_prop_tags.add(tag)

        prop_type = prop["prop_type"]
        if prop_type not in _ALLOWED_PROP_TYPES:
            raise ValueError(
                f"prop {tag!r} has unknown prop_type {prop_type!r}, must be one of "
                f"{sorted(_ALLOWED_PROP_TYPES)}"
            )

        visual_material = prop.get("visual_material")
        if visual_material not in _ALLOWED_VISUAL_MATERIALS:
            raise ValueError(
                f"prop {tag!r} has unknown visual_material {visual_material!r}, must be one of "
                f"{sorted(_ALLOWED_VISUAL_MATERIALS)} (HoloOceanEnvironment.spawn_prop()'s real "
                "material palette)"
            )

        reflectivity = prop.get("acoustic_reflectivity_class")
        if reflectivity not in _ALLOWED_ACOUSTIC_REFLECTIVITY_CLASSES:
            raise ValueError(
                f"prop {tag!r} has unknown acoustic_reflectivity_class {reflectivity!r}, "
                f"must be one of {sorted(_ALLOWED_ACOUSTIC_REFLECTIVITY_CLASSES)}"
            )

    if not task.get("success_conditions"):
        raise ValueError("task must define at least one success condition")

    target = task.get("target", {})
    if not target.get("visual_properties") or not target.get("acoustic_properties"):
        raise ValueError("task target must define both visual and acoustic properties")


def _rate_divides_ticks(hz: float, ticks_per_sec: float) -> bool:
    """HoloOcean quantizes each sensor to an integer tick divisor
    (`SIM-PERF-001`), so `ticks_per_sec / hz` must be a whole number. A
    float ratio is accepted when it is whole to 1e-9 -- the realtime
    profile's 12.5 Hz camera at 25 ticks/s is a divisor of exactly 2 even
    though 12.5 is not an integer rate."""
    if hz <= 0 or ticks_per_sec <= 0:
        return False
    ratio = ticks_per_sec / hz
    return abs(ratio - round(ratio)) < 1e-9 and round(ratio) >= 1


def _build_validation_data(
    scenario: dict[str, Any],
    task: dict[str, Any],
    algorithm_topics: list[str] | None,
) -> dict[str, Any]:
    topics = (
        list(algorithm_topics)
        if algorithm_topics is not None
        else list(scenario.get("algorithm_topics") or DEFAULT_ALGORITHM_TOPICS)
    )
    return {"scenario": scenario, "task": task, "algorithm_topics": topics}


def _manifest_from_validation_data(data: dict[str, Any]) -> RealtimeScenarioManifest:
    scenario = data["scenario"]
    task = data["task"]
    agent = scenario["agents"][0]
    uw_metadata = scenario.get("uw_metadata", {})
    actuator = uw_metadata.get("pilot_command_model", {})
    sonar_model = uw_metadata.get("sonar_model", {})

    sensors = tuple(
        SensorSpec(
            sensor_name=sensor["sensor_name"],
            sensor_type=sensor["sensor_type"],
            socket=sensor["socket"],
            hz=sensor["Hz"],
            location_m=tuple(sensor["location"]) if "location" in sensor else None,
            configuration=dict(sensor.get("configuration", {})),
        )
        for sensor in agent["sensors"]
    )

    props = tuple(
        PropSpec(
            tag=prop["tag"],
            prop_type=prop["prop_type"],
            dimensions_m=tuple(prop["dimensions_m"]),
            location_m=tuple(prop["location_m"]),
            visual_material=prop["visual_material"],
            acoustic_reflectivity_class=prop["acoustic_reflectivity_class"],
        )
        for prop in task.get("props", [])
    )

    start_pose = task["start_pose"]
    task_spec = TaskSpec(
        task_id=task["task_id"],
        version=task["version"],
        max_duration_s=task["max_duration_s"],
        arena_dimensions_m=tuple(task["arena_dimensions_m"]),
        start_translation_m=tuple(start_pose["translation_m"]),
        start_quaternion_xyzw=tuple(start_pose["quaternion_xyzw"]),
        props=props,
        target=dict(task["target"]),
        success_conditions=dict(task["success_conditions"]),
    )

    return RealtimeScenarioManifest(
        name=scenario["name"],
        package_name=scenario["package_name"],
        world=scenario["world"],
        main_agent=scenario["main_agent"],
        ticks_per_sec=scenario["ticks_per_sec"],
        frames_per_sec=scenario["frames_per_sec"],
        agent_name=agent["agent_name"],
        agent_type=agent["agent_type"],
        control_scheme=agent["control_scheme"],
        start_location_m=tuple(agent["location"]),
        start_rotation_deg=tuple(agent["rotation"]),
        sensors=sensors,
        actuator_model=ActuatorModelSpec(
            limit=actuator["limit"],
            deadzone=actuator["deadzone"],
            time_constant_s=actuator["time_constant_s"],
            thruster_count=uw_metadata["thruster_count"],
        ),
        dynamics_calibration_status=uw_metadata["dynamics_calibration_status"],
        sonar_operating_frequency_hz=sonar_model["operating_frequency_hz"],
        sonar_calibration_status=sonar_model["calibration_status"],
        algorithm_topics=tuple(data["algorithm_topics"]),
        task=task_spec,
        uw_metadata=dict(uw_metadata),
        _raw_scenario=scenario,
        _raw_task=task,
    )
