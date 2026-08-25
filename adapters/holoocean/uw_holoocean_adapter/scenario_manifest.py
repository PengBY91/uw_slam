"""Versioned, validated BlueROV2/AI-D/SV1213 realtime scenario manifests.

Loads a repo-owned scenario JSON (HoloOcean's own `scenario_cfg` shape, plus
two repo-only keys stripped before the file reaches `holoocean.make` — see
`holoocean_scenario_cfg()`) together with a task YAML (arena/structure,
props, target/path truth, success conditions) into one typed, validated
`RealtimeScenarioManifest`. Nothing here talks to HoloOcean or ROS2; this is
pure data loading/validation so it can be unit tested on any machine (see
`adapters/holoocean/README.md`'s "what's real vs not tested here" section).
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

_REQUIRED_SENSOR_TYPES = {
    "LeftCamera": "RGBCamera",
    "RightCamera": "RGBCamera",
    "PilotCamera": "RGBCamera",
    "ImagingSonar": "ImagingSonar",
    "VehicleOrientation": "OrientationSensor",
    "IMUSensor": "IMUSensor",
    "DepthSensor": "DepthSensor",
}

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

    def sensor(self, sensor_name: str) -> SensorSpec:
        for spec in self.sensors:
            if spec.sensor_name == sensor_name:
                return spec
        raise KeyError(f"no sensor named {sensor_name!r} in this manifest")

    def holoocean_scenario_cfg(self) -> dict[str, Any]:
        """The HoloOcean-clean `scenario_cfg` dict, with repo-only keys
        (`uw_metadata`, `algorithm_topics`) stripped — safe to pass directly
        to `holoocean.make(scenario_cfg=...)`."""
        cfg = json.loads(json.dumps(self._raw_scenario))
        cfg.pop("uw_metadata", None)
        cfg.pop("algorithm_topics", None)
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
) -> RealtimeScenarioManifest:
    """Loads a scenario JSON file and a task YAML file, validates the
    combination, and returns a typed manifest. Always takes real file paths
    (or at minimum path-like objects that resolve to real files) — never
    accepts a bare scenario name, so there is no hidden name->path registry
    to keep in sync."""
    if not isinstance(scenario_path, (str, pathlib.Path)) or not isinstance(
        task_path, (str, pathlib.Path)
    ):
        raise TypeError("scenario_path/task_path must be real file paths")

    scenario_path = pathlib.Path(scenario_path)
    task_path = pathlib.Path(task_path)
    scenario = json.loads(scenario_path.read_text())
    task = yaml.safe_load(task_path.read_text())

    data = _build_validation_data(scenario, task, algorithm_topics)
    validate_realtime_manifest(data)
    return _manifest_from_validation_data(data)


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
        if hz <= 0 or ticks_per_sec % hz != 0:
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
