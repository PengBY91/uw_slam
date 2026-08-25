import copy
import pathlib

import pytest

from uw_holoocean_adapter.scenario_manifest import (
    load_realtime_manifest,
    validate_realtime_manifest,
)

REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
BASE_SCENARIO = REPO_ROOT / "adapters/holoocean/scenarios/blue_rov_aid_sv1213_base.json"
SEARCH_TASK = REPO_ROOT / "adapters/holoocean/scenarios/aquaculture_search.yaml"
STRUCTURE_TASK = REPO_ROOT / "adapters/holoocean/scenarios/structure_inspection.yaml"


def valid_manifest_dict() -> dict:
    manifest = load_realtime_manifest(BASE_SCENARIO, SEARCH_TASK)
    return copy.deepcopy(manifest.validation_data())


def test_base_manifest_has_independent_20_10_50_hz_sensor_rates():
    manifest = load_realtime_manifest(BASE_SCENARIO, SEARCH_TASK)

    assert manifest.ticks_per_sec == 100
    assert manifest.sensor("LeftCamera").hz == 20
    assert manifest.sensor("RightCamera").hz == 20
    assert manifest.sensor("PilotCamera").hz == 20
    assert manifest.sensor("ImagingSonar").hz == 10
    assert manifest.sensor("VehicleOrientation").hz == 50
    assert manifest.sensor("DepthSensor").hz == 50


def test_manifest_rejects_algorithm_truth_subscription():
    data = valid_manifest_dict()
    data["algorithm_topics"].append("/uw/sim/ground_truth")
    with pytest.raises(ValueError, match="ground truth"):
        validate_realtime_manifest(data)


def test_manifest_rejects_duplicate_sensor_names():
    data = valid_manifest_dict()
    sensors = data["scenario"]["agents"][0]["sensors"]
    duplicate = copy.deepcopy(sensors[0])
    sensors.append(duplicate)
    with pytest.raises(ValueError, match="duplicate"):
        validate_realtime_manifest(data)


def test_manifest_rejects_sensor_rate_not_dividing_ticks_per_sec():
    data = valid_manifest_dict()
    for sensor in data["scenario"]["agents"][0]["sensors"]:
        if sensor["sensor_name"] == "ImagingSonar":
            sensor["Hz"] = 7
    with pytest.raises(ValueError, match="divide"):
        validate_realtime_manifest(data)


def test_manifest_rejects_missing_sonar_calibration():
    data = valid_manifest_dict()
    for sensor in data["scenario"]["agents"][0]["sensors"]:
        if sensor["sensor_name"] == "ImagingSonar":
            del sensor["configuration"]["WaterSpeedSound"]
    with pytest.raises(ValueError, match="sonar calibration"):
        validate_realtime_manifest(data)


def test_manifest_rejects_missing_task_success_conditions():
    data = valid_manifest_dict()
    data["task"]["success_conditions"] = {}
    with pytest.raises(ValueError, match="success condition"):
        validate_realtime_manifest(data)


def test_manifest_rejects_task_target_without_visual_and_acoustic_properties():
    data = valid_manifest_dict()
    del data["task"]["target"]["acoustic_properties"]
    with pytest.raises(ValueError, match="visual and acoustic"):
        validate_realtime_manifest(data)


def test_manifest_rejects_duplicate_prop_tags():
    data = valid_manifest_dict()
    props = data["task"]["props"]
    duplicate = copy.deepcopy(props[0])
    props.append(duplicate)
    with pytest.raises(ValueError, match="duplicate"):
        validate_realtime_manifest(data)


def test_manifest_rejects_unknown_prop_type():
    data = valid_manifest_dict()
    data["task"]["props"][0]["prop_type"] = "pyramid"
    with pytest.raises(ValueError, match="prop_type"):
        validate_realtime_manifest(data)


def test_manifest_rejects_unknown_acoustic_reflectivity_class():
    data = valid_manifest_dict()
    data["task"]["props"][0]["acoustic_reflectivity_class"] = "supersonic"
    with pytest.raises(ValueError, match="reflectivity"):
        validate_realtime_manifest(data)


def test_manifest_rejects_wrong_thruster_count():
    data = valid_manifest_dict()
    data["scenario"]["uw_metadata"]["thruster_count"] = 6
    with pytest.raises(ValueError, match="thruster"):
        validate_realtime_manifest(data)


def test_manifest_rejects_out_of_bounds_actuator_values():
    data = valid_manifest_dict()
    data["scenario"]["uw_metadata"]["pilot_command_model"]["deadzone"] = 500.0
    with pytest.raises(ValueError, match="actuator"):
        validate_realtime_manifest(data)


def test_manifest_rejects_unknown_dynamics_calibration_status():
    data = valid_manifest_dict()
    data["scenario"]["uw_metadata"]["dynamics_calibration_status"] = "definitely_calibrated_trust_me"
    with pytest.raises(ValueError, match="dynamics_calibration_status"):
        validate_realtime_manifest(data)


def test_manifest_strips_repo_only_keys_for_holoocean():
    manifest = load_realtime_manifest(BASE_SCENARIO, SEARCH_TASK)
    cfg = manifest.holoocean_scenario_cfg()

    assert "uw_metadata" not in cfg
    assert "algorithm_topics" not in cfg
    assert cfg["name"] == "uw_bluerov_aid_sv1213_v1"
    assert manifest.uw_metadata["vehicle_variant"]


def test_loader_requires_real_files_not_a_bare_name():
    with pytest.raises((TypeError, FileNotFoundError)):
        load_realtime_manifest("blue_rov_aid_sv1213_base", "aquaculture_search")


def test_structure_inspection_task_has_a_path_target_with_both_properties():
    manifest = load_realtime_manifest(BASE_SCENARIO, STRUCTURE_TASK)

    assert manifest.task.target["kind"] == "path"
    assert manifest.task.target["visual_properties"]
    assert manifest.task.target["acoustic_properties"]
    assert len(manifest.task.props) >= 1


def test_search_task_has_a_point_target_with_both_properties():
    manifest = load_realtime_manifest(BASE_SCENARIO, SEARCH_TASK)

    assert manifest.task.target["kind"] == "point"
    assert manifest.task.target["visual_properties"]
    assert manifest.task.target["acoustic_properties"]
