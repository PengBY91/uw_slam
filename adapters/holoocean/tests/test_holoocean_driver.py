import dataclasses
import random

import pytest

from uw_holoocean_adapter import holoocean_driver as driver
from uw_holoocean_adapter.holoocean_driver import HoloOceanSession
from uw_holoocean_adapter.scenario_randomization import PRESET_CLEAR, PRESET_TURBID


class _FakeEnv:
    def __init__(self):
        self.reset_calls = 0
        self.water_fog_calls = []
        self.water_color_calls = []
        self.set_ocean_currents_calls = []
        self.turn_on_flashlight_calls = []
        self.spawn_prop_calls = []
        self.agents = {"auv0": object()}
        self._scenario = None
        self._ticks_per_sec = 100

    def reset(self):
        self.reset_calls += 1

    def water_fog(self, *args, **kwargs):
        self.water_fog_calls.append(args)

    def water_color(self, *args, **kwargs):
        self.water_color_calls.append(args)

    def set_ocean_currents(self, *args, **kwargs):
        self.set_ocean_currents_calls.append(args)

    def turn_on_flashlight(self, *args, **kwargs):
        self.turn_on_flashlight_calls.append((args, kwargs))

    def spawn_prop(self, *args, **kwargs):
        self.spawn_prop_calls.append((args, kwargs))

    def __exit__(self, exc_type, exc_value, traceback):
        return None


class FakeHoloocean:
    def __init__(self):
        self.env = _FakeEnv()
        self.make_calls = []

    def make(self, *args, **kwargs):
        if "scenario_cfg" in kwargs:
            self.make_calls.append({"scenario_cfg": kwargs["scenario_cfg"]})
            self.env._scenario = kwargs["scenario_cfg"]
        else:
            self.make_calls.append({"scenario_name": args[0]})
        return self.env


def minimal_scenario_dict():
    return {
        "name": "test_scenario",
        "package_name": "Ocean",
        "world": "TestWorld",
        "main_agent": "auv0",
        "ticks_per_sec": 100,
        "agents": [
            {
                "agent_name": "auv0",
                "agent_type": "BlueROV2",
                "sensors": [],
            }
        ],
    }


def test_session_passes_complete_scenario_cfg_and_seeds_python_random(monkeypatch):
    fake = FakeHoloocean()
    monkeypatch.setattr(driver, "_import_holoocean", lambda: fake)
    cfg = minimal_scenario_dict()

    session = HoloOceanSession(cfg, seed=123, randomization=PRESET_TURBID)

    assert fake.make_calls == [{"scenario_cfg": cfg}]
    assert fake.env.reset_calls == 0  # holoocean.make already resets
    assert fake.env.water_fog_calls[-1][0] == pytest.approx(0.6)
    session.close()


def test_legacy_scenario_name_string_still_calls_make_positionally(monkeypatch):
    fake = FakeHoloocean()
    monkeypatch.setattr(driver, "_import_holoocean", lambda: fake)

    session = HoloOceanSession("OpenWater-HoveringCamera", seed=1)

    assert fake.make_calls == [{"scenario_name": "OpenWater-HoveringCamera"}]
    session.close()


def test_close_restores_prior_global_random_state(monkeypatch):
    fake = FakeHoloocean()
    monkeypatch.setattr(driver, "_import_holoocean", lambda: fake)
    random.seed(999)
    snapshot = random.getstate()

    session = HoloOceanSession(minimal_scenario_dict(), seed=5, randomization=PRESET_CLEAR)
    assert random.getstate() != snapshot
    session.close()

    assert random.getstate() == snapshot


def test_spawn_prop_forwards_sim_physics(monkeypatch):
    fake = FakeHoloocean()
    monkeypatch.setattr(driver, "_import_holoocean", lambda: fake)
    session = HoloOceanSession(minimal_scenario_dict(), seed=2, randomization=PRESET_CLEAR)

    session.spawn_prop("box", location=(0.0, 0.0, 0.0), sim_physics=True)

    assert fake.env.spawn_prop_calls[-1][1]["sim_physics"] is True
    session.close()


def test_sonar_speckle_and_range_sigma_applied_before_make_without_mutating_caller_dict(monkeypatch):
    fake = FakeHoloocean()
    monkeypatch.setattr(driver, "_import_holoocean", lambda: fake)
    cfg = minimal_scenario_dict()
    cfg["agents"][0]["sensors"].append(
        {
            "sensor_name": "ImagingSonar",
            "sensor_type": "ImagingSonar",
            "socket": "CameraSocket",
            "configuration": {"MultSigma": 0.02, "RangeSigma": 0.02, "WaterSpeedSound": 1480},
        }
    )
    randomization = dataclasses.replace(
        PRESET_CLEAR,
        sonar=dataclasses.replace(
            PRESET_CLEAR.sonar,
            speckle_sigma=0.09,
            range_noise_sigma_m=0.04,
            sound_speed_scale=1.05,
        ),
    )

    session = HoloOceanSession(cfg, seed=3, randomization=randomization)

    sent_cfg = fake.make_calls[-1]["scenario_cfg"]
    sonar_config = sent_cfg["agents"][0]["sensors"][0]["configuration"]
    assert sonar_config["MultSigma"] == pytest.approx(0.09)
    assert sonar_config["RangeSigma"] == pytest.approx(0.04)
    assert sonar_config["WaterSpeedSound"] == pytest.approx(1480 * 1.05)
    assert cfg["agents"][0]["sensors"][0]["configuration"]["MultSigma"] == pytest.approx(0.02)
    session.close()


def test_flashlight_intensity_scales_with_illumination(monkeypatch):
    fake = FakeHoloocean()
    monkeypatch.setattr(driver, "_import_holoocean", lambda: fake)
    cfg = minimal_scenario_dict()
    cfg["flashlight"] = [{"flashlight_name": "flashlight1", "intensity": 4000.0}]
    dim = dataclasses.replace(
        PRESET_CLEAR,
        visual=dataclasses.replace(PRESET_CLEAR.visual, illumination_scale=0.5),
    )

    session = HoloOceanSession(cfg, seed=4, randomization=dim)

    args, kwargs = fake.env.turn_on_flashlight_calls[-1]
    assert args[0] == "flashlight1"
    assert kwargs["intensity"] == pytest.approx(2000.0)
    session.close()
