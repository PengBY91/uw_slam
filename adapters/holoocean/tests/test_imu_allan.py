"""PREP-B-05: tools/calib/imu_allan.py must recover known IMU noise
parameters from a synthetic static stream (white noise density + bias
random walk) and read them back through the canonical MCAP path."""
import importlib.util
import pathlib
import sys
import tempfile

import numpy as np
from uw.domain import imu_pb2, observation_pb2, time_pb2  # noqa: E402

from uw_holoocean_adapter.canonical_writer import CanonicalMcapWriter
from uw_holoocean_adapter.imu_conversion import holoocean_imu_to_imu_sample

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SCRIPT = _REPO_ROOT / "tools" / "calib" / "imu_allan.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("imu_allan_under_test", _SCRIPT)
    module = importlib.util.module_from_spec(spec)
    # dataclasses + `from __future__ import annotations` resolve the module
    # through sys.modules, so it must be registered before exec.
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


_RATE_HZ = 200.0


def _synthetic_stream(rng: np.random.Generator, *, duration_s: float, white_density: float,
                      walk_density: float, gravity: np.ndarray):
    """White noise with continuous density `white_density` (unit/sqrt(Hz)) is
    N(0, density * sqrt(rate)) per sample; a rate random walk with density
    `walk_density` (unit/s/sqrt(Hz)) accumulates N(0, density * sqrt(dt))
    increments — the same discretisation the README documents for
    HoloOcean's per-tick sigmas."""
    n = int(duration_s * _RATE_HZ)
    dt = 1.0 / _RATE_HZ
    white = rng.normal(0.0, white_density * np.sqrt(_RATE_HZ), size=(n, 3))
    walk = np.cumsum(rng.normal(0.0, walk_density * np.sqrt(dt), size=(n, 3)), axis=0)
    t = np.arange(n) * dt
    return t, white + walk + gravity


def test_allan_deviation_matches_analytic_white_noise_and_random_walk():
    m = _load_module()
    rng = np.random.default_rng(20260903)
    gyro_n, gyro_k = 1.7e-4, 2.0e-5
    t, gyro = _synthetic_stream(rng, duration_s=2000.0, white_density=gyro_n, walk_density=gyro_k,
                                gravity=np.zeros(3))
    taus = m.default_taus(_RATE_HZ, 2000.0)
    assert taus[0] == 0.01
    assert taus[-1] <= 2000.0 / 9.0
    taus_used, adev = m.allan_deviation(t, gyro, taus)
    assert adev.shape == (len(taus_used), 3)
    # At tau = 1 sample the deviation must sit on the N / sqrt(tau) line.
    expected_short = gyro_n / np.sqrt(taus_used[0])
    assert np.allclose(adev[0], expected_short, rtol=0.05)
    results = m.analyze_axes(taus_used, adev)
    for r in results:
        assert abs(r.white_noise.value / gyro_n - 1.0) < 0.2
        assert r.random_walk is not None
        assert abs(r.random_walk.value / gyro_k - 1.0) < 0.3
        # Bias instability from the minimum is bounded by the two other terms.
        assert 0.0 < r.bias_instability.value < gyro_n


def test_random_walk_is_not_invented_for_a_pure_white_noise_stream():
    m = _load_module()
    rng = np.random.default_rng(3)
    t, accel = _synthetic_stream(rng, duration_s=600.0, white_density=2.0e-3, walk_density=0.0,
                                 gravity=np.array([0.0, 0.0, 9.80665]))
    taus_used, adev = m.allan_deviation(t, accel, m.default_taus(_RATE_HZ, 600.0))
    results = m.analyze_axes(taus_used, adev)
    for r in results:
        assert abs(r.white_noise.value / 2.0e-3 - 1.0) < 0.2
        # The de-meaned gravity axis must not leak into the fit.
        assert r.random_walk is None or r.random_walk.value < 0.05 * 2.0e-3
    fragment = m.imu_noise_fragment(results, results, rate_hz=_RATE_HZ, gravity_mps2=9.80665)
    assert fragment["sigma_accel_bias_walk_c"] < 0.05 * 2.0e-3
    yaml_text = m.render_yaml(fragment, source="synthetic")
    assert "imu_noise:" in yaml_text and "sigma_accel_c:" in yaml_text and "rate_hz: 200" in yaml_text


def test_default_taus_rejects_a_recording_too_short_for_nine_clusters():
    m = _load_module()
    try:
        m.default_taus(200.0, 0.05)
    except ValueError as exc:
        assert "too short" in str(exc)
    else:  # pragma: no cover
        raise AssertionError("expected ValueError")


def test_mcap_path_reads_imu_samples_ordered_by_capture_time():
    m = _load_module()
    rng = np.random.default_rng(11)
    n = 400
    dt = 1.0 / _RATE_HZ
    gyro = rng.normal(0.0, 1e-3, size=(n, 3))
    accel = rng.normal(0.0, 1e-2, size=(n, 3)) + np.array([0.0, 0.0, 9.80665])
    with tempfile.TemporaryDirectory() as tmp:
        path = str(pathlib.Path(tmp) / "static.mcap")
        with CanonicalMcapWriter(path) as writer:
            # Written deliberately out of order to prove the reader sorts by capture_time.
            for i in list(range(n))[::-1]:
                sample = holoocean_imu_to_imu_sample(
                    imu_pb2, observation_pb2, time_pb2, np.vstack([accel[i], gyro[i]]),
                    sensor_id="imu0", sensor_frame="imu_link", observation_id=f"tick{i}",
                    capture_time_s=i * dt,
                )
                writer.write_message("/raw/imu", int(i * dt * 1e9), sample)
        t, g, a = m.load_imu_from_mcap(path, "/raw/imu")
    assert t.shape == (n,) and g.shape == (n, 3) and a.shape == (n, 3)
    assert np.all(np.diff(t) > 0)
    assert np.allclose(g, gyro, atol=1e-12)
    assert np.allclose(a, accel, atol=1e-12)


def test_cli_writes_yaml_and_json(tmp_path):
    m = _load_module()
    rng = np.random.default_rng(5)
    n = int(60 * _RATE_HZ)
    dt = 1.0 / _RATE_HZ
    gyro = rng.normal(0.0, 1.7e-4 * np.sqrt(_RATE_HZ), size=(n, 3))
    accel = rng.normal(0.0, 2e-3 * np.sqrt(_RATE_HZ), size=(n, 3)) + np.array([0.0, 0.0, 9.80665])
    bag = tmp_path / "static.mcap"
    with CanonicalMcapWriter(str(bag)) as writer:
        for i in range(n):
            sample = holoocean_imu_to_imu_sample(
                imu_pb2, observation_pb2, time_pb2, np.vstack([accel[i], gyro[i]]),
                sensor_id="imu0", sensor_frame="imu_link", observation_id=f"tick{i}",
                capture_time_s=i * dt,
            )
            writer.write_message("/raw/imu", int(i * dt * 1e9), sample)
    out_yaml = tmp_path / "imu_noise.yaml"
    out_json = tmp_path / "allan.json"
    m.main(["--bag", str(bag), "--out", str(out_yaml), "--json", str(out_json)])
    import json
    import yaml
    fragment = yaml.safe_load(out_yaml.read_text())["imu_noise"]
    assert abs(fragment["sigma_gyro_c"] / 1.7e-4 - 1.0) < 0.2
    assert abs(fragment["sigma_accel_c"] / 2e-3 - 1.0) < 0.2
    assert abs(fragment["rate_hz"] - _RATE_HZ) < 0.5
    payload = json.loads(out_json.read_text())
    assert payload["samples"] == n and len(payload["taus_s"]) == len(payload["gyro_adev"])
