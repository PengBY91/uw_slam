"""PREP-A-10: adapters/holoocean/tools/sonar_stats.py must recover a known
Rayleigh noise floor and target contrast from synthetic frames and read
frames back through the canonical MCAP path."""
import importlib.util
import json
import pathlib
import sys
import tempfile

import numpy as np
from uw.domain import observation_pb2, sonar_pb2, time_pb2  # noqa: E402

from uw_holoocean_adapter.canonical_writer import CanonicalMcapWriter
from uw_holoocean_adapter.sonar_conversion import holoocean_sonar_to_sonar_frame

_SCRIPT = pathlib.Path(__file__).resolve().parents[1] / "tools" / "sonar_stats.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("sonar_stats_under_test", _SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


_ROWS, _COLS = 128, 96


def _synthetic_frames(rng: np.random.Generator, *, sigma_units: float, count: int, target_level: float = 0.8,
                      decay_per_row: float = 0.0):
    """Rayleigh clutter with scale `sigma_units` (in [0,1] HoloOcean units)
    plus a bright target block; optional exponential attenuation of the
    floor along range (row index)."""
    frames = []
    rows = np.arange(_ROWS)[:, None]
    attenuation = np.exp(-decay_per_row * rows)
    for _ in range(count):
        floor = rng.rayleigh(sigma_units, size=(_ROWS, _COLS)) * attenuation
        img = np.clip(floor, 0.0, 1.0)
        img[40:56, 30:50] = target_level
        frames.append(img)
    return frames


def _to_uint8(frames_unit):
    return [np.clip(np.round(f * 255.0), 0, 255).astype(np.uint8) for f in frames_unit]


def test_rayleigh_floor_and_scr_recovered_from_synthetic_frames():
    m = _load_module()
    rng = np.random.default_rng(1)
    sigma_units = 0.03  # 7.65 LSB, well above quantisation
    frames = _to_uint8(_synthetic_frames(rng, sigma_units=sigma_units, count=8))
    stats = m.aggregate_stats(frames)
    assert abs(stats["floor_rayleigh_sigma"] / (sigma_units * 255.0) - 1.0) < 0.10
    assert stats["target_cell_fraction"] > 0.02  # the 16x20 block out of 128x96
    # The 1e-3 Rayleigh tail of the clutter lands above the threshold and
    # dilutes the target mean slightly (documented behaviour, not a bug).
    assert abs(stats["target_mean"] / (0.8 * 255.0) - 1.0) < 0.05
    expected_scr = 0.8 * 255.0 / (sigma_units * 255.0 * np.sqrt(np.pi / 2.0))
    assert abs(stats["signal_to_clutter_ratio"] / expected_scr - 1.0) < 0.15
    assert stats["saturation_fraction"] == 0.0
    assert sum(stats["histogram"]) == 8 * _ROWS * _COLS
    assert stats["num_ranges"] == _ROWS and stats["num_beams"] == _COLS


def test_range_attenuation_curve_is_monotone_for_exponential_decay():
    m = _load_module()
    rng = np.random.default_rng(2)
    frames = _to_uint8(_synthetic_frames(rng, sigma_units=0.05, count=6, decay_per_row=0.02))
    stats = m.aggregate_stats(frames, range_centers_m=np.linspace(0.3, 30.0, _ROWS))
    floor_curve = np.asarray([np.nan if v is None else v for v in stats["per_range_mean_floor"]])
    # Average over 16-row blocks to beat the per-row noise, then require a strict decrease.
    blocks = np.nanmean(floor_curve.reshape(8, 16), axis=1)
    assert np.all(np.diff(blocks) < 0)
    assert stats["range_centers_m"][0] == 0.3 and abs(stats["range_centers_m"][-1] - 30.0) < 1e-9


def test_compare_two_synthetic_bags_reports_noise_ratio():
    m = _load_module()
    rng = np.random.default_rng(3)
    a = m.aggregate_stats(_to_uint8(_synthetic_frames(rng, sigma_units=0.02, count=4)))
    b = m.aggregate_stats(_to_uint8(_synthetic_frames(rng, sigma_units=0.04, count=4)))
    rows = {key: ratio for key, _va, _vb, ratio in m.compare_stats(a, b)}
    assert abs(rows["floor_rayleigh_sigma"] - 2.0) < 0.2
    assert rows["signal_to_clutter_ratio"] < 0.6  # same target, doubled clutter
    table = m.render_markdown({"A": a, "B": b})
    assert "| floor_rayleigh_sigma |" in table and "B/A" in table


def test_all_zero_frame_does_not_crash():
    m = _load_module()
    stats = m.aggregate_stats([np.zeros((8, 8), dtype=np.uint8)])
    assert stats["zero_fraction"] == 1.0
    assert stats["floor_rayleigh_sigma"] == 0.0
    assert stats["signal_to_clutter_ratio"] == 0.0


def test_range_centers_handles_edges_and_centres():
    m = _load_module()
    edges = m.range_centers([0.0, 1.0, 2.0, 3.0], 3, 0.0, 3.0)
    assert np.allclose(edges, [0.5, 1.5, 2.5])
    centres = m.range_centers([0.5, 1.5, 2.5], 3, 0.0, 3.0)
    assert np.allclose(centres, [0.5, 1.5, 2.5])
    fallback = m.range_centers([], 3, 0.0, 3.0)
    assert np.allclose(fallback, [0.5, 1.5, 2.5])


def test_mcap_round_trip_and_cli(tmp_path):
    m = _load_module()
    rng = np.random.default_rng(4)
    unit_frames = _synthetic_frames(rng, sigma_units=0.03, count=3)
    bag = tmp_path / "sonar.mcap"
    with CanonicalMcapWriter(str(bag)) as writer:
        for i, img in enumerate(unit_frames):
            frame = holoocean_sonar_to_sonar_frame(
                sonar_pb2, observation_pb2, time_pb2, img.astype(np.float32),
                sensor_id="sonar0", sensor_frame="sonar_link", observation_id=f"tick{i}",
                capture_time_s=0.1 * i, horizontal_fov_rad=2.443, min_range_m=0.3, max_range_m=30.0,
            )
            writer.write_message("/raw/sonar_frame", int(0.1 * i * 1e9), frame)
    frames = m.load_frames_from_mcap(str(bag), "/raw/sonar_frame")
    assert len(frames) == 3
    assert frames[0].intensity.shape == (_ROWS, _COLS)
    # sonar_conversion mirrors columns; the target block must land mirrored.
    assert frames[0].intensity[48, _COLS - 1 - 40] == 204
    assert frames[0].range_centers_m.shape == (_ROWS,)
    assert abs(frames[0].range_centers_m[0] - (0.3 + 0.5 * (29.7 / _ROWS))) < 1e-6
    out = tmp_path / "stats.json"
    m.main(["--bag", str(bag), "--compare", str(bag), "--out", str(out), "--max-frames", "2"])
    payload = json.loads(out.read_text())
    assert payload["A"]["frames"] == 2 and payload["B"]["frames"] == 2
    assert abs(payload["A"]["floor_rayleigh_sigma"] / (0.03 * 255.0) - 1.0) < 0.10
    assert (tmp_path / "stats.md").exists()
