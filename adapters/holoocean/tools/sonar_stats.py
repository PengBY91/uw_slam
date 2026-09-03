"""PREP-A-10: sonar image statistics for HoloOcean-vs-real gap quantification.

Usage (adapters/holoocean/.venv):

    .venv/bin/python tools/sonar_stats.py --bag sim_1200khz.mcap \
        [--compare real_1200khz.mcap] [--topic /raw/sonar_frame] \
        [--max-frames 200] --out docs/perf/sonar_stats_<date>.json [--plot out.png]

Decodes every `uw.domain.SonarFrame` on the topic (uint8 row-major
[num_ranges, num_beams], rows = range, columns = ascending bearing) and
reports, per bag:

  * intensity histogram (256 bins, all frames aggregated);
  * noise floor: Rayleigh scale sigma of the target-free cells. Target-free
    is decided by a robust two-step rule — a first sigma from the median of
    all non-zero cells (Rayleigh median = sigma * sqrt(2 ln 2), robust to a
    small target fraction), then the floor is every non-zero cell below the
    Rayleigh 99.9 % quantile (sigma * 3.717) and sigma is re-estimated on
    those cells by the Rayleigh MLE sqrt(mean(x^2) / 2) with Sheppard's
    quantisation correction. Zero-valued cells are excluded (HoloOcean clips
    negative additive noise to 0, so zeros carry no amplitude information)
    and reported separately as `zero_fraction`;
  * target-region signal-to-clutter ratio: mean of the above-threshold cells
    over the Rayleigh mean of the floor (sigma * sqrt(pi/2));
  * intensity-vs-range attenuation: mean intensity per range row for all
    cells and for floor cells only;
  * per-beam mean profile (bearing-dependent gain/shadowing);
  * non-zero and saturated (255) cell fractions.

With --compare the same numbers are computed for the second bag and a diff
table (B / A ratio) is emitted, which is how HoloOcean's AddSigma /
MultSigma / RangeSigma get tuned against the real sonar on delivery day —
see adapters/holoocean/docs/sonar-stats-procedure.md. Output is JSON plus a
markdown table next to it (same convention as tick_budget.py). The stats
are pure numpy functions (`frame_stats`, `aggregate_stats`, ...) tested in
tests/test_sonar_stats.py; only `load_frames_from_mcap` touches MCAP.
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

_ADAPTER_DIR = Path(__file__).resolve().parents[1]
_SCHEMA_DIR = _ADAPTER_DIR / "uw_holoocean_adapter" / "schema_pb2"

_RAYLEIGH_MEDIAN_FACTOR = math.sqrt(2.0 * math.log(2.0))   # median = sigma * 1.1774
_RAYLEIGH_MEAN_FACTOR = math.sqrt(math.pi / 2.0)           # mean   = sigma * 1.2533
_TARGET_QUANTILE_FACTOR = math.sqrt(-2.0 * math.log(1e-3))  # P(x > k sigma) = 1e-3 -> k = 3.717


def _bootstrap_paths() -> None:
    for entry in (str(_ADAPTER_DIR), str(_SCHEMA_DIR)):
        if entry not in sys.path:
            sys.path.insert(0, entry)


# ---------------------------------------------------------------------------
# Pure stats
# ---------------------------------------------------------------------------


@dataclasses.dataclass(frozen=True)
class DecodedFrame:
    intensity: np.ndarray      # uint8 (num_ranges, num_beams)
    range_centers_m: np.ndarray  # (num_ranges,)
    azimuth_rad: np.ndarray    # (num_beams,)


def range_centers(range_bins: Sequence[float], num_ranges: int, min_range: float, max_range: float) -> np.ndarray:
    """Handles both `range_bins` conventions the proto allows: num_ranges
    centres, or num_ranges + 1 edges (what sonar_conversion.py writes)."""
    bins = np.asarray(range_bins, dtype=np.float64)
    if bins.size == num_ranges + 1:
        return 0.5 * (bins[:-1] + bins[1:])
    if bins.size == num_ranges:
        return bins
    step = (max_range - min_range) / max(num_ranges, 1)
    return min_range + (np.arange(num_ranges) + 0.5) * step


def decode_sonar_frame(frame) -> DecodedFrame:
    rows, cols = int(frame.num_ranges), int(frame.num_beams)
    data = np.frombuffer(frame.intensity_tensor, dtype=np.uint8)
    if data.size != rows * cols:
        raise ValueError(f"intensity_tensor has {data.size} bytes, expected {rows}x{cols}")
    return DecodedFrame(
        intensity=data.reshape(rows, cols),
        range_centers_m=range_centers(frame.range_bins, rows, float(frame.min_range), float(frame.max_range)),
        azimuth_rad=np.asarray(frame.azimuth_angles, dtype=np.float64),
    )


def rayleigh_sigma_mle(values: np.ndarray, *, quantised: bool = True) -> float:
    """sigma = sqrt(E[x^2] / 2); Sheppard's correction (-1/12 LSB^2) removes
    the variance that uint8 rounding adds."""
    v = np.asarray(values, dtype=np.float64)
    if v.size == 0:
        return 0.0
    second_moment = float(np.mean(v * v))
    if quantised:
        second_moment = max(second_moment - 1.0 / 12.0, 0.0)
    return math.sqrt(second_moment / 2.0)


def floor_threshold(intensity: np.ndarray) -> float:
    """Robust initial floor/target split: Rayleigh 99.9 % quantile of a sigma
    read from the median of the non-zero cells."""
    nonzero = intensity[intensity > 0]
    if nonzero.size == 0:
        return 0.0
    sigma0 = float(np.median(nonzero)) / _RAYLEIGH_MEDIAN_FACTOR
    return sigma0 * _TARGET_QUANTILE_FACTOR


def aggregate_stats(frames: Sequence[np.ndarray], range_centers_m: Optional[np.ndarray] = None) -> Dict[str, object]:
    """Statistics over a stack of uint8 (num_ranges, num_beams) frames (all
    frames must share a shape)."""
    if not frames:
        raise ValueError("no frames")
    stack = np.stack([np.asarray(f, dtype=np.uint8) for f in frames], axis=0)
    n_frames, rows, cols = stack.shape
    flat = stack.reshape(-1).astype(np.float64)
    histogram = np.bincount(stack.reshape(-1), minlength=256).astype(np.int64)

    threshold = floor_threshold(stack)
    floor_mask = (stack > 0) & (stack <= threshold)
    target_mask = stack > threshold
    floor_values = flat[floor_mask.reshape(-1)]
    target_values = flat[target_mask.reshape(-1)]
    sigma = rayleigh_sigma_mle(floor_values)
    floor_mean = sigma * _RAYLEIGH_MEAN_FACTOR
    target_mean = float(np.mean(target_values)) if target_values.size else 0.0
    scr = target_mean / floor_mean if floor_mean > 0 else float("inf") if target_mean > 0 else 0.0

    per_range_all = stack.mean(axis=(0, 2))
    with np.errstate(invalid="ignore", divide="ignore"):
        per_range_floor = np.where(
            floor_mask.sum(axis=(0, 2)) > 0,
            (stack * floor_mask).sum(axis=(0, 2)) / np.maximum(floor_mask.sum(axis=(0, 2)), 1),
            np.nan,
        )
    per_beam_all = stack.mean(axis=(0, 1))

    if range_centers_m is None:
        range_centers_m = np.arange(rows, dtype=np.float64)

    return {
        "frames": int(n_frames),
        "num_ranges": int(rows),
        "num_beams": int(cols),
        "histogram": histogram.tolist(),
        "mean_intensity": float(flat.mean()),
        "nonzero_fraction": float(np.count_nonzero(stack) / flat.size),
        "zero_fraction": float(1.0 - np.count_nonzero(stack) / flat.size),
        "saturation_fraction": float(np.count_nonzero(stack == 255) / flat.size),
        "floor_threshold": float(threshold),
        "floor_cell_fraction": float(floor_mask.sum() / flat.size),
        "floor_rayleigh_sigma": float(sigma),
        "floor_rayleigh_mean": float(floor_mean),
        "floor_sample_std": float(np.std(floor_values)) if floor_values.size else 0.0,
        "target_cell_fraction": float(target_mask.sum() / flat.size),
        "target_mean": target_mean,
        "signal_to_clutter_ratio": float(scr),
        "signal_to_clutter_db": float(10.0 * math.log10(scr)) if 0 < scr < float("inf") else None,
        "range_centers_m": np.asarray(range_centers_m, dtype=np.float64).tolist(),
        "per_range_mean_all": per_range_all.tolist(),
        "per_range_mean_floor": [None if np.isnan(v) else float(v) for v in per_range_floor],
        "per_beam_mean": per_beam_all.tolist(),
    }


_SCALAR_KEYS = (
    "frames", "num_ranges", "num_beams", "mean_intensity", "nonzero_fraction", "saturation_fraction",
    "floor_threshold", "floor_rayleigh_sigma", "floor_sample_std", "target_cell_fraction", "target_mean",
    "signal_to_clutter_ratio", "signal_to_clutter_db",
)


def compare_stats(a: Dict[str, object], b: Dict[str, object]) -> List[Tuple[str, object, object, Optional[float]]]:
    """Rows (key, A, B, B/A ratio or None)."""
    rows = []
    for key in _SCALAR_KEYS:
        va, vb = a.get(key), b.get(key)
        ratio = None
        if isinstance(va, (int, float)) and isinstance(vb, (int, float)) and va not in (0, None) and vb is not None:
            ratio = float(vb) / float(va)
        rows.append((key, va, vb, ratio))
    return rows


def render_markdown(stats_by_name: Dict[str, Dict[str, object]]) -> str:
    names = list(stats_by_name)
    lines = ["| metric | " + " | ".join(names) + (" | B/A |" if len(names) == 2 else " |"),
             "|---|" + "---|" * len(names) + ("---|" if len(names) == 2 else "")]
    if len(names) == 2:
        for key, va, vb, ratio in compare_stats(stats_by_name[names[0]], stats_by_name[names[1]]):
            lines.append(f"| {key} | {_fmt(va)} | {_fmt(vb)} | {_fmt(ratio)} |")
    else:
        for key in _SCALAR_KEYS:
            lines.append(f"| {key} | {_fmt(stats_by_name[names[0]].get(key))} |")
    return "\n".join(lines) + "\n"


def _fmt(v) -> str:
    if v is None:
        return "n/a"
    if isinstance(v, float):
        return f"{v:.4g}"
    return str(v)


# ---------------------------------------------------------------------------
# MCAP shell + plotting
# ---------------------------------------------------------------------------


def load_frames_from_mcap(path: str, topic: str, max_frames: Optional[int] = None) -> List[DecodedFrame]:
    _bootstrap_paths()
    from uw.domain import sonar_pb2  # noqa: E402

    from uw_holoocean_adapter.canonical_writer import read_canonical_messages  # noqa: E402

    frames: List[DecodedFrame] = []
    for _log_time_ns, frame in read_canonical_messages(path, topic, sonar_pb2.SonarFrame):
        frames.append(decode_sonar_frame(frame))
        if max_frames is not None and len(frames) >= max_frames:
            break
    if not frames:
        raise ValueError(f"no SonarFrame messages on {topic!r} in {path}")
    return frames


def stats_for_bag(path: str, topic: str, max_frames: Optional[int]) -> Dict[str, object]:
    frames = load_frames_from_mcap(path, topic, max_frames)
    shape = frames[0].intensity.shape
    usable = [f.intensity for f in frames if f.intensity.shape == shape]
    if len(usable) != len(frames):
        print(f"warning: {len(frames) - len(usable)} frames with a different shape than {shape} skipped", file=sys.stderr)
    stats = aggregate_stats(usable, frames[0].range_centers_m)
    stats["bag"] = path
    return stats


def _plot(path: str, stats_by_name: Dict[str, Dict[str, object]]) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:  # pragma: no cover - optional extra
        raise SystemExit("matplotlib is not installed; install the 'plot' extra or omit --plot") from exc
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))
    for name, st in stats_by_name.items():
        hist = np.asarray(st["histogram"], dtype=np.float64)
        axes[0].semilogy(np.arange(256), hist / hist.sum(), label=name)
        r = np.asarray(st["range_centers_m"])
        axes[1].plot(r, st["per_range_mean_all"], label=f"{name} all")
        floor = np.asarray([np.nan if v is None else v for v in st["per_range_mean_floor"]], dtype=np.float64)
        axes[1].plot(r, floor, "--", label=f"{name} floor")
        axes[2].plot(st["per_beam_mean"], label=name)
    axes[0].set_title("intensity histogram (uint8)"); axes[0].set_xlabel("intensity"); axes[0].legend(fontsize=8)
    axes[1].set_title("mean intensity vs range"); axes[1].set_xlabel("range [m]"); axes[1].legend(fontsize=8)
    axes[2].set_title("mean intensity per beam"); axes[2].set_xlabel("beam index"); axes[2].legend(fontsize=8)
    for ax in axes:
        ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=120)


def main(argv: Optional[Sequence[str]] = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bag", required=True, help="MCAP with SonarFrame messages (bag A)")
    parser.add_argument("--compare", default=None, help="second MCAP (bag B) to diff against A")
    parser.add_argument("--topic", default="/raw/sonar_frame")
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--out", required=True, help="JSON output; a .md table is written next to it")
    parser.add_argument("--plot", default=None, help="optional PNG (needs matplotlib)")
    args = parser.parse_args(argv)

    stats_by_name: Dict[str, Dict[str, object]] = {"A": stats_for_bag(args.bag, args.topic, args.max_frames)}
    if args.compare:
        stats_by_name["B"] = stats_for_bag(args.compare, args.topic, args.max_frames)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(stats_by_name, indent=2), encoding="utf8")
    table = render_markdown(stats_by_name)
    out.with_suffix(".md").write_text(
        f"# sonar_stats: A={args.bag}" + (f", B={args.compare}" if args.compare else "") + "\n\n" + table,
        encoding="utf8",
    )
    if args.plot:
        _plot(args.plot, stats_by_name)
    print(table, end="")


if __name__ == "__main__":
    main()
