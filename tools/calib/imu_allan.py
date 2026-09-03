#!/usr/bin/env python3
"""PREP-B-05: Allan-deviation IMU noise identification -> rig `imu_noise` YAML.

Usage (runs under adapters/holoocean/.venv, which has numpy/mcap/protobuf and
the generated schema_pb2; nothing under tools/ is otherwise venv-bound):

    adapters/holoocean/.venv/bin/python tools/calib/imu_allan.py \
        --bag /path/to/static.mcap --topic /raw/imu \
        --out imu_noise.yaml --json allan.json --plot allan.png

Reads every `uw.domain.ImuSample` on --topic (capture_time is the clock,
never receive_time / MCAP log_time), de-means each axis, computes the
overlapping Allan deviation per axis on a log-spaced tau grid, and fits the
three classic segments:

  * white noise (slope -1/2): sigma_A(tau) = N / sqrt(tau)  ->  N = sigma_A(1 s),
    reported as the continuous-time noise density `sigma_*_c`
    (gyro rad/s/sqrt(Hz), accel m/s^2/sqrt(Hz));
  * bias instability (flat minimum): B = min(sigma_A) / 0.664
    (gyro rad/s, accel m/s^2) -> `sigma_*_bias`;
  * rate random walk (slope +1/2): sigma_A(tau) = K * sqrt(tau / 3)
    -> K = sigma_A(tau) * sqrt(3 / tau), the bias random-walk density
    (gyro rad/s^2/sqrt(Hz), accel m/s^3/sqrt(Hz)) -> `sigma_*_bias_walk_c`.

Per-axis values are combined with the worst (largest) axis so the rig noise
is conservative. The Allan math lives in pure numpy functions
(`allan_deviation`, `fit_white_noise`, ...) with no MCAP/protobuf import so
adapters/holoocean/tests/test_imu_allan.py can validate them on synthetic
streams with known parameters; the MCAP shell is `load_imu_from_mcap`.

Collection procedure for the real HWT9053-485 and the HoloOcean validation
recipe (including the sigma conversion HoloOcean's per-tick sigmas need)
are in tools/calib/README.md.
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import math
import pathlib
import sys
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
_ADAPTER_DIR = _REPO_ROOT / "adapters" / "holoocean"
_SCHEMA_DIR = _ADAPTER_DIR / "uw_holoocean_adapter" / "schema_pb2"

_AXES = ("x", "y", "z")
_BIAS_INSTABILITY_FACTOR = 0.664  # sigma_A at the flicker floor = 0.664 * B


def _bootstrap_paths() -> None:
    for entry in (str(_ADAPTER_DIR), str(_SCHEMA_DIR)):
        if entry not in sys.path:
            sys.path.insert(0, entry)


# ---------------------------------------------------------------------------
# Pure Allan math (no I/O, no protobuf)
# ---------------------------------------------------------------------------


def default_taus(rate_hz: float, duration_s: float, *, min_tau_s: float = 0.01,
                 max_tau_s: float = 3600.0, points_per_decade: int = 12,
                 min_clusters: int = 9) -> np.ndarray:
    """Log-spaced tau grid from `min_tau_s` (never below one sample period)
    up to `max_tau_s`, additionally capped so every tau still has at least
    `min_clusters` independent clusters (tau <= duration / min_clusters);
    with fewer clusters the estimate is dominated by its own variance."""
    if rate_hz <= 0 or duration_s <= 0:
        raise ValueError("rate_hz and duration_s must be positive")
    dt = 1.0 / rate_hz
    lo = max(min_tau_s, dt)
    hi = min(max_tau_s, duration_s / float(min_clusters))
    if hi <= lo:
        raise ValueError(
            f"recording too short: duration {duration_s:.1f}s allows max tau {hi:.3g}s <= min tau {lo:.3g}s"
        )
    n = max(2, int(math.ceil(math.log10(hi / lo) * points_per_decade)) + 1)
    grid = np.logspace(math.log10(lo), math.log10(hi), n)
    # Snap to whole-sample multiples and de-duplicate so no two taus share a cluster size.
    m = np.unique(np.maximum(1, np.round(grid / dt).astype(np.int64)))
    return m.astype(np.float64) * dt


def allan_deviation(timestamps_s: np.ndarray, samples: np.ndarray, taus: Sequence[float]
                    ) -> Tuple[np.ndarray, np.ndarray]:
    """Overlapping Allan deviation per axis.

    `timestamps_s` (N,) must be monotonic and (approximately) uniformly
    spaced — the mean spacing is used as the sample period and the sample
    index is used for clustering, which is the standard treatment for a
    fixed-rate IMU. `samples` is (N, k). Returns (taus_used, adev (len(taus), k)).
    Each axis is de-meaned first (a static rig's accel carries ~g on one axis
    and the mean does not affect the deviation anyway, but de-meaning keeps
    the cumulative sums numerically small)."""
    t = np.asarray(timestamps_s, dtype=np.float64)
    x = np.asarray(samples, dtype=np.float64)
    if x.ndim == 1:
        x = x[:, None]
    n = x.shape[0]
    if n < 3 or t.shape[0] != n:
        raise ValueError("need at least 3 samples with matching timestamps")
    dt = float(np.mean(np.diff(t)))
    if dt <= 0:
        raise ValueError("timestamps must be strictly increasing on average")
    x = x - x.mean(axis=0, keepdims=True)
    # theta = cumulative integral of the rate signal (angle / velocity), one leading zero.
    theta = np.concatenate([np.zeros((1, x.shape[1])), np.cumsum(x, axis=0) * dt], axis=0)
    taus_used: List[float] = []
    out: List[np.ndarray] = []
    for tau in taus:
        m = int(round(float(tau) / dt))
        if m < 1 or 2 * m >= n:
            continue
        # sigma^2(tau) = 1/(2 tau^2 (N-2m)) * sum (theta[i+2m] - 2 theta[i+m] + theta[i])^2
        d = theta[2 * m:] - 2.0 * theta[m:-m] + theta[:-2 * m]
        var = np.sum(d * d, axis=0) / (2.0 * (m * dt) ** 2 * d.shape[0])
        taus_used.append(m * dt)
        out.append(np.sqrt(var))
    if not out:
        raise ValueError("no tau in range for this recording length")
    return np.asarray(taus_used), np.vstack(out)


def _log_slope(taus: np.ndarray, adev: np.ndarray) -> np.ndarray:
    """Local log-log slope at every grid point (central differences)."""
    lt = np.log(taus)
    la = np.log(np.maximum(adev, 1e-300))
    return np.gradient(la, lt)


@dataclasses.dataclass(frozen=True)
class SegmentFit:
    value: float          # the identified parameter (N, B or K) in rate units
    tau_lo_s: float       # segment used for the fit
    tau_hi_s: float
    points: int


def fit_white_noise(taus: np.ndarray, adev: np.ndarray, *, slope_tol: float = 0.15,
                    min_points: int = 3) -> SegmentFit:
    """Fits sigma_A = N / sqrt(tau) on the longest leading run of points whose
    local slope is within `slope_tol` of -1/2 (falls back to the first
    `min_points` if no run qualifies) and returns N = sigma_A(1 s)."""
    taus = np.asarray(taus, dtype=np.float64)
    adev = np.asarray(adev, dtype=np.float64)
    slope = _log_slope(taus, adev)
    mask = np.abs(slope + 0.5) <= slope_tol
    # leading run: white noise is the shortest-tau regime
    end = 0
    while end < len(mask) and mask[end]:
        end += 1
    if end < min_points:
        end = min(min_points, len(taus))
    seg_t, seg_a = taus[:end], adev[:end]
    # least squares on log sigma = log N - 0.5 log tau
    log_n = np.mean(np.log(seg_a) + 0.5 * np.log(seg_t))
    return SegmentFit(float(np.exp(log_n)), float(seg_t[0]), float(seg_t[-1]), int(end))


def fit_bias_instability(taus: np.ndarray, adev: np.ndarray) -> SegmentFit:
    """Bias instability from the Allan minimum: B = min(sigma_A) / 0.664.
    If the curve never flattens (still falling at the longest tau) this is an
    upper bound, flagged by tau_hi_s == the last tau; the README explains why
    a longer static recording is then needed."""
    taus = np.asarray(taus, dtype=np.float64)
    adev = np.asarray(adev, dtype=np.float64)
    i = int(np.argmin(adev))
    return SegmentFit(float(adev[i] / _BIAS_INSTABILITY_FACTOR), float(taus[i]), float(taus[i]), 1)


def fit_allan_model(taus: np.ndarray, adev: np.ndarray) -> Tuple[float, float, float]:
    """Joint non-negative least-squares fit of the three-term Allan variance
    model sigma_A^2(tau) = N^2 / tau + C^2 + K^2 tau / 3 with relative
    (1/sigma^2) weighting, returning (N, C, K). Unlike a slope-window fit
    this stays unbiased where the white-noise and random-walk regimes
    overlap (a 30-minute recording at 200 Hz only just reaches the +1/2
    slope), which is why the reported random walk comes from here. Terms
    whose coefficient goes negative are dropped (active-set NNLS)."""
    taus = np.asarray(taus, dtype=np.float64)
    adev = np.asarray(adev, dtype=np.float64)
    basis = np.stack([1.0 / taus, np.ones_like(taus), taus / 3.0], axis=1)
    y = adev ** 2
    w = 1.0 / np.maximum(y, 1e-300)
    active = [0, 1, 2]
    coeffs = np.zeros(3)
    while active:
        c, _, _, _ = np.linalg.lstsq(basis[:, active] * w[:, None], y * w, rcond=None)
        if np.all(c >= 0.0):
            coeffs[active] = c
            break
        active = [a for a, ci in zip(active, c) if ci > 0.0]
    n, c_flat, k = np.sqrt(np.maximum(coeffs, 0.0))
    return float(n), float(c_flat), float(k)


def fit_random_walk(taus: np.ndarray, adev: np.ndarray, *, slope_tol: float = 0.2,
                    min_points: int = 2) -> Optional[SegmentFit]:
    """Rate random walk K (sigma_A = K sqrt(tau/3)). The value comes from
    `fit_allan_model`; the tau range reported is the trailing run of points
    whose local slope is within `slope_tol` of +1/2 (after the Allan
    minimum). Returns None when neither the joint fit nor the slope test
    sees a random walk — the recording was too short to reach that regime,
    in which case the rig keeps the walk at 0 and the estimator falls back to
    `sigma_*_bias`."""
    taus = np.asarray(taus, dtype=np.float64)
    adev = np.asarray(adev, dtype=np.float64)
    _n, _c, k = fit_allan_model(taus, adev)
    if k <= 0.0:
        return None
    i_min = int(np.argmin(adev))
    slope = _log_slope(taus, adev)
    mask = (np.abs(slope - 0.5) <= slope_tol)
    mask[: i_min + 1] = False
    idx = np.flatnonzero(mask)
    if idx.size < min_points:
        return None
    end = int(idx[-1])
    start = end
    while start - 1 in idx:
        start -= 1
    if end - start + 1 < min_points:
        return None
    return SegmentFit(k, float(taus[start]), float(taus[end]), int(end - start + 1))


@dataclasses.dataclass(frozen=True)
class AxisResult:
    white_noise: SegmentFit
    bias_instability: SegmentFit
    random_walk: Optional[SegmentFit]


def analyze_axes(taus: np.ndarray, adev: np.ndarray) -> List[AxisResult]:
    return [
        AxisResult(
            white_noise=fit_white_noise(taus, adev[:, k]),
            bias_instability=fit_bias_instability(taus, adev[:, k]),
            random_walk=fit_random_walk(taus, adev[:, k]),
        )
        for k in range(adev.shape[1])
    ]


def worst_axis(results: Sequence[AxisResult], field: str) -> float:
    """Conservative combination: the largest per-axis value; 0.0 for a
    random walk that no axis reached."""
    values = []
    for r in results:
        fit = getattr(r, field)
        if fit is not None:
            values.append(fit.value)
    return float(max(values)) if values else 0.0


def imu_noise_fragment(gyro: Sequence[AxisResult], accel: Sequence[AxisResult], *,
                       rate_hz: float, gravity_mps2: float) -> Dict[str, float]:
    """The rig `imu_noise` block (include/runtime/config.hpp's loader keys)."""
    return {
        "sigma_gyro_c": worst_axis(gyro, "white_noise"),
        "sigma_accel_c": worst_axis(accel, "white_noise"),
        "sigma_gyro_bias": worst_axis(gyro, "bias_instability"),
        "sigma_accel_bias": worst_axis(accel, "bias_instability"),
        "sigma_gyro_bias_walk_c": worst_axis(gyro, "random_walk"),
        "sigma_accel_bias_walk_c": worst_axis(accel, "random_walk"),
        "rate_hz": float(rate_hz),
        "gravity_mps2": float(gravity_mps2),
    }


def render_yaml(fragment: Dict[str, float], *, source: str) -> str:
    lines = [
        "# Generated by tools/calib/imu_allan.py (PREP-B-05) from " + source,
        "# Units: sigma_gyro_c rad/s/sqrt(Hz), sigma_accel_c m/s^2/sqrt(Hz) (white-noise",
        "#   density, Allan slope -1/2 read at tau = 1 s); sigma_gyro_bias rad/s,",
        "#   sigma_accel_bias m/s^2 (bias instability, Allan minimum / 0.664);",
        "#   sigma_gyro_bias_walk_c rad/s^2/sqrt(Hz), sigma_accel_bias_walk_c m/s^3/sqrt(Hz)",
        "#   (rate random walk, Allan slope +1/2; 0.0 = regime not reached, estimator",
        "#   falls back to sigma_*_bias). Worst axis of three is reported.",
        "imu_noise:",
    ]
    for key, value in fragment.items():
        if key == "rate_hz":
            lines.append(f"  {key}: {value:g}")
        else:
            lines.append(f"  {key}: {value:.6e}")
    return "\n".join(lines) + "\n"


# ---------------------------------------------------------------------------
# MCAP shell
# ---------------------------------------------------------------------------


def load_imu_from_mcap(path: str, topic: str) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Returns (timestamps_s, gyro (N,3), accel (N,3)) from `topic`'s ImuSample
    stream, ordered by capture_time."""
    _bootstrap_paths()
    from uw.domain import imu_pb2  # noqa: E402  (generated, see tools/codegen/gen_py.sh)

    from uw_holoocean_adapter.canonical_writer import read_canonical_messages  # noqa: E402

    t: List[float] = []
    gyro: List[List[float]] = []
    accel: List[List[float]] = []
    for _log_time_ns, sample in read_canonical_messages(path, topic, imu_pb2.ImuSample):
        stamp = sample.header.capture_time
        t.append(stamp.seconds + stamp.nanos * 1e-9)
        gyro.append(list(sample.angular_velocity_radps))
        accel.append(list(sample.linear_acceleration_mps2))
    if not t:
        raise ValueError(f"no ImuSample messages on topic {topic!r} in {path}")
    order = np.argsort(np.asarray(t), kind="stable")
    return (np.asarray(t)[order], np.asarray(gyro)[order], np.asarray(accel)[order])


def _fit_to_dict(fit: Optional[SegmentFit]) -> Optional[Dict[str, float]]:
    return None if fit is None else dataclasses.asdict(fit)


def _plot(path: str, taus: np.ndarray, gyro_adev: np.ndarray, accel_adev: np.ndarray,
          gyro_res: Sequence[AxisResult], accel_res: Sequence[AxisResult]) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:  # pragma: no cover - optional extra
        raise SystemExit(
            "matplotlib is not installed; install the 'plot' extra "
            "(uv pip install -e '.[plot]' in adapters/holoocean) or omit --plot"
        ) from exc
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    for ax, adev, res, title, unit in (
        (axes[0], gyro_adev, gyro_res, "gyro", "rad/s"),
        (axes[1], accel_adev, accel_res, "accel", "m/s^2"),
    ):
        for k, name in enumerate(_AXES):
            ax.loglog(taus, adev[:, k], label=f"{name}")
        n = worst_axis(res, "white_noise")
        ax.loglog(taus, n / np.sqrt(taus), "k--", lw=0.8, label=f"N={n:.3g}")
        k_walk = worst_axis(res, "random_walk")
        if k_walk > 0:
            ax.loglog(taus, k_walk * np.sqrt(taus / 3.0), "k:", lw=0.8, label=f"K={k_walk:.3g}")
        ax.set_title(f"Allan deviation ({title})")
        ax.set_xlabel("tau [s]")
        ax.set_ylabel(f"sigma_A [{unit}]")
        ax.grid(True, which="both", alpha=0.3)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=120)


def main(argv: Optional[Sequence[str]] = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bag", required=True, help="MCAP file with a static-rig IMU recording")
    parser.add_argument("--topic", default="/raw/imu")
    parser.add_argument("--out", required=True, help="YAML fragment (rig imu_noise block) to write")
    parser.add_argument("--json", default=None, help="optional JSON dump of curves and fits")
    parser.add_argument("--plot", default=None, help="optional PNG Allan plot (needs matplotlib)")
    parser.add_argument("--max-tau-s", type=float, default=3600.0)
    parser.add_argument("--gravity", type=float, default=9.80665)
    args = parser.parse_args(argv)

    t, gyro, accel = load_imu_from_mcap(args.bag, args.topic)
    duration = float(t[-1] - t[0])
    rate_hz = (len(t) - 1) / duration if duration > 0 else 0.0
    taus = default_taus(rate_hz, duration, max_tau_s=args.max_tau_s)
    taus_g, adev_g = allan_deviation(t, gyro, taus)
    taus_a, adev_a = allan_deviation(t, accel, taus)
    gyro_res = analyze_axes(taus_g, adev_g)
    accel_res = analyze_axes(taus_a, adev_a)
    fragment = imu_noise_fragment(gyro_res, accel_res, rate_hz=round(rate_hz, 3), gravity_mps2=args.gravity)

    pathlib.Path(args.out).write_text(render_yaml(fragment, source=f"{args.bag} ({args.topic})"), encoding="utf8")
    if args.json:
        payload = {
            "bag": args.bag,
            "topic": args.topic,
            "samples": int(len(t)),
            "duration_s": duration,
            "rate_hz": rate_hz,
            "taus_s": taus_g.tolist(),
            "gyro_adev": adev_g.tolist(),
            "accel_adev": adev_a.tolist(),
            "gyro_fits": [
                {"white_noise": _fit_to_dict(r.white_noise), "bias_instability": _fit_to_dict(r.bias_instability),
                 "random_walk": _fit_to_dict(r.random_walk)} for r in gyro_res],
            "accel_fits": [
                {"white_noise": _fit_to_dict(r.white_noise), "bias_instability": _fit_to_dict(r.bias_instability),
                 "random_walk": _fit_to_dict(r.random_walk)} for r in accel_res],
            "imu_noise": fragment,
        }
        pathlib.Path(args.json).write_text(json.dumps(payload, indent=2), encoding="utf8")
    if args.plot:
        _plot(args.plot, taus_g, adev_g, adev_a, gyro_res, accel_res)

    print(f"{len(t)} samples, {duration:.1f} s, {rate_hz:.2f} Hz, tau {taus_g[0]:.3g}..{taus_g[-1]:.3g} s")
    for key, value in fragment.items():
        print(f"  {key}: {value:.6g}")
    unreached = [n for n, res in (("gyro", gyro_res), ("accel", accel_res))
                 if all(r.random_walk is None for r in res)]
    if unreached:
        print(f"  note: random-walk regime not reached for {', '.join(unreached)} (walk left at 0; record longer)")


if __name__ == "__main__":
    main()
