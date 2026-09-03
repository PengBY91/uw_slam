"""PREP-A-01: measure HoloOcean's real tick budget per sensor configuration.

Runs the contract-vehicle sensor variants (docs/ROV平台到货前准备工作规格-2026-09-02.md
PREP-A-01) against the real simulator and records, per variant: achieved
ticks/s, wall-clock ms per tick, real-time factor (sim seconds per wall
second), per-sensor message rate, and GPU memory/utilisation sampled
mid-run. The sonar render cost is inferred from the (c) vs (c_nosonar)
difference -- HoloOcean exposes no per-sensor timing.

Runs where real HoloOcean runs (the native Windows host, see
holoocean_bridge_sensor_host.py); invoke as a .py file via powershell.exe from
WSL2. Writes JSON + a markdown table next to --out.
"""
from __future__ import annotations

import argparse
import copy
import json
import subprocess
import sys
import time
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from uw_holoocean_adapter.holoocean_driver import HoloOceanSession  # noqa: E402
from uw_holoocean_adapter.scenario_manifest import load_realtime_manifest  # noqa: E402
from uw_holoocean_adapter.scenario_randomization import ScenarioRandomization  # noqa: E402


def _sensors(cfg):
    return cfg["agents"][0]["sensors"]


def _drop(cfg, names):
    cfg["agents"][0]["sensors"] = [s for s in _sensors(cfg) if s["sensor_name"] not in names]


def _find(cfg, name):
    for s in _sensors(cfg):
        if s["sensor_name"] == name:
            return s
    raise KeyError(name)


def _mono(cfg, width, height, hz):
    _drop(cfg, {"LeftCamera", "RightCamera"})
    cam = _find(cfg, "PilotCamera")
    cam["sensor_name"] = "MainCamera"
    cam["Hz"] = hz
    cam["configuration"]["CaptureWidth"] = width
    cam["configuration"]["CaptureHeight"] = height


def _sonar(cfg, hz, range_bins):
    s = _find(cfg, "ImagingSonar")
    s["Hz"] = hz
    s["configuration"]["RangeBins"] = range_bins


def _clock(cfg, ticks_per_sec, frames_per_sec):
    """frames_per_sec is HoloOcean's wall-clock FPS cap (True == match
    ticks_per_sec, False == uncapped, number == cap). The base scenario pins
    20, which is exactly the 20 ticks/s every round-1 variant measured."""
    cfg["ticks_per_sec"] = ticks_per_sec
    cfg["frames_per_sec"] = frames_per_sec


def _rates(cfg, camera_hz=None, sonar_hz=None, imu_hz=None):
    for s in _sensors(cfg):
        if s["sensor_type"] == "RGBCamera" and camera_hz:
            s["Hz"] = camera_hz
        if s["sensor_type"] == "ImagingSonar" and sonar_hz:
            s["Hz"] = sonar_hz
        if s["sensor_type"] in ("IMUSensor", "OrientationSensor", "DepthSensor") and imu_hz:
            s["Hz"] = imu_hz


VARIANTS = {
    # round 2 (2026-09-02): frames_per_sec uncapped -> true GPU/physics ceiling;
    # sensor Hz chosen to divide ticks_per_sec exactly (HoloOcean quantises
    # sensor rates to whole ticks: 30 Hz on 100 ticks/s came out as 33.4 Hz).
    "u1_uncapped_t100_mono1080p25_sonar10x512": lambda c: (
        _mono(c, 1920, 1080, 25), _clock(c, 100, False)),
    "u2_uncapped_t100_nocamera_sonar10x512": lambda c: (
        _drop(c, {"LeftCamera", "RightCamera", "PilotCamera"}), _clock(c, 100, False)),
    "u3_uncapped_t100_mono960x540_25_sonar20x256": lambda c: (
        _mono(c, 960, 540, 25), _sonar(c, 20, 256), _clock(c, 100, False)),
    "u4_uncapped_t200_mono1080p25_sonar20x512_imu200": lambda c: (
        _mono(c, 1920, 1080, 25), _sonar(c, 20, 512), _rates(c, imu_hz=200), _clock(c, 200, False)),
    "u5_uncapped_t100_mono1080p25_sonar25x512": lambda c: (
        _mono(c, 1920, 1080, 25), _sonar(c, 25, 512), _clock(c, 100, False)),
    # round 1: the contract-vehicle variants with the base scenario's frames_per_sec: 20 cap
    "a_current_stereo720p20_sonar10x512": lambda c: None,
    "b_mono960x540_20hz_sonar10x512": lambda c: _mono(c, 960, 540, 20),
    "c_mono1080p_30hz_sonar10x512": lambda c: _mono(c, 1920, 1080, 30),
    "c_nosonar_mono1080p_30hz": lambda c: (_mono(c, 1920, 1080, 30), _drop(c, {"ImagingSonar"})),
    "d_mono1080p_30hz_sonar40x512": lambda c: (_mono(c, 1920, 1080, 30), _sonar(c, 40, 512)),
    "e_mono1080p_30hz_sonar20x256": lambda c: (_mono(c, 1920, 1080, 30), _sonar(c, 20, 256)),
}


def gpu_sample():
    try:
        out = subprocess.run(
            ["nvidia-smi", "--query-gpu=memory.used,utilization.gpu", "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=10,
        ).stdout.strip().split(",")
        return {"gpu_mem_used_mib": float(out[0]), "gpu_util_pct": float(out[1])}
    except Exception as error:  # noqa: BLE001
        return {"gpu_mem_used_mib": None, "gpu_util_pct": None, "gpu_error": str(error)}


def run_variant(name, base_cfg, warmup, ticks):
    cfg = copy.deepcopy(base_cfg)
    VARIANTS[name](cfg)
    sensor_summary = [
        f"{s['sensor_name']}@{s.get('Hz')}Hz"
        + (f" {s['configuration'].get('CaptureWidth')}x{s['configuration'].get('CaptureHeight')}"
           if s["sensor_type"] == "RGBCamera" else "")
        + (f" {s['configuration'].get('AzimuthBins')}x{s['configuration'].get('RangeBins')}"
           if s["sensor_type"] == "ImagingSonar" else "")
        for s in _sensors(cfg)
    ]
    print(f"[{name}] launching: {sensor_summary}", flush=True)
    t_launch = time.perf_counter()
    session = HoloOceanSession(cfg, seed=42, randomization=ScenarioRandomization())
    launch_s = time.perf_counter() - t_launch
    command = [0.0] * 8
    counts = {}
    result = {"variant": name, "sensors": sensor_summary, "launch_s": round(launch_s, 1),
              "ticks_per_sec_configured": cfg["ticks_per_sec"]}
    try:
        for _ in range(warmup):
            session.step(command)
        gpu_idle = gpu_sample()
        t0 = time.perf_counter()
        first = session.step(command)
        sim_t0 = first.sim_time_s
        for key in first.sensors:
            counts[key] = counts.get(key, 0) + 1
        per_tick = []
        last = time.perf_counter()
        gpu_mid = None
        frame = first
        for i in range(1, ticks):
            frame = session.step(command)
            now = time.perf_counter()
            per_tick.append(now - last)
            last = now
            for key in frame.sensors:
                counts[key] = counts.get(key, 0) + 1
            if i == ticks // 2:
                gpu_mid = gpu_sample()
        wall = time.perf_counter() - t0
        sim_elapsed = frame.sim_time_s - sim_t0
        per_tick.sort()
        result.update({
            "ticks": ticks,
            "wall_s": round(wall, 2),
            "achieved_ticks_per_s": round(ticks / wall, 2),
            "ms_per_tick_mean": round(1000 * wall / ticks, 1),
            "ms_per_tick_p50": round(1000 * per_tick[len(per_tick) // 2], 1),
            "ms_per_tick_p95": round(1000 * per_tick[int(len(per_tick) * 0.95)], 1),
            "sim_elapsed_s": round(sim_elapsed, 3),
            "real_time_factor": round(sim_elapsed / wall, 3) if wall > 0 else None,
            "sensor_msgs_per_sim_s": {k: round(v / sim_elapsed, 1) for k, v in counts.items() if sim_elapsed > 0},
            "gpu_mid_run": gpu_mid,
            "gpu_before_run": gpu_idle,
        })
        print(f"[{name}] {result['achieved_ticks_per_s']} ticks/s, {result['ms_per_tick_mean']} ms/tick, "
              f"RTF {result['real_time_factor']}, gpu {gpu_mid}", flush=True)
    finally:
        session.close()
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", default=str(ROOT / "scenarios" / "blue_rov_aid_sv1213_base.json"))
    parser.add_argument("--task", default=str(ROOT / "scenarios" / "aquaculture_search.yaml"))
    parser.add_argument("--variants", nargs="*", default=list(VARIANTS))
    parser.add_argument("--warmup", type=int, default=40)
    parser.add_argument("--ticks", type=int, default=300)
    parser.add_argument("--out", default=str(ROOT / "tick_budget.json"))
    args = parser.parse_args()

    manifest = load_realtime_manifest(args.scenario, args.task)
    base_cfg = manifest.holoocean_scenario_cfg()
    results = []
    for name in args.variants:
        try:
            results.append(run_variant(name, base_cfg, args.warmup, args.ticks))
        except Exception:  # noqa: BLE001
            traceback.print_exc()
            results.append({"variant": name, "error": traceback.format_exc().splitlines()[-1]})
        time.sleep(3)  # let the UE5 process exit before the next launch
    Path(args.out).write_text(json.dumps(results, indent=2), encoding="utf8")

    lines = ["| variant | ticks/s | ms/tick mean (p95) | RTF | GPU mem MiB / util % | launch s | sensors |", "|---|---|---|---|---|---|---|"]
    for r in results:
        if "error" in r:
            lines.append(f"| {r['variant']} | ERROR | {r['error']} | | | | |")
            continue
        g = r.get("gpu_mid_run") or {}
        lines.append(
            f"| {r['variant']} | {r['achieved_ticks_per_s']} | {r['ms_per_tick_mean']} ({r['ms_per_tick_p95']}) | "
            f"{r['real_time_factor']} | {g.get('gpu_mem_used_mib')} / {g.get('gpu_util_pct')} | {r['launch_s']} | "
            f"{'; '.join(r['sensors'])} |"
        )
    Path(args.out).with_suffix(".md").write_text("\n".join(lines) + "\n", encoding="utf8")
    print("\n".join(lines), flush=True)


if __name__ == "__main__":
    main()
