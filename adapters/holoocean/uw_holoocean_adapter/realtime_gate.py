"""Supervises the four isolated processes a realtime closed-loop gate run
needs -- the HoloOcean session (Task 3's `realtime_ros_session.py`), the
C++ realtime gateway/algorithm/HMI (Task 4's `holoocean_realtime_node`),
the scripted pilot (Task 3's `ScriptedPilot`), and the scorer (Task 5's
`TaskScorer`) -- and produces one `run_report.RunReport`, gated by
`run_report.evaluate_gate`.

Only the scorer process ever touches `/uw/sim/ground_truth`; the scripted
pilot only ever consumes `/uw/hmi/status` and publishes
`/uw/pilot/thrusters`. This module cannot actually be exercised end to end
on this machine -- it needs a real HoloOcean/GPU/ROS2 host (same status as
every native-host-only piece of Tasks 1-5; see adapters/holoocean/
README.md's "What's real vs not tested here").

Process isolation strategy: the HoloOcean session (Task 3's own CLI,
`python -m uw_holoocean_adapter.realtime_ros_session`) and the C++ gateway
(a real, separately-built executable) are launched as genuine OS
subprocesses via `subprocess.Popen`, since both already are, or must be,
standalone executables. The scripted pilot and scorer have no CLI
entrypoint of their own (Tasks 3/5 never added one -- adding one would mean
editing those already-committed files outside this task's scope), so they
run as `multiprocessing.Process` targets calling directly into
`scripted_pilot.ScriptedPilot`/`task_scorer.TaskScorer` -- still genuinely
separate OS processes (fork), just without a second CLI module to
maintain.
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import math
import multiprocessing
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml

from uw_holoocean_adapter.scenario_manifest import load_realtime_manifest
from uw_holoocean_adapter.run_report import (
    GateFailure,
    disturbed_gate,
    evaluate_gate,
    minimum_gate,
    nominal_gate,
    overload_gate,
    tether_limited_gate,
)

_DEFAULT_GATEWAY_BINARY = "build_ros2/bin/holoocean_realtime_node"
_DEFAULT_SCENARIO = "adapters/holoocean/scenarios/blue_rov_aid_sv1213_base.json"

# SIM-ACC-003/SYS-ACC-004/SYS-ACC-005: a 10-seed nominal campaign must reach
# 8/10, a 10-seed disturbed/perturbed campaign must reach 7/10. `minimum`/
# `overload` are single continuous soak runs, not seed campaigns -- every
# seed run (there's normally exactly one) must pass, same as before this
# fraction concept existed.
_REQUIRED_SUCCESS_FRACTION = {"nominal": 0.8, "disturbed": 0.7}


class RealtimeGateError(Exception):
    """A required process could not be started, or exited unexpectedly."""


_FAULT_PROFILE_CHOICES = ("none", "critical", "bandwidth", "critical+bandwidth")


@dataclasses.dataclass(frozen=True)
class BandwidthGateConfig:
    """The `bandwidth:` block of a rov_realtime_*.yaml profile, forwarded
    verbatim to `realtime_ros_session --bandwidth-*` (PREP-E-02)."""

    nominal_mbps: float = 20.0
    min_mbps: float = 10.0
    max_mbps: float = 40.0
    walk_sigma_mbps_per_s: float = 0.0


@dataclasses.dataclass(frozen=True)
class GateProfile:
    profile: str
    duration_s: Optional[float]
    camera_hz: float
    sonar_hz: float
    state_hz: float
    faults_enabled: bool
    disturbance_matrix_enabled: bool
    gate: str
    soak: bool = False
    # PREP-E-02: `fault_profile` used to be read by nothing -- `faults_enabled:
    # true` in a YAML had no runtime effect because run_gate() never passed a
    # --fault-profile to the session subprocess. It is now forwarded (see
    # session_fault_args()). "none" keeps the argv byte-identical to before.
    fault_profile: str = "none"
    fault_seed: Optional[int] = None
    bandwidth: Optional[BandwidthGateConfig] = None


def load_profile(path: str) -> GateProfile:
    profile_path = Path(path)
    if not profile_path.is_file():
        raise RealtimeGateError(f"profile file not found: {path}")
    data = yaml.safe_load(profile_path.read_text())
    camera = data.get("camera", {})
    fault_profile = str(data.get("fault_profile", "none"))
    if fault_profile not in _FAULT_PROFILE_CHOICES:
        raise RealtimeGateError(
            f"{path}: fault_profile {fault_profile!r} is not one of {_FAULT_PROFILE_CHOICES}"
        )
    bandwidth = None
    if "bandwidth" in data and data["bandwidth"] is not None:
        block = data["bandwidth"]
        bandwidth = BandwidthGateConfig(
            nominal_mbps=float(block.get("nominal_mbps", 20.0)),
            min_mbps=float(block.get("min_mbps", 10.0)),
            max_mbps=float(block.get("max_mbps", 40.0)),
            walk_sigma_mbps_per_s=float(block.get("walk_sigma_mbps_per_s", 0.0)),
        )
    fault_seed = data.get("fault_seed")
    return GateProfile(
        profile=data["profile"],
        duration_s=data.get("duration_s"),
        camera_hz=float(camera.get("hz", 20.0)),
        sonar_hz=float(data["sonar_hz"]),
        state_hz=float(data["state_hz"]),
        faults_enabled=bool(data.get("faults_enabled", False)),
        disturbance_matrix_enabled=bool(data.get("disturbance_matrix_enabled", False)),
        gate=str(data.get("gate", data["profile"])),
        soak=bool(data.get("soak", False)),
        fault_profile=fault_profile,
        fault_seed=int(fault_seed) if fault_seed is not None else None,
        bandwidth=bandwidth,
    )


def session_fault_args(profile: GateProfile) -> List[str]:
    """Extra argv for `python -m uw_holoocean_adapter.realtime_ros_session`
    expressing the profile's fault configuration. Empty (argv unchanged
    from before PREP-E-02) when the profile declares no fault_profile; the
    bandwidth flags are only emitted when a `bandwidth` profile is selected
    so a `critical`-only profile does not pick up default link shaping."""
    if profile.fault_profile == "none":
        return []
    args = ["--fault-profile", profile.fault_profile]
    if profile.fault_seed is not None:
        args += ["--fault-seed", str(profile.fault_seed)]
    if "bandwidth" in profile.fault_profile:
        bandwidth = profile.bandwidth or BandwidthGateConfig()
        args += [
            "--bandwidth-mbps", repr(bandwidth.nominal_mbps),
            "--bandwidth-min-mbps", repr(bandwidth.min_mbps),
            "--bandwidth-max-mbps", repr(bandwidth.max_mbps),
            "--bandwidth-walk-sigma", repr(bandwidth.walk_sigma_mbps_per_s),
        ]
    return args


def required_passes(gate_profile: str, seed_count: int) -> int:
    """How many of `seed_count` seed runs must PASS for the campaign as a
    whole to pass, per SIM-ACC-003/SYS-ACC-004/SYS-ACC-005."""
    required_fraction = _REQUIRED_SUCCESS_FRACTION.get(gate_profile, 1.0)
    return math.ceil(required_fraction * seed_count)


def _gate_spec_for(profile: GateProfile, min_duration_s: float):
    if profile.gate == "minimum":
        return minimum_gate()
    if profile.gate == "overload":
        return overload_gate()
    if profile.gate == "disturbed":
        return disturbed_gate()
    if profile.gate == "tether_limited":
        return tether_limited_gate()
    return nominal_gate(min_duration_s=min_duration_s)


class ProcessGroup:
    """Owns every process this gate run started, and always tears every one
    of them down -- even if one already exited, even if the caller raises.
    `run()` guarantees `stop()` runs via try/finally; nothing outside this
    class needs its own cleanup logic."""

    def __init__(self):
        self._subprocesses: List[subprocess.Popen] = []
        self._mp_processes: List[multiprocessing.Process] = []

    def start_subprocess(self, args: List[str]) -> subprocess.Popen:
        process = subprocess.Popen(args)
        self._subprocesses.append(process)
        return process

    def start_process(self, target, args=()) -> multiprocessing.Process:
        process = multiprocessing.Process(target=target, args=args)
        process.start()
        self._mp_processes.append(process)
        return process

    def poll_exited(self) -> List[str]:
        """Names/ids of anything that has already exited, so the caller can
        fail fast on an unexpected exit rather than waiting out the full
        run duration."""
        exited = []
        for process in self._subprocesses:
            if process.poll() is not None:
                exited.append(f"subprocess pid={process.pid} exit_code={process.returncode}")
        for process in self._mp_processes:
            if not process.is_alive() and process.exitcode is not None:
                exited.append(f"process pid={process.pid} exit_code={process.exitcode}")
        return exited

    def stop(self, timeout_s: float = 10.0) -> None:
        for process in self._subprocesses:
            if process.poll() is None:
                process.terminate()
        for process in self._subprocesses:
            try:
                process.wait(timeout=timeout_s)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=timeout_s)
        for process in self._mp_processes:
            if process.is_alive():
                process.terminate()
        for process in self._mp_processes:
            process.join(timeout=timeout_s)
            if process.is_alive():
                process.kill()
                process.join(timeout=timeout_s)


def _run_scripted_pilot_process(scenario_path: str, task_path: str) -> None:
    # Runs entirely within its own forked process (see module docstring for
    # why this isn't a second CLI module). Delegates to pilot_ros_bridge's
    # real rclpy node -- subscribes /uw/hmi/status, publishes
    # /uw/pilot/thrusters -- see docs/archive/rov-realtime-closed-loop-code-review-
    # 2026-08-27.md finding A3 for why this used to be an inert placeholder.
    from uw_holoocean_adapter.pilot_ros_bridge import run_scripted_pilot_bridge
    from uw_holoocean_adapter.scenario_manifest import load_realtime_manifest as _load

    manifest = _load(scenario_path, task_path)
    run_scripted_pilot_bridge(manifest.task)


def _run_scorer_process(scenario_path: str, task_path: str, seed: int, out_path: str) -> None:
    # See _run_scripted_pilot_process's comment -- same real-ROS2-bridge
    # status now, via scorer_ros_bridge (subscribes /uw/sim/ground_truth +
    # /uw/hmi/status, periodically writes its report to out_path).
    from uw_holoocean_adapter.scenario_manifest import load_realtime_manifest as _load
    from uw_holoocean_adapter.scorer_ros_bridge import run_scorer_bridge

    manifest = _load(scenario_path, task_path)
    run_scorer_bridge(manifest.task, seed, out_path)


def run_gate(
    profile_path: str,
    task_path: str,
    seed: int,
    scenario_path: str = _DEFAULT_SCENARIO,
    gateway_binary: str = _DEFAULT_GATEWAY_BINARY,
    soak_duration_s: Optional[float] = None,
    out_dir: str = "/tmp",
) -> dict:
    """Launches the four processes, waits for the run duration (or until
    one exits unexpectedly), tears everything down, and returns the
    (not-yet-gate-evaluated) report dict a caller passes to
    `run_report.evaluate_gate`. Raises `RealtimeGateError` if a required
    process fails to start or exits before the run duration elapses."""
    if not Path(scenario_path).is_file():
        raise RealtimeGateError(f"scenario file not found: {scenario_path}")
    if not Path(task_path).is_file():
        raise RealtimeGateError(f"task file not found: {task_path}")
    load_realtime_manifest(scenario_path, task_path)  # fail fast on an invalid manifest combination

    profile = load_profile(profile_path)
    duration_s = soak_duration_s if soak_duration_s is not None else profile.duration_s
    if duration_s is None:
        raise RealtimeGateError(
            f"{profile_path} does not set duration_s and no --soak-duration-s was given"
        )

    gateway_path = Path(gateway_binary)
    if not gateway_path.is_file():
        raise RealtimeGateError(
            f"gateway binary not found: {gateway_binary} -- build it first with "
            "`cmake -S . -B build_ros2 -DUW_BUILD_ROS2=ON && cmake --build build_ros2 "
            "--target holoocean_realtime_node` on a host with a sourced ROS2 install"
        )

    scorer_out = str(Path(out_dir) / f"task_score_seed{seed}.json")
    # holoocean_realtime_node writes its RuntimeMetricsCollector report here
    # roughly once a second (see RealtimeAssistOutputSink::MaybeWriteReport
    # in src/application/holoocean_realtime_sink.cpp) when given a non-empty
    # run_report_path parameter -- read back below and merged into the flat
    # report dict evaluate_gate() checks. deadline_ms=250.0 is FUS-RT-002's
    # nominal result-age target; it does not vary per profile (only the
    # ACCEPTABLE deadline_miss_fraction does, and that bound lives in
    # run_report.py's per-profile GateSpec, not here).
    gateway_run_report_out = str(Path(out_dir) / f"gateway_run_report_seed{seed}.json")
    group = ProcessGroup()
    try:
        group.start_subprocess(
            [
                sys.executable,
                "-m",
                "uw_holoocean_adapter.realtime_ros_session",
                "--scenario",
                scenario_path,
                "--task",
                task_path,
                "--seed",
                str(seed),
                *session_fault_args(profile),
            ]
        )
        group.start_subprocess(
            [
                str(gateway_path),
                "--ros-args",
                "-p",
                f"run_report_path:={gateway_run_report_out}",
                "-p",
                "deadline_ms:=250.0",
            ]
        )
        group.start_process(_run_scripted_pilot_process, args=(scenario_path, task_path))
        group.start_process(_run_scorer_process, args=(scenario_path, task_path, seed, scorer_out))

        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            exited = group.poll_exited()
            if exited:
                raise RealtimeGateError(f"a required process exited before the run completed: {exited}")
            time.sleep(1.0)
    finally:
        group.stop()

    report: Dict[str, Any] = {
        "profile": profile.profile,
        "seed": seed,
        "task_id": Path(task_path).stem,
        "duration_s": duration_s,
    }
    # holoocean_realtime_node's RuntimeMetricsCollector report -- read back
    # whatever it last wrote and merge its fields directly into the flat
    # report dict (result/state age percentiles, rtf, queue stats, RSS/CPU
    # headroom, deadline misses, detection counts, recovery duration,
    # staleness marking). gpu_headroom_fraction_avg is never present in it
    # -- see runtime_metrics_collector.hpp's own doc comment for why that
    # omission is deliberate rather than a bug; evaluate_gate() correctly
    # GateFailures on it for nominal/disturbed profiles until real GPU
    # sampling exists (minimum/overload don't require it).
    gateway_report_path = Path(gateway_run_report_out)
    if gateway_report_path.is_file():
        try:
            report.update(json.loads(gateway_report_path.read_text(encoding="utf-8")))
        except json.JSONDecodeError:
            pass  # gateway process may have been killed mid-write; leave those fields unset

    # scorer_ros_bridge.run_scorer_bridge writes this file at least once a
    # second and once more on shutdown -- read back whatever it last wrote
    # rather than leaving `task_score` unset.
    scorer_path = Path(scorer_out)
    if scorer_path.is_file():
        try:
            report["task_score"] = json.loads(scorer_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            pass  # scorer process may have been killed mid-write; leave task_score unset
    return report


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--task", required=True)
    parser.add_argument("--scenario", default=_DEFAULT_SCENARIO)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--seeds", type=int, nargs="+", default=None)
    parser.add_argument("--soak-duration-s", type=float, default=None)
    parser.add_argument("--gateway-binary", default=_DEFAULT_GATEWAY_BINARY)
    parser.add_argument("--out-dir", default="/tmp")
    args = parser.parse_args(argv)

    if (args.seed is None) == (args.seeds is None):
        print("exactly one of --seed or --seeds is required", file=sys.stderr)
        return 2
    seeds = args.seeds if args.seeds is not None else [args.seed]

    try:
        profile = load_profile(args.profile)
    except RealtimeGateError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    gate = _gate_spec_for(profile, min_duration_s=args.soak_duration_s or 0.0)

    passed = 0
    for seed in seeds:
        try:
            report = run_gate(
                args.profile,
                args.task,
                seed,
                scenario_path=args.scenario,
                gateway_binary=args.gateway_binary,
                soak_duration_s=args.soak_duration_s,
                out_dir=args.out_dir,
            )
            evaluate_gate(report, gate)
        except (RealtimeGateError, GateFailure) as error:
            print(f"seed={seed}: FAILED ({error})", file=sys.stderr)
            continue
        print(f"seed={seed}: PASSED")
        passed += 1

    needed = required_passes(profile.gate, len(seeds))
    print(f"{passed}/{len(seeds)} seeds passed the {profile.gate} gate (required: {needed}/{len(seeds)})")
    return 0 if passed >= needed else 1


if __name__ == "__main__":
    raise SystemExit(main())
