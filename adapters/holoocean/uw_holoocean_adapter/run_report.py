"""The realtime closed-loop run report: everything a minimum/nominal/
disturbed/overload gate needs to accept or reject one `realtime_gate.py`
run, plus the gate evaluation itself.

`RunReport` is the JSON-serializable shape `realtime_gate.py` writes at the
end of a run (mirroring `task_scorer.TaskScoreReport`'s established
dataclass-with-`as_dict()` pattern). `evaluate_gate(report, gate_spec)`
deliberately takes a plain `dict` (not a `RunReport` instance) -- the same
report a caller loads back from a JSON file on disk, or a `RunReport`'s own
`as_dict()` output, satisfies it identically. `GateFailure` names exactly
which field violated which bound, so a failing native-host run reports the
actual failed gate rather than a bare "gate failed".
"""
from __future__ import annotations

import dataclasses
from typing import Any, Dict, List, Optional

# Fusion/target-result data age budgets (ms), per profile -- the plan's own
# numbers, not derived from anything else in this repo. State (vehicle
# state) age budgets are the plan's second, tighter number in the same
# "350/150 ms" / "250/100 ms" / "500/200 ms" triples.
_RESULT_AGE_P95_MS = {"minimum": 350.0, "nominal": 250.0, "overload": 500.0}
_STATE_AGE_P95_MS = {"minimum": 150.0, "nominal": 100.0, "overload": 200.0}
_DEADLINE_MISS_FRACTION = {"minimum": 0.01, "nominal": 0.01, "overload": 0.05}
_RTF_P95_MIN = 1.0
_RECOVERY_DURATION_S_MAX = 2.0
_RSS_GROWTH_AFTER_WARMUP_MIB_MAX = 256.0
_CPU_GPU_HEADROOM_MIN_FRACTION = 0.20
# ScriptedPilot's own 500ms staleness bound (scripted_pilot.py's
# _MAX_STALENESS_S) -- the gate cross-checks the report against the SAME
# number rather than a second, independently-invented one.
_STALE_GUIDANCE_AFTER_S = 0.5


@dataclasses.dataclass
class RunReport:
    """One realtime_gate.py run's complete evidence. Every field here is
    meant to be JSON-serializable as-is (`dataclasses.asdict`)."""

    profile: str  # "minimum" | "nominal" | "disturbed" | "overload"
    seed: int
    task_id: str

    code_hash: str
    scenario_hash: str
    task_hash: str
    config_hash: str
    calibration_hash: str

    host: str
    holoocean_version: str
    unreal_version: str

    sensor_rates_hz: Dict[str, float]

    rtf_p50: float
    rtf_p95: float

    result_age_p50_ms: float
    result_age_p95_ms: float
    result_age_p99_ms: float
    state_age_p50_ms: float
    state_age_p95_ms: float
    state_age_p99_ms: float

    deadline_miss_fraction: float
    queue_high_watermark: int
    queue_drops: int
    queue_rejects: int

    duration_s: float
    recovery_duration_s_max: float
    rss_growth_after_warmup_mib: float
    cpu_headroom_fraction_avg: float
    gpu_headroom_fraction_avg: float

    sonar_detection_count: int
    visual_detection_count: int
    fused_track_count: int

    guidance_marked_stale_when_overdue: bool

    health_timeline: List[Dict[str, Any]] = dataclasses.field(default_factory=list)
    fault_timeline: List[Dict[str, Any]] = dataclasses.field(default_factory=list)

    task_score: Optional[Dict[str, Any]] = None

    def as_dict(self) -> Dict[str, Any]:
        return dataclasses.asdict(self)


class GateFailure(Exception):
    """Raised by `evaluate_gate` naming the specific field/bound violated --
    the message always contains the offending field's name so a caller can
    `pytest.raises(GateFailure, match="<field>")` or grep a CI log for it."""


@dataclasses.dataclass(frozen=True)
class GateSpec:
    profile: str
    result_age_p95_ms_max: float
    state_age_p95_ms_max: float
    deadline_miss_fraction_max: float
    rtf_p95_min: float = _RTF_P95_MIN
    recovery_duration_s_max: float = _RECOVERY_DURATION_S_MAX
    rss_growth_after_warmup_mib_max: float = _RSS_GROWTH_AFTER_WARMUP_MIB_MAX
    cpu_gpu_headroom_min_fraction: float = _CPU_GPU_HEADROOM_MIN_FRACTION
    min_duration_s: float = 0.0
    require_headroom_gate: bool = True


def minimum_gate() -> GateSpec:
    return GateSpec(
        profile="minimum",
        result_age_p95_ms_max=_RESULT_AGE_P95_MS["minimum"],
        state_age_p95_ms_max=_STATE_AGE_P95_MS["minimum"],
        deadline_miss_fraction_max=_DEADLINE_MISS_FRACTION["minimum"],
        min_duration_s=1800.0,
        require_headroom_gate=False,
    )


def nominal_gate(min_duration_s: float = 0.0) -> GateSpec:
    """`min_duration_s` defaults to 0 (per-seed task-scoring runs, which are
    much shorter than the run duration budget); pass 7200.0 for the separate
    2-hour nominal soak run's gate."""
    return GateSpec(
        profile="nominal",
        result_age_p95_ms_max=_RESULT_AGE_P95_MS["nominal"],
        state_age_p95_ms_max=_STATE_AGE_P95_MS["nominal"],
        deadline_miss_fraction_max=_DEADLINE_MISS_FRACTION["nominal"],
        min_duration_s=min_duration_s,
    )


def disturbed_gate() -> GateSpec:
    """Disturbed keeps nominal rates and nominal latency/deadline budgets
    (the plan's Task 6 prose: "disturbed profile keeps nominal rates") --
    the perturbation matrix affects task success rate (scored separately by
    `task_scorer.py` across a 10-seed campaign), not this report's realtime
    budgets."""
    return GateSpec(
        profile="disturbed",
        result_age_p95_ms_max=_RESULT_AGE_P95_MS["nominal"],
        state_age_p95_ms_max=_STATE_AGE_P95_MS["nominal"],
        deadline_miss_fraction_max=_DEADLINE_MISS_FRACTION["nominal"],
    )


def overload_gate() -> GateSpec:
    return GateSpec(
        profile="overload",
        result_age_p95_ms_max=_RESULT_AGE_P95_MS["overload"],
        state_age_p95_ms_max=_STATE_AGE_P95_MS["overload"],
        deadline_miss_fraction_max=_DEADLINE_MISS_FRACTION["overload"],
        min_duration_s=1800.0,
        require_headroom_gate=False,
    )


def _require(condition: bool, field: str, message: str) -> None:
    if not condition:
        raise GateFailure(f"{field}: {message}")


def evaluate_gate(report: Dict[str, Any], gate: GateSpec) -> None:
    """Raises `GateFailure` on the first violated bound; returns None (does
    not return a report/summary object) when every bound is satisfied --
    same "raises on failure, silent on success" contract as the plan's own
    shown example test."""
    _require(
        report["result_age_p95_ms"] <= gate.result_age_p95_ms_max,
        "result_age_p95_ms",
        f"{report['result_age_p95_ms']} exceeds the {gate.profile} bound of {gate.result_age_p95_ms_max} ms",
    )
    _require(
        report["state_age_p95_ms"] <= gate.state_age_p95_ms_max,
        "state_age_p95_ms",
        f"{report['state_age_p95_ms']} exceeds the {gate.profile} bound of {gate.state_age_p95_ms_max} ms",
    )
    _require(
        report["rtf_p95"] >= gate.rtf_p95_min,
        "rtf_p95",
        f"{report['rtf_p95']} is below the required {gate.rtf_p95_min}",
    )
    _require(
        report["deadline_miss_fraction"] <= gate.deadline_miss_fraction_max,
        "deadline_miss_fraction",
        f"{report['deadline_miss_fraction']} exceeds the {gate.profile} bound of {gate.deadline_miss_fraction_max}",
    )
    _require(
        report["queue_high_watermark"] >= 0 and report.get("queue_capacity_violations", 0) == 0,
        "queue_capacity_violations",
        f"{report.get('queue_capacity_violations', 0)} capacity violations observed, must be 0",
    )
    _require(
        report["recovery_duration_s_max"] <= gate.recovery_duration_s_max,
        "recovery_duration_s_max",
        f"{report['recovery_duration_s_max']}s exceeds the {gate.recovery_duration_s_max}s bound",
    )
    _require(report["sonar_detection_count"] > 0, "sonar_detection_count", "must be nonzero")
    _require(report["visual_detection_count"] > 0, "visual_detection_count", "must be nonzero")
    _require(report["fused_track_count"] > 0, "fused_track_count", "must be nonzero")
    _require(
        report["duration_s"] >= gate.min_duration_s,
        "duration_s",
        f"{report['duration_s']}s is shorter than the required {gate.min_duration_s}s",
    )
    _require(
        report["rss_growth_after_warmup_mib"] <= gate.rss_growth_after_warmup_mib_max,
        "rss_growth_after_warmup_mib",
        f"{report['rss_growth_after_warmup_mib']} MiB exceeds the {gate.rss_growth_after_warmup_mib_max} MiB bound",
    )
    if gate.require_headroom_gate:
        _require(
            report["cpu_headroom_fraction_avg"] >= gate.cpu_gpu_headroom_min_fraction,
            "cpu_headroom_fraction_avg",
            f"{report['cpu_headroom_fraction_avg']} is below the required {gate.cpu_gpu_headroom_min_fraction}",
        )
        _require(
            report["gpu_headroom_fraction_avg"] >= gate.cpu_gpu_headroom_min_fraction,
            "gpu_headroom_fraction_avg",
            f"{report['gpu_headroom_fraction_avg']} is below the required {gate.cpu_gpu_headroom_min_fraction}",
        )
    _require(
        report["guidance_marked_stale_when_overdue"] is True,
        "guidance_marked_stale_when_overdue",
        f"guidance older than {_STALE_GUIDANCE_AFTER_S * 1000:.0f}ms must be marked stale, was not observed to be",
    )
    task_score = report.get("task_score")
    if task_score is not None:
        _require(
            "task_success" in task_score,
            "task_score",
            "task_score must carry task_success when present",
        )
