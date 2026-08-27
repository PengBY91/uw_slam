import pytest

from uw_holoocean_adapter.run_report import (
    GateFailure,
    disturbed_gate,
    evaluate_gate,
    minimum_gate,
    nominal_gate,
    overload_gate,
)


def _base_report(**overrides):
    report = {
        "result_age_p95_ms": 200.0,
        "state_age_p95_ms": 80.0,
        "rtf_p95": 1.05,
        "deadline_miss_fraction": 0.0,
        "queue_high_watermark": 3,
        "queue_capacity_violations": 0,
        "recovery_duration_s_max": 1.0,
        "sonar_detection_count": 5,
        "visual_detection_count": 5,
        "fused_track_count": 2,
        "duration_s": 100.0,
        "rss_growth_after_warmup_mib": 10.0,
        "cpu_headroom_fraction_avg": 0.5,
        "gpu_headroom_fraction_avg": 0.5,
        "guidance_marked_stale_when_overdue": True,
    }
    report.update(overrides)
    return report


def valid_nominal_report():
    return _base_report(duration_s=7200.0)


def valid_minimum_report():
    return _base_report(result_age_p95_ms=340.0, state_age_p95_ms=140.0, duration_s=1800.0)


def valid_overload_report():
    return _base_report(
        result_age_p95_ms=490.0, state_age_p95_ms=190.0, deadline_miss_fraction=0.04, duration_s=1800.0
    )


def test_incomplete_report_raises_gate_failure_not_key_error():
    # docs/rov-realtime-closed-loop-code-review-2026-08-27.md finding A2:
    # run_gate() used to return a report missing ~20 required fields, and
    # evaluate_gate's first plain report[...] lookup crashed with an
    # uncaught KeyError instead of a named, catchable GateFailure.
    incomplete = {"profile": "nominal", "seed": 1, "task_id": "t", "duration_s": 1.0}
    with pytest.raises(GateFailure, match="result_age_p95_ms"):
        evaluate_gate(incomplete, nominal_gate())


def test_nominal_gate_rejects_old_results_and_memory_growth():
    report = valid_nominal_report()
    report["result_age_p95_ms"] = 251.0
    with pytest.raises(GateFailure, match="result_age_p95_ms"):
        evaluate_gate(report, nominal_gate(min_duration_s=7200.0))
    report = valid_nominal_report()
    report["rss_growth_after_warmup_mib"] = 257.0
    with pytest.raises(GateFailure, match="rss_growth"):
        evaluate_gate(report, nominal_gate(min_duration_s=7200.0))


def test_nominal_gate_accepts_a_fully_valid_report():
    evaluate_gate(valid_nominal_report(), nominal_gate(min_duration_s=7200.0))


def test_nominal_gate_rejects_rtf_below_one():
    report = valid_nominal_report()
    report["rtf_p95"] = 0.99
    with pytest.raises(GateFailure, match="rtf_p95"):
        evaluate_gate(report, nominal_gate())


def test_nominal_gate_rejects_deadline_miss_fraction_above_one_percent():
    report = valid_nominal_report()
    report["deadline_miss_fraction"] = 0.011
    with pytest.raises(GateFailure, match="deadline_miss_fraction"):
        evaluate_gate(report, nominal_gate())


def test_nominal_gate_rejects_capacity_violations():
    report = valid_nominal_report()
    report["queue_capacity_violations"] = 1
    with pytest.raises(GateFailure, match="queue_capacity_violations"):
        evaluate_gate(report, nominal_gate())


def test_nominal_gate_rejects_recovery_over_two_seconds():
    report = valid_nominal_report()
    report["recovery_duration_s_max"] = 2.1
    with pytest.raises(GateFailure, match="recovery_duration_s_max"):
        evaluate_gate(report, nominal_gate())


def test_nominal_gate_rejects_zero_tracks():
    for field in ("sonar_detection_count", "visual_detection_count", "fused_track_count"):
        report = valid_nominal_report()
        report[field] = 0
        with pytest.raises(GateFailure, match=field):
            evaluate_gate(report, nominal_gate())


def test_nominal_gate_rejects_short_soak_duration():
    report = valid_nominal_report()
    report["duration_s"] = 7199.0
    with pytest.raises(GateFailure, match="duration_s"):
        evaluate_gate(report, nominal_gate(min_duration_s=7200.0))


def test_nominal_gate_rejects_insufficient_cpu_gpu_headroom():
    report = valid_nominal_report()
    report["cpu_headroom_fraction_avg"] = 0.19
    with pytest.raises(GateFailure, match="cpu_headroom_fraction_avg"):
        evaluate_gate(report, nominal_gate())
    report = valid_nominal_report()
    report["gpu_headroom_fraction_avg"] = 0.19
    with pytest.raises(GateFailure, match="gpu_headroom_fraction_avg"):
        evaluate_gate(report, nominal_gate())


def test_nominal_gate_requires_stale_guidance_to_be_marked():
    report = valid_nominal_report()
    report["guidance_marked_stale_when_overdue"] = False
    with pytest.raises(GateFailure, match="guidance_marked_stale_when_overdue"):
        evaluate_gate(report, nominal_gate())


def test_minimum_gate_boundary_350_150_ms():
    report = valid_minimum_report()
    report["result_age_p95_ms"] = 350.0
    report["state_age_p95_ms"] = 150.0
    evaluate_gate(report, minimum_gate())

    over_result = valid_minimum_report()
    over_result["result_age_p95_ms"] = 350.1
    with pytest.raises(GateFailure, match="result_age_p95_ms"):
        evaluate_gate(over_result, minimum_gate())

    over_state = valid_minimum_report()
    over_state["state_age_p95_ms"] = 150.1
    with pytest.raises(GateFailure, match="state_age_p95_ms"):
        evaluate_gate(over_state, minimum_gate())


def test_minimum_gate_does_not_require_cpu_gpu_headroom():
    report = valid_minimum_report()
    report["cpu_headroom_fraction_avg"] = 0.0
    report["gpu_headroom_fraction_avg"] = 0.0
    evaluate_gate(report, minimum_gate())


def test_nominal_gate_boundary_250_100_ms():
    report = valid_nominal_report()
    report["result_age_p95_ms"] = 250.0
    report["state_age_p95_ms"] = 100.0
    evaluate_gate(report, nominal_gate())

    over_result = valid_nominal_report()
    over_result["result_age_p95_ms"] = 250.1
    with pytest.raises(GateFailure, match="result_age_p95_ms"):
        evaluate_gate(over_result, nominal_gate())

    over_state = valid_nominal_report()
    over_state["state_age_p95_ms"] = 100.1
    with pytest.raises(GateFailure, match="state_age_p95_ms"):
        evaluate_gate(over_state, nominal_gate())


def test_overload_gate_boundary_500_200_ms_hard_limits():
    report = valid_overload_report()
    report["result_age_p95_ms"] = 500.0
    report["state_age_p95_ms"] = 200.0
    evaluate_gate(report, overload_gate())

    over_result = valid_overload_report()
    over_result["result_age_p95_ms"] = 500.1
    with pytest.raises(GateFailure, match="result_age_p95_ms"):
        evaluate_gate(over_result, overload_gate())

    over_state = valid_overload_report()
    over_state["state_age_p95_ms"] = 200.1
    with pytest.raises(GateFailure, match="state_age_p95_ms"):
        evaluate_gate(over_state, overload_gate())


def test_overload_gate_boundary_five_percent_deadline_misses():
    report = valid_overload_report()
    report["deadline_miss_fraction"] = 0.05
    evaluate_gate(report, overload_gate())

    over = valid_overload_report()
    over["deadline_miss_fraction"] = 0.0501
    with pytest.raises(GateFailure, match="deadline_miss_fraction"):
        evaluate_gate(over, overload_gate())


def test_overload_gate_does_not_require_cpu_gpu_headroom():
    report = valid_overload_report()
    report["cpu_headroom_fraction_avg"] = 0.0
    report["gpu_headroom_fraction_avg"] = 0.0
    evaluate_gate(report, overload_gate())


def test_disturbed_gate_reuses_nominal_latency_budget():
    report = valid_nominal_report()
    evaluate_gate(report, disturbed_gate())
    report["result_age_p95_ms"] = 250.1
    with pytest.raises(GateFailure, match="result_age_p95_ms"):
        evaluate_gate(report, disturbed_gate())


def test_rss_growth_boundary_256_mib():
    report = valid_nominal_report()
    report["rss_growth_after_warmup_mib"] = 256.0
    evaluate_gate(report, nominal_gate(min_duration_s=7200.0))
    report["rss_growth_after_warmup_mib"] = 256.1
    with pytest.raises(GateFailure, match="rss_growth"):
        evaluate_gate(report, nominal_gate(min_duration_s=7200.0))


def test_task_score_must_carry_task_success_when_present():
    report = valid_nominal_report()
    report["task_score"] = {"task_success": True}
    evaluate_gate(report, nominal_gate())
    report["task_score"] = {"completion_time_s": 12.0}
    with pytest.raises(GateFailure, match="task_score"):
        evaluate_gate(report, nominal_gate())
