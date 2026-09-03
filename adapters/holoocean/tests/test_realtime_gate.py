"""Covers the pure/portable pieces of realtime_gate.py -- profile loading,
gate-spec selection, and the SIM-ACC-003 seed-campaign pass fraction.
`run_gate()`/`main()` themselves launch real subprocesses (a HoloOcean
session, the C++ gateway, rclpy-based pilot/scorer bridges) and are not
exercised here -- same "needs a real native host" status as every other
process-launching piece in this package (see
docs/rov-realtime-closed-loop-code-review-2026-08-27.md finding A2's "still
open" note for what remains gateway-side telemetry work)."""
import time

import pytest
import yaml

from uw_holoocean_adapter.realtime_gate import (
    ProcessGroup,
    RealtimeGateError,
    _gate_spec_for,
    load_profile,
    required_passes,
)
from uw_holoocean_adapter.run_report import disturbed_gate, minimum_gate, nominal_gate, overload_gate


def _immediately_exits():
    """Pickleable process target for spawn/forkserver-based Python runtimes."""


def _write_profile(tmp_path, **overrides):
    data = {
        "profile": "nominal",
        "sonar_hz": 10.0,
        "state_hz": 50.0,
        "camera": {"hz": 20.0},
    }
    data.update(overrides)
    path = tmp_path / "profile.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return str(path)


def test_load_profile_missing_file_raises_gate_error(tmp_path):
    with pytest.raises(RealtimeGateError, match="profile file not found"):
        load_profile(str(tmp_path / "nope.yaml"))


def test_load_profile_reads_required_and_defaulted_fields(tmp_path):
    path = _write_profile(tmp_path, duration_s=1800.0, faults_enabled=True)
    profile = load_profile(path)
    assert profile.profile == "nominal"
    assert profile.duration_s == 1800.0
    assert profile.camera_hz == pytest.approx(20.0)
    assert profile.sonar_hz == pytest.approx(10.0)
    assert profile.state_hz == pytest.approx(50.0)
    assert profile.faults_enabled is True
    assert profile.gate == "nominal"  # defaults to profile name when `gate` is omitted


def test_load_profile_gate_can_diverge_from_profile_name(tmp_path):
    path = _write_profile(tmp_path, profile="nominal_campaign", gate="disturbed")
    profile = load_profile(path)
    assert profile.gate == "disturbed"


@pytest.mark.parametrize(
    "gate_name,expected_gate_fn",
    [("minimum", minimum_gate), ("overload", overload_gate), ("disturbed", disturbed_gate)],
)
def test_gate_spec_for_maps_known_gate_names(tmp_path, gate_name, expected_gate_fn):
    path = _write_profile(tmp_path, gate=gate_name)
    profile = load_profile(path)
    spec = _gate_spec_for(profile, min_duration_s=0.0)
    assert spec == expected_gate_fn()


def test_gate_spec_for_defaults_to_nominal_with_min_duration(tmp_path):
    path = _write_profile(tmp_path, gate="nominal")
    profile = load_profile(path)
    spec = _gate_spec_for(profile, min_duration_s=7200.0)
    assert spec == nominal_gate(min_duration_s=7200.0)


def test_required_passes_nominal_campaign_is_eight_of_ten():
    assert required_passes("nominal", 10) == 8


def test_required_passes_disturbed_campaign_is_seven_of_ten():
    assert required_passes("disturbed", 10) == 7


def test_required_passes_minimum_and_overload_require_every_seed():
    assert required_passes("minimum", 1) == 1
    assert required_passes("overload", 1) == 1


def test_required_passes_rounds_up_for_non_ten_seed_campaigns():
    # 3 seeds at 80% -> 2.4, must round up to 3 (can't require a fractional pass).
    assert required_passes("nominal", 3) == 3


def test_process_group_start_process_reports_exit_and_stop_reaps_it():
    group = ProcessGroup()

    group.start_process(_immediately_exits)
    deadline = time.monotonic() + 2.0
    exited = []
    while time.monotonic() < deadline:
        exited = group.poll_exited()
        if exited:
            break
        time.sleep(0.02)
    assert exited, "expected the started process to have exited within 2s"
    group.stop()  # must not raise even though the process already exited


# ---- PREP-E-02: fault_profile / bandwidth forwarding ---------------------------

import pathlib  # noqa: E402

from uw_holoocean_adapter.realtime_gate import BandwidthGateConfig, session_fault_args  # noqa: E402
from uw_holoocean_adapter.run_report import tether_limited_gate  # noqa: E402

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_TETHER_LIMITED_YAML = _REPO_ROOT / "configs/experiment/rov_realtime_tether_limited.yaml"


def test_profiles_without_fault_keys_forward_no_extra_session_argv(tmp_path):
    # The pre-PREP-E-02 argv must stay byte-identical for nominal/minimum runs.
    profile = load_profile(_write_profile(tmp_path))
    assert profile.fault_profile == "none"
    assert profile.bandwidth is None
    assert session_fault_args(profile) == []


def test_critical_fault_profile_is_forwarded_without_bandwidth_flags(tmp_path):
    profile = load_profile(_write_profile(tmp_path, fault_profile="critical", fault_seed=3))
    assert session_fault_args(profile) == ["--fault-profile", "critical", "--fault-seed", "3"]


def test_load_profile_rejects_unknown_fault_profile(tmp_path):
    with pytest.raises(RealtimeGateError, match="fault_profile"):
        load_profile(_write_profile(tmp_path, fault_profile="chaos"))


def test_tether_limited_experiment_forwards_bandwidth_flags_and_gate():
    profile = load_profile(str(_TETHER_LIMITED_YAML))
    assert profile.fault_profile == "bandwidth"
    assert profile.gate == "tether_limited"
    assert profile.bandwidth == BandwidthGateConfig(
        nominal_mbps=20.0, min_mbps=10.0, max_mbps=40.0, walk_sigma_mbps_per_s=2.0
    )
    assert session_fault_args(profile) == [
        "--fault-profile", "bandwidth",
        "--fault-seed", "42",
        "--bandwidth-mbps", "20.0",
        "--bandwidth-min-mbps", "10.0",
        "--bandwidth-max-mbps", "40.0",
        "--bandwidth-walk-sigma", "2.0",
    ]
    assert _gate_spec_for(profile, min_duration_s=0.0) == tether_limited_gate()


def test_bandwidth_profile_without_block_uses_contract_tether_defaults(tmp_path):
    profile = load_profile(_write_profile(tmp_path, fault_profile="critical+bandwidth"))
    args = session_fault_args(profile)
    assert args[:2] == ["--fault-profile", "critical+bandwidth"]
    assert args[args.index("--bandwidth-mbps") + 1] == "20.0"
    assert args[args.index("--bandwidth-min-mbps") + 1] == "10.0"
    assert args[args.index("--bandwidth-max-mbps") + 1] == "40.0"
    assert args[args.index("--bandwidth-walk-sigma") + 1] == "0.0"
