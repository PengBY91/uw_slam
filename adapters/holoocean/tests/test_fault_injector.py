from itertools import pairwise

import numpy as np
import pytest

from uw_holoocean_adapter.fault_injector import (
    FaultInjectionProfile,
    FaultInjector,
    ScheduledFault,
    SensorDegradationSchedule,
    SensorDegradationWindow,
    ThrusterFaultConfig,
    TopicFaultConfig,
    apply_thruster_fault,
    build_fault_schedule,
    build_sensor_degradation_schedule,
    resolve_active_degradation,
    sensor_degradation_active,
)
from uw_holoocean_adapter.scenario_randomization import PRESET_CRITICAL_DEGRADED

_TOPICS = ("/holoocean/auv0/LeftCamera", "/holoocean/auv0/RightCamera", "/holoocean/auv0/ImagingSonar")


def critical_profile() -> FaultInjectionProfile:
    return FaultInjectionProfile(
        per_topic={
            topic: TopicFaultConfig(
                clock_offset_s=0.01,
                jitter_sigma_s=0.005,
                drop_probability=0.1,
                duplicate_probability=0.05,
                reorder_probability=0.08,
                reorder_max_distance=2,
                outage_count=1,
                outage_duration_s=1.5,
            )
            for topic in _TOPICS
        },
        check_interval_s=0.5,
    )


def test_fault_schedule_is_repeatable_and_never_reorders_without_request():
    schedule_a = build_fault_schedule(seed=42, profile=critical_profile(), duration_s=30)
    schedule_b = build_fault_schedule(seed=42, profile=critical_profile(), duration_s=30)

    assert schedule_a == schedule_b
    assert all(a.release_time_s <= b.release_time_s for a, b in pairwise(schedule_a))


def test_different_seed_yields_a_different_schedule():
    schedule_a = build_fault_schedule(seed=42, profile=critical_profile(), duration_s=30)
    schedule_b = build_fault_schedule(seed=43, profile=critical_profile(), duration_s=30)

    assert schedule_a != schedule_b


def test_no_faults_configured_yields_an_empty_schedule():
    schedule = build_fault_schedule(seed=1, profile=FaultInjectionProfile(), duration_s=30)
    assert schedule == ()


def test_drop_removes_the_next_message_on_that_topic():
    schedule = (ScheduledFault(release_time_s=1.0, sort_index=0, topic="t", kind="drop"),)
    injector = FaultInjector(schedule, FaultInjectionProfile(), np.random.default_rng(1))

    delivered_before = injector.apply(0.5, [("t", "m0")])
    delivered_after = injector.apply(1.0, [("t", "m1")])
    perturbations_after_drop = list(injector.last_active_perturbations)
    delivered_next = injector.apply(1.1, [("t", "m2")])

    assert delivered_before == [("t", "m0")]
    assert delivered_after == []
    assert perturbations_after_drop == ["drop:t"]
    assert delivered_next == [("t", "m2")]


def test_duplicate_emits_the_message_twice():
    schedule = (ScheduledFault(release_time_s=1.0, sort_index=0, topic="t", kind="duplicate"),)
    injector = FaultInjector(schedule, FaultInjectionProfile(), np.random.default_rng(1))

    delivered = injector.apply(1.0, [("t", "m")])

    assert delivered == [("t", "m"), ("t", "m")]


def test_reorder_delays_the_tagged_message_by_at_most_its_bounded_distance():
    # m0 is tagged for reorder with distance 2: it must not be delivered
    # before 2 subsequent messages on "t" have been processed, but must be
    # delivered no later than that -- a "bounded" reorder, not an unbounded
    # hold.
    schedule = (
        ScheduledFault(release_time_s=1.0, sort_index=0, topic="t", kind="reorder", reorder_distance=2),
    )
    injector = FaultInjector(schedule, FaultInjectionProfile(), np.random.default_rng(1))

    first = injector.apply(1.0, [("t", "m0")])
    second = injector.apply(1.1, [("t", "m1")])
    third = injector.apply(1.2, [("t", "m2")])

    assert first == []
    assert second == [("t", "m1")]
    assert third == [("t", "m0"), ("t", "m2")]
    assert injector.flush() == []


def test_outage_drops_every_message_on_that_topic_for_its_duration():
    schedule = (
        ScheduledFault(release_time_s=1.0, sort_index=0, topic="t", kind="outage", duration_s=2.0),
    )
    injector = FaultInjector(schedule, FaultInjectionProfile(), np.random.default_rng(1))

    during = injector.apply(2.0, [("t", "m")])
    after = injector.apply(3.1, [("t", "m2")])

    assert during == []
    assert after == [("t", "m2")]


def test_clock_offset_and_jitter_shift_the_effective_log_time():
    profile = FaultInjectionProfile(per_topic={"t": TopicFaultConfig(clock_offset_s=0.1)})
    injector = FaultInjector((), profile, np.random.default_rng(1))

    shifted = injector.effective_log_time_ns("t", 1_000_000_000)

    assert shifted == 1_100_000_000


def test_apply_thruster_fault_scales_only_the_targeted_channel():
    command = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
    faulted = apply_thruster_fault(command, ThrusterFaultConfig(thruster_index=4, effectiveness_multiplier=0.0))

    assert faulted[4] == 0.0
    assert faulted[:4] == command[:4]
    assert faulted[5:] == command[5:]


def test_apply_thruster_fault_is_a_no_op_when_no_fault_configured():
    command = [1.0, 2.0, 3.0]
    assert apply_thruster_fault(command, None) == command


# docs/archive/rov-realtime-closed-loop-code-review-2026-08-27.md finding B4: visual/
# sonar degradation used to be a static per-run parameter, not a runtime-
# schedulable fault with start/duration/recovery like the timing/outage
# faults above already were.


def test_sensor_degradation_schedule_is_repeatable():
    schedule_a = build_sensor_degradation_schedule(
        seed=7, duration_s=120.0, visual_window_count=2, visual_window_duration_s=10.0,
        sonar_window_count=1, sonar_window_duration_s=20.0,
    )
    schedule_b = build_sensor_degradation_schedule(
        seed=7, duration_s=120.0, visual_window_count=2, visual_window_duration_s=10.0,
        sonar_window_count=1, sonar_window_duration_s=20.0,
    )
    assert schedule_a == schedule_b


def test_sensor_degradation_schedule_default_window_counts_yield_no_windows():
    schedule = build_sensor_degradation_schedule(seed=1, duration_s=60.0)
    assert schedule == SensorDegradationSchedule()


def test_sensor_degradation_windows_stay_within_the_run_duration():
    schedule = build_sensor_degradation_schedule(
        seed=3, duration_s=50.0, visual_window_count=5, visual_window_duration_s=8.0,
        sonar_window_count=5, sonar_window_duration_s=8.0,
    )
    for window in schedule.visual_windows + schedule.sonar_windows:
        assert window.start_s >= 0.0
        assert window.start_s + window.duration_s <= 50.0 + 1e-9


def test_sensor_degradation_active_checks_half_open_interval():
    windows = (SensorDegradationWindow(start_s=10.0, duration_s=5.0),)
    assert not sensor_degradation_active(windows, 9.999)
    assert sensor_degradation_active(windows, 10.0)
    assert sensor_degradation_active(windows, 14.999)
    assert not sensor_degradation_active(windows, 15.0)


def test_resolve_active_degradation_with_no_schedule_always_returns_baseline():
    visual, sonar = resolve_active_degradation(
        None, PRESET_CRITICAL_DEGRADED.visual, PRESET_CRITICAL_DEGRADED.sonar, sim_time_s=42.0,
    )
    assert visual is None
    assert sonar is None


def test_resolve_active_degradation_applies_profile_only_inside_its_own_window():
    schedule = SensorDegradationSchedule(
        visual_windows=(SensorDegradationWindow(start_s=10.0, duration_s=5.0),),
        sonar_windows=(SensorDegradationWindow(start_s=30.0, duration_s=5.0),),
    )

    before = resolve_active_degradation(
        schedule, PRESET_CRITICAL_DEGRADED.visual, PRESET_CRITICAL_DEGRADED.sonar, sim_time_s=5.0,
    )
    assert before == (None, None)

    during_visual_only = resolve_active_degradation(
        schedule, PRESET_CRITICAL_DEGRADED.visual, PRESET_CRITICAL_DEGRADED.sonar, sim_time_s=12.0,
    )
    assert during_visual_only == (PRESET_CRITICAL_DEGRADED.visual, None)

    during_sonar_only = resolve_active_degradation(
        schedule, PRESET_CRITICAL_DEGRADED.visual, PRESET_CRITICAL_DEGRADED.sonar, sim_time_s=32.0,
    )
    assert during_sonar_only == (None, PRESET_CRITICAL_DEGRADED.sonar)

    after = resolve_active_degradation(
        schedule, PRESET_CRITICAL_DEGRADED.visual, PRESET_CRITICAL_DEGRADED.sonar, sim_time_s=100.0,
    )
    assert after == (None, None)


def test_resolve_active_degradation_recovers_after_window_ends():
    # The whole point of a scheduled window over a permanently-on flag:
    # degradation must actually turn back off.
    schedule = SensorDegradationSchedule(
        visual_windows=(SensorDegradationWindow(start_s=10.0, duration_s=5.0),),
    )
    inside = resolve_active_degradation(schedule, PRESET_CRITICAL_DEGRADED.visual, None, sim_time_s=12.0)
    recovered = resolve_active_degradation(schedule, PRESET_CRITICAL_DEGRADED.visual, None, sim_time_s=16.0)
    assert inside[0] is PRESET_CRITICAL_DEGRADED.visual
    assert recovered[0] is None


def test_build_sensor_degradation_schedule_rejects_negative_duration():
    with pytest.raises(ValueError):
        build_sensor_degradation_schedule(seed=1, duration_s=-1.0)


def test_build_sensor_degradation_schedule_rejects_non_positive_window_duration():
    with pytest.raises(ValueError):
        build_sensor_degradation_schedule(seed=1, duration_s=60.0, visual_window_count=1,
                                          visual_window_duration_s=0.0)


# ---- PREP-E-02: bandwidth / latency profile -----------------------------------

from uw_holoocean_adapter.fault_injector import (  # noqa: E402
    BandwidthProfile,
    BandwidthSample,
    BandwidthShaper,
    build_bandwidth_schedule,
)

_SONAR = "/holoocean/auv0/ImagingSonar"
_STATE = "/holoocean/auv0/VehicleState"
_LEFT = "/holoocean/auv0/LeftCamera"
_RIGHT = "/holoocean/auv0/RightCamera"


def tether_profile(**overrides) -> BandwidthProfile:
    base = dict(
        nominal_mbps=8.0,  # 1 MB/s: keeps the arithmetic below exact
        min_mbps=4.0,
        max_mbps=16.0,
        walk_sigma_mbps_per_s=0.0,
        topic_priority={_SONAR: 0, _STATE: 1, _LEFT: 3, _RIGHT: 3},
        topic_bytes={_SONAR: 400_000, _STATE: 1_000, _LEFT: 300_000, _RIGHT: 300_000},
        max_queue_s=1.0,
        bucket_depth_s=0.1,  # 100 KB burst at 1 MB/s
    )
    base.update(overrides)
    return BandwidthProfile(**base)


def test_bandwidth_schedule_is_repeatable_for_a_seed_and_clamped_to_the_link_range():
    profile = tether_profile(walk_sigma_mbps_per_s=6.0, walk_interval_s=0.5)
    a = build_bandwidth_schedule(7, profile, 120.0)
    b = build_bandwidth_schedule(7, profile, 120.0)
    assert a == b
    assert a[0] == BandwidthSample(0.0, 8.0)
    assert len(a) == 240  # one sample per walk_interval_s inside [0, duration)
    assert all(profile.min_mbps <= s.mbps <= profile.max_mbps for s in a)
    assert all(later.time_s > earlier.time_s for earlier, later in pairwise(a))
    # A large sigma against a narrow [4, 16] band must actually hit both clamps,
    # otherwise the test would not be proving the clamp exists.
    assert min(s.mbps for s in a) == pytest.approx(profile.min_mbps)
    assert max(s.mbps for s in a) == pytest.approx(profile.max_mbps)
    assert build_bandwidth_schedule(8, profile, 120.0) != a


def test_bandwidth_schedule_without_walk_is_a_single_constant_sample():
    assert build_bandwidth_schedule(1, tether_profile(), 600.0) == (BandwidthSample(0.0, 8.0),)


def test_bandwidth_schedule_rejects_inconsistent_link_range():
    with pytest.raises(ValueError, match="min_mbps <= nominal_mbps <= max_mbps"):
        build_bandwidth_schedule(1, tether_profile(nominal_mbps=2.0), 10.0)
    with pytest.raises(ValueError, match="walk_interval_s"):
        build_bandwidth_schedule(1, tether_profile(walk_interval_s=0.0), 10.0)


def test_offered_rate_below_the_link_budget_passes_every_message_untouched():
    profile = tether_profile()
    shaper = BandwidthShaper(build_bandwidth_schedule(1, profile, 10.0), profile)
    # 1 kB state at 50 Hz = 50 kB/s against a 1 MB/s link.
    for tick in range(50):
        now_s = tick * 0.02
        out = shaper.apply(now_s, [(_STATE, f"s{tick}"), ("/clock", f"c{tick}")])
        assert out == [("/clock", f"c{tick}"), (_STATE, f"s{tick}")]
        assert shaper.last_active_perturbations == []
    stats = shaper.stats()
    assert stats["per_topic"][_STATE]["sent_count"] == 50
    assert stats["per_topic"][_STATE]["sent_bytes"] == 50_000
    assert stats["per_topic"][_STATE]["dropped_count"] == 0
    assert "/clock" not in stats["per_topic"]  # bypass topics are never accounted
    assert stats["queue_latency_p95_s"] == 0.0


def test_above_budget_traffic_is_queued_by_priority_and_released_after_serialization_time():
    profile = tether_profile()
    shaper = BandwidthShaper(build_bandwidth_schedule(1, profile, 10.0), profile)
    # One tick offers a 400 kB sonar frame, two 300 kB camera frames and a
    # state sample against a 1 MB/s link with a 100 kB burst allowance.
    out = shaper.apply(0.0, [(_LEFT, "L0"), (_RIGHT, "R0"), (_STATE, "S0"), (_SONAR, "N0")])
    # Bucket starts full (100 kB >= 0): sonar goes first (priority 0) and
    # drives the bucket to -300 kB; nothing else fits this tick.
    assert out == [(_SONAR, "N0")]
    assert sorted(shaper.last_active_perturbations) == sorted(
        [f"bandwidth_queue:{_STATE}", f"bandwidth_queue:{_LEFT}", f"bandwidth_queue:{_RIGHT}"]
    )
    # 0.1 s later the bucket is at -200 kB: still nothing.
    assert shaper.apply(0.1, []) == []
    # At 0.3 s the bucket is back to 0: state (1 kB, priority 1) then LEFT (seq
    # order among equal priorities) go; RIGHT waits for the 300 kB deficit.
    assert shaper.apply(0.3, []) == [(_STATE, "S0")]
    # After sending S0 the bucket is -1 kB; at 0.31 it is 9 kB -> LEFT goes.
    assert shaper.apply(0.31, []) == [(_LEFT, "L0")]
    assert shaper.apply(0.5, []) == []
    assert shaper.apply(0.61, []) == [(_RIGHT, "R0")]
    delays = dict(shaper.last_delays_s)
    assert delays[_RIGHT] == pytest.approx(0.61)
    stats = shaper.stats()
    assert stats["queue_depth"] == 0
    assert stats["queue_latency_max_s"] == pytest.approx(0.61)
    assert stats["per_topic"][_SONAR]["sent_bytes"] == 400_000
    assert stats["max_queue_depth"] == 4


def test_queue_latency_grows_with_queue_depth_until_lowest_priority_is_dropped():
    profile = tether_profile(max_queue_s=1.0)
    shaper = BandwidthShaper(build_bandwidth_schedule(1, profile, 10.0), profile)
    # Offer a stereo pair (600 kB) every 0.1 s against 1 MB/s: 6 MB/s offered.
    delays_by_tick = []
    drops = 0
    for tick in range(30):
        now_s = tick * 0.1
        shaper.apply(now_s, [(_LEFT, f"L{tick}"), (_RIGHT, f"R{tick}")])
        drops += sum(1 for p in shaper.last_active_perturbations if p.startswith("bandwidth_drop:"))
        delays_by_tick.extend(d for _t, d in shaper.last_delays_s)
    stats = shaper.stats()
    # Latency climbed with the backlog...
    assert delays_by_tick[0] == pytest.approx(0.0)
    assert max(delays_by_tick) > 0.5
    # ...but never past the configured bound, because the shaper drops the
    # newest lowest-priority frames instead of letting the queue grow.
    assert max(delays_by_tick) <= profile.max_queue_s + 0.1 + 1e-9
    assert drops > 0
    assert stats["per_topic"][_LEFT]["dropped_count"] + stats["per_topic"][_RIGHT]["dropped_count"] == drops
    offered = stats["per_topic"][_LEFT]["offered_bytes"] + stats["per_topic"][_RIGHT]["offered_bytes"]
    sent = stats["per_topic"][_LEFT]["sent_bytes"] + stats["per_topic"][_RIGHT]["sent_bytes"]
    dropped = stats["per_topic"][_LEFT]["dropped_bytes"] + stats["per_topic"][_RIGHT]["dropped_bytes"]
    assert offered == sent + dropped + stats["queued_bytes"]
    # Roughly the link rate actually got through: 2.9 s of link time at
    # 1 MB/s, plus the 100 kB burst allowance, plus at most one frame's worth
    # of deficit (the last accepted 300 kB frame drives the bucket negative).
    assert 2_500_000 <= sent <= 3_400_000
    assert any(event["kind"] == "bandwidth_drop" for event in shaper.timeline)


def test_bandwidth_drops_spare_higher_priority_topics():
    profile = tether_profile(max_queue_s=0.5)
    shaper = BandwidthShaper(build_bandwidth_schedule(1, profile, 10.0), profile)
    for tick in range(20):
        now_s = tick * 0.1
        shaper.apply(now_s, [(_SONAR, f"N{tick}"), (_LEFT, f"L{tick}"), (_RIGHT, f"R{tick}"), (_STATE, f"S{tick}")])
    stats = shaper.stats()["per_topic"]

    def dropped_fraction(topic):
        return stats[topic]["dropped_count"] / stats[topic]["offered_count"]

    # Sonar alone is 4 MB/s against a 1 MB/s link, so even sonar frames get
    # dropped -- but strictly in PREP-E-01's order: cameras give way first,
    # then vehicle state, sonar last.
    assert stats[_LEFT]["dropped_count"] > 0 and stats[_RIGHT]["dropped_count"] > 0
    assert dropped_fraction(_LEFT) >= dropped_fraction(_STATE) >= dropped_fraction(_SONAR)
    assert dropped_fraction(_SONAR) < 1.0
    assert stats[_SONAR]["sent_count"] > stats[_LEFT]["sent_count"]


def test_bandwidth_flush_releases_queued_messages_in_priority_order():
    profile = tether_profile()
    shaper = BandwidthShaper(build_bandwidth_schedule(1, profile, 10.0), profile)
    shaper.apply(0.0, [(_LEFT, "L0"), (_SONAR, "N0"), (_STATE, "S0")])
    assert shaper.flush() == [(_STATE, "S0"), (_LEFT, "L0")]
    assert shaper.stats()["queue_depth"] == 0
    assert shaper.flush() == []


def test_bandwidth_uses_the_size_callable_when_given():
    profile = tether_profile(topic_bytes={})
    shaper = BandwidthShaper(
        build_bandwidth_schedule(1, profile, 10.0), profile, size_bytes=lambda _topic, message: len(message)
    )
    shaper.apply(0.0, [(_STATE, "x" * 2_000)])
    assert shaper.stats()["per_topic"][_STATE]["offered_bytes"] == 2_000


def test_bandwidth_base_latency_holds_messages_until_the_link_delay_elapsed():
    profile = tether_profile(base_latency_s=0.05)
    shaper = BandwidthShaper(build_bandwidth_schedule(1, profile, 10.0), profile)
    assert shaper.apply(0.0, [(_STATE, "S0")]) == []
    assert shaper.apply(0.02, []) == []
    assert shaper.apply(0.05, []) == [(_STATE, "S0")]


def test_bandwidth_rate_follows_the_schedule_walk():
    profile = tether_profile()
    schedule = (BandwidthSample(0.0, 8.0), BandwidthSample(1.0, 4.0), BandwidthSample(2.0, 16.0))
    shaper = BandwidthShaper(schedule, profile)
    shaper.apply(0.0, [])
    assert shaper.current_mbps == 8.0
    shaper.apply(1.5, [])
    assert shaper.current_mbps == 4.0
    shaper.apply(2.0, [])
    assert shaper.current_mbps == 16.0
    assert [e["mbps"] for e in shaper.timeline if e["kind"] == "bandwidth_rate"] == [4.0, 16.0]


def test_fault_injector_chains_bandwidth_after_drop_and_flushes_it():
    profile = tether_profile()
    shaper = BandwidthShaper(build_bandwidth_schedule(1, profile, 10.0), profile)
    schedule = (ScheduledFault(0.0, 0, _SONAR, "drop"),)
    injector = FaultInjector(schedule, FaultInjectionProfile(), np.random.default_rng(1), bandwidth=shaper)
    out = injector.apply(0.0, [(_SONAR, "N0"), (_LEFT, "L0"), (_RIGHT, "R0")])
    # The dropped sonar frame never reaches the link, so the bucket goes to
    # LEFT instead; RIGHT is queued by the shaper.
    assert out == [(_LEFT, "L0")]
    assert injector.last_active_perturbations == [f"drop:{_SONAR}", f"bandwidth_queue:{_RIGHT}"]
    assert _SONAR not in injector.bandwidth_stats()["per_topic"]  # never reached the link
    assert injector.flush() == [(_RIGHT, "R0")]


def test_fault_injector_without_bandwidth_reports_no_stats():
    injector = FaultInjector((), FaultInjectionProfile(), np.random.default_rng(1))
    assert injector.bandwidth is None
    assert injector.bandwidth_stats() is None
