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


# docs/rov-realtime-closed-loop-code-review-2026-08-27.md finding B4: visual/
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
