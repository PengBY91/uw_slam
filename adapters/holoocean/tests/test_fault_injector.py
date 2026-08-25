from itertools import pairwise

import numpy as np

from uw_holoocean_adapter.fault_injector import (
    FaultInjectionProfile,
    FaultInjector,
    ScheduledFault,
    ThrusterFaultConfig,
    TopicFaultConfig,
    apply_thruster_fault,
    build_fault_schedule,
)

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
