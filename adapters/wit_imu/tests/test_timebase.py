import random
import statistics

import pytest

from uw_wit_imu.timebase import UniformTimebase


def jittered_arrivals(count, period, jitter, seed, start=1000.0):
    """A perfect device clock seen through a jittery transport.

    ``jitter`` is kept inside the +/-0.25-period band UniformTimebase
    documents as its operating range; beyond it, arrival times alone
    cannot separate a late sample from a dropped one (see
    ``test_period_is_clamped_to_the_configured_drift_band`` and the class
    docstring), so a test using more jitter would be asserting something
    the estimator never claimed.

    The RNG is explicitly seeded and passed nowhere else -- CLAUDE.md's
    rule about never reseeding a global generator mid-run applies to test
    fixtures too, and these assertions are only meaningful if the sequence
    is reproducible.
    """
    rng = random.Random(seed)
    return [start + i * period + rng.uniform(-jitter, jitter) for i in range(count)]


def test_recovers_the_true_period_from_jittered_arrivals():
    period = 1.0 / 200.0
    timebase = UniformTimebase(200.0)
    for arrival in jittered_arrivals(1000, period, jitter=1.2e-3, seed=7):
        timebase.update(arrival)
    assert timebase.estimated_rate_hz == pytest.approx(200.0, rel=2e-3)
    assert timebase.locked


def test_recovers_a_device_clock_that_is_not_exactly_nominal():
    # A real oscillator is a few hundred ppm off; the fit must track the
    # device, not snap back to the configured nominal rate.
    true_rate = 200.0 * 1.002
    timebase = UniformTimebase(200.0)
    for arrival in jittered_arrivals(2000, 1.0 / true_rate, jitter=1e-3, seed=11):
        timebase.update(arrival)
    assert timebase.estimated_rate_hz == pytest.approx(true_rate, rel=1e-3)


def test_output_timestamps_are_far_smoother_than_the_arrivals():
    period = 1.0 / 200.0
    arrivals = jittered_arrivals(2000, period, jitter=1.2e-3, seed=13)
    timebase = UniformTimebase(200.0)
    stamps = [timebase.update(a) for a in arrivals]

    # Compare the spread of consecutive intervals: this is the whole point
    # of the reconstruction, so assert on it directly rather than on the
    # fitted period alone.
    arrival_gaps = [b - a for a, b in zip(arrivals[500:], arrivals[501:])]
    stamp_gaps = [b - a for a, b in zip(stamps[500:], stamps[501:])]
    assert statistics.pstdev(stamp_gaps) < 0.02 * statistics.pstdev(arrival_gaps)
    assert all(gap > 0 for gap in stamp_gaps)


def test_timestamps_are_strictly_increasing_even_when_an_arrival_runs_early():
    timebase = UniformTimebase(200.0)
    period = 1.0 / 200.0
    stamps = []
    for i in range(200):
        arrival = 1000.0 + i * period
        if i == 100:
            arrival -= 0.9 * period  # a burst that overtakes the model
        stamps.append(timebase.update(arrival))
    assert all(b > a for a, b in zip(stamps, stamps[1:]))


def test_a_dropped_cycle_advances_the_index_instead_of_compressing_time():
    period = 1.0 / 200.0
    timebase = UniformTimebase(200.0)
    arrivals = [1000.0 + i * period for i in range(100)]
    del arrivals[50]  # one cycle lost on the link
    stamps = [timebase.update(a) for a in arrivals]
    gap = stamps[50] - stamps[49]
    assert gap == pytest.approx(2 * period, rel=1e-6)
    assert timebase.counters.samples_missing == 1


def test_a_long_stall_resynchronises_rather_than_inventing_thousands_of_samples():
    period = 1.0 / 200.0
    timebase = UniformTimebase(200.0, resync_threshold_s=0.5)
    for i in range(100):
        timebase.update(1000.0 + i * period)
    timebase.update(1010.0)  # device reset / port reopened, 10 s later
    assert timebase.counters.resyncs == 1
    assert timebase.counters.samples_missing < 10


def test_period_is_clamped_to_the_configured_drift_band():
    # A device running 10% fast is out of any plausible oscillator band,
    # so the fit is clamped rather than followed -- otherwise a
    # misconfigured output rate would be silently absorbed into the
    # timeline instead of showing up as a fault.
    timebase = UniformTimebase(200.0, max_period_drift=0.05)
    for i in range(500):
        timebase.update(1000.0 + i * (1.0 / 220.0))
    assert timebase.period_s == pytest.approx((1.0 / 200.0) * 0.95, rel=1e-9)


def test_a_device_running_at_half_rate_is_reported_as_missing_cycles():
    # This is indistinguishable from "every other cycle is dropped" from
    # arrival times alone, and reporting it as loss is the useful answer:
    # the health report surfaces it either way.
    timebase = UniformTimebase(200.0)
    for i in range(200):
        timebase.update(1000.0 + i * (1.0 / 100.0))
    assert timebase.counters.samples_missing == 199
    assert timebase.period_s == pytest.approx(1.0 / 200.0, rel=1e-6)


def test_jitter_within_the_documented_band_never_invents_a_dropped_cycle():
    period = 1.0 / 200.0
    timebase = UniformTimebase(200.0)
    # +/-0.24 of a period, just inside the gap_threshold=1.5 limit the
    # class documents, in the worst-case alternating pattern.
    for i in range(500):
        timebase.update(1000.0 + i * period + (0.24 if i % 2 else -0.24) * period)
    assert timebase.counters.samples_missing == 0
    assert timebase.counters.resyncs == 0


def test_rejects_a_gap_threshold_that_cannot_separate_jitter_from_loss():
    with pytest.raises(ValueError):
        UniformTimebase(200.0, gap_threshold=1.0)


def test_is_deterministic_for_a_given_arrival_sequence():
    arrivals = jittered_arrivals(500, 1.0 / 200.0, jitter=1.5e-3, seed=29)
    first = [UniformTimebase(200.0).update(a) for a in arrivals]  # noqa: F841 (shape only)
    a = UniformTimebase(200.0)
    b = UniformTimebase(200.0)
    assert [a.update(x) for x in arrivals] == [b.update(x) for x in arrivals]


def test_rejects_nonsense_construction():
    with pytest.raises(ValueError):
        UniformTimebase(0.0)
    with pytest.raises(ValueError):
        UniformTimebase(200.0, window=1)
