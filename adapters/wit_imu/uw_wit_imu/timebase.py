"""Uniform sample timeline reconstruction (PREP-D-02 step 2).

The problem: the HWT9053 free-runs at a fixed internal rate and carries no
sequence number and no device timestamp. All the Pi sees is an arrival
time per packet group, and those arrival times carry USB/RS-485 buffering
jitter of a millisecond or more — comparable to the 5 ms sample period
itself. Stamping each sample with its raw arrival time would push that
jitter straight into the IMU preintegration frontend, whose zero-order
hold assumes the interval boundaries it is given are real.

So: estimate the device's own phase and frequency from the arrival times
and emit a uniform timeline, keeping arrival time only as a corrector.

Implementation is a windowed least-squares fit of ``t = origin + n *
period`` over the last ``window`` accepted samples, rather than the
gain-tuned type-2 loop the spec sketched as "a PLL". Same steady-state
behaviour, but with no loop gains to tune, an exactly reproducible output
for a given input sequence (this repo's determinism rule), and a directly
testable convergence property. The sample index ``n`` is inferred from the
arrival time — a gap of k missing samples advances ``n`` by k+1, so a drop
shifts nothing downstream.

Everything here is pure arithmetic on floats; no clock is read inside, the
caller passes arrival times in. That is what makes the tests below
deterministic and hardware-free.
"""
from __future__ import annotations

import dataclasses
from collections import deque
from typing import Deque, Optional, Tuple


@dataclasses.dataclass
class TimebaseCounters:
    samples_accepted: int = 0
    samples_missing: int = 0   # inferred gaps, i.e. dropped device cycles
    resyncs: int = 0           # arrival so far off the model that it was rebased


class UniformTimebase:
    """Turns jittery arrival times into a uniform sample timeline.

    Parameters
    ----------
    nominal_rate_hz:
        The configured device output rate. Used as the initial period and
        as the anchor for the ``max_period_drift`` sanity clamp — a fitted
        period outside that band means the device is not running at the
        rate it was configured for, which is a fault to report rather than
        a value to track.
    window:
        Number of recent samples in the regression. 400 at 200 Hz is 2 s,
        long enough to average the jitter down by ~20x, short enough to
        follow real oscillator drift.
    resync_threshold_s:
        An inter-arrival gap this large is treated as a stream
        discontinuity (device reset, port reopened, host suspend) rather
        than as dropped cycles: the model restarts from that arrival.
    gap_threshold:
        How many periods an inter-arrival gap must exceed before it is
        read as dropped cycles rather than jitter. 1.5 splits the two
        cases cleanly: a real single drop lands at ~2.0 periods, while
        jitter has to reach +/-0.25 of a period to reach 1.5. Beyond that
        much jitter, arrival times alone cannot distinguish "late sample"
        from "dropped sample" at all, and the right answer is a faster
        link rather than a cleverer estimator -- at 200 Hz and 230400
        baud a full four-packet cycle takes 1.9 ms to transmit, so
        sub-millisecond jitter is what this is designed for.
    max_period_drift:
        Fractional band around the nominal period the fitted period is
        clamped to.
    """

    def __init__(
        self,
        nominal_rate_hz: float,
        window: int = 400,
        resync_threshold_s: float = 0.5,
        max_period_drift: float = 0.05,
        gap_threshold: float = 1.5,
    ) -> None:
        if nominal_rate_hz <= 0.0:
            raise ValueError("nominal_rate_hz must be > 0")
        if window < 2:
            raise ValueError("window must be >= 2")
        self._nominal_period = 1.0 / nominal_rate_hz
        self._window = window
        self._resync_threshold_s = resync_threshold_s
        self._max_period_drift = max_period_drift
        if gap_threshold <= 1.0:
            raise ValueError("gap_threshold must be > 1")
        self._gap_threshold = gap_threshold

        self._samples: Deque[Tuple[int, float]] = deque(maxlen=window)
        self._origin: Optional[float] = None
        self._period = self._nominal_period
        self._next_index = 0
        self._last_index: Optional[int] = None
        self._last_arrival_s: Optional[float] = None
        self.counters = TimebaseCounters()

    @property
    def period_s(self) -> float:
        return self._period

    @property
    def estimated_rate_hz(self) -> float:
        return 1.0 / self._period

    @property
    def locked(self) -> bool:
        """True once the fit has enough samples to be better than nominal."""
        return len(self._samples) >= min(self._window, 20)

    def _reset(self, arrival_time_s: float) -> None:
        self._samples.clear()
        self._origin = arrival_time_s
        self._period = self._nominal_period
        self._next_index = 0
        self._last_index = None
        self._last_arrival_s = None

    def update(self, arrival_time_s: float) -> float:
        """Feeds one sample's arrival time; returns its uniform timestamp.

        The returned timestamp is the model's prediction for the inferred
        sample index, NOT the arrival time — that is the whole point.
        """
        if self._origin is None:
            self._reset(arrival_time_s)

        if self._last_index is None or self._last_arrival_s is None:
            index = 0
        else:
            # Deliberately measured against the PREVIOUS ARRIVAL rather
            # than against the fitted line's absolute prediction. The
            # absolute form feeds any origin error straight back into the
            # index inference, which then biases the next fit in the same
            # direction -- a loop that runs away until the period clamp
            # catches it. The local form has no such feedback path.
            elapsed = arrival_time_s - self._last_arrival_s
            if abs(elapsed) > self._resync_threshold_s:
                self.counters.resyncs += 1
                self._reset(arrival_time_s)
                index = 0
            else:
                steps = 1
                if elapsed > self._gap_threshold * self._period:
                    steps = max(1, round(elapsed / self._period))
                    self.counters.samples_missing += steps - 1
                index = self._last_index + steps

        self._samples.append((index, arrival_time_s))
        self._last_index = index
        self._last_arrival_s = arrival_time_s
        self._next_index = index + 1
        self.counters.samples_accepted += 1
        self._refit()
        return self._origin + index * self._period

    def _refit(self) -> None:
        n = len(self._samples)
        if n < 2:
            return
        sum_i = sum(float(i) for i, _ in self._samples)
        sum_t = sum(t for _, t in self._samples)
        mean_i = sum_i / n
        mean_t = sum_t / n
        numerator = sum((i - mean_i) * (t - mean_t) for i, t in self._samples)
        denominator = sum((i - mean_i) ** 2 for i, _ in self._samples)
        if denominator <= 0.0:
            return
        period = numerator / denominator
        low = self._nominal_period * (1.0 - self._max_period_drift)
        high = self._nominal_period * (1.0 + self._max_period_drift)
        self._period = min(max(period, low), high)
        self._origin = mean_t - mean_i * self._period
