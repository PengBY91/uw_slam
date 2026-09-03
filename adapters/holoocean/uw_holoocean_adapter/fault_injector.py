"""Deterministic, per-topic network fault scheduling and delivery for the
realtime closed loop — camera-sonar desync, drop, duplicate, bounded
reorder, clock offset/jitter, and finite outage intervals, plus a
single-thruster effectiveness fault applied to the pilot command path.

Two separate concerns, both required by the plan:
  - `build_fault_schedule` (a pure function of `seed`/`profile`/`duration_s`)
    produces a deterministic, time-sorted sequence of `ScheduledFault`
    events for the whole run up front — same seed always yields the same
    schedule (this repo's determinism rule: an explicit owned
    `numpy.random.Generator`, never global `numpy.random.seed()`).
  - `FaultInjector` consumes that schedule at runtime, one `apply()` call
    per realtime tick. It never sleeps the simulation loop: `apply()` only
    drains whatever schedule entries have a `release_time_s <= now_s`
    (a min-heap pop, not a timer/thread) and applies their effect to
    whatever (topic, message) pairs that tick actually produced.

Camera-sonar desync has no separate mechanism here — it is expressed as a
nonzero `clock_offset_s` on the sonar topic's `TopicFaultConfig` relative to
whatever the camera topics' `clock_offset_s` is (typically 0), the same way
a fixed clock skew on any single topic is expressed.

PREP-E-02 (2026-09-03) adds the `bandwidth` profile: the contract tether
(docs/ROV平台参数.md ROV-05) measures 10–40 Mbps, and the sonar alone at
768×512×8 bit @ 40 Hz is ≈126 Mbps (spec PREP-E-01), so the realtime loop
must be exercised under a link that cannot carry everything. Same two-half
split as the fault schedule: `build_bandwidth_schedule` pre-samples the
whole run's available-Mbps random walk from a seed (pure, comparable with
`==`), and `BandwidthShaper` consumes it at runtime as a deficit token
bucket with a priority queue — messages that do not fit are held (delay
grows with queue depth) or, once the backlog would exceed `max_queue_s`,
dropped lowest-priority-first (PREP-E-01's degradation order: stereo/main
camera first, then pilot video, then vehicle state; sonar frames last).
"""
from __future__ import annotations

import collections
import dataclasses
import heapq
from typing import Any, Callable, Deque, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np

from uw_holoocean_adapter.scenario_randomization import SonarDegradation, VisualDegradation


@dataclasses.dataclass(frozen=True)
class TopicFaultConfig:
    clock_offset_s: float = 0.0
    jitter_sigma_s: float = 0.0
    drop_probability: float = 0.0
    duplicate_probability: float = 0.0
    reorder_probability: float = 0.0
    reorder_max_distance: int = 3
    outage_count: int = 0
    outage_duration_s: float = 0.0


@dataclasses.dataclass(frozen=True)
class FaultInjectionProfile:
    per_topic: Mapping[str, TopicFaultConfig] = dataclasses.field(default_factory=dict)
    # How often drop/duplicate/reorder are rolled for, in simulated seconds
    # — a coarser interval than every single message keeps schedules small
    # for long runs while still landing faults across the whole duration.
    check_interval_s: float = 1.0


@dataclasses.dataclass(frozen=True, order=True)
class ScheduledFault:
    release_time_s: float
    sort_index: int  # generation-order tie-breaker: makes ordering total and
    # deterministic even when two events land on the same release_time_s,
    # without relying on comparing `topic`/`kind` strings against each other.
    topic: str
    kind: str  # "drop" | "duplicate" | "reorder" | "outage"
    reorder_distance: int = 0
    duration_s: float = 0.0  # only meaningful for "outage"


def build_fault_schedule(
    seed: int, profile: FaultInjectionProfile, duration_s: float
) -> Tuple[ScheduledFault, ...]:
    """Deterministically samples the whole run's fault schedule from `seed`
    up front. `schedule_a == schedule_b` for two calls with the same
    `(seed, profile, duration_s)`; the returned tuple's `release_time_s`
    values are always non-decreasing (see `ScheduledFault`'s `sort_index`
    for how ties are broken without depending on iteration order)."""
    if duration_s < 0:
        raise ValueError(f"duration_s must be non-negative, got {duration_s}")
    rng = np.random.default_rng(seed)
    events: List[ScheduledFault] = []
    sort_index = 0

    for topic, config in profile.per_topic.items():
        check_interval_s = profile.check_interval_s
        if check_interval_s <= 0:
            raise ValueError(f"check_interval_s must be positive, got {check_interval_s}")
        t = 0.0
        while t < duration_s:
            if config.drop_probability > 0 and rng.random() < config.drop_probability:
                events.append(ScheduledFault(t, sort_index, topic, "drop"))
                sort_index += 1
            if config.duplicate_probability > 0 and rng.random() < config.duplicate_probability:
                events.append(ScheduledFault(t, sort_index, topic, "duplicate"))
                sort_index += 1
            if config.reorder_probability > 0 and rng.random() < config.reorder_probability:
                distance = int(rng.integers(1, max(2, config.reorder_max_distance + 1)))
                events.append(ScheduledFault(t, sort_index, topic, "reorder", reorder_distance=distance))
                sort_index += 1
            t += check_interval_s

        for _ in range(config.outage_count):
            span = max(0.0, duration_s - config.outage_duration_s)
            start = float(rng.uniform(0.0, span)) if span > 0 else 0.0
            events.append(
                ScheduledFault(start, sort_index, topic, "outage", duration_s=config.outage_duration_s)
            )
            sort_index += 1

    events.sort()
    return tuple(events)


@dataclasses.dataclass(frozen=True)
class SensorDegradationWindow:
    """One scheduled degraded period: `sim_time_s` in
    `[start_s, start_s + duration_s)` is "inside" this window (see
    `sensor_degradation_active`)."""

    start_s: float
    duration_s: float


@dataclasses.dataclass(frozen=True)
class SensorDegradationSchedule:
    """Visual and sonar degradation windows are scheduled independently --
    real turbidity events and real sonar multipath/gain events have no
    reason to coincide, and testing them separately exercises the
    single-modality-degraded paths the fault matrix table actually cares
    about (see docs/rov-realtime-closed-loop-code-review-2026-08-27.md
    finding B4)."""

    visual_windows: Tuple[SensorDegradationWindow, ...] = ()
    sonar_windows: Tuple[SensorDegradationWindow, ...] = ()


def _sample_windows(
    rng: np.random.Generator, duration_s: float, window_count: int, window_duration_s: float
) -> Tuple[SensorDegradationWindow, ...]:
    if window_duration_s <= 0:
        raise ValueError(f"window_duration_s must be positive, got {window_duration_s}")
    windows = []
    for _ in range(window_count):
        span = max(0.0, duration_s - window_duration_s)
        start = float(rng.uniform(0.0, span)) if span > 0 else 0.0
        windows.append(SensorDegradationWindow(start, window_duration_s))
    return tuple(sorted(windows, key=lambda w: w.start_s))


def build_sensor_degradation_schedule(
    seed: int,
    duration_s: float,
    *,
    visual_window_count: int = 0,
    visual_window_duration_s: float = 30.0,
    sonar_window_count: int = 0,
    sonar_window_duration_s: float = 30.0,
) -> SensorDegradationSchedule:
    """Deterministically samples visual/sonar degradation window start times
    within `[0, duration_s)` -- same determinism contract as
    `build_fault_schedule`: identical arguments always yield an identical
    schedule. A window_count of 0 (the default for both modalities) yields
    no windows for that modality, matching "no scheduling" -- this function
    is opt-in, not a behavior change for any existing caller that doesn't
    invoke it."""
    if duration_s < 0:
        raise ValueError(f"duration_s must be non-negative, got {duration_s}")
    rng = np.random.default_rng(seed)
    visual_windows = (
        _sample_windows(rng, duration_s, visual_window_count, visual_window_duration_s)
        if visual_window_count > 0
        else ()
    )
    sonar_windows = (
        _sample_windows(rng, duration_s, sonar_window_count, sonar_window_duration_s)
        if sonar_window_count > 0
        else ()
    )
    return SensorDegradationSchedule(visual_windows, sonar_windows)


def sensor_degradation_active(windows: Sequence[SensorDegradationWindow], sim_time_s: float) -> bool:
    return any(w.start_s <= sim_time_s < w.start_s + w.duration_s for w in windows)


def resolve_active_degradation(
    schedule: Optional[SensorDegradationSchedule],
    visual_profile: Optional[VisualDegradation],
    sonar_profile: Optional[SonarDegradation],
    sim_time_s: float,
    *,
    baseline_visual: Optional[VisualDegradation] = None,
    baseline_sonar: Optional[SonarDegradation] = None,
) -> Tuple[Optional[VisualDegradation], Optional[SonarDegradation]]:
    """The pure "what degradation applies right now" decision --
    RealtimeRosSession.tick() (real-HoloOcean-only, never unit tested
    directly) calls this and nothing else to pick this tick's
    visual_degradation/sonar_degradation, keeping the actual scheduling
    logic in this fully unit-testable function instead.

    `schedule=None` always returns the baseline (typically None/clear) --
    exactly the pre-B4 "no scheduling at all, degradation is constant for
    the whole run or absent" behavior, so existing callers that never pass
    a schedule see no behavior change. Inside a scheduled window,
    `visual_profile`/`sonar_profile` (typically
    scenario_randomization.PRESET_CRITICAL_DEGRADED's components) apply;
    outside every window, `baseline_visual`/`baseline_sonar` applies --
    this is what gives a fault an actual start/duration/RECOVERY, per the
    simulation spec's fault matrix, rather than a config that is either
    permanently on or never exercised at all."""
    if schedule is None:
        return baseline_visual, baseline_sonar
    visual = visual_profile if sensor_degradation_active(schedule.visual_windows, sim_time_s) else baseline_visual
    sonar = sonar_profile if sensor_degradation_active(schedule.sonar_windows, sim_time_s) else baseline_sonar
    return visual, sonar


@dataclasses.dataclass(frozen=True)
class ThrusterFaultConfig:
    """A single degraded/damaged thruster: `effectiveness_multiplier` scales
    that one channel's already-shaped (deadzone/saturation/lag-filtered)
    output — 1.0 is healthy, 0.0 is a fully seized/disconnected thruster."""

    thruster_index: int
    effectiveness_multiplier: float = 1.0


def apply_thruster_fault(
    shaped_command: Sequence[float], fault: Optional[ThrusterFaultConfig]
) -> List[float]:
    """Post-processing step around `PilotCommandModel.step()`'s output —
    deliberately not part of that class, which stays a pure per-channel
    actuator-response model with no fault-injection concerns of its own."""
    command = list(shaped_command)
    if fault is not None and 0 <= fault.thruster_index < len(command):
        command[fault.thruster_index] *= fault.effectiveness_multiplier
    return command


@dataclasses.dataclass(frozen=True)
class BandwidthProfile:
    """A tether link budget. `topic_priority` maps a topic to an integer
    where LOWER means higher priority (sent first, dropped last); topics not
    listed get `default_priority`. `topic_bytes` is the nominal serialized
    size of one message on a topic, used when the shaper is not given a
    `size_bytes` callable (unit tests pass bare strings as messages; the
    realtime session passes nominal sizes derived from the scenario
    manifest). `bypass_topics` (default: `/clock`) are never shaped — the
    simulation clock is not tether traffic."""

    nominal_mbps: float = 20.0
    min_mbps: float = 10.0
    max_mbps: float = 40.0
    # Random-walk step of the available rate, in Mbps per sqrt(second); 0
    # keeps the rate constant at `nominal_mbps` for the whole run.
    walk_sigma_mbps_per_s: float = 0.0
    walk_interval_s: float = 1.0
    topic_priority: Mapping[str, int] = dataclasses.field(default_factory=dict)
    topic_bytes: Mapping[str, int] = dataclasses.field(default_factory=dict)
    default_priority: int = 100
    default_bytes: int = 1024
    # Once the queued backlog (bytes still waiting, at the current rate)
    # would take longer than this to drain, the lowest-priority queued
    # messages are dropped until it fits — a real link's bounded send
    # buffer, so a starved camera never accumulates unbounded latency.
    max_queue_s: float = 2.0
    # Fixed one-way link latency every message experiences on top of the
    # serialization/queueing delay (a message becomes eligible for release
    # only `base_latency_s` after it was offered).
    base_latency_s: float = 0.0
    # Token-bucket burst allowance in seconds of link time at the current
    # rate: how much may be sent "for free" after an idle period.
    bucket_depth_s: float = 0.1
    bypass_topics: Tuple[str, ...] = ("/clock",)


@dataclasses.dataclass(frozen=True, order=True)
class BandwidthSample:
    time_s: float
    mbps: float


def build_bandwidth_schedule(
    seed: int, profile: BandwidthProfile, duration_s: float
) -> Tuple[BandwidthSample, ...]:
    """Deterministically pre-samples the available link rate for the whole
    run: a Gaussian random walk starting at `nominal_mbps`, stepped every
    `walk_interval_s`, clamped to `[min_mbps, max_mbps]`. Same contract as
    `build_fault_schedule`: identical `(seed, profile, duration_s)` always
    yields an identical tuple; `walk_sigma_mbps_per_s == 0` yields the
    single constant sample, so a "just cap the link" profile has no random
    component at all."""
    if duration_s < 0:
        raise ValueError(f"duration_s must be non-negative, got {duration_s}")
    if profile.walk_interval_s <= 0:
        raise ValueError(f"walk_interval_s must be positive, got {profile.walk_interval_s}")
    if not (0.0 < profile.min_mbps <= profile.nominal_mbps <= profile.max_mbps):
        raise ValueError(
            "bandwidth profile must satisfy 0 < min_mbps <= nominal_mbps <= max_mbps, got "
            f"min={profile.min_mbps} nominal={profile.nominal_mbps} max={profile.max_mbps}"
        )
    if profile.walk_sigma_mbps_per_s < 0:
        raise ValueError(f"walk_sigma_mbps_per_s must be non-negative, got {profile.walk_sigma_mbps_per_s}")
    samples = [BandwidthSample(0.0, float(profile.nominal_mbps))]
    if profile.walk_sigma_mbps_per_s == 0:
        return tuple(samples)
    rng = np.random.default_rng(seed)
    step_sigma = profile.walk_sigma_mbps_per_s * float(np.sqrt(profile.walk_interval_s))
    mbps = float(profile.nominal_mbps)
    t = profile.walk_interval_s
    while t < duration_s:
        mbps += float(rng.normal(0.0, step_sigma))
        mbps = min(profile.max_mbps, max(profile.min_mbps, mbps))
        samples.append(BandwidthSample(float(t), mbps))
        t += profile.walk_interval_s
    return tuple(samples)


class BandwidthShaper:
    """Runtime half of the bandwidth profile: a deficit token bucket fed at
    the scheduled rate, plus a priority queue for whatever does not fit.

    Per `apply(now_s, messages)` call: (1) advance the rate to the latest
    schedule sample at or before `now_s`; (2) refill the bucket for the
    elapsed simulated time (capped at `bucket_depth_s` of link time);
    (3) enqueue this tick's messages; (4) release queued messages in
    (priority, arrival) order while the bucket is non-negative, charging
    each message's bytes — a message larger than the bucket is sent as
    soon as the bucket is non-negative and drives it negative, which is
    exactly its serialization time at the current rate; (5) drop the
    lowest-priority queued messages until the remaining backlog drains
    within `max_queue_s`. Nothing here sleeps: delay is expressed purely as
    "released on a later `apply()` call", same as the reorder buffer."""

    def __init__(
        self,
        schedule: Sequence[BandwidthSample],
        profile: BandwidthProfile,
        size_bytes: Optional[Callable[[str, Any], int]] = None,
    ):
        if not schedule:
            raise ValueError("bandwidth schedule must contain at least one sample")
        self._schedule: List[BandwidthSample] = sorted(schedule)
        self._schedule_index = 0
        self._profile = profile
        self._size_bytes = size_bytes
        self._current_mbps = self._schedule[0].mbps
        self._tokens_bytes: Optional[float] = None  # None until the first apply()
        self._last_time_s: Optional[float] = None
        self._sequence = 0
        # Each entry: [priority, sequence, topic, message, offered_time_s, size_bytes]
        self._queue: List[List[Any]] = []
        self._offered: Dict[str, List[int]] = collections.defaultdict(lambda: [0, 0])  # [count, bytes]
        self._sent: Dict[str, List[int]] = collections.defaultdict(lambda: [0, 0])
        self._dropped: Dict[str, List[int]] = collections.defaultdict(lambda: [0, 0])
        self._delays_s: List[float] = []
        self._max_queue_depth = 0
        self.last_active_perturbations: List[str] = []
        self.last_delays_s: List[Tuple[str, float]] = []
        self.timeline: List[Dict[str, Any]] = []

    @property
    def current_mbps(self) -> float:
        return self._current_mbps

    def _rate_bytes_per_s(self) -> float:
        return self._current_mbps * 1e6 / 8.0

    def _advance_rate(self, now_s: float) -> None:
        while (
            self._schedule_index + 1 < len(self._schedule)
            and self._schedule[self._schedule_index + 1].time_s <= now_s
        ):
            self._schedule_index += 1
            new_mbps = self._schedule[self._schedule_index].mbps
            if new_mbps != self._current_mbps:
                self._current_mbps = new_mbps
                self.timeline.append({"time_s": now_s, "kind": "bandwidth_rate", "mbps": new_mbps})

    def _message_size(self, topic: str, message: Any) -> int:
        if self._size_bytes is not None:
            return max(0, int(self._size_bytes(topic, message)))
        return max(0, int(self._profile.topic_bytes.get(topic, self._profile.default_bytes)))

    def _priority(self, topic: str) -> int:
        return int(self._profile.topic_priority.get(topic, self._profile.default_priority))

    def apply(self, now_s: float, messages: Sequence[Tuple[str, Any]]) -> List[Tuple[str, Any]]:
        self.last_active_perturbations = []
        self.last_delays_s = []
        self._advance_rate(now_s)
        rate = self._rate_bytes_per_s()
        cap = self._profile.bucket_depth_s * rate
        if self._tokens_bytes is None or self._last_time_s is None:
            self._tokens_bytes = cap
        else:
            elapsed = max(0.0, now_s - self._last_time_s)
            self._tokens_bytes = min(cap, self._tokens_bytes + elapsed * rate)
        self._last_time_s = now_s

        output: List[Tuple[str, Any]] = []
        for topic, message in messages:
            if topic in self._profile.bypass_topics:
                output.append((topic, message))
                continue
            size = self._message_size(topic, message)
            self._offered[topic][0] += 1
            self._offered[topic][1] += size
            self._queue.append([self._priority(topic), self._sequence, topic, message, now_s, size])
            self._sequence += 1
        self._queue.sort(key=lambda entry: (entry[0], entry[1]))
        self._max_queue_depth = max(self._max_queue_depth, len(self._queue))

        still_waiting: List[List[Any]] = []
        for entry in self._queue:
            priority, _seq, topic, message, offered_s, size = entry
            eligible = now_s >= offered_s + self._profile.base_latency_s
            # -1e-6 bytes: float round-off from summing tick intervals
            # (0.1 + 0.2 ...) must not hold a message for an extra tick.
            if eligible and self._tokens_bytes >= -1e-6:
                self._tokens_bytes -= size
                delay = now_s - offered_s
                self._delays_s.append(delay)
                self.last_delays_s.append((topic, delay))
                self._sent[topic][0] += 1
                self._sent[topic][1] += size
                output.append((topic, message))
            else:
                still_waiting.append(entry)
        self._queue = still_waiting

        # Backlog control: everything still queued has to wait for the
        # bucket to recover from its deficit AND for every higher-priority
        # byte ahead of it. Drop from the lowest-priority end until the
        # drain time fits within max_queue_s.
        deficit_bytes = max(0.0, -self._tokens_bytes)
        queued_bytes = sum(entry[5] for entry in self._queue)
        while self._queue and (deficit_bytes + queued_bytes) / rate > self._profile.max_queue_s:
            victim = self._queue.pop()  # sorted ascending by (priority, seq): last == lowest priority, newest
            _priority, _seq, topic, _message, offered_s, size = victim
            queued_bytes -= size
            self._dropped[topic][0] += 1
            self._dropped[topic][1] += size
            self.last_active_perturbations.append(f"bandwidth_drop:{topic}")
            self.timeline.append(
                {"time_s": now_s, "kind": "bandwidth_drop", "topic": topic, "queued_s": now_s - offered_s}
            )
        for entry in self._queue:
            if entry[4] == now_s:
                self.last_active_perturbations.append(f"bandwidth_queue:{entry[2]}")
        return output

    def flush(self) -> List[Tuple[str, Any]]:
        """Releases everything still queued, in (priority, arrival) order,
        without charging the bucket — run-end drain, same role as
        `FaultInjector.flush()` for the reorder buffers."""
        self._queue.sort(key=lambda entry: (entry[0], entry[1]))
        output = [(entry[2], entry[3]) for entry in self._queue]
        for entry in self._queue:
            self._sent[entry[2]][0] += 1
            self._sent[entry[2]][1] += entry[5]
        self._queue = []
        return output

    def stats(self) -> Dict[str, Any]:
        """Byte accounting per topic plus queueing-latency percentiles over
        every message released so far (flush() releases are not counted as
        delays, they never actually crossed the link)."""
        topics = sorted(set(self._offered) | set(self._sent) | set(self._dropped))
        per_topic = {
            topic: {
                "offered_count": self._offered[topic][0],
                "offered_bytes": self._offered[topic][1],
                "sent_count": self._sent[topic][0],
                "sent_bytes": self._sent[topic][1],
                "dropped_count": self._dropped[topic][0],
                "dropped_bytes": self._dropped[topic][1],
            }
            for topic in topics
        }
        delays = np.asarray(self._delays_s, dtype=float)
        return {
            "current_mbps": self._current_mbps,
            "queue_depth": len(self._queue),
            "queued_bytes": int(sum(entry[5] for entry in self._queue)),
            "max_queue_depth": self._max_queue_depth,
            "queue_latency_p50_s": float(np.percentile(delays, 50)) if delays.size else 0.0,
            "queue_latency_p95_s": float(np.percentile(delays, 95)) if delays.size else 0.0,
            "queue_latency_max_s": float(delays.max()) if delays.size else 0.0,
            "per_topic": per_topic,
        }


class FaultInjector:
    """Runtime delivery half of the fault system: owns the schedule
    (a min-heap keyed by `release_time_s`) plus whatever per-topic delivery
    state (active outages, one-shot pending effects, bounded reorder
    buffers) that schedule has produced so far. `apply()` never sleeps or
    blocks — it only ever reacts to the `now_s` the caller supplies."""

    def __init__(self, schedule: Sequence[ScheduledFault], profile: FaultInjectionProfile,
                 rng: np.random.Generator, bandwidth: Optional[BandwidthShaper] = None):
        self._heap: List[ScheduledFault] = list(schedule)
        heapq.heapify(self._heap)
        self._profile = profile
        self._rng = rng
        # Optional PREP-E-02 link shaper, applied AFTER drop/duplicate/
        # reorder/outage so what reaches the tether is what the per-topic
        # faults left over. None keeps every existing caller byte-identical.
        self._bandwidth = bandwidth
        self._outage_until: Dict[str, float] = {}
        self._pending_effects: Dict[str, Deque[Tuple[str, int]]] = collections.defaultdict(collections.deque)
        # Each buffered entry is (topic, message, messages_remaining_before_release):
        # every subsequent message on that topic (this one included) decrements
        # every buffered entry's countdown by one BEFORE this message itself is
        # emitted, so a reordered message is bounded to be delivered no later
        # than `reorder_distance` messages after the one that triggered it —
        # not just delayed by a single one-shot slot, which a naive "hold the
        # one flagged message" implementation would produce.
        self._reorder_buffers: Dict[str, Deque[List[Any]]] = collections.defaultdict(collections.deque)
        self.last_active_perturbations: List[str] = []

    def _drain_due(self, now_s: float) -> None:
        while self._heap and self._heap[0].release_time_s <= now_s:
            event = heapq.heappop(self._heap)
            if event.kind == "outage":
                until = event.release_time_s + event.duration_s
                self._outage_until[event.topic] = max(self._outage_until.get(event.topic, float("-inf")), until)
            else:
                self._pending_effects[event.topic].append((event.kind, event.reorder_distance))

    def _clock_offset_s(self, topic: str) -> float:
        config = self._profile.per_topic.get(topic)
        if config is None:
            return 0.0
        offset_s = config.clock_offset_s
        if config.jitter_sigma_s > 0:
            offset_s += float(self._rng.normal(0.0, config.jitter_sigma_s))
        return offset_s

    def effective_log_time_ns(self, topic: str, nominal_log_time_ns: int) -> int:
        """Applies this topic's configured fixed clock offset plus sampled
        jitter to a nominal log time — the "fixed clock offset, jitter,
        camera-sonar desync" axes, which are continuous per-message biases
        rather than discrete schedule events."""
        return nominal_log_time_ns + int(round(self._clock_offset_s(topic) * 1e9))

    def apply(self, now_s: float, messages: Sequence[Tuple[str, Any]]) -> List[Tuple[str, Any]]:
        """Applies drop/duplicate/bounded-reorder/outage to one tick's
        (topic, message) pairs, given the current simulated time `now_s`.
        Returns the messages that should actually be published this tick —
        possibly fewer (drop, outage, a message held back for reorder),
        possibly more (duplicate, a previously held-back message released).
        Call `flush()` once at run end to release anything still held in a
        reorder buffer, so a bounded reorder never permanently swallows a
        message."""
        self._drain_due(now_s)
        self.last_active_perturbations = []
        output: List[Tuple[str, Any]] = []

        for topic, message in messages:
            if now_s < self._outage_until.get(topic, float("-inf")):
                self.last_active_perturbations.append(f"outage:{topic}")
                continue

            pending = self._pending_effects.get(topic)
            kind, reorder_distance = pending.popleft() if pending else (None, 0)

            if kind == "drop":
                self.last_active_perturbations.append(f"drop:{topic}")
                continue

            buffer = self._reorder_buffers.get(topic)
            if buffer:
                still_waiting: Deque[List[Any]] = collections.deque()
                for entry in buffer:
                    entry[1] -= 1
                    if entry[1] <= 0:
                        output.append((topic, entry[0]))
                    else:
                        still_waiting.append(entry)
                self._reorder_buffers[topic] = still_waiting

            if kind == "reorder":
                self._reorder_buffers[topic].append([message, reorder_distance])
                self.last_active_perturbations.append(f"reorder:{topic}")
                continue

            output.append((topic, message))
            if kind == "duplicate":
                self.last_active_perturbations.append(f"duplicate:{topic}")
                output.append((topic, message))

        if self._bandwidth is not None:
            output = self._bandwidth.apply(now_s, output)
            self.last_active_perturbations.extend(self._bandwidth.last_active_perturbations)
        return output

    @property
    def bandwidth(self) -> Optional[BandwidthShaper]:
        return self._bandwidth

    def bandwidth_stats(self) -> Optional[Dict[str, Any]]:
        return None if self._bandwidth is None else self._bandwidth.stats()

    def flush(self) -> List[Tuple[str, Any]]:
        """Drains every message still held in a reorder buffer, in FIFO
        order per topic. Call once at run end, so a bounded reorder never
        permanently swallows a message that never got enough subsequent
        traffic on its topic to finish counting down."""
        output: List[Tuple[str, Any]] = []
        for topic, buffer in self._reorder_buffers.items():
            while buffer:
                message, _remaining = buffer.popleft()
                output.append((topic, message))
        if self._bandwidth is not None:
            output.extend(self._bandwidth.flush())
        return output
