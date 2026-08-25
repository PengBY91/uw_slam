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
"""
from __future__ import annotations

import collections
import dataclasses
import heapq
from typing import Any, Deque, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np


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


class FaultInjector:
    """Runtime delivery half of the fault system: owns the schedule
    (a min-heap keyed by `release_time_s`) plus whatever per-topic delivery
    state (active outages, one-shot pending effects, bounded reorder
    buffers) that schedule has produced so far. `apply()` never sleeps or
    blocks — it only ever reacts to the `now_s` the caller supplies."""

    def __init__(self, schedule: Sequence[ScheduledFault], profile: FaultInjectionProfile,
                 rng: np.random.Generator):
        self._heap: List[ScheduledFault] = list(schedule)
        heapq.heapify(self._heap)
        self._profile = profile
        self._rng = rng
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

        return output

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
        return output
