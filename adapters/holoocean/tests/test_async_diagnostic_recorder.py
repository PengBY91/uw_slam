import threading
import time

import pytest

from uw_holoocean_adapter.async_diagnostic_recorder import AsyncDiagnosticRecorder


def make_record(sequence):
    return {"sequence": sequence}


class BlockingThenFailingSink:
    """First write() blocks until released, then raises on any later call
    (never reached by design, since the worker stops after one failure) --
    exercises both "the worker is stuck" and "the worker just died"
    without needing a real disk."""

    def __init__(self):
        self.release = threading.Event()
        self.write_started = threading.Event()

    def write(self, record):
        self.write_started.set()
        self.release.wait(timeout=5.0)
        raise RuntimeError("simulated write failure")


class ImmediateSink:
    def __init__(self):
        self.written = []
        self._lock = threading.Lock()

    def write(self, record):
        with self._lock:
            self.written.append(record)


def test_blocked_or_failed_recorder_never_blocks_live_submit():
    recorder = AsyncDiagnosticRecorder(capacity=2, sink=BlockingThenFailingSink())
    started = time.monotonic()
    for sequence in range(100):
        recorder.try_submit(make_record(sequence))
    elapsed = time.monotonic() - started

    assert elapsed < 0.05
    assert recorder.stats().dropped_oldest > 0
    # The sink's first write() never returns (its release Event is never
    # set) -- the worker can't join, so keep this cleanup fast rather than
    # waiting out close()'s full default timeout.
    recorder.close(timeout_s=0.1)


def test_bounded_queue_never_exceeds_capacity_worth_of_drops():
    sink = BlockingThenFailingSink()
    recorder = AsyncDiagnosticRecorder(capacity=3, sink=sink)
    # Submit one record and wait for the worker to actually pick it up (and
    # block in write()) before submitting the rest -- otherwise whether the
    # worker manages to pop anything before the loop below finishes is a
    # genuine race, which would make the exact drop count non-deterministic.
    recorder.try_submit(make_record(0))
    assert sink.write_started.wait(timeout=1.0)

    for sequence in range(1, 10):
        recorder.try_submit(make_record(sequence))
    stats = recorder.stats()

    assert stats.submitted == 10
    assert stats.dropped_oldest == 9 - 3
    recorder.close(timeout_s=0.1)  # sink never releases -- see comment above


def test_healthy_sink_eventually_writes_submitted_records():
    sink = ImmediateSink()
    recorder = AsyncDiagnosticRecorder(capacity=8, sink=sink)
    for sequence in range(5):
        recorder.try_submit(make_record(sequence))
    recorder.close()

    assert sink.written == [make_record(i) for i in range(5)]
    stats = recorder.stats()
    assert stats.written == 5
    assert stats.worker_alive is True


def test_write_failure_marks_worker_dead_but_future_submits_still_return_immediately():
    sink = BlockingThenFailingSink()
    recorder = AsyncDiagnosticRecorder(capacity=4, sink=sink)
    recorder.try_submit(make_record(0))
    assert sink.write_started.wait(timeout=1.0)
    sink.release.set()

    deadline = time.monotonic() + 1.0
    while recorder.stats().worker_alive and time.monotonic() < deadline:
        time.sleep(0.01)
    assert recorder.stats().worker_alive is False

    started = time.monotonic()
    recorder.try_submit(make_record(1))
    assert time.monotonic() - started < 0.05
    recorder.close()


def test_capacity_must_be_positive():
    with pytest.raises(ValueError):
        AsyncDiagnosticRecorder(capacity=0, sink=ImmediateSink())
