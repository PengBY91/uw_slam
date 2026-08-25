"""A bounded, drop-oldest, non-blocking tap for diagnostic recording (e.g.
an MCAP bag via `canonical_writer.CanonicalMcapWriter`) that must never
affect the live sensor/algorithm/HMI/pilot-command scheduling loop.

`try_submit()` only ever touches an in-process bounded queue — it never
waits on the sink (disk I/O, a slow/blocked writer, ENOSPC) or on the
worker thread. A dedicated background thread is the only thing that ever
calls `sink.write()`; if that call blocks or raises, the worker degrades
(stops recording, `stats().worker_alive` goes false) but `try_submit()`
keeps accepting and dropping oldest exactly as before — recording is
diagnostic evidence only, never load-bearing for anything else in the
realtime loop.
"""
from __future__ import annotations

import collections
import dataclasses
import threading
from typing import Any, Deque, Protocol


class DiagnosticSink(Protocol):
    def write(self, record: Any) -> None: ...


@dataclasses.dataclass
class AsyncDiagnosticRecorderStats:
    submitted: int = 0
    written: int = 0
    dropped_oldest: int = 0
    write_failures: int = 0
    worker_alive: bool = True


class AsyncDiagnosticRecorder:
    def __init__(self, capacity: int, sink: DiagnosticSink):
        if capacity <= 0:
            raise ValueError(f"capacity must be positive, got {capacity}")
        self._capacity = capacity
        self._sink = sink
        self._lock = threading.Lock()
        self._not_empty = threading.Condition(self._lock)
        self._queue: Deque[Any] = collections.deque()
        self._stats = AsyncDiagnosticRecorderStats()
        self._closed = False
        self._worker = threading.Thread(
            target=self._run, name="async-diagnostic-recorder", daemon=True
        )
        self._worker.start()

    def try_submit(self, record: Any) -> None:
        """Never blocks: pushes onto a bounded queue, dropping the oldest
        queued (not-yet-written) record if already at capacity. Safe to
        call even after the worker has stopped (e.g. a failed sink) —
        records just keep accumulating dropped-oldest with nothing ever
        draining them, which is exactly the intended degraded behavior."""
        with self._not_empty:
            if len(self._queue) >= self._capacity:
                self._queue.popleft()
                self._stats.dropped_oldest += 1
            self._queue.append(record)
            self._stats.submitted += 1
            self._not_empty.notify()

    def stats(self) -> AsyncDiagnosticRecorderStats:
        with self._lock:
            return dataclasses.replace(self._stats)

    def close(self, timeout_s: float = 5.0) -> None:
        """Signals the worker to stop once the queue drains (or it is
        already dead) and joins it. Does not itself block `try_submit` —
        only meant to be called once, at shutdown."""
        with self._not_empty:
            self._closed = True
            self._not_empty.notify_all()
        self._worker.join(timeout=timeout_s)

    def _run(self) -> None:
        while True:
            with self._not_empty:
                while not self._queue and not self._closed:
                    self._not_empty.wait()
                if not self._queue and self._closed:
                    return
                record = self._queue.popleft()
            try:
                self._sink.write(record)
            except Exception:
                with self._lock:
                    self._stats.write_failures += 1
                    self._stats.worker_alive = False
                return
            with self._lock:
                self._stats.written += 1
