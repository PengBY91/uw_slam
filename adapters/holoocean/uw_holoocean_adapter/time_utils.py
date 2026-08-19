"""Capture-time / receive-time separation, per ObservationHeader's contract
(schemas/proto/uw/domain/observation.proto, time.proto).

Audit finding (platform architecture section 22.3): ocean_t's main.py and
svin2_pipeline.py each computed a single timestamp differently
(`count/ticks_per_sec` vs `state.get("t", ...)`), with no capture/receive
distinction and no declared clock domain. This module is the one place
that produces uw.domain.Stamp/ClockDomain values for the HoloOcean adapter,
so every observation goes through the same logic.
"""
from __future__ import annotations

import time as _time


def make_stamp(schema_pb2_time_module, seconds: float):
    """Builds a uw.domain.Stamp from a float seconds value. Takes the
    generated time_pb2 module as a parameter (rather than importing it here)
    so this module has no hard dependency on the generated-code output
    directory layout."""
    stamp = schema_pb2_time_module.Stamp()
    whole = int(seconds // 1)
    stamp.seconds = whole
    stamp.nanos = int(round((seconds - whole) * 1e9))
    return stamp


def wall_clock_seconds() -> float:
    """Receive-time source. Deliberately NOT used for estimation (capture_time
    is) — only for scheduling/latency accounting, per the ObservationHeader contract."""
    return _time.time()
