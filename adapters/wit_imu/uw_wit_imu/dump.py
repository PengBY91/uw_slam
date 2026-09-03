"""Packet counter / sanity dump for a configured HWT9053-485 (PREP-D-01
acceptance).

Counts decoded packets per type, checksum failures and resynchronisation
bytes over a fixed window, and prints the reconstructed sample rate from
``UniformTimebase``. The acceptance criterion is roughly 12000 of each of
the four enabled types in 60 s at 200 Hz, with no checksum failures.

Also reads from a file (``--replay``) instead of a serial port, which is
how the offline tests and PREP-A-13's device-emulation stream exercise the
exact same counting code with no hardware.
"""
from __future__ import annotations

import argparse
import collections
import sys
import time
from typing import BinaryIO, List, Optional

from uw_wit_imu import protocol, registers
from uw_wit_imu.timebase import UniformTimebase


def count_stream(
    source: BinaryIO,
    duration_s: float,
    nominal_rate_hz: float,
    clock=time.monotonic,
    chunk_size: int = 4096,
) -> dict:
    stream = protocol.PacketStream()
    timebase = UniformTimebase(nominal_rate_hz)
    per_type: collections.Counter = collections.Counter()
    started = clock()
    last_cycle_start: Optional[float] = None
    seen_this_cycle = set()

    while clock() - started < duration_s:
        chunk = source.read(chunk_size)
        if not chunk:
            break
        now = clock()
        for packet in stream.feed(chunk):
            per_type[packet.type.name] += 1
            if packet.type in seen_this_cycle:
                seen_this_cycle.clear()
            if not seen_this_cycle:
                timebase.update(now)
                last_cycle_start = now
            seen_this_cycle.add(packet.type)

    elapsed = max(clock() - started, 1e-9)
    return {
        "elapsed_s": elapsed,
        "per_type": dict(per_type),
        "packets_decoded": stream.counters.packets_decoded,
        "checksum_failures": stream.counters.checksum_failures,
        "resync_bytes_discarded": stream.counters.resync_bytes_discarded,
        "fitted_rate_hz": timebase.estimated_rate_hz,
        "timebase_locked": timebase.locked,
        "cycles_missing": timebase.counters.samples_missing,
        "last_cycle_start_s": last_cycle_start,
    }


def format_report(result: dict, nominal_rate_hz: float) -> str:
    lines = [f"window: {result['elapsed_s']:.1f} s at a nominal {nominal_rate_hz:g} Hz"]
    expected = int(round(nominal_rate_hz * result["elapsed_s"]))
    for packet_type in protocol.ENABLED_PACKET_TYPES:
        got = result["per_type"].get(packet_type.name, 0)
        ratio = got / expected if expected else 0.0
        flag = "OK " if ratio >= 0.999 else "LOW"
        lines.append(f"  {flag} {packet_type.name:<18} {got:>7} / {expected} ({ratio * 100:.2f}%)")
    for name, count in sorted(result["per_type"].items()):
        if name not in {t.name for t in protocol.ENABLED_PACKET_TYPES}:
            lines.append(f"  !!  {name:<18} {count:>7}  (should be disabled — re-run configure.py)")
    lines.append(f"checksum failures: {result['checksum_failures']}")
    lines.append(f"resync bytes discarded: {result['resync_bytes_discarded']}")
    lines.append(f"inferred missing cycles: {result['cycles_missing']}")
    lines.append(f"fitted rate: {result['fitted_rate_hz']:.4f} Hz "
                 f"(locked={result['timebase_locked']})")
    return "\n".join(lines)


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--port", help="serial device, e.g. /dev/ttyUSB0")
    source.add_argument("--replay", help="raw capture file to read instead of a serial port")
    parser.add_argument("--baud", type=int, default=registers.TARGET_BAUD_RATE)
    parser.add_argument("--rate", type=float, default=registers.TARGET_OUTPUT_RATE_HZ)
    parser.add_argument("--seconds", type=float, default=60.0)
    args = parser.parse_args(argv)

    if args.port:
        import serial

        handle = serial.Serial(args.port, args.baud, timeout=0.1)
    else:
        handle = open(args.replay, "rb")
    try:
        result = count_stream(handle, args.seconds, args.rate)
    finally:
        handle.close()

    print(format_report(result, args.rate))
    ok = result["checksum_failures"] == 0 and all(
        result["per_type"].get(t.name, 0) >= 0.999 * args.rate * result["elapsed_s"]
        for t in protocol.ENABLED_PACKET_TYPES
    )
    if not ok:
        print("\nFAIL: see the flagged lines above", file=sys.stderr)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
