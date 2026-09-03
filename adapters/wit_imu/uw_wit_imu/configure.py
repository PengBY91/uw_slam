"""One-shot commissioning of an HWT9053-485 (PREP-D-01).

Factory defaults are 9600 baud / 10 Hz, which is roughly 11% of the link
capacity this project needs and an order of magnitude below the required
sample rate. This script walks the register sequence in
``registers.configuration_sequence`` — unlock, output content, output
rate, baud rate, (reopen at the new baud) unlock, save — so the settings
survive a power cycle.

**Dry run is the default.** ``protocol.MANUAL_REVISION`` is empty until
someone has checked the register numbers and rate/baud encodings in
``registers.py`` against the manual that ships with the unit, and the
script refuses to write to a device while that is the case. Run it
without ``--port`` to print the exact frames and check them off:

    python -m uw_wit_imu.configure

Then, once the constants are confirmed and ``MANUAL_REVISION`` is filled
in:

    python -m uw_wit_imu.configure --port /dev/ttyUSB0 --apply
    python -m uw_wit_imu.dump --port /dev/ttyUSB0 --baud 230400 --seconds 60

The dump is the acceptance check: about 12000 of each of the four packet
types in one minute, after a power cycle.
"""
from __future__ import annotations

import argparse
import sys
import time
from typing import List, Optional, Sequence, Tuple

from uw_wit_imu import protocol, registers


def format_sequence(sequence: Sequence[Tuple[str, bytes]]) -> str:
    lines = []
    for step, (name, frame) in enumerate(sequence, start=1):
        hexed = " ".join(f"{b:02X}" for b in frame)
        lines.append(f"  {step}. {name:<8} {hexed}")
    return "\n".join(lines)


def apply_sequence(
    port: str,
    from_baud: int,
    to_baud: int,
    sequence: Sequence[Tuple[str, bytes]],
    settle_s: float = 0.2,
) -> None:
    """Writes the sequence, reopening the port at ``to_baud`` after the
    baud-rate frame (which is the last frame the old speed can carry)."""
    import serial  # imported here so --dry-run needs no pyserial

    handle = serial.Serial(port, from_baud, timeout=1.0)
    try:
        for name, frame in sequence:
            handle.write(frame)
            handle.flush()
            time.sleep(settle_s)
            if name == "baud":
                handle.close()
                time.sleep(settle_s)
                handle = serial.Serial(port, to_baud, timeout=1.0)
    finally:
        handle.close()


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="serial device, e.g. /dev/ttyUSB0; omit for a dry run")
    parser.add_argument("--from-baud", type=int, default=registers.FACTORY_BAUD_RATE,
                        help="baud the device is CURRENTLY at (default: factory 9600)")
    parser.add_argument("--baud", type=int, default=registers.TARGET_BAUD_RATE)
    parser.add_argument("--rate", type=float, default=registers.TARGET_OUTPUT_RATE_HZ)
    parser.add_argument("--apply", action="store_true",
                        help="actually write to the device (requires a filled-in MANUAL_REVISION)")
    args = parser.parse_args(argv)

    sequence = registers.configuration_sequence(args.rate, args.baud)
    required = registers.required_bytes_per_second(args.rate, registers.PREP_D01_OUTPUT_CONTENT)
    sufficient = registers.baud_rate_is_sufficient(args.baud, args.rate, registers.PREP_D01_OUTPUT_CONTENT)

    print(f"target: {args.rate:g} Hz, {args.baud} baud, content mask "
          f"0x{registers.PREP_D01_OUTPUT_CONTENT:04X} (accel + gyro + mag + quaternion)")
    print(f"link budget: {required:.0f} B/s needed, {args.baud / 10.0:.0f} B/s available "
          f"-> {'OK' if sufficient else 'INSUFFICIENT'}")
    print("register writes:")
    print(format_sequence(sequence))
    if not sufficient:
        print("refusing: the requested baud rate cannot carry the requested output rate", file=sys.stderr)
        return 2

    if not args.apply or args.port is None:
        print("\ndry run — nothing written. Check every frame above against the HWT9053 manual,")
        print("record its revision in uw_wit_imu/protocol.py MANUAL_REVISION, then re-run with")
        print("--port <device> --apply.")
        return 0
    if not protocol.MANUAL_REVISION:
        print("refusing to write: protocol.MANUAL_REVISION is empty, so the register map in "
              "registers.py has not been checked against the shipped manual.", file=sys.stderr)
        return 2

    apply_sequence(args.port, args.from_baud, args.baud, sequence)
    print(f"\nwrote {len(sequence)} frames to {args.port}. Power-cycle the device, then verify with:")
    print(f"  python -m uw_wit_imu.dump --port {args.port} --baud {args.baud} --seconds 60")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
