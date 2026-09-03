"""The BlueOS-extension entry point (PREP-D-02 step 4): the thin I/O loop
around ``ImuForwarder``.

Everything that could be got wrong in a way a test can catch lives in
``forwarder.py``/``protocol.py``/``timebase.py``; this file only opens a
serial port, reads a clock, and writes a UDP socket, so it stays small
enough to review by eye. It is also the only module that imports
``pyserial``.

Timestamps come from ``time.time()`` (CLOCK_REALTIME), NOT
``time.monotonic()``: PREP-D-04 disciplines the Pi's realtime clock to the
shore master with chrony, and the shore side needs stamps it can compare
against the sonar and ArduSub streams. The uniform-timeline reconstruction
in ``timebase.py`` removes the transport jitter; chrony removes the
offset. A monotonic clock would remove the wrong one.
"""
from __future__ import annotations

import argparse
import socket
import time
from typing import List, Optional

from uw_wit_imu import registers
from uw_wit_imu.forwarder import ImuForwarder


def run(
    port: str,
    baud: int,
    dest_host: str,
    dest_port: int,
    rate_hz: float,
    sensor_id: str,
    sensor_frame: str,
    read_size: int = 256,
    max_seconds: Optional[float] = None,
) -> int:
    import serial

    forwarder = ImuForwarder(nominal_rate_hz=rate_hz, sensor_id=sensor_id, sensor_frame=sensor_frame)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    handle = serial.Serial(port, baud, timeout=0.05)
    started = time.monotonic()
    try:
        while max_seconds is None or time.monotonic() - started < max_seconds:
            chunk = handle.read(read_size)
            now = time.time()
            readings = forwarder.feed(chunk, now) if chunk else []
            for datagram in forwarder.datagrams_for(readings, now):
                sock.sendto(datagram, (dest_host, dest_port))
    except KeyboardInterrupt:
        pass
    finally:
        handle.close()
        sock.close()
    return 0


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=registers.TARGET_BAUD_RATE)
    parser.add_argument("--dest-host", default="192.168.2.1", help="shore host (BlueOS default topside address)")
    parser.add_argument("--dest-port", type=int, default=27500)
    parser.add_argument("--rate", type=float, default=registers.TARGET_OUTPUT_RATE_HZ)
    parser.add_argument("--sensor-id", default="hwt9053")
    parser.add_argument("--sensor-frame", default="imu_link")
    parser.add_argument("--max-seconds", type=float, default=None)
    args = parser.parse_args(argv)
    return run(args.port, args.baud, args.dest_host, args.dest_port, args.rate,
               args.sensor_id, args.sensor_frame, max_seconds=args.max_seconds)


if __name__ == "__main__":
    raise SystemExit(main())
