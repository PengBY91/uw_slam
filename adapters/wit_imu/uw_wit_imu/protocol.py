"""WIT-Motion HWT9053-485 wire protocol: continuous-output packet parsing
and configuration-register framing (PREP-D-01 / PREP-D-02 step 1).

Two separate protocols live on the same RS-485 link and both are here:

1. **Continuous output** — the device free-runs, emitting fixed 11-byte
   packets: ``0x55 <type> <8 data bytes> <checksum>``, checksum = low byte
   of the sum of the preceding 10. Each type carries four little-endian
   signed 16-bit words. This module only decodes the four types PREP-D-01
   configures the device to emit (acceleration, angular velocity, magnetic
   field, quaternion); the Euler-angle packet is deliberately turned off
   (CLAUDE.md: no Euler angles anywhere in this repo) and is recognised
   here only so the parser can skip it without desynchronising if a device
   was not configured by ``configure.py``.
2. **Register access** — host-initiated ``0xFF 0xAA <reg> <lo> <hi>``
   writes, used once at commissioning time to set baud rate, output rate
   and output content.

Byte-level only: no serial port, no threading, no timestamps. That keeps
every rule below testable against synthetic byte streams on a machine with
no hardware attached, which is the entire point of doing this work before
the ROV arrives.

.. warning::
   The register numbers and the rate/baud encodings in ``registers`` are
   the WIT standard map (shared across the WT901/HWT9053 family) and have
   NOT been checked against a physical HWT9053-485 or against the exact
   manual revision shipped with the contract unit. ``configure.py``
   therefore defaults to a dry run that prints the frames it would write.
   Confirm every value against the manual (and record its revision in
   ``MANUAL_REVISION`` below) before running it for real.
"""
from __future__ import annotations

import dataclasses
import enum
import struct
from typing import Iterator, List, Optional, Sequence

# Manual revision the constants below were transcribed against. "" means
# "not yet verified against the shipped manual" — configure.py refuses to
# leave dry-run mode while this is empty.
MANUAL_REVISION = ""

PACKET_LENGTH = 11
HEADER_BYTE = 0x55

# The device reports acceleration in units of g and the manual writes
# g = 9.8 m/s^2. This repo uses the SI standard value everywhere else
# (uw::sensor_models gravity_mps2 defaults to 9.80665), and the 0.07%
# difference is far below the part's own scale-factor error, so the SI
# value is used here too and the residual scale error is left for the
# accelerometer scale/bias calibration in PREP-D-05 to absorb.
GRAVITY_MPS2 = 9.80665
ACCEL_FULL_SCALE_G = 16.0
GYRO_FULL_SCALE_DPS = 2000.0
_INT16_FULL_SCALE = 32768.0
_DEG_TO_RAD = 3.141592653589793 / 180.0


class PacketType(enum.IntEnum):
    ACCELERATION = 0x51
    ANGULAR_VELOCITY = 0x52
    EULER_ANGLE = 0x53  # disabled by configure.py; parsed only to stay in sync
    MAGNETIC_FIELD = 0x54
    QUATERNION = 0x59


# The four PREP-D-01 turns on. A forwarder that never sees one of these
# after configuration is a configuration failure, not a link failure.
ENABLED_PACKET_TYPES = (
    PacketType.ACCELERATION,
    PacketType.ANGULAR_VELOCITY,
    PacketType.MAGNETIC_FIELD,
    PacketType.QUATERNION,
)


@dataclasses.dataclass(frozen=True)
class Packet:
    """One decoded 11-byte packet.

    ``values`` is in the packet type's own physical units:
      * ACCELERATION      m/s^2, 3 entries (sensor frame, includes gravity)
      * ANGULAR_VELOCITY  rad/s, 3 entries
      * MAGNETIC_FIELD    raw counts, 3 entries (PREP-D-02 keeps these raw:
                          the magnetometer is only used by PREP-B-02's
                          heading gate, which calibrates its own scale)
      * QUATERNION        unitless, 4 entries ordered [w, x, y, z]
      * EULER_ANGLE       degrees, 3 entries (never consumed)
    ``temperature_c`` is present for every type except QUATERNION, whose
    fourth word is a quaternion component rather than a temperature.
    """

    type: PacketType
    values: Sequence[float]
    temperature_c: Optional[float] = None
    raw: bytes = b""


def checksum(packet: bytes) -> int:
    """Low byte of the sum of the first 10 bytes."""
    return sum(packet[:10]) & 0xFF


def _words(packet: bytes) -> List[int]:
    return list(struct.unpack("<hhhh", packet[2:10]))


def decode(packet: bytes) -> Optional[Packet]:
    """Decodes one complete, checksum-valid 11-byte packet.

    Returns None for a wrong length, a bad header, a bad checksum, or an
    unrecognised type byte — the caller (``iter_packets``) treats all four
    the same way: not a packet, resynchronise.
    """
    if len(packet) != PACKET_LENGTH:
        return None
    if packet[0] != HEADER_BYTE:
        return None
    if packet[10] != checksum(packet):
        return None
    try:
        packet_type = PacketType(packet[1])
    except ValueError:
        return None

    words = _words(packet)
    if packet_type is PacketType.ACCELERATION:
        scale = ACCEL_FULL_SCALE_G * GRAVITY_MPS2 / _INT16_FULL_SCALE
        return Packet(packet_type, [w * scale for w in words[:3]], words[3] / 100.0, packet)
    if packet_type is PacketType.ANGULAR_VELOCITY:
        scale = GYRO_FULL_SCALE_DPS * _DEG_TO_RAD / _INT16_FULL_SCALE
        return Packet(packet_type, [w * scale for w in words[:3]], words[3] / 100.0, packet)
    if packet_type is PacketType.MAGNETIC_FIELD:
        return Packet(packet_type, [float(w) for w in words[:3]], words[3] / 100.0, packet)
    if packet_type is PacketType.EULER_ANGLE:
        scale = 180.0 / _INT16_FULL_SCALE
        return Packet(packet_type, [w * scale for w in words[:3]], None, packet)
    # QUATERNION: all four words are components, ordered q0..q3 = w,x,y,z.
    return Packet(packet_type, [w / _INT16_FULL_SCALE for w in words], None, packet)


@dataclasses.dataclass
class ParseCounters:
    packets_decoded: int = 0
    checksum_failures: int = 0
    resync_bytes_discarded: int = 0


class PacketStream:
    """Resynchronising byte-stream framer.

    RS-485 over a tether loses and corrupts bytes; a framer that assumed
    packets arrive whole and aligned would desynchronise permanently after
    the first dropped byte. This one holds a byte buffer, scans forward to
    the next plausible header, and validates the checksum before accepting
    anything — so a corrupt packet costs exactly one packet, and a
    ``0x55`` that happens to appear inside a data field is rejected by the
    checksum and skipped one byte at a time.

    Counters are cumulative and are what the forwarder's HealthReport
    (PREP-D-02 step 3) reports.
    """

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.counters = ParseCounters()

    def feed(self, chunk: bytes) -> List[Packet]:
        self._buffer.extend(chunk)
        return list(self._drain())

    def _drain(self) -> Iterator[Packet]:
        while True:
            start = self._buffer.find(HEADER_BYTE)
            if start < 0:
                # No header at all: the whole buffer is garbage, but keep
                # nothing — a header cannot span the discard.
                self.counters.resync_bytes_discarded += len(self._buffer)
                self._buffer.clear()
                return
            if start > 0:
                self.counters.resync_bytes_discarded += start
                del self._buffer[:start]
            if len(self._buffer) < PACKET_LENGTH:
                return  # wait for more bytes
            candidate = bytes(self._buffer[:PACKET_LENGTH])
            decoded = decode(candidate)
            if decoded is None:
                # Not a real packet after all. Drop just the header byte so
                # a genuine packet starting one byte later is still found.
                if candidate[0] == HEADER_BYTE and candidate[10] != checksum(candidate):
                    self.counters.checksum_failures += 1
                self.counters.resync_bytes_discarded += 1
                del self._buffer[:1]
                continue
            del self._buffer[:PACKET_LENGTH]
            self.counters.packets_decoded += 1
            yield decoded

    @property
    def buffered_bytes(self) -> int:
        return len(self._buffer)
