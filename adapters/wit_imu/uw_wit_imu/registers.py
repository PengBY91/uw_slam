"""HWT9053-485 configuration registers and the frames that write them
(PREP-D-01).

.. warning::
   Transcribed from the WIT standard register map that the WT901/HWT
   family shares, NOT verified against a physical HWT9053-485 or against
   the manual revision shipped with the contract unit — see
   ``protocol.MANUAL_REVISION``. Every constant below is one line so a
   reviewer with the manual open can check them off; ``configure.py``
   stays in dry-run mode until ``MANUAL_REVISION`` is filled in.

Why 230400 baud: at 200 Hz with four packet types enabled the device emits
4 x 11 = 44 bytes per cycle, 8800 B/s, 88000 bits/s with 10 bits per byte
of framing overhead. The factory default 9600 baud carries 960 B/s — under
11% of what is needed — and the device does not flow-control, so it
silently drops packets rather than slowing down. 115200 (11520 B/s) would
technically fit but leaves only 30% headroom; 230400 leaves 160%.
"""
from __future__ import annotations

import struct
from typing import List, Tuple

# --- register addresses ---------------------------------------------------
REG_SAVE = 0x00        # write 0x0000 to persist, 0x0001 to factory reset
REG_OUTPUT_CONTENT = 0x02  # bit mask of which packet types to emit
REG_OUTPUT_RATE = 0x03
REG_BAUD_RATE = 0x04
REG_UNLOCK = 0x69      # must be written before any other register takes effect

UNLOCK_KEY = 0xB588
SAVE_VALUE = 0x0000

# --- output-content bit mask (REG_OUTPUT_CONTENT) -------------------------
# One bit per continuous-output packet type, in the manual's documented
# order. PREP-D-01 enables acceleration + angular velocity + magnetic
# field + quaternion and disables everything else, most importantly the
# Euler-angle packet (CLAUDE.md forbids Euler angles in this repo, and at
# 200 Hz every disabled packet is 11 bytes/cycle of link budget back).
CONTENT_BIT_TIME = 1 << 0
CONTENT_BIT_ACCELERATION = 1 << 1
CONTENT_BIT_ANGULAR_VELOCITY = 1 << 2
CONTENT_BIT_EULER_ANGLE = 1 << 3
CONTENT_BIT_MAGNETIC_FIELD = 1 << 4
CONTENT_BIT_PORT_STATUS = 1 << 5
CONTENT_BIT_PRESSURE_HEIGHT = 1 << 6
CONTENT_BIT_GPS = 1 << 7
CONTENT_BIT_GPS_ACCURACY = 1 << 8
CONTENT_BIT_QUATERNION = 1 << 9

PREP_D01_OUTPUT_CONTENT = (
    CONTENT_BIT_ACCELERATION
    | CONTENT_BIT_ANGULAR_VELOCITY
    | CONTENT_BIT_MAGNETIC_FIELD
    | CONTENT_BIT_QUATERNION
)

# --- encodings ------------------------------------------------------------
OUTPUT_RATE_CODES = {
    0.2: 0x01,
    0.5: 0x02,
    1.0: 0x03,
    2.0: 0x04,
    5.0: 0x05,
    10.0: 0x06,
    20.0: 0x07,
    50.0: 0x08,
    100.0: 0x09,
    125.0: 0x0A,
    200.0: 0x0B,
}

BAUD_RATE_CODES = {
    4800: 0x01,
    9600: 0x02,
    19200: 0x03,
    38400: 0x04,
    57600: 0x05,
    115200: 0x06,
    230400: 0x07,
    460800: 0x08,
    921600: 0x09,
}

# What PREP-D-01 configures the contract unit to.
TARGET_OUTPUT_RATE_HZ = 200.0
TARGET_BAUD_RATE = 230400
FACTORY_BAUD_RATE = 9600
FACTORY_OUTPUT_RATE_HZ = 10.0


def write_frame(register: int, value: int) -> bytes:
    """One register write: ``0xFF 0xAA <reg> <value lo> <value hi>``."""
    if not 0 <= register <= 0xFF:
        raise ValueError(f"register out of range: {register:#x}")
    if not 0 <= value <= 0xFFFF:
        raise ValueError(f"value out of range: {value:#x}")
    return b"\xff\xaa" + bytes([register]) + struct.pack("<H", value)


def configuration_sequence(
    output_rate_hz: float = TARGET_OUTPUT_RATE_HZ,
    baud_rate: int = TARGET_BAUD_RATE,
    output_content: int = PREP_D01_OUTPUT_CONTENT,
) -> List[Tuple[str, bytes]]:
    """The full commissioning sequence, as (description, frame) pairs.

    Order matters and is not arbitrary:

    * ``unlock`` first — every other register write is ignored without it.
    * output content and output rate before the baud rate, because the
      baud-rate write is the last one the host can still send at the OLD
      baud rate; everything after it needs the port reopened at the new
      speed.
    * ``save`` last, at the NEW baud rate, so the settings survive a power
      cycle (the acceptance criterion in PREP-D-01 is explicitly "still
      outputs after re-powering").

    The caller (``configure.py``) is responsible for reopening the serial
    port between the baud-rate frame and the save frame; the split point is
    marked by the ``"baud"`` description.
    """
    if output_rate_hz not in OUTPUT_RATE_CODES:
        raise ValueError(f"unsupported output rate: {output_rate_hz} Hz")
    if baud_rate not in BAUD_RATE_CODES:
        raise ValueError(f"unsupported baud rate: {baud_rate}")
    return [
        ("unlock", write_frame(REG_UNLOCK, UNLOCK_KEY)),
        ("content", write_frame(REG_OUTPUT_CONTENT, output_content)),
        ("rate", write_frame(REG_OUTPUT_RATE, OUTPUT_RATE_CODES[output_rate_hz])),
        ("baud", write_frame(REG_BAUD_RATE, BAUD_RATE_CODES[baud_rate])),
        ("unlock", write_frame(REG_UNLOCK, UNLOCK_KEY)),
        ("save", write_frame(REG_SAVE, SAVE_VALUE)),
    ]


def required_bytes_per_second(output_rate_hz: float, output_content: int) -> float:
    """Link budget check: 11 bytes per enabled packet per cycle."""
    enabled = bin(output_content).count("1")
    return output_rate_hz * enabled * 11.0


def baud_rate_is_sufficient(baud_rate: int, output_rate_hz: float, output_content: int) -> bool:
    """True when the port can carry the configured stream with headroom.

    10 bits per byte (8N1 start/stop framing) and a 1.5x margin — the
    device does not flow-control, so a marginal link drops packets
    silently, which is exactly the failure PREP-D-01 exists to prevent.
    """
    capacity_bytes_per_s = baud_rate / 10.0
    return capacity_bytes_per_s >= 1.5 * required_bytes_per_second(output_rate_hz, output_content)
