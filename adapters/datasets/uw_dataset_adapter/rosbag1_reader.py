"""Minimal, dependency-free ROS1 bag v2.0 reader, hand-rolled instead of
using the `rosbags` package.

Why hand-rolled: `rosbags.rosbag1.Reader.open()` unconditionally seeks to
the bag's declared `index_pos` (a consolidated CONNECTION+CHUNK_INFO index
written once, at close time, at the end of the file — see
http://wiki.ros.org/Bags/Format/2.0) and raises `ReaderError` if that
region is missing or damaged. That makes it unusable for reading a
TRUNCATED bag file — which is exactly what this adapter needs: public
mirrors of the EuRoC MAV dataset's ROS bags run into the GB range, and
downloading one in full just to convert a short public-dataset smoke test
into this repo's canonical schema isn't worth the bandwidth/time. This
reader instead walks the bag SEQUENTIALLY from the start and never touches
the trailing index:

  - Each CHUNK record embeds its own CONNECTION records (redundant copies
    of the index's connection table, written once per chunk the first time
    a connection is used within it — confirmed empirically against a real
    EuRoC bag, not assumed from the format spec alone) followed by MSGDATA
    records for the messages in that chunk. That's everything needed to
    decode messages without ever reading the trailing index.
  - IDXDATA records (per-connection message offsets within a chunk, written
    once per chunk right after it) are skipped — they exist purely to make
    the FINAL random-access index cheap to rebuild; sequential reading
    doesn't need them.
  - Reaching a truncated/incomplete record (a short read) or the file's
    actual end just stops iteration — whatever complete chunks were parsed
    before that point are returned. This is a feature (works on a
    partial-download prefix of a bag), not degraded-mode error handling.

Only decodes the two message types this adapter's converters need
(sensor_msgs/Image; add more `_parse_*` functions here if a future
converter needs e.g. sensor_msgs/Imu or geometry_msgs/*Stamped — this file
intentionally does NOT pull in ROS message-definition/md5sum machinery like
`rosbags.typesys`, since hand-parsing two known wire layouts is far less
code and far fewer moving parts than a general .msg deserializer).
"""
from __future__ import annotations

import bz2
import dataclasses
import io
import struct
from typing import BinaryIO, Callable, Dict, Iterator, Optional, Tuple

_RECORDTYPE_CHUNK = 5
_RECORDTYPE_CONNECTION = 7
_RECORDTYPE_MSGDATA = 2
_RECORDTYPE_IDXDATA = 4
_RECORDTYPE_BAGHEADER = 3


def _read_header(f: BinaryIO) -> Optional[Dict[str, bytes]]:
    """Reads one ROS1 bag record header (a length-prefixed block of
    length-prefixed `name=value` fields). Returns None on a short/failed
    read — the signal this reader uses to stop (see module docstring)."""
    length_bytes = f.read(4)
    if len(length_bytes) < 4:
        return None
    (length,) = struct.unpack("<I", length_bytes)
    raw = f.read(length)
    if len(raw) < length:
        return None
    fields: Dict[str, bytes] = {}
    pos = 0
    while pos < len(raw):
        if pos + 4 > len(raw):
            return None
        (field_len,) = struct.unpack_from("<I", raw, pos)
        pos += 4
        if pos + field_len > len(raw):
            return None
        field = raw[pos : pos + field_len]
        pos += field_len
        name, sep, value = field.partition(b"=")
        if not sep:
            return None
        fields[name.decode()] = value
    return fields


def _read_data(f: BinaryIO) -> Optional[bytes]:
    """Reads one length-prefixed data block. Returns None on a short read."""
    length_bytes = f.read(4)
    if len(length_bytes) < 4:
        return None
    (length,) = struct.unpack("<I", length_bytes)
    data = f.read(length)
    if len(data) < length:
        return None
    return data


@dataclasses.dataclass(frozen=True)
class ImageMessage:
    stamp_ns: int
    frame_id: str
    height: int
    width: int
    encoding: str
    step: int
    data: bytes


def parse_image_message(raw: bytes) -> ImageMessage:
    """Decodes a sensor_msgs/Image wire payload (ROS1 serialization:
    std_msgs/Header {seq uint32, stamp {secs uint32, nsecs uint32},
    frame_id string} + height uint32 + width uint32 + encoding string +
    is_bigendian uint8 + step uint32 + data uint8[])."""
    pos = 0
    _seq, secs, nsecs = struct.unpack_from("<III", raw, pos)
    pos += 12
    (frame_id_len,) = struct.unpack_from("<I", raw, pos)
    pos += 4
    frame_id = raw[pos : pos + frame_id_len].decode()
    pos += frame_id_len
    height, width = struct.unpack_from("<II", raw, pos)
    pos += 8
    (encoding_len,) = struct.unpack_from("<I", raw, pos)
    pos += 4
    encoding = raw[pos : pos + encoding_len].decode()
    pos += encoding_len
    pos += 1  # is_bigendian (uint8) — unused, EuRoC/ROS1 images are little-endian mono8
    (step,) = struct.unpack_from("<I", raw, pos)
    pos += 4
    (data_len,) = struct.unpack_from("<I", raw, pos)
    pos += 4
    data = raw[pos : pos + data_len]
    return ImageMessage(
        stamp_ns=secs * 1_000_000_000 + nsecs,
        frame_id=frame_id,
        height=height,
        width=width,
        encoding=encoding,
        step=step,
        data=data,
    )


# One decoded message, tagged with which topic/type it came from — the
# caller (a converter) filters by topic itself; this reader stays
# generic across message types rather than special-casing "camera topics."
@dataclasses.dataclass(frozen=True)
class RawMessage:
    topic: str
    msgtype: str
    conn_id: int
    raw: bytes


def iter_messages(path: str) -> Iterator[RawMessage]:
    """Sequentially decodes every MSGDATA record in the bag at `path`,
    yielding one RawMessage per message, in file order — NOT necessarily
    global timestamp order across topics (that's the trailing index's job,
    which this reader deliberately never reads; see module docstring).
    Stops cleanly at the first truncated/short record, which is the normal
    case for a partial-download bag prefix."""
    with open(path, "rb") as f:
        magic = f.readline()
        if not magic.startswith(b"#ROSBAG V2.0"):
            raise ValueError(f"unsupported bag magic: {magic!r}")

        bagheader = _read_header(f)
        if bagheader is None or bagheader.get("op", b"\x00")[0] != _RECORDTYPE_BAGHEADER:
            raise ValueError("missing/invalid BAGHEADER record")
        if _read_data(f) is None:  # BAGHEADER's padding data block
            return

        connections: Dict[int, Tuple[str, str]] = {}

        while True:
            header = _read_header(f)
            if header is None:
                return  # EOF or truncated — stop cleanly, see module docstring
            op = header["op"][0]

            if op == _RECORDTYPE_CHUNK:
                compression = header.get("compression", b"none").decode()
                chunk_data = _read_data(f)
                if chunk_data is None:
                    return  # truncated chunk — stop cleanly
                if compression == "bz2":
                    raw_chunk = bz2.decompress(chunk_data)
                elif compression == "none":
                    raw_chunk = chunk_data
                else:
                    raise ValueError(f"unsupported chunk compression: {compression!r}")

                chunk_file = io.BytesIO(raw_chunk)
                while True:
                    inner_header = _read_header(chunk_file)
                    if inner_header is None:
                        break  # end of this chunk's embedded records
                    inner_op = inner_header["op"][0]
                    if inner_op == _RECORDTYPE_CONNECTION:
                        conn_id = struct.unpack("<I", inner_header["conn"])[0]
                        topic = inner_header["topic"].decode()
                        conn_data = _read_data(chunk_file)
                        if conn_data is None:
                            return
                        msgtype = "?"
                        ipos = 0
                        while ipos + 4 <= len(conn_data):
                            (flen,) = struct.unpack_from("<I", conn_data, ipos)
                            ipos += 4
                            field = conn_data[ipos : ipos + flen]
                            ipos += flen
                            name, sep, value = field.partition(b"=")
                            if sep and name == b"type":
                                msgtype = value.decode()
                        connections[conn_id] = (topic, msgtype)
                    elif inner_op == _RECORDTYPE_MSGDATA:
                        conn_id = struct.unpack("<I", inner_header["conn"])[0]
                        msg_data = _read_data(chunk_file)
                        if msg_data is None:
                            return
                        topic, msgtype = connections.get(conn_id, ("?", "?"))
                        yield RawMessage(topic=topic, msgtype=msgtype, conn_id=conn_id, raw=msg_data)
                    else:
                        break  # unexpected nested record type — stop this chunk
            elif op == _RECORDTYPE_IDXDATA:
                if _read_data(f) is None:
                    return
            else:
                # Reached the trailing CONNECTION/CHUNK_INFO index section
                # (or something unexpected) — by design, this reader never
                # parses that; stop here with everything decoded so far.
                return
