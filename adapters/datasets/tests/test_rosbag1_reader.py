"""Unit tests for rosbag1_reader.py, built against a small hand-crafted
ROS1 bag v2.0 file (not a downloaded fixture — see this module's helper
`_write_minimal_bag`) so these tests run offline and don't depend on
network access or a real EuRoC download."""
from __future__ import annotations

import struct
import tempfile
from pathlib import Path

from uw_dataset_adapter.rosbag1_reader import iter_messages, parse_image_message


def _header_block(fields: dict) -> bytes:
    out = b""
    for name, value in fields.items():
        field = name.encode() + b"=" + value
        out += struct.pack("<I", len(field)) + field
    return struct.pack("<I", len(out)) + out


def _data_block(data: bytes) -> bytes:
    return struct.pack("<I", len(data)) + data


def _connection_record(conn_id: int, topic: str, msgtype: str) -> bytes:
    header = _header_block({"op": bytes([7]), "conn": struct.pack("<I", conn_id), "topic": topic.encode()})
    data = _header_block({"type": msgtype.encode()})
    # data block's payload is itself a header-field-encoded blob (matches
    # the real format: the CONNECTION record's data is a nested header).
    inner = data[4:]  # strip the outer 4-byte length _header_block added, re-wrap below
    return header + _data_block(inner)


def _image_wire_bytes(width: int, height: int, encoding: bytes, pixel_data: bytes) -> bytes:
    out = struct.pack("<III", 0, 0, 0)  # seq, stamp secs, nsecs
    frame_id = b"cam0"
    out += struct.pack("<I", len(frame_id)) + frame_id
    out += struct.pack("<II", height, width)
    out += struct.pack("<I", len(encoding)) + encoding
    out += struct.pack("<B", 0)  # is_bigendian
    out += struct.pack("<I", width)  # step
    out += struct.pack("<I", len(pixel_data)) + pixel_data
    return out


def _msgdata_record(conn_id: int, payload: bytes) -> bytes:
    header = _header_block({"op": bytes([2]), "conn": struct.pack("<I", conn_id), "time": struct.pack("<II", 0, 0)})
    return header + _data_block(payload)


def _write_minimal_bag(path: Path, num_messages: int, truncate_last: bool = False) -> None:
    """Writes ONE chunk (uncompressed) containing a CONNECTION record for
    '/cam0/image_raw' followed by `num_messages` MSGDATA image records —
    enough structure to exercise iter_messages' chunk-parsing path without
    needing bz2 or a real multi-chunk bag."""
    pixel_data = bytes([42] * (4 * 3))  # 4x3 mono8 image
    chunk_body = _connection_record(0, "/cam0/image_raw", "sensor_msgs/Image")
    for _ in range(num_messages):
        chunk_body += _msgdata_record(0, _image_wire_bytes(4, 3, b"mono8", pixel_data))

    chunk_header = _header_block({"op": bytes([5]), "compression": b"none"})
    chunk_record = chunk_header + _data_block(chunk_body)

    bagheader = _header_block(
        {
            "op": bytes([3]),
            "index_pos": struct.pack("<Q", 0),
            "conn_count": struct.pack("<I", 1),
            "chunk_count": struct.pack("<I", 1),
        }
    )
    bagheader_data = _data_block(b"\x00" * 4000)  # padding, matches real bags' large BAGHEADER data block

    with open(path, "wb") as f:
        f.write(b"#ROSBAG V2.0\n")
        f.write(bagheader)
        f.write(bagheader_data)
        if truncate_last:
            # Cut off partway through the chunk record's data block, like a
            # partial download would.
            f.write(chunk_record[: len(chunk_record) // 2])
        else:
            f.write(chunk_record)


def test_iter_messages_decodes_all_messages_in_one_chunk():
    with tempfile.TemporaryDirectory() as tmp:
        bag_path = Path(tmp) / "test.bag"
        _write_minimal_bag(bag_path, num_messages=5)

        messages = list(iter_messages(str(bag_path)))
        assert len(messages) == 5
        for m in messages:
            assert m.topic == "/cam0/image_raw"
            assert m.msgtype == "sensor_msgs/Image"
            img = parse_image_message(m.raw)
            assert img.width == 4
            assert img.height == 3
            assert img.encoding == "mono8"
            assert img.data == bytes([42] * 12)


def test_iter_messages_stops_cleanly_on_truncated_chunk():
    with tempfile.TemporaryDirectory() as tmp:
        bag_path = Path(tmp) / "truncated.bag"
        _write_minimal_bag(bag_path, num_messages=5, truncate_last=True)

        # Must not raise — a truncated chunk (partial download) should just
        # yield whatever was decodable before the cut, per this reader's
        # documented contract (see rosbag1_reader.py's module docstring).
        messages = list(iter_messages(str(bag_path)))
        assert len(messages) == 0  # the whole chunk was cut before any MSGDATA was reached


def test_parse_image_message_round_trips_known_bytes():
    pixel_data = bytes(range(6))
    raw = _image_wire_bytes(3, 2, b"mono8", pixel_data)
    img = parse_image_message(raw)
    assert img.width == 3
    assert img.height == 2
    assert img.encoding == "mono8"
    assert img.step == 3
    assert img.data == pixel_data
