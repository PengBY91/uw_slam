"""Wire protocol for `RawSensorFrame`/thruster commands across the process
boundary this repo's real-HoloOcean validation work needs to bridge: real
HoloOcean only runs on native Windows (no Vulkan ray-tracing support in this
WSL2 sandbox -- see the `project-holoocean-deployment` memory record), while
the ROS2 realtime gateway (`holoocean_realtime_node`, C++) and this
package's rclpy-dependent publishing code only build/run in WSL2 Linux.
Rather than installing ROS2/rclpy on Windows or bridging cross-host DDS
discovery (both unexplored, both possibly substantial efforts on their own),
`holoocean_bridge_sensor_host.py` (Windows side, owns the real
`HoloOceanSession`) and `bridged_realtime_ros_session.py` (WSL2 side, owns
rclpy + `build_realtime_messages`) exchange raw sensor frames and thruster
commands over one small, purpose-built TCP protocol defined here -- not a
ROS2/DDS bridge, no rclpy or HoloOcean import needed by this module itself.

Each array is framed explicitly as (name, dtype string, shape, raw bytes)
rather than via `numpy.save`/`pickle` -- this bridge spans two different
Python versions (Windows 3.11, this repo's WSL2 venv on 3.13), and pickling
a `RawSensorFrame` dataclass instance would also require the receiving end
to have an identically-importable class, which the Windows-side copy (a
flat directory, not a full checkout -- see the memory record's "Reusable
verification harness" section) does not guarantee.
"""
from __future__ import annotations

import json
import socket
import struct
from typing import List

import numpy as np

from uw_holoocean_adapter.holoocean_driver import RawSensorFrame

_BLOB_LENGTH_PREFIX_FMT = ">I"
_HEADER_LENGTH_PREFIX_FMT = ">I"

THRUSTER_COUNT = 8
_THRUSTER_COMMAND_NBYTES = THRUSTER_COUNT * 8  # float64


def _send_exact(sock: socket.socket, data: bytes) -> None:
    sock.sendall(data)


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    """Reads exactly `n` bytes or raises `ConnectionError` -- a single
    `socket.recv(n)` call may return fewer bytes than requested even on a
    still-open connection, so it is not by itself a correct read loop."""
    chunks: List[bytes] = []
    remaining = n
    while remaining > 0:
        chunk = sock.recv(remaining)
        if not chunk:
            raise ConnectionError("connection closed while expecting more data")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def encode_raw_sensor_frame(frame: RawSensorFrame) -> bytes:
    """Pure, socket-free encode -- returns one self-describing blob (a
    length-prefixed JSON header naming each sensor's dtype/shape, followed
    by every array's raw bytes concatenated in the same order) that
    `decode_raw_sensor_frame` can consume without any additional framing."""
    arrays = [(name, np.asarray(value)) for name, value in frame.sensors.items()]
    header = {
        "sim_time_s": frame.sim_time_s,
        "receive_time_s": frame.receive_time_s,
        "sensors": [{"name": name, "dtype": str(arr.dtype), "shape": list(arr.shape)} for name, arr in arrays],
    }
    header_bytes = json.dumps(header).encode("utf-8")
    body = b"".join(arr.tobytes() for _, arr in arrays)
    return struct.pack(_HEADER_LENGTH_PREFIX_FMT, len(header_bytes)) + header_bytes + body


def decode_raw_sensor_frame(blob: bytes) -> RawSensorFrame:
    (header_len,) = struct.unpack_from(_HEADER_LENGTH_PREFIX_FMT, blob, 0)
    offset = struct.calcsize(_HEADER_LENGTH_PREFIX_FMT)
    header = json.loads(blob[offset : offset + header_len].decode("utf-8"))
    offset += header_len
    sensors = {}
    for entry in header["sensors"]:
        dtype = np.dtype(entry["dtype"])
        shape = tuple(entry["shape"])
        count = int(np.prod(shape)) if shape else 1
        nbytes = count * dtype.itemsize
        # .copy() -- np.frombuffer is a read-only view into `blob`; callers
        # downstream (camera/sonar conversion) must get an owned array, not
        # one that aliases this function's own local bytes object.
        sensors[entry["name"]] = np.frombuffer(blob[offset : offset + nbytes], dtype=dtype).reshape(shape).copy()
        offset += nbytes
    return RawSensorFrame(sim_time_s=header["sim_time_s"], receive_time_s=header["receive_time_s"], sensors=sensors)


def send_raw_sensor_frame(sock: socket.socket, frame: RawSensorFrame) -> None:
    blob = encode_raw_sensor_frame(frame)
    _send_exact(sock, struct.pack(_BLOB_LENGTH_PREFIX_FMT, len(blob)) + blob)


def recv_raw_sensor_frame(sock: socket.socket) -> RawSensorFrame:
    (blob_len,) = struct.unpack(_BLOB_LENGTH_PREFIX_FMT, _recv_exact(sock, struct.calcsize(_BLOB_LENGTH_PREFIX_FMT)))
    return decode_raw_sensor_frame(_recv_exact(sock, blob_len))


def encode_thruster_command(values) -> bytes:
    arr = np.asarray(values, dtype=np.float64)
    if arr.shape != (THRUSTER_COUNT,):
        raise ValueError(f"thruster command must have shape ({THRUSTER_COUNT},), got {arr.shape}")
    return arr.tobytes()


def decode_thruster_command(data: bytes) -> List[float]:
    return np.frombuffer(data, dtype=np.float64).tolist()


def send_thruster_command(sock: socket.socket, values) -> None:
    _send_exact(sock, encode_thruster_command(values))


def recv_thruster_command(sock: socket.socket) -> List[float]:
    return decode_thruster_command(_recv_exact(sock, _THRUSTER_COMMAND_NBYTES))
