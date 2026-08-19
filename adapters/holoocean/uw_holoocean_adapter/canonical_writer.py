"""Python counterpart of runtime/include/uw/runtime/mcap_io.hpp's
McapProtobufWriter — same wire format (protobuf-encoded messages, one MCAP
channel per topic, self-describing schema via a FileDescriptorSet), so a
bag written here is readable by the C++ apps/replay_demo without any
format translation. Compression is disabled (CompressionType.NONE): the
C++ side's mcap SDK integration defines MCAP_COMPRESSION_NO_ZSTD/LZ4 to
avoid pulling in those libraries, so it cannot read zstd/lz4-compressed
chunks — this writer must stay uncompressed to remain cross-language
compatible.
"""
from __future__ import annotations

from collections import deque
from typing import Dict

from google.protobuf import descriptor_pb2
from google.protobuf.message import Message
from mcap.writer import CompressionType, Writer


def build_file_descriptor_set(message: Message) -> bytes:
    """Serializes the transitive closure of .proto file dependencies for
    `message`'s type — same approach as the C++ side's BuildFileDescriptorSet
    in runtime/src/mcap_io.cpp, kept algorithmically identical on purpose."""
    fd_set = descriptor_pb2.FileDescriptorSet()
    visited = set()
    to_visit = deque([message.DESCRIPTOR.file])
    while to_visit:
        file_descriptor = to_visit.popleft()
        if file_descriptor.name in visited:
            continue
        visited.add(file_descriptor.name)
        file_descriptor.CopyToProto(fd_set.file.add())
        to_visit.extend(file_descriptor.dependencies)
    return fd_set.SerializeToString()


class CanonicalMcapWriter:
    def __init__(self, path: str):
        self._file = open(path, "wb")
        self._writer = Writer(self._file, compression=CompressionType.NONE)
        self._writer.start()
        self._schema_id_by_type_name: Dict[str, int] = {}
        self._channel_id_by_topic: Dict[str, int] = {}
        self._sequence_by_channel: Dict[int, int] = {}

    def _ensure_channel(self, topic: str, message: Message) -> int:
        if topic in self._channel_id_by_topic:
            return self._channel_id_by_topic[topic]

        type_name = message.DESCRIPTOR.full_name
        schema_id = self._schema_id_by_type_name.get(type_name)
        if schema_id is None:
            schema_id = self._writer.register_schema(
                name=type_name, encoding="protobuf", data=build_file_descriptor_set(message)
            )
            self._schema_id_by_type_name[type_name] = schema_id

        channel_id = self._writer.register_channel(
            topic=topic, message_encoding="protobuf", schema_id=schema_id
        )
        self._channel_id_by_topic[topic] = channel_id
        self._sequence_by_channel[channel_id] = 0
        return channel_id

    def write_message(self, topic: str, log_time_ns: int, message: Message) -> None:
        channel_id = self._ensure_channel(topic, message)
        sequence = self._sequence_by_channel[channel_id]
        self._sequence_by_channel[channel_id] = sequence + 1
        self._writer.add_message(
            channel_id=channel_id,
            log_time=log_time_ns,
            publish_time=log_time_ns,
            data=message.SerializeToString(),
            sequence=sequence,
        )

    def close(self) -> None:
        self._writer.finish()
        self._file.close()

    def __enter__(self) -> "CanonicalMcapWriter":
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()


def read_canonical_messages(path: str, topic: str, message_factory):
    """Yields (log_time_ns, message) pairs for every message on `topic`.
    `message_factory` is the protobuf message class to parse into (e.g.
    ObservationHeader) — mirrors the C++ side's ReadMcapMessages<T>."""
    from mcap.reader import make_reader

    with open(path, "rb") as f:
        reader = make_reader(f)
        for schema, channel, mcap_message in reader.iter_messages(topics=[topic]):
            del schema  # unused; message type is caller-specified via message_factory
            message = message_factory()
            message.ParseFromString(mcap_message.data)
            yield mcap_message.log_time, message
