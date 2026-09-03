"""Serial -> parse -> uniform timestamp -> UDP protobuf forwarder
(PREP-D-02): the service that runs on the ROV's Raspberry Pi as a BlueOS
extension and puts the HWT9053-485 stream on the tether.

The design rule here is the same one the C++ frontends follow: all the
logic that can be tested without hardware IS separated from the I/O that
cannot. ``SampleAssembler`` and ``ImuForwarder`` take bytes and an arrival
time and hand back datagrams; ``service.py`` is the thin loop that reads a
serial port, calls the clock, and writes a socket. Every rule below is
therefore covered by tests running against synthetic byte streams on a
machine with no IMU attached, which is what PREP-D-02 asks for ("用 PC 上
的 USB-485 先验证" is the step after this one, not before it).

Wire format on the tether, one datagram per message::

    b"UWIM" | version u8 | message type u8 | payload length u32 BE | payload

The magic + version exist because this is UDP: the shore adapter
(PREP-D-03) has no connection state to tell it whether the first datagram
after a restart is an ImuSample or a HealthReport, and a bare protobuf
payload is not self-describing. The length prefix is redundant for UDP
but makes the exact same framing replayable over a file or a TCP socket,
which is how the offline tests and PREP-A-13's device-emulation stream
consume it.
"""
from __future__ import annotations

import dataclasses
import pathlib
import struct
import sys
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from uw_wit_imu import protocol, registers
from uw_wit_imu.timebase import UniformTimebase

WIRE_MAGIC = b"UWIM"
WIRE_VERSION = 1
MESSAGE_TYPE_IMU_SAMPLE = 1
MESSAGE_TYPE_HEALTH_REPORT = 2
_WIRE_HEADER = struct.Struct(">4sBBI")

COMPONENT_ID = "hwt9053_forwarder"
DEFAULT_SENSOR_ID = "hwt9053"
DEFAULT_SENSOR_FRAME = "imu_link"


def _bootstrap_schema_path() -> None:
    """Generated schema_pb2/ sits alongside this file (see
    tools/codegen/gen_py.sh) but isn't on sys.path by default — same
    pattern as adapters/datasets/uw_dataset_adapter/euroc_converter.py's
    identically-named helper."""
    schema_dir = pathlib.Path(__file__).parent / "schema_pb2"
    if schema_dir.is_dir() and str(schema_dir) not in sys.path:
        sys.path.insert(0, str(schema_dir))


def encode_datagram(message_type: int, payload: bytes) -> bytes:
    return _WIRE_HEADER.pack(WIRE_MAGIC, WIRE_VERSION, message_type, len(payload)) + payload


def decode_datagram(datagram: bytes) -> Tuple[int, bytes]:
    """Returns (message_type, payload). Raises ValueError on anything
    that is not a well-formed datagram of a version we understand."""
    if len(datagram) < _WIRE_HEADER.size:
        raise ValueError("datagram shorter than header")
    magic, version, message_type, length = _WIRE_HEADER.unpack(datagram[: _WIRE_HEADER.size])
    if magic != WIRE_MAGIC:
        raise ValueError(f"bad magic: {magic!r}")
    if version != WIRE_VERSION:
        raise ValueError(f"unsupported wire version: {version}")
    payload = datagram[_WIRE_HEADER.size :]
    if len(payload) != length:
        raise ValueError(f"payload length mismatch: header says {length}, got {len(payload)}")
    return message_type, payload


@dataclasses.dataclass
class ImuReading:
    """One complete device cycle, in canonical units.

    ``capture_time_s`` is the reconstructed uniform timestamp (what goes on
    the wire as ObservationHeader.capture_time); ``receive_time_s`` is the
    raw arrival time of the cycle's first packet, kept so the shore side
    can audit the reconstruction rather than having to trust it.
    """

    sequence: int
    capture_time_s: float
    receive_time_s: float
    linear_acceleration_mps2: Tuple[float, float, float]
    angular_velocity_radps: Tuple[float, float, float]
    magnetic_field_counts: Optional[Tuple[float, float, float]] = None
    quaternion_wxyz: Optional[Tuple[float, float, float, float]] = None
    temperature_c: Optional[float] = None


@dataclasses.dataclass
class AssemblerCounters:
    cycles_emitted: int = 0
    cycles_incomplete: int = 0  # a duplicate type arrived before the set was complete


class SampleAssembler:
    """Groups the per-cycle packet burst into one reading.

    The device emits its enabled packet types back to back once per cycle,
    with no cycle delimiter. The grouping rule is therefore: collect types
    until every expected type has been seen (emit), or until a type
    repeats (a packet in this cycle was lost — discard the partial group
    and start a new one from the repeated packet). Never emit a cycle
    missing its accelerometer or gyro reading: a half-filled ImuSample
    would be indistinguishable downstream from a real zero reading.
    """

    def __init__(self, expected_types: Sequence[protocol.PacketType] = protocol.ENABLED_PACKET_TYPES) -> None:
        required = {protocol.PacketType.ACCELERATION, protocol.PacketType.ANGULAR_VELOCITY}
        if not required.issubset(set(expected_types)):
            raise ValueError("expected_types must include acceleration and angular velocity")
        self._expected = tuple(expected_types)
        self._pending: Dict[protocol.PacketType, protocol.Packet] = {}
        self._first_arrival: Optional[float] = None
        self.counters = AssemblerCounters()

    def feed(self, packet: protocol.Packet, arrival_time_s: float) -> Optional[Dict[protocol.PacketType, protocol.Packet]]:
        if packet.type not in self._expected:
            return None  # e.g. a stale Euler packet from an unconfigured unit
        if packet.type in self._pending:
            self.counters.cycles_incomplete += 1
            self._pending.clear()
            self._first_arrival = None
        if not self._pending:
            self._first_arrival = arrival_time_s
        self._pending[packet.type] = packet
        if len(self._pending) < len(self._expected):
            return None
        group = dict(self._pending)
        self._pending.clear()
        self._first_arrival = None
        self.counters.cycles_emitted += 1
        return group

    @property
    def pending_arrival_time_s(self) -> Optional[float]:
        return self._first_arrival


@dataclasses.dataclass
class ForwarderCounters:
    readings: int = 0
    datagrams_sent: int = 0
    health_reports: int = 0


class ImuForwarder:
    """Bytes in, datagrams out. Holds no socket and reads no clock."""

    def __init__(
        self,
        nominal_rate_hz: float = registers.TARGET_OUTPUT_RATE_HZ,
        expected_types: Sequence[protocol.PacketType] = protocol.ENABLED_PACKET_TYPES,
        sensor_id: str = DEFAULT_SENSOR_ID,
        sensor_frame: str = DEFAULT_SENSOR_FRAME,
        health_interval_s: float = 1.0,
    ) -> None:
        self._stream = protocol.PacketStream()
        self._assembler = SampleAssembler(expected_types)
        self._timebase = UniformTimebase(nominal_rate_hz)
        self._sensor_id = sensor_id
        self._sensor_frame = sensor_frame
        self._health_interval_s = health_interval_s
        self._nominal_rate_hz = nominal_rate_hz
        self._sequence = 0
        self._last_health_time_s: Optional[float] = None
        self._readings_since_health = 0
        self.counters = ForwarderCounters()

    # -- accessors the tests and the health report read ------------------
    @property
    def parse_counters(self) -> protocol.ParseCounters:
        return self._stream.counters

    @property
    def assembler_counters(self) -> AssemblerCounters:
        return self._assembler.counters

    @property
    def timebase(self) -> UniformTimebase:
        return self._timebase

    def feed(self, chunk: bytes, arrival_time_s: float) -> List[ImuReading]:
        """Decodes one serial read.

        All packets in a chunk share the read's arrival time. That is
        correct rather than lossy: at 200 Hz a cycle's four packets are 44
        bytes arriving within ~2 ms of each other at 230400 baud, so the
        per-packet arrival spread is well inside the jitter the timebase
        already averages out — and only the cycle's FIRST arrival is fed
        to the timebase anyway.
        """
        readings: List[ImuReading] = []
        for packet in self._stream.feed(chunk):
            group = self._assembler.feed(packet, arrival_time_s)
            if group is None:
                continue
            readings.append(self._make_reading(group, arrival_time_s))
        self.counters.readings += len(readings)
        self._readings_since_health += len(readings)
        return readings

    def _make_reading(
        self, group: Dict[protocol.PacketType, protocol.Packet], arrival_time_s: float
    ) -> ImuReading:
        accel = group[protocol.PacketType.ACCELERATION]
        gyro = group[protocol.PacketType.ANGULAR_VELOCITY]
        mag = group.get(protocol.PacketType.MAGNETIC_FIELD)
        quat = group.get(protocol.PacketType.QUATERNION)
        capture_time_s = self._timebase.update(arrival_time_s)
        reading = ImuReading(
            sequence=self._sequence,
            capture_time_s=capture_time_s,
            receive_time_s=arrival_time_s,
            linear_acceleration_mps2=tuple(accel.values),  # type: ignore[arg-type]
            angular_velocity_radps=tuple(gyro.values),  # type: ignore[arg-type]
            magnetic_field_counts=tuple(mag.values) if mag else None,  # type: ignore[arg-type]
            quaternion_wxyz=tuple(quat.values) if quat else None,  # type: ignore[arg-type]
            temperature_c=accel.temperature_c,
        )
        self._sequence += 1
        return reading

    # -- protobuf encoding ------------------------------------------------
    def encode_imu_sample(self, reading: ImuReading) -> bytes:
        _bootstrap_schema_path()
        from uw.domain import imu_pb2, observation_pb2, time_pb2  # noqa: E402

        sample = imu_pb2.ImuSample()
        header = sample.header
        header.observation_id.value = f"{self._sensor_id}:{reading.sequence}"
        header.sensor_id.value = self._sensor_id
        header.sequence_id.value = reading.sequence
        _fill_stamp(time_pb2, header.capture_time, reading.capture_time_s)
        _fill_stamp(time_pb2, header.receive_time, reading.receive_time_s)
        # Both stamps are on the Pi's CLOCK_REALTIME, which PREP-D-04 keeps
        # chrony-disciplined to the shore master; the reconstruction only
        # removes jitter, it does not change clock domain.
        header.clock_domain = time_pb2.ClockDomain.CLOCK_DOMAIN_SYSTEM_REALTIME
        header.receive_clock_domain = time_pb2.ClockDomain.CLOCK_DOMAIN_SYSTEM_REALTIME
        header.sensor_frame.value = self._sensor_frame
        header.validity = observation_pb2.ObservationHeader.VALIDITY_OK
        header.provenance = (
            f"uw_wit_imu.forwarder;rate_hz={self._nominal_rate_hz:g};"
            f"fitted_rate_hz={self._timebase.estimated_rate_hz:.4f};"
            f"locked={int(self._timebase.locked)}"
        )
        sample.linear_acceleration_mps2.extend(reading.linear_acceleration_mps2)
        sample.angular_velocity_radps.extend(reading.angular_velocity_radps)
        # has_bias stays false: this device reports no internal bias
        # estimate (unlike HoloOcean's IMUSensor ReturnBias option), and
        # PREP-D-05's Allan-variance calibration fills the rig instead.
        sample.has_bias = False
        return sample.SerializeToString()

    def maybe_build_health_report(self, now_s: float, force: bool = False) -> Optional[bytes]:
        """One HealthReport per ``health_interval_s`` (PREP-D-02 step 3)."""
        if self._last_health_time_s is None:
            self._last_health_time_s = now_s
            if not force:
                return None
        elapsed = now_s - self._last_health_time_s
        if not force and elapsed < self._health_interval_s:
            return None
        expected = max(1.0, self._nominal_rate_hz * max(elapsed, 1e-9))
        observed_rate = self._readings_since_health / max(elapsed, 1e-9)
        self._last_health_time_s = now_s
        self._readings_since_health = 0

        _bootstrap_schema_path()
        from uw.domain import health_pb2  # noqa: E402

        report = health_pb2.HealthReport()
        report.component_id = COMPONENT_ID
        parse = self._stream.counters
        assembler = self._assembler.counters
        timebase = self._timebase.counters
        dropped = timebase.samples_missing + assembler.cycles_incomplete
        report.dropped_frame_count = dropped
        report.rejected_frame_count = parse.checksum_failures
        report.input_valid_rate = min(1.0, observed_rate / self._nominal_rate_hz)
        report.queue_depth = self._stream.buffered_bytes

        # Thresholds mirror the PREP-D-02 acceptance criterion (<0.1% loss
        # over 24 h): a full order of magnitude of margin before SUSPECT,
        # and UNAVAILABLE only when the stream is effectively gone.
        if observed_rate < 0.5 * self._nominal_rate_hz:
            report.status = health_pb2.HealthReport.STATUS_UNAVAILABLE
            report.reason_code = "imu_stream_starved"
        elif report.input_valid_rate < 0.99 or parse.checksum_failures > 0:
            report.status = health_pb2.HealthReport.STATUS_SUSPECT
            report.reason_code = "imu_packet_loss"
        elif not self._timebase.locked:
            report.status = health_pb2.HealthReport.STATUS_RECOVERING
            report.reason_code = "timebase_not_locked"
        else:
            report.status = health_pb2.HealthReport.STATUS_HEALTHY
            report.reason_code = ""
        self.counters.health_reports += 1
        return report.SerializeToString()

    def datagrams_for(self, readings: Iterable[ImuReading], now_s: float) -> List[bytes]:
        out = [encode_datagram(MESSAGE_TYPE_IMU_SAMPLE, self.encode_imu_sample(r)) for r in readings]
        health = self.maybe_build_health_report(now_s)
        if health is not None:
            out.append(encode_datagram(MESSAGE_TYPE_HEALTH_REPORT, health))
        self.counters.datagrams_sent += len(out)
        return out


def _fill_stamp(time_pb2, stamp, seconds: float) -> None:
    total_ns = int(round(seconds * 1e9))
    stamp.seconds = total_ns // 1_000_000_000
    stamp.nanos = total_ns % 1_000_000_000
