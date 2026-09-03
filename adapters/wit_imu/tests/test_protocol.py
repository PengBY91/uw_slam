import math
import struct

import pytest

from uw_wit_imu import protocol


def make_packet(packet_type: int, words) -> bytes:
    body = bytes([protocol.HEADER_BYTE, packet_type]) + struct.pack("<hhhh", *words)
    return body + bytes([protocol.checksum(body + b"\x00")])


def accel_packet(ax_raw, ay_raw, az_raw, temp_raw=2500):
    return make_packet(protocol.PacketType.ACCELERATION, (ax_raw, ay_raw, az_raw, temp_raw))


def gyro_packet(wx_raw, wy_raw, wz_raw, temp_raw=2500):
    return make_packet(protocol.PacketType.ANGULAR_VELOCITY, (wx_raw, wy_raw, wz_raw, temp_raw))


def test_checksum_is_low_byte_of_the_first_ten():
    packet = accel_packet(0, 0, 2048)
    assert len(packet) == protocol.PACKET_LENGTH
    assert packet[10] == sum(packet[:10]) & 0xFF


def test_acceleration_scaling_matches_the_manual_formula():
    # 1 g on Z: raw = 32768 / 16 = 2048
    packet = protocol.decode(accel_packet(0, 0, 2048))
    assert packet is not None
    assert packet.type is protocol.PacketType.ACCELERATION
    assert packet.values[2] == pytest.approx(protocol.GRAVITY_MPS2, rel=1e-9)
    assert packet.values[0] == pytest.approx(0.0)
    assert packet.temperature_c == pytest.approx(25.0)


def test_acceleration_is_signed():
    packet = protocol.decode(accel_packet(0, 0, -2048))
    assert packet.values[2] == pytest.approx(-protocol.GRAVITY_MPS2, rel=1e-9)


def test_angular_velocity_scaling_matches_the_manual_formula():
    # 2000 deg/s full scale at raw 32767 ~= full scale.
    quarter_scale = 32768 // 4  # 500 deg/s
    packet = protocol.decode(gyro_packet(quarter_scale, 0, 0))
    assert packet.values[0] == pytest.approx(500.0 * math.pi / 180.0, rel=1e-6)


def test_quaternion_is_ordered_w_x_y_z_and_unitless():
    raw = 32768
    packet = protocol.decode(make_packet(protocol.PacketType.QUATERNION, (raw - 1, 0, 0, 0)))
    assert packet.type is protocol.PacketType.QUATERNION
    assert len(packet.values) == 4
    assert packet.values[0] == pytest.approx(1.0, abs=1e-4)
    assert packet.temperature_c is None


def test_magnetic_field_stays_in_raw_counts():
    packet = protocol.decode(make_packet(protocol.PacketType.MAGNETIC_FIELD, (100, -200, 300, 2500)))
    assert packet.values == [100.0, -200.0, 300.0]


def test_decode_rejects_bad_checksum_and_unknown_type():
    packet = bytearray(accel_packet(1, 2, 3))
    packet[10] ^= 0xFF
    assert protocol.decode(bytes(packet)) is None

    unknown = bytearray(accel_packet(1, 2, 3))
    unknown[1] = 0x7E
    unknown[10] = protocol.checksum(bytes(unknown))
    assert protocol.decode(bytes(unknown)) is None


def test_stream_decodes_a_clean_burst():
    stream = protocol.PacketStream()
    packets = stream.feed(accel_packet(0, 0, 2048) + gyro_packet(10, 20, 30))
    assert [p.type for p in packets] == [
        protocol.PacketType.ACCELERATION,
        protocol.PacketType.ANGULAR_VELOCITY,
    ]
    assert stream.counters.checksum_failures == 0
    assert stream.counters.resync_bytes_discarded == 0


def test_stream_survives_a_split_across_reads():
    stream = protocol.PacketStream()
    packet = accel_packet(0, 0, 2048)
    assert stream.feed(packet[:5]) == []
    got = stream.feed(packet[5:])
    assert len(got) == 1
    assert stream.buffered_bytes == 0


def test_stream_resynchronises_after_a_dropped_byte():
    stream = protocol.PacketStream()
    good = accel_packet(0, 0, 2048)
    corrupted = good[:-1]  # one byte lost in transit
    packets = stream.feed(corrupted + good + gyro_packet(1, 2, 3))
    # The truncated packet is unrecoverable, but the two that follow it
    # must still be decoded -- a framer that desynchronised here would
    # lose the rest of the session, not just one packet.
    assert [p.type for p in packets] == [
        protocol.PacketType.ACCELERATION,
        protocol.PacketType.ANGULAR_VELOCITY,
    ]
    assert stream.counters.resync_bytes_discarded > 0


def test_stream_rejects_a_header_byte_appearing_inside_data():
    # 0x5555 as a data word puts two 0x55 bytes in the payload; the
    # checksum is what must reject the false start, not the header scan.
    stream = protocol.PacketStream()
    packets = stream.feed(accel_packet(0x5555, 0, 2048) + gyro_packet(0, 0, 0))
    assert len(packets) == 2
    assert packets[0].values[0] != 0.0


def test_stream_counts_checksum_failures_separately_from_resync():
    stream = protocol.PacketStream()
    bad = bytearray(accel_packet(0, 0, 2048))
    bad[10] ^= 0xFF
    stream.feed(bytes(bad) + gyro_packet(0, 0, 0))
    assert stream.counters.checksum_failures >= 1


def test_euler_packet_is_recognised_so_it_does_not_desynchronise():
    # configure.py disables this type, but an unconfigured unit emits it and
    # the framer must skip it cleanly rather than lose the packets after it.
    stream = protocol.PacketStream()
    packets = stream.feed(
        make_packet(protocol.PacketType.EULER_ANGLE, (100, 200, 300, 0)) + gyro_packet(1, 2, 3)
    )
    assert [p.type for p in packets] == [
        protocol.PacketType.EULER_ANGLE,
        protocol.PacketType.ANGULAR_VELOCITY,
    ]
