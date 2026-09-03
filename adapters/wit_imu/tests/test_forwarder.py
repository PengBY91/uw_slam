import struct

import pytest

from uw_wit_imu import forwarder, protocol, registers
from uw_wit_imu.forwarder import ImuForwarder, SampleAssembler

from test_protocol import accel_packet, gyro_packet, make_packet


def mag_packet(x=10, y=20, z=30):
    return make_packet(protocol.PacketType.MAGNETIC_FIELD, (x, y, z, 2500))


def quat_packet(w=32767, x=0, y=0, z=0):
    return make_packet(protocol.PacketType.QUATERNION, (w, x, y, z))


def cycle_bytes(index=0):
    """One full device cycle in the order the device emits it."""
    return accel_packet(0, 0, 2048) + gyro_packet(index, 0, 0) + mag_packet() + quat_packet()


def test_assembler_emits_only_when_every_expected_type_is_present():
    assembler = SampleAssembler()
    stream = protocol.PacketStream()
    packets = stream.feed(cycle_bytes())
    emitted = [assembler.feed(p, 1000.0) for p in packets]
    assert emitted[:3] == [None, None, None]
    assert emitted[3] is not None
    assert set(emitted[3]) == set(protocol.ENABLED_PACKET_TYPES)


def test_assembler_discards_a_partial_cycle_when_a_type_repeats():
    assembler = SampleAssembler()
    stream = protocol.PacketStream()
    # accel, gyro, then accel again: the mag/quat of the first cycle were
    # lost, so that cycle must be dropped rather than half-reported.
    packets = stream.feed(accel_packet(0, 0, 2048) + gyro_packet(1, 0, 0) + cycle_bytes())
    emitted = [assembler.feed(p, 1000.0) for p in packets]
    assert sum(1 for e in emitted if e is not None) == 1
    assert assembler.counters.cycles_incomplete == 1


def test_assembler_requires_accel_and_gyro_in_the_expected_set():
    with pytest.raises(ValueError):
        SampleAssembler([protocol.PacketType.MAGNETIC_FIELD, protocol.PacketType.QUATERNION])


def test_forwarder_produces_one_reading_per_cycle_with_canonical_units():
    fwd = ImuForwarder()
    readings = fwd.feed(cycle_bytes(), 1000.0)
    assert len(readings) == 1
    reading = readings[0]
    assert reading.sequence == 0
    assert reading.linear_acceleration_mps2[2] == pytest.approx(protocol.GRAVITY_MPS2, rel=1e-9)
    assert reading.magnetic_field_counts == (10.0, 20.0, 30.0)
    assert reading.quaternion_wxyz[0] == pytest.approx(1.0, abs=1e-4)
    assert reading.receive_time_s == 1000.0


def test_forwarder_timestamps_come_from_the_reconstructed_timeline_not_the_arrival():
    fwd = ImuForwarder(nominal_rate_hz=200.0)
    period = 1.0 / 200.0
    stamps = []
    for i in range(300):
        # +/-0.8 ms of transport jitter, the order a 230400-baud link
        # actually shows at 200 Hz (one 44-byte cycle is 1.9 ms on the
        # wire).
        arrival = 1000.0 + i * period + (0.8e-3 if i % 2 else -0.8e-3)
        readings = fwd.feed(cycle_bytes(i), arrival)
        stamps.extend(r.capture_time_s for r in readings)
    assert len(stamps) == 300
    gaps = [b - a for a, b in zip(stamps[100:], stamps[101:])]
    # The raw arrivals carry 1.6 ms peak-to-peak; the reconstructed
    # timeline carries well under a twentieth of a period. It is not
    # bit-exactly uniform because the fit is refreshed after every sample,
    # and a strictly alternating jitter pattern is the worst case for a
    # windowed mean (the window's mean flips by +/-jitter/window each
    # step) -- which is exactly why the bound is stated as a fraction of
    # the period rather than as equality.
    assert max(gaps) - min(gaps) < 0.05 * period
    assert max(gaps) - min(gaps) < 0.1 * 1.6e-3
    assert all(gap > 0 for gap in gaps)
    assert gaps[0] == pytest.approx(period, rel=1e-2)
    assert fwd.timebase.counters.samples_missing == 0


def test_forwarder_sequence_ids_are_contiguous_across_a_lost_cycle():
    fwd = ImuForwarder()
    period = 1.0 / 200.0
    fwd.feed(cycle_bytes(0), 1000.0)
    # A whole cycle is lost on the link; the next cycle still gets the next
    # sequence id (sequence counts what we FORWARDED) while the timebase
    # records the gap.
    fwd.feed(cycle_bytes(1), 1000.0 + 2 * period)
    assert fwd.timebase.counters.samples_missing == 1
    readings = fwd.feed(cycle_bytes(2), 1000.0 + 3 * period)
    assert readings[0].sequence == 2


def test_datagram_round_trip():
    payload = b"\x01\x02\x03"
    datagram = forwarder.encode_datagram(forwarder.MESSAGE_TYPE_IMU_SAMPLE, payload)
    message_type, decoded = forwarder.decode_datagram(datagram)
    assert message_type == forwarder.MESSAGE_TYPE_IMU_SAMPLE
    assert decoded == payload


def test_datagram_rejects_foreign_traffic_and_wrong_versions():
    with pytest.raises(ValueError):
        forwarder.decode_datagram(b"junk-datagram-from-something-else")
    good = forwarder.encode_datagram(forwarder.MESSAGE_TYPE_HEALTH_REPORT, b"x")
    wrong_version = bytearray(good)
    wrong_version[4] = 99
    with pytest.raises(ValueError):
        forwarder.decode_datagram(bytes(wrong_version))
    truncated = good[:-1]
    with pytest.raises(ValueError):
        forwarder.decode_datagram(truncated)


def test_imu_sample_protobuf_carries_both_stamps_and_canonical_units():
    fwd = ImuForwarder()
    reading = fwd.feed(cycle_bytes(), 1234.5)[0]
    blob = fwd.encode_imu_sample(reading)

    forwarder._bootstrap_schema_path()
    from uw.domain import imu_pb2, observation_pb2, time_pb2

    sample = imu_pb2.ImuSample()
    sample.ParseFromString(blob)
    assert list(sample.linear_acceleration_mps2)[2] == pytest.approx(protocol.GRAVITY_MPS2, rel=1e-9)
    assert len(sample.angular_velocity_radps) == 3
    assert sample.has_bias is False
    header = sample.header
    assert header.sensor_id.value == "hwt9053"
    assert header.sensor_frame.value == "imu_link"
    assert header.validity == observation_pb2.ObservationHeader.VALIDITY_OK
    assert header.clock_domain == time_pb2.ClockDomain.CLOCK_DOMAIN_SYSTEM_REALTIME
    # capture_time is the reconstruction, receive_time the raw arrival --
    # the shore side must be able to audit one against the other.
    assert header.receive_time.seconds == 1234
    assert header.capture_time.seconds == 1234
    assert "uw_wit_imu.forwarder" in header.provenance


def test_health_report_is_emitted_once_per_interval():
    fwd = ImuForwarder(nominal_rate_hz=200.0, health_interval_s=1.0)
    period = 1.0 / 200.0
    reports = 0
    for i in range(400):
        now = 1000.0 + i * period
        readings = fwd.feed(cycle_bytes(i), now)
        for datagram in fwd.datagrams_for(readings, now):
            if forwarder.decode_datagram(datagram)[0] == forwarder.MESSAGE_TYPE_HEALTH_REPORT:
                reports += 1
    assert reports == 1  # 400 cycles at 200 Hz == 2 s, minus the first interval


def test_health_report_flags_packet_loss_and_recovers():
    fwd = ImuForwarder(nominal_rate_hz=200.0, health_interval_s=1.0)
    forwarder._bootstrap_schema_path()
    from uw.domain import health_pb2

    period = 1.0 / 200.0
    # A clean second first, so the timebase is locked.
    for i in range(200):
        fwd.feed(cycle_bytes(i), 1000.0 + i * period)
    fwd.maybe_build_health_report(1000.0 + 200 * period, force=True)

    # Now a second where every other cycle is lost.
    for i in range(200, 400, 2):
        fwd.feed(cycle_bytes(i), 1000.0 + i * period)
    blob = fwd.maybe_build_health_report(1000.0 + 400 * period, force=True)
    report = health_pb2.HealthReport()
    report.ParseFromString(blob)
    assert report.component_id == forwarder.COMPONENT_ID
    assert report.status in (
        health_pb2.HealthReport.STATUS_SUSPECT,
        health_pb2.HealthReport.STATUS_UNAVAILABLE,
    )
    assert report.dropped_frame_count > 0
    assert report.input_valid_rate < 0.99


def test_health_report_counts_checksum_failures_as_rejected_frames():
    fwd = ImuForwarder(nominal_rate_hz=200.0)
    forwarder._bootstrap_schema_path()
    from uw.domain import health_pb2

    corrupted = bytearray(accel_packet(0, 0, 2048))
    corrupted[10] ^= 0xFF
    fwd.feed(bytes(corrupted) + cycle_bytes(), 1000.0)
    blob = fwd.maybe_build_health_report(1001.0, force=True)
    report = health_pb2.HealthReport()
    report.ParseFromString(blob)
    assert report.rejected_frame_count >= 1


def test_a_clean_stream_reports_healthy():
    fwd = ImuForwarder(nominal_rate_hz=200.0, health_interval_s=1.0)
    forwarder._bootstrap_schema_path()
    from uw.domain import health_pb2

    period = 1.0 / 200.0
    for i in range(400):
        fwd.feed(cycle_bytes(i), 1000.0 + i * period)
    blob = fwd.maybe_build_health_report(1000.0 + 400 * period, force=True)
    report = health_pb2.HealthReport()
    report.ParseFromString(blob)
    assert report.status == health_pb2.HealthReport.STATUS_HEALTHY
    assert report.dropped_frame_count == 0
    assert report.rejected_frame_count == 0
