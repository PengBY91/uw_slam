import io

import pytest

from uw_wit_imu import configure, dump, protocol, registers


def test_write_frame_layout():
    frame = registers.write_frame(registers.REG_UNLOCK, registers.UNLOCK_KEY)
    assert frame == bytes([0xFF, 0xAA, 0x69, 0x88, 0xB5])  # value is little-endian
    assert len(frame) == 5


def test_write_frame_rejects_out_of_range():
    with pytest.raises(ValueError):
        registers.write_frame(0x100, 0)
    with pytest.raises(ValueError):
        registers.write_frame(0x00, 0x10000)


def test_configuration_sequence_order_is_the_one_the_device_requires():
    names = [name for name, _ in registers.configuration_sequence()]
    # unlock first, save last, and the baud change late enough that every
    # earlier write still reaches the device at the old speed.
    assert names[0] == "unlock"
    assert names[-1] == "save"
    assert names.index("baud") > names.index("rate")
    assert names.index("baud") > names.index("content")
    # A second unlock after the port is reopened: the device relocks when
    # the link is dropped, so the save would otherwise be ignored.
    assert names.index("unlock", 1) > names.index("baud")


def test_prep_d01_content_mask_enables_exactly_the_four_wanted_packets():
    mask = registers.PREP_D01_OUTPUT_CONTENT
    assert mask & registers.CONTENT_BIT_ACCELERATION
    assert mask & registers.CONTENT_BIT_ANGULAR_VELOCITY
    assert mask & registers.CONTENT_BIT_MAGNETIC_FIELD
    assert mask & registers.CONTENT_BIT_QUATERNION
    # Euler angles are forbidden repo-wide (CLAUDE.md) and every other
    # packet type is link budget spent for nothing.
    assert not mask & registers.CONTENT_BIT_EULER_ANGLE
    assert bin(mask).count("1") == 4
    assert len(protocol.ENABLED_PACKET_TYPES) == 4


def test_link_budget_rejects_the_factory_baud_rate_at_the_target_output_rate():
    # This is the whole reason PREP-D-01 exists: at 9600 baud the device
    # silently drops packets instead of slowing down.
    assert not registers.baud_rate_is_sufficient(
        registers.FACTORY_BAUD_RATE, registers.TARGET_OUTPUT_RATE_HZ, registers.PREP_D01_OUTPUT_CONTENT
    )
    assert registers.baud_rate_is_sufficient(
        registers.TARGET_BAUD_RATE, registers.TARGET_OUTPUT_RATE_HZ, registers.PREP_D01_OUTPUT_CONTENT
    )


def test_required_bytes_per_second_matches_the_packet_arithmetic():
    assert registers.required_bytes_per_second(200.0, registers.PREP_D01_OUTPUT_CONTENT) == 200.0 * 4 * 11


def test_unsupported_rate_or_baud_is_a_hard_error_not_a_silent_fallback():
    with pytest.raises(ValueError):
        registers.configuration_sequence(output_rate_hz=137.0)
    with pytest.raises(ValueError):
        registers.configuration_sequence(baud_rate=250000)


def test_configure_defaults_to_a_dry_run_and_writes_nothing(capsys):
    assert configure.main([]) == 0
    out = capsys.readouterr().out
    assert "dry run" in out
    assert "FF AA 69 88 B5" in out


def test_configure_refuses_to_apply_while_the_manual_is_unverified(capsys, monkeypatch):
    monkeypatch.setattr(protocol, "MANUAL_REVISION", "")
    assert configure.main(["--port", "/dev/null", "--apply"]) == 2
    assert "MANUAL_REVISION is empty" in capsys.readouterr().err


def test_configure_refuses_an_insufficient_baud_rate(capsys):
    assert configure.main(["--baud", "9600", "--rate", "200"]) == 2
    assert "cannot carry" in capsys.readouterr().err


class FakeClock:
    """Advances only when read, so count_stream's window is deterministic."""

    def __init__(self, step: float) -> None:
        self._now = 0.0
        self._step = step

    def __call__(self) -> float:
        now = self._now
        self._now += self._step
        return now


def synthetic_capture(cycles: int) -> bytes:
    from test_protocol import accel_packet, gyro_packet, make_packet

    out = bytearray()
    for _ in range(cycles):
        out += accel_packet(0, 0, 2048)
        out += gyro_packet(0, 0, 0)
        out += make_packet(protocol.PacketType.MAGNETIC_FIELD, (1, 2, 3, 2500))
        out += make_packet(protocol.PacketType.QUATERNION, (32767, 0, 0, 0))
    return bytes(out)


def test_count_stream_counts_every_enabled_type_from_a_replay():
    cycles = 500
    source = io.BytesIO(synthetic_capture(cycles))
    result = dump.count_stream(source, duration_s=1e9, nominal_rate_hz=200.0, clock=FakeClock(0.0))
    for packet_type in protocol.ENABLED_PACKET_TYPES:
        assert result["per_type"][packet_type.name] == cycles
    assert result["checksum_failures"] == 0
    assert result["resync_bytes_discarded"] == 0


def test_count_stream_reports_corruption_rather_than_hiding_it():
    payload = bytearray(synthetic_capture(50))
    payload[11 * 10 + 10] ^= 0xFF  # break one packet's checksum
    result = dump.count_stream(io.BytesIO(bytes(payload)), 1e9, 200.0, clock=FakeClock(0.0))
    assert result["checksum_failures"] >= 1
    assert result["packets_decoded"] == 50 * 4 - 1


def test_format_report_flags_a_disabled_packet_type_that_is_still_arriving():
    result = {
        "elapsed_s": 1.0,
        "per_type": {protocol.PacketType.EULER_ANGLE.name: 200},
        "packets_decoded": 200,
        "checksum_failures": 0,
        "resync_bytes_discarded": 0,
        "fitted_rate_hz": 200.0,
        "timebase_locked": True,
        "cycles_missing": 0,
        "last_cycle_start_s": 0.0,
    }
    text = dump.format_report(result, 200.0)
    assert "should be disabled" in text
    assert "LOW ACCELERATION" in text
