import socket
import threading

import numpy as np
import pytest

from uw_holoocean_adapter.holoocean_driver import RawSensorFrame
from uw_holoocean_adapter.raw_frame_wire import (
    THRUSTER_COUNT,
    decode_raw_sensor_frame,
    decode_thruster_command,
    encode_raw_sensor_frame,
    encode_thruster_command,
    recv_raw_sensor_frame,
    recv_thruster_command,
    send_raw_sensor_frame,
    send_thruster_command,
)


def _sample_frame() -> RawSensorFrame:
    return RawSensorFrame(
        sim_time_s=1.5,
        receive_time_s=1000.25,
        sensors={
            "LeftCamera": np.arange(2 * 3 * 4, dtype=np.uint8).reshape(2, 3, 4),
            "ImagingSonar": np.linspace(0.0, 1.0, 6, dtype=np.float32).reshape(2, 3),
            "DepthSensor": np.array([-3.03], dtype=np.float64),
            "VehicleOrientation": np.eye(3, dtype=np.float64),
        },
    )


def test_encode_decode_round_trips_scalar_fields_and_every_array():
    frame = _sample_frame()
    decoded = decode_raw_sensor_frame(encode_raw_sensor_frame(frame))

    assert decoded.sim_time_s == frame.sim_time_s
    assert decoded.receive_time_s == frame.receive_time_s
    assert set(decoded.sensors.keys()) == set(frame.sensors.keys())
    for name, array in frame.sensors.items():
        np.testing.assert_array_equal(decoded.sensors[name], array)
        assert decoded.sensors[name].dtype == array.dtype
        assert decoded.sensors[name].shape == array.shape


def test_decode_returns_an_owned_writable_array_not_a_view_into_the_blob():
    frame = _sample_frame()
    blob = encode_raw_sensor_frame(frame)
    decoded = decode_raw_sensor_frame(blob)
    del blob  # the decoded arrays must not depend on this object staying alive
    decoded.sensors["LeftCamera"][0, 0, 0] = 255  # must not raise (read-only view would)
    assert decoded.sensors["LeftCamera"][0, 0, 0] == 255


def test_encode_decode_handles_frame_with_no_sensors():
    frame = RawSensorFrame(sim_time_s=0.0, receive_time_s=0.0, sensors={})
    decoded = decode_raw_sensor_frame(encode_raw_sensor_frame(frame))
    assert decoded.sensors == {}


def test_thruster_command_round_trips():
    values = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7, -0.8]
    assert decode_thruster_command(encode_thruster_command(values)) == pytest.approx(values)


def test_encode_thruster_command_rejects_wrong_length():
    with pytest.raises(ValueError, match=f"\\({THRUSTER_COUNT},\\)"):
        encode_thruster_command([0.0, 0.0, 0.0])


def _socket_pair():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect(server.getsockname())
    accepted, _ = server.accept()
    server.close()
    return client, accepted


def test_send_recv_raw_sensor_frame_over_a_real_socket_round_trips():
    sender, receiver = _socket_pair()
    try:
        frame = _sample_frame()
        thread = threading.Thread(target=send_raw_sensor_frame, args=(sender, frame))
        thread.start()
        received = recv_raw_sensor_frame(receiver)
        thread.join(timeout=5.0)
        assert not thread.is_alive()
        np.testing.assert_array_equal(received.sensors["LeftCamera"], frame.sensors["LeftCamera"])
        assert received.sim_time_s == frame.sim_time_s
    finally:
        sender.close()
        receiver.close()


def test_send_recv_thruster_command_over_a_real_socket_round_trips():
    sender, receiver = _socket_pair()
    try:
        values = [1.0, 2.0, 3.0, 4.0, -1.0, -2.0, -3.0, -4.0]
        thread = threading.Thread(target=send_thruster_command, args=(sender, values))
        thread.start()
        received = recv_thruster_command(receiver)
        thread.join(timeout=5.0)
        assert not thread.is_alive()
        assert received == pytest.approx(values)
    finally:
        sender.close()
        receiver.close()


def test_recv_raw_sensor_frame_raises_connection_error_on_early_close():
    sender, receiver = _socket_pair()
    sender.close()
    with pytest.raises(ConnectionError):
        recv_raw_sensor_frame(receiver)
    receiver.close()
