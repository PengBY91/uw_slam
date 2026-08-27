import pathlib
import socket
import threading

import numpy as np
import pytest

from uw_holoocean_adapter.bridged_realtime_ros_session import BridgedRealtimeRosSession
from uw_holoocean_adapter.holoocean_driver import RawSensorFrame
from uw_holoocean_adapter.raw_frame_wire import recv_thruster_command, send_raw_sensor_frame
from uw_holoocean_adapter.scenario_manifest import load_realtime_manifest

from test_ros_message_conversion import fake_message_types  # noqa: E402  (shared fakes, same dir)

REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
BASE_SCENARIO = REPO_ROOT / "adapters/holoocean/scenarios/blue_rov_aid_sv1213_base.json"
SEARCH_TASK = REPO_ROOT / "adapters/holoocean/scenarios/aquaculture_search.yaml"


def _connected_pair():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect(server.getsockname())
    accepted, _ = server.accept()
    server.close()
    return client, accepted


def _fake_windows_frame(sim_time_s: float) -> RawSensorFrame:
    return RawSensorFrame(
        sim_time_s=sim_time_s,
        receive_time_s=sim_time_s,
        sensors={
            "LeftCamera": np.zeros((4, 4, 4), dtype=np.uint8),
            "ImagingSonar": np.zeros((2, 3), dtype=np.float32),
        },
    )


@pytest.fixture
def manifest():
    return load_realtime_manifest(BASE_SCENARIO, SEARCH_TASK)


def test_tick_sends_shaped_command_then_returns_messages_built_from_the_received_frame(manifest):
    session_sock, windows_sock = _connected_pair()
    try:
        session = BridgedRealtimeRosSession(manifest, seed=1, rng=np.random.default_rng(1), sock=session_sock)

        received_commands = []

        def fake_windows_peer():
            received_commands.append(recv_thruster_command(windows_sock))
            send_raw_sensor_frame(windows_sock, _fake_windows_frame(sim_time_s=2.5))

        thread = threading.Thread(target=fake_windows_peer)
        thread.start()
        messages = session.tick(fake_message_types())
        thread.join(timeout=5.0)

        assert not thread.is_alive()
        assert len(received_commands) == 1
        assert len(received_commands[0]) == 8
        topics = [topic for topic, _ in messages]
        assert session._topics.left_camera in topics
        assert session._topics.imaging_sonar in topics
        assert session._topics.clock in topics
    finally:
        session_sock.close()
        windows_sock.close()


def test_on_thruster_command_updates_the_command_sent_on_the_next_tick(manifest):
    session_sock, windows_sock = _connected_pair()
    try:
        session = BridgedRealtimeRosSession(manifest, seed=1, rng=np.random.default_rng(1), sock=session_sock)
        # Must exceed this manifest's actuator deadzone (5.0, see
        # blue_rov_aid_sv1213_base.json's uw_metadata.pilot_command_model)
        # or PilotCommandModel treats it as zero regardless of whether
        # on_thruster_command was ever called.
        session.on_thruster_command([50.0] * 8)

        received_commands = []

        def fake_windows_peer():
            received_commands.append(recv_thruster_command(windows_sock))
            send_raw_sensor_frame(windows_sock, _fake_windows_frame(sim_time_s=0.0))

        thread = threading.Thread(target=fake_windows_peer)
        thread.start()
        session.tick(fake_message_types())
        thread.join(timeout=5.0)

        # The pilot command model shapes (rate-limits) the raw [1.0]*8
        # command rather than passing it through unchanged on the very
        # first tick -- exactly zero would mean on_thruster_command's value
        # was never read at all.
        assert any(v != 0.0 for v in received_commands[0])
    finally:
        session_sock.close()
        windows_sock.close()


def test_set_socket_swaps_the_socket_tick_uses(manifest):
    old_session_sock, old_windows_sock = _connected_pair()
    new_session_sock, new_windows_sock = _connected_pair()
    try:
        session = BridgedRealtimeRosSession(manifest, seed=1, rng=np.random.default_rng(1), sock=old_session_sock)
        old_session_sock.close()  # simulate the dropped connection tick() would otherwise use
        session.set_socket(new_session_sock)

        def fake_windows_peer():
            recv_thruster_command(new_windows_sock)
            send_raw_sensor_frame(new_windows_sock, _fake_windows_frame(sim_time_s=1.0))

        thread = threading.Thread(target=fake_windows_peer)
        thread.start()
        messages = session.tick(fake_message_types())
        thread.join(timeout=5.0)

        assert not thread.is_alive()
        assert messages  # reached the new peer successfully, not the closed old one
    finally:
        old_windows_sock.close()
        new_session_sock.close()
        new_windows_sock.close()


def test_on_thruster_command_rejects_wrong_length(manifest):
    session_sock, windows_sock = _connected_pair()
    try:
        session = BridgedRealtimeRosSession(manifest, seed=1, rng=np.random.default_rng(1), sock=session_sock)
        session.on_thruster_command([1.0, 2.0, 3.0])
        assert session._last_thruster_command == [0.0] * 8
    finally:
        session_sock.close()
        windows_sock.close()
