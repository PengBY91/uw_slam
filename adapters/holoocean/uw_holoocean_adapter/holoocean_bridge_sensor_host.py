"""Runs on the native Windows host where real HoloOcean actually works (this
WSL2 dev sandbox has no Vulkan ray-tracing support -- see the
`project-holoocean-deployment` memory record). Owns the real
`HoloOceanSession`, spawns the manifest's task props exactly like
`RealtimeRosSession.__init__` does, then connects OUT (as a TCP client) to
`bridged_realtime_ros_session.py` on the WSL2 side and runs a simple
request/response loop: receive the thruster command the WSL2 side wants
applied this tick, `session.step()` it, send back the resulting
`RawSensorFrame`. See `raw_frame_wire.py`'s module docstring for the wire
protocol and why this two-host split exists.

Deliberately has NO rclpy/ROS2 dependency and no pilot-command
shaping/fault-injection/degradation logic of its own -- all of that already
lives in `BridgedRealtimeRosSession` on the WSL2 side (reusing the exact
same tested code `RealtimeRosSession` uses locally); this script's only job
is "run the real simulator and shuttle raw frames/commands across the wire."

Deployment: copy this file plus `raw_frame_wire.py`, `holoocean_driver.py`,
`scenario_manifest.py`, `scenario_randomization.py`, and the scenario/task
files it needs into the existing
`C:\\Users\\pengb\\uw_slam_holoocean_check\\uw_holoocean_adapter\\` harness
directory (see that memory record's "Reusable verification harness"
section) -- this machine's Python has no path back into a full uw_slam
checkout. Invoke via a `.py` file (not inline `-c`, the documented
WSL-interop-via-powershell.exe gotcha) with `-WorkingDirectory` set to a
real local path (the documented WinError 4551 gotcha).

`bridged_realtime_ros_session.py` (WSL2 side) must already be listening
before this script is started -- it prints the host:port it bound and waits
for this script's connection.
"""
from __future__ import annotations

import argparse
import socket
import time

from uw_holoocean_adapter.holoocean_driver import HoloOceanSession
from uw_holoocean_adapter.raw_frame_wire import recv_thruster_command, send_raw_sensor_frame
from uw_holoocean_adapter.scenario_manifest import load_realtime_manifest
from uw_holoocean_adapter.scenario_randomization import ScenarioRandomization


def _reconnect(host: str, port: int, attempt: int) -> socket.socket:
    print(f"holoocean_bridge_sensor_host: reconnecting to {host}:{port} (attempt #{attempt})...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host, port))
    print("holoocean_bridge_sensor_host: reconnected")
    return sock


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--task", required=True)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--bridge-host", default="127.0.0.1")
    parser.add_argument("--bridge-port", type=int, default=5599)
    args = parser.parse_args()

    manifest = load_realtime_manifest(args.scenario, args.task)

    print(f"holoocean_bridge_sensor_host: constructing HoloOceanSession (seed={args.seed})...")
    session = HoloOceanSession(
        manifest.holoocean_scenario_cfg(), args.seed, randomization=ScenarioRandomization()
    )
    for prop in manifest.task.props:
        session.spawn_prop(
            prop.prop_type, location=list(prop.location_m), material=prop.visual_material, tag=prop.tag,
        )
    print("holoocean_bridge_sensor_host: real HoloOcean session ready.")

    print(f"holoocean_bridge_sensor_host: connecting to {args.bridge_host}:{args.bridge_port}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((args.bridge_host, args.bridge_port))
    print("holoocean_bridge_sensor_host: connected -- entering step loop.")

    tick_count = 0
    reconnects = 0
    try:
        while True:
            # This dev machine's WSL2<->Windows TCP path has been observed
            # to drop long-lived connections mid-run (network/security-
            # policy flakiness, not a pipeline bug -- confirmed directly
            # against this bridge: 476 real ticks flowed, then one
            # ConnectionResetError). Reconnect and keep going rather than
            # exit -- the WSL2-side listener stays open across reconnects
            # (see bridged_realtime_ros_session.py's own reconnect loop).
            while True:
                try:
                    command = recv_thruster_command(sock)
                    break
                except OSError as error:
                    reconnects += 1
                    print(f"holoocean_bridge_sensor_host: connection issue receiving command "
                          f"({error!r}), reconnecting (#{reconnects})...")
                    sock.close()
                    time.sleep(1.0)
                    sock = _reconnect(args.bridge_host, args.bridge_port, reconnects)

            # session.step() already ran a real UE5 tick -- a network hiccup
            # sending its result must not drop that tick's data or re-step
            # the sim, it should just retry sending the SAME frame.
            frame = session.step(command)
            while True:
                try:
                    send_raw_sensor_frame(sock, frame)
                    break
                except OSError as error:
                    reconnects += 1
                    print(f"holoocean_bridge_sensor_host: connection issue sending frame "
                          f"({error!r}), reconnecting (#{reconnects})...")
                    sock.close()
                    time.sleep(1.0)
                    sock = _reconnect(args.bridge_host, args.bridge_port, reconnects)

            tick_count += 1
            if tick_count % 100 == 0:
                print(f"holoocean_bridge_sensor_host: {tick_count} ticks, sim_time_s={frame.sim_time_s:.2f}, "
                      f"{reconnects} reconnects")
    except KeyboardInterrupt:
        print(f"holoocean_bridge_sensor_host: interrupted after {tick_count} ticks, {reconnects} reconnects")
    finally:
        sock.close()
        session.close()


if __name__ == "__main__":
    main()
