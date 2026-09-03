"""Windows-side physics shuttle for the ArduSub SITL bridge (PREP-A-05).

Runs on the native Windows host where HoloOcean/UE5 actually works (this
WSL2 sandbox has no Vulkan ray-tracing support -- see the
`project-holoocean-deployment` memory record). Owns the real
`HoloOceanSession`, connects OUT as a TCP client to
`ardusub_sitl_bridge.py --physics holoocean` on the WSL2 side, and runs the
same request/response loop `holoocean_bridge_sensor_host.py` runs: receive
the thruster forces to apply this tick, `session.step()` them, send back the
resulting `RawSensorFrame`.

Why this exists instead of reusing `holoocean_bridge_sensor_host.py`: that
script goes through `scenario_manifest.load_realtime_manifest`, which
validates the full perception sensor set (stereo cameras + imaging sonar +
orientation/IMU/depth/pose) because it serves the realtime perception loop.
A SITL physics bridge needs none of that, and dragging cameras and sonar
into the scenario would cost roughly 60 ms of rendering per tick each
(adapters/holoocean/docs/perf/tick_budget_2026-09-02.md) -- which is
precisely the tick rate the PREP-A-05 feasibility question is about. So
this host takes a plain HoloOcean `scenario_cfg` JSON with no manifest
validation, and the lean scenario next to it turns every renderer off.

Deployment: copy this file and the scenario JSON into the existing
``C:\\Users\\pengb\\uw_slam_holoocean_check\\uw_holoocean_adapter\\``
harness directory alongside the `holoocean_driver.py` and
`raw_frame_wire.py` already there. Start the WSL2 side FIRST (it listens),
then this, with ``--bridge-host`` set to the WSL2 IP.

    python holoocean_sitl_physics_host.py \\
        --scenario ardusub_sitl_bridge.json --bridge-host 192.168.3.162
"""
from __future__ import annotations

import argparse
import json
import pathlib
import socket
import time

from uw_holoocean_adapter.holoocean_driver import HoloOceanSession
from uw_holoocean_adapter.raw_frame_wire import recv_thruster_command, send_raw_sensor_frame


def _connect(host: str, port: int, attempt: int) -> socket.socket:
    label = "connecting" if attempt == 0 else f"reconnecting (attempt #{attempt})"
    print(f"holoocean_sitl_physics_host: {label} to {host}:{port} ...", flush=True)
    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((host, port))
            # The bridge serves one SITL frame per round trip, so Nagle's
            # algorithm would add up to 40 ms of latency to every physics
            # step -- comparable to the step itself.
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print("holoocean_sitl_physics_host: connected", flush=True)
            return sock
        except OSError as error:
            print(f"holoocean_sitl_physics_host: {error!r}, retrying in 2 s", flush=True)
            time.sleep(2.0)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", required=True, help="plain HoloOcean scenario_cfg JSON")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--bridge-host", default="127.0.0.1")
    parser.add_argument("--bridge-port", type=int, default=5601)
    parser.add_argument("--report-every", type=int, default=200)
    args = parser.parse_args()

    scenario_cfg = json.loads(pathlib.Path(args.scenario).read_text(encoding="utf-8"))
    # Repo-only keys (documented in the scenario file itself) are stripped
    # before handing the config to HoloOcean, which rejects unknown keys.
    scenario_cfg = {k: v for k, v in scenario_cfg.items() if not k.startswith(("_", "uw_"))}

    print(f"holoocean_sitl_physics_host: constructing HoloOceanSession from {args.scenario} "
          f"(ticks_per_sec={scenario_cfg.get('ticks_per_sec')})...", flush=True)
    session = HoloOceanSession(scenario_cfg, args.seed)
    print("holoocean_sitl_physics_host: session ready -- entering step loop.", flush=True)

    sock = _connect(args.bridge_host, args.bridge_port, 0)
    ticks = 0
    reconnects = 0
    started = time.monotonic()
    try:
        while True:
            while True:
                try:
                    command = recv_thruster_command(sock)
                    break
                except OSError as error:
                    reconnects += 1
                    print(f"holoocean_sitl_physics_host: receive failed ({error!r})", flush=True)
                    sock.close()
                    sock = _connect(args.bridge_host, args.bridge_port, reconnects)

            # session.step() already ran a real UE5 tick; a network hiccup
            # while sending its result must not drop that tick's data or
            # re-step the sim, only retry sending the SAME frame.
            frame = session.step(command)
            while True:
                try:
                    send_raw_sensor_frame(sock, frame)
                    break
                except OSError as error:
                    reconnects += 1
                    print(f"holoocean_sitl_physics_host: send failed ({error!r})", flush=True)
                    sock.close()
                    sock = _connect(args.bridge_host, args.bridge_port, reconnects)

            ticks += 1
            if args.report_every > 0 and ticks % args.report_every == 0:
                elapsed = time.monotonic() - started
                print(f"holoocean_sitl_physics_host: {ticks} ticks, sim_time={frame.sim_time_s:.2f}s, "
                      f"{ticks / max(elapsed, 1e-9):.1f} ticks/s wall, {reconnects} reconnects",
                      flush=True)
    except KeyboardInterrupt:
        elapsed = time.monotonic() - started
        print(f"holoocean_sitl_physics_host: interrupted after {ticks} ticks in {elapsed:.1f} s "
              f"({ticks / max(elapsed, 1e-9):.1f} ticks/s), {reconnects} reconnects", flush=True)
    finally:
        sock.close()
        session.close()


if __name__ == "__main__":
    main()
