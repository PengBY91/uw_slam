"""PREP-C-01 acceptance: connect to a running ArduSub SITL with pymavlink,
see heartbeats, read ATTITUDE and SCALED_PRESSURE2, switch to DEPTH_HOLD.
Exit code 0 only if every step succeeded.

    tools/sitl/.venv/bin/python tools/sitl/sitl_smoke.py [--conn tcp:127.0.0.1:5760]
"""
from __future__ import annotations

import argparse
import sys
import time

from pymavlink import mavutil


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--conn", default="tcp:127.0.0.1:5760")
    parser.add_argument("--timeout-s", type=float, default=60.0)
    args = parser.parse_args()

    link = mavutil.mavlink_connection(args.conn)
    print(f"waiting for heartbeat on {args.conn} ...", flush=True)
    hb = link.wait_heartbeat(timeout=args.timeout_s)
    if hb is None:
        print("FAIL: no heartbeat", flush=True)
        return 1
    print(f"heartbeat: sysid={link.target_system} type={hb.type} autopilot={hb.autopilot} "
          f"mode={mavutil.mode_string_v10(hb)}", flush=True)

    deadline = time.time() + args.timeout_s
    got = {}
    while time.time() < deadline and not {"ATTITUDE", "SCALED_PRESSURE2"} <= got.keys():
        msg = link.recv_match(type=["ATTITUDE", "SCALED_PRESSURE2"], blocking=True, timeout=5)
        if msg is not None:
            got[msg.get_type()] = msg
    for name in ("ATTITUDE", "SCALED_PRESSURE2"):
        if name not in got:
            print(f"FAIL: no {name} within timeout", flush=True)
            return 1
    att = got["ATTITUDE"]
    print(f"ATTITUDE roll={att.roll:.3f} pitch={att.pitch:.3f} yaw={att.yaw:.3f}", flush=True)
    print(f"SCALED_PRESSURE2 press_abs={got['SCALED_PRESSURE2'].press_abs:.1f} hPa", flush=True)

    mode = "ALT_HOLD"  # ArduSub's DEPTH_HOLD is named ALT_HOLD in the MAVLink mode table
    mode_id = link.mode_mapping().get(mode)
    if mode_id is None:
        print(f"FAIL: mode {mode} not in mapping {sorted(link.mode_mapping())}", flush=True)
        return 1
    link.set_mode(mode_id)
    deadline = time.time() + 15.0
    while time.time() < deadline:
        hb = link.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
        if hb is not None and mavutil.mode_string_v10(hb) == mode:
            print(f"mode switched to {mode}", flush=True)
            print("RESULT: OK", flush=True)
            return 0
    print(f"FAIL: mode did not switch to {mode}", flush=True)
    return 1


if __name__ == "__main__":
    sys.exit(main())
