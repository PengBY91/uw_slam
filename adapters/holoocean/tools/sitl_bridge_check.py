"""PREP-A-05 acceptance driver: exercise ArduSub SITL through the
ardusub_sitl_bridge and check that the closed loop actually behaves.

Run it against `--physics mock` first (proves the SITL-side protocol and
the thruster correspondence with no simulator attached) and then, unchanged,
against `--physics holoocean` (the same checks, now through UE5). Anything
that passes on mock and fails on HoloOcean is a HoloOcean problem, which is
the whole point of splitting it that way.

Checks, in order:
  1. heartbeat + SIM_RATE_HZ matches the bridge's step rate (a mismatch
     silently makes both simulators run at different speeds while each
     believes it is in sync -- ArduSub's SITL default is 1200 Hz, which no
     external physics backend can serve);
  2. depth from the simulated barometer is plausible for the spawn depth
     (catches a sign error in the NED position conversion, which otherwise
     shows up much later as an EKF that will not settle);
  3. per-axis sign checks: forward / lateral / vertical / yaw each moved
     the vehicle the way the pilot asked. Unit tests pin
     ARDUSUB_TO_HOLOOCEAN / ARDUSUB_THRUSTER_SIGN against the engine's own
     C++ constants; this is what checks that those constants describe the
     simulator that is actually running.

     Read the MAGNITUDES, not just the pass/fail. This check asks "did it
     move the right way by more than a threshold", which a tumbling vehicle
     satisfies by accident -- that is exactly how a mapping under which a
     pure surge demand produced ZERO force still passed the forward axis
     (2026-09-03, see docs/ardusub-sitl-bridge-feasibility.md 2.4). Sane
     values on the current geometry are ~2 m/s translation and ~1 rad/s
     yaw; tenths of a m/s, or tens of rad/s, mean something is wrong even
     when every line says OK;
  4. DEPTH_HOLD (ArduSub calls the mode ALT_HOLD) keeps the depth;
  5. no loss of sync over the requested soak duration.

    adapters/holoocean/.venv/bin/python adapters/holoocean/tools/sitl_bridge_check.py \
        --expect-rate-hz 50 --soak-s 60
"""
from __future__ import annotations

import argparse
import csv
import dataclasses
import math
import statistics
import sys
import time
from typing import Dict, List, Optional, Tuple

from pymavlink import mavutil

# MANUAL_CONTROL axes for ArduSub: x -> forward, y -> lateral (right),
# r -> yaw, all int16 in [-1000, 1000] with 0 neutral -- but z -> throttle
# is 0..1000 with 500 NEUTRAL, in every mode. Verified against
# SERVO_OUTPUT_RAW on a running SITL: with z = 0 the four vertical
# thrusters sit at PWM 1700 (half thrust) rather than 1500, i.e. z = 0 is a
# half-rate DESCENT command, not neutral. Getting this wrong makes
# DEPTH_HOLD look like it is drifting when it is faithfully obeying a
# descend command.
#
# ArduSub also scales pilot input by JS_GAIN_DEFAULT (0.5 by default), so
# a full-scale axis only reaches half thrust; that is why the thresholds
# below are set well under the vehicle's terminal speeds.
_FULL = 800
_NEUTRAL_Z = 500


@dataclasses.dataclass
class AxisResult:
    name: str
    commanded: str
    observed_m: float
    expected_sign: int
    passed: bool
    detail: str = ""


def _wait_heartbeat(link, timeout_s: float):
    heartbeat = link.wait_heartbeat(timeout=timeout_s)
    if heartbeat is None:
        raise SystemExit("FAIL: no MAVLink heartbeat -- is SITL running?")
    return heartbeat


def _get_param(link, name: str, timeout_s: float = 10.0) -> Optional[float]:
    link.mav.param_request_read_send(link.target_system, link.target_component, name.encode(), -1)
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        message = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
        if message is not None and message.param_id.strip("\x00") == name:
            return float(message.param_value)
    return None


def _set_param(link, name: str, value: float, timeout_s: float = 10.0) -> bool:
    link.mav.param_set_send(
        link.target_system, link.target_component, name.encode(), float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        message = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
        if message is not None and message.param_id.strip("\x00") == name:
            return abs(float(message.param_value) - float(value)) < 1e-3
    return False


def _read_trace_window(path: str, start_monotonic_s: float, end_monotonic_s: float) -> List[dict]:
    """Rows of the bridge's --trace-csv whose wall clock falls in a window.

    The bridge stamps each frame with time.monotonic(), which on Linux is
    CLOCK_MONOTONIC and therefore comparable across processes, so this
    script can select exactly the frames that were served while it was
    holding an axis.
    """
    rows: List[dict] = []
    try:
        with open(path, newline="", encoding="ascii") as handle:
            for row in csv.DictReader(handle):
                try:
                    wall = float(row["wall_s"])
                except (KeyError, TypeError, ValueError):
                    continue
                if start_monotonic_s <= wall <= end_monotonic_s:
                    rows.append(row)
    except OSError:
        return []
    return rows


def _trace_body_velocity(rows: List[dict]) -> Optional[Tuple[float, float, float]]:
    """Mean (forward, right, down) body velocity from trace rows.

    The trace carries the world-frame velocity the PHYSICS produced and the
    body's yaw within that world, so this is ground truth: it does not go
    anywhere near ArduSub's EKF. That matters -- during the lateral hold
    the EKF was seen reporting 2.9 m/s (and, on other runs, -5 m/s) while
    the physics never exceeded 1.06 m/s, because the parasitic yaw moment
    in HoloOcean's own thruster table spins the vehicle and the estimator
    degrades. The sign test is about the bridge's thruster mapping, so it
    must measure the bridge, not the estimator.
    """
    samples: List[Tuple[float, float, float]] = []
    for row in rows:
        try:
            vx, vy, vz = float(row["vx"]), float(row["vy"]), float(row["vz"])
            yaw = math.radians(float(row["yaw_deg"]))
        except (KeyError, TypeError, ValueError):
            continue
        forward = vx * math.cos(yaw) + vy * math.sin(yaw)
        left = -vx * math.sin(yaw) + vy * math.cos(yaw)
        # World is z-up and the body frame is FLU, so "right" is -left and
        # "down" is -vz.
        samples.append((forward, -left, -vz))
    if not samples:
        return None
    return (
        statistics.fmean(s[0] for s in samples),
        statistics.fmean(s[1] for s in samples),
        statistics.fmean(s[2] for s in samples),
    )


def _pump(link) -> Dict[str, object]:
    """Drains everything queued and returns the LATEST message of each type.

    Necessary, not tidiness: ArduSub streams roughly two dozen message
    types, so at 10 Hz each there are ~250 messages a second arriving. A
    sampling loop that calls recv_match() once per iteration at 20 Hz falls
    a factor of ten behind and ends up reading state from seconds ago --
    which showed up here as axis measurements that flipped sign between
    otherwise identical runs, because each axis test was scoring the tail
    of the PREVIOUS axis test. Draining to empty every iteration keeps the
    samples current.
    """
    latest: Dict[str, object] = {}
    while True:
        message = link.recv_match(blocking=False)
        if message is None:
            return latest
        latest[message.get_type()] = message


def _request_streams(link, rate_hz: int = 10) -> None:
    """ArduSub sends only a small default message set until a GCS asks for
    more; GLOBAL_POSITION_INT in particular arrives too slowly to sample an
    axis test from. (LOCAL_POSITION_NED is not streamed by ArduSub at all,
    even after this request -- confirmed empirically -- which is why the
    axis checks below read GLOBAL_POSITION_INT velocities instead.)"""
    link.mav.request_data_stream_send(
        link.target_system, link.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, rate_hz, 1
    )


def _mean_velocity_ned(link, seconds: float) -> Optional[Tuple[float, float, float]]:
    """Mean EKF velocity over a window, m/s, NED.

    Velocity rather than a position delta: it needs no reference point, it
    settles within the hold time, and it is the quantity that answers "did
    the vehicle go the way the pilot asked" without depending on where the
    EKF thinks the origin is.
    """
    samples: List[Tuple[float, float, float]] = []
    deadline = time.time() + seconds
    while time.time() < deadline:
        message = link.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
        if message is not None:
            samples.append((message.vx / 100.0, message.vy / 100.0, message.vz / 100.0))
    if not samples:
        return None
    return (
        statistics.fmean(s[0] for s in samples),
        statistics.fmean(s[1] for s in samples),
        statistics.fmean(s[2] for s in samples),
    )


def _drain(link, seconds: float) -> None:
    deadline = time.time() + seconds
    while time.time() < deadline:
        _pump(link)
        time.sleep(0.05)


def _send_manual(link, x: int = 0, y: int = 0, z: int = _NEUTRAL_Z, r: int = 0) -> None:
    link.mav.manual_control_send(link.target_system, x, y, z, r, 0)


def _hold_axis(link, axis: str, magnitude: int, duration_s: float, rate_hz: float = 20.0):
    """Holds one MANUAL_CONTROL axis for `duration_s`, then returns the mean
    BODY-frame velocity (forward, right, down) over the second half; the
    first half is acceleration.

    Body frame, not NED. MANUAL_CONTROL axes are body-relative, so
    comparing them against earth-frame velocities is only valid at heading
    zero -- and STABILIZE holds whatever heading the EKF happened to settle
    on, which differs run to run. Measured directly: two identical runs
    reported +0.53 and -0.12 m/s "north" for the same forward command
    purely because the vehicle was pointing differently. Rotating the
    horizontal velocity back through the current yaw makes the check
    heading-independent and reproducible."""
    axes = {"x": 0, "y": 0, "z": _NEUTRAL_Z, "r": 0}
    axes[axis] = magnitude
    period = 1.0 / rate_hz
    settle_deadline = time.time() + duration_s * 0.5
    while time.time() < settle_deadline:
        _send_manual(link, axes["x"], axes["y"], axes["z"], axes["r"])
        _pump(link)
        time.sleep(period)

    samples: List[Tuple[float, float, float]] = []
    yaw_rad: Optional[float] = None
    window_start = time.monotonic()
    measure_deadline = time.time() + duration_s * 0.5
    while time.time() < measure_deadline:
        _send_manual(link, axes["x"], axes["y"], axes["z"], axes["r"])
        latest = _pump(link)
        # SIMSTATE is SITL's ground-truth attitude. ATTITUDE (the EKF's
        # estimate) agrees with it to about a degree here, but the point of
        # this check is to catch frame errors, so it should not depend on
        # the EKF being right.
        truth = latest.get("SIMSTATE") or latest.get("ATTITUDE")
        if truth is not None:
            yaw_rad = float(truth.yaw)
        velocity = latest.get("GLOBAL_POSITION_INT")
        if velocity is not None and yaw_rad is not None:
            north, east, down = velocity.vx / 100.0, velocity.vy / 100.0, velocity.vz / 100.0
            forward = north * math.cos(yaw_rad) + east * math.sin(yaw_rad)
            right = -north * math.sin(yaw_rad) + east * math.cos(yaw_rad)
            samples.append((forward, right, down))
        time.sleep(period)

    window_end = time.monotonic()
    # Return to neutral and let it coast down before the next axis.
    for _ in range(int(rate_hz * 2)):
        _send_manual(link)
        _pump(link)
        time.sleep(period)
    estimated = None
    if samples:
        estimated = (
            statistics.fmean(s[0] for s in samples),
            statistics.fmean(s[1] for s in samples),
            statistics.fmean(s[2] for s in samples),
        )
    return estimated, (window_start, window_end)


def _hold_axis_rate(link, axis: str, magnitude: int, duration_s: float,
                    rate_hz: float = 20.0) -> Optional[float]:
    """Like _hold_axis but returns the mean ground-truth body yaw rate."""
    axes = {"x": 0, "y": 0, "z": _NEUTRAL_Z, "r": 0}
    axes[axis] = magnitude
    period = 1.0 / rate_hz
    settle_deadline = time.time() + duration_s * 0.5
    while time.time() < settle_deadline:
        _send_manual(link, axes["x"], axes["y"], axes["z"], axes["r"])
        _pump(link)
        time.sleep(period)
    rates: List[float] = []
    measure_deadline = time.time() + duration_s * 0.5
    while time.time() < measure_deadline:
        _send_manual(link, axes["x"], axes["y"], axes["z"], axes["r"])
        truth = _pump(link).get("SIMSTATE")
        if truth is not None:
            rates.append(float(truth.zgyro))
        time.sleep(period)
    for _ in range(int(rate_hz * 2)):
        _send_manual(link)
        _pump(link)
        time.sleep(period)
    return statistics.fmean(rates) if rates else None


def _arm(link, timeout_s: float = 20.0) -> bool:
    link.mav.command_long_send(
        link.target_system, link.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0,
    )
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        heartbeat = link.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if heartbeat is not None and (heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            return True
        # Keep the RC/manual stream alive; ArduSub disarms on failsafe.
        _send_manual(link)
    return False


def _set_mode(link, mode: str, timeout_s: float = 15.0) -> bool:
    mapping = link.mode_mapping() or {}
    if mode not in mapping:
        return False
    link.set_mode(mapping[mode])
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        heartbeat = link.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if heartbeat is not None and mavutil.mode_string_v10(heartbeat) == mode:
            return True
    return False


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--conn", default="tcp:127.0.0.1:5760")
    parser.add_argument("--expect-rate-hz", type=float, default=50.0)
    parser.add_argument("--expect-depth-m", type=float, default=3.0)
    parser.add_argument("--depth-tolerance-m", type=float, default=1.5)
    parser.add_argument("--axis-hold-s", type=float, default=6.0)
    parser.add_argument("--axis-min-speed-mps", type=float, default=0.10)
    parser.add_argument("--axis-min-yaw-rate-radps", type=float, default=0.03)
    parser.add_argument("--trace-csv", default=None,
                        help="the bridge's --trace-csv file; when given, the axis signs are "
                             "measured from the physics itself instead of from ArduSub's EKF")
    parser.add_argument("--depth-hold-settle-s", type=float, default=12.0)
    parser.add_argument("--depth-hold-measure-s", type=float, default=15.0)
    parser.add_argument("--depth-hold-tolerance-m", type=float, default=0.5)
    parser.add_argument("--soak-s", type=float, default=0.0)
    parser.add_argument("--skip-axes", action="store_true")
    args = parser.parse_args(argv)

    failures: List[str] = []
    print(f"connecting to {args.conn} ...", flush=True)
    link = mavutil.mavlink_connection(args.conn)
    heartbeat = _wait_heartbeat(link, 60.0)
    print(f"heartbeat: sysid={link.target_system} mode={mavutil.mode_string_v10(heartbeat)}", flush=True)
    _request_streams(link)

    # 1. clock agreement
    rate = _get_param(link, "SIM_RATE_HZ")
    if rate is None:
        failures.append("could not read SIM_RATE_HZ")
    elif abs(rate - args.expect_rate_hz) > 1.0:
        failures.append(
            f"SIM_RATE_HZ={rate:g} but the bridge steps at {args.expect_rate_hz:g} Hz -- "
            f"the two clocks will diverge (restart SITL with SITL_RATE_HZ={args.expect_rate_hz:g})"
        )
    else:
        print(f"OK   SIM_RATE_HZ={rate:g} matches the bridge step rate", flush=True)

    # 2. depth sign / magnitude, read from the RAW simulated depth sensor.
    #
    # Not from VFR_HUD.alt: ArduSub zeroes its depth reference at boot (the
    # normal procedure is to power the vehicle on at the surface), so a
    # vehicle spawned at 3 m reports "depth 0" and its altitude is only
    # ever relative to wherever it started. SCALED_PRESSURE2 carries the
    # absolute hydrostatic pressure, which is what actually tells us the
    # NED position we sent has the right sign and magnitude.
    _drain(link, 2.0)
    depth_message = _pump(link).get("SCALED_PRESSURE2") or link.recv_match(
        type="SCALED_PRESSURE2", blocking=True, timeout=10)
    if depth_message is None:
        failures.append("no SCALED_PRESSURE2 -- cannot read the simulated depth sensor")
    else:
        # AP_Baro's water model: 101325 Pa at the surface, 9800 Pa per metre.
        expected_hpa = (101325.0 + 9800.0 * args.expect_depth_m) / 100.0
        observed_hpa = float(depth_message.press_abs)
        implied_depth_m = (observed_hpa * 100.0 - 101325.0) / 9800.0
        if abs(implied_depth_m - args.expect_depth_m) > args.depth_tolerance_m:
            failures.append(
                f"depth sensor implies {implied_depth_m:.2f} m but the vehicle was spawned at "
                f"{args.expect_depth_m:.2f} m (press_abs={observed_hpa:.1f} hPa, expected "
                f"{expected_hpa:.1f}) -- suspect the NED position sign, or a SITL home that is "
                f"not at sea level (ArduSub's depth is minus its AMSL altitude)"
            )
        else:
            print(f"OK   depth sensor implies {implied_depth_m:.2f} m at the spawn depth "
                  f"{args.expect_depth_m:.2f} m (press_abs={observed_hpa:.1f} hPa)", flush=True)

    # 3. per-axis signs
    axis_results: List[AxisResult] = []
    if not args.skip_axes:
        for attempt_param, value in (("ARMING_CHECK", 0), ("FS_GCS_ENABLE", 0), ("FS_EKF_ACTION", 0)):
            _set_param(link, attempt_param, value)
        # STABILIZE, not MANUAL: MANUAL is a direct passthrough with no
        # attitude hold, so any thrust asymmetry lets the vehicle roll and
        # yaw freely and the next axis measurement is taken in an unknown
        # attitude. STABILIZE is also what the PREP-A-05 acceptance text
        # asks for ("STABILIZE mode, joystick moves the vehicle").
        mode = "STABILIZE" if "STABILIZE" in (link.mode_mapping() or {}) else "MANUAL"
        if not _set_mode(link, mode):
            failures.append(f"could not switch to {mode}")
        elif not _arm(link):
            failures.append("could not arm (check ARMING_CHECK / EKF status)")
        else:
            print(f"OK   armed in {mode}", flush=True)
            # (axis, magnitude, NED component index, expected sign, label)
            plan: List[Tuple[str, int, int, int, str]] = [
                ("x", _FULL, 0, +1, "forward -> +forward velocity"),
                ("y", _FULL, 1, +1, "right -> +right velocity"),
                ("z", _NEUTRAL_Z - _FULL, 2, +1, "down -> +down velocity"),
            ]
            for axis, magnitude, component, expected_sign, label in plan:
                estimated, window = _hold_axis(link, axis, magnitude, args.axis_hold_s)
                truth = None
                if args.trace_csv:
                    truth = _trace_body_velocity(_read_trace_window(args.trace_csv, *window))
                source = "physics" if truth is not None else "EKF"
                velocity = truth if truth is not None else estimated
                if velocity is None:
                    axis_results.append(AxisResult(axis, label, 0.0, expected_sign, False,
                                                    "no velocity samples"))
                    continue
                observed = velocity[component]
                ok = (observed * expected_sign) > args.axis_min_speed_mps
                detail = f"m/s ({source})"
                if truth is not None and estimated is not None:
                    detail += f"; EKF said {estimated[component]:+.2f}"
                axis_results.append(AxisResult(axis, label, observed, expected_sign, ok, detail))

            # Yaw is checked on the ground-truth yaw RATE, not on a net
            # heading change: STABILIZE's own heading hold fights a yaw
            # demand, so the net angle is small, and it is contaminated by
            # the vehicle still settling from the previous axis. SIMSTATE's
            # gyro is SITL truth in FRD, so a nose-right command must show
            # a positive z rate.
            yaw_rates = _hold_axis_rate(link, "r", _FULL, args.axis_hold_s)
            if yaw_rates is None:
                axis_results.append(AxisResult("r", "yaw right -> +yaw rate", 0.0, +1, False,
                                                "no SIMSTATE"))
            else:
                axis_results.append(
                    AxisResult("r", "yaw right -> +yaw rate", yaw_rates, +1,
                               yaw_rates > args.axis_min_yaw_rate_radps, "rad/s")
                )

            for result in axis_results:
                status = "OK  " if result.passed else "FAIL"
                print(f"{status} axis {result.name}: {result.commanded}, observed "
                      f"{result.observed_m:+.3f} {result.detail or 'm'}", flush=True)
                if not result.passed:
                    failures.append(f"axis {result.name} ({result.commanded}) did not move as commanded "
                                    f"(observed {result.observed_m:+.3f}) -- check "
                                    f"ardusub_sitl_bridge.ARDUSUB_TO_HOLOOCEAN / "
                                    f"ARDUSUB_THRUSTER_SIGN against the engine's BlueROV2.h/.cpp")

            # 4. DEPTH_HOLD
            if _set_mode(link, "ALT_HOLD"):
                # Let the vehicle come to rest FIRST: the axis tests leave
                # it at up to 1 m/s, and measuring the depth spread while
                # DEPTH_HOLD is still arresting that would score the
                # controller's step response, not its hold.
                settle_deadline = time.time() + args.depth_hold_settle_s
                while time.time() < settle_deadline:
                    _send_manual(link)
                    _pump(link)
                    time.sleep(0.05)
                depths: List[float] = []
                deadline = time.time() + args.depth_hold_measure_s
                while time.time() < deadline:
                    _send_manual(link)
                    message = _pump(link).get("VFR_HUD")
                    if message is not None:
                        depths.append(-float(message.alt))
                    time.sleep(0.05)
                if len(depths) >= 5:
                    spread = max(depths) - min(depths)
                    if spread > args.depth_hold_tolerance_m:
                        failures.append(f"DEPTH_HOLD drifted {spread:.2f} m over "
                                        f"{args.depth_hold_measure_s:.0f} s")
                    else:
                        print(f"OK   DEPTH_HOLD held within {spread:.2f} m over "
                              f"{args.depth_hold_measure_s:.0f} s "
                              f"(mean depth {statistics.fmean(depths):.2f} m)", flush=True)
                else:
                    failures.append("DEPTH_HOLD: too few depth samples")
            else:
                failures.append("could not switch to ALT_HOLD (DEPTH_HOLD)")

    # 5. soak
    if args.soak_s > 0.0:
        print(f"soaking for {args.soak_s:.0f} s ...", flush=True)
        started = time.time()
        last_heartbeat = started
        gaps: List[float] = []
        while time.time() - started < args.soak_s:
            message = link.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
            now = time.time()
            if message is None:
                failures.append("lost the MAVLink heartbeat during the soak")
                break
            gaps.append(now - last_heartbeat)
            last_heartbeat = now
            _send_manual(link)
        if gaps:
            print(f"OK   soak: {len(gaps)} heartbeats, max gap {max(gaps):.2f} s", flush=True)

    print()
    if failures:
        print("RESULT: FAIL", flush=True)
        for failure in failures:
            print(f"  - {failure}", flush=True)
        return 1
    print("RESULT: OK", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
