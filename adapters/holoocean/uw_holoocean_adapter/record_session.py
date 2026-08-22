"""Records a real HoloOcean session into a canonical MCAP bag —
apps/tools/synth_bag_gen's real-sensor counterpart. Ties together
HoloOceanSession (holoocean_driver.py) + camera_conversion.py +
state_conversion.py + sonar_conversion.py + imu_conversion.py +
dvl_conversion.py + CanonicalMcapWriter into one recording run.

Only runs where HoloOcean itself can render: this needs a real GPU with
native Vulkan ray-tracing support. WSL2's Dozen (Vulkan-on-D3D12)
translation layer lacks ray-tracing support, so this has only been
exercised on native Windows — see the HoloOcean deployment notes for the
full WSL2-vs-native investigation. Run with:

    python -m uw_holoocean_adapter.record_session --out bag.mcap

A keyframe is emitted only on ticks where the camera sensors actually
published (they run at their own configured Hz, slower than the
simulation's tick rate — see holoocean_driver.py's RawSensorFrame comment);
non-camera ticks are stepped but produce no bag messages, matching how
apps/tools/synth_bag_gen ties every message to a keyframe rather than to a
raw tick. Sonar/IMU/DVL (P1 workstream B4, docs/superpowers/plans/
2026-08-21-p1-real-multisensor-closed-loop.md) are each written to their
own topic (/raw/sonar_frame, /raw/imu, /raw/dvl) whenever present on a
camera-bearing tick — none of the three is required for the others or for
the tick to count as a keyframe, since real hardware would run them at
independent rates too.
"""
from __future__ import annotations

import argparse
import dataclasses
import math
import pathlib
import sys
from typing import Iterable

import numpy as np

from uw_holoocean_adapter.camera_conversion import holoocean_camera_to_image_frame
from uw_holoocean_adapter.canonical_writer import CanonicalMcapWriter
from uw_holoocean_adapter.dvl_conversion import holoocean_dvl_to_dvl_sample
from uw_holoocean_adapter.holoocean_driver import HoloOceanSession, RawSensorFrame
from uw_holoocean_adapter.imu_conversion import holoocean_imu_to_imu_sample
from uw_holoocean_adapter.sonar_conversion import holoocean_sonar_to_sonar_frame
from uw_holoocean_adapter.state_conversion import depth_sensor_to_evidence, pose_sensor_to_state_snapshot

# HoloOcean ImagingSonar's own config defaults (holoocean.sensors.
# ImagingSonar docstring: Azimuth=120 degrees, RangeMin=0.1m, RangeMax=10m)
# — used when converting a raw sonar reading, since the Python sensor's
# returned array carries no calibration of its own (see sonar_conversion.py's
# module docstring). Override via record_session()'s sonar_* parameters if a
# scenario configures the sensor differently.
_DEFAULT_SONAR_HORIZONTAL_FOV_RAD = math.radians(120.0)
_DEFAULT_SONAR_MIN_RANGE_M = 0.1
_DEFAULT_SONAR_MAX_RANGE_M = 10.0

# HoloOcean sensor classes' own default `name=` (holoocean.sensors.
# {IMUSensor,DVLSensor,ImagingSonar} constructors) — the RawSensorFrame.
# sensors dict key each publishes under unless a scenario JSON overrides
# `name`, same status as "LeftCamera"/"RightCamera"/"PoseSensor"/
# "DepthSensor" above (this file's pre-existing camera/state keys): a
# reasonable, grounded default, not independently re-verified against every
# possible scenario config in this sandbox (no live HoloOcean here).
_SONAR_SENSOR_KEY = "ImagingSonar"
_IMU_SENSOR_KEY = "IMUSensor"
_DVL_SENSOR_KEY = "DVLSensor"


@dataclasses.dataclass(frozen=True)
class SchemaModules:
    """Bundles the generated protobuf modules this recorder needs, so
    call sites thread one object instead of an ever-growing positional
    list. Still passed in explicitly (not imported by this module) — same
    reasoning as camera_conversion.py/state_conversion.py's individual
    module parameters: no hard dependency on the generated-code output
    directory layout."""

    image: object
    observation: object
    time: object
    state: object
    measurement: object
    sonar: object
    imu: object
    dvl: object


def _default_command():
    # HoveringAUV's 8-thruster command layout (4 vertical + 4 horizontal) —
    # same shape used by the manual real-install test that first got a
    # frame out of HoloOcean this session. Mild forward thrust, not tuned
    # for any particular trajectory shape.
    return [0, 0, 0, 0, 10, 10, 10, 10]


def _write_keyframe(
    writer: CanonicalMcapWriter,
    modules: SchemaModules,
    frame: RawSensorFrame,
    keyframe_index: int,
    *,
    sonar_horizontal_fov_rad: float = _DEFAULT_SONAR_HORIZONTAL_FOV_RAD,
    sonar_min_range_m: float = _DEFAULT_SONAR_MIN_RANGE_M,
    sonar_max_range_m: float = _DEFAULT_SONAR_MAX_RANGE_M,
) -> bool:
    """Writes one RawSensorFrame's messages if it carries a stereo camera
    pair; returns whether it did (so the caller knows whether to advance
    its keyframe counter). Sonar/IMU/DVL are each written independently
    when present in this tick — none of the three gates on the others or
    on the camera pair being present, since they run at their own rates
    (matching this file's existing camera-pair gating rationale, see the
    module docstring)."""
    sensors = frame.sensors
    if "LeftCamera" not in sensors or "RightCamera" not in sensors:
        return False

    kf_id = "kf" + str(keyframe_index)
    log_time_ns = int(frame.sim_time_s * 1e9)

    left_image = holoocean_camera_to_image_frame(
        modules.image,
        modules.observation,
        modules.time,
        np.asarray(sensors["LeftCamera"]),
        sensor_id="camera_left",
        sensor_frame="camera_left_link",
        observation_id=kf_id,
        capture_time_s=frame.sim_time_s,
    )
    right_image = holoocean_camera_to_image_frame(
        modules.image,
        modules.observation,
        modules.time,
        np.asarray(sensors["RightCamera"]),
        sensor_id="camera_right",
        sensor_frame="camera_right_link",
        observation_id=kf_id,
        capture_time_s=frame.sim_time_s,
    )
    writer.write_message("/raw/camera/left", log_time_ns, left_image)
    writer.write_message("/raw/camera/right", log_time_ns, right_image)

    if "PoseSensor" in sensors:
        snapshot = pose_sensor_to_state_snapshot(
            modules.state,
            modules.time,
            np.asarray(sensors["PoseSensor"]),
            state_id=kf_id,
            capture_time_s=frame.sim_time_s,
        )
        writer.write_message("/gt/state", log_time_ns, snapshot)

    if "DepthSensor" in sensors:
        evidence = depth_sensor_to_evidence(
            modules.measurement,
            np.asarray(sensors["DepthSensor"]),
            evidence_id="depth_" + kf_id,
            source_observation_id=kf_id,
        )
        writer.write_message("/evidence/depth", log_time_ns, evidence)

    if _SONAR_SENSOR_KEY in sensors:
        sonar_frame = holoocean_sonar_to_sonar_frame(
            modules.sonar,
            modules.observation,
            modules.time,
            np.asarray(sensors[_SONAR_SENSOR_KEY]),
            sensor_id="sonar0",
            sensor_frame="sonar_link",
            observation_id=kf_id,
            capture_time_s=frame.sim_time_s,
            horizontal_fov_rad=sonar_horizontal_fov_rad,
            min_range_m=sonar_min_range_m,
            max_range_m=sonar_max_range_m,
        )
        writer.write_message("/raw/sonar_frame", log_time_ns, sonar_frame)

    if _IMU_SENSOR_KEY in sensors:
        imu_sample = holoocean_imu_to_imu_sample(
            modules.imu,
            modules.observation,
            modules.time,
            np.asarray(sensors[_IMU_SENSOR_KEY]),
            sensor_id="imu0",
            sensor_frame="imu_link",
            observation_id=kf_id,
            capture_time_s=frame.sim_time_s,
        )
        writer.write_message("/raw/imu", log_time_ns, imu_sample)

    if _DVL_SENSOR_KEY in sensors:
        dvl_sample = holoocean_dvl_to_dvl_sample(
            modules.dvl,
            modules.observation,
            modules.time,
            np.asarray(sensors[_DVL_SENSOR_KEY]),
            sensor_id="dvl0",
            sensor_frame="dvl_link",
            observation_id=kf_id,
            capture_time_s=frame.sim_time_s,
        )
        writer.write_message("/raw/dvl", log_time_ns, dvl_sample)

    return True


def record_frames(
    modules: SchemaModules,
    frames: Iterable[RawSensorFrame],
    out_path: str,
    *,
    sonar_horizontal_fov_rad: float = _DEFAULT_SONAR_HORIZONTAL_FOV_RAD,
    sonar_min_range_m: float = _DEFAULT_SONAR_MIN_RANGE_M,
    sonar_max_range_m: float = _DEFAULT_SONAR_MAX_RANGE_M,
) -> int:
    """Writes an iterable of RawSensorFrame (see holoocean_driver.py) into a
    canonical MCAP bag at `out_path`, emitting one keyframe per
    camera-bearing frame. This is the testable core — it knows nothing
    about HoloOceanSession, so a test can pass in hand-built frames with no
    real HoloOcean install; record_session() below wraps it with a real
    session for the CLI entrypoint.
    """
    keyframe_index = 0
    with CanonicalMcapWriter(out_path) as writer:
        for frame in frames:
            wrote = _write_keyframe(
                writer,
                modules,
                frame,
                keyframe_index,
                sonar_horizontal_fov_rad=sonar_horizontal_fov_rad,
                sonar_min_range_m=sonar_min_range_m,
                sonar_max_range_m=sonar_max_range_m,
            )
            if wrote:
                keyframe_index += 1
    return keyframe_index


def record_session(
    modules: SchemaModules,
    *,
    scenario_name: str,
    seed: int,
    num_ticks: int,
    out_path: str,
    command=None,
    sonar_horizontal_fov_rad: float = _DEFAULT_SONAR_HORIZONTAL_FOV_RAD,
    sonar_min_range_m: float = _DEFAULT_SONAR_MIN_RANGE_M,
    sonar_max_range_m: float = _DEFAULT_SONAR_MAX_RANGE_M,
) -> int:
    """Runs `num_ticks` simulation steps against a real HoloOcean scenario
    and records every camera-bearing one. Returns the number of keyframes
    written."""
    session = HoloOceanSession(scenario_name, seed)
    resolved_command = command if command is not None else _default_command()
    try:
        frames = (session.step(resolved_command) for _ in range(num_ticks))
        return record_frames(
            modules,
            frames,
            out_path,
            sonar_horizontal_fov_rad=sonar_horizontal_fov_rad,
            sonar_min_range_m=sonar_min_range_m,
            sonar_max_range_m=sonar_max_range_m,
        )
    finally:
        session.close()


def _bootstrap_schema_path() -> None:
    """Generated schema_pb2/ sits alongside this file (see
    tools/codegen/gen_py.sh) but isn't on sys.path by default when this
    module is run as a script rather than through pytest's conftest.py."""
    schema_dir = pathlib.Path(__file__).parent / "schema_pb2"
    if schema_dir.is_dir() and str(schema_dir) not in sys.path:
        sys.path.insert(0, str(schema_dir))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", default="OpenWater-HoveringCamera")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--num-ticks", type=int, default=200)
    parser.add_argument("--out", required=True)
    # 8 floats: HoveringAUV's [4 vertical, 4 horizontal] thruster layout
    # (see _default_command()'s comment) — constant for the whole session
    # (record_session() applies the same command every tick; there is no
    # time-varying command schedule). Omit to keep the existing mild-
    # forward-thrust default. See docs/uw-slam-real-recording-spec-
    # 2026-08-22.md for the three fixed scenarios' concrete values.
    parser.add_argument("--command", type=float, nargs=8, default=None, metavar="V")
    args = parser.parse_args()

    _bootstrap_schema_path()
    from uw.domain import (  # noqa: E402
        dvl_pb2,
        image_pb2,
        imu_pb2,
        measurement_pb2,
        observation_pb2,
        sonar_pb2,
        state_pb2,
        time_pb2,
    )

    modules = SchemaModules(
        image=image_pb2,
        observation=observation_pb2,
        time=time_pb2,
        state=state_pb2,
        measurement=measurement_pb2,
        sonar=sonar_pb2,
        imu=imu_pb2,
        dvl=dvl_pb2,
    )
    num_keyframes = record_session(
        modules,
        scenario_name=args.scenario,
        seed=args.seed,
        num_ticks=args.num_ticks,
        out_path=args.out,
        command=args.command,
    )
    print(f"wrote {num_keyframes} keyframes to {args.out}")


if __name__ == "__main__":
    main()
