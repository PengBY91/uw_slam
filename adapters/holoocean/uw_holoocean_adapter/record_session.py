"""Records a real HoloOcean session into a canonical MCAP bag —
apps/tools/synth_bag_gen's real-sensor counterpart. Ties together
HoloOceanSession (holoocean_driver.py) + camera_conversion.py +
state_conversion.py + CanonicalMcapWriter into one recording run.

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
raw tick.
"""
from __future__ import annotations

import argparse
import pathlib
import sys
from typing import Iterable

import numpy as np

from uw_holoocean_adapter.camera_conversion import holoocean_camera_to_image_frame
from uw_holoocean_adapter.canonical_writer import CanonicalMcapWriter
from uw_holoocean_adapter.holoocean_driver import HoloOceanSession, RawSensorFrame
from uw_holoocean_adapter.state_conversion import depth_sensor_to_evidence, pose_sensor_to_state_snapshot


def _default_command():
    # HoveringAUV's 8-thruster command layout (4 vertical + 4 horizontal) —
    # same shape used by the manual real-install test that first got a
    # frame out of HoloOcean this session. Mild forward thrust, not tuned
    # for any particular trajectory shape.
    return [0, 0, 0, 0, 10, 10, 10, 10]


def _write_keyframe(
    writer: CanonicalMcapWriter,
    image_pb2_module,
    observation_pb2_module,
    time_pb2_module,
    state_pb2_module,
    measurement_pb2_module,
    frame: RawSensorFrame,
    keyframe_index: int,
) -> bool:
    """Writes one RawSensorFrame's messages if it carries a stereo camera
    pair; returns whether it did (so the caller knows whether to advance
    its keyframe counter)."""
    sensors = frame.sensors
    if "LeftCamera" not in sensors or "RightCamera" not in sensors:
        return False

    kf_id = "kf" + str(keyframe_index)
    log_time_ns = int(frame.sim_time_s * 1e9)

    left_image = holoocean_camera_to_image_frame(
        image_pb2_module,
        observation_pb2_module,
        time_pb2_module,
        np.asarray(sensors["LeftCamera"]),
        sensor_id="camera_left",
        sensor_frame="camera_left_link",
        observation_id=kf_id,
        capture_time_s=frame.sim_time_s,
    )
    right_image = holoocean_camera_to_image_frame(
        image_pb2_module,
        observation_pb2_module,
        time_pb2_module,
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
            state_pb2_module,
            time_pb2_module,
            np.asarray(sensors["PoseSensor"]),
            state_id=kf_id,
            capture_time_s=frame.sim_time_s,
        )
        writer.write_message("/gt/state", log_time_ns, snapshot)

    if "DepthSensor" in sensors:
        evidence = depth_sensor_to_evidence(
            measurement_pb2_module,
            np.asarray(sensors["DepthSensor"]),
            evidence_id="depth_" + kf_id,
            source_observation_id=kf_id,
        )
        writer.write_message("/evidence/depth", log_time_ns, evidence)

    return True


def record_frames(
    image_pb2_module,
    observation_pb2_module,
    time_pb2_module,
    state_pb2_module,
    measurement_pb2_module,
    frames: Iterable[RawSensorFrame],
    out_path: str,
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
                image_pb2_module,
                observation_pb2_module,
                time_pb2_module,
                state_pb2_module,
                measurement_pb2_module,
                frame,
                keyframe_index,
            )
            if wrote:
                keyframe_index += 1
    return keyframe_index


def record_session(
    image_pb2_module,
    observation_pb2_module,
    time_pb2_module,
    state_pb2_module,
    measurement_pb2_module,
    *,
    scenario_name: str,
    seed: int,
    num_ticks: int,
    out_path: str,
    command=None,
) -> int:
    """Runs `num_ticks` simulation steps against a real HoloOcean scenario
    and records every camera-bearing one. Returns the number of keyframes
    written."""
    session = HoloOceanSession(scenario_name, seed)
    resolved_command = command if command is not None else _default_command()
    try:
        frames = (session.step(resolved_command) for _ in range(num_ticks))
        return record_frames(
            image_pb2_module,
            observation_pb2_module,
            time_pb2_module,
            state_pb2_module,
            measurement_pb2_module,
            frames,
            out_path,
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
    args = parser.parse_args()

    _bootstrap_schema_path()
    from uw.domain import image_pb2, measurement_pb2, observation_pb2, state_pb2, time_pb2  # noqa: E402

    num_keyframes = record_session(
        image_pb2,
        observation_pb2,
        time_pb2,
        state_pb2,
        measurement_pb2,
        scenario_name=args.scenario,
        seed=args.seed,
        num_ticks=args.num_ticks,
        out_path=args.out,
    )
    print(f"wrote {num_keyframes} keyframes to {args.out}")


if __name__ == "__main__":
    main()
