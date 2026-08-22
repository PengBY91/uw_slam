"""Converts a EuRoC MAV Dataset (ASL, ETH Zurich) ROS1 bag's stereo camera
topics into a canonical MCAP bag this repo's apps/replay_demo can consume
directly — P1 workstream A3 (docs/superpowers/plans/2026-08-21-p1-real-
multisensor-closed-loop.md): the point is proving schemas/proto/uw/domain/
and the stereo_landmark_vo pipeline aren't HoloOcean-specific, using real
external sensor data, not synthetic placeholders.

Source format: a ROS1 bag (not the ASL raw CSV+PNG archive) — every
reachable public mirror of the ASL raw archive during this work turned out
to be either unreachable from this sandbox (robotics.ethz.ch resolves to an
RFC 2544 benchmark-range address here, i.e. effectively blocked) or bundled
as a single multi-GB zip with no per-file access (GlowBond/EuRoC_MAV_Dataset
on Hugging Face: a 12.7GB machine_hall.zip containing a further zip-of-zip
per sequence — downloading enough of it to reach individual files would
cost more bandwidth than the bag route below). A ROS1 bag mirror
(huggingface.co/datasets/kavehsgh/EuRoC_MAV_Dataset_Machine_Hall_Easy_01)
supports HTTP Range requests, so this converter downloads only a PREFIX of
the bag (default 200MB, ~14s of real 20Hz stereo — plenty for a schema/
pipeline smoke test) via rosbag1_reader.py's truncation-tolerant sequential
reader, instead of the full ~2.7GB file.

Ground truth and IMU are deliberately NOT converted: (1) apps/replay_demo's
stereo_landmark_vo path computes relative pose from images directly, no GT
needed to run — GT only feeds --align-ate scoring, which this converter's
verification doesn't require; (2) this repo has no IMU factor/estimator
consumer yet (confirmed: P3's current-state audit found zero non-test
users of any IMU-related type), so writing IMU wire messages here would be
unused plumbing, not something this converter's own verification exercises.
Both are naturally available in the same bag (topics /vicon/... or
/leica/position for GT, /imu0 for IMU) if a future workstream needs them —
this file's scope is deliberately the smallest real slice that proves the
schema/pipeline works end to end.

Keyframe timing: apps/replay_demo.cpp hardcodes a fixed 5Hz keyframe grid
(kKeyframeIntervalS = 0.2, "matches synth_bag_gen's 5 Hz spacing" per its
own comment) for deriving a camera ImageFrame's keyframe id from its
capture_time — a real v1 limitation of that app, not something this
converter works around. So stereo pairs are subsampled from EuRoC's native
20Hz to that fixed grid, and written with SYNTHETIC capture_time values
(kf_index * 0.2s) rather than the original recording's true timestamps —
documented here rather than silently relabeling real data as if the
original timestamps were preserved. The image CONTENT is real; only the
wire-level capture_time is rebinned to satisfy the consuming app's fixed-
rate assumption.

Undistortion turned out to be REQUIRED, not optional: verified this the
hard way (this repo's own convention — see CLAUDE.md's "已经踩过的坑" — of not
trusting an assumption without actually running the real pipeline). A
first version of this converter wrote raw (distorted) frames directly with
is_rectified=False, matching P1 workstream A2's precedent of not wiring
include/sensor_models/camera_rectifier.hpp into any real-camera path
without also retuning the downstream matcher. That produced ZERO relative-
pose factors end to end. Root-caused with a standalone probe (HarrisCorner-
Detector + PatchMatcher run directly against the real decoded frames,
outside replay_demo): 60 corners were found per frame (detection was never
the problem), but stereo matching against undistorted-geometry assumptions
on real distorted images in this specific scene (machine-hall room full of
repetitive structure — parallel pipes, a metal rack, wooden pallet slats)
produced mostly WRONG correspondences (observed left-minus-right pixel
"disparity" was negative for nearly every match, i.e. geometrically
backwards for a true correspondence), which the frontend's
min_disparity_px=1.0 filter then correctly rejected — leaving zero
triangulated landmarks and hence zero relative-pose factors. So this
converter undistorts each frame itself (undistort.py — a Python port of
camera_rectifier.hpp's exact algorithm, see that module's docstring) before
writing it, and sets is_rectified=True truthfully. This does NOT contradict
A2's precedent (not wiring rectification into apps/replay_demo.cpp itself,
which regressed real-bag VO tracking on a DIFFERENT dataset for a DIFFERENT
reason — bilinear smoothing removing fine noise-like texture Harris relied
on there): here rectification is applied once, offline, at conversion time,
against a scene with plenty of large-scale structure (not noise-dependent
texture), and it fixes a correctness problem (wrong epipolar geometry)
rather than trading one property for another.
"""
from __future__ import annotations

import argparse
import pathlib
import sys

import requests

from uw_dataset_adapter.canonical_writer import CanonicalMcapWriter
from uw_dataset_adapter.rosbag1_reader import ImageMessage, iter_messages, parse_image_message
from uw_dataset_adapter.undistort import undistort_mono8

CAM0_TOPIC = "/cam0/image_raw"
CAM1_TOPIC = "/cam1/image_raw"
# Matches apps/replay_demo.cpp's `constexpr double kKeyframeIntervalS = 0.2`
# exactly — see this module's docstring for why that coupling exists.
KEYFRAME_INTERVAL_S = 0.2

# MUST stay in sync with configs/rig/euroc_mh01.yaml's cameras: block — this
# converter undistorts against the same calibration the C++ side's rig file
# declares, so a StereoGeometry/PinholeCamera computation downstream and
# this converter's own preprocessing agree on what "undistorted" means for
# these two cameras. Sourced from the same OKVIS republication of the
# dataset's calibration (see the rig YAML's header comment for the full
# provenance note).
CAMERA_INTRINSICS = {
    "camera_left": dict(
        fx=458.654880721, fy=457.296696463, cx=367.215803962, cy=248.37534061,
        k1=-0.28340811217, k2=0.0739590738929, p1=0.000193595028569, p2=1.76187114545e-05,
    ),
    "camera_right": dict(
        fx=457.587426604, fy=456.13442556, cx=379.99944652, cy=255.238185386,
        k1=-0.283683654496, k2=0.0745128430929, p1=-0.000104738949098, p2=-3.55590700274e-05,
    ),
}


def download_bag_prefix(url: str, out_path: pathlib.Path, max_bytes: int) -> None:
    """Downloads only the first `max_bytes` of `url` via an HTTP Range
    request. Works because rosbag1_reader.py reads sequentially from the
    start and tolerates a truncated tail (see its module docstring) — this
    is not a generic partial-download helper, it depends on that contract."""
    response = requests.get(url, headers={"Range": f"bytes=0-{max_bytes - 1}"}, stream=True, timeout=120)
    response.raise_for_status()
    with open(out_path, "wb") as f:
        for chunk in response.iter_content(chunk_size=4 * 1024 * 1024):
            f.write(chunk)


def extract_stereo_pairs(bag_path: pathlib.Path, max_keyframes: int) -> list:
    """Returns up to `max_keyframes` (ImageMessage, ImageMessage) cam0/cam1
    pairs, subsampled onto KEYFRAME_INTERVAL_S spacing (see module
    docstring) and paired by nearest capture timestamp (EuRoC's stereo pair
    is hardware-synced, so this is a formality, not a real alignment
    problem — cam0/cam1 timestamps in this dataset differ by microseconds,
    nowhere near the 0.2s keyframe spacing)."""
    cam0: list = []
    cam1: list = []
    for msg in iter_messages(str(bag_path)):
        if msg.msgtype != "sensor_msgs/Image":
            continue
        img = parse_image_message(msg.raw)
        if msg.topic == CAM0_TOPIC:
            cam0.append(img)
        elif msg.topic == CAM1_TOPIC:
            cam1.append(img)

    if not cam0 or not cam1:
        raise RuntimeError(
            f"no stereo images decoded from {bag_path} (found {len(cam0)} {CAM0_TOPIC} / "
            f"{len(cam1)} {CAM1_TOPIC} messages) — the downloaded bag prefix may be too "
            "short (try a larger --max-download-bytes), or this bag uses different topic names"
        )

    t0 = cam0[0].stamp_ns
    pairs = []
    next_target_s = 0.0
    cam1_idx = 0
    for img0 in cam0:
        t_s = (img0.stamp_ns - t0) / 1e9
        if t_s + 1e-6 < next_target_s:
            continue
        while cam1_idx + 1 < len(cam1) and abs(cam1[cam1_idx + 1].stamp_ns - img0.stamp_ns) <= abs(
            cam1[cam1_idx].stamp_ns - img0.stamp_ns
        ):
            cam1_idx += 1
        pairs.append((img0, cam1[cam1_idx]))
        next_target_s += KEYFRAME_INTERVAL_S
        if len(pairs) >= max_keyframes:
            break
    return pairs


def make_image_frame(
    image_pb2_module,
    observation_pb2_module,
    time_pb2_module,
    img: ImageMessage,
    sensor_frame: str,
    sensor_id: str,
    kf_index: int,
):
    """Builds a uw.domain.ImageFrame for keyframe `kf_index` from a decoded
    EuRoC sensor_msgs/Image, undistorted against CAMERA_INTRINSICS[sensor_id]
    (see module docstring for why that turned out to be required). See
    module docstring for why capture_time is a synthetic
    kf_index * KEYFRAME_INTERVAL_S value rather than the original recording
    timestamp."""
    calib = CAMERA_INTRINSICS[sensor_id]
    undistorted_data = undistort_mono8(
        img.data, img.width, img.height, img.step,
        calib["fx"], calib["fy"], calib["cx"], calib["cy"], calib["k1"], calib["k2"], calib["p1"], calib["p2"],
    )

    frame = image_pb2_module.ImageFrame()
    frame.header.sensor_frame.value = sensor_frame
    frame.header.sensor_id.value = sensor_id
    t_ns = kf_index * int(round(KEYFRAME_INTERVAL_S * 1e9))
    frame.header.capture_time.seconds = t_ns // 1_000_000_000
    frame.header.capture_time.nanos = t_ns % 1_000_000_000
    frame.header.clock_domain = time_pb2_module.CLOCK_DOMAIN_SENSOR_HARDWARE
    frame.header.validity = observation_pb2_module.ObservationHeader.VALIDITY_OK
    frame.header.provenance = "euroc_converter_v1"
    frame.width = img.width
    frame.height = img.height
    frame.row_stride_bytes = img.width  # undistort_mono8's output is tightly packed (no row padding)
    frame.encoding = image_pb2_module.ImageFrame.IMAGE_ENCODING_MONO8
    frame.pixel_data = undistorted_data
    frame.is_rectified = True
    return frame


def _bootstrap_schema_path() -> None:
    """Generated schema_pb2/ sits alongside this file (see
    tools/codegen/gen_py.sh) but isn't on sys.path by default when this
    module is run as a script — same pattern as adapters/holoocean/
    uw_holoocean_adapter/record_session.py's identically-named helper."""
    schema_dir = pathlib.Path(__file__).parent / "schema_pb2"
    if schema_dir.is_dir() and str(schema_dir) not in sys.path:
        sys.path.insert(0, str(schema_dir))


def convert(bag_path: pathlib.Path, out_path: str, max_keyframes: int) -> int:
    _bootstrap_schema_path()
    from uw.domain import image_pb2, observation_pb2, time_pb2  # noqa: E402

    pairs = extract_stereo_pairs(bag_path, max_keyframes)
    with CanonicalMcapWriter(out_path) as writer:
        for i, (img0, img1) in enumerate(pairs):
            t_ns = i * int(round(KEYFRAME_INTERVAL_S * 1e9))
            left = make_image_frame(image_pb2, observation_pb2, time_pb2, img0, "camera_left_link", "camera_left", i)
            right = make_image_frame(
                image_pb2, observation_pb2, time_pb2, img1, "camera_right_link", "camera_right", i
            )
            writer.write_message("/raw/camera/left", t_ns, left)
            writer.write_message("/raw/camera/right", t_ns, right)
    return len(pairs)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag-url", help="HTTP(S) URL of a ROS1 bag v2.0 file (only a prefix is downloaded)")
    parser.add_argument("--bag-path", help="local .bag file path (skips download if given)")
    parser.add_argument(
        "--max-download-bytes", type=int, default=200 * 1024 * 1024, help="prefix size when using --bag-url"
    )
    parser.add_argument("--max-keyframes", type=int, default=40)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    if args.bag_path:
        bag_path = pathlib.Path(args.bag_path)
    elif args.bag_url:
        bag_path = pathlib.Path(args.out).with_suffix(".source.bag")
        print(f"downloading first {args.max_download_bytes} bytes of {args.bag_url} -> {bag_path}")
        download_bag_prefix(args.bag_url, bag_path, args.max_download_bytes)
    else:
        parser.error("one of --bag-path or --bag-url is required")
        return

    num_keyframes = convert(bag_path, args.out, args.max_keyframes)
    print(f"wrote {num_keyframes} keyframes (stereo pairs) to {args.out}")


if __name__ == "__main__":
    main()
