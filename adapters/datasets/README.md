# Public dataset adapters

Converts public SLAM/underwater datasets into the canonical MCAP schema defined in
`schemas/proto/uw/domain/` — the same wire format `adapters/holoocean` and `apps/synth_bag_gen`
produce, so `apps/replay_demo` and the rest of the pipeline do not depend on a bag's origin.

## `uw_dataset_adapter` (EuRoC MAV Dataset)

`uw_dataset_adapter/euroc_converter.py` converts the EuRoC MAV Dataset's (ASL, ETH Zurich)
machine-hall sequences (verified against MH_01_easy) into a canonical MCAP bag. See that module's
own docstring for the full story — source access constraints (the ASL host isn't reachable from
every network), why it reads from a ROS1 bag rather than the dataset's raw CSV+PNG archive
(`rosbag1_reader.py`, hand-rolled to tolerate a truncated/partial-download bag file), and why it
undistorts every frame itself before writing it (`undistort.py` — raw distorted frames produced
zero relative-pose factors on this scene's repetitive structure; the converter docstring records
the diagnosis).

```bash
cd adapters/datasets
python3 -m venv .venv && .venv/bin/pip install -e ".[dev]"
.venv/bin/pytest -q   # offline unit tests, no network/download needed

.venv/bin/python3 -m uw_dataset_adapter.euroc_converter \
  --bag-url https://huggingface.co/datasets/kavehsgh/EuRoC_MAV_Dataset_Machine_Hall_Easy_01/resolve/main/MH_01_easy.bag \
  --max-download-bytes 209715200 \
  --max-keyframes 50 \
  --out /tmp/euroc_mh01.mcap

build/bin/replay_demo --bag /tmp/euroc_mh01.mcap \
  --experiment configs/experiment/euroc_mh01_vo.yaml --out /tmp/euroc_demo
```

Ground truth and IMU are deliberately not converted (see `euroc_converter.py`'s docstring for why —
short version: nothing in this pipeline consumes IMU yet, and `stereo_landmark_vo` doesn't need GT
to run).

## Adding another dataset

Not implemented yet: SVIn's own public datasets, RUSSO/Tank/SonarSweep if/when available (see the
platform architecture's sim-to-real section). Follow `euroc_converter.py`'s pattern (and
`apps/synth_bag_gen.cpp`'s topic/message/keyframe-id conventions underneath it) rather than
inventing a new one per dataset — in particular, check whether the new dataset's `apps/replay_demo`
consumer needs frames on a fixed 5Hz grid (`apps/replay_demo.cpp`'s `kKeyframeIntervalS`, a real v1
limitation, not a convention worth re-deriving per adapter) and whether its camera frames are
already rectified or need the same `undistort.py`-style preprocessing this converter needed.
