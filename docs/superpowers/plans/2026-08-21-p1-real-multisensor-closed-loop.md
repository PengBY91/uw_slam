# P1 Implementation Plan: Real Offline Multi-Sensor Closed Loop

**Source:** `docs/uw-slam-production-readiness-and-roadmap-2026-08-21.md` section 7,
P1 ("真实离线多传感器闭环"). That document owns the team-level roadmap and phase
acceptance criteria; per its own section 11 maintenance rule, this file owns the
concrete task breakdown for P1 specifically. Do not duplicate the phase-level
"why" here — read the roadmap doc first.

**Status of this file:** planning/sequencing only. No code has been written yet.
This is the artifact requested to "start" P1 — it turns the roadmap's 7-bullet
P1 work list into an ordered, file-level task breakdown with an explicit map of
what can run in this repo today versus what is blocked on the user's Windows
HoloOcean machine or on team formation.

## Why this needs to be one coordinating plan, not seven independent ones

P1's seven roadmap bullets are not independent — several are blocked on
artifacts the others produce, and two of the seven require actions outside
this repo (a live HoloOcean session on Windows) that no amount of code-reading
in this sandbox can substitute for. Writing seven parallel deep-dive plans up
front would produce plans for work whose actual shape depends on data that
doesn't exist yet (e.g. the rectification module's exact interface can't be
finalized against a real distorted stereo pair until one has been recorded).
This file instead: (1) records what's actually in the repo today for each
bullet (verified by reading the code, not assumed from the roadmap prose),
(2) groups the seven bullets into three tracks by what they're blocked on, and
(3) gives per-workstream scope + acceptance criteria at the depth needed to
start — deep enough for the three code-only workstreams to begin immediately,
intentionally lighter for the two recording-dependent ones since their design
should follow from what a first real recording actually looks like.

## Verified current state (read 2026-08-21, this session)

| P1 bullet | Current state | Evidence |
|---|---|---|
| 固定三类 HoloOcean 场景 + 全传感器录制 | Recorder only writes `/raw/camera/{left,right}` + `/evidence/depth`. No sonar, IMU, or DVL topics exist in the writer at all. | `adapters/holoocean/uw_holoocean_adapter/record_session.py` — no sonar/imu/dvl write calls anywhere in the file |
| topic/时间/TF/标定 audit | No audit tooling exists yet. `configs/rig/` holds static rig YAML, parsed into `RigCalibrationSnapshot`; nothing currently checks a *recorded* bag's actual clock domains or TF chain against it. | `find` for `*rig*`/`*audit*` turns up only `configs/rig/`, `rigid_transform_fit.{hpp,cpp}` (a fitting algorithm, not an audit tool) |
| 去畸变 + 双目极线校正 | Explicitly out of scope in the current camera model — v1 assumes input is already undistorted/rectified pixels. `is_rectified` is a proto flag nothing reads. | `include/sensor_models/camera_model.hpp:12-16`; `schemas/proto/uw/domain/measurement.proto:103-104` ("no frontend currently enforces that flag itself") |
| 让 experiment 配置里的 frontend/factor_builder/estimator/map_backend 选择真正生效 | `estimator_mode` **is** dispatched (black_box_vio vs stereo_landmark_vo genuinely switch behavior). `sonar_frontend`, `optical_frontend`, `map_backend` are parsed from YAML into `ExperimentConfig` but **never read again** — `replay_demo` hardcodes one frontend/factor-builder/map-backend pipeline regardless of what the config says. This is documented in-repo already, not a surprise finding. | `apps/replay_demo.cpp:28-33` (own header comment says so explicitly); confirmed by grep — `config.sonar_frontend`/`.optical_frontend`/`.map_backend` appear nowhere outside `config.hpp`/`config.cpp` except that one log line |
| 公开水下数据集 adapter | Stub directory + README only, explicitly "not implemented in this pass." | `adapters/datasets/README.md` (4 sentences, no code) |
| 四组固定 baseline（VIO-only / +depth / +sonar / full fusion） | No baseline harness exists. `configs/experiment/` only has synthetic experiments; nothing runs the same real bag through four ablated configs and tabulates results. | `find configs/experiment` — synthetic-only; no ablation runner found |

## Three tracks, by blocking dependency

**Track A — code-only, startable in this session, no live sensors needed:**
1. Config-driven component selection (make `sonar_frontend`/`optical_frontend`/`map_backend` either dispatch for real or fail loudly instead of silently ignoring the config — directly closes the section-10 risk "配置存在但不驱动实现: 未识别或未实现的算法选择必须启动失败，不能静默回退")
2. Rectification/undistortion module + raw/rectified image contract
3. Public dataset adapter (EuRoC-style stereo+IMU first, per architecture doc's own sim-to-real note)

**Track B — needs a real recording to design against, but the recording spec
itself can be written now:**
4. Canonical MCAP recording spec (topics, rates, coordinate/clock conventions)
   for the three fixed scenarios (straight line, turn, small loop), extending
   `record_session.py` to also write sonar/IMU/DVL topics
5. Topic/capture-time/clock-domain/TF/calibration audit tool — needs a real
   bag to validate against, but the checks it should run are derivable from
   the schema + rig config now

**Track C — blocked on Track B's actual recording landing, plus B done first:**
6. Four-baseline ablation harness (VIO-only / +depth / +sonar / full fusion)
   run against the same real bag
7. Recording the three scenarios themselves on the Windows HoloOcean machine
   — this is a user action, not something this session can do; everything
   else in this plan is designed so it doesn't block on it happening first

Recommended order: **A1 → A2 → B4(spec)+B5(tool) in parallel → user records
on Windows using the B4 spec → B5 validates the real bag → C6 → A3 can happen
any time in parallel, it has no dependency on the others.**

---

## Workstream A1: Make component selection real (or fail loudly) — DONE

Implemented 2026-08-21: `uw::runtime::ValidateExperimentConfigSelections`
(`include/runtime/config.hpp` + `src/runtime/config.cpp`), called from
`apps/replay_demo` immediately after `LoadExperimentConfig`. An experiment
YAML naming an unrecognized `sonar_frontend`/`optical_frontend`/`map_backend`/
`estimator_mode`/`landmark_detector` now fails the run at startup
(`std::cerr` + exit 1) instead of silently running the one hardcoded
pipeline. Verified: 7 new unit tests in `tests/runtime/config_test.cpp`
(all pass, 121/121 ctest total, up from 114); manually ran `replay_demo`
against a config with `map_backend: tsdf_v2_does_not_exist` (fails with a
clear message, exit 1) and against the unmodified `synthetic_smoke.yaml`
(byte-identical ATE 0.0666m/6 iterations — no regression). `configs/
README.md` and `configs/experiment/synthetic_smoke.yaml`'s own comments
updated to describe the new behavior instead of the old "read but not
dispatched" state.

**Left as-is, per the workstream's own scope note:** `sonar_frontend`/
`optical_frontend`/`map_backend` still only ever run one implementation
each — this closes the "must fail on an unrecognized value" half of the
section-10 risk, not the "actually offer more than one implementation"
half, which was never this workstream's goal.

### Original scope (for reference)

**Goal:** `apps/replay_demo` either actually dispatches on
`config.sonar_frontend`/`config.optical_frontend`/`config.map_backend`, or —
for whichever of the three don't have more than one real implementation yet —
the app validates the configured value against the one it actually runs and
exits with an error if they don't match, instead of silently running a
different pipeline than the config claims. This is the minimum fix for the
section-10 risk; it does not require building new frontend/factor-builder/
map-backend implementations that don't exist yet.

**Files:**
- Modify: `apps/replay_demo.cpp` (remove the "read but not dispatched" comment
  once it's no longer true; add the validate-or-dispatch logic near where
  `sonar_frontend`/estimator construction currently happens, ~line 477+)
- Modify: `include/runtime/config.hpp` / `src/runtime/config.cpp` if a
  registry of "known frontend/factor_builder/map_backend identifier strings"
  needs to live somewhere shared (avoid duplicating the string literal list)
- Check: every `configs/experiment/*.yaml` still names a value this fails
  correctly against (should already match `sonar_cfar_frontend_v1` /
  `stereo_depth_frontend_v1` / `submap_point_cloud_v1` defaults)
- Test: extend `tests/runtime/` or add a `replay_demo`-level test asserting
  an unrecognized `sonar_frontend` value in an experiment config causes a
  non-zero exit, not silent fallback

**Acceptance:** setting `sonar_frontend: does_not_exist_v1` in an experiment
YAML and running `replay_demo` against it fails with a clear error instead of
silently running `sonar_cfar_frontend_v1` anyway.

## Workstream A2: Rectification/undistortion module + raw/rectified contract — DONE (module), DEFERRED (wiring)

Implemented 2026-08-22: `PlumbBobDistortion` + `ApplyPlumbBobDistortion` +
`UndistortImage` in `include/sensor_models/camera_rectifier.hpp` +
`src/sensor_models/camera_rectifier.cpp` (added to the `core` CMake target,
next to `camera_model.cpp`). Standard remap approach (forward-distort the
destination grid, bilinearly sample the source — no polynomial inversion
needed); v1 scope matches `camera_model.hpp`'s existing parallel-stereo-only
assumption (same K in and out, no separate rotation-based epipolar step).
9 new tests in `tests/core/camera_rectifier_test.cpp`, including a
closed-form correctness check (a linear-ramp source image whose distorted
output is independently predictable from `ApplyPlumbBobDistortion` alone,
without needing OpenCV as an oracle — this repo has no OpenCV dependency).
All pass; 130/130 ctest total (up from 121).

**Wiring into `apps/replay_demo`'s real-camera path was attempted, then
reverted after actually running it against the real bag** (this repo's own
convention: verify end-to-end, not just unit tests). Findings, so nobody
re-discovers this the hard way:
- The warp itself is correct — dumped actual `/tmp/real_session.mcap` frames
  before/after, visually and statistically normal (mean/std nearly
  unchanged, small expected black wedges near corners from removing
  distortion).
- But wiring it into the `stereo_landmark_vo` block (undistort right after
  `ConvertToMono8`, looking up each camera's `CameraIntrinsics` by
  `sensor_id` from `rig->cameras()`) dropped real-bag VO tracking from
  50/50 keyframes (49 relative-pose factors) to 8/50 (7 factors). Root
  cause: bilinear resampling measurably smooths this bag's texture
  (Laplacian variance −43%), and `harris_corner`'s matcher thresholds
  (`max_row_diff_px`/`min_score_margin`, already documented in
  `apps/replay_demo.cpp` as "a first empirical pass... not a calibrated
  constant") turn out to depend on that fine noise-like texture surviving
  intact.
- This is a retuning problem, not a rectification bug — decided with the
  user (2026-08-22) to ship the module standalone and leave the real-camera
  wiring for whoever next works the real-data VO path, so threshold
  retuning happens deliberately and gets tested against the real bag
  properly, rather than folded silently into a plan that was primarily
  about the rectification module. `apps/replay_demo.cpp`'s
  `stereo_landmark_vo` block carries an inline comment pointing back here.

**Acceptance (module):** met — `UndistortImage` correctly removes lens
distortion (closed-form-tested), synthetic path unaffected (`example_auv.
yaml`'s all-zero distortion means `UndistortImage` is a documented no-op —
this was never exercised in the synthetic demo, only verified directly
against the real bag's actual calibrated distortion).

**Acceptance (wiring into a real pipeline path, `is_rectified` truthfully
set on a real run):** NOT met — deliberately deferred, see above. Whoever
picks this up next should re-wire the same `UndistortImage` call (the code
existed and worked; it was reverted, not deleted from history — see this
plan doc's own prior revision) together with retuning
`vo_params.stereo_matcher`/`vo_params.temporal_matcher` thresholds against
undistorted real frames, and verify keyframe-tracking count doesn't regress
before calling it done.

**Note:** don't scope this to arbitrary/non-parallel rigs — `camera_model.hpp`
already explicitly limits itself to the parallel-stereo case; matched that
scope, no reason found yet to generalize further.

## Workstream A3: Public dataset adapter (EuRoC-style first) — DONE

Implemented and verified end-to-end 2026-08-22: `adapters/datasets/` (a new
`uw-dataset-adapter` Python package, `.venv`-based like `adapters/holoocean`)
converts the EuRoC MAV Dataset's MH_01_easy sequence into a canonical MCAP
`apps/replay_demo` consumes directly, with **real relative-pose factors
computed from real external stereo imagery** — not a "doesn't crash on
empty input" pass.

**Source access problem, and how it was solved:** the ASL host
(`robotics.ethz.ch`) resolves to an RFC 2544 benchmark-range address from
this sandbox — effectively blocked, not a transient failure (confirmed via
`curl -v`, both plain HTTP and through the HTTPS proxy). The only reachable
mirrors bundle the dataset as multi-GB archives with no per-file access
(a 12.7GB `machine_hall.zip` containing a further nested, deflate-
compressed zip — extracting anything from it means decompressing ~1.5GB
regardless of what's actually needed). Pivoted to a ROS1 bag mirror instead
(`huggingface.co/datasets/kavehsgh/EuRoC_MAV_Dataset_Machine_Hall_Easy_01`),
which supports HTTP Range requests — but `rosbags` (the standard pure-
Python ROS bag library) requires the bag's trailing index, which doesn't
exist in a partial download. Wrote a small hand-rolled ROS1 bag v2.0
reader instead (`uw_dataset_adapter/rosbag1_reader.py`) that reads
sequentially from the start and never touches the trailing index — CHUNK
records embed their own CONNECTION+MSGDATA records (verified empirically
against the real bag, not assumed from the spec), so a truncated download
prefix (200MB, ~14s of real 20Hz stereo) decodes cleanly. 3 unit tests
against a hand-crafted minimal bag (`tests/test_rosbag1_reader.py`),
including one asserting a truncated chunk stops cleanly rather than raising.

**A real finding, not assumed away:** a first version wrote raw (distorted)
EuRoC frames directly with `is_rectified=false`, matching P1 workstream
A2's own precedent of not wiring `camera_rectifier.hpp` into a real-camera
path without retuning the matcher against it. That produced **zero**
relative-pose factors end to end. Root-caused with a standalone C++ probe
(HarrisCornerDetector + PatchMatcher run directly against the decoded
frames, bypassing `replay_demo`): 60 corners were found per frame (never
the problem), but on raw distorted images of this specific scene (an ETH
machine-hall room full of repetitive structure — parallel pipes, a metal
rack, wooden pallet slats) stereo matching produced almost entirely
WRONG correspondences — observed left-minus-right pixel "disparity" was
*negative* for nearly every match, i.e. geometrically backwards for a true
correspondence — which `min_disparity_px=1.0` then correctly rejected,
leaving zero triangulated landmarks. So this converter undistorts each
frame itself before writing it (`uw_dataset_adapter/undistort.py` — a
Python port of `camera_rectifier.hpp`'s exact forward-remap algorithm,
closed-form-tested the same way `tests/core/camera_rectifier_test.cpp`
is), and sets `is_rectified=true` truthfully. This does **not** contradict
A2's decision to not wire rectification into `apps/replay_demo.cpp` — that
regression (50→8 keyframes) was on a *different* dataset for a *different*
reason (bilinear smoothing erasing fine noise-like texture Harris
depended on there); here rectification fixes a geometric-correctness bug
against a scene with large-scale structure, not noise-dependent texture.
Calibration (both cameras' intrinsics/distortion, and the ~0.1101m stereo
baseline used to build `configs/rig/euroc_mh01.yaml`) is sourced from
OKVIS's published republication of the same EuRoC calibration (the ASL
host serving the dataset's own `sensor.yaml` files being unreachable, per
above) — verified numerically (baseline magnitude, near-identity
inter-camera rotation) before trusting it, not copied blind.

**Verified real-pipeline result:** `build/bin/replay_demo --bag
<converted.mcap> --experiment configs/experiment/euroc_mh01_vo.yaml`
produces `added 4 relative-pose factors, 5 keyframes ... solver: 1
iterations ... converged ... 1107628 map evidence points added` — real VO
transforms computed from real external camera frames, not a synthetic
stand-in. A follow-up probe (extending the same standalone harness to 12
frames) found stereo-valid (positive-disparity) landmark counts per frame
sitting right at/just above `min_landmarks_for_pose=3`, so some individual
transitions fail RANSAC by small-N chance and — because `replay_demo`
requires the `from` keyframe of a transition to already be in the pose
graph — a single failed transition cascades and stops the rest of the
50-keyframe chain from being added. This is a legitimate frontier-tuning
limitation of the existing `harris_corner`/`PatchMatcher` parameters
against a small `max_corners=60` cap on this specific repetitive scene —
the same class of finding already on record for the HoloOcean real bag
(also partial, also honestly reported, also flagged as future retuning
work), not something this workstream's own scope (prove the schema/
pipeline work on real external data) needed to chase further. Ground truth
and IMU were deliberately not converted (see `euroc_converter.py`'s own
docstring): `stereo_landmark_vo` doesn't need GT to run, and this repo has
no IMU consumer yet (confirmed by the P3 current-state audit).

**Verified:** full C++ suite 142/142 (unaffected — this workstream is
Python-only), lint clean, `adapters/datasets` pytest 5/5
(`rosbag1_reader` + `undistort` unit tests).

**Files:**
- `adapters/datasets/pyproject.toml`, `uw_dataset_adapter/{__init__.py,
  canonical_writer.py, rosbag1_reader.py, undistort.py, euroc_converter.py}`
- `adapters/datasets/tests/{test_rosbag1_reader.py, test_undistort.py}`
- `configs/rig/euroc_mh01.yaml`, `configs/experiment/euroc_mh01_vo.yaml`
- `tools/codegen/gen_py.sh` — parameterized to accept an output dir
  (backward compatible; still defaults to `adapters/holoocean`'s path) so
  `adapters/datasets` could generate its own `schema_pb2/` the same way

**Acceptance:** met — one public dataset sequence (EuRoC MH_01_easy)
converts to canonical MCAP and replays through `apps/replay_demo`
producing real relative-pose factors from real external imagery, not just
"doesn't crash."

## Workstream B4: Canonical recording spec for the three fixed scenarios — DONE

Implemented 2026-08-22. Bigger than "add three topic writers" — checking
`schemas/proto/uw/domain/` first found sonar already had a message
(`SonarFrame`, just never wired into `record_session.py`), but **IMU and
DVL had no raw-sample message type at all** (`ImuNoiseModel` is calibration
params, `ImuPreintegrationMeasurement` is a frontend-produced summary that
doesn't exist yet — neither is a raw per-tick reading). Per `CLAUDE.md`'s
protobuf-is-the-only-source-of-truth rule, added two new domain messages
rather than inventing a parallel Python struct:

- `schemas/proto/uw/domain/imu.proto` (`ImuSample`): header + `repeated
  double linear_acceleration_mps2`/`angular_velocity_radps` (3 entries
  each) + optional `has_bias`/`bias_*` fields — field shape taken directly
  from HoloOcean's real `IMUSensor` output (`holoocean.sensors.IMUSensor`:
  a (2,3) or (4,3) array), not invented.
- `schemas/proto/uw/domain/dvl.proto` (`DvlSample`): header + `velocity_mps`
  (3 entries) + optional `has_beam_ranges`/`beam_ranges_m` (up to 4) —
  matches HoloOcean's real `DVLSensor` output shape (3 or 3+N values).
- Both follow `SonarFrame`/`ImageFrame`'s existing "raw sensor frame"
  style (`ObservationHeader` first, plain fields after) rather than
  `measurement.proto`'s post-frontend payload style — these are raw
  readings, not measurements yet. 2 new C++ contract round-trip tests in
  `tests/contracts/domain_contract_test.cpp`
  (`ImuSampleRoundTripsWithAndWithoutBias`,
  `DvlSampleRoundTripsWithAndWithoutBeamRanges`), matching
  `ImageFrameRoundTripsWithCanonicalHeader`'s existing pattern. No C++
  codegen script needed — `cmake/UwProtobuf.cmake` globs `*.proto` and
  regenerates on reconfigure.

**Python wiring:** three new conversion modules (matching
`camera_conversion.py`'s established pattern — take the generated pb2
modules as explicit parameters, no hard import): `imu_conversion.py`,
`dvl_conversion.py`, `sonar_conversion.py`. The sonar converter is
deliberately algorithmically identical to the already real-HoloOcean-
verified C++ path
(`src/adapters/holoocean_ros_bridge_sonar_frame_provider.cpp`'s
`PushImagingSonar`) — same column-mirror-then-quantize-to-uint8 handling of
HoloOcean's float32-in-[0,1] intensity image, same ascending-azimuth
construction — rather than re-deriving the same real-sensor quirks from
scratch. `record_session.py`'s `_write_keyframe` now writes `/raw/
sonar_frame`, `/raw/imu`, `/raw/dvl` whenever each sensor is present on a
camera-bearing tick (independently of each other — none gates on the
others). The growing list of pb2-module parameters (now 8) was bundled
into a `SchemaModules` dataclass to keep `_write_keyframe`/
`record_frames`/`record_session`'s signatures from degrading into an
ever-longer positional list — a mechanical refactor of every call site in
this file, not a behavior change. `canonical_writer.py` needed NO changes
— it's already fully generic over any protobuf `Message` via reflection.
Also added a `--command` CLI flag to `main()` (the underlying
`record_session()` already accepted a `command=` override; `main()` just
never exposed it), since the recording spec doc below needs a way to
actually select a trajectory shape from the command line.

**A real, honestly-documented limitation found while writing this:**
`_write_keyframe` only writes ANY message (camera, GT, depth, and now
sonar/IMU/DVL) on ticks where both camera keys are present — a
pre-existing constraint (this repo's whole architecture keys evidence off
camera keyframes), not something this workstream introduced. But it means
sonar/IMU/DVL are effectively downsampled to camera-keyframe rate, not
recorded at their own (likely higher) native rate — e.g. a 100-200Hz real
IMU would only contribute the one sample nearest each camera tick, not a
real preintegration-ready stream. Documented explicitly in both
`record_session.py`'s module docstring and the recording spec doc (below)
rather than left for someone to discover the hard way; flagged as
follow-up work for whenever P2's IMU preintegration frontend actually
needs a real-rate stream.

**The recording spec:** `docs/uw-slam-real-recording-spec-2026-08-22.md`
(new top-level doc, matching this repo's existing `docs/uw-slam-*-2026-*`
naming — `configs/scenario/*.yaml` was checked and rejected as a home
since that's specifically `synth_bag_gen`'s synthetic-scene parameters, a
different concept from a real recording session's specification). Defines
all three scenarios as the same `OpenWater-HoveringCamera` HoloOcean
scenario with different constant 8-value thruster commands
(`straight_line`: existing symmetric default; `turn`/`small_loop`: the
same asymmetric command, distinguished only by duration — enough ticks for
a partial heading change vs. enough to close a full loop). Explicitly
flagged as unverified starting points, not calibrated constants — this
sandbox has no live HoloOcean to close the loop on actual turning radius.

**Verified (real, not simulated):**
- `cmake --build build` clean; `ctest --test-dir build`: **144/144** (up
  from 142 — the 2 new contract tests).
- `tools/lint/check_no_ros_in_core.sh`: OK.
- `adapters/holoocean` pytest: **49/49** (up from 35 — 2 extended/new
  `record_session` tests plus 12 new direct unit tests across
  `test_imu_conversion.py`/`test_dvl_conversion.py`/
  `test_sonar_conversion.py`, including a closed-form check on the sonar
  mirror+quantize math using hand-computable pixel values).
- `python -m uw_holoocean_adapter.record_session --help` confirmed the new
  `--command` flag parses correctly.

**Not done, and explicitly out of scope for this workstream:** actually
recording the three scenarios on the Windows HoloOcean machine (a separate
user action, per this plan's own Track B/C split) — nothing here was
simulated or faked as a substitute for that.

### Original scope (for reference)

**Goal:** Write down, before recording, what "straight line / turn / small
loop, 1–3 minutes, full sensor set" means precisely enough that the Windows
HoloOcean recording session (a user action, not something this session
performs) produces a bag Track B5's audit tool and Track C's baseline harness
can actually consume. Also extend `record_session.py` to write the topics the
spec requires — sonar and IMU/DVL currently have zero write calls.

**Files:**
- New: a short spec doc (this plan doesn't prescribe its exact path/format —
  whoever starts this should check whether `docs/` already has a natural home
  for scenario specs before creating a new one)
- Modify: `adapters/holoocean/uw_holoocean_adapter/record_session.py` — add
  sonar/IMU/DVL topic writers alongside the existing stereo+depth ones,
  following the same pattern as `_write_keyframe`'s existing camera/depth
  calls
- Modify: `adapters/holoocean/uw_holoocean_adapter/canonical_writer.py` if
  it needs new message-type support for sonar/IMU/DVL topics
- Test: `adapters/holoocean/tests/test_record_session.py` — extend with
  sonar/IMU/DVL coverage matching the existing stereo/depth test pattern

**Acceptance:** `record_session.py` can write a bag containing camera, sonar,
IMU, DVL, depth, and GT pose topics (verified against a HoloOcean mock/stub
in tests, since this sandbox has no live HoloOcean) — actually recording a
real 1–3 minute scenario still requires the user's Windows machine.

## Workstream B5: Topic/time/TF/calibration audit tool — DONE

Implemented 2026-08-22: `apps/bag_audit.cpp` (C++, registered in
`cmake/Applications.cmake`) — chosen over Python because `uw::runtime::
ReadMcapMessages<T>` and `uw::runtime::LoadRigConfig` already exist and
handle exactly the MCAP/rig parsing this tool needs; reusing them beat
re-implementing either in Python for no real benefit. The pure check logic
(timestamp monotonicity, clock-domain collection, TF-chain BFS) was pulled
out into `include/runtime/bag_audit_checks.hpp` + `src/runtime/
bag_audit_checks.cpp` (wired into the existing `runtime` CMake target) so
it's unit-testable without needing a real MCAP file — 10 new tests in
`tests/runtime/bag_audit_checks_test.cpp`.

**What it checks**, against a `--require`-composable expectation profile
(camera left/right always required; every other topic optional unless
named, since `apps/synth_bag_gen` and a real B4 recording legitimately
expect different topic sets):
1. Topic presence — missing topics reported by name, not silently skipped.
2. Rate plausibility — `/gt/state`/`/evidence/depth`/`/raw/imu`/`/raw/dvl`
   bounded by camera-keyframe count (each is written at most once per
   camera tick in both `synth_bag_gen` and `record_session.py`).
   `/raw/sonar_frame` is deliberately **excluded** from that bound — see
   the real finding below.
3. `capture_time`/`receive_time` populated and monotonic per topic (only
   checked on the four topics that actually carry a full
   `ObservationHeader` — `StateSnapshot` has only a bare `capture_timestamp`,
   `MeasurementEvidence` has no timestamp field at all; this is a real
   schema constraint the tool respects rather than assuming uniformity).
4. Clock-domain consistency across all header-bearing topics.
5. TF-chain resolution: BFS from `base_link` over the rig's `frame_tree`,
   for every `sensor_frame` referenced by any present message.

**A real finding while building this, not a hypothetical:** running the
tool against a fresh synthetic bag surfaced two genuine things, not tool
bugs to paper over —
- My first draft's rate check assumed every non-camera topic is bounded by
  camera-keyframe count. Wrong for `/raw/sonar_frame`: `apps/
  synth_bag_gen.cpp` deliberately writes one `SonarFrame` per in-range
  target per keyframe (a documented v1 simplification — see
  `BuildSyntheticSonarFrame`'s call site), so a synthetic bag legitimately
  has *more* sonar messages than keyframes. Fixed by excluding
  `/raw/sonar_frame` from that bound, not by weakening the bound itself.
- `receive_time` is **never populated** by `apps/synth_bag_gen.cpp` on any
  topic (confirmed by grep — zero `receive_time` writes in the whole
  file), and the same is true of the real 76MB HoloOcean bag. This means
  the "should pass clean" assumption in this workstream's original scope
  note (below) doesn't actually hold — a definitionally-synthetic bag
  still fails the receive_time-populated check, correctly. That's the
  audit tool working as designed, not a bug: `receive_time` really is
  unset repo-wide today. Fixing that is out of B5's own scope (B5 audits,
  it doesn't patch every producer it finds a gap in) — left as a real,
  documented follow-up for whoever next touches `synth_bag_gen.cpp`/
  `record_session.py`'s timestamp population.

**Verified (all real, not simulated):**
- Fresh synthetic bag (`synth_bag_gen` against `synthetic_smoke.yaml`),
  audited with `--require /raw/sonar_frame --require /gt/state --require
  /evidence/depth`: topic presence, rate-plausibility, and TF-chain checks
  all pass; only the pre-existing `receive_time` gap above fails (exit 1) —
  not the "fully clean" result originally assumed, but the *correct* one.
- Real `/tmp/real_session.mcap` (78MB, stereo+pose+depth only), audited
  with `--require /raw/sonar_frame --require /raw/imu --require /raw/dvl`:
  correctly reports all three as `MISSING (required)` — this workstream's
  literal acceptance criterion, met.
- A hand-built one-message probe bag containing a `/raw/dvl` message
  (`sensor_frame = "dvl_link"`), audited against `configs/rig/
  example_auv_real_camera.yaml`: TF-chain check correctly flags
  `dvl_link` as unresolved — the exact, already-known gap B4 found (that
  rig has `camera_left_link`/`camera_right_link`/`sonar_link`/`imu_link`
  edges but no `dvl_link` edge), now caught mechanically instead of by
  manual inspection.
- `cmake --build`: clean. `ctest --test-dir build`: 154/154 (up from 144 —
  10 new `BagAuditChecks` tests). `tools/lint/check_no_ros_in_core.sh`:
  OK. `adapters/holoocean` pytest: 49/49 (unaffected — this workstream
  touched no Python).

**Not done, explicitly out of scope:** fixing the `receive_time` gap this
tool found (that's a producer-side fix, not this workstream's job); a
generic/unknown-topic audit mode (this tool only knows the seven canonical
topics, by design — see the app's own header comment).

### Original scope (for reference)

**Goal:** A tool that, given a recorded canonical MCAP bag and the rig config
it claims to match, checks: every expected topic is present at a plausible
rate; capture-time and receive-time are both populated and monotonic per
topic; the clock domain is internally consistent across topics (no silent
mixed sim-time/wall-time bag); the TF chain implied by the rig config actually
resolves for every frame the bag's messages claim to be in.

**Files:**
- New: likely a small standalone tool (C++ under `apps/` reading MCAP + the
  domain protos, mirroring how `apps/replay_demo` already reads bags — or
  Python under `adapters/holoocean/` if the checks are easier to express
  there; decide once B4's actual topic set is finalized, since the audit
  tool's shape follows the recording spec, not the other way around)
- Test: run against `apps/synth_bag_gen`'s synthetic output first (should
  pass clean — a synthetic bag is definitionally well-formed) to validate the
  tool itself before pointing it at a real, potentially-messy recording

**Acceptance:** running the audit tool against the existing 76 MB real
HoloOcean bag mentioned in the roadmap's 2.4 section (stereo+pose+depth only,
no sonar/IMU/DVL) correctly reports which expected topics are *missing* —
that's the tool proving it can distinguish "present and consistent" from
"absent," using data that already exists today, before B4's fuller recording
lands.

## Workstream C6: Four-baseline ablation harness

**Blocked on:** B4's extended recorder actually producing a bag with sonar +
IMU + DVL + depth + GT (the current 76 MB real bag has neither sonar nor IMU,
so a "+sonar" or full-fusion baseline can't be computed from it — this is a
hard blocker, not a nice-to-have).

**Goal:** Given one real bag, run VIO-only / VIO+depth / VIO+sonar / full
fusion as four experiment configs against the same data, and produce one
comparison table (input counts, factor counts, solver status, ATE/RPE, map
point count, effective coverage, P95 latency — the exact column set the
roadmap's "最近两周建议动作" item 6 already specifies).

**Files:**
- New: four `configs/experiment/*.yaml` variants (ablating which factor
  builders/evidence sources are active) — this is exactly what Workstream
  A1 needs to already be done for, since these variants only mean what they
  claim if config-driven selection is actually real by then
- New: a small comparison-table generator (script or app) consuming each
  variant's `RunManifest` + evaluator output

**Acceptance:** one table, four rows, all four produced from the same input
bag and same evaluator, matching roadmap P1's acceptance criterion "所有
baseline 结果和产物由 manifest 关联并纳入固定回归数据集."

---

## Explicit non-goals for this plan

- Does not attempt P2 (Ceres/GTSAM, sliding window, IMU preintegration) —
  P1's VIO path stays the existing black_box_vio/stereo_landmark_vo estimator
  modes; P1 only makes their *inputs* real, not the estimator itself.
- Does not attempt P3 (TSDF/surfel/occupancy, ROS2 online scheduler) — out of
  scope per roadmap phase boundary.
- Does not decide the GPLv3 boundary question (roadmap section 10's risk
  table item) — that's a licensing/business decision for the user, not an
  engineering task this plan can schedule.
- Does not solve team formation — roadmap section 8/10 already flag that all
  phase timelines assume 3–5 engineers and the repo currently has one author;
  this plan's workstreams are written to be pickable independently by
  whoever is available, but doesn't change the calendar-time implications of
  running them with fewer people.

Repository policy carried over from the roadmap doc and `CLAUDE.md`: do not
create git commits unless the user explicitly asks.
