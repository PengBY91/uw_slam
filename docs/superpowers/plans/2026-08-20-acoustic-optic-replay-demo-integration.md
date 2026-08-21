# Acoustic-Optic Replay Demo Integration Plan

**Goal:** Wire the 6-plan acoustic-optic series into `apps/replay_demo` for
real — until now every component was proven correct in isolation (unit
tests) and end-to-end only against synthetic in-process trials
(`acoustic_optic_scenario_matrix`), never against an actual MCAP bag inside
the app that owns the pose graph. This plan makes `replay_demo` construct
and run `StereoOpticalDepthFrontend` → `SonarCfarFrontend` (reused, not
duplicated) → `AcousticOpticDepthFusionFrontend` →
`BuildMapEvidenceFromFusedDepth` per keyframe, using real per-keyframe
camera images that `synth_bag_gen` must now also emit.

**Explicit non-goal, stated up front:** this plan does **not** feed dense
depth into the pose graph as a new factor type. `PoseGraphProblem`/
`GaussNewtonSolver`/the trajectory ATE are **completely unaffected** —
acoustic-optic output is stored as a third, parallel `MapEvidence` bucket
(alongside the existing "landmarks" bucket), exactly matching plan 6's own
scope ("mapping handoff", not "estimation feedback"). Turning dense depth
into a solver factor would be a materially different, much larger
undertaking (new residual model, information weighting, no existing
`factor_builders/` precedent for a dense/multi-point measurement) — out of
scope here and not implied to be a small follow-up.

**Regression discipline (this plan's main risk):** `apps/tools/synth_bag_gen`
and `apps/replay_demo` are the two most battle-tested binaries in the repo —
`tests/l2_replay/determinism_test.sh` asserts their trajectory output is
byte-identical across reruns, and it invokes both **without** `--experiment`
(no rig loaded, matching today's behavior). Every change in this plan is
gated behind "a rig with cameras was actually loaded via `--experiment`" —
the no-`--experiment` code path must be provably untouched. Task 3 verifies
this explicitly before touching anything else is considered done.

**A real, honest finding this plan does not paper over:** direct computation
against the existing `configs/scenario/synthetic_smoke.yaml` +
`configs/rig/example_auv.yaml` shows that **no keyframe ever has a sonar
target inside the camera's (much narrower) field of view** — the camera's
half-FOV is ~0.65 rad vs. the sonar's synthetic 3.0 rad half-FOV. Running
this plan's integration against the existing default scenario will
therefore correctly show zero accepted acoustic-optic associations (optical
depth still gets computed and stored; sonar just never geometrically
matches anything within camera view). This is real geometry, not a bug —
this plan does not alter `configs/scenario/synthetic_smoke.yaml` or
`configs/rig/example_auv.yaml` to manufacture a nicer number. Instead, Task
4 adds a **new, additive** `configs/scenario/acoustic_optic_demo.yaml` +
`configs/experiment/acoustic_optic_demo.yaml` with one target placed inside
the camera's view, so there is also a genuine "it detects and corrects a
pixel" demonstration — without touching the config the regression test
depends on.

**Tech Stack:** C++17, Eigen, GoographSolver unchanged. Reuses every
acoustic-optic component from plans 2-6 as-is; no new algorithm code.

## File map

### Create

- `configs/scenario/acoustic_optic_demo.yaml`
- `configs/experiment/acoustic_optic_demo.yaml`

### Modify

- `apps/tools/synth_bag_gen/src/main.cpp` — emit per-keyframe stereo images when a rig is loaded.
- `apps/tools/synth_bag_gen/CMakeLists.txt` — link the new dependencies.
- `apps/replay_demo/src/main.cpp` — run the acoustic-optic pipeline per keyframe when a rig is loaded.
- `apps/replay_demo/CMakeLists.txt` — link the new dependencies.
- `docs/uw-slam-codebase-reference-2026-08-18.md` — document the integration and its real numbers.
- `README.md` — correct the "没有真正运行光学前端" line now that `replay_demo` can, conditionally.

Repository policy: do not create git commits unless the user explicitly
authorizes them. Each task ends with a review checkpoint, not a commit.

---

### Task 1: `synth_bag_gen` emits per-keyframe stereo images (rig-gated)

**Files:** `apps/tools/synth_bag_gen/src/main.cpp`, `CMakeLists.txt`

- [ ] **Step 1: Load the rig only when `--experiment` is given**

In `main()`, alongside the existing `ApplyScenarioConfig(config.scenario, opt)` call inside
the `--experiment` first pass, capture the rig too:

```cpp
std::optional<uw::domain::RigCalibrationSnapshot> rig;
// ... inside the existing `if (std::string(argv[i]) == "--experiment")` block:
      const auto config = uw::runtime::LoadExperimentConfig(argv[++i]);
      ApplyScenarioConfig(config.scenario, opt);
      if (config.rig.cameras_size() > 0) rig = config.rig;
```

- [ ] **Step 2: Add a self-contained stereo-pair synthesizer**

Mirrors `acoustic_optic_scenario_matrix/src/scenarios.cpp`'s proven paint-background-then-
paste-target technique (same precedent as that plan's own bug-fix — see its `MakeStereoPair`
header comment for why the naive per-pixel approach is wrong), but simplified: no noise, no
degraded-texture variants (this app has no depth-accuracy scoring to protect, unlike
`acoustic_optic_scenario_matrix`), and driven by the REAL per-keyframe trajectory pose
instead of a single static scene. Add near the other helper functions:

```cpp
namespace {
// ... existing helpers ...

constexpr uint32_t kCameraWidth = 640;
constexpr uint32_t kCameraHeight = 480;

uint8_t StereoTexture(int u, int v) { return static_cast<uint8_t>((u * 131 + v * 67 + 19) % 256); }

// Finds the FrameEdge for `child_frame`; returns Pose3::Identity() if absent
// (same fallback convention as core/sensor_models::FindEdgePose-style
// helpers used throughout algorithms/frontends/acoustic_optic_*).
Pose3 FindRigEdgePose(const uw::domain::RigCalibrationSnapshot& rig, const std::string& child_frame) {
  for (const auto& edge : rig.frame_tree()) {
    if (edge.child_frame().value() == child_frame) return Pose3::FromProto(edge.transform());
  }
  return Pose3::Identity();
}

const uw::domain::CameraIntrinsics* FindRigCamera(const uw::domain::RigCalibrationSnapshot& rig,
                                                   const std::string& sensor_id) {
  for (const auto& camera : rig.cameras()) {
    if (camera.sensor_id().value() == sensor_id) return &camera;
  }
  return nullptr;
}

// Builds one keyframe's stereo pair: a uniform far background (disparity
// picked for a fixed far depth) with, IF `target_camera_optical` is
// non-nullopt (the nearest in-camera-view target, already resolved by the
// caller), a real target patch pasted at its true per-keyframe depth —
// exercised through the actual StereoOpticalDepthFrontend block matcher
// downstream, not fabricated. No noise/degradation: this app has no
// depth-accuracy scoring to protect (unlike acoustic_optic_scenario_matrix,
// plan 5), just needs a working, honest scene for the pipeline to run on.
std::pair<uw::domain::ImageFrame, uw::domain::ImageFrame> BuildStereoPair(
    const uw::sensor_models::StereoGeometry& stereo_geometry,
    const std::optional<Eigen::Vector3d>& target_camera_optical, uint64_t t_ns) {
  constexpr double kBackgroundDepthM = 15.0;
  constexpr int kPatchHalfSize = 12;
  const int background_disparity_px = std::max(
      1, static_cast<int>(std::lround(stereo_geometry.left.fx * stereo_geometry.baseline_m / kBackgroundDepthM)));

  std::string left_pixels(static_cast<std::size_t>(kCameraWidth) * kCameraHeight, '\0');
  std::string right_pixels(static_cast<std::size_t>(kCameraWidth) * kCameraHeight, '\0');
  for (uint32_t v = 0; v < kCameraHeight; ++v) {
    for (uint32_t u = 0; u < kCameraWidth; ++u) {
      left_pixels[static_cast<std::size_t>(v) * kCameraWidth + u] =
          static_cast<char>(StereoTexture(static_cast<int>(u), static_cast<int>(v)));
      right_pixels[static_cast<std::size_t>(v) * kCameraWidth + u] =
          static_cast<char>(StereoTexture(static_cast<int>(u) + background_disparity_px, static_cast<int>(v)));
    }
  }

  if (target_camera_optical.has_value() && target_camera_optical->z() > 0.5) {
    const Eigen::Vector2d pixel = stereo_geometry.left.Project(*target_camera_optical);
    const int center_u = static_cast<int>(std::lround(pixel.x()));
    const int center_v = static_cast<int>(std::lround(pixel.y()));
    const int target_disparity_px = std::max(
        1, static_cast<int>(std::lround(stereo_geometry.left.fx * stereo_geometry.baseline_m /
                                        target_camera_optical->z())));
    for (int dv = -kPatchHalfSize; dv <= kPatchHalfSize; ++dv) {
      const int v = center_v + dv;
      if (v < 0 || v >= static_cast<int>(kCameraHeight)) continue;
      for (int du = -kPatchHalfSize; du <= kPatchHalfSize; ++du) {
        const int u_left = center_u + du;
        const int u_right = u_left - target_disparity_px;
        if (u_left < 0 || u_left >= static_cast<int>(kCameraWidth) || u_right < 0 ||
            u_right >= static_cast<int>(kCameraWidth)) {
          continue;
        }
        right_pixels[static_cast<std::size_t>(v) * kCameraWidth + static_cast<std::size_t>(u_right)] =
            static_cast<char>(StereoTexture(u_left, v));
      }
    }
  }

  auto make_frame = [&](const std::string& frame_name, std::string pixels) {
    uw::domain::ImageFrame image;
    image.mutable_header()->mutable_sensor_frame()->set_value(frame_name);
    image.mutable_header()->mutable_sensor_id()->set_value(
        frame_name == "camera_left_link" ? "camera_left" : "camera_right");
    image.mutable_header()->mutable_capture_time()->set_seconds(static_cast<int64_t>(t_ns / 1'000'000'000ULL));
    image.mutable_header()->mutable_capture_time()->set_nanos(static_cast<int32_t>(t_ns % 1'000'000'000ULL));
    image.mutable_header()->set_clock_domain(uw::domain::CLOCK_DOMAIN_SIMULATION);
    image.mutable_header()->set_validity(uw::domain::ObservationHeader::VALIDITY_OK);
    image.mutable_header()->set_provenance("synth_bag_gen_v1");
    image.set_width(kCameraWidth);
    image.set_height(kCameraHeight);
    image.set_row_stride_bytes(kCameraWidth);
    image.set_encoding(uw::domain::ImageFrame::IMAGE_ENCODING_MONO8);
    image.set_pixel_data(std::move(pixels));
    image.set_is_rectified(true);
    return image;
  };
  return {make_frame("camera_left_link", std::move(left_pixels)),
          make_frame("camera_right_link", std::move(right_pixels))};
}

}  // namespace
```

- [ ] **Step 3: Emit the pair per keyframe, when a target is in camera view**

Inside the main per-keyframe loop, after the existing sonar-per-target block, add (only
compiled/reached when `rig.has_value()`):

```cpp
    if (rig.has_value()) {
      const auto* left_intrinsics = FindRigCamera(*rig, "camera_left");
      const auto* right_intrinsics = FindRigCamera(*rig, "camera_right");
      if (left_intrinsics != nullptr && right_intrinsics != nullptr) {
        const auto stereo_geometry = uw::sensor_models::StereoGeometry::Resolve(
            *rig, "camera_left", "camera_left_link", "camera_right", "camera_right_link");
        if (stereo_geometry.valid) {
          const Pose3 camera_pose = FindRigEdgePose(*rig, "camera_left_link");
          std::optional<Eigen::Vector3d> nearest_visible_target;
          double nearest_visible_range = std::numeric_limits<double>::max();
          for (const auto& target : targets) {
            const Eigen::Vector3d local_body = trajectory[i].Inverse().Apply(target);
            const Eigen::Vector3d local_camera_body = camera_pose.Inverse().Apply(local_body);
            const Eigen::Vector3d local_optical =
                uw::sensor_models::OpticalFromBodyRotation() * local_camera_body;
            if (local_optical.z() <= 0.5) continue;  // behind or too close to the camera
            const Eigen::Vector2d pixel = stereo_geometry.left.Project(local_optical);
            if (pixel.x() < 0 || pixel.x() >= stereo_geometry.left.width || pixel.y() < 0 ||
                pixel.y() >= stereo_geometry.left.height) {
              continue;  // outside the camera's (narrower than sonar) field of view
            }
            if (local_optical.z() < nearest_visible_range) {
              nearest_visible_range = local_optical.z();
              nearest_visible_target = local_optical;
            }
          }
          const auto [left_frame, right_frame] = BuildStereoPair(stereo_geometry, nearest_visible_target, t_ns);
          writer.WriteMessage("/raw/camera/left", t_ns, left_frame);
          writer.WriteMessage("/raw/camera/right", t_ns, right_frame);
        }
      }
    }
```

`trajectory[i].Inverse().Apply(target)` and `t_ns` already exist earlier in this loop
iteration (reused, not recomputed) — this block goes right after the existing sonar-per-
target loop, still inside the `for (int i = 0; i < opt.num_keyframes; ++i)` body.

- [ ] **Step 4: Includes and CMake**

Add to `synth_bag_gen/src/main.cpp`'s includes: `#include "uw/sensor_models/camera_model.hpp"`
and `<limits>`/`<optional>`/`<utility>` as needed.

`apps/tools/synth_bag_gen/CMakeLists.txt` already links `uw_sensor_models` (unchanged — the
new code only uses types already exposed by that target, no new library dependency).

- [ ] **Step 5: Build and verify the no-`--experiment` path is untouched**

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH="$HOME/miniconda3/envs/uw_slam_build"
cmake --build build --target synth_bag_gen -j"$(nproc)"
./build/apps/tools/synth_bag_gen/synth_bag_gen --out /tmp/no_rig.mcap --seed 7
```

Expected: exits 0, prints the same `wrote N keyframes to ...` message as before — no camera
topics are written (rig is `std::nullopt` without `--experiment`), so this bag is byte-
identical in content to what today's `synth_bag_gen` would produce (mcap header
timestamps aside — the determinism test in Task 3 checks this precisely, not this step).

- [ ] **Step 6: Build and verify the `--experiment` path produces camera topics**

```bash
./build/apps/tools/synth_bag_gen/synth_bag_gen --out /tmp/with_rig.mcap \
  --experiment configs/experiment/synthetic_smoke.yaml
```

Expected: exits 0. A quick manual check (e.g. a small `python -c` using the `mcap` package,
or just trusting Task 2's `replay_demo` read-back) confirms `/raw/camera/left`/
`/raw/camera/right` topics exist with 12 messages each (one per keyframe).

- [ ] **Step 7: Review checkpoint**

Confirm nothing outside the new `if (rig.has_value())` block changed — `git diff` should
show only additive code, no modified line in the existing sonar/relative-pose/depth writing
logic. Do not commit without explicit authorization.

---

### Task 2: `replay_demo` runs the acoustic-optic pipeline per keyframe (rig-gated)

**Files:** `apps/replay_demo/src/main.cpp`, `CMakeLists.txt`

- [ ] **Step 1: Load the rig only when `--experiment` is given**

Where `defaults`/`write_run_manifest` are currently set from `LoadExperimentConfig`, also
capture the rig:

```cpp
  std::optional<uw::domain::RigCalibrationSnapshot> rig;
  if (!opt.experiment_path.empty()) {
    const auto config = uw::runtime::LoadExperimentConfig(opt.experiment_path);
    defaults = config.defaults;
    write_run_manifest = config.write_run_manifest;
    if (config.rig.cameras_size() > 0) rig = config.rig;
    std::cout << "loaded experiment config: " << opt.experiment_path << " (sonar_frontend="
              << config.sonar_frontend << ", estimator_mode=" << config.estimator_mode << ")\n";
  }
```

- [ ] **Step 2: Add the acoustic-optic pass, after the existing per-keyframe pose-commit loop**

Insert after the `for (int i = 0; ...) { ... submap_manager.UpdateKeyframePose(kf_id, pose); ... }`
loop (poses must already be committed to `submap_manager` before this runs, since
`BuildMapEvidenceFromFusedDepth`'s output is only meaningful once `WorldPointsForKeyframe`
can transform it) and before the ATE computation:

```cpp
  int num_keyframes_with_camera = 0;
  int num_acoustic_optic_accepted = 0;
  int num_acoustic_optic_ambiguous = 0;
  int num_acoustic_optic_conflict = 0;
  int num_acoustic_optic_rejected = 0;
  int num_map_evidence_points = 0;
  if (rig.has_value()) {
    std::unordered_map<std::string, uw::domain::ImageFrame> left_by_kf, right_by_kf;
    std::unordered_map<std::string, uw::domain::SonarFrame> sonar_by_kf;
    uw::runtime::ReadMcapMessages<uw::domain::ImageFrame>(
        opt.bag_path, "/raw/camera/left",
        [&](uint64_t, const uw::domain::ImageFrame& f) { left_by_kf[f.header().sensor_id().value() == "camera_left" ? f.header().observation_id().value() : ""] = f; });
    // NOTE: synth_bag_gen doesn't set observation_id on camera frames (only sensor_frame) —
    // key by capture_time's keyframe index instead. See Step 2b below for the corrected keying.

    uw::frontends::StereoOpticalDepthFrontendParams stereo_params;
    uw::frontends::StereoOpticalDepthFrontend stereo_frontend(stereo_params);
    uw::frontends::AcousticOpticDepthFusionParams fusion_params;
    uw::frontends::AcousticOpticDepthFusionFrontend fusion_frontend(fusion_params);
    uw::mapping::AcousticOpticMapBridgeParams bridge_params;

    for (const auto& kf_id : problem.KeyframeOrder()) {
      auto left_it = left_by_kf.find(kf_id);
      auto right_it = right_by_kf.find(kf_id);
      if (left_it == left_by_kf.end() || right_it == right_by_kf.end()) continue;
      ++num_keyframes_with_camera;

      uw::runtime::SynchronizerParams sync_params;
      const auto sonar_it = sonar_by_kf.find(kf_id);
      const uw::domain::SonarFrame empty_sonar;
      const auto sync_bundle = uw::runtime::SynchronizeAcousticOptic(
          left_it->second, std::optional<uw::domain::ImageFrame>(right_it->second),
          sonar_it != sonar_by_kf.end() ? sonar_it->second : empty_sonar, *rig, sync_params);

      uw::measurement_api::CameraFrameBundle bundle;
      bundle.primary = left_it->second;
      bundle.secondary = right_it->second;
      const auto optical_evidence = stereo_frontend.Process(bundle, *rig);
      if (!optical_evidence.has_value()) continue;

      uw::domain::HypothesisSet sonar_hypotheses;
      if (sonar_it != sonar_by_kf.end()) {
        sonar_hypotheses = sonar_frontend.ProcessSonarFrame(sonar_it->second);
      }

      const double time_delta = sync_bundle.has_value() ? sync_bundle->max_pairwise_time_delta_s : 0.0;
      const auto fused_result = fusion_frontend.Fuse(sonar_hypotheses, *optical_evidence, *rig, time_delta);
      if (!fused_result.has_value()) continue;

      const auto& fused = uw::domain::GetPayload<uw::domain::FusedDepthMeasurement>(fused_result->fused_evidence);
      if (fused.associations_size() > 0) {
        switch (fused.associations(0).status()) {
          case uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_ACCEPTED: ++num_acoustic_optic_accepted; break;
          case uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_AMBIGUOUS: ++num_acoustic_optic_ambiguous; break;
          case uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_CONFLICT: ++num_acoustic_optic_conflict; break;
          case uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_REJECTED: ++num_acoustic_optic_rejected; break;
          default: break;
        }
      }

      const auto state_version = static_cast<uint64_t>(&kf_id - &problem.KeyframeOrder()[0]) + 1;
      const auto map_evidence =
          uw::mapping::BuildMapEvidenceFromFusedDepth(fused_result->fused_evidence, *rig, bridge_params, kf_id,
                                                      state_version);
      if (map_evidence.has_value()) {
        num_map_evidence_points += static_cast<int>(map_evidence->geometry_or_occupancy().size() / (3 * sizeof(float)));
        submap_manager.AddMapEvidence(*map_evidence);
      }
    }
    std::cout << "acoustic-optic: " << num_keyframes_with_camera << " keyframes with camera data, "
              << num_acoustic_optic_accepted << " accepted, " << num_acoustic_optic_ambiguous
              << " ambiguous, " << num_acoustic_optic_conflict << " conflict, " << num_acoustic_optic_rejected
              << " rejected, " << num_map_evidence_points << " map evidence points added\n";
  }
```

- [ ] **Step 2b: Fix the keying — camera frames must be looked up by keyframe id**

The Step 2 sketch above has a placeholder bug (flagged deliberately, matching this plan's own
"verify before trusting a first draft" discipline established in every prior plan): `synth_bag_gen`
writes `header.sensor_frame`/`sensor_id`, not `header.observation_id`, on camera frames (see
Task 1 Step 3's `make_frame` — it never calls `mutable_observation_id()`), so keying
`left_by_kf`/`sonar_by_kf` by `observation_id().value()` will find nothing. Fix by keying on
capture-time-derived keyframe index instead, matching `kKeyframeIntervalS` already used
elsewhere in this file:

```cpp
    auto keyframe_id_for_time = [&](const uw::domain::Stamp& capture_time) -> std::string {
      const double t_s = uw::domain::ToSeconds(capture_time);
      const int index = static_cast<int>(std::lround(t_s / kKeyframeIntervalS));
      return "kf" + std::to_string(index);
    };
    uw::runtime::ReadMcapMessages<uw::domain::ImageFrame>(
        opt.bag_path, "/raw/camera/left",
        [&](uint64_t, const uw::domain::ImageFrame& f) { left_by_kf[keyframe_id_for_time(f.header().capture_time())] = f; });
    uw::runtime::ReadMcapMessages<uw::domain::ImageFrame>(
        opt.bag_path, "/raw/camera/right",
        [&](uint64_t, const uw::domain::ImageFrame& f) { right_by_kf[keyframe_id_for_time(f.header().capture_time())] = f; });
    uw::runtime::ReadMcapMessages<uw::domain::SonarFrame>(
        opt.bag_path, "/raw/sonar_frame", [&](uint64_t, const uw::domain::SonarFrame& f) {
          const std::string kf_id = f.header().observation_id().value();  // sonar DOES set this already
          if (sonar_by_kf.find(kf_id) == sonar_by_kf.end()) sonar_by_kf[kf_id] = f;  // keep the first (top-1 rule)
        });
```

Replace Step 2's placeholder `left_by_kf`/`sonar_by_kf` population with this corrected
version before building.

- [ ] **Step 3: Includes and CMake**

Add to `replay_demo/src/main.cpp`: `#include "uw/frontends/acoustic_optic_depth_fusion_frontend.hpp"`,
`#include "uw/mapping/acoustic_optic_map_bridge.hpp"`, `#include "uw/runtime/acoustic_optic_synchronizer.hpp"`,
`#include "uw/sensor_models/camera_model.hpp"`, `#include <unordered_map>` (likely already present via
`<unordered_set>`'s sibling — verify, add if missing).

Add to `apps/replay_demo/CMakeLists.txt`'s `target_link_libraries`:

```cmake
  uw_stereo_optical_depth_frontend
  uw_acoustic_optic_depth_fusion
  uw_acoustic_optic_map_bridge
```

- [ ] **Step 4: Build**

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH="$HOME/miniconda3/envs/uw_slam_build"
cmake --build build --target replay_demo -j"$(nproc)"
```

- [ ] **Step 5: Review checkpoint**

Confirm the acoustic-optic block is entirely inside `if (rig.has_value())`, added strictly
after the existing pose-commit loop, and does not modify any line of the existing pose-graph
construction, solving, or ATE computation code. Do not commit without explicit authorization.

---

### Task 3: Regression verification — the no-`--experiment` path is unchanged

**Files:** Verify only.

- [ ] **Step 1: Rerun the existing determinism test unmodified**

```bash
cmake --build build -j"$(nproc)"
ctest --test-dir build -R '^uw_l2_replay_determinism_test$' --output-on-failure
```

Expected: passes, exactly as before this plan. This is the load-bearing regression check —
`determinism_test.sh` calls `synth_bag_gen`/`replay_demo` with no `--experiment`, so it
proves the new code paths (both gated on `rig.has_value()`) never activate and the existing
trajectory output is byte-for-byte unchanged.

- [ ] **Step 2: Rerun the full existing suite**

```bash
ctest --test-dir build --output-on-failure
```

Expected: 100% passed, same count as before this plan (no new test targets added by this
plan — Tasks 1/2 only modify two existing apps).

- [ ] **Step 3: Review checkpoint**

If Step 1 fails, the bug is in this plan's new code leaking into the unconditional path —
do not "fix" it by weakening the determinism test; find and remove the leak. Do not commit
without explicit authorization.

---

### Task 4: A genuine "it detects and corrects" demonstration

**Files:** Create `configs/scenario/acoustic_optic_demo.yaml`, `configs/experiment/acoustic_optic_demo.yaml`.

- [ ] **Step 1: Create the scenario**

One target placed directly ahead of `kf0`'s camera (translation `(0.15,0.06,0)` relative to
`base_link`, per `configs/rig/example_auv.yaml`), so bearing ≈ 0 and it's comfortably inside
both the camera's narrow FOV and the sonar's wide one at the start of the trajectory:

```yaml
# configs/scenario/acoustic_optic_demo.yaml
seed: 42
num_keyframes: 12
radius_m: 8.0
arc_radians: 1.4
depth_m: 12.0

noise:
  relative_pose_noise_m: 0.02
  sonar_range_noise_m: 0.03
  sonar_bearing_noise_rad: 0.01

sonar_targets_world:
  - [5.15, 0.06, -12.0]
```

```yaml
# configs/experiment/acoustic_optic_demo.yaml
rig: rig/example_auv.yaml
scenario: scenario/acoustic_optic_demo.yaml
defaults: defaults/platform.yaml

frontends:
  optical: stereo_depth_frontend_v1
  sonar: sonar_cfar_frontend_v1

factor_builders:
  - relative_pose_v1
  - sonar_range_v1
  - depth_v1

estimator_mode: black_box_vio
map_backend: submap_point_cloud_v1

output:
  trajectory_format: tum
  write_run_manifest: true
```

- [ ] **Step 2: Run it for real**

```bash
build/apps/tools/synth_bag_gen/synth_bag_gen --out /tmp/acoustic_optic_demo.mcap \
  --experiment configs/experiment/acoustic_optic_demo.yaml
build/apps/replay_demo/replay_demo --bag /tmp/acoustic_optic_demo.mcap \
  --experiment configs/experiment/acoustic_optic_demo.yaml --out /tmp/acoustic_optic_demo
```

Expected: the printed `acoustic-optic: ...` line shows `num_keyframes_with_camera >= 1` and
at least one `accepted` (the target is visible and geometrically consistent at kf0). Report
the exact numbers — do not adjust the target position after the fact just to inflate the
accepted count; if it's low, that itself is useful information about how forgiving the
associator's default gates are, matching plan 5's own honest-reporting precedent. Confirm
trajectory ATE (printed) is still in the same ballpark as the existing default scenario
(sub-meter) — a wildly different ATE here would indicate this scenario's own geometry is
broken, not an acoustic-optic issue, and needs fixing before trusting the accepted count.

- [ ] **Step 3: Review checkpoint**

Confirm neither new YAML file's path collides with or shadows an existing one, and that
`configs/scenario/synthetic_smoke.yaml`/`configs/experiment/synthetic_smoke.yaml` have a
zero-line diff (`git diff --stat -- configs/scenario/synthetic_smoke.yaml
configs/experiment/synthetic_smoke.yaml` empty). Do not commit without explicit authorization.

---

### Task 5: Documentation and final verification

**Files:** `docs/uw-slam-codebase-reference-2026-08-18.md`, `README.md`.

- [ ] **Step 1: Update the codebase reference**

Add a note to the `6.11` acoustic_optic_map_bridge section (or a new `6.12`) recording that
`replay_demo` now constructs and runs the full acoustic-optic pipeline per keyframe when a
rig is loaded via `--experiment`, gated so the no-`--experiment` path is unchanged, with the
real numbers from both Task 4's demo scenario and the existing default scenario (0 accepted,
by real geometry, not a bug — explain why in one sentence, don't just assert it). State
plainly this still does not feed dense depth into the pose graph — trajectory ATE is
unaffected by design.

- [ ] **Step 2: Update `README.md`'s now-partially-stale claim**

`README.md` currently states (around line 303) "没有真正运行光学/VIO 前端" — this is now only
true when `--experiment` isn't passed, or targets aren't in camera view. Update it to state
the conditional/gated reality precisely, not to overclaim "wired in" without the caveat.

- [ ] **Step 3: Scan for contradictory capability claims**

```bash
rg -n 'camera|optical|stereo|声光|synchroniz|associat|fusion|posterior|scenario|mapping|MapEvidence' \
  README.md configs/README.md docs/uw-slam-codebase-reference-2026-08-18.md
```

- [ ] **Step 4: Full build and test suite**

```bash
export PATH="$HOME/miniconda3/envs/uw_slam_build/bin:$PATH"
cmake -S . -B build -DCMAKE_PREFIX_PATH="$HOME/miniconda3/envs/uw_slam_build"
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
cd adapters/holoocean && python -m pytest -q && cd ../..
tools/lint/check_no_ros_in_core.sh
```

Expected: 100% passed (same count as before this plan — no new test targets); Python 11/11;
lint clean.

- [ ] **Step 5: Final review checkpoint**

Run `git status --short`, list every changed/new tracked file, compare against this plan's
file map. Report the verification commands, the real printed numbers from both scenarios
(default: 0 accepted by real geometry; demo: whatever Task 4 actually produced), and confirm
`uw_l2_replay_determinism_test` still passes. Do not commit unless the user explicitly
requests it.
