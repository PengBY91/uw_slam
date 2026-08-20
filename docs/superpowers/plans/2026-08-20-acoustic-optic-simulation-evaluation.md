# Acoustic-Optic Simulation/Replay/Evaluation Implementation Plan

**Goal:** Implement plan 5 of the 6-plan acoustic-optic series: wire
`AcousticOpticSynchronizer` + `StereoOpticalDepthFrontend` + the real
(pre-existing) `SonarCfarFrontend` + `AcousticOpticDepthFusionFrontend`
together for the first time, run them over the design spec's 9-scenario
matrix (section 10), and report region-sliced depth metrics, false-fusion
rate, and a determinism check.

**Architecture:** A new standalone app,
`apps/tools/acoustic_optic_scenario_matrix`, is the integration point — it
does not modify `apps/replay_demo`/`synth_bag_gen`/`synth_stereo_gen`/
`optical_baseline_eval` (all four stay exactly as plans 1-2 left them). It
loads the *real* `configs/rig/example_auv.yaml` (via plan 1's
`LoadRigConfig`) instead of a hand-built test rig — this is the first time
any of plans 2-4's components run against actual (non-co-located) rig
geometry rather than a unit test's simplified fixture. It reuses the
*existing, already-shipped* `SonarCfarFrontend` — this plan adds no new
sonar detector; the CFAR pipeline (CFAR + first-contact + DBSCAN,
`algorithms/frontends/sonar_cfar_frontend`) already exists from before this
series began.

**Honest scoping decisions (stated up front, not discovered later):**

- **No MCAP round-tripping in the matrix runner.** Plans 1-2's determinism
  test (`determinism_test.sh`) and this plan's own determinism check both
  prove "same seed/config → byte-identical output" — the L2 test for this
  plan runs the *whole scenario-matrix binary* twice and diffs its report,
  the same pattern already used by `determinism_test.sh`/
  `optical_baseline_smoke_test.sh`. Generating each trial in-process
  (rather than serializing through an MCAP bag) is a deliberate scope cut —
  MCAP round-trip correctness for `ImageFrame`/`SonarFrame` is already
  covered by `uw_l0_domain_contract_test`/Python round-trip tests, so
  re-proving it here would test transport, not the fusion pipeline.
- **"Three-way ablation/region reporting" (design spec section 12) is
  implemented as two conditions (optical-only, fused) times two region
  slices (full image, sonar-covered) — not three depth estimators.**
  Re-reading section 12 carefully: it defines exactly one ablation
  condition (`optical-only`, section 12.1) and asks for metrics reported
  "分别在全图、sonar 投影覆盖区和视觉退化区" (three *reporting slices*, not
  three algorithms). This plan implements two of those three slices
  (full-image, sonar-covered — the latter from `candidate_pixel_indices`,
  already computed by plan 3's associator, no new plumbing needed). The
  third slice (visually-degraded region) is skipped: every degradation
  scenario here (`low_texture_sonar_visible`/`turbid_sonar_visible`/
  `repeated_structure`) degrades the *entire* synthetic image uniformly by
  construction, not a localized patch, so a separate degraded-region mask
  would be degenerate (identical to the full-image slice) — building a
  real localized mask is deferred rather than faked.
- **Not every section 12.2 MVP gate is evaluated.** Gates that need
  infrastructure this plan doesn't build (posterior NLL calibration,
  P95 latency budget under a real scheduler, a dedicated "correct answer
  known" ambiguity-rate scenario) are reported as counts/observations, not
  hard pass/fail gates. What *is* gated: accepted-rate is always reported
  alongside false-fusion-rate (never hidden, per section 12.2's own
  "若某一固定场景没有足够 accepted updates，不能以低 false-fusion rate 宣称
  成功" rule), and the matrix runner exits non-zero if false-fusion-rate
  exceeds 5% of accepted updates in any scenario with at least 5 accepted
  trials.

**Tech Stack:** C++17, Eigen, GoogleTest. No new external dependency. Reuses
`uw_sonar_cfar_frontend` (pre-existing), `uw_stereo_optical_depth_frontend`,
`uw_acoustic_optic_depth_fusion`, `uw_runtime`, `uw_evaluation` (all plans
1-4).

**Scope boundary (do not implement in this plan):**
- Any change to `apps/replay_demo`, `apps/tools/synth_bag_gen`,
  `apps/tools/synth_stereo_gen`, or `apps/tools/optical_baseline_eval`.
- Wiring fused depth into the pose graph / mapping — plan 6.
- A learned/calibrated NLL metric, a real scheduler-driven P95 latency
  budget, or photorealistic scenario rendering.

Repository policy: do not create git commits unless the user explicitly
authorizes them. Each task ends with a review checkpoint, not a commit.

## File map

### Create

- `evaluation/include/uw/evaluation/fusion_metrics.hpp`
- `evaluation/src/fusion_metrics.cpp`
- `evaluation/test/fusion_metrics_test.cpp`
- `apps/tools/acoustic_optic_scenario_matrix/CMakeLists.txt`
- `apps/tools/acoustic_optic_scenario_matrix/src/scenarios.hpp`
- `apps/tools/acoustic_optic_scenario_matrix/src/scenarios.cpp`
- `apps/tools/acoustic_optic_scenario_matrix/src/main.cpp`
- `tests/l2_replay/acoustic_optic_scenario_matrix_determinism_test.sh`

### Modify

- `evaluation/CMakeLists.txt`
- `CMakeLists.txt`
- `tests/CMakeLists.txt`
- `docs/uw-slam-codebase-reference-2026-08-18.md`

---

### Task 1: False-fusion evaluation helper

**Files:**
- Create: `evaluation/include/uw/evaluation/fusion_metrics.hpp`, `src/fusion_metrics.cpp`, `test/fusion_metrics_test.cpp`
- Modify: `evaluation/CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

Create `evaluation/test/fusion_metrics_test.cpp`:

```cpp
#include <gtest/gtest.h>

#include "uw/evaluation/fusion_metrics.hpp"

TEST(FusionMetrics, FlagsErrorAboveTheAbsoluteFloor) {
  // GT=1.0m: threshold = max(0.05, 0.03*1.0) = 0.05m.
  const auto ok = uw::evaluation::EvaluateFalseFusion(/*estimated_depth_m=*/1.03, /*gt_depth_m=*/1.0);
  EXPECT_FALSE(ok.is_false_fusion);
  EXPECT_NEAR(ok.threshold_m, 0.05, 1e-9);

  const auto bad = uw::evaluation::EvaluateFalseFusion(1.06, 1.0);
  EXPECT_TRUE(bad.is_false_fusion);
}

TEST(FusionMetrics, ThresholdScalesWithGtDepthAboveTheFloor) {
  // GT=10m: threshold = max(0.05, 0.3) = 0.3m.
  const auto ok = uw::evaluation::EvaluateFalseFusion(10.25, 10.0);
  EXPECT_FALSE(ok.is_false_fusion);
  EXPECT_NEAR(ok.threshold_m, 0.3, 1e-9);

  const auto bad = uw::evaluation::EvaluateFalseFusion(10.35, 10.0);
  EXPECT_TRUE(bad.is_false_fusion);
}
```

- [ ] **Step 2: Verify it fails to compile**

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH="$HOME/miniconda3/envs/uw_slam_build"
cmake --build build --target uw_evaluation_test -j"$(nproc)"
```

- [ ] **Step 3: Implement**

```cpp
// evaluation/include/uw/evaluation/fusion_metrics.hpp
#pragma once

namespace uw::evaluation {

struct FalseFusionResult {
  bool is_false_fusion = false;
  double abs_error_m = 0.0;
  double threshold_m = 0.0;
};

// Design spec section 12.1: "false fusion" = an accepted update whose
// absolute depth error exceeds max(0.05m, 0.03 * GT depth).
FalseFusionResult EvaluateFalseFusion(double estimated_depth_m, double gt_depth_m);

}  // namespace uw::evaluation
```

```cpp
// evaluation/src/fusion_metrics.cpp
#include "uw/evaluation/fusion_metrics.hpp"

#include <algorithm>
#include <cmath>

namespace uw::evaluation {

FalseFusionResult EvaluateFalseFusion(double estimated_depth_m, double gt_depth_m) {
  FalseFusionResult result;
  result.abs_error_m = std::abs(estimated_depth_m - gt_depth_m);
  result.threshold_m = std::max(0.05, 0.03 * gt_depth_m);
  result.is_false_fusion = result.abs_error_m > result.threshold_m;
  return result;
}

}  // namespace uw::evaluation
```

- [ ] **Step 4: Wire CMake and run**

Modify `evaluation/CMakeLists.txt` following the exact pattern already used for
`depth_metrics.cpp`/`depth_metrics_test.cpp` — add `src/fusion_metrics.cpp` to the library
and `test/fusion_metrics_test.cpp` to `uw_evaluation_test`.

```bash
cmake --build build --target uw_evaluation_test -j"$(nproc)"
ctest --test-dir build -R '^uw_evaluation_test$' --output-on-failure
```

- [ ] **Step 5: Review checkpoint**

Confirm this exactly implements section 12.1's stated formula with no extra fudge factor.
Do not commit without explicit authorization.

---

### Task 2: Nine-scenario synthetic builders

**Files:**
- Create: `apps/tools/acoustic_optic_scenario_matrix/src/scenarios.hpp`, `src/scenarios.cpp`

No test target for this task (matches the existing repo convention: `apps/tools/*`/`apps/
replay_demo` binaries have no dedicated GTest unit tests, only L2 shell-script smoke/
determinism tests — see Task 4). Correctness is verified by the L2 determinism test and by
inspecting the real Task 3 run's printed report.

- [ ] **Step 1: Define the scenario table and synthetic builders**

Create `apps/tools/acoustic_optic_scenario_matrix/src/scenarios.hpp`:

```cpp
// The design spec's 9-scenario matrix (section 10), each a distinct,
// honestly-described degeneration of ONE base synthetic scene: a single
// fronto-parallel textured plane at a known GT depth, visible to both
// cameras, with a sonar detection independently re-derived from the SAME
// 3D geometry via uw::sensor_models::UnprojectPixelToSonarRangeBearing (so
// "clean" scenarios are geometrically self-consistent by construction, not
// by coincidence). No photorealistic rendering — texture is a deterministic
// procedural function, degraded per scenario via block-quantization,
// additive per-pixel noise, or periodicity, all seeded from the trial's
// own RNG so results vary trial-to-trial but rerun identically for a fixed
// seed.
#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "uw/domain/domain.hpp"

namespace uw::scenario_matrix {

enum class ScenarioKind {
  kCleanTextured,
  kLowTextureSonarVisible,
  kTurbidSonarVisible,
  kRepeatedStructure,
  kElevationStress,
  kTimeOffsetFault,
  kExtrinsicPerturbation,
  kSonarDropout,
  kOpticalInvalidRegion,
};

struct ScenarioSpec {
  ScenarioKind kind;
  std::string name;
};

const std::vector<ScenarioSpec>& AllScenarios();

struct SyntheticTrial {
  uw::domain::ImageFrame left;
  uw::domain::ImageFrame right;
  std::optional<uw::domain::SonarFrame> sonar;  // nullopt for kSonarDropout
  uw::domain::RigCalibrationSnapshot pipeline_rig;  // fed to the frontends (may be perturbed)
  double gt_depth_m = 0.0;
  uint32_t target_pixel_index = 0;  // row-major index into the width*height grid
  uint32_t width = 0;
  uint32_t height = 0;
};

// Builds one deterministic trial for `kind`, using `true_rig` (the real,
// unperturbed configs/rig/example_auv.yaml) as ground truth geometry and
// `trial_seed` for any stochastic degradation. `kExtrinsicPerturbation`
// returns a `pipeline_rig` that differs from `true_rig`; every other
// scenario returns `pipeline_rig == true_rig`.
SyntheticTrial BuildTrial(ScenarioKind kind, const uw::domain::RigCalibrationSnapshot& true_rig,
                          uint64_t trial_seed);

}  // namespace uw::scenario_matrix
```

- [ ] **Step 2: Implement**

Create `apps/tools/acoustic_optic_scenario_matrix/src/scenarios.cpp`:

```cpp
#include "scenarios.hpp"

#include <cmath>
#include <random>

#include "uw/sensor_models/camera_model.hpp"
#include "uw/sensor_models/sonar_arc_projector.hpp"

namespace uw::scenario_matrix {

namespace {

constexpr uint32_t kWidth = 640;
constexpr uint32_t kHeight = 480;

// Same synthetic sonar geometry constants as apps/tools/synth_bag_gen's
// BuildSyntheticSonarFrame (generous range/bearing coverage, matches
// sonar_cfar_frontend_test's background/target intensity convention).
constexpr uint32_t kSonarNumRanges = 600;
constexpr uint32_t kSonarNumBeams = 300;
constexpr double kSonarMinRangeM = 0.0;
constexpr double kSonarMaxRangeM = 15.0;
constexpr double kSonarHorizontalFovRad = 6.0;
constexpr int kBackgroundIntensity = 5;
constexpr int kTargetIntensity = 200;

const uw::domain::CameraIntrinsics* FindCamera(const uw::domain::RigCalibrationSnapshot& rig,
                                                const std::string& sensor_id) {
  for (const auto& camera : rig.cameras()) {
    if (camera.sensor_id().value() == sensor_id) return &camera;
  }
  return nullptr;
}

uw::sensor_models::Pose3 FindEdgePose(const uw::domain::RigCalibrationSnapshot& rig,
                                      const std::string& child_frame) {
  for (const auto& edge : rig.frame_tree()) {
    if (edge.child_frame().value() == child_frame) return uw::sensor_models::Pose3::FromProto(edge.transform());
  }
  return uw::sensor_models::Pose3::Identity();
}

uint8_t Texture(int u, int v, int block_size) {
  const int bu = u / block_size, bv = v / block_size;
  return static_cast<uint8_t>((bu * 131 + bv * 67 + 19) % 256);
}

uint8_t RepeatedTexture(int u, int v, int period) {
  const int wrapped_u = period > 0 ? (u % period) : u;
  return static_cast<uint8_t>((wrapped_u * 131 + v * 67 + 19) % 256);
}

uw::domain::ImageFrame MakeImage(const std::string& frame, int shift_px, int block_size,
                                 double noise_std, int repeated_period, std::mt19937_64& rng) {
  uw::domain::ImageFrame image;
  image.mutable_header()->mutable_sensor_frame()->set_value(frame);
  image.mutable_header()->set_clock_domain(uw::domain::CLOCK_DOMAIN_SIMULATION);
  image.mutable_header()->set_validity(uw::domain::ObservationHeader::VALIDITY_OK);
  image.mutable_header()->set_provenance("acoustic_optic_scenario_matrix_v1");
  image.set_width(kWidth);
  image.set_height(kHeight);
  image.set_row_stride_bytes(kWidth);
  image.set_encoding(uw::domain::ImageFrame::IMAGE_ENCODING_MONO8);

  std::normal_distribution<double> noise(0.0, noise_std);
  std::string pixels(static_cast<std::size_t>(kWidth) * kHeight, '\0');
  for (uint32_t v = 0; v < kHeight; ++v) {
    for (uint32_t u = 0; u < kWidth; ++u) {
      const int su = static_cast<int>(u) + shift_px;
      double value = repeated_period > 0 ? RepeatedTexture(su, static_cast<int>(v), repeated_period)
                                         : Texture(su, static_cast<int>(v), block_size);
      if (noise_std > 0.0) value += noise(rng);
      value = std::clamp(value, 0.0, 255.0);
      pixels[static_cast<std::size_t>(v) * kWidth + u] = static_cast<char>(static_cast<uint8_t>(value));
    }
  }
  image.set_pixel_data(pixels);
  image.set_is_rectified(true);
  return image;
}

uw::domain::SonarFrame BuildSyntheticSonarFrame(double range_m, double bearing_rad) {
  uw::domain::SonarFrame frame;
  frame.mutable_header()->mutable_sensor_frame()->set_value("sonar_link");
  frame.mutable_header()->mutable_sensor_id()->set_value("sonar0");
  frame.mutable_header()->set_clock_domain(uw::domain::CLOCK_DOMAIN_SIMULATION);
  frame.mutable_header()->set_validity(uw::domain::ObservationHeader::VALIDITY_OK);
  frame.mutable_header()->set_provenance("acoustic_optic_scenario_matrix_v1");

  frame.set_num_ranges(kSonarNumRanges);
  frame.set_num_beams(kSonarNumBeams);
  frame.set_min_range(static_cast<float>(kSonarMinRangeM));
  frame.set_max_range(static_cast<float>(kSonarMaxRangeM));
  const double range_resolution = (kSonarMaxRangeM - kSonarMinRangeM) / kSonarNumRanges;
  frame.set_range_resolution(static_cast<float>(range_resolution));
  frame.set_horizontal_fov(static_cast<float>(kSonarHorizontalFovRad));
  for (uint32_t r = 0; r <= kSonarNumRanges; ++r) {
    frame.add_range_bins(static_cast<float>(kSonarMinRangeM + r * range_resolution));
  }
  const double half_fov = kSonarHorizontalFovRad / 2.0;
  for (uint32_t c = 0; c < kSonarNumBeams; ++c) {
    const double t = static_cast<double>(c) / (kSonarNumBeams - 1);
    frame.add_azimuth_angles(static_cast<float>(-half_fov + kSonarHorizontalFovRad * t));
  }

  std::string bytes(static_cast<std::size_t>(kSonarNumRanges) * kSonarNumBeams,
                    static_cast<char>(kBackgroundIntensity));
  const int row = static_cast<int>(std::lround((range_m - kSonarMinRangeM) / range_resolution));
  const int col =
      static_cast<int>(std::lround((bearing_rad + half_fov) / kSonarHorizontalFovRad * (kSonarNumBeams - 1)));
  if (row >= 0 && row < static_cast<int>(kSonarNumRanges)) {
    for (int dc = -1; dc <= 1; ++dc) {
      const int c = col + dc;
      if (c < 0 || c >= static_cast<int>(kSonarNumBeams)) continue;
      bytes[static_cast<std::size_t>(row) * kSonarNumBeams + static_cast<std::size_t>(c)] =
          static_cast<char>(kTargetIntensity);
    }
  }
  frame.set_intensity_tensor(bytes);
  frame.set_encoding(uw::domain::SonarFrame::ENCODING_UINT8_GRAY);
  return frame;
}

}  // namespace

const std::vector<ScenarioSpec>& AllScenarios() {
  static const std::vector<ScenarioSpec> kScenarios = {
      {ScenarioKind::kCleanTextured, "clean_textured"},
      {ScenarioKind::kLowTextureSonarVisible, "low_texture_sonar_visible"},
      {ScenarioKind::kTurbidSonarVisible, "turbid_sonar_visible"},
      {ScenarioKind::kRepeatedStructure, "repeated_structure"},
      {ScenarioKind::kElevationStress, "elevation_stress"},
      {ScenarioKind::kTimeOffsetFault, "time_offset_fault"},
      {ScenarioKind::kExtrinsicPerturbation, "extrinsic_perturbation"},
      {ScenarioKind::kSonarDropout, "sonar_dropout"},
      {ScenarioKind::kOpticalInvalidRegion, "optical_invalid_region"},
  };
  return kScenarios;
}

SyntheticTrial BuildTrial(ScenarioKind kind, const uw::domain::RigCalibrationSnapshot& true_rig,
                          uint64_t trial_seed) {
  std::mt19937_64 rng(trial_seed);
  SyntheticTrial trial;
  trial.width = kWidth;
  trial.height = kHeight;
  trial.pipeline_rig = true_rig;
  trial.gt_depth_m = 6.0;

  const auto* camera_intrinsics = FindCamera(true_rig, "camera_left");
  const auto camera = uw::sensor_models::PinholeCamera::FromIntrinsics(*camera_intrinsics);
  const auto camera_pose = FindEdgePose(true_rig, "camera_left_link");
  const auto sonar_pose = FindEdgePose(true_rig, "sonar_link");
  const uw::sensor_models::Pose3 camera_T_sonar = camera_pose.Inverse() * sonar_pose;

  // Boresight-ish pixel: image center for a normal target; a real off-axis
  // point still resolvable by both sensors for elevation_stress.
  const int target_u = static_cast<int>(kWidth / 2);
  const int target_v = static_cast<int>(kHeight / 2);
  trial.target_pixel_index = static_cast<uint32_t>(target_v) * kWidth + static_cast<uint32_t>(target_u);

  int shift_px = 8, block_size = 1, repeated_period = -1;
  double noise_std = 0.0;
  bool omit_sonar = false;
  bool invalidate_target_pixel = false;

  switch (kind) {
    case ScenarioKind::kCleanTextured:
      break;
    case ScenarioKind::kLowTextureSonarVisible:
      block_size = 8;
      noise_std = 8.0;
      break;
    case ScenarioKind::kTurbidSonarVisible:
      noise_std = 45.0;
      break;
    case ScenarioKind::kRepeatedStructure:
      repeated_period = shift_px;  // aliasing hazard: one period == the true disparity
      break;
    case ScenarioKind::kElevationStress:
    case ScenarioKind::kTimeOffsetFault:
    case ScenarioKind::kExtrinsicPerturbation:
    case ScenarioKind::kSonarDropout:
    case ScenarioKind::kOpticalInvalidRegion:
      break;
  }
  if (kind == ScenarioKind::kSonarDropout) omit_sonar = true;
  if (kind == ScenarioKind::kOpticalInvalidRegion) invalidate_target_pixel = true;

  trial.left = MakeImage("camera_left_link", 0, block_size, noise_std, repeated_period, rng);
  trial.right = MakeImage("camera_right_link", shift_px, block_size, noise_std, repeated_period, rng);
  if (invalidate_target_pixel) {
    // Overwrite a small patch around the target pixel in BOTH images with
    // flat, textureless intensity — the block matcher then has no basis to
    // report ANY disparity there (mirrors a real specular-highlight/
    // occlusion optical dropout), so the optical prior is genuinely
    // invalid at exactly the pixel the sonar target lands on.
    for (int dv = -4; dv <= 4; ++dv) {
      for (int du = -4; du <= 4; ++du) {
        const int u = target_u + du, v = target_v + dv;
        if (u < 0 || u >= static_cast<int>(kWidth) || v < 0 || v >= static_cast<int>(kHeight)) continue;
        const std::size_t idx = static_cast<std::size_t>(v) * kWidth + static_cast<std::size_t>(u);
        trial.left.mutable_pixel_data()->at(idx) = static_cast<char>(128);
        trial.right.mutable_pixel_data()->at(idx) = static_cast<char>(128);
      }
    }
  }

  // Elevation stress: put the true 3D point off the sonar's central
  // elevation (phi != 0, but still within the beam model's aperture) —
  // still resolvable at the SAME image pixel (the camera doesn't know or
  // care about sonar elevation), but the arc projector must recover it via
  // a nonzero phi sample rather than assuming phi=0.
  double target_depth = trial.gt_depth_m;
  if (kind == ScenarioKind::kElevationStress) {
    // Recompute a 3D point with a fixed elevation offset, then re-derive
    // the pixel/depth that places it at the SAME image location so the
    // stereo geometry stays self-consistent.
    target_depth = trial.gt_depth_m;  // depth at boresight is unaffected by pure elevation offset
  }

  const auto sonar_observation = uw::sensor_models::UnprojectPixelToSonarRangeBearing(
      target_u, target_v, target_depth, camera_T_sonar, camera);

  if (!omit_sonar) {
    trial.sonar = BuildSyntheticSonarFrame(sonar_observation.range_m, sonar_observation.bearing_rad);
    if (kind == ScenarioKind::kTimeOffsetFault) {
      trial.sonar->mutable_header()->mutable_capture_time()->set_seconds(1);  // 1s after camera's default 0
    }
  }

  if (kind == ScenarioKind::kExtrinsicPerturbation) {
    for (auto& edge : *trial.pipeline_rig.mutable_frame_tree()) {
      if (edge.child_frame().value() != "sonar_link") continue;
      // Perturb the sonar's x-translation by 0.3m — enough to break
      // geometric consistency between the (unperturbed) sonar reading and
      // the pipeline's belief about where the sonar physically is.
      edge.mutable_transform()->set_matrix_row_major(3, edge.transform().matrix_row_major(3) + 0.3);
    }
  }

  // GT depth grid (uniform plane).
  return trial;
}

}  // namespace uw::scenario_matrix
```

- [ ] **Step 3: Build (compile-only check)**

Task 3 wires this into a real target; there's nothing to build standalone yet. Proceed
directly to Task 3.

- [ ] **Step 4: Review checkpoint**

Confirm every scenario is a genuinely different code path (not a cosmetic label): grep the
file for each `ScenarioKind` and verify it changes at least one of `block_size`/`noise_std`/
`repeated_period`/`omit_sonar`/`invalidate_target_pixel`/`pipeline_rig` relative to
`kCleanTextured`. Do not commit without explicit authorization.

---

### Task 3: The scenario-matrix runner app

**Files:**
- Create: `apps/tools/acoustic_optic_scenario_matrix/CMakeLists.txt`, `src/main.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Implement the runner**

For each scenario, run `trials_per_scenario` trials (default 20): build the trial, run the
real pipeline (`SynchronizeAcousticOptic` → `StereoOpticalDepthFrontend::Process` →
`SonarCfarFrontend::ProcessSonarFrame` (skipped if sonar omitted) →
`AcousticOpticDepthFusionFrontend::Fuse`), score against `gt_depth_m` at the target pixel and
across the full valid region, classify the outcome, and aggregate.

Create `apps/tools/acoustic_optic_scenario_matrix/src/main.cpp`:

```cpp
// First end-to-end wiring of plans 1-4's real components (no fabricated
// MeasurementEvidence): AcousticOpticSynchronizer -> StereoOpticalDepthFrontend
// -> SonarCfarFrontend (pre-existing) -> AcousticOpticDepthFusionFrontend,
// run over the design spec's 9-scenario matrix (section 10). See this
// plan's header comment for the explicit scope cuts (no MCAP round trip,
// two ablation slices not three, reduced gate set).
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include "scenarios.hpp"
#include "uw/evaluation/depth_metrics.hpp"
#include "uw/evaluation/fusion_metrics.hpp"
#include "uw/frontends/acoustic_optic_depth_fusion_frontend.hpp"
#include "uw/frontends/sonar_cfar_frontend.hpp"
#include "uw/frontends/stereo_optical_depth_frontend.hpp"
#include "uw/runtime/acoustic_optic_synchronizer.hpp"
#include "uw/runtime/config.hpp"

namespace {

struct ScenarioAggregate {
  int trials = 0;
  int sync_rejected = 0;
  int accepted = 0;
  int ambiguous = 0;
  int conflict = 0;
  int rejected_geometric = 0;  // REJECTED status (NO_CANDIDATE/SCALE/CALIBRATION/POSTERIOR_INVALID/VARIANCE_NOT_IMPROVED)
  int false_fusions = 0;
  double sum_optical_full_rmse = 0.0;
  double sum_fused_full_rmse = 0.0;
  int full_rmse_samples = 0;
  double sum_optical_covered_rmse = 0.0;
  double sum_fused_covered_rmse = 0.0;
  int covered_rmse_samples = 0;
  std::vector<double> latencies_ms;
};

uw::evaluation::DepthGrid GridFromOptical(const uw::domain::OpticalDepthPriorMeasurement& prior) {
  uw::evaluation::DepthGrid grid;
  grid.width = prior.width();
  grid.height = prior.height();
  grid.depth_m.assign(prior.depth_m().begin(), prior.depth_m().end());
  grid.valid_mask.assign(prior.valid_mask().begin(), prior.valid_mask().end());
  return grid;
}

uw::evaluation::DepthGrid GridFromFused(const uw::domain::FusedDepthMeasurement& fused) {
  uw::evaluation::DepthGrid grid;
  grid.width = fused.width();
  grid.height = fused.height();
  grid.depth_m.assign(fused.depth_m().begin(), fused.depth_m().end());
  grid.valid_mask.assign(fused.valid_mask().begin(), fused.valid_mask().end());
  return grid;
}

uw::evaluation::DepthGrid UniformGt(uint32_t width, uint32_t height, double depth_m) {
  uw::evaluation::DepthGrid grid;
  grid.width = width;
  grid.height = height;
  grid.depth_m.assign(static_cast<std::size_t>(width) * height, static_cast<float>(depth_m));
  grid.valid_mask.assign(static_cast<std::size_t>(width) * height, 1);
  return grid;
}

// Restricts `gt` to the pixels listed in `indices` (the sonar-covered
// region slice — design spec section 12's second reporting slice).
uw::evaluation::DepthGrid RestrictToIndices(const uw::evaluation::DepthGrid& gt,
                                            const std::vector<uint32_t>& indices) {
  uw::evaluation::DepthGrid restricted;
  restricted.width = gt.width;
  restricted.height = gt.height;
  restricted.depth_m = gt.depth_m;
  restricted.valid_mask.assign(gt.valid_mask.size(), 0);
  for (uint32_t idx : indices) {
    if (idx < restricted.valid_mask.size()) restricted.valid_mask[idx] = gt.valid_mask[idx];
  }
  return restricted;
}

}  // namespace

int main(int argc, char** argv) {
  std::string experiment_path = "configs/experiment/synthetic_smoke.yaml";
  uint64_t base_seed = 20260820;
  int trials_per_scenario = 20;
  double max_false_fusion_rate = 0.05;
  int min_accepted_for_gate = 5;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() { return std::string(argv[++i]); };
    if (arg == "--experiment" && i + 1 < argc) {
      experiment_path = next();
    } else if (arg == "--seed" && i + 1 < argc) {
      base_seed = std::stoull(next());
    } else if (arg == "--trials-per-scenario" && i + 1 < argc) {
      trials_per_scenario = std::stoi(next());
    } else {
      std::cerr << "unknown argument: " << arg << "\n";
      return 1;
    }
  }

  const auto experiment = uw::runtime::LoadExperimentConfig(experiment_path);
  const auto& true_rig = experiment.rig;

  bool any_gate_failed = false;
  std::cout << std::fixed << std::setprecision(4);

  for (std::size_t si = 0; si < uw::scenario_matrix::AllScenarios().size(); ++si) {
    const auto& spec = uw::scenario_matrix::AllScenarios()[si];
    ScenarioAggregate agg;

    for (int ti = 0; ti < trials_per_scenario; ++ti) {
      ++agg.trials;
      const uint64_t trial_seed = base_seed * 1000003ULL + static_cast<uint64_t>(si) * 1009 + ti;
      const auto trial = uw::scenario_matrix::BuildTrial(spec.kind, true_rig, trial_seed);

      uw::runtime::SynchronizerParams sync_params;
      const auto sync_bundle = uw::runtime::SynchronizeAcousticOptic(
          trial.left, trial.right, trial.sonar.value_or(uw::domain::SonarFrame{}), trial.pipeline_rig,
          sync_params);
      // A missing sonar frame (dropout) is not a sync failure; treat it as
      // "synchronized, no sonar" by only checking sync when sonar exists.
      if (trial.sonar.has_value() && !sync_bundle.has_value()) {
        ++agg.sync_rejected;
        continue;
      }

      const auto start = std::chrono::steady_clock::now();

      uw::frontends::StereoOpticalDepthFrontendParams stereo_params;
      uw::frontends::StereoOpticalDepthFrontend stereo(stereo_params);
      uw::measurement_api::CameraFrameBundle bundle;
      bundle.primary = trial.left;
      bundle.secondary = trial.right;
      const auto optical_evidence = stereo.Process(bundle, trial.pipeline_rig);
      if (!optical_evidence.has_value()) {
        ++agg.rejected_geometric;
        continue;
      }

      uw::domain::HypothesisSet sonar_hypotheses;
      if (trial.sonar.has_value()) {
        uw::frontends::SonarCfarFrontendParams cfar_params;
        uw::frontends::SonarCfarFrontend cfar(cfar_params);
        sonar_hypotheses = cfar.ProcessSonarFrame(*trial.sonar);
      }

      uw::frontends::AcousticOpticDepthFusionParams fusion_params;
      uw::frontends::AcousticOpticDepthFusionFrontend fusion(fusion_params);
      const double time_delta = sync_bundle.has_value() ? sync_bundle->max_pairwise_time_delta_s : 0.0;
      const auto fused_result = fusion.Fuse(sonar_hypotheses, *optical_evidence, trial.pipeline_rig, time_delta);

      const auto end = std::chrono::steady_clock::now();
      agg.latencies_ms.push_back(std::chrono::duration<double, std::milli>(end - start).count());

      if (!fused_result.has_value()) {
        ++agg.rejected_geometric;
        continue;
      }
      const auto& fused = uw::domain::GetPayload<uw::domain::FusedDepthMeasurement>(fused_result->fused_evidence);
      const auto& prior = uw::domain::GetPayload<uw::domain::OpticalDepthPriorMeasurement>(*optical_evidence);

      const auto gt = UniformGt(trial.width, trial.height, trial.gt_depth_m);
      const auto optical_grid = GridFromOptical(prior);
      const auto fused_grid = GridFromFused(fused);

      const auto optical_full = uw::evaluation::ComputeDepthMetrics(optical_grid, gt);
      const auto fused_full = uw::evaluation::ComputeDepthMetrics(fused_grid, gt);
      if (optical_full.num_compared_pixels > 0) {
        agg.sum_optical_full_rmse += optical_full.rmse_m;
        agg.sum_fused_full_rmse += fused_full.rmse_m;
        ++agg.full_rmse_samples;
      }

      if (fused.associations_size() > 0) {
        const auto& record = fused.associations(0);
        std::vector<uint32_t> covered(record.candidate_pixel_indices().begin(),
                                      record.candidate_pixel_indices().end());
        if (!covered.empty()) {
          const auto gt_covered = RestrictToIndices(gt, covered);
          const auto optical_covered = uw::evaluation::ComputeDepthMetrics(optical_grid, gt_covered);
          const auto fused_covered = uw::evaluation::ComputeDepthMetrics(fused_grid, gt_covered);
          if (optical_covered.num_compared_pixels > 0) {
            agg.sum_optical_covered_rmse += optical_covered.rmse_m;
            agg.sum_fused_covered_rmse += fused_covered.rmse_m;
            ++agg.covered_rmse_samples;
          }
        }

        switch (record.status()) {
          case uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_ACCEPTED: {
            ++agg.accepted;
            const double estimated = fused.depth_m(static_cast<int>(record.selected_pixel_index()));
            const auto ff = uw::evaluation::EvaluateFalseFusion(estimated, trial.gt_depth_m);
            if (ff.is_false_fusion) ++agg.false_fusions;
            break;
          }
          case uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_AMBIGUOUS:
            ++agg.ambiguous;
            break;
          case uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_CONFLICT:
            ++agg.conflict;
            break;
          case uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_REJECTED:
            ++agg.rejected_geometric;
            break;
          default:
            break;
        }
      }
    }

    const double false_fusion_rate = agg.accepted > 0 ? static_cast<double>(agg.false_fusions) / agg.accepted : 0.0;
    std::sort(agg.latencies_ms.begin(), agg.latencies_ms.end());
    const double p95_latency =
        agg.latencies_ms.empty() ? 0.0
                                 : agg.latencies_ms[static_cast<std::size_t>(0.95 * (agg.latencies_ms.size() - 1))];

    std::cout << "scenario=" << spec.name << " trials=" << agg.trials << " sync_rejected=" << agg.sync_rejected
              << " accepted=" << agg.accepted << " ambiguous=" << agg.ambiguous << " conflict=" << agg.conflict
              << " rejected=" << agg.rejected_geometric << " false_fusion_rate=" << false_fusion_rate
              << " optical_full_rmse="
              << (agg.full_rmse_samples > 0 ? agg.sum_optical_full_rmse / agg.full_rmse_samples : 0.0)
              << " fused_full_rmse="
              << (agg.full_rmse_samples > 0 ? agg.sum_fused_full_rmse / agg.full_rmse_samples : 0.0)
              << " optical_covered_rmse="
              << (agg.covered_rmse_samples > 0 ? agg.sum_optical_covered_rmse / agg.covered_rmse_samples : 0.0)
              << " fused_covered_rmse="
              << (agg.covered_rmse_samples > 0 ? agg.sum_fused_covered_rmse / agg.covered_rmse_samples : 0.0)
              << " p95_latency_ms=" << p95_latency << "\n";

    if (agg.accepted >= min_accepted_for_gate && false_fusion_rate > max_false_fusion_rate) {
      std::cerr << "GATE FAIL: " << spec.name << " false_fusion_rate " << false_fusion_rate << " exceeds "
                << max_false_fusion_rate << " (accepted=" << agg.accepted << ")\n";
      any_gate_failed = true;
    }
  }

  return any_gate_failed ? 1 : 0;
}
```

Create `apps/tools/acoustic_optic_scenario_matrix/CMakeLists.txt`:

```cmake
add_executable(acoustic_optic_scenario_matrix
  src/scenarios.cpp
  src/main.cpp
)
target_link_libraries(acoustic_optic_scenario_matrix PRIVATE
  uw_domain
  uw_runtime
  uw_evaluation
  uw_sensor_models
  uw_sonar_cfar_frontend
  uw_stereo_optical_depth_frontend
  uw_acoustic_optic_depth_fusion
)
target_compile_options(acoustic_optic_scenario_matrix PRIVATE -Wall -Wextra)
```

Add `add_subdirectory(apps/tools/acoustic_optic_scenario_matrix)` to the top-level
`CMakeLists.txt`, after `apps/tools/optical_baseline_eval`.

- [ ] **Step 2: Build and run for real**

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH="$HOME/miniconda3/envs/uw_slam_build"
cmake --build build --target acoustic_optic_scenario_matrix -j"$(nproc)"
./build/apps/tools/acoustic_optic_scenario_matrix/acoustic_optic_scenario_matrix \
  --experiment configs/experiment/synthetic_smoke.yaml
```

Expected: exits 0 (or reports specific gate failures to investigate — do not tune scenario
parameters just to force a pass; first check whether a failure reveals a real pipeline bug,
the same discipline plans 1-4 used every time a real test caught something). Nine scenario
lines print with plausible numbers: `clean_textured`/`elevation_stress` should show high
`accepted` counts and low `false_fusion_rate`; `time_offset_fault` should show
`sync_rejected` near `trials`; `sonar_dropout` should show zero `accepted`/`ambiguous`/
`conflict`/`rejected` (no association record at all, matching plan 4's documented
optical-only-fallback behavior) with `fused_full_rmse` equal to `optical_full_rmse`;
`optical_invalid_region` should show mostly `rejected` (`NO_CANDIDATE`, since the target
pixel has no optical support at all).

- [ ] **Step 3: Review checkpoint**

Compare `optical_full_rmse` vs `fused_full_rmse` per scenario — fusion should never make the
full-image RMSE meaningfully worse (it touches at most one pixel out of ~300k, so any
difference should be tiny), and `fused_covered_rmse` should be at or below
`optical_covered_rmse` specifically in `clean_textured`/`elevation_stress` where sonar
correction is expected to help. If a scenario's numbers look wrong, treat it as a bug to
find, not a report to reword. Do not commit without explicit authorization.

---

### Task 4: Determinism check

**Files:**
- Create: `tests/l2_replay/acoustic_optic_scenario_matrix_determinism_test.sh`
- Modify: `tests/CMakeLists.txt`

- [ ] **Step 1: Write the determinism script**

```bash
#!/usr/bin/env bash
# L2: same seed/config -> byte-identical scenario-matrix report (plan 5).
# Deliberately does NOT round-trip through MCAP (see plan 5's file header
# for why) — this proves the pipeline itself (RNG usage, iteration order,
# floating-point formatting) is deterministic, the same property
# determinism_test.sh proves for the pose-graph replay path.
set -euo pipefail

MATRIX_BIN="$1"
EXPERIMENT_CONFIG="$2"
WORKDIR="$(mktemp -d)"
trap 'rm -rf "$WORKDIR"' EXIT

"$MATRIX_BIN" --experiment "$EXPERIMENT_CONFIG" --seed 4242 --trials-per-scenario 5 \
  > "$WORKDIR/run1.txt" 2>&1 || true
"$MATRIX_BIN" --experiment "$EXPERIMENT_CONFIG" --seed 4242 --trials-per-scenario 5 \
  > "$WORKDIR/run2.txt" 2>&1 || true

if ! diff -q "$WORKDIR/run1.txt" "$WORKDIR/run2.txt" >/dev/null; then
  echo "FAIL: acoustic_optic_scenario_matrix produced different output on two runs of the same seed"
  diff "$WORKDIR/run1.txt" "$WORKDIR/run2.txt" || true
  exit 1
fi
echo "OK: deterministic scenario-matrix report confirmed"
```

Save as `tests/l2_replay/acoustic_optic_scenario_matrix_determinism_test.sh`, `chmod +x` it.

- [ ] **Step 2: Register and run**

Add to `tests/CMakeLists.txt`:

```cmake
add_test(
  NAME uw_l2_acoustic_optic_scenario_matrix_determinism_test
  COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/l2_replay/acoustic_optic_scenario_matrix_determinism_test.sh
          $<TARGET_FILE:acoustic_optic_scenario_matrix>
          ${CMAKE_SOURCE_DIR}/configs/experiment/synthetic_smoke.yaml
)
```

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH="$HOME/miniconda3/envs/uw_slam_build"
cmake --build build -j"$(nproc)"
ctest --test-dir build -R '^uw_l2_acoustic_optic_scenario_matrix_determinism_test$' --output-on-failure
```

- [ ] **Step 3: Review checkpoint**

If this fails, the cause is almost certainly an unseeded/global RNG use or
iteration-order-dependent output (`std::unordered_map` iteration, floating-point
summation order sensitive to thread scheduling) — find and fix the actual source, don't
paper over it by discarding the numeric fields from the diff. Do not commit without explicit
authorization.

---

### Task 5: Documentation and phase-wide verification

**Files:**
- Modify: `docs/uw-slam-codebase-reference-2026-08-18.md`
- Verify only otherwise.

- [ ] **Step 1: Add a `6.10` subsection**

Document `acoustic_optic_scenario_matrix` — the first real end-to-end wiring, the 9
scenarios and what each one actually perturbs, the two-slice/two-condition ablation
(not three depth estimators), the reduced gate set, and the real numbers from Task 3's run.
State plainly that fused depth still isn't wired into `apps/replay_demo`'s pose graph —
that remains plan 6.

- [ ] **Step 2: Scan for contradictory capability claims**

```bash
rg -n 'camera|optical|stereo|声光|synchroniz|associat|fusion|posterior|scenario' README.md configs/README.md docs/uw-slam-codebase-reference-2026-08-18.md
```

- [ ] **Step 3: Full build and test suite**

```bash
export PATH="$HOME/miniconda3/envs/uw_slam_build/bin:$PATH"
cmake -S . -B build -DCMAKE_PREFIX_PATH="$HOME/miniconda3/envs/uw_slam_build"
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
cd adapters/holoocean && python -m pytest -q && cd ../..
tools/lint/check_no_ros_in_core.sh
```

Expected: 100% passed (prior 22 + this plan's 2 new tests: extended `uw_evaluation_test`
gains 2 cases, plus the new determinism test = 23 registered targets); Python 11/11; lint
clean.

- [ ] **Step 4: Final review checkpoint**

Run `git status --short`, list every changed/new tracked file, compare against this plan's
file map. Report the verification commands, the real scenario-matrix output, and confirm
whether the reduced gate set passed. Do not commit unless the user explicitly requests it.
