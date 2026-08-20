# Acoustic-Optic Mapping Handoff Implementation Plan

**Goal:** Implement plan 6 (the last) of the 6-plan acoustic-optic series:
convert plan 4's `FusedDepthMeasurement` into local-frame `MapEvidence` and
verify it survives a pose-graph correction the same way every other
`MapEvidence` producer in this repo already does — via the existing,
pre-series `SubmapManager`, not a new mapping data structure.

**Architecture:** This is a small glue module, not a new algorithm.
`algorithms/mapping/submap_manager` (existed before this series) already
implements the entire "local evidence + on-demand world retransform"
contract (`AddMapEvidence`/`UpdateKeyframePose`/`WorldPointsForKeyframe`) —
plan 6 does not modify it. The only new code is a converter,
`BuildMapEvidenceFromFusedDepth`, that unprojects each valid,
non-`INVALID`-contribution pixel of a `FusedDepthMeasurement` into a 3D
point and expresses it in **base_link frame** (not camera-optical frame,
not world frame) — base_link is the frame `SubmapManager::WorldPointsForKeyframe`
composes with the keyframe's `pose_WB` via `pose_WB.Apply(local)`, so storing
points in any other frame would silently produce wrong world points. This
supersedes `apps/replay_demo`'s existing landmark-insertion code (lines
~296-309), which works around the same problem by storing points pre-baked
into `local_frame="world"` with an identity keyframe pose — a documented
v1 shortcut, not the pattern this plan implements "properly," per the
architecture's actual intent (`map.proto`'s own header comment).

**Frame chain (composes three already-existing pieces, no new geometry primitive):**

```
pixel (u,v) + depth  --Unproject-->  point_optical (plan 2, PinholeCamera)
            --OpticalFromBodyRotation().transpose()-->  point_camera_body (plan 3's fixed rotation)
            --camera_pose.Apply()-->  point_base_link (rig frame_tree, plan 1's RigCalibrationSnapshot)
            [stored as MapEvidence.geometry_or_occupancy, local_frame="base_link"]
            --pose_WB.Apply()-->  point_world  (SubmapManager, pre-existing, unchanged)
```

**Tech Stack:** C++17, Eigen, GoogleTest. No new external dependency, no new app.

**Scope boundary (do not implement in this plan):**
- Any change to `algorithms/mapping/submap_manager` itself, or to
  `apps/replay_demo`'s existing (hack) landmark-insertion path — this plan
  adds a parallel, correct producer of `MapEvidence`; it does not touch or
  replace what `replay_demo` already does with sonar landmarks.
- Occupancy/TSDF/surfel representations — `MAP_REPRESENTATION_POINT_CLOUD`
  only, matching every other `MapEvidence` producer in this repo.
- Reintegrating on RIG calibration changes (only pose changes are handled —
  `REINTEGRATION_POLICY_TRANSFORM_ONLY` is correct only because the camera
  extrinsic is treated as fixed per plan 2-4's own stated v1 scope; a
  calibration change would need `FULL_REFUSE`, out of scope here).

Repository policy: do not create git commits unless the user explicitly
authorizes them. Each task ends with a review checkpoint, not a commit.

## File map

### Create

- `algorithms/mapping/acoustic_optic_map_bridge/CMakeLists.txt`
- `algorithms/mapping/acoustic_optic_map_bridge/include/uw/mapping/acoustic_optic_map_bridge.hpp`
- `algorithms/mapping/acoustic_optic_map_bridge/src/acoustic_optic_map_bridge.cpp`
- `algorithms/mapping/acoustic_optic_map_bridge/test/acoustic_optic_map_bridge_test.cpp`

### Modify

- `CMakeLists.txt` — register the new subdirectory.
- `docs/uw-slam-codebase-reference-2026-08-18.md` — document the new module and the series' completion state.

---

### Task 1: `BuildMapEvidenceFromFusedDepth` + retransform verification

**Files:**
- Create: all four files above.
- Modify: `CMakeLists.txt`.

- [ ] **Step 1: Create the module skeleton**

Create `algorithms/mapping/acoustic_optic_map_bridge/CMakeLists.txt`:

```cmake
add_library(uw_acoustic_optic_map_bridge STATIC
  src/acoustic_optic_map_bridge.cpp
)
target_include_directories(uw_acoustic_optic_map_bridge PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(uw_acoustic_optic_map_bridge PUBLIC uw_sensor_models)
target_compile_options(uw_acoustic_optic_map_bridge PRIVATE -Wall -Wextra)

if(UW_BUILD_TESTS)
  add_executable(uw_acoustic_optic_map_bridge_test test/acoustic_optic_map_bridge_test.cpp)
  target_link_libraries(uw_acoustic_optic_map_bridge_test PRIVATE
    uw_acoustic_optic_map_bridge uw_submap_manager GTest::gtest GTest::gtest_main
  )
  add_test(NAME uw_acoustic_optic_map_bridge_test COMMAND uw_acoustic_optic_map_bridge_test)
endif()
```

Add `add_subdirectory(algorithms/mapping/acoustic_optic_map_bridge)` to the top-level
`CMakeLists.txt`, after `algorithms/mapping/submap_manager` (the test links `uw_submap_manager`
to prove real end-to-end retransform, not just the converter in isolation).

- [ ] **Step 2: Write the failing tests**

Create `algorithms/mapping/acoustic_optic_map_bridge/test/acoustic_optic_map_bridge_test.cpp`.
Numeric design: camera at `base_link` translation `(0.1,0,0)`, identity rotation; boresight
pixel `(10,5)` on a `20x10` image (`fx=fy=100,cx=10,cy=5`) at `depth=5.0` — the SAME
boresight-unprojection identity established in plan 3's tests
(`Unproject(cx,cy,d)=(0,0,d)` optical, `R^T*(0,0,d)=(d,0,0)` body) gives an exact,
hand-checkable expected point:

```cpp
#include <gtest/gtest.h>

#include "uw/mapping/acoustic_optic_map_bridge.hpp"
#include "uw/mapping/submap_manager.hpp"

namespace {

uw::domain::RigCalibrationSnapshot MakeRig() {
  uw::domain::RigCalibrationSnapshot rig;
  auto* camera = rig.add_cameras();
  camera->mutable_sensor_id()->set_value("camera_left");
  camera->set_width(20);
  camera->set_height(10);
  for (double v : {100.0, 0.0, 10.0, 0.0, 100.0, 5.0, 0.0, 0.0, 1.0}) camera->add_k_matrix_row_major(v);

  auto* edge = rig.add_frame_tree();
  edge->mutable_parent_frame()->set_value("base_link");
  edge->mutable_child_frame()->set_value("camera_left_link");
  for (double v : {1.0, 0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0}) {
    edge->mutable_transform()->add_matrix_row_major(v);
  }
  return rig;
}

uw::domain::MeasurementEvidence MakeFusedEvidence(int width, int height, int valid_index,
                                                   float depth_m, float variance_m2) {
  uw::domain::FusedDepthMeasurement fused;
  fused.set_width(width);
  fused.set_height(height);
  const int pixels = width * height;
  std::string valid_mask(pixels, '\0');
  std::string contribution_mask(pixels, static_cast<char>(uw::domain::DEPTH_CONTRIBUTION_INVALID));
  for (int i = 0; i < pixels; ++i) {
    fused.add_depth_m(i == valid_index ? depth_m : 0.0f);
    fused.add_variance_m2(i == valid_index ? variance_m2 : 0.0f);
  }
  valid_mask[valid_index] = 1;
  contribution_mask[valid_index] = static_cast<char>(uw::domain::DEPTH_CONTRIBUTION_ACOUSTIC_OPTIC);
  fused.set_valid_mask(valid_mask);
  fused.set_contribution_mask(contribution_mask);
  uw::domain::EvidenceId id;
  id.set_value("fused_1");
  return uw::domain::MakeEvidence(id, {}, fused, 0.5, "acoustic_optic_depth_fusion_v1");
}

}  // namespace

TEST(AcousticOpticMapBridge, ConvertsBoresightPixelToBaseLinkFramePoint) {
  const auto rig = MakeRig();
  // index 5*20+10 = 110, matching the (10,5) boresight pixel used throughout plans 3/4.
  const auto fused_evidence = MakeFusedEvidence(20, 10, 110, 5.0f, 0.02f);

  uw::mapping::AcousticOpticMapBridgeParams params;
  const auto map_evidence =
      uw::mapping::BuildMapEvidenceFromFusedDepth(fused_evidence, rig, params, "kf3", /*state_version=*/7);

  ASSERT_TRUE(map_evidence.has_value());
  EXPECT_EQ(map_evidence->keyframe_id().value(), "kf3");
  EXPECT_EQ(map_evidence->state_version().value(), 7u);
  EXPECT_EQ(map_evidence->local_frame().value(), "base_link");
  EXPECT_EQ(map_evidence->representation_type(), uw::domain::MAP_REPRESENTATION_POINT_CLOUD);
  EXPECT_EQ(map_evidence->reintegration_policy(),
            uw::domain::MapEvidence::REINTEGRATION_POLICY_TRANSFORM_ONLY);

  ASSERT_EQ(map_evidence->geometry_or_occupancy().size(), 3 * sizeof(float));
  const auto* raw = reinterpret_cast<const float*>(map_evidence->geometry_or_occupancy().data());
  // Unproject(10,5,5.0) = (0,0,5) optical -> R^T*(0,0,5) = (5,0,0) body ->
  // + camera translation (0.1,0,0) = (5.1,0,0) base_link.
  EXPECT_NEAR(raw[0], 5.1f, 1e-4f);
  EXPECT_NEAR(raw[1], 0.0f, 1e-4f);
  EXPECT_NEAR(raw[2], 0.0f, 1e-4f);

  ASSERT_EQ(map_evidence->uncertainty_size(), 1);
  EXPECT_NEAR(map_evidence->uncertainty(0), 0.02, 1e-6);
}

TEST(AcousticOpticMapBridge, SkipsInvalidAndOpticalOnlyPixelsByDefaultConfig) {
  // valid_index pixel is ACOUSTIC_OPTIC (kept); every other pixel is
  // INVALID contribution (default) and correctly excluded.
  const auto rig = MakeRig();
  const auto fused_evidence = MakeFusedEvidence(20, 10, 110, 5.0f, 0.02f);
  uw::mapping::AcousticOpticMapBridgeParams params;
  const auto map_evidence =
      uw::mapping::BuildMapEvidenceFromFusedDepth(fused_evidence, rig, params, "kf3", 7);
  ASSERT_TRUE(map_evidence.has_value());
  // Only 1 of 200 pixels is non-INVALID contribution -> exactly 1 point.
  EXPECT_EQ(map_evidence->geometry_or_occupancy().size(), 3 * sizeof(float));
}

TEST(AcousticOpticMapBridge, ReturnsNulloptWhenEvidenceHasNoFusedDepthPayload) {
  const auto rig = MakeRig();
  uw::domain::PressureDepthMeasurement wrong_payload;
  uw::domain::EvidenceId id;
  id.set_value("not_fused");
  const auto not_fused_evidence = uw::domain::MakeEvidence(id, {}, wrong_payload, 1.0, "irrelevant_v1");
  uw::mapping::AcousticOpticMapBridgeParams params;
  EXPECT_FALSE(uw::mapping::BuildMapEvidenceFromFusedDepth(not_fused_evidence, rig, params, "kf3", 7)
                   .has_value());
}

TEST(AcousticOpticMapBridge, SurvivesPoseGraphCorrectionThroughRealSubmapManager) {
  // The design spec's actual completion condition (section 16): local
  // fused evidence must be re-transformable after a state update, not
  // baked into world frame — proven here against the REAL, pre-existing
  // SubmapManager, not a mock.
  const auto rig = MakeRig();
  const auto fused_evidence = MakeFusedEvidence(20, 10, 110, 5.0f, 0.02f);
  uw::mapping::AcousticOpticMapBridgeParams params;
  const auto map_evidence =
      uw::mapping::BuildMapEvidenceFromFusedDepth(fused_evidence, rig, params, "kf3", 7);
  ASSERT_TRUE(map_evidence.has_value());

  uw::mapping::SubmapManager manager;
  manager.AddMapEvidence(*map_evidence);

  uw::sensor_models::Pose3 pose1;
  pose1.translation = Eigen::Vector3d(10.0, 0.0, 0.0);
  manager.UpdateKeyframePose("kf3", pose1);
  auto points1 = manager.WorldPointsForKeyframe("kf3");
  ASSERT_EQ(points1.size(), 1u);
  EXPECT_NEAR((points1[0] - Eigen::Vector3d(15.1, 0.0, 0.0)).norm(), 0.0, 1e-4);

  // Pose-graph correction: pose changes, evidence is NOT re-added or
  // regenerated from the frontend, yet WorldPointsForKeyframe reflects it
  // immediately (same guarantee submap_manager_test.cpp already proves
  // generically — this test proves THIS plan's evidence source honors it).
  uw::sensor_models::Pose3 pose2;
  pose2.translation = Eigen::Vector3d(0.0, 20.0, 0.0);
  manager.UpdateKeyframePose("kf3", pose2);
  auto points2 = manager.WorldPointsForKeyframe("kf3");
  ASSERT_EQ(points2.size(), 1u);
  EXPECT_NEAR((points2[0] - Eigen::Vector3d(5.1, 20.0, 0.0)).norm(), 0.0, 1e-4);
  EXPECT_TRUE(manager.StaleKeyframes().empty());
}
```

- [ ] **Step 3: Verify it fails to compile**

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH="$HOME/miniconda3/envs/uw_slam_build"
cmake --build build --target uw_acoustic_optic_map_bridge_test -j"$(nproc)"
```

- [ ] **Step 4: Implement**

```cpp
// algorithms/mapping/acoustic_optic_map_bridge/include/uw/mapping/acoustic_optic_map_bridge.hpp
//
// Converts plan 4's FusedDepthMeasurement into local-frame MapEvidence
// (design spec section 7/16, "mapping handoff"). Every valid pixel whose
// contribution_mask != DEPTH_CONTRIBUTION_INVALID is unprojected and
// expressed in BASE_LINK frame — not camera-optical, not world — because
// that is the frame algorithms/mapping/submap_manager (pre-existing, NOT
// modified by this plan) composes with a keyframe's pose_WB via
// pose_WB.Apply(local). Points are unprojected via the same fixed
// body/optical rotation used since plan 3
// (uw::sensor_models::OpticalFromBodyRotation) and the same rig-derived
// camera extrinsic used throughout plans 2-4 — no new geometry primitive.
#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "uw/domain/domain.hpp"

namespace uw::mapping {

struct AcousticOpticMapBridgeParams {
  std::string camera_sensor_id = "camera_left";
  std::string camera_frame = "camera_left_link";
};

// Returns nullopt if fused_evidence has no FusedDepthMeasurement payload,
// or if the rig cannot resolve the named camera's intrinsics/extrinsic
// (fail-closed, matching plan 3/4's CALIBRATION-rejection precedent).
std::optional<uw::domain::MapEvidence> BuildMapEvidenceFromFusedDepth(
    const uw::domain::MeasurementEvidence& fused_evidence,
    const uw::domain::RigCalibrationSnapshot& rig, const AcousticOpticMapBridgeParams& params,
    const std::string& keyframe_id, uint64_t state_version);

}  // namespace uw::mapping
```

```cpp
// algorithms/mapping/acoustic_optic_map_bridge/src/acoustic_optic_map_bridge.cpp
#include "uw/mapping/acoustic_optic_map_bridge.hpp"

#include <optional>
#include <vector>

#include "uw/sensor_models/camera_model.hpp"
#include "uw/sensor_models/geometry.hpp"

namespace uw::mapping {

namespace {

const uw::domain::CameraIntrinsics* FindCamera(const uw::domain::RigCalibrationSnapshot& rig,
                                                const std::string& sensor_id) {
  for (const auto& camera : rig.cameras()) {
    if (camera.sensor_id().value() == sensor_id) return &camera;
  }
  return nullptr;
}

std::optional<uw::sensor_models::Pose3> FindEdgePose(const uw::domain::RigCalibrationSnapshot& rig,
                                                      const std::string& child_frame) {
  for (const auto& edge : rig.frame_tree()) {
    if (edge.child_frame().value() != child_frame) continue;
    return uw::sensor_models::Pose3::FromProto(edge.transform());
  }
  return std::nullopt;
}

}  // namespace

std::optional<uw::domain::MapEvidence> BuildMapEvidenceFromFusedDepth(
    const uw::domain::MeasurementEvidence& fused_evidence,
    const uw::domain::RigCalibrationSnapshot& rig, const AcousticOpticMapBridgeParams& params,
    const std::string& keyframe_id, uint64_t state_version) {
  if (!uw::domain::HasPayload<uw::domain::FusedDepthMeasurement>(fused_evidence)) {
    return std::nullopt;
  }
  const auto& fused = uw::domain::GetPayload<uw::domain::FusedDepthMeasurement>(fused_evidence);

  const auto* camera_intrinsics = FindCamera(rig, params.camera_sensor_id);
  const auto camera_pose = FindEdgePose(rig, params.camera_frame);
  if (camera_intrinsics == nullptr || !camera_pose.has_value()) return std::nullopt;
  const auto camera = uw::sensor_models::PinholeCamera::FromIntrinsics(*camera_intrinsics);

  std::vector<float> points_xyz;
  const std::size_t pixels = static_cast<std::size_t>(fused.width()) * fused.height();
  std::vector<double> uncertainty;
  for (std::size_t i = 0; i < pixels; ++i) {
    if (i >= fused.contribution_mask().size() ||
        static_cast<unsigned char>(fused.contribution_mask()[i]) == uw::domain::DEPTH_CONTRIBUTION_INVALID) {
      continue;
    }
    if (i >= fused.valid_mask().size() || fused.valid_mask()[i] == 0) continue;
    const double depth_m = fused.depth_m(static_cast<int>(i));
    if (!(depth_m > 0.0)) continue;

    const double u = static_cast<double>(i % fused.width());
    const double v = static_cast<double>(i / fused.width());
    const Eigen::Vector3d point_optical = camera.Unproject(u, v, depth_m);
    const Eigen::Vector3d point_camera_body =
        uw::sensor_models::OpticalFromBodyRotation().transpose() * point_optical;
    const Eigen::Vector3d point_base_link = camera_pose->Apply(point_camera_body);

    points_xyz.push_back(static_cast<float>(point_base_link.x()));
    points_xyz.push_back(static_cast<float>(point_base_link.y()));
    points_xyz.push_back(static_cast<float>(point_base_link.z()));
    uncertainty.push_back(fused.variance_m2(static_cast<int>(i)));
  }

  uw::domain::MapEvidence evidence;
  evidence.mutable_evidence_id()->set_value(fused_evidence.evidence_id().value() + "_map");
  evidence.mutable_keyframe_id()->set_value(keyframe_id);
  evidence.mutable_state_version()->set_value(state_version);
  evidence.mutable_local_frame()->set_value("base_link");
  evidence.set_representation_type(uw::domain::MAP_REPRESENTATION_POINT_CLOUD);
  evidence.set_geometry_or_occupancy(
      std::string(reinterpret_cast<const char*>(points_xyz.data()), points_xyz.size() * sizeof(float)));
  for (double u : uncertainty) evidence.add_uncertainty(u);
  *evidence.mutable_source_observations() = fused_evidence.source_observations();
  evidence.set_reintegration_policy(uw::domain::MapEvidence::REINTEGRATION_POLICY_TRANSFORM_ONLY);
  return evidence;
}

}  // namespace uw::mapping
```

- [ ] **Step 5: Build and run**

```bash
cmake --build build --target uw_acoustic_optic_map_bridge_test -j"$(nproc)"
ctest --test-dir build -R '^uw_acoustic_optic_map_bridge_test$' --output-on-failure
```

Expected: all 4 tests pass, including the real-`SubmapManager` retransform test.

- [ ] **Step 6: Review checkpoint**

Confirm `algorithms/mapping/submap_manager/{include,src}` has a zero-line diff (this plan
must not modify it — `git diff -- algorithms/mapping/submap_manager` should be empty).
Confirm the packed-point byte layout matches `submap_manager_test.cpp`'s own
`MakePointCloudEvidence` helper exactly (little-endian float32 x,y,z triples via
`reinterpret_cast`, no manual byte-order handling — matches the platform's existing
convention, not a new one). Do not commit without explicit authorization.

---

### Task 2: Documentation and phase-wide (and series-wide) verification

**Files:**
- Modify: `docs/uw-slam-codebase-reference-2026-08-18.md`
- Verify only otherwise.

- [ ] **Step 1: Add a `6.11` subsection**

Document `BuildMapEvidenceFromFusedDepth`'s frame chain and its `base_link`-frame choice
(contrasted explicitly with `apps/replay_demo`'s existing `local_frame="world"` landmark
shortcut — state plainly that this plan does not change that code path, it adds a second,
differently-scoped one). Note this closes the 6-plan acoustic-optic series: contracts →
optical baseline → cross-modal geometry → probabilistic fusion → simulation/evaluation →
mapping handoff — and that **none of the six plans wire into `apps/replay_demo`'s actual
pose-graph loop**; that integration (driving `replay_demo` to construct and run
`StereoOpticalDepthFrontend`/`AcousticOpticDepthFusionFrontend`/
`BuildMapEvidenceFromFusedDepth` against a real bag) remains unimplemented and should not be
described as done.

- [ ] **Step 2: Scan for contradictory capability claims**

```bash
rg -n 'camera|optical|stereo|声光|synchroniz|associat|fusion|posterior|scenario|mapping|MapEvidence' \
  README.md configs/README.md docs/uw-slam-codebase-reference-2026-08-18.md
```

Expected: no line claims `replay_demo` consumes acoustic-optic fused depth or that the
6-plan series is wired into the live pose-graph loop.

- [ ] **Step 3: Full build and test suite**

```bash
export PATH="$HOME/miniconda3/envs/uw_slam_build/bin:$PATH"
cmake -S . -B build -DCMAKE_PREFIX_PATH="$HOME/miniconda3/envs/uw_slam_build"
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
cd adapters/holoocean && python -m pytest -q && cd ../..
tools/lint/check_no_ros_in_core.sh
```

Expected: 100% passed (prior 23 + this plan's 1 new test target with 4 cases = 24); Python
11/11; lint clean.

- [ ] **Step 4: Confirm `submap_manager` is untouched**

```bash
git diff --stat -- algorithms/mapping/submap_manager
```

Expected: no output (zero-line diff).

- [ ] **Step 5: Final review checkpoint**

Run `git status --short`, list every changed/new tracked file, compare against this plan's
file map. Report the verification commands and outputs to the user, and summarize the
6-plan series' actual end state (what runs, what's still unwired). Do not commit unless the
user explicitly requests it.
