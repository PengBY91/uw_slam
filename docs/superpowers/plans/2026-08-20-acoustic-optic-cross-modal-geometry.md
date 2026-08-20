# Acoustic-Optic Cross-Modal Geometry Implementation Plan

**Goal:** Implement plan 3 of the 6-plan acoustic-optic series: a capture-time
synchronizer, FLS arc-band projection into the camera frame, geometric
candidate generation, and an association audit that produces
`AcousticOpticAssociationRecord`s — without running any posterior depth
optimization (that is plan 4's job).

**Architecture:** Builds directly on plan 1 (contracts: `AcousticOpticAssociationRecord`,
`AcousticOpticAssociationStatus/Reason` enums, already in `measurement.proto`) and
plan 2 (`PinholeCamera`/`StereoGeometry` in `core/sensor_models`, `OpticalDepthPriorMeasurement`
producer). New code: a pure synchronizer function in `runtime/`, an FLS arc
projector in `core/sensor_models/` (design spec section 5.2 explicitly places
"FLS arc geometry" there), and a new `algorithms/frontends/acoustic_optic_associator/`
module (design spec section 5.2 groups "sonar projection/association" under
`algorithms/frontends/`, alongside `sonar_cfar_frontend` and
`stereo_optical_depth_frontend`).

**A convention issue this plan must resolve explicitly:** `configs/rig/example_auv.yaml`'s
`camera_left_link`/`camera_right_link`/`sonar_link` frame_tree edges are all defined in
this platform's body convention (x-forward, y-left, z-up — the same convention the
design spec states for the sonar's own local frame, section 8.1). `PinholeCamera::Project`/
`Unproject` (plan 2) operate in the standard optical convention (z-forward, x-right,
y-down) — plan 2 never had to reconcile these because `StereoOpticalDepthFrontend`
only ever used `baseline_m`/`fx` magnitudes, never composed a rig `Pose3` with
`Project`. This plan is the first to compose a rig-derived `Pose3` (sonar→camera)
with `PinholeCamera::Project`, so it must apply a fixed body→optical rotation
(a hardware-mounting constant, not a per-rig calibration value) wherever that
composition happens. This is implemented once, in `core/sensor_models`, and
documented there — see Task 1.

**Tech Stack:** C++17, Eigen, GoogleTest. No new external dependency.

**Scope boundary (do not implement in this plan):**
- Robust posterior depth optimization, `posterior_depth_m`/`posterior_variance_m2`,
  or the `POSTERIOR_INVALID`/`VARIANCE_NOT_IMPROVED` reasons — plan 4
  (`AcousticOpticDepthFusionFrontend`) owns those. This plan's association
  records leave `posterior_depth_m`/`posterior_variance_m2` at their zero
  default and never sets those two reasons.
- `FusedDepthMeasurement` construction — that requires the posterior update
  (plan 4) to actually produce a fused per-pixel depth grid; this plan's
  output type is an in-process `AssociationAuditResult` (a vector of
  `AcousticOpticAssociationRecord` protos), not a wire-level fused evidence
  message.
- Multi-hypothesis association: this plan inherits the existing repo-wide v1
  rule (`hypothesis.proto`'s documented limitation, already followed by
  `apps/replay_demo`) that only the top-ranked `HypothesisSet` candidate is
  consumed — `AcousticOpticAssociator::Associate` processes at most one
  sonar detection per call.
- Any change to `apps/replay_demo`, `apps/tools/synth_bag_gen`, or
  `apps/tools/synth_stereo_gen` — this plan adds no new app; end-to-end
  wiring happens once plan 4 has something meaningful to evaluate
  (a real fused depth grid).

Repository policy: do not create git commits unless the user explicitly
authorizes them. Each task ends with a review checkpoint, not a commit.

## File map

### Create

- `core/sensor_models/include/uw/sensor_models/sonar_arc_projector.hpp`
- `core/sensor_models/src/sonar_arc_projector.cpp`
- `core/sensor_models/test/sonar_arc_projector_test.cpp`
- `runtime/include/uw/runtime/acoustic_optic_synchronizer.hpp`
- `runtime/src/acoustic_optic_synchronizer.cpp`
- `runtime/test/acoustic_optic_synchronizer_test.cpp`
- `algorithms/frontends/acoustic_optic_associator/CMakeLists.txt`
- `algorithms/frontends/acoustic_optic_associator/include/uw/frontends/acoustic_optic_associator.hpp`
- `algorithms/frontends/acoustic_optic_associator/src/acoustic_optic_associator.cpp`
- `algorithms/frontends/acoustic_optic_associator/test/acoustic_optic_associator_test.cpp`

### Modify

- `core/sensor_models/include/uw/sensor_models/camera_model.hpp` — add `OpticalFromBodyRotation()`.
- `core/sensor_models/src/camera_model.cpp`
- `core/sensor_models/CMakeLists.txt` — add `sonar_arc_projector.cpp` + its test target.
- `runtime/CMakeLists.txt` — add `acoustic_optic_synchronizer.cpp` + its test target.
- `CMakeLists.txt` — register the new `acoustic_optic_associator` subdirectory.
- `docs/uw-slam-codebase-reference-2026-08-18.md` — document the new modules and the
  body/optical convention fix.

---

### Task 1: Body→optical rotation + FLS arc projector

**Files:**
- Modify: `core/sensor_models/include/uw/sensor_models/camera_model.hpp`, `src/camera_model.cpp`
- Create: `core/sensor_models/include/uw/sensor_models/sonar_arc_projector.hpp`, `src/sonar_arc_projector.cpp`, `test/sonar_arc_projector_test.cpp`
- Modify: `core/sensor_models/CMakeLists.txt`

- [ ] **Step 1: Add `OpticalFromBodyRotation()` to `camera_model.hpp`/`.cpp`**

Append to `camera_model.hpp` (inside `namespace uw::sensor_models`, after `StereoGeometry`):

```cpp
// Fixed axis-convention rotation relating a rig body-frame camera link
// (x-forward/y-left/z-up — this platform's frame_tree convention, matching
// the sonar's own local-frame convention, design spec section 8.1) to that
// same physical camera's OPTICAL frame (z-forward/x-right/y-down, REP-103).
// This is a hardware-mounting constant, not a per-rig calibration value.
// Project()/Unproject() above operate in optical convention; any caller
// that derives a camera-frame point via RigCalibrationSnapshot's
// frame_tree Pose3 composition (body convention) must rotate through this
// before calling Project/Unproject — see sonar_arc_projector.hpp, the
// first caller that needs it (plan 2's StereoOpticalDepthFrontend never
// did, since it only ever used baseline/fx magnitudes, not a composed
// Pose3).
const Eigen::Matrix3d& OpticalFromBodyRotation();
```

Append to `camera_model.cpp`:

```cpp
const Eigen::Matrix3d& OpticalFromBodyRotation() {
  static const Eigen::Matrix3d kRotation = (Eigen::Matrix3d() <<
       0.0, -1.0,  0.0,
       0.0,  0.0, -1.0,
       1.0,  0.0,  0.0).finished();
  return kRotation;
}
```

- [ ] **Step 2: Write the failing arc-projector tests**

Create `core/sensor_models/test/sonar_arc_projector_test.cpp`:

```cpp
#include <cmath>

#include <gtest/gtest.h>

#include "uw/sensor_models/sonar_arc_projector.hpp"

namespace {

uw::sensor_models::PinholeCamera MakeCamera() {
  uw::sensor_models::PinholeCamera camera;
  camera.fx = 100.0;
  camera.fy = 100.0;
  camera.cx = 10.0;
  camera.cy = 5.0;
  camera.width = 20;
  camera.height = 10;
  return camera;
}

}  // namespace

TEST(SonarArcProjector, ProjectsBoresightPointToImageCenter) {
  // range=5m straight ahead (bearing=0), zero-aperture single sample
  // (phi=0) — with camera co-located and co-oriented with the sonar in
  // BODY convention, the fixed body->optical rotation puts this point on
  // the camera's optical axis, landing exactly at (cx, cy).
  const auto candidates = uw::sensor_models::ProjectSonarArcToCamera(
      /*range_m=*/5.0, /*bearing_rad=*/0.0, /*elevation_aperture_rad=*/0.0,
      uw::sensor_models::Pose3::Identity(), MakeCamera(), /*num_samples=*/1);
  ASSERT_EQ(candidates.size(), 1u);
  EXPECT_NEAR(candidates[0].pixel_u, 10.0, 1e-9);
  EXPECT_NEAR(candidates[0].pixel_v, 5.0, 1e-9);
  EXPECT_NEAR(candidates[0].point_sonar.x(), 5.0, 1e-9);
  EXPECT_NEAR(candidates[0].point_sonar.y(), 0.0, 1e-9);
  EXPECT_NEAR(candidates[0].point_sonar.z(), 0.0, 1e-9);
}

TEST(SonarArcProjector, UnprojectIsExactInverseOfProjectAtBoresight) {
  const auto camera = MakeCamera();
  const auto candidates = uw::sensor_models::ProjectSonarArcToCamera(
      5.0, 0.0, 0.0, uw::sensor_models::Pose3::Identity(), camera, 1);
  ASSERT_EQ(candidates.size(), 1u);

  const auto observed = uw::sensor_models::UnprojectPixelToSonarRangeBearing(
      candidates[0].pixel_u, candidates[0].pixel_v, /*depth_m=*/5.0,
      uw::sensor_models::Pose3::Identity(), camera);
  EXPECT_NEAR(observed.range_m, 5.0, 1e-9);
  EXPECT_NEAR(observed.bearing_rad, 0.0, 1e-9);
}

TEST(SonarArcProjector, RoundTripsForNonZeroBearing) {
  const auto camera = MakeCamera();
  const double range = 4.0;
  const double bearing = 0.3;
  const auto candidates = uw::sensor_models::ProjectSonarArcToCamera(
      range, bearing, 0.0, uw::sensor_models::Pose3::Identity(), camera, 1);
  ASSERT_EQ(candidates.size(), 1u);

  const auto observed = uw::sensor_models::UnprojectPixelToSonarRangeBearing(
      candidates[0].pixel_u, candidates[0].pixel_v, candidates[0].point_sonar.norm(),
      uw::sensor_models::Pose3::Identity(), camera);
  EXPECT_NEAR(observed.range_m, range, 1e-9);
  EXPECT_NEAR(observed.bearing_rad, bearing, 1e-9);
}

TEST(SonarArcProjector, SamplesFullApertureAndSkipsOutOfImageSamples) {
  // A wide aperture pushes some samples off-camera (small image, cx/cy
  // near center) — the projector must drop those, not clamp or fabricate.
  const auto candidates = uw::sensor_models::ProjectSonarArcToCamera(
      2.0, 0.0, /*elevation_aperture_rad=*/2.0, uw::sensor_models::Pose3::Identity(),
      MakeCamera(), /*num_samples=*/9);
  EXPECT_LT(candidates.size(), 9u);
  for (const auto& c : candidates) {
    EXPECT_GE(c.pixel_u, 0.0);
    EXPECT_LT(c.pixel_u, 20.0);
    EXPECT_GE(c.pixel_v, 0.0);
    EXPECT_LT(c.pixel_v, 10.0);
  }
}
```

- [ ] **Step 3: Verify it fails to compile**

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH="$HOME/miniconda3/envs/uw_slam_build"
cmake --build build --target uw_sensor_models_sonar_arc_projector_test -j"$(nproc)"
```

Expected: fails — target/types don't exist yet.

- [ ] **Step 4: Implement `sonar_arc_projector.hpp`/`.cpp`**

```cpp
// core/sensor_models/include/uw/sensor_models/sonar_arc_projector.hpp
#pragma once

#include <vector>

#include <Eigen/Core>

#include "uw/sensor_models/camera_model.hpp"
#include "uw/sensor_models/geometry.hpp"

namespace uw::sensor_models {

struct ArcCandidate {
  double phi_rad = 0.0;                          // elevation angle sampled within the aperture
  Eigen::Vector3d point_sonar = Eigen::Vector3d::Zero();  // 3D point in the sonar's frame
  double pixel_u = 0.0;
  double pixel_v = 0.0;
};

// Samples the ideal FLS arc p_S(phi) = rho * [cos(phi)cos(theta), cos(phi)sin(theta), sin(phi)],
// phi in [-aperture/2, +aperture/2] (design spec section 8.1), transforms each sample through
// `camera_T_sonar` (a rig-derived Pose3 in this platform's BODY convention — e.g.
// camera_left_edge.Inverse() * sonar_edge, both straight from RigCalibrationSnapshot's
// frame_tree) into the target camera's frame, applies the fixed body->optical rotation
// (OpticalFromBodyRotation(), camera_model.hpp), then projects with `camera`. Keeps only
// samples with positive optical-frame depth that land inside [0,width)x[0,height) — does not
// clamp or extrapolate out-of-image samples.
std::vector<ArcCandidate> ProjectSonarArcToCamera(double range_m, double bearing_rad,
                                                   double elevation_aperture_rad,
                                                   const Pose3& camera_T_sonar,
                                                   const PinholeCamera& camera, int num_samples);

struct SonarFrameObservation {
  double range_m = 0.0;
  double bearing_rad = 0.0;
};

// Inverse direction: given a pixel + a depth already resolved at that pixel (e.g. from an
// OpticalDepthPriorMeasurement), express the corresponding 3D point in the sonar's frame and
// reduce to range/bearing. Elevation is intentionally discarded on the way out — matches
// SonarRangeBearing's own contract (architecture invariant: FLS is 2D range-bearing only).
SonarFrameObservation UnprojectPixelToSonarRangeBearing(double pixel_u, double pixel_v,
                                                         double depth_m,
                                                         const Pose3& camera_T_sonar,
                                                         const PinholeCamera& camera);

}  // namespace uw::sensor_models
```

```cpp
// core/sensor_models/src/sonar_arc_projector.cpp
#include "uw/sensor_models/sonar_arc_projector.hpp"

#include <cmath>

namespace uw::sensor_models {

std::vector<ArcCandidate> ProjectSonarArcToCamera(double range_m, double bearing_rad,
                                                   double elevation_aperture_rad,
                                                   const Pose3& camera_T_sonar,
                                                   const PinholeCamera& camera, int num_samples) {
  std::vector<ArcCandidate> candidates;
  if (num_samples <= 0) return candidates;

  for (int i = 0; i < num_samples; ++i) {
    const double phi = num_samples > 1
                            ? -elevation_aperture_rad / 2.0 +
                                  elevation_aperture_rad * static_cast<double>(i) / (num_samples - 1)
                            : 0.0;
    const Eigen::Vector3d point_sonar(range_m * std::cos(phi) * std::cos(bearing_rad),
                                      range_m * std::cos(phi) * std::sin(bearing_rad),
                                      range_m * std::sin(phi));
    const Eigen::Vector3d point_camera_body = camera_T_sonar.Apply(point_sonar);
    const Eigen::Vector3d point_optical = OpticalFromBodyRotation() * point_camera_body;
    if (point_optical.z() <= 0.0) continue;

    const Eigen::Vector2d pixel = camera.Project(point_optical);
    if (pixel.x() < 0.0 || pixel.x() >= static_cast<double>(camera.width) || pixel.y() < 0.0 ||
        pixel.y() >= static_cast<double>(camera.height)) {
      continue;
    }
    candidates.push_back(ArcCandidate{phi, point_sonar, pixel.x(), pixel.y()});
  }
  return candidates;
}

SonarFrameObservation UnprojectPixelToSonarRangeBearing(double pixel_u, double pixel_v,
                                                         double depth_m,
                                                         const Pose3& camera_T_sonar,
                                                         const PinholeCamera& camera) {
  const Eigen::Vector3d point_optical = camera.Unproject(pixel_u, pixel_v, depth_m);
  const Eigen::Vector3d point_camera_body = OpticalFromBodyRotation().transpose() * point_optical;
  const Eigen::Vector3d point_sonar = camera_T_sonar.Inverse().Apply(point_camera_body);

  SonarFrameObservation observation;
  observation.range_m = point_sonar.norm();
  observation.bearing_rad = std::atan2(point_sonar.y(), point_sonar.x());
  return observation;
}

}  // namespace uw::sensor_models
```

- [ ] **Step 5: Wire CMake and run**

Modify `core/sensor_models/CMakeLists.txt`: add `src/sonar_arc_projector.cpp` to the
library's source list, and a `uw_sensor_models_sonar_arc_projector_test` target/test
following the exact pattern already used for `uw_sensor_models_camera_model_test`.

```bash
cmake --build build --target uw_sensor_models_sonar_arc_projector_test -j"$(nproc)"
ctest --test-dir build -R '^uw_sensor_models_sonar_arc_projector_test$' --output-on-failure
```

Expected: all 4 tests pass.

- [ ] **Step 6: Review checkpoint**

Confirm `OpticalFromBodyRotation()` is applied consistently (forward: multiply; inverse:
transpose, since it's orthonormal) and that `camera_model.hpp`'s existing `Project`/`Unproject`
signatures are unchanged (plan 2's tests must still pass — `ctest -R camera_model_test`).
Do not commit without explicit authorization.

---

### Task 2: Capture-time synchronizer

**Files:**
- Create: `runtime/include/uw/runtime/acoustic_optic_synchronizer.hpp`, `src/acoustic_optic_synchronizer.cpp`, `test/acoustic_optic_synchronizer_test.cpp`
- Modify: `runtime/CMakeLists.txt`

- [ ] **Step 1: Write the failing tests**

Create `runtime/test/acoustic_optic_synchronizer_test.cpp`:

```cpp
#include <gtest/gtest.h>

#include "uw/runtime/acoustic_optic_synchronizer.hpp"

namespace {

uw::domain::ImageFrame MakeImage(const std::string& sensor_id, int64_t seconds, int32_t nanos) {
  uw::domain::ImageFrame image;
  image.mutable_header()->mutable_sensor_id()->set_value(sensor_id);
  image.mutable_header()->mutable_capture_time()->set_seconds(seconds);
  image.mutable_header()->mutable_capture_time()->set_nanos(nanos);
  return image;
}

uw::domain::SonarFrame MakeSonar(const std::string& sensor_id, int64_t seconds, int32_t nanos) {
  uw::domain::SonarFrame sonar;
  sonar.mutable_header()->mutable_sensor_id()->set_value(sensor_id);
  sonar.mutable_header()->mutable_capture_time()->set_seconds(seconds);
  sonar.mutable_header()->mutable_capture_time()->set_nanos(nanos);
  return sonar;
}

uw::domain::RigCalibrationSnapshot MakeRigWithOffsets() {
  uw::domain::RigCalibrationSnapshot rig;
  (*rig.mutable_time_offset_seconds())["camera_left"] = 0.0;
  (*rig.mutable_time_offset_seconds())["sonar0"] = 0.01;  // sonar clock reads 10ms late
  return rig;
}

}  // namespace

TEST(AcousticOpticSynchronizer, AcceptsFramesWithinToleranceAfterOffsetCorrection) {
  const auto rig = MakeRigWithOffsets();
  const auto left = MakeImage("camera_left", 100, 0);
  // Raw sonar capture_time is 100.005s; corrected: 100.005 - 0.01 = 99.995s (offset applied as
  // t_reference = t_sensor_capture + time_offset_seconds[sensor_id], so recovering t_sensor's
  // OWN reading back into reference time subtracts nothing extra — the sonar's header already
  // carries its own clock's capture_time, and time_offset_seconds shifts it directly).
  const auto sonar = MakeSonar("sonar0", 99, 995'000'000);  // 99.995s raw

  uw::runtime::SynchronizerParams params;
  params.max_time_delta_s = 0.02;
  const auto bundle = uw::runtime::SynchronizeAcousticOptic(left, std::nullopt, sonar, rig, params);
  ASSERT_TRUE(bundle.has_value());
  EXPECT_NEAR(bundle->max_pairwise_time_delta_s, 0.0, 1e-6);
}

TEST(AcousticOpticSynchronizer, RejectsFramesBeyondTolerance) {
  const auto rig = MakeRigWithOffsets();
  const auto left = MakeImage("camera_left", 100, 0);
  const auto sonar = MakeSonar("sonar0", 100, 200'000'000);  // 200ms raw drift, no offset applied here

  uw::runtime::SynchronizerParams params;
  params.max_time_delta_s = 0.02;
  const auto bundle = uw::runtime::SynchronizeAcousticOptic(left, std::nullopt, sonar, rig, params);
  EXPECT_FALSE(bundle.has_value());
}

TEST(AcousticOpticSynchronizer, DefaultsMissingSensorOffsetToZero) {
  uw::domain::RigCalibrationSnapshot rig;  // no time_offset_seconds entries at all
  const auto left = MakeImage("camera_left", 100, 0);
  const auto sonar = MakeSonar("sonar0", 100, 5'000'000);  // 5ms drift

  uw::runtime::SynchronizerParams params;
  params.max_time_delta_s = 0.02;
  const auto bundle = uw::runtime::SynchronizeAcousticOptic(left, std::nullopt, sonar, rig, params);
  ASSERT_TRUE(bundle.has_value());
  EXPECT_NEAR(bundle->max_pairwise_time_delta_s, 0.005, 1e-6);
}
```

- [ ] **Step 2: Verify it fails to compile**

```bash
cmake --build build --target uw_runtime_acoustic_optic_synchronizer_test -j"$(nproc)"
```

- [ ] **Step 3: Implement**

```cpp
// runtime/include/uw/runtime/acoustic_optic_synchronizer.hpp
//
// Pure, stateless capture-time pairing (design spec section 5.1): given
// already-captured frames and the rig's time_offset_seconds
// (t_reference = t_sensor_capture + time_offset_seconds[sensor_id]),
// decides whether they form one valid synchronized bundle. Uses
// capture_time, not receive_time. Rejects (nullopt) rather than
// extrapolating when any pairwise corrected-time delta exceeds
// max_time_delta_s — no motion model exists yet to interpolate across.
// A sensor_id missing from time_offset_seconds defaults to a zero offset
// (documented v1 simplification; full audit trail via RunManifest/health
// is a later integration concern, not this pure function's job).
#pragma once

#include <optional>

#include "uw/domain/domain.hpp"
#include "uw/measurement_api/frontend.hpp"

namespace uw::runtime {

struct SynchronizerParams {
  double max_time_delta_s = 0.05;
};

struct SynchronizedAcousticOpticBundle {
  uw::measurement_api::CameraFrameBundle images;
  uw::domain::SonarFrame sonar;
  double max_pairwise_time_delta_s = 0.0;
};

std::optional<SynchronizedAcousticOpticBundle> SynchronizeAcousticOptic(
    const uw::domain::ImageFrame& primary, const std::optional<uw::domain::ImageFrame>& secondary,
    const uw::domain::SonarFrame& sonar, const uw::domain::RigCalibrationSnapshot& rig,
    const SynchronizerParams& params);

}  // namespace uw::runtime
```

```cpp
// runtime/src/acoustic_optic_synchronizer.cpp
#include "uw/runtime/acoustic_optic_synchronizer.hpp"

#include <algorithm>
#include <vector>

namespace uw::runtime {

namespace {

double CorrectedTime(const uw::domain::ObservationHeader& header,
                     const uw::domain::RigCalibrationSnapshot& rig) {
  const double offset = rig.time_offset_seconds().count(header.sensor_id().value()) > 0
                            ? rig.time_offset_seconds().at(header.sensor_id().value())
                            : 0.0;
  return uw::domain::ToSeconds(header.capture_time()) + offset;
}

}  // namespace

std::optional<SynchronizedAcousticOpticBundle> SynchronizeAcousticOptic(
    const uw::domain::ImageFrame& primary, const std::optional<uw::domain::ImageFrame>& secondary,
    const uw::domain::SonarFrame& sonar, const uw::domain::RigCalibrationSnapshot& rig,
    const SynchronizerParams& params) {
  std::vector<double> times;
  times.push_back(CorrectedTime(primary.header(), rig));
  if (secondary.has_value()) times.push_back(CorrectedTime(secondary->header(), rig));
  times.push_back(CorrectedTime(sonar.header(), rig));

  const double max_time = *std::max_element(times.begin(), times.end());
  const double min_time = *std::min_element(times.begin(), times.end());
  const double delta = max_time - min_time;
  if (delta > params.max_time_delta_s) return std::nullopt;

  SynchronizedAcousticOpticBundle bundle;
  bundle.images.primary = primary;
  bundle.images.secondary = secondary;
  bundle.sonar = sonar;
  bundle.max_pairwise_time_delta_s = delta;
  return bundle;
}

}  // namespace uw::runtime
```

- [ ] **Step 4: Wire CMake and run**

Modify `runtime/CMakeLists.txt`: add `src/acoustic_optic_synchronizer.cpp` to
`uw_runtime`'s sources, and a test target following the exact pattern already used for
`uw_runtime_config_test` (same `UW_REPO_ROOT` injection is NOT needed here — this test
reads no files).

```bash
cmake --build build --target uw_runtime_acoustic_optic_synchronizer_test -j"$(nproc)"
ctest --test-dir build -R '^uw_runtime_acoustic_optic_synchronizer_test$' --output-on-failure
```

Expected: all 3 tests pass.

- [ ] **Step 5: Review checkpoint**

Confirm the function is pure (no I/O, no mutable state) and the offset sign convention
matches `configs/README.md`'s documented `t_reference = t_sensor_capture +
time_offset_seconds[sensor_id]` rule from plan 1. Do not commit without explicit
authorization.

---

### Task 3: Acoustic-optic association audit

**Files:**
- Create: `algorithms/frontends/acoustic_optic_associator/CMakeLists.txt`, `include/uw/frontends/acoustic_optic_associator.hpp`, `src/acoustic_optic_associator.cpp`, `test/acoustic_optic_associator_test.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Create the module skeleton**

Create `algorithms/frontends/acoustic_optic_associator/CMakeLists.txt`:

```cmake
add_library(uw_acoustic_optic_associator STATIC
  src/acoustic_optic_associator.cpp
)
target_include_directories(uw_acoustic_optic_associator PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(uw_acoustic_optic_associator PUBLIC uw_measurement_api uw_sensor_models)
target_compile_options(uw_acoustic_optic_associator PRIVATE -Wall -Wextra)

if(UW_BUILD_TESTS)
  add_executable(uw_acoustic_optic_associator_test test/acoustic_optic_associator_test.cpp)
  target_link_libraries(uw_acoustic_optic_associator_test PRIVATE uw_acoustic_optic_associator GTest::gtest GTest::gtest_main)
  add_test(NAME uw_acoustic_optic_associator_test COMMAND uw_acoustic_optic_associator_test)
endif()
```

Add `add_subdirectory(algorithms/frontends/acoustic_optic_associator)` to the top-level
`CMakeLists.txt`, after `stereo_optical_depth_frontend`.

- [ ] **Step 2: Write the failing tests**

Create `algorithms/frontends/acoustic_optic_associator/test/acoustic_optic_associator_test.cpp`:

```cpp
#include <gtest/gtest.h>

#include "uw/frontends/acoustic_optic_associator.hpp"

namespace {

uw::domain::RigCalibrationSnapshot MakeCoLocatedRig() {
  uw::domain::RigCalibrationSnapshot rig;
  auto* camera = rig.add_cameras();
  camera->mutable_sensor_id()->set_value("camera_left");
  camera->set_width(20);
  camera->set_height(10);
  for (double v : {100.0, 0.0, 10.0, 0.0, 100.0, 5.0, 0.0, 0.0, 1.0}) camera->add_k_matrix_row_major(v);

  auto add_identity_edge = [&](const std::string& child) {
    auto* edge = rig.add_frame_tree();
    edge->mutable_parent_frame()->set_value("base_link");
    edge->mutable_child_frame()->set_value(child);
    for (double v : {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0}) {
      edge->mutable_transform()->add_matrix_row_major(v);
    }
  };
  add_identity_edge("camera_left_link");
  add_identity_edge("sonar_link");

  auto* beam_model = rig.add_sonar_beam_models();
  beam_model->mutable_sensor_id()->set_value("sonar0");
  beam_model->set_elevation_aperture_rad(0.0);
  return rig;
}

uw::domain::HypothesisSet MakeSonarHypothesis(double range_m, double bearing_rad) {
  uw::domain::SonarRangeBearing measurement;
  measurement.set_range_m(range_m);
  measurement.set_bearing_rad(bearing_rad);
  measurement.set_range_sigma_m(0.1);
  measurement.set_bearing_sigma_rad(0.05);
  uw::domain::EvidenceId id;
  id.set_value("sonar_hyp_1");
  auto evidence = uw::domain::MakeEvidence(id, {}, measurement, 1.0, "sonar_cfar_frontend_v1");
  uw::domain::HypothesisSet hypotheses;
  *hypotheses.add_candidates() = evidence;
  hypotheses.add_calibrated_likelihoods(1.0);
  return hypotheses;
}

uw::domain::MeasurementEvidence MakeOpticalEvidence(int width, int height, int valid_index,
                                                     float depth_m,
                                                     uw::domain::OpticalDepthScaleStatus scale) {
  uw::domain::OpticalDepthPriorMeasurement prior;
  prior.set_width(width);
  prior.set_height(height);
  const int pixels = width * height;
  std::string valid_mask(pixels, '\0');
  for (int i = 0; i < pixels; ++i) {
    prior.add_depth_m(i == valid_index ? depth_m : 0.0f);
    prior.add_variance_m2(i == valid_index ? 0.01f : 0.0f);
  }
  valid_mask[valid_index] = 1;
  prior.set_valid_mask(valid_mask);
  prior.set_scale_status(scale);
  prior.set_producer_type("stereo");
  uw::domain::EvidenceId id;
  id.set_value("optical_1");
  return uw::domain::MakeEvidence(id, {}, prior, 1.0, "stereo_depth_frontend_v1");
}

}  // namespace

TEST(AcousticOpticAssociator, AcceptsConsistentBoresightDetection) {
  const auto rig = MakeCoLocatedRig();
  const auto sonar_hypotheses = MakeSonarHypothesis(/*range_m=*/5.0, /*bearing_rad=*/0.0);
  // Pixel (10,5) = index 5*20+10 = 110, matching Task 1's boresight math (range=5 -> pixel
  // center of a 20x10 image with cx=10, cy=5).
  const auto optical_evidence =
      MakeOpticalEvidence(20, 10, /*valid_index=*/110, /*depth_m=*/5.0,
                          uw::domain::OPTICAL_DEPTH_SCALE_STATUS_METRIC);

  uw::frontends::AcousticOpticAssociatorParams params;
  uw::frontends::AcousticOpticAssociator associator(params);
  const auto result = associator.Associate(sonar_hypotheses, optical_evidence, rig, /*time_delta_seconds=*/0.01);

  ASSERT_EQ(result.records.size(), 1u);
  const auto& record = result.records[0];
  EXPECT_EQ(record.status(), uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_ACCEPTED);
  EXPECT_EQ(record.reason(), uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_NONE);
  EXPECT_TRUE(record.has_selected_pixel());
  EXPECT_EQ(record.selected_pixel_index(), 110u);
  EXPECT_NEAR(record.prior_depth_m(), 5.0, 1e-6);
  EXPECT_NEAR(record.best_score(), 0.0, 1e-6);
  EXPECT_NEAR(record.time_delta_seconds(), 0.01, 1e-9);
  // Posterior fields are explicitly plan 4's job — this plan must not set them.
  EXPECT_EQ(record.posterior_depth_m(), 0.0);
  EXPECT_EQ(record.posterior_variance_m2(), 0.0);
}

TEST(AcousticOpticAssociator, RejectsWhenNoOpticalPixelSurvivesTheRangeGate) {
  const auto rig = MakeCoLocatedRig();
  const auto sonar_hypotheses = MakeSonarHypothesis(5.0, 0.0);
  // Optical depth at the boresight pixel is 9.0m, far outside a tight range gate for a
  // 5.0m sonar detection.
  const auto optical_evidence =
      MakeOpticalEvidence(20, 10, 110, /*depth_m=*/9.0, uw::domain::OPTICAL_DEPTH_SCALE_STATUS_METRIC);

  uw::frontends::AcousticOpticAssociatorParams params;
  params.range_gate_m = 0.5;
  uw::frontends::AcousticOpticAssociator associator(params);
  const auto result = associator.Associate(sonar_hypotheses, optical_evidence, rig, 0.0);

  ASSERT_EQ(result.records.size(), 1u);
  EXPECT_EQ(result.records[0].status(), uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_REJECTED);
  EXPECT_EQ(result.records[0].reason(), uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_NO_CANDIDATE);
}

TEST(AcousticOpticAssociator, RejectsRelativeScaleOpticalPrior) {
  const auto rig = MakeCoLocatedRig();
  const auto sonar_hypotheses = MakeSonarHypothesis(5.0, 0.0);
  const auto optical_evidence = MakeOpticalEvidence(
      20, 10, 110, 5.0, uw::domain::OPTICAL_DEPTH_SCALE_STATUS_RELATIVE_SCALE);

  uw::frontends::AcousticOpticAssociatorParams params;
  uw::frontends::AcousticOpticAssociator associator(params);
  const auto result = associator.Associate(sonar_hypotheses, optical_evidence, rig, 0.0);

  ASSERT_EQ(result.records.size(), 1u);
  EXPECT_EQ(result.records[0].status(), uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_REJECTED);
  EXPECT_EQ(result.records[0].reason(), uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_SCALE);
}

TEST(AcousticOpticAssociator, ReturnsNoRecordsWhenHypothesisSetIsEmpty) {
  const auto rig = MakeCoLocatedRig();
  uw::domain::HypothesisSet empty_hypotheses;
  const auto optical_evidence =
      MakeOpticalEvidence(20, 10, 110, 5.0, uw::domain::OPTICAL_DEPTH_SCALE_STATUS_METRIC);

  uw::frontends::AcousticOpticAssociatorParams params;
  uw::frontends::AcousticOpticAssociator associator(params);
  const auto result = associator.Associate(empty_hypotheses, optical_evidence, rig, 0.0);
  EXPECT_TRUE(result.records.empty());
}
```

- [ ] **Step 3: Verify it fails to compile**

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH="$HOME/miniconda3/envs/uw_slam_build"
cmake --build build --target uw_acoustic_optic_associator_test -j"$(nproc)"
```

- [ ] **Step 4: Implement**

```cpp
// algorithms/frontends/acoustic_optic_associator/include/uw/frontends/acoustic_optic_associator.hpp
//
// Cross-modal geometric association (design spec section 8.2/8.3) — NOT
// the posterior depth update (section 8.4, plan 4's
// AcousticOpticDepthFusionFrontend). Produces AcousticOpticAssociationRecords
// with prior_depth_m/prior_variance_m2 filled in and
// posterior_depth_m/posterior_variance_m2 left at zero; never sets
// POSTERIOR_INVALID or VARIANCE_NOT_IMPROVED. Inherits the repo-wide v1
// rule (hypothesis.proto) of consuming only the top-ranked HypothesisSet
// candidate — at most one record per call.
#pragma once

#include <string>
#include <vector>

#include "uw/domain/domain.hpp"
#include "uw/sensor_models/camera_model.hpp"

namespace uw::frontends {

struct AcousticOpticAssociatorParams {
  std::string camera_sensor_id = "camera_left";
  std::string camera_frame = "camera_left_link";
  std::string sonar_sensor_id = "sonar0";
  std::string sonar_frame = "sonar_link";
  int arc_samples = 16;
  double range_gate_m = 0.5;
  double bearing_gate_rad = 0.1;
  double ambiguity_margin = 1.0;  // second_best_score - best_score must exceed this to accept
  int max_candidates = 8;
};

struct AssociationAuditResult {
  std::vector<uw::domain::AcousticOpticAssociationRecord> records;
  uw::domain::HealthReport health;
};

class AcousticOpticAssociator {
 public:
  explicit AcousticOpticAssociator(AcousticOpticAssociatorParams params);

  AssociationAuditResult Associate(const uw::domain::HypothesisSet& sonar_hypotheses,
                                   const uw::domain::MeasurementEvidence& optical_evidence,
                                   const uw::domain::RigCalibrationSnapshot& rig,
                                   double time_delta_seconds);

 private:
  AcousticOpticAssociatorParams params_;
  uint64_t frames_processed_ = 0;
  uint64_t frames_accepted_ = 0;
};

}  // namespace uw::frontends
```

```cpp
// algorithms/frontends/acoustic_optic_associator/src/acoustic_optic_associator.cpp
#include "uw/frontends/acoustic_optic_associator.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

#include "uw/sensor_models/sonar_arc_projector.hpp"

namespace uw::frontends {

namespace {

const uw::domain::CameraIntrinsics* FindCamera(const uw::domain::RigCalibrationSnapshot& rig,
                                                const std::string& sensor_id) {
  for (const auto& camera : rig.cameras()) {
    if (camera.sensor_id().value() == sensor_id) return &camera;
  }
  return nullptr;
}

const uw::domain::SonarBeamModel* FindBeamModel(const uw::domain::RigCalibrationSnapshot& rig,
                                                 const std::string& sensor_id) {
  for (const auto& model : rig.sonar_beam_models()) {
    if (model.sensor_id().value() == sensor_id) return &model;
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

AcousticOpticAssociator::AcousticOpticAssociator(AcousticOpticAssociatorParams params)
    : params_(std::move(params)) {}

AssociationAuditResult AcousticOpticAssociator::Associate(
    const uw::domain::HypothesisSet& sonar_hypotheses,
    const uw::domain::MeasurementEvidence& optical_evidence,
    const uw::domain::RigCalibrationSnapshot& rig, double time_delta_seconds) {
  AssociationAuditResult result;
  result.health.set_component_id("acoustic_optic_associator");

  if (sonar_hypotheses.candidates_size() == 0) return result;
  ++frames_processed_;

  const auto& top_evidence = sonar_hypotheses.candidates(0);
  if (!uw::domain::HasPayload<uw::domain::SonarRangeBearing>(top_evidence)) return result;
  const auto& top_sonar = uw::domain::GetPayload<uw::domain::SonarRangeBearing>(top_evidence);

  uw::domain::AcousticOpticAssociationRecord record;
  *record.mutable_sonar_evidence_id() = top_evidence.evidence_id();
  record.set_time_delta_seconds(time_delta_seconds);

  if (!uw::domain::HasPayload<uw::domain::OpticalDepthPriorMeasurement>(optical_evidence)) {
    record.set_status(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_REJECTED);
    record.set_reason(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_NO_CANDIDATE);
    result.records.push_back(record);
    return result;
  }
  const auto& prior = uw::domain::GetPayload<uw::domain::OpticalDepthPriorMeasurement>(optical_evidence);
  if (prior.scale_status() != uw::domain::OPTICAL_DEPTH_SCALE_STATUS_METRIC) {
    record.set_status(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_REJECTED);
    record.set_reason(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_SCALE);
    result.records.push_back(record);
    return result;
  }

  const auto* camera_intrinsics = FindCamera(rig, params_.camera_sensor_id);
  const auto* beam_model = FindBeamModel(rig, params_.sonar_sensor_id);
  const auto camera_pose = FindEdgePose(rig, params_.camera_frame);
  const auto sonar_pose = FindEdgePose(rig, params_.sonar_frame);
  if (camera_intrinsics == nullptr || beam_model == nullptr || !camera_pose.has_value() ||
      !sonar_pose.has_value()) {
    record.set_status(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_REJECTED);
    record.set_reason(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_CALIBRATION);
    result.records.push_back(record);
    return result;
  }

  const auto camera = uw::sensor_models::PinholeCamera::FromIntrinsics(*camera_intrinsics);
  const uw::sensor_models::Pose3 camera_T_sonar = camera_pose->Inverse() * (*sonar_pose);

  const auto arc_candidates = uw::sensor_models::ProjectSonarArcToCamera(
      top_sonar.range_m(), top_sonar.bearing_rad(), beam_model->elevation_aperture_rad(),
      camera_T_sonar, camera, params_.arc_samples);

  struct Scored {
    std::size_t pixel_index = 0;
    double score = 0.0;
    float depth_m = 0.0f;
    float variance_m2 = 0.0f;
  };
  std::vector<Scored> passed;
  const double range_sigma = top_sonar.range_sigma_m() > 0.0 ? top_sonar.range_sigma_m() : 1.0;
  const double bearing_sigma = top_sonar.bearing_sigma_rad() > 0.0 ? top_sonar.bearing_sigma_rad() : 1.0;

  for (const auto& candidate : arc_candidates) {
    const int u = static_cast<int>(std::lround(candidate.pixel_u));
    const int v = static_cast<int>(std::lround(candidate.pixel_v));
    if (u < 0 || u >= static_cast<int>(prior.width()) || v < 0 || v >= static_cast<int>(prior.height())) {
      continue;
    }
    const std::size_t idx = static_cast<std::size_t>(v) * prior.width() + static_cast<std::size_t>(u);
    if (idx >= prior.valid_mask().size() || prior.valid_mask()[idx] == 0) continue;

    const double depth_m = prior.depth_m(static_cast<int>(idx));
    const auto observed = uw::sensor_models::UnprojectPixelToSonarRangeBearing(
        candidate.pixel_u, candidate.pixel_v, depth_m, camera_T_sonar, camera);
    const double range_residual = observed.range_m - top_sonar.range_m();
    const double bearing_residual = observed.bearing_rad - top_sonar.bearing_rad();
    if (std::abs(range_residual) > params_.range_gate_m) continue;
    if (std::abs(bearing_residual) > params_.bearing_gate_rad) continue;

    const double score = (range_residual * range_residual) / (range_sigma * range_sigma) +
                         (bearing_residual * bearing_residual) / (bearing_sigma * bearing_sigma);
    passed.push_back(Scored{idx, score, static_cast<float>(depth_m), prior.variance_m2(static_cast<int>(idx))});
  }

  if (passed.empty()) {
    record.set_status(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_REJECTED);
    record.set_reason(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_NO_CANDIDATE);
    result.records.push_back(record);
    return result;
  }
  std::sort(passed.begin(), passed.end(), [](const Scored& a, const Scored& b) { return a.score < b.score; });
  for (std::size_t i = 0; i < passed.size() && static_cast<int>(i) < params_.max_candidates; ++i) {
    record.add_candidate_pixel_indices(static_cast<uint32_t>(passed[i].pixel_index));
  }
  record.set_best_score(passed[0].score);

  if (passed.size() > 1) {
    record.set_second_best_score(passed[1].score);
    if (passed[1].score - passed[0].score < params_.ambiguity_margin) {
      record.set_status(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_AMBIGUOUS);
      record.set_reason(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_AMBIGUOUS_MARGIN);
      result.records.push_back(record);
      return result;
    }
  }

  record.set_status(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_ACCEPTED);
  record.set_reason(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_NONE);
  record.set_has_selected_pixel(true);
  record.set_selected_pixel_index(static_cast<uint32_t>(passed[0].pixel_index));
  record.set_prior_depth_m(passed[0].depth_m);
  record.set_prior_variance_m2(passed[0].variance_m2);
  ++frames_accepted_;
  result.records.push_back(record);
  return result;
}

}  // namespace uw::frontends
```

- [ ] **Step 5: Build and run**

```bash
cmake --build build --target uw_acoustic_optic_associator_test -j"$(nproc)"
ctest --test-dir build -R '^uw_acoustic_optic_associator_test$' --output-on-failure
```

Expected: all 4 tests pass.

- [ ] **Step 6: Review checkpoint**

Grep the implementation for `POSTERIOR_INVALID`/`VARIANCE_NOT_IMPROVED`/`posterior_depth_m`/
`posterior_variance_m2` setters — confirm none appear (that is plan 4's exclusive territory).
Confirm `CROSS_MODAL_CONFLICT` is likewise never set by this plan (design spec section 8.3
ties it to post-update residual, which this plan does not compute) — this plan's REJECTED
path only ever uses `NO_CANDIDATE`, `SCALE`, or `CALIBRATION`. Do not commit without explicit
authorization.

---

### Task 4: Documentation and phase-wide verification

**Files:**
- Modify: `docs/uw-slam-codebase-reference-2026-08-18.md`
- Verify only otherwise.

- [ ] **Step 1: Add a `6.8` subsection**

Document `OpticalFromBodyRotation()`'s convention fix, `sonar_arc_projector.hpp`'s two
functions, `AcousticOpticSynchronizer`'s pure-function contract, and
`AcousticOpticAssociator`'s scope (geometric association only, explicitly not posterior
optimization/fused depth). State plainly that none of this is wired into
`apps/replay_demo` yet — plan 4 is the first plan with something end-to-end worth
running (`FusedDepthMeasurement`).

- [ ] **Step 2: Scan for contradictory capability claims**

```bash
rg -n 'camera|optical|stereo|声光|synchroniz|associat' README.md configs/README.md docs/uw-slam-codebase-reference-2026-08-18.md
```

Expected: no line claims fusion, posterior depth, or `replay_demo` integration exist.

- [ ] **Step 3: Full build and test suite**

```bash
export PATH="$HOME/miniconda3/envs/uw_slam_build/bin:$PATH"
cmake -S . -B build -DCMAKE_PREFIX_PATH="$HOME/miniconda3/envs/uw_slam_build"
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
cd adapters/holoocean && python -m pytest -q && cd ../..
tools/lint/check_no_ros_in_core.sh
```

Expected: build exits 0; ctest 100% passed (the prior 18 plus this plan's 3 new targets =
21); Python still 11/11; lint exits 0.

- [ ] **Step 4: Confirm the phase boundary**

```bash
rg -n 'class AcousticOpticDepthFusionFrontend|FusedDepthMeasurement fused_depth' core algorithms runtime apps
```

Expected: no `AcousticOpticDepthFusionFrontend` implementation; the only
`FusedDepthMeasurement` reference is plan 1's payload-trait registration in
`core/domain/include/uw/domain/domain.hpp` — nothing constructs one.

- [ ] **Step 5: Final review checkpoint**

Run `git status --short`, list every changed/new tracked file, compare against this
plan's file map. Report the verification commands and outputs to the user. Do not commit
unless the user explicitly requests it.
