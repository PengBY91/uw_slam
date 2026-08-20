# Acoustic-Optic Optical Baseline Implementation Plan

**Goal:** Implement the stereo optical-depth baseline — a real pinhole camera
model, a deterministic block-matching stereo frontend that produces metric
`OpticalDepthPriorMeasurement` evidence, a synthetic stereo/GT-depth data
path (this dev machine has neither HoloOcean nor ROS2, per
`apps/tools/synth_bag_gen`'s header), and depth evaluation metrics —
without touching sonar/optical fusion, synchronization, or the pose-graph
estimator loop.

**Architecture:** This is plan 2 of the 6-plan series in
`docs/superpowers/plans/2026-08-20-acoustic-optic-contract-foundation.md`
(plan 1, contracts/calibration, is done — 15/15 C++ tests, 11/11 Python
tests passing on `uw_l0_domain_contract_test`/`uw_l0_measurement_api_contract_test`
et al.). `StereoOpticalDepthFrontend` implements the `OpticalDepthFrontend`
interface added in plan 1 and emits the `OpticalDepthPriorMeasurement`
payload added in plan 1 — no wire schema changes are needed in this plan.
New code is additive: a new `core/sensor_models` camera model, a new
`algorithms/frontends/stereo_optical_depth_frontend` module (mirroring
`sonar_cfar_frontend`'s directory shape), a new `evaluation` depth-metrics
module, and two new standalone `apps/tools/` binaries. `synth_bag_gen` and
`replay_demo` are NOT modified — this keeps the already-battle-tested
pose-graph replay path and its determinism test untouched.

**Tech Stack:** C++17, Eigen (no OpenCV — this repo has no vendor image
dependency; the stereo matcher below is an original, non-ported
implementation, same precedent as `dbscan.hpp`), GoogleTest, MCAP.

**Scope boundary (do not implement in this plan):** capture-time
synchronization of independent `ImageFrame`s into `CameraFrameBundle` (that
is plan 3's `AcousticOpticSynchronizer`), FLS arc-band projection or any
acoustic-optic association/fusion, general off-axis stereo rectification
(image warping for arbitrary relative camera rotation — v1 requires the
rig's two cameras to already share identical orientation, which is true of
`configs/rig/example_auv.yaml`'s actual `camera_left_link`/
`camera_right_link` edges; a misaligned pair is rejected, not silently
warped), the 9-scenario matrix or ablation harness (plan 5), and any change
to `apps/replay_demo`'s pose-graph loop.

Repository policy: do not create git commits unless the user explicitly
authorizes them. Each task ends with a review checkpoint, not a commit.

## File map

### Create

- `core/sensor_models/include/uw/sensor_models/camera_model.hpp` — `PinholeCamera`, `StereoGeometry`.
- `core/sensor_models/src/camera_model.cpp`
- `core/sensor_models/test/camera_model_test.cpp`
- `algorithms/frontends/stereo_optical_depth_frontend/CMakeLists.txt`
- `algorithms/frontends/stereo_optical_depth_frontend/include/uw/frontends/block_matcher.hpp`
- `algorithms/frontends/stereo_optical_depth_frontend/include/uw/frontends/stereo_optical_depth_frontend.hpp`
- `algorithms/frontends/stereo_optical_depth_frontend/src/block_matcher.cpp`
- `algorithms/frontends/stereo_optical_depth_frontend/src/stereo_optical_depth_frontend.cpp`
- `algorithms/frontends/stereo_optical_depth_frontend/test/block_matcher_test.cpp`
- `algorithms/frontends/stereo_optical_depth_frontend/test/stereo_optical_depth_frontend_test.cpp`
- `evaluation/include/uw/evaluation/depth_metrics.hpp`
- `evaluation/src/depth_metrics.cpp`
- `evaluation/test/depth_metrics_test.cpp`
- `apps/tools/synth_stereo_gen/CMakeLists.txt`
- `apps/tools/synth_stereo_gen/src/main.cpp`
- `apps/tools/optical_baseline_eval/CMakeLists.txt`
- `apps/tools/optical_baseline_eval/src/main.cpp`
- `tests/l2_replay/optical_baseline_smoke_test.sh`

### Modify

- `CMakeLists.txt` — register the four new subdirectories.
- `core/sensor_models/CMakeLists.txt` — add `camera_model.cpp` + test target.
- `evaluation/CMakeLists.txt` — add `depth_metrics.cpp` + test target.
- `tests/CMakeLists.txt` — register the new L2 smoke test.
- `docs/uw-slam-codebase-reference-2026-08-18.md` — document the new modules; correct the now-stale "no `OpticalDepthFrontend` implementation exists" note from plan 1.
- `configs/README.md` — note that `stereo_depth_frontend_v1` now has a real implementation, still not wired into `replay_demo`.

---

### Task 1: Pinhole camera model and stereo geometry resolution

**Files:**
- Create: `core/sensor_models/include/uw/sensor_models/camera_model.hpp`
- Create: `core/sensor_models/src/camera_model.cpp`
- Create: `core/sensor_models/test/camera_model_test.cpp`
- Modify: `core/sensor_models/CMakeLists.txt`

- [ ] **Step 1: Write failing tests**

Create `core/sensor_models/test/camera_model_test.cpp`:

```cpp
#include <gtest/gtest.h>

#include "uw/sensor_models/camera_model.hpp"

TEST(CameraModel, ProjectUnprojectRoundTrip) {
  uw::domain::CameraIntrinsics intrinsics;
  intrinsics.set_width(640);
  intrinsics.set_height(480);
  for (double v : {420.0, 0.0, 320.0, 0.0, 420.0, 240.0, 0.0, 0.0, 1.0}) {
    intrinsics.add_k_matrix_row_major(v);
  }
  const auto camera = uw::sensor_models::PinholeCamera::FromIntrinsics(intrinsics);
  EXPECT_DOUBLE_EQ(camera.fx, 420.0);
  EXPECT_DOUBLE_EQ(camera.cx, 320.0);

  const Eigen::Vector3d point_camera(1.5, -0.5, 6.3);
  const Eigen::Vector2d pixel = camera.Project(point_camera);
  const Eigen::Vector3d recovered = camera.Unproject(pixel.x(), pixel.y(), point_camera.z());
  EXPECT_NEAR(recovered.x(), point_camera.x(), 1e-9);
  EXPECT_NEAR(recovered.y(), point_camera.y(), 1e-9);
  EXPECT_NEAR(recovered.z(), point_camera.z(), 1e-9);
}

TEST(StereoGeometryTest, ResolvesBaselineForParallelRig) {
  uw::domain::RigCalibrationSnapshot rig;
  auto add_camera = [&](const std::string& id) {
    auto* camera = rig.add_cameras();
    camera->mutable_sensor_id()->set_value(id);
    camera->set_width(640);
    camera->set_height(480);
    for (double v : {420.0, 0.0, 320.0, 0.0, 420.0, 240.0, 0.0, 0.0, 1.0}) {
      camera->add_k_matrix_row_major(v);
    }
  };
  add_camera("camera_left");
  add_camera("camera_right");

  auto add_edge = [&](const std::string& child, double y) {
    auto* edge = rig.add_frame_tree();
    edge->mutable_parent_frame()->set_value("base_link");
    edge->mutable_child_frame()->set_value(child);
    for (double v : {1.0, 0.0, 0.0, 0.15, 0.0, 1.0, 0.0, y, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0}) {
      edge->mutable_transform()->add_matrix_row_major(v);
    }
  };
  add_edge("camera_left_link", 0.06);
  add_edge("camera_right_link", -0.06);

  const auto geometry = uw::sensor_models::StereoGeometry::Resolve(
      rig, "camera_left", "camera_left_link", "camera_right", "camera_right_link");
  ASSERT_TRUE(geometry.valid);
  EXPECT_NEAR(geometry.baseline_m, 0.12, 1e-9);
  EXPECT_DOUBLE_EQ(geometry.left.fx, 420.0);
}

TEST(StereoGeometryTest, RejectsNonParallelOrientation) {
  uw::domain::RigCalibrationSnapshot rig;
  auto* left_cam = rig.add_cameras();
  left_cam->mutable_sensor_id()->set_value("camera_left");
  left_cam->set_width(640);
  left_cam->set_height(480);
  auto* right_cam = rig.add_cameras();
  right_cam->mutable_sensor_id()->set_value("camera_right");
  right_cam->set_width(640);
  right_cam->set_height(480);

  auto* left_edge = rig.add_frame_tree();
  left_edge->mutable_parent_frame()->set_value("base_link");
  left_edge->mutable_child_frame()->set_value("camera_left_link");
  for (double v : {1.0, 0.0, 0.0, 0.15, 0.0, 1.0, 0.0, 0.06, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0}) {
    left_edge->mutable_transform()->add_matrix_row_major(v);
  }
  // Right camera rotated 90 degrees about Z relative to left — not a valid
  // rectified pair for this v1's parallel-baseline assumption.
  auto* right_edge = rig.add_frame_tree();
  right_edge->mutable_parent_frame()->set_value("base_link");
  right_edge->mutable_child_frame()->set_value("camera_right_link");
  for (double v : {0.0, -1.0, 0.0, 0.15, 1.0, 0.0, 0.0, -0.06, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0}) {
    right_edge->mutable_transform()->add_matrix_row_major(v);
  }

  const auto geometry = uw::sensor_models::StereoGeometry::Resolve(
      rig, "camera_left", "camera_left_link", "camera_right", "camera_right_link");
  EXPECT_FALSE(geometry.valid);
}
```

- [ ] **Step 2: Verify the test fails to compile**

Run: `cmake --build build --target uw_sensor_models_camera_model_test -j"$(nproc)"` — expected to fail (target doesn't exist yet; confirm via the CMake reconfigure step immediately after Step 4 instead if the target itself is what's missing).

- [ ] **Step 3: Implement `camera_model.hpp`/`.cpp`**

```cpp
// core/sensor_models/include/uw/sensor_models/camera_model.hpp
#pragma once

#include <cstdint>
#include <string>

#include <Eigen/Core>

#include "uw/domain/domain.hpp"

namespace uw::sensor_models {

// Pinhole intrinsics only. v1 assumes already-undistorted pixel coordinates
// — CameraIntrinsics.distortion is not applied here. This mirrors
// StereoGeometry's parallel-rig assumption below: general
// undistort+rectify (image warping for arbitrary distortion/orientation)
// is out of scope until a later plan needs a non-parallel or distorted rig.
struct PinholeCamera {
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  uint32_t width = 0;
  uint32_t height = 0;

  static PinholeCamera FromIntrinsics(const uw::domain::CameraIntrinsics& intrinsics);

  // Projects a point already expressed in this camera's frame (z > 0 is
  // the caller's responsibility to check) to a pixel coordinate.
  Eigen::Vector2d Project(const Eigen::Vector3d& point_camera) const;
  Eigen::Vector3d Unproject(double u, double v, double depth_m) const;
};

// Validated geometry for a stereo pair. v1 REQUIRES the two cameras to
// share identical orientation and a purely-translational baseline (true of
// configs/rig/example_auv.yaml's camera_left_link/camera_right_link
// edges) — general off-axis rectification is explicitly out of scope;
// `valid` is false rather than silently producing wrong depth for a
// misaligned pair.
struct StereoGeometry {
  PinholeCamera left;
  PinholeCamera right;
  double baseline_m = 0.0;
  bool valid = false;

  static StereoGeometry Resolve(const uw::domain::RigCalibrationSnapshot& rig,
                                 const std::string& left_sensor_id,
                                 const std::string& left_frame,
                                 const std::string& right_sensor_id,
                                 const std::string& right_frame);
};

}  // namespace uw::sensor_models
```

```cpp
// core/sensor_models/src/camera_model.cpp
#include "uw/sensor_models/camera_model.hpp"

#include <optional>

namespace uw::sensor_models {

namespace {

const uw::domain::CameraIntrinsics* FindCamera(const uw::domain::RigCalibrationSnapshot& rig,
                                                const std::string& sensor_id) {
  for (const auto& camera : rig.cameras()) {
    if (camera.sensor_id().value() == sensor_id) return &camera;
  }
  return nullptr;
}

std::optional<Eigen::Matrix4d> FindEdgeTransform(const uw::domain::RigCalibrationSnapshot& rig,
                                                  const std::string& child_frame) {
  for (const auto& edge : rig.frame_tree()) {
    if (edge.child_frame().value() != child_frame) continue;
    if (edge.transform().matrix_row_major_size() != 16) return std::nullopt;
    Eigen::Matrix4d m;
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        m(row, col) = edge.transform().matrix_row_major(row * 4 + col);
      }
    }
    return m;
  }
  return std::nullopt;
}

}  // namespace

PinholeCamera PinholeCamera::FromIntrinsics(const uw::domain::CameraIntrinsics& intrinsics) {
  PinholeCamera camera;
  camera.width = intrinsics.width();
  camera.height = intrinsics.height();
  if (intrinsics.k_matrix_row_major_size() == 9) {
    camera.fx = intrinsics.k_matrix_row_major(0);
    camera.cx = intrinsics.k_matrix_row_major(2);
    camera.fy = intrinsics.k_matrix_row_major(4);
    camera.cy = intrinsics.k_matrix_row_major(5);
  }
  return camera;
}

Eigen::Vector2d PinholeCamera::Project(const Eigen::Vector3d& point_camera) const {
  return Eigen::Vector2d(fx * point_camera.x() / point_camera.z() + cx,
                         fy * point_camera.y() / point_camera.z() + cy);
}

Eigen::Vector3d PinholeCamera::Unproject(double u, double v, double depth_m) const {
  return Eigen::Vector3d((u - cx) / fx * depth_m, (v - cy) / fy * depth_m, depth_m);
}

StereoGeometry StereoGeometry::Resolve(const uw::domain::RigCalibrationSnapshot& rig,
                                       const std::string& left_sensor_id,
                                       const std::string& left_frame,
                                       const std::string& right_sensor_id,
                                       const std::string& right_frame) {
  StereoGeometry geometry;
  const auto* left_intrinsics = FindCamera(rig, left_sensor_id);
  const auto* right_intrinsics = FindCamera(rig, right_sensor_id);
  if (left_intrinsics == nullptr || right_intrinsics == nullptr) return geometry;

  const auto left_transform = FindEdgeTransform(rig, left_frame);
  const auto right_transform = FindEdgeTransform(rig, right_frame);
  if (!left_transform.has_value() || !right_transform.has_value()) return geometry;

  const Eigen::Matrix3d left_rotation = left_transform->topLeftCorner<3, 3>();
  const Eigen::Matrix3d right_rotation = right_transform->topLeftCorner<3, 3>();
  if (!left_rotation.isApprox(right_rotation, 1e-9)) return geometry;

  geometry.left = PinholeCamera::FromIntrinsics(*left_intrinsics);
  geometry.right = PinholeCamera::FromIntrinsics(*right_intrinsics);
  geometry.baseline_m =
      (left_transform->topRightCorner<3, 1>() - right_transform->topRightCorner<3, 1>()).norm();
  geometry.valid = geometry.baseline_m > 1e-9;
  return geometry;
}

}  // namespace uw::sensor_models
```

- [ ] **Step 4: Wire the CMake test target**

Add to `core/sensor_models/CMakeLists.txt`:

```cmake
add_library(uw_sensor_models STATIC
  src/geometry.cpp
  src/sonar_beam_model.cpp
  src/camera_model.cpp
)
target_include_directories(uw_sensor_models PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(uw_sensor_models PUBLIC uw_domain Eigen3::Eigen)
target_compile_options(uw_sensor_models PRIVATE -Wall -Wextra)

if(UW_BUILD_TESTS)
  add_executable(uw_sensor_models_camera_model_test test/camera_model_test.cpp)
  target_link_libraries(uw_sensor_models_camera_model_test PRIVATE uw_sensor_models GTest::gtest GTest::gtest_main)
  add_test(NAME uw_sensor_models_camera_model_test COMMAND uw_sensor_models_camera_model_test)
endif()
```

- [ ] **Step 5: Reconfigure, build, run**

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH="$HOME/miniconda3/envs/uw_slam_build"
cmake --build build --target uw_sensor_models_camera_model_test -j"$(nproc)"
ctest --test-dir build -R '^uw_sensor_models_camera_model_test$' --output-on-failure
```

Expected: build succeeds, all 3 assertions pass.

- [ ] **Step 6: Review checkpoint**

Confirm `camera_model.hpp`/`.cpp` add no ROS/OpenCV dependency (`tools/lint/check_no_ros_in_core.sh` still exits 0) and `StereoGeometry::Resolve` fails closed (returns `valid=false`) rather than throwing, matching `core/domain`'s validation-result style from plan 1. Do not commit without explicit authorization.

---

### Task 2: Deterministic block-matching disparity estimator

**Files:**
- Create: `algorithms/frontends/stereo_optical_depth_frontend/CMakeLists.txt`
- Create: `algorithms/frontends/stereo_optical_depth_frontend/include/uw/frontends/block_matcher.hpp`
- Create: `algorithms/frontends/stereo_optical_depth_frontend/src/block_matcher.cpp`
- Create: `algorithms/frontends/stereo_optical_depth_frontend/test/block_matcher_test.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Create the module skeleton and CMake target**

Create `algorithms/frontends/stereo_optical_depth_frontend/CMakeLists.txt`:

```cmake
add_library(uw_stereo_optical_depth_frontend STATIC
  src/block_matcher.cpp
  src/stereo_optical_depth_frontend.cpp
)
target_include_directories(uw_stereo_optical_depth_frontend PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(uw_stereo_optical_depth_frontend PUBLIC uw_measurement_api uw_sensor_models)
target_compile_options(uw_stereo_optical_depth_frontend PRIVATE -Wall -Wextra)

if(UW_BUILD_TESTS)
  add_executable(uw_stereo_optical_depth_frontend_test
    test/block_matcher_test.cpp
    test/stereo_optical_depth_frontend_test.cpp
  )
  target_link_libraries(uw_stereo_optical_depth_frontend_test PRIVATE uw_stereo_optical_depth_frontend GTest::gtest GTest::gtest_main)
  add_test(NAME uw_stereo_optical_depth_frontend_test COMMAND uw_stereo_optical_depth_frontend_test)
endif()
```

Add `add_subdirectory(algorithms/frontends/stereo_optical_depth_frontend)` to the top-level
`CMakeLists.txt`, right after the existing `add_subdirectory(algorithms/frontends/sonar_cfar_frontend)`.
Leave `src/stereo_optical_depth_frontend.cpp` as an empty translation unit
(`#include "uw/frontends/stereo_optical_depth_frontend.hpp"` only) for now — Task 3 fills it in.

- [ ] **Step 2: Write the failing block matcher test**

Create `algorithms/frontends/stereo_optical_depth_frontend/test/block_matcher_test.cpp`:

```cpp
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "uw/frontends/block_matcher.hpp"

namespace {

// Deterministic, non-repeating-enough texture for exact block matching —
// not meant to resemble a real image, only to give each block a unique
// intensity fingerprint so SAD has a clean global minimum at the true
// disparity.
uint8_t Texture(int u, int v) { return static_cast<uint8_t>((u * 31 + v * 17 + 7) % 256); }

}  // namespace

TEST(BlockMatcher, RecoversConstantDisparityOnCleanTexture) {
  constexpr int kWidth = 20;
  constexpr int kHeight = 3;
  constexpr int kTrueDisparity = 4;

  std::vector<uint8_t> left(kWidth * kHeight);
  std::vector<uint8_t> right(kWidth * kHeight);
  for (int v = 0; v < kHeight; ++v) {
    for (int u = 0; u < kWidth; ++u) {
      left[v * kWidth + u] = Texture(u, v);
      right[v * kWidth + u] = Texture(u + kTrueDisparity, v);
    }
  }

  uw::frontends::BlockMatcherParams params;
  params.window_radius = 1;
  params.min_disparity = 0;
  params.max_disparity = 6;
  uw::frontends::BlockMatcher matcher(params);

  const auto result = matcher.Compute(left.data(), right.data(), kWidth, kHeight, kWidth);
  ASSERT_EQ(result.width, static_cast<uint32_t>(kWidth));
  ASSERT_EQ(result.height, static_cast<uint32_t>(kHeight));

  // Valid region: u in [radius+max_disparity, width-1-radius], v in [radius, height-1-radius].
  const int v = 1;
  for (int u = 1 + 6; u <= kWidth - 1 - 1; ++u) {
    const std::size_t idx = static_cast<std::size_t>(v) * kWidth + u;
    ASSERT_EQ(result.valid[idx], 1) << "u=" << u;
    EXPECT_FLOAT_EQ(result.disparity_px[idx], static_cast<float>(kTrueDisparity)) << "u=" << u;
  }
  // Outside the valid region (borders, and the excluded top/bottom rows).
  EXPECT_EQ(result.valid[0 * kWidth + 0], 0);
  EXPECT_EQ(result.valid[2 * kWidth + 10], 0);
}
```

- [ ] **Step 3: Verify it fails to compile, then implement `block_matcher.hpp`/`.cpp`**

Run `cmake -S . -B build && cmake --build build --target uw_stereo_optical_depth_frontend_test -j"$(nproc)"`
and confirm it fails (missing `BlockMatcher`/`BlockMatcherParams`/`DisparityResult`).

```cpp
// algorithms/frontends/stereo_optical_depth_frontend/include/uw/frontends/block_matcher.hpp
//
// Original implementation, not ported from any third party (same
// precedent as sonar_cfar_frontend's dbscan.hpp — see NOTICE) — this repo
// has no OpenCV/vendor image dependency and this MVP does not need one.
#pragma once

#include <cstdint>
#include <vector>

namespace uw::frontends {

struct BlockMatcherParams {
  int window_radius = 3;         // block is (2r+1)x(2r+1)
  int min_disparity = 1;         // >=1: disparity 0 implies infinite depth, excluded by construction
  int max_disparity = 32;
  double max_mean_abs_diff = 40.0;  // reject a match if best SAD / window_pixel_count exceeds this
};

struct DisparityResult {
  uint32_t width = 0;
  uint32_t height = 0;
  std::vector<float> disparity_px;  // row-major width*height; meaningless where valid[i] == 0
  std::vector<uint8_t> valid;       // 1 = confident match found inside the search range
};

// Fixed-window sum-of-absolute-differences stereo matcher over rectified
// MONO8 pairs. Deterministic, single-threaded, fixed iteration order (no
// unordered iteration — platform architecture section on reproducibility).
// Sign convention: for a pixel (u, v) in `left`, the matcher searches
// `right` at (u - d, v) for d in [min_disparity, max_disparity] — i.e. it
// assumes right(u, v) == left(u + d_true, v) for the true disparity
// d_true at that point (see stereo_optical_depth_frontend_test.cpp and
// apps/tools/synth_stereo_gen for the synthetic data generated under this
// exact convention).
class BlockMatcher {
 public:
  explicit BlockMatcher(BlockMatcherParams params);

  DisparityResult Compute(const uint8_t* left, const uint8_t* right, uint32_t width,
                          uint32_t height, uint32_t stride_px) const;

 private:
  BlockMatcherParams params_;
};

}  // namespace uw::frontends
```

```cpp
// algorithms/frontends/stereo_optical_depth_frontend/src/block_matcher.cpp
#include "uw/frontends/block_matcher.hpp"

#include <cstdlib>
#include <limits>

namespace uw::frontends {

BlockMatcher::BlockMatcher(BlockMatcherParams params) : params_(params) {}

DisparityResult BlockMatcher::Compute(const uint8_t* left, const uint8_t* right, uint32_t width,
                                      uint32_t height, uint32_t stride_px) const {
  DisparityResult result;
  result.width = width;
  result.height = height;
  result.disparity_px.assign(static_cast<std::size_t>(width) * height, 0.0f);
  result.valid.assign(static_cast<std::size_t>(width) * height, 0);

  const int r = params_.window_radius;
  const int window_pixels = (2 * r + 1) * (2 * r + 1);
  const int u_min = r + params_.max_disparity;
  const int u_max = static_cast<int>(width) - 1 - r;
  const int v_min = r;
  const int v_max = static_cast<int>(height) - 1 - r;

  auto at = [&](const uint8_t* image, int u, int v) -> int {
    return image[static_cast<std::size_t>(v) * stride_px + static_cast<std::size_t>(u)];
  };

  for (int v = v_min; v <= v_max; ++v) {
    for (int u = u_min; u <= u_max; ++u) {
      int best_disparity = -1;
      long best_sad = std::numeric_limits<long>::max();
      for (int d = params_.min_disparity; d <= params_.max_disparity; ++d) {
        long sad = 0;
        for (int dy = -r; dy <= r; ++dy) {
          for (int dx = -r; dx <= r; ++dx) {
            sad += std::abs(at(left, u + dx, v + dy) - at(right, u + dx - d, v + dy));
          }
        }
        if (sad < best_sad) {
          best_sad = sad;
          best_disparity = d;
        }
      }
      const double mean_abs_diff = static_cast<double>(best_sad) / window_pixels;
      const std::size_t idx = static_cast<std::size_t>(v) * width + static_cast<std::size_t>(u);
      if (best_disparity >= 0 && mean_abs_diff <= params_.max_mean_abs_diff) {
        result.disparity_px[idx] = static_cast<float>(best_disparity);
        result.valid[idx] = 1;
      }
    }
  }
  return result;
}

}  // namespace uw::frontends
```

- [ ] **Step 4: Build and run**

```bash
cmake --build build --target uw_stereo_optical_depth_frontend_test -j"$(nproc)"
ctest --test-dir build -R '^uw_stereo_optical_depth_frontend_test$' --output-on-failure
```

Expected: build succeeds; `BlockMatcher.RecoversConstantDisparityOnCleanTexture` passes
(the `stereo_optical_depth_frontend_test.cpp` test file does not exist yet — create it as an
empty placeholder `#include <gtest/gtest.h>` file for this step only if CMake requires the
listed source to exist; Task 3 replaces it with real tests).

- [ ] **Step 5: Review checkpoint**

Confirm the matcher's iteration order is fully deterministic (nested `for` loops, no
`std::unordered_map`/`std::thread`) and that `min_disparity` defaults to `1`, never `0`
(division by a zero disparity is undefined in Task 3's depth conversion). Do not commit
without explicit authorization.

---

### Task 3: `StereoOpticalDepthFrontend` — wire block matcher + geometry into evidence

**Files:**
- Modify: `algorithms/frontends/stereo_optical_depth_frontend/include/uw/frontends/stereo_optical_depth_frontend.hpp`
- Modify: `algorithms/frontends/stereo_optical_depth_frontend/src/stereo_optical_depth_frontend.cpp`
- Modify: `algorithms/frontends/stereo_optical_depth_frontend/test/stereo_optical_depth_frontend_test.cpp`

- [ ] **Step 1: Write the failing integration tests**

Create/replace `algorithms/frontends/stereo_optical_depth_frontend/test/stereo_optical_depth_frontend_test.cpp`:

```cpp
#include <cmath>
#include <cstdint>
#include <string>

#include <gtest/gtest.h>

#include "uw/frontends/stereo_optical_depth_frontend.hpp"

namespace {

uint8_t Texture(int u, int v) { return static_cast<uint8_t>((u * 31 + v * 17 + 7) % 256); }

uw::domain::RigCalibrationSnapshot MakeRig(double fx, double baseline_half) {
  uw::domain::RigCalibrationSnapshot rig;
  auto add_camera = [&](const std::string& id, uint32_t w, uint32_t h) {
    auto* camera = rig.add_cameras();
    camera->mutable_sensor_id()->set_value(id);
    camera->set_width(w);
    camera->set_height(h);
    for (double v : {fx, 0.0, 0.0, 0.0, fx, 0.0, 0.0, 0.0, 1.0}) camera->add_k_matrix_row_major(v);
  };
  add_camera("camera_left", 20, 3);
  add_camera("camera_right", 20, 3);
  auto add_edge = [&](const std::string& child, double y) {
    auto* edge = rig.add_frame_tree();
    edge->mutable_parent_frame()->set_value("base_link");
    edge->mutable_child_frame()->set_value(child);
    for (double v : {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, y, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0}) {
      edge->mutable_transform()->add_matrix_row_major(v);
    }
  };
  add_edge("camera_left_link", baseline_half);
  add_edge("camera_right_link", -baseline_half);
  return rig;
}

uw::domain::ImageFrame MakeImage(const std::string& frame, int width, int height, int shift) {
  uw::domain::ImageFrame image;
  image.mutable_header()->mutable_sensor_frame()->set_value(frame);
  image.set_width(width);
  image.set_height(height);
  image.set_row_stride_bytes(width);
  image.set_encoding(uw::domain::ImageFrame::IMAGE_ENCODING_MONO8);
  std::string pixels(static_cast<std::size_t>(width) * height, '\0');
  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      pixels[static_cast<std::size_t>(v) * width + u] = static_cast<char>(Texture(u + shift, v));
    }
  }
  image.set_pixel_data(pixels);
  return image;
}

}  // namespace

TEST(StereoOpticalDepthFrontend, RecoversMetricDepthFromConstantDisparity) {
  // fx=100, baseline=0.5m, true disparity=4px -> depth = fx*baseline/d = 12.5m.
  const auto rig = MakeRig(/*fx=*/100.0, /*baseline_half=*/0.25);
  uw::measurement_api::CameraFrameBundle bundle;
  bundle.primary = MakeImage("camera_left_link", 20, 3, /*shift=*/0);
  bundle.secondary = MakeImage("camera_right_link", 20, 3, /*shift=*/4);

  uw::frontends::StereoOpticalDepthFrontendParams params;
  params.left_sensor_id = "camera_left";
  params.left_frame = "camera_left_link";
  params.right_sensor_id = "camera_right";
  params.right_frame = "camera_right_link";
  params.matcher.window_radius = 1;
  params.matcher.min_disparity = 1;
  params.matcher.max_disparity = 6;
  uw::frontends::StereoOpticalDepthFrontend frontend(params);

  const auto evidence = frontend.Process(bundle, rig);
  ASSERT_TRUE(evidence.has_value());
  const auto& prior = uw::domain::GetPayload<uw::domain::OpticalDepthPriorMeasurement>(*evidence);
  EXPECT_EQ(prior.scale_status(), uw::domain::OPTICAL_DEPTH_SCALE_STATUS_METRIC);
  EXPECT_EQ(prior.producer_type(), "stereo");
  ASSERT_EQ(uw::domain::ValidateOpticalDepthPrior(prior).code, uw::domain::ValidationCode::kOk);

  const int u = 10, v = 1;  // inside the matcher's valid region for this fixture
  const std::size_t idx = static_cast<std::size_t>(v) * 20 + u;
  ASSERT_EQ(static_cast<unsigned char>(prior.valid_mask()[idx]), 1);
  EXPECT_NEAR(prior.depth_m(idx), 12.5, 1e-6);
  const double expected_variance = std::pow(12.5 * 12.5 / (100.0 * 0.5) * params.disparity_sigma_px, 2);
  EXPECT_NEAR(prior.variance_m2(idx), expected_variance, 1e-6);
}

TEST(StereoOpticalDepthFrontend, RejectsBundleWithoutSecondaryFrame) {
  const auto rig = MakeRig(100.0, 0.25);
  uw::measurement_api::CameraFrameBundle bundle;
  bundle.primary = MakeImage("camera_left_link", 20, 3, 0);

  uw::frontends::StereoOpticalDepthFrontendParams params;
  params.left_sensor_id = "camera_left";
  params.left_frame = "camera_left_link";
  params.right_sensor_id = "camera_right";
  params.right_frame = "camera_right_link";
  uw::frontends::StereoOpticalDepthFrontend frontend(params);

  EXPECT_FALSE(frontend.Process(bundle, rig).has_value());
}

TEST(StereoOpticalDepthFrontend, RejectsUnresolvableRigGeometry) {
  uw::domain::RigCalibrationSnapshot empty_rig;
  uw::measurement_api::CameraFrameBundle bundle;
  bundle.primary = MakeImage("camera_left_link", 20, 3, 0);
  bundle.secondary = MakeImage("camera_right_link", 20, 3, 4);

  uw::frontends::StereoOpticalDepthFrontendParams params;
  params.left_sensor_id = "camera_left";
  params.left_frame = "camera_left_link";
  params.right_sensor_id = "camera_right";
  params.right_frame = "camera_right_link";
  uw::frontends::StereoOpticalDepthFrontend frontend(params);

  EXPECT_FALSE(frontend.Process(bundle, empty_rig).has_value());
}
```

- [ ] **Step 2: Verify it fails to compile**

```bash
cmake --build build --target uw_stereo_optical_depth_frontend_test -j"$(nproc)"
```

Expected: fails — `StereoOpticalDepthFrontend`/`StereoOpticalDepthFrontendParams` undefined.

- [ ] **Step 3: Implement the frontend**

```cpp
// algorithms/frontends/stereo_optical_depth_frontend/include/uw/frontends/stereo_optical_depth_frontend.hpp
//
// MVP stereo implementation of uw::measurement_api::OpticalDepthFrontend
// (core/measurement_api/include/uw/measurement_api/frontend.hpp, added in
// the contract-foundation plan). Consumes only geometry — no learned
// disparity, no semantic gating (architecture invariant: frontends own
// measurement evidence, not policy).
#pragma once

#include <cstdint>
#include <string>

#include "uw/frontends/block_matcher.hpp"
#include "uw/measurement_api/frontend.hpp"
#include "uw/sensor_models/camera_model.hpp"

namespace uw::frontends {

struct StereoOpticalDepthFrontendParams {
  std::string left_sensor_id = "camera_left";
  std::string left_frame = "camera_left_link";
  std::string right_sensor_id = "camera_right";
  std::string right_frame = "camera_right_link";
  BlockMatcherParams matcher;
  double disparity_sigma_px = 0.5;  // assumed fixed matcher uncertainty, propagated to variance_m2
};

class StereoOpticalDepthFrontend : public uw::measurement_api::OpticalDepthFrontend {
 public:
  explicit StereoOpticalDepthFrontend(StereoOpticalDepthFrontendParams params);

  std::optional<uw::domain::MeasurementEvidence> Process(
      const uw::measurement_api::CameraFrameBundle& bundle,
      const uw::domain::RigCalibrationSnapshot& rig) override;
  uw::domain::HealthReport Health() const override;

 private:
  StereoOpticalDepthFrontendParams params_;
  BlockMatcher matcher_;
  uint64_t frames_processed_ = 0;
  uint64_t frames_rejected_ = 0;
  uint64_t next_evidence_id_ = 1;
};

}  // namespace uw::frontends
```

```cpp
// algorithms/frontends/stereo_optical_depth_frontend/src/stereo_optical_depth_frontend.cpp
#include "uw/frontends/stereo_optical_depth_frontend.hpp"

#include <cmath>

namespace uw::frontends {

StereoOpticalDepthFrontend::StereoOpticalDepthFrontend(StereoOpticalDepthFrontendParams params)
    : params_(params), matcher_(params.matcher) {}

std::optional<uw::domain::MeasurementEvidence> StereoOpticalDepthFrontend::Process(
    const uw::measurement_api::CameraFrameBundle& bundle,
    const uw::domain::RigCalibrationSnapshot& rig) {
  ++frames_processed_;
  if (!bundle.secondary.has_value()) {
    ++frames_rejected_;
    return std::nullopt;
  }

  const auto geometry = uw::sensor_models::StereoGeometry::Resolve(
      rig, params_.left_sensor_id, params_.left_frame, params_.right_sensor_id, params_.right_frame);
  if (!geometry.valid) {
    ++frames_rejected_;
    return std::nullopt;
  }

  const auto& left_image = bundle.primary;
  const auto& right_image = *bundle.secondary;
  if (left_image.encoding() != uw::domain::ImageFrame::IMAGE_ENCODING_MONO8 ||
      right_image.encoding() != uw::domain::ImageFrame::IMAGE_ENCODING_MONO8 ||
      left_image.width() != right_image.width() || left_image.height() != right_image.height()) {
    ++frames_rejected_;
    return std::nullopt;
  }

  const auto disparity = matcher_.Compute(
      reinterpret_cast<const uint8_t*>(left_image.pixel_data().data()),
      reinterpret_cast<const uint8_t*>(right_image.pixel_data().data()), left_image.width(),
      left_image.height(), left_image.row_stride_bytes());

  uw::domain::OpticalDepthPriorMeasurement prior;
  *prior.mutable_reference_camera_frame() = left_image.header().sensor_frame();
  prior.set_width(left_image.width());
  prior.set_height(left_image.height());
  prior.set_scale_status(uw::domain::OPTICAL_DEPTH_SCALE_STATUS_METRIC);
  prior.set_producer_type("stereo");

  const std::size_t pixels = static_cast<std::size_t>(left_image.width()) * left_image.height();
  std::string valid_mask(pixels, '\0');
  for (std::size_t i = 0; i < pixels; ++i) {
    if (disparity.valid[i] == 0) {
      prior.add_depth_m(0.0f);
      prior.add_variance_m2(0.0f);
      continue;
    }
    const double depth_m = geometry.left.fx * geometry.baseline_m / disparity.disparity_px[i];
    const double variance_m2 =
        std::pow(depth_m * depth_m / (geometry.left.fx * geometry.baseline_m) * params_.disparity_sigma_px, 2);
    prior.add_depth_m(static_cast<float>(depth_m));
    prior.add_variance_m2(static_cast<float>(variance_m2));
    valid_mask[i] = 1;
  }
  prior.set_valid_mask(valid_mask);

  uw::domain::EvidenceId evidence_id;
  evidence_id.set_value("stereo_depth_" + std::to_string(next_evidence_id_++));
  std::vector<uw::domain::ObservationId> sources;
  if (left_image.header().has_observation_id()) sources.push_back(left_image.header().observation_id());
  if (right_image.header().has_observation_id()) sources.push_back(right_image.header().observation_id());

  return uw::domain::MakeEvidence(evidence_id, sources, prior, /*noise_scale=*/1.0,
                                  "stereo_depth_frontend_v1");
}

uw::domain::HealthReport StereoOpticalDepthFrontend::Health() const {
  uw::domain::HealthReport report;
  report.set_component_id("stereo_optical_depth_frontend");
  report.set_status(frames_processed_ > 0 && frames_rejected_ == frames_processed_
                        ? uw::domain::HealthReport::STATUS_SUSPECT
                        : uw::domain::HealthReport::STATUS_HEALTHY);
  return report;
}

}  // namespace uw::frontends
```

- [ ] **Step 4: Build and run**

```bash
cmake --build build --target uw_stereo_optical_depth_frontend_test -j"$(nproc)"
ctest --test-dir build -R '^uw_stereo_optical_depth_frontend_test$' --output-on-failure
```

Expected: all tests in the target pass (block matcher + all 3 frontend tests).

- [ ] **Step 5: Review checkpoint**

Diff against plan 1's `OpticalDepthFrontend`/`CameraFrameBundle` interface and confirm no
signature drift. Confirm the invalid-pixel branch (`disparity.valid[i] == 0`) writes
`depth_m=0`/`variance_m2=0` (matching plan 1's `ValidateOpticalDepthPrior`'s rule that
invalid-pixel values carry no semantics) rather than leaving the repeated fields undersized.
Do not commit without explicit authorization.

---

### Task 4: Depth evaluation metrics

**Files:**
- Create: `evaluation/include/uw/evaluation/depth_metrics.hpp`
- Create: `evaluation/src/depth_metrics.cpp`
- Create: `evaluation/test/depth_metrics_test.cpp`
- Modify: `evaluation/CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

Create `evaluation/test/depth_metrics_test.cpp`:

```cpp
#include <gtest/gtest.h>

#include "uw/evaluation/depth_metrics.hpp"

TEST(DepthMetrics, ComputesRmseMaeAndCoverageOverSharedValidPixels) {
  uw::evaluation::DepthGrid estimated;
  estimated.width = 2;
  estimated.height = 1;
  estimated.depth_m = {5.0f, 6.0f};
  estimated.valid_mask = {1, 1};

  uw::evaluation::DepthGrid ground_truth;
  ground_truth.width = 2;
  ground_truth.height = 1;
  ground_truth.depth_m = {5.5f, 6.0f};
  ground_truth.valid_mask = {1, 0};  // second pixel has no GT -> excluded

  const auto result = uw::evaluation::ComputeDepthMetrics(estimated, ground_truth);
  EXPECT_EQ(result.num_compared_pixels, 1u);
  EXPECT_NEAR(result.mae_m, 0.5, 1e-9);
  EXPECT_NEAR(result.rmse_m, 0.5, 1e-9);
  EXPECT_NEAR(result.valid_coverage_fraction, 1.0, 1e-9);  // 1 of 1 GT-valid pixel also estimated-valid
}

TEST(DepthMetrics, ZeroGtValidPixelsYieldsZeroCoverageNotNan) {
  uw::evaluation::DepthGrid estimated;
  estimated.width = 1;
  estimated.height = 1;
  estimated.depth_m = {1.0f};
  estimated.valid_mask = {0};

  uw::evaluation::DepthGrid ground_truth;
  ground_truth.width = 1;
  ground_truth.height = 1;
  ground_truth.depth_m = {1.0f};
  ground_truth.valid_mask = {0};

  const auto result = uw::evaluation::ComputeDepthMetrics(estimated, ground_truth);
  EXPECT_EQ(result.num_compared_pixels, 0u);
  EXPECT_DOUBLE_EQ(result.valid_coverage_fraction, 0.0);
  EXPECT_DOUBLE_EQ(result.rmse_m, 0.0);
}
```

- [ ] **Step 2: Verify it fails to compile**

```bash
cmake --build build --target uw_evaluation_test -j"$(nproc)"
```

Expected: fails — `uw::evaluation::DepthGrid`/`ComputeDepthMetrics` undefined (this
target doesn't yet include `depth_metrics_test.cpp`; add it in Step 4 first, then this
compile-failure check applies).

- [ ] **Step 3: Implement**

```cpp
// evaluation/include/uw/evaluation/depth_metrics.hpp
#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>

namespace uw::evaluation {

struct DepthGrid {
  uint32_t width = 0;
  uint32_t height = 0;
  std::vector<float> depth_m;      // row-major width*height
  std::vector<uint8_t> valid_mask; // row-major width*height, 1 = valid
};

struct DepthMetricsResult {
  double rmse_m = 0.0;
  double mae_m = 0.0;
  // Fraction of ground-truth-valid pixels that `estimated` also marks
  // valid. NOT the design spec's sonar-covered/degraded-region split
  // (section 12.1) — that needs scenario masks and sonar association,
  // both introduced in later plans.
  double valid_coverage_fraction = 0.0;
  std::size_t num_compared_pixels = 0;
};

// Compares only pixels valid in BOTH grids. v1 limitation, documented
// rather than hidden (same pattern as ComputeAte's v1 note): no
// full-image/sonar-covered/visually-degraded split yet.
DepthMetricsResult ComputeDepthMetrics(const DepthGrid& estimated, const DepthGrid& ground_truth);

}  // namespace uw::evaluation
```

```cpp
// evaluation/src/depth_metrics.cpp
#include "uw/evaluation/depth_metrics.hpp"

#include <cmath>

namespace uw::evaluation {

DepthMetricsResult ComputeDepthMetrics(const DepthGrid& estimated, const DepthGrid& ground_truth) {
  DepthMetricsResult result;
  const std::size_t pixels = static_cast<std::size_t>(ground_truth.width) * ground_truth.height;

  std::size_t gt_valid_count = 0;
  std::size_t both_valid_count = 0;
  double sum_abs_error = 0.0;
  double sum_sq_error = 0.0;

  for (std::size_t i = 0; i < pixels && i < ground_truth.valid_mask.size(); ++i) {
    if (ground_truth.valid_mask[i] == 0) continue;
    ++gt_valid_count;
    if (i >= estimated.valid_mask.size() || estimated.valid_mask[i] == 0) continue;
    ++both_valid_count;
    const double error = static_cast<double>(estimated.depth_m[i]) - static_cast<double>(ground_truth.depth_m[i]);
    sum_abs_error += std::abs(error);
    sum_sq_error += error * error;
  }

  result.num_compared_pixels = both_valid_count;
  if (both_valid_count > 0) {
    result.mae_m = sum_abs_error / static_cast<double>(both_valid_count);
    result.rmse_m = std::sqrt(sum_sq_error / static_cast<double>(both_valid_count));
  }
  result.valid_coverage_fraction =
      gt_valid_count > 0 ? static_cast<double>(both_valid_count) / static_cast<double>(gt_valid_count) : 0.0;
  return result;
}

}  // namespace uw::evaluation
```

- [ ] **Step 4: Wire into CMake**

Modify `evaluation/CMakeLists.txt`:

```cmake
add_library(uw_evaluation STATIC src/trajectory_metrics.cpp src/depth_metrics.cpp)
target_include_directories(uw_evaluation PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(uw_evaluation PUBLIC uw_sensor_models)
target_compile_options(uw_evaluation PRIVATE -Wall -Wextra)

if(UW_BUILD_TESTS)
  add_executable(uw_evaluation_test test/trajectory_metrics_test.cpp test/depth_metrics_test.cpp)
  target_link_libraries(uw_evaluation_test PRIVATE uw_evaluation GTest::gtest GTest::gtest_main)
  add_test(NAME uw_evaluation_test COMMAND uw_evaluation_test)
endif()
```

- [ ] **Step 5: Build and run**

```bash
cmake --build build --target uw_evaluation_test -j"$(nproc)"
ctest --test-dir build -R '^uw_evaluation_test$' --output-on-failure
```

Expected: both new tests pass alongside the existing trajectory metrics tests.

- [ ] **Step 6: Review checkpoint**

Confirm `ComputeDepthMetrics` never divides by zero (the `gt_valid_count > 0` guard) and
that coverage is defined relative to GT-valid pixels, not total image pixels. Do not commit
without explicit authorization.

---

### Task 5: Synthetic stereo + GT depth generator (`synth_stereo_gen`)

**Files:**
- Create: `apps/tools/synth_stereo_gen/CMakeLists.txt`
- Create: `apps/tools/synth_stereo_gen/src/main.cpp`
- Modify: `CMakeLists.txt`

This app does NOT touch `apps/tools/synth_bag_gen` — it is a separate, additive binary so the
existing determinism-tested pose-graph data path is not put at risk.

- [ ] **Step 1: Implement the generator**

Design: a single static fronto-parallel textured plane at a known constant depth, sized to
the real example rig (`fx=420`, `baseline=0.12m` from `configs/rig/example_auv.yaml`'s
`camera_left_link`/`camera_right_link` — see plan 1). Choose true disparity `d=8px` exactly
so `depth = fx * baseline / d = 420 * 0.12 / 8 = 6.3m` is exact — this lets
`optical_baseline_eval` (Task 6) assert a small, non-circular RMSE bound rather than an
exact-zero bound (texture-collision false matches are possible at full image scale, even
though the small fixture in Task 3 is collision-free by direct construction).

Create `apps/tools/synth_stereo_gen/src/main.cpp`:

```cpp
// Generates a single synthetic stereo frame pair + a ground-truth depth
// grid for optical-baseline evaluation (plan 2 of the acoustic-optic
// series). Deliberately independent of apps/tools/synth_bag_gen: this is a
// single static-scene generator for the optical baseline, not a
// trajectory/pose-graph bag — see that tool's own header for why THIS repo
// needs synthetic generators at all (no HoloOcean/ROS2 on this dev
// machine).
//
// Topics written:
//   /raw/camera/left    uw.domain.ImageFrame
//   /raw/camera/right   uw.domain.ImageFrame
//   /gt/depth            uw.domain.MeasurementEvidence (OpticalDepthPriorMeasurement,
//                         producer_type="ground_truth" — reuses plan 1's metric depth-grid
//                         contract instead of inventing a parallel GT schema; ground truth is
//                         exactly a perfect metric depth prior)
#include <cstdint>
#include <iostream>
#include <string>

#include "uw/domain/domain.hpp"
#include "uw/runtime/config.hpp"
#include "uw/runtime/mcap_io.hpp"

namespace {

uint8_t Texture(int u, int v) { return static_cast<uint8_t>((u * 131 + v * 67 + 19) % 256); }

uw::domain::ImageFrame MakeImage(const std::string& frame, uint32_t width, uint32_t height, int shift) {
  uw::domain::ImageFrame image;
  image.mutable_header()->mutable_sensor_frame()->set_value(frame);
  image.mutable_header()->set_clock_domain(uw::domain::CLOCK_DOMAIN_SIMULATION);
  image.mutable_header()->set_validity(uw::domain::ObservationHeader::VALIDITY_OK);
  image.mutable_header()->set_provenance("synth_stereo_gen_v1");
  image.set_width(width);
  image.set_height(height);
  image.set_row_stride_bytes(width);
  image.set_encoding(uw::domain::ImageFrame::IMAGE_ENCODING_MONO8);
  std::string pixels(static_cast<std::size_t>(width) * height, '\0');
  for (uint32_t v = 0; v < height; ++v) {
    for (uint32_t u = 0; u < width; ++u) {
      pixels[static_cast<std::size_t>(v) * width + u] =
          static_cast<char>(Texture(static_cast<int>(u) + shift, static_cast<int>(v)));
    }
  }
  image.set_pixel_data(pixels);
  image.set_is_rectified(true);
  return image;
}

}  // namespace

int main(int argc, char** argv) {
  std::string out_path = "/tmp/synthetic_stereo.mcap";
  uint32_t width = 640;
  uint32_t height = 480;
  int true_disparity_px = 8;
  double depth_m = 6.3;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() { return std::string(argv[++i]); };
    if (arg == "--out" && i + 1 < argc) {
      out_path = next();
    } else if (arg == "--width" && i + 1 < argc) {
      width = static_cast<uint32_t>(std::stoul(next()));
    } else if (arg == "--height" && i + 1 < argc) {
      height = static_cast<uint32_t>(std::stoul(next()));
    } else if (arg == "--disparity-px" && i + 1 < argc) {
      true_disparity_px = std::stoi(next());
    } else if (arg == "--depth-m" && i + 1 < argc) {
      depth_m = std::stod(next());
    } else {
      std::cerr << "unknown argument: " << arg << "\n";
      return 1;
    }
  }

  uw::runtime::McapProtobufWriter writer;
  if (!writer.Open(out_path)) {
    std::cerr << "failed to open " << out_path << " for writing\n";
    return 1;
  }

  const auto left = MakeImage("camera_left_link", width, height, 0);
  const auto right = MakeImage("camera_right_link", width, height, true_disparity_px);
  writer.WriteMessage("/raw/camera/left", 0, left);
  writer.WriteMessage("/raw/camera/right", 0, right);

  uw::domain::OpticalDepthPriorMeasurement gt;
  *gt.mutable_reference_camera_frame() = left.header().sensor_frame();
  gt.set_width(width);
  gt.set_height(height);
  gt.set_scale_status(uw::domain::OPTICAL_DEPTH_SCALE_STATUS_METRIC);
  gt.set_producer_type("ground_truth");
  std::string valid_mask(static_cast<std::size_t>(width) * height, 1);
  for (uint32_t i = 0; i < width * height; ++i) {
    gt.add_depth_m(static_cast<float>(depth_m));
    gt.add_variance_m2(1e-6f);
  }
  gt.set_valid_mask(valid_mask);

  uw::domain::EvidenceId gt_id;
  gt_id.set_value("gt_depth_0");
  auto gt_evidence = uw::domain::MakeEvidence(gt_id, {}, gt, 0.0, "synth_stereo_gen_v1");
  writer.WriteMessage("/gt/depth", 0, gt_evidence);

  writer.Close();
  std::cout << "wrote " << width << "x" << height << " stereo pair (disparity=" << true_disparity_px
            << "px, depth=" << depth_m << "m) to " << out_path << "\n";
  return 0;
}
```

Create `apps/tools/synth_stereo_gen/CMakeLists.txt`:

```cmake
add_executable(synth_stereo_gen src/main.cpp)
target_link_libraries(synth_stereo_gen PRIVATE uw_domain uw_runtime)
target_compile_options(synth_stereo_gen PRIVATE -Wall -Wextra)
```

Add `add_subdirectory(apps/tools/synth_stereo_gen)` to the top-level `CMakeLists.txt`, next to
the existing `add_subdirectory(apps/tools/synth_bag_gen)`.

- [ ] **Step 2: Build and smoke-run**

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH="$HOME/miniconda3/envs/uw_slam_build"
cmake --build build --target synth_stereo_gen -j"$(nproc)"
./build/apps/tools/synth_stereo_gen/synth_stereo_gen --out /tmp/synthetic_stereo.mcap
```

Expected: exits 0, prints the wrote-file message, `/tmp/synthetic_stereo.mcap` exists and is non-empty.

- [ ] **Step 3: Review checkpoint**

Confirm this task did not modify `apps/tools/synth_bag_gen/` or its determinism test. Do not
commit without explicit authorization.

---

### Task 6: `optical_baseline_eval` + L2 smoke test

**Files:**
- Create: `apps/tools/optical_baseline_eval/CMakeLists.txt`
- Create: `apps/tools/optical_baseline_eval/src/main.cpp`
- Create: `tests/l2_replay/optical_baseline_smoke_test.sh`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

- [ ] **Step 1: Implement the evaluation app**

Reads a bag written by `synth_stereo_gen`, builds a `RigCalibrationSnapshot` from
`--experiment` (reusing plan 1's config loader and the real
`configs/rig/example_auv.yaml`), runs `StereoOpticalDepthFrontend`, and reports depth
metrics against `/gt/depth` via `uw::evaluation::ComputeDepthMetrics`. Exits non-zero if
`--max-rmse-m`/`--min-coverage` thresholds are violated, so this doubles as an L2 CI gate
without needing a separate assertion script.

Create `apps/tools/optical_baseline_eval/src/main.cpp`:

```cpp
// Optical-only baseline harness (plan 2 of the acoustic-optic series):
// StereoOpticalDepthFrontend against a synthetic bag from
// apps/tools/synth_stereo_gen, scored with uw::evaluation::ComputeDepthMetrics
// against /gt/depth. Does NOT fuse sonar and does NOT touch
// apps/replay_demo's pose-graph loop — see plan 2's scope boundary.
#include <iostream>
#include <optional>
#include <string>

#include "uw/domain/domain.hpp"
#include "uw/evaluation/depth_metrics.hpp"
#include "uw/frontends/stereo_optical_depth_frontend.hpp"
#include "uw/runtime/config.hpp"
#include "uw/runtime/mcap_io.hpp"

int main(int argc, char** argv) {
  std::string bag_path;
  std::string experiment_path = "configs/experiment/synthetic_smoke.yaml";
  double max_rmse_m = 0.05;
  double min_coverage = 0.9;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() { return std::string(argv[++i]); };
    if (arg == "--bag" && i + 1 < argc) {
      bag_path = next();
    } else if (arg == "--experiment" && i + 1 < argc) {
      experiment_path = next();
    } else if (arg == "--max-rmse-m" && i + 1 < argc) {
      max_rmse_m = std::stod(next());
    } else if (arg == "--min-coverage" && i + 1 < argc) {
      min_coverage = std::stod(next());
    } else {
      std::cerr << "unknown argument: " << arg << "\n";
      return 1;
    }
  }
  if (bag_path.empty()) {
    std::cerr << "--bag is required\n";
    return 1;
  }

  const auto experiment = uw::runtime::LoadExperimentConfig(experiment_path);

  std::optional<uw::domain::ImageFrame> left;
  std::optional<uw::domain::ImageFrame> right;
  std::optional<uw::domain::OpticalDepthPriorMeasurement> gt;

  uw::runtime::ReadMcapMessages<uw::domain::ImageFrame>(
      bag_path, "/raw/camera/left",
      [&](uint64_t, const uw::domain::ImageFrame& msg) { left = msg; });
  uw::runtime::ReadMcapMessages<uw::domain::ImageFrame>(
      bag_path, "/raw/camera/right",
      [&](uint64_t, const uw::domain::ImageFrame& msg) { right = msg; });
  uw::runtime::ReadMcapMessages<uw::domain::MeasurementEvidence>(
      bag_path, "/gt/depth", [&](uint64_t, const uw::domain::MeasurementEvidence& msg) {
        if (uw::domain::HasPayload<uw::domain::OpticalDepthPriorMeasurement>(msg)) {
          gt = uw::domain::GetPayload<uw::domain::OpticalDepthPriorMeasurement>(msg);
        }
      });

  if (!left.has_value() || !right.has_value() || !gt.has_value()) {
    std::cerr << "bag is missing /raw/camera/left, /raw/camera/right, or /gt/depth\n";
    return 1;
  }

  uw::measurement_api::CameraFrameBundle bundle;
  bundle.primary = *left;
  bundle.secondary = right;

  uw::frontends::StereoOpticalDepthFrontendParams params;
  params.matcher.window_radius = 3;
  params.matcher.min_disparity = 1;
  params.matcher.max_disparity = 32;
  uw::frontends::StereoOpticalDepthFrontend frontend(params);

  const auto evidence = frontend.Process(bundle, experiment.rig);
  if (!evidence.has_value()) {
    std::cerr << "StereoOpticalDepthFrontend rejected the bundle\n";
    return 1;
  }
  const auto& prior = uw::domain::GetPayload<uw::domain::OpticalDepthPriorMeasurement>(*evidence);

  uw::evaluation::DepthGrid estimated_grid;
  estimated_grid.width = prior.width();
  estimated_grid.height = prior.height();
  estimated_grid.depth_m.assign(prior.depth_m().begin(), prior.depth_m().end());
  estimated_grid.valid_mask.assign(prior.valid_mask().begin(), prior.valid_mask().end());

  uw::evaluation::DepthGrid gt_grid;
  gt_grid.width = gt->width();
  gt_grid.height = gt->height();
  gt_grid.depth_m.assign(gt->depth_m().begin(), gt->depth_m().end());
  gt_grid.valid_mask.assign(gt->valid_mask().begin(), gt->valid_mask().end());

  const auto metrics = uw::evaluation::ComputeDepthMetrics(estimated_grid, gt_grid);
  std::cout << "rmse_m=" << metrics.rmse_m << " mae_m=" << metrics.mae_m
            << " coverage=" << metrics.valid_coverage_fraction
            << " compared_pixels=" << metrics.num_compared_pixels << "\n";

  if (metrics.rmse_m > max_rmse_m) {
    std::cerr << "FAIL: rmse_m " << metrics.rmse_m << " exceeds max_rmse_m " << max_rmse_m << "\n";
    return 1;
  }
  if (metrics.valid_coverage_fraction < min_coverage) {
    std::cerr << "FAIL: coverage " << metrics.valid_coverage_fraction << " below min_coverage "
              << min_coverage << "\n";
    return 1;
  }
  std::cout << "OK\n";
  return 0;
}
```

Create `apps/tools/optical_baseline_eval/CMakeLists.txt`:

```cmake
add_executable(optical_baseline_eval src/main.cpp)
target_link_libraries(optical_baseline_eval PRIVATE
  uw_domain
  uw_runtime
  uw_evaluation
  uw_stereo_optical_depth_frontend
)
target_compile_options(optical_baseline_eval PRIVATE -Wall -Wextra)
```

Add `add_subdirectory(apps/tools/optical_baseline_eval)` to the top-level `CMakeLists.txt`.

- [ ] **Step 2: Write the L2 smoke test script**

Create `tests/l2_replay/optical_baseline_smoke_test.sh`:

```bash
#!/usr/bin/env bash
# L2: real synth_stereo_gen + StereoOpticalDepthFrontend + depth metrics,
# exercised end to end (plan 2 of the acoustic-optic series). Not a
# rigorous accuracy benchmark — that's plan 5's ablation/scenario-matrix
# job; this only proves the geometry pipeline recovers a known depth on a
# clean synthetic scene within a small, honestly-loose tolerance.
set -euo pipefail

SYNTH_STEREO_GEN="$1"
OPTICAL_BASELINE_EVAL="$2"
WORKDIR="$(mktemp -d)"
trap 'rm -rf "$WORKDIR"' EXIT

"$SYNTH_STEREO_GEN" --out "$WORKDIR/stereo.mcap"
"$OPTICAL_BASELINE_EVAL" --bag "$WORKDIR/stereo.mcap" --max-rmse-m 0.05 --min-coverage 0.9
```

- [ ] **Step 3: Register the test**

Add to `tests/CMakeLists.txt`:

```cmake
add_test(
  NAME uw_l2_optical_baseline_smoke_test
  COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/l2_replay/optical_baseline_smoke_test.sh
          $<TARGET_FILE:synth_stereo_gen> $<TARGET_FILE:optical_baseline_eval>
)
```

- [ ] **Step 4: Build and run**

```bash
chmod +x tests/l2_replay/optical_baseline_smoke_test.sh
cmake -S . -B build -DCMAKE_PREFIX_PATH="$HOME/miniconda3/envs/uw_slam_build"
cmake --build build -j"$(nproc)"
ctest --test-dir build -R '^uw_l2_optical_baseline_smoke_test$' --output-on-failure
```

Expected: passes, prints `rmse_m=...` well under `0.05` and `coverage=...` above `0.9`
(the scene is noiseless and fully static, so the matcher should recover the exact 6.3m depth
almost everywhere in its valid interior region — the loose thresholds only guard against
incidental texture-collision mismatches at full image scale, not real algorithm error).

- [ ] **Step 5: Review checkpoint**

If the smoke test fails on `rmse_m`/`coverage`, do not loosen the thresholds to force a pass
— first check whether `Texture()`'s modulus is producing enough distinct blocks at 640x480,
or whether `--disparity-px`/`--depth-m`/rig `fx`/`baseline` are inconsistent with the
`depth = fx*baseline/d` identity. Do not commit without explicit authorization.

---

### Task 7: Documentation and phase-wide verification

**Files:**
- Modify: `docs/uw-slam-codebase-reference-2026-08-18.md`
- Modify: `configs/README.md`
- Verify only otherwise.

- [ ] **Step 1: Update the codebase reference**

In the `frontend.hpp` optical-contract block added by plan 1 (search for "只有 L0 contract
test 里的 fake 实现"), replace the "没有真正的 `StereoOpticalDepthFrontend`" sentence with a
short note that `algorithms/frontends/stereo_optical_depth_frontend/` now provides a real,
tested implementation (block-matching stereo, MONO8 only, parallel-rig-only), still not
wired into `apps/replay_demo`. Add a `### algorithms/frontends/stereo_optical_depth_frontend`
subsection (mirroring the existing `sonar_cfar_frontend` one) documenting `PinholeCamera`,
`StereoGeometry`, `BlockMatcher`'s sign convention, and the depth/variance formulas. Update
the directory-structure map (section 3) to list the new module and the two new
`apps/tools/` binaries.

- [ ] **Step 2: Update configs/README.md**

Append to the "声光前端契约字段" section added by plan 1: note that `stereo_depth_frontend_v1`
now has a real implementation (`StereoOpticalDepthFrontend`) exercised by
`apps/tools/optical_baseline_eval` and the `uw_l2_optical_baseline_smoke_test`, but is still
not constructed by `apps/replay_demo` — `frontends.optical` remains a selector contract, not
a dispatched runtime choice, until a later plan wires synchronization/fusion into the replay
loop.

- [ ] **Step 3: Scan for contradictory capability claims**

```bash
rg -n 'camera|optical|stereo|声光' README.md configs/README.md docs/uw-slam-codebase-reference-2026-08-18.md
```

Expected: every statement distinguishes the now-real `StereoOpticalDepthFrontend` baseline
from what's still missing (synchronization, association, fusion, replay_demo wiring); no
line claims sonar+optical fusion exists or that `replay_demo` consumes camera data.

- [ ] **Step 4: Full build and test suite**

```bash
export PATH="$HOME/miniconda3/envs/uw_slam_build/bin:$PATH"
cmake -S . -B build -DCMAKE_PREFIX_PATH="$HOME/miniconda3/envs/uw_slam_build"
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
cd adapters/holoocean && python -m pytest -q && cd ../..
tools/lint/check_no_ros_in_core.sh
```

Expected: build exits 0; `ctest` reports 100% passed, including the pre-existing 15 tests
from plan 1 plus the 3 new targets from this plan (`uw_sensor_models_camera_model_test`,
`uw_stereo_optical_depth_frontend_test`, `uw_l2_optical_baseline_smoke_test`) and the
extended `uw_evaluation_test`; Python suite still 11/11; lint exits 0.

- [ ] **Step 5: Confirm the phase boundary**

```bash
rg -n 'AcousticOpticDepthFusionFrontend|AcousticOpticSynchronizer|class.*Fusion' core algorithms runtime apps
```

Expected: no matches — this plan implements the optical baseline only, no cross-modal
fusion or synchronizer.

- [ ] **Step 6: Final review checkpoint**

Run `git status --short`, list every changed/new tracked file, and compare against this
plan's file map. Report the verification commands and outputs to the user. Do not commit
unless the user explicitly requests it.
