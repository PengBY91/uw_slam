# Acoustic-Optic Probabilistic Fusion Implementation Plan

**Goal:** Implement plan 4 of the 6-plan acoustic-optic series: a bounded
scalar posterior depth optimizer and `AcousticOpticDepthFusionFrontend::Fuse`
(design spec section 7.2), producing real `FusedDepthMeasurement` wire
evidence for the first time — the first plan with something genuinely
end-to-end worth evaluating.

**Architecture:** Composes plan 3's `AcousticOpticAssociator` (geometric
candidate selection) with a new bounded 1D posterior optimizer. Only a
plan-3 `ACCEPTED` record is ever handed to the optimizer; its outcome can
still downgrade that pixel to `REJECTED` (`POSTERIOR_INVALID`/
`VARIANCE_NOT_IMPROVED`) or `CONFLICT` (`CROSS_MODAL_CONFLICT`) — the two
reasons plan 3 explicitly reserved. New module:
`algorithms/frontends/acoustic_optic_depth_fusion/` (design spec section 5.2
groups fusion under `algorithms/frontends/`, alongside the associator and
stereo frontend).

**Fail-closed policy (design spec section 9, "不能证明一致，就不融合"):**
`Fuse()` always returns a full-image `FusedDepthMeasurement` when the
optical prior is present — every pixel starts as
`DEPTH_CONTRIBUTION_OPTICAL_ONLY` (a straight passthrough of the optical
prior's depth/variance). At most one pixel (the plan-3 top-1 sonar
hypothesis's selected pixel, if geometric association accepted it) can be
upgraded to `DEPTH_CONTRIBUTION_ACOUSTIC_OPTIC` — only if the posterior
optimizer converges to a finite result, meaningfully improves variance over
the prior, and its residual against the sonar measurement passes an
innovation gate. Any failure at any of those three checks leaves that pixel
as optical-only, not fabricated. An empty `HypothesisSet` (sonar dropout,
design spec section 10 scenario 8) degrades gracefully to a full
optical-only passthrough with an empty `associations` list — not an error.

**Posterior update math (design spec section 8.4):** for the fixed selected
pixel, optimize scalar depth `d`:

```
min_d  (d - d_o)^2/sigma_d^2 + (range(d) - rho)^2/sigma_rho^2 + (bearing(d) - theta)^2/sigma_theta^2
```

where `range(d)`/`bearing(d)` come directly from plan 3's
`UnprojectPixelToSonarRangeBearing` at the fixed pixel — this plan adds no
new geometry primitive, only a scalar optimizer around the existing one.
v1 uses **plain squared residuals** (Gaussian loss — Huber/Cauchy robust
loss is a documented future enhancement, not implemented here) and a
**deterministic, bounded golden-section search** over
`d in [d_o - k*sigma_d, d_o + k*sigma_d]` — never unconstrained Gauss-Newton,
so it cannot diverge, at the cost of assuming near-unimodality within that
window (same spirit as `GaussNewtonSolver`'s own documented v1 scope,
`algorithms/estimation`). Posterior variance comes from a Laplace
approximation: `variance = 2 / f''(d*)`, `f''` estimated by a central finite
difference — this is a real, standard technique (asymptotic variance of a
least-squares estimator from local curvature of its cost), not a fabricated
number.

**Tech Stack:** C++17, Eigen, GoogleTest. No new external dependency.

**Scope boundary (do not implement in this plan):**
- Robust loss functions (Huber/Cauchy) — plain squared residuals only.
- Visibility/occlusion checks (design spec section 8.3's last acceptance
  criterion) — explicitly out of scope for v1, same as plan 3.
- Multi-hypothesis fusion — inherits the top-1 rule from
  `AcousticOpticAssociator`.
- Any change to `apps/replay_demo`, `apps/tools/synth_bag_gen`,
  `apps/tools/synth_stereo_gen`, or `apps/tools/optical_baseline_eval` —
  this plan adds no new app. A `FusedDepthMeasurement`-producing end-to-end
  harness (mirroring `optical_baseline_eval`'s pattern) is deferred to
  plan 5 (`Simulation/replay/evaluation`), which owns the scenario matrix
  and ablation harness this would need to be meaningfully evaluated
  against.
- The 9-scenario matrix, ablations, or the MVP acceptance gates (design
  spec section 12.2) — plan 5's job.

Repository policy: do not create git commits unless the user explicitly
authorizes them. Each task ends with a review checkpoint, not a commit.

## File map

### Create

- `algorithms/frontends/acoustic_optic_depth_fusion/CMakeLists.txt`
- `algorithms/frontends/acoustic_optic_depth_fusion/include/uw/frontends/posterior_depth_optimizer.hpp`
- `algorithms/frontends/acoustic_optic_depth_fusion/src/posterior_depth_optimizer.cpp`
- `algorithms/frontends/acoustic_optic_depth_fusion/test/posterior_depth_optimizer_test.cpp`
- `algorithms/frontends/acoustic_optic_depth_fusion/include/uw/frontends/acoustic_optic_depth_fusion_frontend.hpp`
- `algorithms/frontends/acoustic_optic_depth_fusion/src/acoustic_optic_depth_fusion_frontend.cpp`
- `algorithms/frontends/acoustic_optic_depth_fusion/test/acoustic_optic_depth_fusion_frontend_test.cpp`

### Modify

- `CMakeLists.txt` — register the new subdirectory.
- `docs/uw-slam-codebase-reference-2026-08-18.md` — document the new module.

---

### Task 1: Bounded posterior depth optimizer

**Files:**
- Create: `algorithms/frontends/acoustic_optic_depth_fusion/CMakeLists.txt`, `include/uw/frontends/posterior_depth_optimizer.hpp`, `src/posterior_depth_optimizer.cpp`, `test/posterior_depth_optimizer_test.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Create the module skeleton**

Create `algorithms/frontends/acoustic_optic_depth_fusion/CMakeLists.txt`:

```cmake
add_library(uw_acoustic_optic_depth_fusion STATIC
  src/posterior_depth_optimizer.cpp
  src/acoustic_optic_depth_fusion_frontend.cpp
)
target_include_directories(uw_acoustic_optic_depth_fusion PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(uw_acoustic_optic_depth_fusion PUBLIC uw_acoustic_optic_associator uw_sensor_models)
target_compile_options(uw_acoustic_optic_depth_fusion PRIVATE -Wall -Wextra)

if(UW_BUILD_TESTS)
  add_executable(uw_acoustic_optic_depth_fusion_test
    test/posterior_depth_optimizer_test.cpp
    test/acoustic_optic_depth_fusion_frontend_test.cpp
  )
  target_link_libraries(uw_acoustic_optic_depth_fusion_test PRIVATE uw_acoustic_optic_depth_fusion GTest::gtest GTest::gtest_main)
  add_test(NAME uw_acoustic_optic_depth_fusion_test COMMAND uw_acoustic_optic_depth_fusion_test)
endif()
```

Add `add_subdirectory(algorithms/frontends/acoustic_optic_depth_fusion)` to the top-level
`CMakeLists.txt`, after `acoustic_optic_associator`. Leave `acoustic_optic_depth_fusion_frontend.cpp`
as an empty translation unit for now (Task 2 fills it in) so the target links.

- [ ] **Step 2: Write the failing optimizer tests**

Create `algorithms/frontends/acoustic_optic_depth_fusion/test/posterior_depth_optimizer_test.cpp`:

```cpp
#include <cmath>

#include <gtest/gtest.h>

#include "uw/frontends/posterior_depth_optimizer.hpp"

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

TEST(PosteriorDepthOptimizer, PullsBoresightPriorTowardWeightedLeastSquaresOptimum) {
  // At boresight (pixel = image center, camera co-located/co-oriented with
  // the sonar), range(d) = d and bearing(d) = 0 for every d — this
  // collapses the cost to a closed-form weighted least squares between
  // the optical prior and the sonar range, letting the test assert an
  // exact expected answer instead of just "moved in the right direction".
  const auto camera = MakeCamera();
  const double prior_depth = 5.2, prior_variance = 0.09;       // sigma_d = 0.3
  const double sonar_range = 5.0, sonar_range_sigma = 0.05;    // sigma_rho = 0.05
  const double sonar_bearing = 0.0, sonar_bearing_sigma = 0.02;

  uw::frontends::PosteriorDepthOptimizerParams params;
  params.iterations = 60;
  params.search_radius_sigma = 3.0;
  const auto result = uw::frontends::OptimizePosteriorDepth(
      /*pixel_u=*/10.0, /*pixel_v=*/5.0, prior_depth, prior_variance, sonar_range,
      sonar_range_sigma, sonar_bearing, sonar_bearing_sigma,
      uw::sensor_models::Pose3::Identity(), camera, params);

  ASSERT_TRUE(result.valid);
  const double w_prior = 1.0 / prior_variance;
  const double w_sonar = 1.0 / (sonar_range_sigma * sonar_range_sigma);
  const double expected_depth = (prior_depth * w_prior + sonar_range * w_sonar) / (w_prior + w_sonar);
  EXPECT_NEAR(result.depth_m, expected_depth, 1e-4);
  const double expected_second_derivative = 2.0 * w_prior + 2.0 * w_sonar;
  EXPECT_NEAR(result.variance_m2, 2.0 / expected_second_derivative, 1e-4);
  EXPECT_LT(result.variance_m2, prior_variance);
  EXPECT_NEAR(result.bearing_residual_rad, 0.0, 1e-6);
}

TEST(PosteriorDepthOptimizer, InvalidWhenSonarSigmaIsNonPositive) {
  const auto camera = MakeCamera();
  uw::frontends::PosteriorDepthOptimizerParams params;
  const auto result = uw::frontends::OptimizePosteriorDepth(
      10.0, 5.0, /*prior_depth_m=*/5.0, /*prior_variance_m2=*/0.09, /*sonar_range_m=*/5.0,
      /*sonar_range_sigma_m=*/0.0,  // invalid: zero uncertainty is not a real measurement
      0.0, 0.02, uw::sensor_models::Pose3::Identity(), camera, params);
  EXPECT_FALSE(result.valid);
}

TEST(PosteriorDepthOptimizer, InvalidWhenPriorVarianceIsNonPositive) {
  const auto camera = MakeCamera();
  uw::frontends::PosteriorDepthOptimizerParams params;
  const auto result = uw::frontends::OptimizePosteriorDepth(
      10.0, 5.0, 5.0, /*prior_variance_m2=*/0.0, 5.0, 0.05, 0.0, 0.02,
      uw::sensor_models::Pose3::Identity(), camera, params);
  EXPECT_FALSE(result.valid);
}
```

- [ ] **Step 3: Verify it fails to compile**

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH="$HOME/miniconda3/envs/uw_slam_build"
cmake --build build --target uw_acoustic_optic_depth_fusion_test -j"$(nproc)"
```

- [ ] **Step 4: Implement**

```cpp
// algorithms/frontends/acoustic_optic_depth_fusion/include/uw/frontends/posterior_depth_optimizer.hpp
#pragma once

#include "uw/sensor_models/camera_model.hpp"
#include "uw/sensor_models/geometry.hpp"

namespace uw::frontends {

struct PosteriorDepthOptimizerParams {
  int iterations = 30;
  double search_radius_sigma = 3.0;
};

struct PosteriorDepthResult {
  bool valid = false;
  double depth_m = 0.0;
  double variance_m2 = 0.0;
  double range_residual_m = 0.0;
  double bearing_residual_rad = 0.0;
};

// Optimizes a scalar depth `d` at a fixed pixel (design spec section 8.4):
//   min_d (d-d_o)^2/sigma_d^2 + (range(d)-rho)^2/sigma_rho^2 + (bearing(d)-theta)^2/sigma_theta^2
// where range(d)/bearing(d) come from UnprojectPixelToSonarRangeBearing at the fixed pixel.
// v1 uses plain squared residuals (Gaussian loss — Huber/Cauchy is a documented future
// enhancement) and a deterministic, bounded golden-section search over
// d in [d_o - k*sigma_d, d_o + k*sigma_d] (k = search_radius_sigma) — never unconstrained
// Gauss-Newton, so it cannot diverge, at the cost of assuming near-unimodality within that
// window. Posterior variance is a Laplace approximation (2/f''(d*), f'' via central finite
// difference). Returns valid=false if any input sigma/prior variance is <= 0 or the optimum
// is non-finite — callers must fall back to the optical prior in that case.
PosteriorDepthResult OptimizePosteriorDepth(double pixel_u, double pixel_v, double prior_depth_m,
                                            double prior_variance_m2, double sonar_range_m,
                                            double sonar_range_sigma_m, double sonar_bearing_rad,
                                            double sonar_bearing_sigma_rad,
                                            const uw::sensor_models::Pose3& camera_T_sonar,
                                            const uw::sensor_models::PinholeCamera& camera,
                                            const PosteriorDepthOptimizerParams& params);

}  // namespace uw::frontends
```

```cpp
// algorithms/frontends/acoustic_optic_depth_fusion/src/posterior_depth_optimizer.cpp
#include "uw/frontends/posterior_depth_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

#include "uw/sensor_models/sonar_arc_projector.hpp"

namespace uw::frontends {

namespace {

double GoldenSectionMinimize(const std::function<double(double)>& f, double lo, double hi,
                             int iterations) {
  const double phi = (std::sqrt(5.0) - 1.0) / 2.0;
  double a = lo, b = hi;
  double c = b - phi * (b - a);
  double d = a + phi * (b - a);
  for (int i = 0; i < iterations; ++i) {
    if (f(c) < f(d)) {
      b = d;
    } else {
      a = c;
    }
    c = b - phi * (b - a);
    d = a + phi * (b - a);
  }
  return (a + b) / 2.0;
}

}  // namespace

PosteriorDepthResult OptimizePosteriorDepth(double pixel_u, double pixel_v, double prior_depth_m,
                                            double prior_variance_m2, double sonar_range_m,
                                            double sonar_range_sigma_m, double sonar_bearing_rad,
                                            double sonar_bearing_sigma_rad,
                                            const uw::sensor_models::Pose3& camera_T_sonar,
                                            const uw::sensor_models::PinholeCamera& camera,
                                            const PosteriorDepthOptimizerParams& params) {
  PosteriorDepthResult result;
  if (prior_variance_m2 <= 0.0 || sonar_range_sigma_m <= 0.0 || sonar_bearing_sigma_rad <= 0.0) {
    return result;
  }

  const double sigma_d = std::sqrt(prior_variance_m2);
  auto cost = [&](double d) {
    const auto observed = uw::sensor_models::UnprojectPixelToSonarRangeBearing(
        pixel_u, pixel_v, d, camera_T_sonar, camera);
    const double range_residual = observed.range_m - sonar_range_m;
    const double bearing_residual = observed.bearing_rad - sonar_bearing_rad;
    const double prior_residual = d - prior_depth_m;
    return (prior_residual * prior_residual) / (sigma_d * sigma_d) +
           (range_residual * range_residual) / (sonar_range_sigma_m * sonar_range_sigma_m) +
           (bearing_residual * bearing_residual) / (sonar_bearing_sigma_rad * sonar_bearing_sigma_rad);
  };

  const double radius = params.search_radius_sigma * sigma_d;
  const double lo = std::max(1e-3, prior_depth_m - radius);
  const double hi = prior_depth_m + radius;
  if (!(hi > lo)) return result;

  const double d_star = GoldenSectionMinimize(cost, lo, hi, params.iterations);
  const double f_star = cost(d_star);
  if (!std::isfinite(d_star) || !std::isfinite(f_star)) return result;

  const double h = std::max(1e-6, 1e-4 * std::max(1.0, std::abs(d_star)));
  const double second_derivative = (cost(d_star + h) - 2.0 * f_star + cost(d_star - h)) / (h * h);
  if (!(second_derivative > 0.0) || !std::isfinite(second_derivative)) return result;

  const auto observed_at_star = uw::sensor_models::UnprojectPixelToSonarRangeBearing(
      pixel_u, pixel_v, d_star, camera_T_sonar, camera);

  result.valid = true;
  result.depth_m = d_star;
  result.variance_m2 = 2.0 / second_derivative;
  result.range_residual_m = observed_at_star.range_m - sonar_range_m;
  result.bearing_residual_rad = observed_at_star.bearing_rad - sonar_bearing_rad;
  return result;
}

}  // namespace uw::frontends
```

- [ ] **Step 5: Build and run**

```bash
cmake --build build --target uw_acoustic_optic_depth_fusion_test -j"$(nproc)"
ctest --test-dir build -R '^uw_acoustic_optic_depth_fusion_test$' --output-on-failure
```

Expected: all 3 tests pass (the frontend test file is still a placeholder at this point —
Task 2 replaces it).

- [ ] **Step 6: Review checkpoint**

Confirm the golden-section search is deterministic (fixed iteration count, no early-exit
tolerance check that could vary iteration count run-to-run) and that `valid=false` is
returned rather than a NaN/Inf result on any of the three guarded failure modes. Do not
commit without explicit authorization.

---

### Task 2: `AcousticOpticDepthFusionFrontend::Fuse`

**Files:**
- Modify: `algorithms/frontends/acoustic_optic_depth_fusion/include/uw/frontends/acoustic_optic_depth_fusion_frontend.hpp`, `src/acoustic_optic_depth_fusion_frontend.cpp`, `test/acoustic_optic_depth_fusion_frontend_test.cpp`

- [ ] **Step 1: Write the failing integration tests**

Create `algorithms/frontends/acoustic_optic_depth_fusion/test/acoustic_optic_depth_fusion_frontend_test.cpp`
(reuses the co-located rig / boresight fixture pattern from plan 3's associator test):

```cpp
#include <cmath>

#include <gtest/gtest.h>

#include "uw/frontends/acoustic_optic_depth_fusion_frontend.hpp"

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

uw::domain::HypothesisSet MakeSonarHypothesis(double range_m, double bearing_rad,
                                               double range_sigma_m, double bearing_sigma_rad) {
  uw::domain::SonarRangeBearing measurement;
  measurement.set_range_m(range_m);
  measurement.set_bearing_rad(bearing_rad);
  measurement.set_range_sigma_m(range_sigma_m);
  measurement.set_bearing_sigma_rad(bearing_sigma_rad);
  uw::domain::EvidenceId id;
  id.set_value("sonar_hyp_1");
  auto evidence = uw::domain::MakeEvidence(id, {}, measurement, 1.0, "sonar_cfar_frontend_v1");
  uw::domain::HypothesisSet hypotheses;
  *hypotheses.add_candidates() = evidence;
  return hypotheses;
}

uw::domain::MeasurementEvidence MakeOpticalEvidence(int width, int height, int valid_index,
                                                     float depth_m, float variance_m2) {
  uw::domain::OpticalDepthPriorMeasurement prior;
  prior.set_width(width);
  prior.set_height(height);
  const int pixels = width * height;
  std::string valid_mask(pixels, '\0');
  for (int i = 0; i < pixels; ++i) {
    prior.add_depth_m(i == valid_index ? depth_m : 1.0f);
    prior.add_variance_m2(i == valid_index ? variance_m2 : 0.01f);
  }
  valid_mask[valid_index] = 1;
  prior.set_valid_mask(valid_mask);
  prior.set_scale_status(uw::domain::OPTICAL_DEPTH_SCALE_STATUS_METRIC);
  prior.set_producer_type("stereo");
  uw::domain::EvidenceId id;
  id.set_value("optical_1");
  return uw::domain::MakeEvidence(id, {}, prior, 1.0, "stereo_depth_frontend_v1");
}

}  // namespace

TEST(AcousticOpticDepthFusionFrontend, AcceptsAndCorrectsTheSelectedPixel) {
  const auto rig = MakeCoLocatedRig();
  const auto sonar_hypotheses =
      MakeSonarHypothesis(/*range_m=*/5.0, 0.0, /*range_sigma_m=*/0.05, /*bearing_sigma_rad=*/0.02);
  const auto optical_evidence =
      MakeOpticalEvidence(20, 10, /*valid_index=*/110, /*depth_m=*/5.2, /*variance_m2=*/0.09);

  uw::frontends::AcousticOpticDepthFusionParams params;
  params.optimizer.iterations = 60;
  uw::frontends::AcousticOpticDepthFusionFrontend fusion(params);
  const auto result = fusion.Fuse(sonar_hypotheses, optical_evidence, rig, /*time_delta_seconds=*/0.0);

  ASSERT_TRUE(result.has_value());
  const auto& fused = uw::domain::GetPayload<uw::domain::FusedDepthMeasurement>(result->fused_evidence);
  ASSERT_EQ(uw::domain::ValidateFusedDepth(fused).code, uw::domain::ValidationCode::kOk);

  const double w_prior = 1.0 / 0.09, w_sonar = 1.0 / (0.05 * 0.05);
  const double expected_depth = (5.2 * w_prior + 5.0 * w_sonar) / (w_prior + w_sonar);
  EXPECT_NEAR(fused.depth_m(110), expected_depth, 1e-3);
  EXPECT_LT(fused.variance_m2(110), 0.09);
  EXPECT_EQ(static_cast<unsigned char>(fused.contribution_mask()[110]),
            uw::domain::DEPTH_CONTRIBUTION_ACOUSTIC_OPTIC);
  // An untouched but optical-valid pixel stays a pure passthrough.
  EXPECT_NEAR(fused.depth_m(0), 1.0, 1e-9);
  EXPECT_EQ(static_cast<unsigned char>(fused.contribution_mask()[0]),
            uw::domain::DEPTH_CONTRIBUTION_OPTICAL_ONLY);

  ASSERT_EQ(fused.associations_size(), 1);
  EXPECT_EQ(fused.associations(0).status(), uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_ACCEPTED);
  EXPECT_NEAR(fused.associations(0).posterior_depth_m(), expected_depth, 1e-3);
}

TEST(AcousticOpticDepthFusionFrontend, FallsBackToOpticalOnlyOnSonarDropout) {
  const auto rig = MakeCoLocatedRig();
  uw::domain::HypothesisSet empty_hypotheses;
  const auto optical_evidence = MakeOpticalEvidence(20, 10, 110, 5.2, 0.09);

  uw::frontends::AcousticOpticDepthFusionParams params;
  uw::frontends::AcousticOpticDepthFusionFrontend fusion(params);
  const auto result = fusion.Fuse(empty_hypotheses, optical_evidence, rig, 0.0);

  ASSERT_TRUE(result.has_value());
  const auto& fused = uw::domain::GetPayload<uw::domain::FusedDepthMeasurement>(result->fused_evidence);
  EXPECT_EQ(fused.associations_size(), 0);
  EXPECT_EQ(static_cast<unsigned char>(fused.contribution_mask()[110]),
            uw::domain::DEPTH_CONTRIBUTION_OPTICAL_ONLY);
  EXPECT_NEAR(fused.depth_m(110), 5.2, 1e-9);  // unchanged
}

TEST(AcousticOpticDepthFusionFrontend, FallsBackWhenInnovationGateRejectsThePosterior) {
  const auto rig = MakeCoLocatedRig();
  const auto sonar_hypotheses = MakeSonarHypothesis(5.0, 0.0, 0.05, 0.02);
  const auto optical_evidence = MakeOpticalEvidence(20, 10, 110, 5.2, 0.09);

  uw::frontends::AcousticOpticDepthFusionParams params;
  params.optimizer.iterations = 60;
  params.innovation_gate_sigma = 0.05;  // unrealistically tight — the ~0.005m residual will fail it
  uw::frontends::AcousticOpticDepthFusionFrontend fusion(params);
  const auto result = fusion.Fuse(sonar_hypotheses, optical_evidence, rig, 0.0);

  ASSERT_TRUE(result.has_value());
  const auto& fused = uw::domain::GetPayload<uw::domain::FusedDepthMeasurement>(result->fused_evidence);
  ASSERT_EQ(fused.associations_size(), 1);
  EXPECT_EQ(fused.associations(0).status(), uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_CONFLICT);
  EXPECT_EQ(fused.associations(0).reason(), uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_CROSS_MODAL_CONFLICT);
  EXPECT_EQ(static_cast<unsigned char>(fused.contribution_mask()[110]),
            uw::domain::DEPTH_CONTRIBUTION_OPTICAL_ONLY);
  EXPECT_NEAR(fused.depth_m(110), 5.2, 1e-9);  // fallback: unchanged
}

TEST(AcousticOpticDepthFusionFrontend, FallsBackWhenVarianceIsNotSufficientlyImproved) {
  const auto rig = MakeCoLocatedRig();
  const auto sonar_hypotheses = MakeSonarHypothesis(5.0, 0.0, 0.05, 0.02);
  const auto optical_evidence = MakeOpticalEvidence(20, 10, 110, 5.2, 0.09);

  uw::frontends::AcousticOpticDepthFusionParams params;
  params.optimizer.iterations = 60;
  params.min_variance_improvement_fraction = 0.999;  // demand a 99.9% reduction — unattainable here
  uw::frontends::AcousticOpticDepthFusionFrontend fusion(params);
  const auto result = fusion.Fuse(sonar_hypotheses, optical_evidence, rig, 0.0);

  ASSERT_TRUE(result.has_value());
  const auto& fused = uw::domain::GetPayload<uw::domain::FusedDepthMeasurement>(result->fused_evidence);
  ASSERT_EQ(fused.associations_size(), 1);
  EXPECT_EQ(fused.associations(0).status(), uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_REJECTED);
  EXPECT_EQ(fused.associations(0).reason(), uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_VARIANCE_NOT_IMPROVED);
}

TEST(AcousticOpticDepthFusionFrontend, ReturnsNulloptWhenOpticalEvidenceHasNoPrior) {
  const auto rig = MakeCoLocatedRig();
  const auto sonar_hypotheses = MakeSonarHypothesis(5.0, 0.0, 0.05, 0.02);
  uw::domain::PressureDepthMeasurement wrong_payload;
  uw::domain::EvidenceId id;
  id.set_value("not_optical");
  const auto not_optical_evidence =
      uw::domain::MakeEvidence(id, {}, wrong_payload, 1.0, "irrelevant_v1");

  uw::frontends::AcousticOpticDepthFusionParams params;
  uw::frontends::AcousticOpticDepthFusionFrontend fusion(params);
  EXPECT_FALSE(fusion.Fuse(sonar_hypotheses, not_optical_evidence, rig, 0.0).has_value());
}
```

- [ ] **Step 2: Verify it fails to compile**

```bash
cmake --build build --target uw_acoustic_optic_depth_fusion_test -j"$(nproc)"
```

- [ ] **Step 3: Implement the fusion frontend**

```cpp
// algorithms/frontends/acoustic_optic_depth_fusion/include/uw/frontends/acoustic_optic_depth_fusion_frontend.hpp
#pragma once

#include <cstdint>
#include <optional>

#include "uw/frontends/acoustic_optic_associator.hpp"
#include "uw/frontends/posterior_depth_optimizer.hpp"

namespace uw::frontends {

struct AcousticOpticDepthFusionParams {
  AcousticOpticAssociatorParams associator;
  PosteriorDepthOptimizerParams optimizer;
  double min_variance_improvement_fraction = 0.05;
  double innovation_gate_sigma = 3.0;
};

struct FusedDepthResult {
  uw::domain::MeasurementEvidence fused_evidence;
  uw::domain::HealthReport health;
};

// AcousticOpticDepthFusionFrontend::Fuse (design spec section 7.2/8.4): runs
// AcousticOpticAssociator's geometric association, then — only for an
// ACCEPTED record — a bounded posterior depth optimization at the selected
// pixel. Fails closed at every stage (design spec section 9's "不能证明
// 一致，就不融合" policy): any geometric rejection, non-finite/un-improved
// posterior, or innovation-gate failure leaves that pixel's fused depth
// equal to the optical prior (DEPTH_CONTRIBUTION_OPTICAL_ONLY), never
// fabricates a corrected value. An empty HypothesisSet (sonar dropout,
// design spec section 10 scenario 8) degrades gracefully to a full
// optical-only passthrough — documented behavior, not an error. Returns
// nullopt only when the optical evidence itself has no
// OpticalDepthPriorMeasurement payload (nothing to build from).
class AcousticOpticDepthFusionFrontend {
 public:
  explicit AcousticOpticDepthFusionFrontend(AcousticOpticDepthFusionParams params);

  std::optional<FusedDepthResult> Fuse(const uw::domain::HypothesisSet& sonar_hypotheses,
                                       const uw::domain::MeasurementEvidence& optical_evidence,
                                       const uw::domain::RigCalibrationSnapshot& rig,
                                       double time_delta_seconds);

 private:
  AcousticOpticDepthFusionParams params_;
  AcousticOpticAssociator associator_;
  uint64_t next_evidence_id_ = 1;
};

}  // namespace uw::frontends
```

```cpp
// algorithms/frontends/acoustic_optic_depth_fusion/src/acoustic_optic_depth_fusion_frontend.cpp
#include "uw/frontends/acoustic_optic_depth_fusion_frontend.hpp"

#include <cmath>
#include <optional>
#include <string>
#include <vector>

namespace uw::frontends {

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

AcousticOpticDepthFusionFrontend::AcousticOpticDepthFusionFrontend(AcousticOpticDepthFusionParams params)
    : params_(params), associator_(params.associator) {}

std::optional<FusedDepthResult> AcousticOpticDepthFusionFrontend::Fuse(
    const uw::domain::HypothesisSet& sonar_hypotheses,
    const uw::domain::MeasurementEvidence& optical_evidence,
    const uw::domain::RigCalibrationSnapshot& rig, double time_delta_seconds) {
  if (!uw::domain::HasPayload<uw::domain::OpticalDepthPriorMeasurement>(optical_evidence)) {
    return std::nullopt;
  }
  const auto& prior = uw::domain::GetPayload<uw::domain::OpticalDepthPriorMeasurement>(optical_evidence);

  uw::domain::FusedDepthMeasurement fused;
  *fused.mutable_reference_camera_frame() = prior.reference_camera_frame();
  fused.set_width(prior.width());
  fused.set_height(prior.height());
  const std::size_t pixels = static_cast<std::size_t>(prior.width()) * prior.height();
  std::string valid_mask(pixels, '\0');
  std::string contribution_mask(pixels, static_cast<char>(uw::domain::DEPTH_CONTRIBUTION_INVALID));
  for (std::size_t i = 0; i < pixels; ++i) {
    fused.add_depth_m(prior.depth_m(static_cast<int>(i)));
    fused.add_variance_m2(prior.variance_m2(static_cast<int>(i)));
    if (i < prior.valid_mask().size() && prior.valid_mask()[i] != 0) {
      valid_mask[i] = 1;
      contribution_mask[i] = static_cast<char>(uw::domain::DEPTH_CONTRIBUTION_OPTICAL_ONLY);
    }
  }

  auto audit = associator_.Associate(sonar_hypotheses, optical_evidence, rig, time_delta_seconds);

  if (!audit.records.empty()) {
    auto& record = audit.records[0];
    if (record.status() == uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_ACCEPTED &&
        record.has_selected_pixel()) {
      const auto* camera_intrinsics = FindCamera(rig, params_.associator.camera_sensor_id);
      const auto camera_pose = FindEdgePose(rig, params_.associator.camera_frame);
      const auto sonar_pose = FindEdgePose(rig, params_.associator.sonar_frame);

      if (camera_intrinsics != nullptr && camera_pose.has_value() && sonar_pose.has_value()) {
        const auto& top_sonar =
            uw::domain::GetPayload<uw::domain::SonarRangeBearing>(sonar_hypotheses.candidates(0));
        const auto camera = uw::sensor_models::PinholeCamera::FromIntrinsics(*camera_intrinsics);
        const uw::sensor_models::Pose3 camera_T_sonar = camera_pose->Inverse() * (*sonar_pose);
        const std::size_t idx = record.selected_pixel_index();
        const std::size_t v = idx / prior.width();
        const std::size_t u = idx % prior.width();

        const auto posterior = OptimizePosteriorDepth(
            static_cast<double>(u), static_cast<double>(v), record.prior_depth_m(),
            record.prior_variance_m2(), top_sonar.range_m(), top_sonar.range_sigma_m(),
            top_sonar.bearing_rad(), top_sonar.bearing_sigma_rad(), camera_T_sonar, camera,
            params_.optimizer);

        if (!posterior.valid) {
          record.set_status(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_REJECTED);
          record.set_reason(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_POSTERIOR_INVALID);
        } else if (posterior.variance_m2 >
                   record.prior_variance_m2() * (1.0 - params_.min_variance_improvement_fraction)) {
          record.set_status(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_REJECTED);
          record.set_reason(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_VARIANCE_NOT_IMPROVED);
        } else if (std::abs(posterior.range_residual_m) >
                       params_.innovation_gate_sigma * top_sonar.range_sigma_m() ||
                   std::abs(posterior.bearing_residual_rad) >
                       params_.innovation_gate_sigma * top_sonar.bearing_sigma_rad()) {
          record.set_status(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_CONFLICT);
          record.set_reason(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_CROSS_MODAL_CONFLICT);
        } else {
          record.set_posterior_depth_m(posterior.depth_m);
          record.set_posterior_variance_m2(posterior.variance_m2);
          fused.set_depth_m(static_cast<int>(idx), static_cast<float>(posterior.depth_m));
          fused.set_variance_m2(static_cast<int>(idx), static_cast<float>(posterior.variance_m2));
          contribution_mask[idx] = static_cast<char>(uw::domain::DEPTH_CONTRIBUTION_ACOUSTIC_OPTIC);
        }
      } else {
        record.set_status(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_REJECTED);
        record.set_reason(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_CALIBRATION);
      }
    }
    *fused.add_associations() = record;
  }

  fused.set_valid_mask(valid_mask);
  fused.set_contribution_mask(contribution_mask);

  uw::domain::EvidenceId evidence_id;
  evidence_id.set_value("fused_depth_" + std::to_string(next_evidence_id_++));
  std::vector<uw::domain::ObservationId> sources(optical_evidence.source_observations().begin(),
                                                  optical_evidence.source_observations().end());

  FusedDepthResult result;
  result.fused_evidence = uw::domain::MakeEvidence(evidence_id, sources, fused, /*noise_scale=*/1.0,
                                                    "acoustic_optic_depth_fusion_v1");
  result.health.set_component_id("acoustic_optic_depth_fusion_frontend");
  return result;
}

}  // namespace uw::frontends
```

- [ ] **Step 4: Build and run**

```bash
cmake --build build --target uw_acoustic_optic_depth_fusion_test -j"$(nproc)"
ctest --test-dir build -R '^uw_acoustic_optic_depth_fusion_test$' --output-on-failure
```

Expected: all 8 tests pass (3 from Task 1 + 5 from Task 2).

- [ ] **Step 5: Review checkpoint**

Confirm `ValidateFusedDepth` (plan 1) accepts the produced evidence, and that every fallback
path (`POSTERIOR_INVALID`/`VARIANCE_NOT_IMPROVED`/`CROSS_MODAL_CONFLICT`/sonar dropout) leaves
`contribution_mask` at `OPTICAL_ONLY` and `depth_m`/`variance_m2` exactly equal to the optical
prior's values at that pixel — never a partially-applied posterior. Do not commit without
explicit authorization.

---

### Task 3: Documentation and phase-wide verification

**Files:**
- Modify: `docs/uw-slam-codebase-reference-2026-08-18.md`
- Verify only otherwise.

- [ ] **Step 1: Add a `6.9` subsection**

Document `posterior_depth_optimizer.hpp` (golden-section search, Laplace variance, the three
`valid=false` guards) and `AcousticOpticDepthFusionFrontend::Fuse` (the fail-closed decision
tree: geometric rejection from plan 3 passes through unchanged; posterior failure modes map
to `POSTERIOR_INVALID`/`VARIANCE_NOT_IMPROVED`/`CROSS_MODAL_CONFLICT`; sonar dropout produces
a full optical-only `FusedDepthMeasurement`, not an error). State plainly that this is still
not wired into any app — plan 5 is where a `FusedDepthMeasurement`-producing end-to-end
harness and the scenario matrix belong.

- [ ] **Step 2: Scan for contradictory capability claims**

```bash
rg -n 'camera|optical|stereo|声光|synchroniz|associat|fusion|posterior' README.md configs/README.md docs/uw-slam-codebase-reference-2026-08-18.md
```

Expected: no line claims an end-to-end app, scenario matrix, or MVP gate result exists yet.

- [ ] **Step 3: Full build and test suite**

```bash
export PATH="$HOME/miniconda3/envs/uw_slam_build/bin:$PATH"
cmake -S . -B build -DCMAKE_PREFIX_PATH="$HOME/miniconda3/envs/uw_slam_build"
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
cd adapters/holoocean && python -m pytest -q && cd ../..
tools/lint/check_no_ros_in_core.sh
```

Expected: build exits 0; ctest 100% passed (the prior 21 plus this plan's 1 new test target
containing 8 cases = 22 registered tests); Python still 11/11; lint exits 0.

- [ ] **Step 4: Final review checkpoint**

Run `git status --short`, list every changed/new tracked file, compare against this plan's
file map. Report the verification commands and outputs to the user. Do not commit unless the
user explicitly requests it.
