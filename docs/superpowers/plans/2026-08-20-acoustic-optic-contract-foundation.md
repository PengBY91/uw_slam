# Acoustic-Optic Contract Foundation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add the canonical camera, optical-depth, fused-depth, calibration, and frontend interface contracts required by the acoustic-optic dense-depth MVP, without implementing stereo matching or fusion algorithms yet.

**Architecture:** Protobuf remains the only cross-language source of truth. Raw camera frames and typed depth evidence enter `uw_domain`; generic camera/optical frontend abstractions enter the header-only `uw_measurement_api`; runtime YAML loading populates the existing `RigCalibrationSnapshot` instead of creating parallel calibration structs. This phase ends with C++/Python round-trip tests and a fully specified stereo rig, providing a testable foundation for the later optical-baseline and fusion plans.

**Tech Stack:** C++17, Protobuf, CMake, GoogleTest, yaml-cpp, Python 3.10+, pytest, MCAP

---

## Scope and plan series

The approved design spans six sequential, independently verifiable plans:

1. **This plan — contract/calibration foundation:** canonical camera/depth messages, validation, interfaces, rig parsing, Python round trips.
2. Optical baseline: canonical stereo data, rectification/disparity, metric `OpticalDepthPriorMeasurement`, GT depth metrics.
3. Cross-modal geometry: capture-time synchronizer, FLS arc-band projection, candidate generation and association audit.
4. Probabilistic fusion: robust posterior depth update, uncertainty, fallback and health reporting.
5. Simulation/replay/evaluation: fixed scenario matrix, ablations, deterministic output, metric and latency gates.
6. Mapping handoff: fused depth to local surface evidence and pose-version-aware retransform.

Do not implement stereo, synchronization, association, posterior optimization, semantic processing, monocular scale alignment, global mapping, or backend solver changes in this plan.

Repository policy overrides generic workflow advice: do not create git commits unless the user explicitly authorizes them. Each task therefore ends with a review checkpoint rather than a commit.

## File map

### Create

- `schemas/proto/uw/domain/image.proto` — canonical raw camera frame.
- `tests/l0_contracts/measurement_api_contract_test.cpp` — compile/runtime contract tests for generic camera and optical frontend interfaces.

### Modify

- `schemas/proto/uw/domain/measurement.proto` — optical prior, fused depth, association audit and controlled enums.
- `core/domain/include/uw/domain/domain.hpp` — generated image include, payload traits and validation API.
- `core/domain/src/domain.cpp` — structural validation implementations.
- `core/measurement_api/include/uw/measurement_api/frontend.hpp` — `CameraFrameBundle` and `OpticalDepthFrontend`.
- `core/measurement_api/include/uw/measurement_api/providers.hpp` — `CameraFrameProvider`.
- `tests/l0_contracts/domain_contract_test.cpp` — schema and validation tests.
- `tests/CMakeLists.txt` — measurement API contract-test target.
- `runtime/include/uw/runtime/config.hpp` — optical frontend selector only; calibration remains protobuf-owned.
- `runtime/src/config.cpp` — camera/time-offset rig parsing and strict matrix-size validation.
- `runtime/test/config_test.cpp` — rig and experiment parsing tests.
- `configs/rig/example_auv.yaml` — dual-camera intrinsics, camera extrinsics and time offsets.
- `configs/experiment/synthetic_smoke.yaml` — configured optical frontend name.
- `adapters/holoocean/tests/test_canonical_writer.py` — Python MCAP round trips for new schemas.
- `configs/README.md` — document new rig and frontend fields as contracts, not implemented algorithms.
- `docs/uw-slam-codebase-reference-2026-08-18.md` — record the new implemented contract foundation and unchanged algorithm boundary.

### Generated but not tracked

- `adapters/holoocean/uw_holoocean_adapter/schema_pb2/uw/domain/image_pb2.py`
- regenerated `measurement_pb2.py` and its imports

`cmake/UwProtobuf.cmake` already uses `file(GLOB ... *.proto)`; creating `image.proto` requires no manual CMake source-list change.

---

### Task 1: Add canonical camera and depth wire contracts

**Files:**

- Create: `schemas/proto/uw/domain/image.proto`
- Modify: `schemas/proto/uw/domain/measurement.proto`
- Modify: `core/domain/include/uw/domain/domain.hpp`
- Test: `tests/l0_contracts/domain_contract_test.cpp`

- [ ] **Step 1: Add failing C++ round-trip tests**

Append these tests to `tests/l0_contracts/domain_contract_test.cpp`:

```cpp
TEST(DomainContract, ImageFrameRoundTripsWithCanonicalHeader) {
  uw::domain::ImageFrame frame;
  frame.mutable_header()->mutable_observation_id()->set_value("left_0001");
  frame.mutable_header()->mutable_sensor_id()->set_value("camera_left");
  frame.mutable_header()->mutable_sensor_frame()->set_value("camera_left_link");
  frame.set_width(2);
  frame.set_height(1);
  frame.set_row_stride_bytes(2);
  frame.set_encoding(uw::domain::ImageFrame::IMAGE_ENCODING_MONO8);
  frame.set_pixel_data(std::string{"\x10\x20", 2});
  frame.set_is_rectified(true);
  frame.set_exposure_seconds(0.004);

  std::string bytes;
  ASSERT_TRUE(frame.SerializeToString(&bytes));
  uw::domain::ImageFrame parsed;
  ASSERT_TRUE(parsed.ParseFromString(bytes));
  EXPECT_EQ(parsed.header().observation_id().value(), "left_0001");
  EXPECT_EQ(parsed.pixel_data(), std::string("\x10\x20", 2));
  EXPECT_TRUE(parsed.is_rectified());
}

TEST(DomainContract, OpticalAndFusedDepthPayloadsRoundTripThroughEvidence) {
  uw::domain::OpticalDepthPriorMeasurement prior;
  prior.mutable_reference_camera_frame()->set_value("camera_left_link");
  prior.set_width(2);
  prior.set_height(1);
  prior.add_depth_m(2.0f);
  prior.add_depth_m(3.0f);
  prior.add_variance_m2(0.04f);
  prior.add_variance_m2(0.09f);
  prior.set_valid_mask(std::string{"\x01\x01", 2});
  prior.set_scale_status(uw::domain::OPTICAL_DEPTH_SCALE_STATUS_METRIC);
  prior.set_producer_type("stereo");

  uw::domain::EvidenceId prior_id;
  prior_id.set_value("optical_depth_1");
  uw::domain::ObservationId left_id;
  left_id.set_value("left_0001");
  uw::domain::ObservationId right_id;
  right_id.set_value("right_0001");
  auto prior_evidence = uw::domain::MakeEvidence(
      prior_id, {left_id, right_id}, prior, 1.0, "stereo_depth_frontend_v1");
  ASSERT_TRUE(uw::domain::HasPayload<uw::domain::OpticalDepthPriorMeasurement>(prior_evidence));
  EXPECT_EQ(prior_evidence.source_observations_size(), 2);

  uw::domain::FusedDepthMeasurement fused;
  fused.mutable_reference_camera_frame()->set_value("camera_left_link");
  fused.set_width(1);
  fused.set_height(1);
  fused.add_depth_m(2.1f);
  fused.add_variance_m2(0.01f);
  fused.set_valid_mask(std::string{"\x01", 1});
  fused.set_contribution_mask(std::string{"\x02", 1});
  auto* association = fused.add_associations();
  association->mutable_sonar_evidence_id()->set_value("sonar_cfar_1");
  association->set_status(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_ACCEPTED);
  association->set_has_selected_pixel(true);
  association->set_selected_pixel_index(0);
  association->set_reason(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_NONE);

  uw::domain::EvidenceId fused_id;
  fused_id.set_value("fused_depth_1");
  uw::domain::ObservationId sonar_id;
  sonar_id.set_value("sonar_0001");
  auto fused_evidence = uw::domain::MakeEvidence(
      fused_id, {left_id, right_id, sonar_id}, fused, 0.5,
      "acoustic_optic_depth_fusion_v1");
  ASSERT_TRUE(uw::domain::HasPayload<uw::domain::FusedDepthMeasurement>(fused_evidence));
  EXPECT_EQ(fused_evidence.source_observations_size(), 3);
  EXPECT_EQ(uw::domain::GetPayload<uw::domain::FusedDepthMeasurement>(fused_evidence)
                .associations(0)
                .selected_pixel_index(),
            0u);
}
```

- [ ] **Step 2: Run the target and verify the tests fail to compile**

Run:

```bash
cmake --build build --target uw_l0_domain_contract_test -j"$(nproc)"
```

Expected: compilation fails because `uw::domain::ImageFrame`,
`OpticalDepthPriorMeasurement`, and `FusedDepthMeasurement` do not exist.

- [ ] **Step 3: Create `image.proto`**

Create `schemas/proto/uw/domain/image.proto` with exactly this contract:

```proto
syntax = "proto3";

package uw.domain;

import "uw/domain/observation.proto";

// Canonical raw camera observation. Each physical camera emits its own
// ImageFrame; runtime pairing is reconstructed from capture time and rig
// configuration instead of being permanently baked into the bag.
message ImageFrame {
  ObservationHeader header = 1;
  uint32 width = 2;
  uint32 height = 3;
  uint32 row_stride_bytes = 4;

  enum ImageEncoding {
    IMAGE_ENCODING_UNSPECIFIED = 0;
    IMAGE_ENCODING_MONO8 = 1;
    IMAGE_ENCODING_RGB8 = 2;
    IMAGE_ENCODING_BGR8 = 3;
  }
  ImageEncoding encoding = 5;

  bytes pixel_data = 6;
  bool is_rectified = 7;
  double exposure_seconds = 8;
}
```

- [ ] **Step 4: Add the typed depth and association messages**

In `schemas/proto/uw/domain/measurement.proto`, keep all existing field numbers and messages, then insert these definitions before `MeasurementEvidence`:

```proto
enum OpticalDepthScaleStatus {
  OPTICAL_DEPTH_SCALE_STATUS_UNSPECIFIED = 0;
  OPTICAL_DEPTH_SCALE_STATUS_METRIC = 1;
  OPTICAL_DEPTH_SCALE_STATUS_RELATIVE_SCALE = 2;
  OPTICAL_DEPTH_SCALE_STATUS_UNOBSERVED_SCALE = 3;
}

message OpticalDepthPriorMeasurement {
  FrameId reference_camera_frame = 1;
  uint32 width = 2;
  uint32 height = 3;
  repeated float depth_m = 4 [packed = true];
  repeated float variance_m2 = 5 [packed = true];
  bytes valid_mask = 6;
  OpticalDepthScaleStatus scale_status = 7;
  string producer_type = 8;
}

enum AcousticOpticAssociationStatus {
  ACOUSTIC_OPTIC_ASSOCIATION_STATUS_UNSPECIFIED = 0;
  ACOUSTIC_OPTIC_ASSOCIATION_STATUS_ACCEPTED = 1;
  ACOUSTIC_OPTIC_ASSOCIATION_STATUS_AMBIGUOUS = 2;
  ACOUSTIC_OPTIC_ASSOCIATION_STATUS_CONFLICT = 3;
  ACOUSTIC_OPTIC_ASSOCIATION_STATUS_REJECTED = 4;
}

enum AcousticOpticAssociationReason {
  ACOUSTIC_OPTIC_ASSOCIATION_REASON_UNSPECIFIED = 0;
  ACOUSTIC_OPTIC_ASSOCIATION_REASON_NONE = 1;
  ACOUSTIC_OPTIC_ASSOCIATION_REASON_TIME_DELTA = 2;
  ACOUSTIC_OPTIC_ASSOCIATION_REASON_CALIBRATION = 3;
  ACOUSTIC_OPTIC_ASSOCIATION_REASON_SCALE = 4;
  ACOUSTIC_OPTIC_ASSOCIATION_REASON_NO_CANDIDATE = 5;
  ACOUSTIC_OPTIC_ASSOCIATION_REASON_AMBIGUOUS_MARGIN = 6;
  ACOUSTIC_OPTIC_ASSOCIATION_REASON_CROSS_MODAL_CONFLICT = 7;
  ACOUSTIC_OPTIC_ASSOCIATION_REASON_POSTERIOR_INVALID = 8;
  ACOUSTIC_OPTIC_ASSOCIATION_REASON_VARIANCE_NOT_IMPROVED = 9;
}

message AcousticOpticAssociationRecord {
  EvidenceId sonar_evidence_id = 1;
  AcousticOpticAssociationStatus status = 2;
  repeated uint32 candidate_pixel_indices = 3 [packed = true];
  bool has_selected_pixel = 4;
  uint32 selected_pixel_index = 5;
  double best_score = 6;
  double second_best_score = 7;
  double time_delta_seconds = 8;
  double prior_depth_m = 9;
  double posterior_depth_m = 10;
  double prior_variance_m2 = 11;
  double posterior_variance_m2 = 12;
  AcousticOpticAssociationReason reason = 13;
}

enum DepthContribution {
  DEPTH_CONTRIBUTION_INVALID = 0;
  DEPTH_CONTRIBUTION_OPTICAL_ONLY = 1;
  DEPTH_CONTRIBUTION_ACOUSTIC_OPTIC = 2;
}

message FusedDepthMeasurement {
  FrameId reference_camera_frame = 1;
  uint32 width = 2;
  uint32 height = 3;
  repeated float depth_m = 4 [packed = true];
  repeated float variance_m2 = 5 [packed = true];
  bytes valid_mask = 6;
  // One byte per pixel, encoded with DepthContribution values.
  bytes contribution_mask = 7;
  repeated AcousticOpticAssociationRecord associations = 8;
}
```

Add these non-conflicting cases to `MeasurementEvidence.payload`:

```proto
    OpticalDepthPriorMeasurement optical_depth_prior = 16;
    FusedDepthMeasurement fused_depth = 17;
```

Keep the existing placeholder `StereoDepthMeasurement stereo_depth = 13` for wire compatibility in this phase. New code must use `OpticalDepthPriorMeasurement`; removing the placeholder is a later explicit schema-migration decision.

- [ ] **Step 5: Register generated types in the C++ domain facade**

Modify `core/domain/include/uw/domain/domain.hpp`:

```cpp
#include "uw/domain/image.pb.h"
```

Add these traits beside the existing `UW_DOMAIN_DEFINE_PAYLOAD_TRAITS` declarations:

```cpp
UW_DOMAIN_DEFINE_PAYLOAD_TRAITS(OpticalDepthPriorMeasurement, optical_depth_prior);
UW_DOMAIN_DEFINE_PAYLOAD_TRAITS(FusedDepthMeasurement, fused_depth);
```

- [ ] **Step 6: Reconfigure, build, and run the contract test**

Run:

```bash
cmake -S . -B build
cmake --build build --target uw_l0_domain_contract_test -j"$(nproc)"
ctest --test-dir build -R '^uw_l0_domain_contract_test$' --output-on-failure
```

Expected: build succeeds and CTest reports `100% tests passed` for the selected test.

- [ ] **Step 7: Review checkpoint**

Run `git diff -- schemas/proto/uw/domain core/domain tests/l0_contracts/domain_contract_test.cpp` and verify that this task changes only the wire contracts, payload registration, and round-trip tests. Do not commit without explicit authorization.

---

### Task 2: Add strict structural validation for camera and depth grids

**Files:**

- Modify: `core/domain/include/uw/domain/domain.hpp`
- Modify: `core/domain/src/domain.cpp`
- Test: `tests/l0_contracts/domain_contract_test.cpp`

- [ ] **Step 1: Add failing validation tests**

Append:

```cpp
TEST(DomainValidation, AcceptsWellFormedImageAndDepthGrids) {
  uw::domain::ImageFrame image;
  image.set_width(2);
  image.set_height(1);
  image.set_row_stride_bytes(2);
  image.set_encoding(uw::domain::ImageFrame::IMAGE_ENCODING_MONO8);
  image.set_pixel_data(std::string{"\x01\x02", 2});
  EXPECT_TRUE(uw::domain::ValidateImageFrame(image).ok());

  uw::domain::OpticalDepthPriorMeasurement prior;
  prior.set_width(2);
  prior.set_height(1);
  prior.add_depth_m(1.0f);
  prior.add_depth_m(2.0f);
  prior.add_variance_m2(0.01f);
  prior.add_variance_m2(0.04f);
  prior.set_valid_mask(std::string{"\x01\x01", 2});
  prior.set_scale_status(uw::domain::OPTICAL_DEPTH_SCALE_STATUS_METRIC);
  EXPECT_TRUE(uw::domain::ValidateOpticalDepthPrior(prior).ok());
}

TEST(DomainValidation, RejectsPayloadAndGridShapeMismatches) {
  uw::domain::ImageFrame image;
  image.set_width(2);
  image.set_height(2);
  image.set_row_stride_bytes(2);
  image.set_encoding(uw::domain::ImageFrame::IMAGE_ENCODING_MONO8);
  image.set_pixel_data(std::string{"\x01\x02", 2});
  EXPECT_EQ(uw::domain::ValidateImageFrame(image).code,
            uw::domain::ValidationCode::kImagePayloadSizeMismatch);

  uw::domain::OpticalDepthPriorMeasurement prior;
  prior.set_width(2);
  prior.set_height(1);
  prior.add_depth_m(1.0f);
  prior.add_variance_m2(0.01f);
  prior.add_variance_m2(0.01f);
  prior.set_valid_mask(std::string{"\x01\x01", 2});
  prior.set_scale_status(uw::domain::OPTICAL_DEPTH_SCALE_STATUS_METRIC);
  EXPECT_EQ(uw::domain::ValidateOpticalDepthPrior(prior).code,
            uw::domain::ValidationCode::kDepthGridSizeMismatch);
}

TEST(DomainValidation, RejectsInvalidMetricDepthValues) {
  uw::domain::OpticalDepthPriorMeasurement prior;
  prior.set_width(1);
  prior.set_height(1);
  prior.add_depth_m(-1.0f);
  prior.add_variance_m2(0.0f);
  prior.set_valid_mask(std::string{"\x01", 1});
  prior.set_scale_status(uw::domain::OPTICAL_DEPTH_SCALE_STATUS_METRIC);
  EXPECT_EQ(uw::domain::ValidateOpticalDepthPrior(prior).code,
            uw::domain::ValidationCode::kInvalidDepthValue);
}
```

- [ ] **Step 2: Verify the new tests fail to compile**

Run:

```bash
cmake --build build --target uw_l0_domain_contract_test -j"$(nproc)"
```

Expected: compilation fails because `ValidationCode`, `ValidateImageFrame`, and
`ValidateOpticalDepthPrior` are undefined.

- [ ] **Step 3: Declare the validation API**

Add to `core/domain/include/uw/domain/domain.hpp` after timestamp helpers:

```cpp
enum class ValidationCode {
  kOk = 0,
  kMissingDimensions,
  kUnsupportedImageEncoding,
  kInvalidImageStride,
  kImagePayloadSizeMismatch,
  kDepthGridSizeMismatch,
  kDepthMaskSizeMismatch,
  kContributionMaskSizeMismatch,
  kInvalidScaleStatus,
  kInvalidDepthValue,
  kInvalidVarianceValue,
};

struct ValidationResult {
  ValidationCode code = ValidationCode::kOk;
  std::string message;
  bool ok() const { return code == ValidationCode::kOk; }
};

ValidationResult ValidateImageFrame(const ImageFrame& frame);
ValidationResult ValidateOpticalDepthPrior(const OpticalDepthPriorMeasurement& prior);
ValidationResult ValidateFusedDepth(const FusedDepthMeasurement& fused);
```

- [ ] **Step 4: Implement the validators**

Add to `core/domain/src/domain.cpp`:

```cpp
namespace {

ValidationResult Invalid(ValidationCode code, std::string message) {
  return ValidationResult{code, std::move(message)};
}

std::size_t PixelCount(uint32_t width, uint32_t height) {
  return static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
}

ValidationResult ValidateDepthValues(const google::protobuf::RepeatedField<float>& depth,
                                     const google::protobuf::RepeatedField<float>& variance,
                                     const std::string& valid_mask) {
  for (int i = 0; i < depth.size(); ++i) {
    if (static_cast<unsigned char>(valid_mask[static_cast<std::size_t>(i)]) == 0) continue;
    if (!std::isfinite(depth.Get(i)) || depth.Get(i) <= 0.0f) {
      return Invalid(ValidationCode::kInvalidDepthValue,
                     "valid depth values must be finite and positive");
    }
    if (!std::isfinite(variance.Get(i)) || variance.Get(i) <= 0.0f) {
      return Invalid(ValidationCode::kInvalidVarianceValue,
                     "valid variance values must be finite and positive");
    }
  }
  return {};
}

}  // namespace

ValidationResult ValidateImageFrame(const ImageFrame& frame) {
  if (frame.width() == 0 || frame.height() == 0) {
    return Invalid(ValidationCode::kMissingDimensions, "image dimensions must be non-zero");
  }
  uint32_t bytes_per_pixel = 0;
  switch (frame.encoding()) {
    case ImageFrame::IMAGE_ENCODING_MONO8:
      bytes_per_pixel = 1;
      break;
    case ImageFrame::IMAGE_ENCODING_RGB8:
    case ImageFrame::IMAGE_ENCODING_BGR8:
      bytes_per_pixel = 3;
      break;
    default:
      return Invalid(ValidationCode::kUnsupportedImageEncoding, "unsupported image encoding");
  }
  const uint32_t minimum_stride = frame.width() * bytes_per_pixel;
  if (frame.row_stride_bytes() < minimum_stride) {
    return Invalid(ValidationCode::kInvalidImageStride, "image stride is smaller than one row");
  }
  const std::size_t expected =
      static_cast<std::size_t>(frame.height()) * frame.row_stride_bytes();
  if (frame.pixel_data().size() != expected) {
    return Invalid(ValidationCode::kImagePayloadSizeMismatch, "image payload size does not match shape");
  }
  return {};
}

ValidationResult ValidateOpticalDepthPrior(const OpticalDepthPriorMeasurement& prior) {
  const std::size_t pixels = PixelCount(prior.width(), prior.height());
  if (pixels == 0) {
    return Invalid(ValidationCode::kMissingDimensions, "depth dimensions must be non-zero");
  }
  if (prior.depth_m_size() != static_cast<int>(pixels) ||
      prior.variance_m2_size() != static_cast<int>(pixels)) {
    return Invalid(ValidationCode::kDepthGridSizeMismatch, "depth and variance must match shape");
  }
  if (prior.valid_mask().size() != pixels) {
    return Invalid(ValidationCode::kDepthMaskSizeMismatch, "valid mask must match shape");
  }
  if (prior.scale_status() == OPTICAL_DEPTH_SCALE_STATUS_UNSPECIFIED) {
    return Invalid(ValidationCode::kInvalidScaleStatus, "optical scale status must be explicit");
  }
  return ValidateDepthValues(prior.depth_m(), prior.variance_m2(), prior.valid_mask());
}

ValidationResult ValidateFusedDepth(const FusedDepthMeasurement& fused) {
  const std::size_t pixels = PixelCount(fused.width(), fused.height());
  if (pixels == 0) {
    return Invalid(ValidationCode::kMissingDimensions, "fused depth dimensions must be non-zero");
  }
  if (fused.depth_m_size() != static_cast<int>(pixels) ||
      fused.variance_m2_size() != static_cast<int>(pixels)) {
    return Invalid(ValidationCode::kDepthGridSizeMismatch, "fused depth and variance must match shape");
  }
  if (fused.valid_mask().size() != pixels) {
    return Invalid(ValidationCode::kDepthMaskSizeMismatch, "fused valid mask must match shape");
  }
  if (fused.contribution_mask().size() != pixels) {
    return Invalid(ValidationCode::kContributionMaskSizeMismatch,
                   "contribution mask must match shape");
  }
  return ValidateDepthValues(fused.depth_m(), fused.variance_m2(), fused.valid_mask());
}
```

Add these explicit includes to `domain.cpp`:

```cpp
#include <utility>

#include <google/protobuf/repeated_field.h>
```

- [ ] **Step 5: Add one fused-grid validation assertion**

Add to `RejectsPayloadAndGridShapeMismatches`:

```cpp
  uw::domain::FusedDepthMeasurement fused;
  fused.set_width(1);
  fused.set_height(1);
  fused.add_depth_m(1.0f);
  fused.add_variance_m2(0.01f);
  fused.set_valid_mask(std::string{"\x01", 1});
  EXPECT_EQ(uw::domain::ValidateFusedDepth(fused).code,
            uw::domain::ValidationCode::kContributionMaskSizeMismatch);
```

- [ ] **Step 6: Build and run the L0 contract test**

Run:

```bash
cmake --build build --target uw_l0_domain_contract_test -j"$(nproc)"
ctest --test-dir build -R '^uw_l0_domain_contract_test$' --output-on-failure
```

Expected: selected test passes with no validation assertion failures.

- [ ] **Step 7: Review checkpoint**

Verify that invalid pixels are ignored only after the mask shape has been validated, and valid pixels require finite positive depth and variance. Do not commit without explicit authorization.

---

### Task 3: Add generic camera provider and optical frontend contracts

**Files:**

- Modify: `core/measurement_api/include/uw/measurement_api/frontend.hpp`
- Modify: `core/measurement_api/include/uw/measurement_api/providers.hpp`
- Create: `tests/l0_contracts/measurement_api_contract_test.cpp`
- Modify: `tests/CMakeLists.txt`

- [ ] **Step 1: Write a failing interface contract test**

Create `tests/l0_contracts/measurement_api_contract_test.cpp`:

```cpp
#include <optional>
#include <utility>

#include <gtest/gtest.h>

#include "uw/measurement_api/frontend.hpp"
#include "uw/measurement_api/providers.hpp"

namespace {

class FakeCameraFrameProvider final : public uw::measurement_api::CameraFrameProvider {
 public:
  explicit FakeCameraFrameProvider(uw::domain::ImageFrame frame) : frame_(std::move(frame)) {}

  std::optional<uw::domain::ImageFrame> PollImageFrame() override {
    if (!frame_.has_value()) return std::nullopt;
    auto result = std::move(frame_);
    frame_.reset();
    return result;
  }

  uw::domain::HealthReport Health() const override { return {}; }

 private:
  std::optional<uw::domain::ImageFrame> frame_;
};

class FakeMetricOpticalFrontend final : public uw::measurement_api::OpticalDepthFrontend {
 public:
  std::optional<uw::domain::MeasurementEvidence> Process(
      const uw::measurement_api::CameraFrameBundle& bundle,
      const uw::domain::RigCalibrationSnapshot&) override {
    uw::domain::OpticalDepthPriorMeasurement prior;
    *prior.mutable_reference_camera_frame() = bundle.primary.header().sensor_frame();
    prior.set_width(1);
    prior.set_height(1);
    prior.add_depth_m(bundle.secondary.has_value() ? 2.0f : 3.0f);
    prior.add_variance_m2(0.01f);
    prior.set_valid_mask(std::string{"\x01", 1});
    prior.set_scale_status(uw::domain::OPTICAL_DEPTH_SCALE_STATUS_METRIC);
    prior.set_producer_type(bundle.secondary.has_value() ? "fake_stereo" : "fake_monocular_metric");
    uw::domain::EvidenceId id;
    id.set_value("fake_optical_depth");
    return uw::domain::MakeEvidence(id, {}, prior, 1.0, "fake_metric_v1");
  }

  uw::domain::HealthReport Health() const override { return {}; }
};

}  // namespace

TEST(MeasurementApiContract, CameraProviderPollsCanonicalImageFrame) {
  uw::domain::ImageFrame frame;
  frame.mutable_header()->mutable_observation_id()->set_value("camera_1");
  FakeCameraFrameProvider provider(frame);
  ASSERT_TRUE(provider.PollImageFrame().has_value());
  EXPECT_FALSE(provider.PollImageFrame().has_value());
}

TEST(MeasurementApiContract, OpticalFrontendDoesNotRequireStereoAtInterfaceBoundary) {
  uw::measurement_api::CameraFrameBundle bundle;
  bundle.primary.mutable_header()->mutable_sensor_frame()->set_value("camera_left_link");
  FakeMetricOpticalFrontend frontend;
  const auto evidence = frontend.Process(bundle, {});
  ASSERT_TRUE(evidence.has_value());
  const auto& prior =
      uw::domain::GetPayload<uw::domain::OpticalDepthPriorMeasurement>(*evidence);
  EXPECT_EQ(prior.producer_type(), "fake_monocular_metric");
  EXPECT_EQ(prior.scale_status(), uw::domain::OPTICAL_DEPTH_SCALE_STATUS_METRIC);
}
```

- [ ] **Step 2: Register the failing test target**

Append to `tests/CMakeLists.txt`:

```cmake
add_executable(uw_l0_measurement_api_contract_test
  l0_contracts/measurement_api_contract_test.cpp
)
target_link_libraries(uw_l0_measurement_api_contract_test PRIVATE
  uw_measurement_api GTest::gtest GTest::gtest_main
)
add_test(NAME uw_l0_measurement_api_contract_test
  COMMAND uw_l0_measurement_api_contract_test
)
```

- [ ] **Step 3: Reconfigure and verify the test fails to compile**

Run:

```bash
cmake -S . -B build
cmake --build build --target uw_l0_measurement_api_contract_test -j"$(nproc)"
```

Expected: compilation fails because `CameraFrameProvider`, `CameraFrameBundle`, and
`OpticalDepthFrontend` do not exist.

- [ ] **Step 4: Add the generic optical frontend interface**

Add `#include <optional>` to `frontend.hpp`, then add:

```cpp
struct CameraFrameBundle {
  uw::domain::ImageFrame primary;
  std::optional<uw::domain::ImageFrame> secondary;
};

class OpticalDepthFrontend {
 public:
  virtual ~OpticalDepthFrontend() = default;
  virtual std::optional<uw::domain::MeasurementEvidence> Process(
      const CameraFrameBundle& bundle,
      const uw::domain::RigCalibrationSnapshot& calibration) = 0;
  virtual uw::domain::HealthReport Health() const = 0;
};
```

Place this beside, not inside, `SonarFrontend`. Do not add stereo-specific methods.

- [ ] **Step 5: Add the camera provider interface**

Add to `providers.hpp`:

```cpp
class CameraFrameProvider {
 public:
  virtual ~CameraFrameProvider() = default;
  virtual std::optional<uw::domain::ImageFrame> PollImageFrame() = 0;
  virtual uw::domain::HealthReport Health() const = 0;
};
```

The provider emits independent raw frames; runtime pairing belongs to the later synchronizer plan.

- [ ] **Step 6: Build and run the interface contract test**

Run:

```bash
cmake --build build --target uw_l0_measurement_api_contract_test -j"$(nproc)"
ctest --test-dir build -R '^uw_l0_measurement_api_contract_test$' --output-on-failure
```

Expected: selected test passes, including the fake metric-monocular path with no secondary frame.

- [ ] **Step 7: Review checkpoint**

Confirm that no ROS, OpenCV, HoloOcean, stereo baseline, disparity, or fusion algorithm types leaked into `core/measurement_api`. Do not commit without explicit authorization.

---

### Task 4: Extend the example rig and parse camera calibration/time offsets

**Files:**

- Modify: `configs/rig/example_auv.yaml`
- Modify: `runtime/src/config.cpp`
- Test: `runtime/test/config_test.cpp`

- [ ] **Step 1: Strengthen the existing rig parsing test**

Replace the current frame-tree assertions in `LoadsExperimentConfigWithAllThreeLayers` with:

```cpp
  EXPECT_EQ(config.rig.calibration_version().value(), "example_auv_v2");
  ASSERT_EQ(config.rig.frame_tree_size(), 4);
  EXPECT_EQ(config.rig.frame_tree(1).child_frame().value(), "camera_left_link");
  EXPECT_EQ(config.rig.frame_tree(2).child_frame().value(), "camera_right_link");
  EXPECT_EQ(config.rig.frame_tree(3).child_frame().value(), "sonar_link");

  ASSERT_EQ(config.rig.cameras_size(), 2);
  EXPECT_EQ(config.rig.cameras(0).sensor_id().value(), "camera_left");
  EXPECT_EQ(config.rig.cameras(0).width(), 640u);
  ASSERT_EQ(config.rig.cameras(0).k_matrix_row_major_size(), 9);
  EXPECT_DOUBLE_EQ(config.rig.cameras(0).k_matrix_row_major(0), 420.0);
  EXPECT_EQ(config.rig.cameras(1).sensor_id().value(), "camera_right");

  ASSERT_EQ(config.rig.sonar_beam_models_size(), 1);
  EXPECT_TRUE(config.rig.sonar_beam_models(0).sonar_enabled());
  EXPECT_DOUBLE_EQ(config.rig.time_offset_seconds().at("camera_left"), 0.0);
  EXPECT_DOUBLE_EQ(config.rig.time_offset_seconds().at("camera_right"), 0.0);
  EXPECT_DOUBLE_EQ(config.rig.time_offset_seconds().at("sonar0"), 0.0);
```

- [ ] **Step 2: Verify the config test fails**

Run:

```bash
cmake --build build --target uw_runtime_config_test -j"$(nproc)"
ctest --test-dir build -R '^uw_runtime_config_test$' --output-on-failure
```

Expected: test fails because the current YAML contains no camera calibration/time offsets and the loader does not parse them.

- [ ] **Step 3: Replace the example rig with a complete synthetic stereo rig**

Update `configs/rig/example_auv.yaml` to retain the IMU, sonar and depth sections while changing/adding these exact values:

```yaml
calibration_version: "example_auv_v2"

frame_tree:
  - parent_frame: base_link
    child_frame: imu_link
    transform_row_major:
      [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]
  - parent_frame: base_link
    child_frame: camera_left_link
    transform_row_major:
      [1,0,0,0.15, 0,1,0,0.06, 0,0,1,0, 0,0,0,1]
  - parent_frame: base_link
    child_frame: camera_right_link
    transform_row_major:
      [1,0,0,0.15, 0,1,0,-0.06, 0,0,1,0, 0,0,0,1]
  - parent_frame: base_link
    child_frame: sonar_link
    transform_row_major:
      [1,0,0,0.1, 0,1,0,0, 0,0,1,-0.05, 0,0,0,1]

cameras:
  - sensor_id: camera_left
    width: 640
    height: 480
    k_matrix_row_major: [420,0,320, 0,420,240, 0,0,1]
    distortion: [0,0,0,0]
    distortion_model: plumb_bob
  - sensor_id: camera_right
    width: 640
    height: 480
    k_matrix_row_major: [420,0,320, 0,420,240, 0,0,1]
    distortion: [0,0,0,0]
    distortion_model: plumb_bob

time_offset_seconds:
  camera_left: 0.0
  camera_right: 0.0
  sonar0: 0.0
  imu0: 0.0
  depth0: 0.0
```

Keep the existing `imu_noise`, `sonar_beam_models`, and `depth_models` blocks after these additions.

- [ ] **Step 4: Parse camera intrinsics and time offsets**

Add these blocks to `LoadRigConfig` in `runtime/src/config.cpp`, after frame-tree parsing and before sensor-specific noise models:

```cpp
  if (root["cameras"]) {
    for (const auto& camera_node : root["cameras"]) {
      auto* camera = snapshot.add_cameras();
      camera->mutable_sensor_id()->set_value(camera_node["sensor_id"].as<std::string>());
      camera->set_width(camera_node["width"].as<uint32_t>());
      camera->set_height(camera_node["height"].as<uint32_t>());
      for (const auto& value : camera_node["k_matrix_row_major"]) {
        camera->add_k_matrix_row_major(value.as<double>());
      }
      if (camera_node["distortion"]) {
        for (const auto& value : camera_node["distortion"]) {
          camera->add_distortion(value.as<double>());
        }
      }
      camera->set_distortion_model(
          GetOr<std::string>(camera_node, "distortion_model", std::string("plumb_bob")));
    }
  }

  if (root["time_offset_seconds"]) {
    for (const auto& entry : root["time_offset_seconds"]) {
      (*snapshot.mutable_time_offset_seconds())[entry.first.as<std::string>()] =
          entry.second.as<double>();
    }
  }
```

- [ ] **Step 5: Run the config test**

Run:

```bash
cmake --build build --target uw_runtime_config_test -j"$(nproc)"
ctest --test-dir build -R '^uw_runtime_config_test$' --output-on-failure
```

Expected: selected config test passes and loads two cameras, four frame edges, and five time offsets.

- [ ] **Step 6: Review checkpoint**

Confirm that `RigCalibrationSnapshot` remains the only typed calibration representation and no new runtime camera-calibration struct was introduced. Do not commit without explicit authorization.

---

### Task 5: Reject malformed calibration matrices at the YAML boundary

**Files:**

- Modify: `runtime/src/config.cpp`
- Test: `runtime/test/config_test.cpp`

- [ ] **Step 1: Add failing malformed-rig tests**

Append to `runtime/test/config_test.cpp`:

```cpp
TEST(Config, RejectsRigTransformThatIsNotFourByFour) {
  const auto path = std::filesystem::temp_directory_path() / "uw_bad_transform_rig.yaml";
  {
    std::ofstream out(path);
    out << "calibration_version: bad\n"
           "frame_tree:\n"
           "  - parent_frame: base_link\n"
           "    child_frame: camera_left_link\n"
           "    transform_row_major: [1, 0, 0]\n";
  }
  EXPECT_THROW(uw::runtime::LoadRigConfig(path.string()), std::runtime_error);
  std::remove(path.string().c_str());
}

TEST(Config, RejectsCameraIntrinsicMatrixThatIsNotThreeByThree) {
  const auto path = std::filesystem::temp_directory_path() / "uw_bad_camera_rig.yaml";
  {
    std::ofstream out(path);
    out << "calibration_version: bad\n"
           "cameras:\n"
           "  - sensor_id: camera_left\n"
           "    width: 640\n"
           "    height: 480\n"
           "    k_matrix_row_major: [420, 0, 320]\n";
  }
  EXPECT_THROW(uw::runtime::LoadRigConfig(path.string()), std::runtime_error);
  std::remove(path.string().c_str());
}
```

- [ ] **Step 2: Verify both tests fail**

Run:

```bash
cmake --build build --target uw_runtime_config_test -j"$(nproc)"
ctest --test-dir build -R '^uw_runtime_config_test$' --output-on-failure
```

Expected: the two new `EXPECT_THROW` assertions fail because malformed sequences are currently accepted.

- [ ] **Step 3: Add a strict sequence-length helper**

Add in the anonymous namespace of `runtime/src/config.cpp`:

```cpp
void RequireSequenceLength(const YAML::Node& node, const char* field,
                           std::size_t expected, const std::string& path) {
  if (!node[field] || !node[field].IsSequence() || node[field].size() != expected) {
    throw std::runtime_error(std::string(field) + " must contain exactly " +
                             std::to_string(expected) + " values: " + path);
  }
}
```

- [ ] **Step 4: Validate before copying matrices**

Before each frame transform loop, add:

```cpp
      RequireSequenceLength(edge_node, "transform_row_major", 16, path);
```

Before each camera K-matrix loop, add:

```cpp
      RequireSequenceLength(camera_node, "k_matrix_row_major", 9, path);
      if (camera_node["width"].as<uint32_t>() == 0 ||
          camera_node["height"].as<uint32_t>() == 0) {
        throw std::runtime_error("camera width/height must be non-zero: " + path);
      }
```

- [ ] **Step 5: Run the config tests**

Run:

```bash
cmake --build build --target uw_runtime_config_test -j"$(nproc)"
ctest --test-dir build -R '^uw_runtime_config_test$' --output-on-failure
```

Expected: all config tests pass; malformed transforms and K matrices throw `std::runtime_error`.

- [ ] **Step 6: Review checkpoint**

Inspect exception messages and verify they include both the invalid field and source YAML path. Do not commit without explicit authorization.

---

### Task 6: Add the optical frontend selector without claiming an implementation

**Files:**

- Modify: `runtime/include/uw/runtime/config.hpp`
- Modify: `runtime/src/config.cpp`
- Modify: `runtime/test/config_test.cpp`
- Modify: `configs/experiment/synthetic_smoke.yaml`

- [ ] **Step 1: Add a failing selector assertion**

In `LoadsExperimentConfigWithAllThreeLayers`, add:

```cpp
  EXPECT_EQ(config.optical_frontend, "stereo_depth_frontend_v1");
```

- [ ] **Step 2: Verify the config test fails to compile**

Run:

```bash
cmake --build build --target uw_runtime_config_test -j"$(nproc)"
```

Expected: compilation fails because `ExperimentConfig::optical_frontend` is undefined.

- [ ] **Step 3: Add the selector field and parse it**

Add to `ExperimentConfig` in `runtime/include/uw/runtime/config.hpp`:

```cpp
  std::string optical_frontend = "stereo_depth_frontend_v1";
```

Add to `LoadExperimentConfig` immediately before sonar frontend parsing:

```cpp
  if (root["frontends"] && root["frontends"]["optical"]) {
    config.optical_frontend = root["frontends"]["optical"].as<std::string>();
  }
```

Add to `configs/experiment/synthetic_smoke.yaml`:

```yaml
frontends:
  optical: stereo_depth_frontend_v1
  sonar: sonar_cfar_frontend_v1
```

Replace the existing `frontends` block rather than creating a second one.

- [ ] **Step 4: Run the config tests**

Run:

```bash
cmake --build build --target uw_runtime_config_test -j"$(nproc)"
ctest --test-dir build -R '^uw_runtime_config_test$' --output-on-failure
```

Expected: config test passes and reads both optical and sonar selector names.

- [ ] **Step 5: Verify the selector remains non-dispatching in this phase**

Run:

```bash
rg -n 'optical_frontend' runtime configs apps
```

Expected: matches occur only in config declaration/loading/test/YAML; no app constructs a stereo frontend yet.

- [ ] **Step 6: Review checkpoint**

Confirm documentation and logs call this a selector contract, not an implemented algorithm switch. Do not commit without explicit authorization.

---

### Task 7: Regenerate Python bindings and test MCAP round trips

**Files:**

- Modify: `adapters/holoocean/tests/test_canonical_writer.py`
- Generated/ignored: `adapters/holoocean/uw_holoocean_adapter/schema_pb2/uw/domain/*_pb2.py`

- [ ] **Step 1: Add Python imports and failing round-trip tests**

At the top of `test_canonical_writer.py`, replace the protobuf import with:

```python
from uw.domain import health_pb2, image_pb2, measurement_pb2  # noqa: E402
```

Append:

```python
def test_round_trips_image_frame_through_canonical_mcap():
    with tempfile.TemporaryDirectory() as tmp_dir:
        path = str(Path(tmp_dir) / "camera.mcap")
        frame = image_pb2.ImageFrame()
        frame.header.observation_id.value = "left_0001"
        frame.header.sensor_id.value = "camera_left"
        frame.header.sensor_frame.value = "camera_left_link"
        frame.width = 2
        frame.height = 1
        frame.row_stride_bytes = 2
        frame.encoding = image_pb2.ImageFrame.IMAGE_ENCODING_MONO8
        frame.pixel_data = bytes([16, 32])
        frame.is_rectified = True

        with CanonicalMcapWriter(path) as writer:
            writer.write_message("/raw/camera/left", 1_000_000, frame)

        messages = list(
            read_canonical_messages(path, "/raw/camera/left", image_pb2.ImageFrame)
        )
        assert len(messages) == 1
        assert messages[0][1].pixel_data == bytes([16, 32])


def test_round_trips_optical_depth_evidence_through_canonical_mcap():
    with tempfile.TemporaryDirectory() as tmp_dir:
        path = str(Path(tmp_dir) / "depth.mcap")
        evidence = measurement_pb2.MeasurementEvidence()
        evidence.evidence_id.value = "optical_depth_1"
        evidence.algorithm_version = "stereo_depth_frontend_v1"
        evidence.optical_depth_prior.reference_camera_frame.value = "camera_left_link"
        evidence.optical_depth_prior.width = 1
        evidence.optical_depth_prior.height = 1
        evidence.optical_depth_prior.depth_m.append(2.0)
        evidence.optical_depth_prior.variance_m2.append(0.04)
        evidence.optical_depth_prior.valid_mask = bytes([1])
        evidence.optical_depth_prior.scale_status = (
            measurement_pb2.OPTICAL_DEPTH_SCALE_STATUS_METRIC
        )
        evidence.optical_depth_prior.producer_type = "stereo"

        with CanonicalMcapWriter(path) as writer:
            writer.write_message("/evidence/optical_depth", 1_000_000, evidence)

        messages = list(
            read_canonical_messages(
                path, "/evidence/optical_depth", measurement_pb2.MeasurementEvidence
            )
        )
        assert len(messages) == 1
        assert messages[0][1].WhichOneof("payload") == "optical_depth_prior"
        assert messages[0][1].optical_depth_prior.depth_m[0] == 2.0
```

- [ ] **Step 2: Verify stale bindings fail**

Run:

```bash
cd adapters/holoocean
python -m pytest tests/test_canonical_writer.py -q
```

Expected: import or attribute failure because `image_pb2` and the new measurement fields have not been generated in the active environment.

- [ ] **Step 3: Regenerate Python protobuf bindings**

From the repository root, run:

```bash
tools/codegen/gen_py.sh
```

Expected: prints `OK: generated Python bindings` and creates ignored `image_pb2.py` plus updated measurement bindings.

- [ ] **Step 4: Run the Python MCAP tests**

Run:

```bash
cd adapters/holoocean
python -m pytest tests/test_canonical_writer.py -q
```

Expected: all tests in `test_canonical_writer.py` pass, including camera and optical-depth round trips.

- [ ] **Step 5: Review checkpoint**

Run `git status --short` and verify generated `*_pb2.py` files remain ignored; only the Python test is tracked. Do not commit without explicit authorization.

---

### Task 8: Document the implemented foundation without overstating capability

**Files:**

- Modify: `configs/README.md`
- Modify: `docs/uw-slam-codebase-reference-2026-08-18.md`

- [ ] **Step 1: Update the config documentation**

Add a `## 声光前端契约字段` section to `configs/README.md` with this content:

```markdown
## 声光前端契约字段

`rig/*.yaml` 的 `cameras`、camera/sonar `frame_tree` 边和
`time_offset_seconds` 已解析进 `RigCalibrationSnapshot`。时间偏移采用
`t_reference = t_sensor_capture + time_offset_seconds[sensor_id]` 的符号约定。

`experiment/*.yaml` 的 `frontends.optical` 已被解析，但当前阶段只建立配置和接口契约；
`stereo_depth_frontend_v1` 尚未实现，也没有被 `replay_demo` 动态构造。实际算法接线完成前，
不能把该配置字段描述成可切换的运行能力。
```

- [ ] **Step 2: Update the codebase reference current-fact sections**

In the schema inventory, replace this exact existing bullet (it currently bundles
`StereoDepthMeasurement` together with three unrelated placeholder payloads, so a blind
"replace the placeholder statement" edit would incorrectly strip the still-accurate
placeholder framing from the other three):

```markdown
- `VisualTrackMeasurement`/`StereoDepthMeasurement`/`SonarRegistrationMeasurement`/
  `ImuPreintegrationMeasurement`：占位消息，暂无对应的 factor_builder 消费。
```

with:

```markdown
- `VisualTrackMeasurement`/`SonarRegistrationMeasurement`/`ImuPreintegrationMeasurement`：
  占位消息，暂无对应的 factor_builder 消费。
- `ImageFrame`：canonical camera raw observation，包含 header、shape、stride、encoding、
  pixel bytes、rectified flag 和 exposure。
- `OpticalDepthPriorMeasurement`/`FusedDepthMeasurement`：已落地 wire contract、C++ validation
  和 C++/Python round-trip tests；尚无 stereo 或 acoustic-optic fusion 实现。
- `StereoDepthMeasurement`：保留的早期占位 payload，新代码不再以它作为通用 optical
  frontend 输出。
```

Immediately after, update the adjacent `MeasurementEvidence` bullet, which currently reads
"然后一个覆盖上述 7 种 payload 的 `oneof`" — change `7` to `9` to account for the two new
oneof cases (`optical_depth_prior`, `fused_depth`) added in Task 1.

In the measurement API section, record `CameraFrameProvider`, `CameraFrameBundle`, and
`OpticalDepthFrontend`, explicitly stating that only fake contract-test implementations exist.

- [ ] **Step 3: Scan for contradictory capability claims**

Run:

```bash
rg -n 'camera|optical|stereo|声光' README.md configs/README.md docs/uw-slam-codebase-reference-2026-08-18.md
```

Expected: every statement distinguishes implemented contracts from missing stereo/fusion algorithms; no line claims that `replay_demo` consumes camera data.

- [ ] **Step 4: Review checkpoint**

Review the documentation diff against the actual test/build results from Tasks 1–7. Do not commit without explicit authorization.

---

### Task 9: Run phase-wide verification

**Files:**

- Verify only; no new implementation file.

- [ ] **Step 1: Configure and build all C++ targets**

Use the repository's configured dependency environment. On the documented conda fallback path:

```bash
export PATH="$HOME/miniconda3/envs/uw_slam_build/bin:$PATH"
cmake -S . -B build -DCMAKE_PREFIX_PATH="$HOME/miniconda3/envs/uw_slam_build"
cmake --build build -j"$(nproc)"
```

Expected: exit code 0; generated `image.pb.cc/.h` compile into `uw_domain_proto`, and both L0 contract targets link.

- [ ] **Step 2: Run the complete C++ test suite**

Run:

```bash
ctest --test-dir build --output-on-failure
```

Expected: `100% tests passed`; no existing sonar, factor, runtime, mapping, replay or evaluation test regresses.

- [ ] **Step 3: Run all HoloOcean adapter tests**

Run:

```bash
cd adapters/holoocean
python -m pytest -q
```

Expected: `100%` of collected Python tests pass, including the two new canonical MCAP tests.

- [ ] **Step 4: Run dependency lint**

From the repository root, run:

```bash
tools/lint/check_no_ros_in_core.sh
```

Expected: exit code 0 and no ROS/HoloOcean/vendor include reported under `core/` or `algorithms/`.

- [ ] **Step 5: Run format and placeholder checks**

Run:

```bash
git diff --check
rg -n '\b(TB[D]|TO[D]O|FIXM[E])\b' \
  schemas/proto/uw/domain/image.proto \
  schemas/proto/uw/domain/measurement.proto \
  core/domain core/measurement_api runtime configs tests \
  adapters/holoocean/tests/test_canonical_writer.py
```

Expected: `git diff --check` exits 0; placeholder scan returns no newly introduced unfinished implementation markers.

- [ ] **Step 6: Confirm the phase boundary**

Run:

```bash
rg -n 'class StereoOpticalDepthFrontend|AcousticOpticDepthFusionFrontend|AcousticOpticSynchronizer' \
  core algorithms runtime apps || true
```

Expected: no concrete stereo, fusion, or synchronizer implementation exists yet. The phase delivers contracts and calibration only.

- [ ] **Step 7: Final review checkpoint**

Run `git status --short`, list every changed tracked file, and compare it with the file map at the top of this plan. Report the verification commands and outputs to the user. Do not commit unless the user explicitly requests it.
