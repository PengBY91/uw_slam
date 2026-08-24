# Acoustic-Optic Online Tracking Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce fresh, source-labelled target tracks and operator-assist overlays from asynchronous stereo, sonar and BlueROV state streams without making dense depth or global SLAM a realtime prerequisite.

**Architecture:** Define a target/track wire contract, extract multiple visual and sonar candidates, pair by corrected capture time, compensate with interpolated vehicle state, and fuse through a deterministic gated tracker. `OnlineAssistPipeline` implements the canonical input port and publishes through a replace-latest sink; local dense depth runs only when its quality and deadline budget pass.

**Tech Stack:** C++17, Protobuf, Eigen, OpenCV, GoogleTest/CTest, yaml-cpp

---

**Test fixture convention:** Every `Make*`, `Test*`, and `Feed*` helper shown below is a test-local helper implemented in the same test file during that task. Builders must populate valid headers, calibration IDs, observation IDs, capture/receive stamps and payload dimensions; each test changes only the field named by the case. `TestRig()` supplies calibrated stereo extrinsics, and `FeedSynchronizedVehicleStereoSonar()` emits 50 Hz vehicle state, 20 Hz stereo and 10 Hz sonar against the injected fake clock.

### Task 1: Define target, track and operator-assist output contracts

**Files:**
- Modify: `schemas/proto/uw/domain/ids.proto`
- Create: `schemas/proto/uw/domain/target.proto`
- Modify: `include/domain/domain.hpp`
- Create: `include/application/assist_output_sink.hpp`
- Test: `tests/contracts/domain_contract_test.cpp`
- Test: `tests/contracts/measurement_api_contract_test.cpp`

- [ ] **Step 1: Write failing contract tests**

```cpp
TEST(DomainContract, TargetTrackPreservesSourcesAgeAndUncertainty) {
  uw::domain::TargetTrack track;
  track.mutable_track_id()->set_value("track_9");
  track.set_class_label("aquaculture_zone");
  track.set_bearing_rad(0.2);
  track.set_range_m(4.0);
  track.add_covariance_2x2_row_major(0.01);
  track.add_covariance_2x2_row_major(0.0);
  track.add_covariance_2x2_row_major(0.0);
  track.add_covariance_2x2_row_major(0.04);
  track.add_sources(uw::domain::ASSIST_SOURCE_VISUAL);
  track.add_sources(uw::domain::ASSIST_SOURCE_SONAR);
  track.add_source_observations()->set_value("cam_1");
  track.add_source_observations()->set_value("sonar_1");
  track.set_status(uw::domain::TARGET_TRACK_STATUS_CONFIRMED);
  EXPECT_EQ(track.sources_size(), 2);
  EXPECT_EQ(track.covariance_2x2_row_major_size(), 4);
}
```

Add a fake `AssistOutputSink` test proving `Publish` receives a complete `OperatorAssistState` without ROS2 or HMI
dependencies. The fake owns its test accessor; the production interface stays publish-only.

- [ ] **Step 2: Build and verify failure**

Run `cmake --build build --target contract_tests -j2`.

Expected: compile failure because target wire types and `AssistOutputSink` do not exist.

- [ ] **Step 3: Add the wire schema and sink interface**

Add `TrackId` to `ids.proto`, then create `target.proto` with this stable shape:

```proto
syntax = "proto3";
package uw.domain;

import "uw/domain/health.proto";
import "uw/domain/ids.proto";
import "uw/domain/time.proto";

enum AssistSource {
  ASSIST_SOURCE_UNSPECIFIED = 0;
  ASSIST_SOURCE_VISUAL = 1;
  ASSIST_SOURCE_SONAR = 2;
  ASSIST_SOURCE_ACOUSTIC_OPTIC = 3;
}

message TargetDetection {
  ObservationId source_observation = 1;
  Stamp capture_time = 2;
  string class_label = 3;
  double confidence = 4;
  double bearing_rad = 5;
  bool has_range = 6;
  double range_m = 7;
  repeated double covariance_2x2_row_major = 8;
  uint32 bbox_x = 9;
  uint32 bbox_y = 10;
  uint32 bbox_width = 11;
  uint32 bbox_height = 12;
  AssistSource source = 13;
  map<string, double> quality_metrics = 14;
  double angular_extent_rad = 15;
  double range_extent_m = 16;
  double intensity_score = 17;
}

enum TargetTrackStatus {
  TARGET_TRACK_STATUS_UNSPECIFIED = 0;
  TARGET_TRACK_STATUS_TENTATIVE = 1;
  TARGET_TRACK_STATUS_CONFIRMED = 2;
  TARGET_TRACK_STATUS_DEGRADED = 3;
  TARGET_TRACK_STATUS_STALE = 4;
}

message TargetTrack {
  TrackId track_id = 1;
  string class_label = 2;
  double class_confidence = 3;
  double bearing_rad = 4;
  double range_m = 5;
  repeated double covariance_2x2_row_major = 6;
  Stamp first_capture_time = 7;
  Stamp last_capture_time = 8;
  Stamp publish_time = 9;
  repeated AssistSource sources = 10;
  repeated ObservationId source_observations = 11;
  TargetTrackStatus status = 12;
}

message TargetTrackSet { repeated TargetTrack tracks = 1; Stamp publish_time = 2; }

message OperatorAssistState {
  TargetTrackSet target_tracks = 1;
  bool has_path_lateral_offset = 2;
  double path_lateral_offset_m = 3;
  double path_offset_sigma_m = 4;
  HealthReport system_health = 5;
  double data_age_ms = 6;
  bool guidance_valid = 7;
  string degradation_reason = 8;
  repeated HealthReport sensor_health = 9;
}
```

Define:

```cpp
class AssistOutputSink {
 public:
  virtual ~AssistOutputSink() = default;
  virtual void Publish(const uw::domain::OperatorAssistState& state) = 0;
};
```

- [ ] **Step 4: Reconfigure and run tests**

```bash
cmake -S . -B build -DUW_BUILD_TESTS=ON
cmake --build build --target contract_tests -j2
ctest --test-dir build -R contract --output-on-failure
```

Expected: contract tests pass.

- [ ] **Step 5: Commit**

```bash
git add schemas/proto/uw/domain/ids.proto schemas/proto/uw/domain/target.proto include/domain/domain.hpp include/application/assist_output_sink.hpp tests/contracts/domain_contract_test.cpp tests/contracts/measurement_api_contract_test.cpp
git commit -m "feat(domain): add operator target track contract"
```

### Task 2: Extract all sonar clusters into target detections and configure CFAR

**Files:**
- Modify: `include/runtime/config.hpp`
- Modify: `src/runtime/config.cpp`
- Modify: `configs/defaults/platform.yaml`
- Modify: `include/frontends/sonar_cfar_frontend.hpp`
- Modify: `src/frontends/sonar_cfar_frontend.cpp`
- Create: `include/frontends/sonar_target_extractor.hpp`
- Create: `src/frontends/sonar_target_extractor.cpp`
- Modify: `cmake/Libraries.cmake`
- Modify: `tests/runtime/config_test.cpp`
- Modify: `tests/frontends/sonar_cfar_frontend_test.cpp`
- Create: `tests/frontends/sonar_target_extractor_test.cpp`
- Modify: `cmake/Tests.cmake`

- [ ] **Step 1: Write failing multi-target and config tests**

Create one sonar frame with two separated CFAR clusters and assert two detections survive. Add health cases for
elevated background noise, excessive false-alarm density, no valid measurements, slow processing and configuration
mismatch:

```cpp
TEST(SonarTargetExtractor, ConvertsEveryAcceptedCluster) {
  SonarCfarFrontend frontend(TestCfarParams());
  const auto hypotheses = frontend.ProcessSonarFrame(MakeTwoClusterSonarFrame());
  const auto detections = SonarTargetExtractor().Extract(hypotheses, MakeTwoClusterSonarFrame());
  ASSERT_EQ(detections.size(), 2u);
  EXPECT_LT(detections[0].bearing_rad(), detections[1].bearing_rad());
  EXPECT_TRUE(detections[0].has_range());
  EXPECT_EQ(detections[0].source(), uw::domain::ASSIST_SOURCE_SONAR);
}
```

Add config assertions for training cells, guard cells, PFA, threshold, DBSCAN epsilon/min samples and default
range/bearing sigma.

- [ ] **Step 2: Build and verify failure**

Run `cmake --build build --target frontends_tests runtime_tests -j2`.

Expected: compile/config failures because extractor and typed sonar config do not exist.

- [ ] **Step 3: Implement typed sonar config and all-candidate conversion**

Add `SonarFrontendConfig` to `PlatformDefaultsConfig` and load this YAML:

```yaml
frontends:
  sonar_cfar:
    training_cells: 16
    guard_cells: 4
    probability_false_alarm: 0.01
    detector_threshold: 50
    dbscan_eps_m: 0.20
    dbscan_min_samples: 2
    default_range_sigma_m: 0.05
    default_bearing_sigma_rad: 0.01
```

`SonarTargetExtractor::Extract` must iterate every `HypothesisSet.candidates`, require
`SonarRangeBearing`, copy source observation/capture time, build a 2×2 diagonal covariance from the candidate
sigmas, preserve angular/range extent, intensity/CFAR score and quality metrics, and sort deterministically by
bearing then range. Do not call `TopCandidate`.

Publish sonar frontend health with background-noise mean, false-alarm density, valid-measurement count, processing
latency and config-hash consistency; threshold violations set `STATUS_SUSPECT` with an exact reason code.

- [ ] **Step 4: Run tests**

```bash
cmake --build build --target frontends_tests runtime_tests -j2
ctest --test-dir build -R 'SonarTargetExtractor|SonarCfarFrontend|Config' --output-on-failure
```

Expected: two-cluster output and config tests pass.

- [ ] **Step 5: Commit**

```bash
git add include/runtime/config.hpp src/runtime/config.cpp configs/defaults/platform.yaml include/frontends/sonar_cfar_frontend.hpp src/frontends/sonar_cfar_frontend.cpp include/frontends/sonar_target_extractor.hpp src/frontends/sonar_target_extractor.cpp cmake/Libraries.cmake tests/runtime/config_test.cpp tests/frontends/sonar_cfar_frontend_test.cpp tests/frontends/sonar_target_extractor_test.cpp cmake/Tests.cmake
git commit -m "feat(frontends): emit configured multi-target sonar detections"
```

### Task 3: Add a simulation visual-assist frontend behind a replaceable interface

**Files:**
- Create: `include/measurement_api/target_frontend.hpp`
- Create: `adapters/opencv/include/adapters/opencv_visual_assist_frontend.hpp`
- Create: `adapters/opencv/src/opencv_visual_assist_frontend.cpp`
- Modify: `cmake/Libraries.cmake`
- Create: `tests/adapters/opencv_visual_assist_frontend_test.cpp`
- Modify: `cmake/Tests.cmake`

- [ ] **Step 1: Write failing target and structure tests**

Use generated RGB fixtures, not an external model. Add cases for low brightness, low contrast/texture, excessive
blur and unavailable depth: quality failures must report exact reasons, while monocular target detection continues
when only stereo depth is unavailable:

```cpp
TEST(OpenCvVisualAssistFrontend, DetectsConfiguredAquacultureColor) {
  auto image = MakeRgbImageWithRectangle(320, 240, 130, 80, 60, 50, {20, 220, 20});
  OpenCvVisualAssistFrontend frontend(TestVisualAssistParams());
  const auto result = frontend.Process(image, std::nullopt, TestCameraIntrinsics());
  ASSERT_EQ(result.targets.size(), 1u);
  EXPECT_EQ(result.targets[0].class_label(), "aquaculture_zone");
  EXPECT_EQ(result.targets[0].source(), uw::domain::ASSIST_SOURCE_VISUAL);
}

TEST(OpenCvVisualAssistFrontend, RejectsStructureOffsetWhenLineSupportIsWeak) {
  OpenCvVisualAssistFrontend frontend(TestVisualAssistParams());
  const auto result = frontend.Process(MakeUniformRgbImage(), std::nullopt, TestCameraIntrinsics());
  EXPECT_FALSE(result.path_lateral_offset_m.has_value());
}
```

- [ ] **Step 2: Build and verify failure**

Run `cmake --build build --target adapters_tests -j2`.

Expected: compile failure because the interface and OpenCV implementation do not exist.

- [ ] **Step 3: Implement the replaceable frontend and deterministic simulator baseline**

Use this interface:

```cpp
struct VisualAssistResult {
  std::vector<uw::domain::TargetDetection> targets;
  std::optional<double> path_lateral_offset_m;
  std::optional<double> path_offset_sigma_m;
  uw::domain::HealthReport health;
};

class VisualAssistFrontend {
 public:
  virtual ~VisualAssistFrontend() = default;
  virtual VisualAssistResult Process(
      const uw::domain::ImageFrame& left_rectified,
      const std::optional<uw::domain::OpticalDepthPriorMeasurement>& depth,
      const uw::domain::CameraIntrinsics& intrinsics) = 0;
};
```

The OpenCV implementation is an explicit simulation baseline: HSV threshold + connected components for the
configured aquaculture color; Canny + probabilistic Hough lines for the structure centerline. Convert target pixel
center to bearing with `atan2(u-cx, fx)`. Only emit range when at least nine valid depth pixels exist inside the
central half of the box; use the median and MAD-derived sigma. Keep thresholds in a params struct and label health
`sim_fixture_detector_v1`, so it cannot be mistaken for the future real task model. Populate brightness, contrast,
Laplacian blur score, texture support and valid-depth ratio in `quality_metrics`; use reason codes `visual_low_light`,
`visual_low_contrast`, `visual_blurred`, and `stereo_depth_unavailable`. The pipeline, which sees both raw camera
streams, assigns `stereo_right_unavailable` when the right stream itself misses three expected periods.

- [ ] **Step 4: Run tests**

```bash
cmake --build build --target adapters_tests -j2
ctest --test-dir build -R OpenCvVisualAssistFrontend --output-on-failure
```

Expected: color target and supported structure line are detected; weak/uniform fixtures fail closed.

- [ ] **Step 5: Commit**

```bash
git add include/measurement_api/target_frontend.hpp adapters/opencv/include/adapters/opencv_visual_assist_frontend.hpp adapters/opencv/src/opencv_visual_assist_frontend.cpp cmake/Libraries.cmake tests/adapters/opencv_visual_assist_frontend_test.cpp cmake/Tests.cmake
git commit -m "feat(frontends): add replaceable visual assist baseline"
```

### Task 4: Buffer and pair asynchronous stereo, sonar and vehicle state by time

**Files:**
- Modify: `schemas/proto/uw/domain/calibration.proto`
- Modify: `include/runtime/config.hpp`
- Modify: `src/runtime/config.cpp`
- Modify: `include/runtime/acoustic_optic_synchronizer.hpp`
- Modify: `src/runtime/acoustic_optic_synchronizer.cpp`
- Create: `include/runtime/acoustic_optic_buffer.hpp`
- Create: `src/runtime/acoustic_optic_buffer.cpp`
- Modify: `cmake/Libraries.cmake`
- Create: `tests/runtime/acoustic_optic_buffer_test.cpp`
- Modify: `tests/runtime/config_test.cpp`
- Modify: `tests/runtime/acoustic_optic_synchronizer_test.cpp`
- Modify: `cmake/Tests.cmake`

- [ ] **Step 1: Write failing mixed-rate, offset and reset tests**

```cpp
TEST(AcousticOpticBuffer, PairsNearestTimeNotObservationId) {
  AcousticOpticBuffer buffer(TestBufferConfig(), TestRig());
  buffer.AddVehicleState(MakeVehicleState(9.95, 0.0));
  buffer.AddVehicleState(MakeVehicleState(10.05, 0.1));
  buffer.AddImage(MakeImage("camera_left", "kf8", 10.01));
  buffer.AddImage(MakeImage("camera_right", "kf8", 10.011));
  const auto bundle = buffer.AddSonar(MakeSonar("tick1001", 10.00));
  ASSERT_TRUE(bundle.has_value());
  EXPECT_EQ(bundle->sonar.header().observation_id().value(), "tick1001");
  EXPECT_EQ(bundle->images.primary.header().observation_id().value(), "kf8");
  EXPECT_LT(bundle->corrected_time_delta_s, 0.05);
}

TEST(AcousticOpticBuffer, CalibrationChangeClearsPendingFrames) {
  AcousticOpticBuffer buffer(TestBufferConfig(), TestRig("rig_v1"));
  buffer.AddImage(MakeImage("camera_left", "a", 1.0));
  buffer.UpdateRig(TestRig("rig_v2"));
  EXPECT_EQ(buffer.Diagnostics().calibration_reset_count, 1u);
  EXPECT_EQ(buffer.Diagnostics().buffered_image_count, 0u);
}
```

Also test left/right delta `2 ms` inclusive, sonar/camera `50 ms` inclusive, required measured zero offsets,
nonzero corrected offsets, same stereo-pair ID as an integrity check, state interpolation, expiry, out-of-order
insertion and no state bracketing. Selection must still be by corrected time; the pair ID must never select a frame.

- [ ] **Step 2: Build and verify failure**

Run `cmake --build build --target runtime_tests -j2`.

Expected: compile failure because `AcousticOpticBuffer` does not exist.

- [ ] **Step 3: Implement bounded time-indexed buffers**

Use this config and output:

```cpp
struct AcousticOpticBufferConfig {
  double max_stereo_delta_s = 0.002;
  double max_sonar_camera_delta_s = 0.050;
  double max_state_bracket_s = 0.100;
  double max_residence_s = 0.500;
  std::size_t max_images_per_camera = 32;
  std::size_t max_sonar_frames = 16;
  std::size_t max_vehicle_states = 128;
};

struct OnlineAcousticOpticBundle {
  uw::measurement_api::CameraFrameBundle images;
  uw::domain::SonarFrame sonar;
  uw::domain::VehicleState interpolated_vehicle_state;
  double corrected_time_delta_s = 0.0;
};
```

Extend `RigCalibrationSnapshot` with `map<string, string> time_offset_provenance = 9`. `LoadRigConfig` must require
both an offset entry and a non-empty provenance entry for each online camera, sonar and vehicle-state sensor; a
measured zero is valid, an absent offset is not. Remove the existing synchronizer behavior that silently treats a
missing sensor offset as zero, and add regression tests for the explicit failure.

Maintain sorted bounded deques by corrected capture time. Pair left/right first, then select the stereo pair nearest
each sonar frame. Interpolate quaternion with normalized slerp, angular velocity/depth linearly. Never compare IDs.
After time selection, require the two camera `observation_id` values to match as a stereo-pair integrity check.
On calibration-version change, clear all buffers and tracker-facing pending state, and diagnostics-record the reset.
Report synchronization candidate/accepted/no-pair/over-window/invalid-time counts plus delta P50/P95/P99/max.

- [ ] **Step 4: Run tests**

```bash
cmake --build build --target runtime_tests -j2
ctest --test-dir build -R AcousticOpticBuffer --output-on-failure
```

Expected: all boundary, offset, interpolation and reset cases pass.

- [ ] **Step 5: Commit**

```bash
git add schemas/proto/uw/domain/calibration.proto include/runtime/config.hpp src/runtime/config.cpp include/runtime/acoustic_optic_buffer.hpp src/runtime/acoustic_optic_buffer.cpp include/runtime/acoustic_optic_synchronizer.hpp src/runtime/acoustic_optic_synchronizer.cpp cmake/Libraries.cmake tests/runtime/config_test.cpp tests/runtime/acoustic_optic_synchronizer_test.cpp tests/runtime/acoustic_optic_buffer_test.cpp cmake/Tests.cmake
git commit -m "feat(runtime): pair asynchronous acoustic optic observations"
```

### Task 5: Associate multiple detections and maintain source-aware tracks

**Files:**
- Modify: `include/runtime/config.hpp`
- Modify: `src/runtime/config.cpp`
- Modify: `configs/defaults/platform.yaml`
- Create: `include/frontends/target_associator.hpp`
- Create: `src/frontends/target_associator.cpp`
- Create: `include/frontends/target_tracker.hpp`
- Create: `src/frontends/target_tracker.cpp`
- Modify: `cmake/Libraries.cmake`
- Modify: `tests/runtime/config_test.cpp`
- Create: `tests/frontends/target_associator_test.cpp`
- Create: `tests/frontends/target_tracker_test.cpp`
- Modify: `cmake/Tests.cmake`

- [ ] **Step 1: Write failing association and lifecycle tests**

Test two targets crossing in bearing, incompatible classes, sonar-only range, visual-only bearing, full rig-extrinsic
projection, confirmation after two hits, unmatched birth, merge/split handling, disappearance, stale at 500 ms, and
source-observation provenance:

```cpp
TEST(TargetTracker, StaleTrackLeavesNormalGuidanceAtFiveHundredMilliseconds) {
  TargetTracker tracker(TestTrackerParams());
  tracker.Update({MakeFusedDetection(0.0, 4.0, "cam0", "sonar0")}, 10.0);
  tracker.Update({MakeFusedDetection(0.01, 4.1, "cam1", "sonar1")}, 10.1);
  ASSERT_EQ(tracker.Tracks(10.1).tracks(0).status(),
            uw::domain::TARGET_TRACK_STATUS_CONFIRMED);
  EXPECT_EQ(tracker.Tracks(10.601).tracks(0).status(),
            uw::domain::TARGET_TRACK_STATUS_STALE);
}
```

- [ ] **Step 2: Build and verify failure**

Run `cmake --build build --target frontends_tests -j2`.

Expected: compile failure because associator and tracker do not exist.

- [ ] **Step 3: Implement deterministic gated association and tracking**

`TargetAssociator` transforms camera rays and sonar bearing/range through the versioned rig into `base_link`, forms
all visual/sonar pairs, and gates corrected time, projected bearing/range Mahalanobis distance, class compatibility,
motion continuity and uncertainty. Sort pair costs and greedily accept lowest non-conflicting pairs. Every accept or
reject records an exact reason plus the configured threshold; association/track parameters come from
`configs/defaults/platform.yaml` and enter the run manifest.

`TargetTracker` uses state `[bearing, range, bearing_rate, range_rate]` with an Eigen 4×4 covariance, constant-
velocity prediction, Joseph-form covariance update, two-hit confirmation, three-miss degraded state and 500 ms
stale state. A visual-only update observes bearing; sonar-only observes bearing/range; fused observes both. Track ID
is monotonically allocated and every update unions supporting observation IDs and source enums. Unmatched detections
create tentative tracks; unmatched tracks predict/degrade; deterministic proximity rules retain the older ID on a
merge and create new tentative IDs after a split rather than silently reusing identity.

- [ ] **Step 4: Run tests**

```bash
cmake --build build --target frontends_tests -j2
ctest --test-dir build -R 'TargetAssociator|TargetTracker' --output-on-failure
```

Expected: multi-target, lifecycle, provenance and covariance tests pass.

- [ ] **Step 5: Commit**

```bash
git add include/runtime/config.hpp src/runtime/config.cpp configs/defaults/platform.yaml include/frontends/target_associator.hpp src/frontends/target_associator.cpp include/frontends/target_tracker.hpp src/frontends/target_tracker.cpp cmake/Libraries.cmake tests/runtime/config_test.cpp tests/frontends/target_associator_test.cpp tests/frontends/target_tracker_test.cpp cmake/Tests.cmake
git commit -m "feat(frontends): fuse detections into target tracks"
```

### Task 6: Assemble `OnlineAssistPipeline` with degradation and dense-budget gates

**Files:**
- Modify: `include/runtime/config.hpp`
- Modify: `src/runtime/config.cpp`
- Modify: `configs/defaults/platform.yaml`
- Create: `include/application/online_assist_pipeline.hpp`
- Create: `src/application/online_assist_pipeline.cpp`
- Create: `include/application/latest_assist_sink.hpp`
- Create: `src/application/latest_assist_sink.cpp`
- Modify: `cmake/Libraries.cmake`
- Create: `tests/application/online_assist_pipeline_test.cpp`
- Modify: `tests/runtime/config_test.cpp`
- Modify: `cmake/Tests.cmake`

- [ ] **Step 1: Write failing end-to-end pipeline tests**

Use fake visual and sonar frontends plus a fake clock. Prove normal fused output, sonar-only, visual-only, stale
vehicle state, dense-branch timeout, output replacement, calibration-change tracker reset and recovery cache clearing:

```cpp
TEST(OnlineAssistPipeline, DenseTimeoutDoesNotBlockFreshTracks) {
  FakeClock clock;
  LatestAssistSink sink;
  OnlineAssistPipeline pipeline(TestPipelineDependencies(clock, sink, DenseBehavior::kTimeout));
  FeedSynchronizedVehicleStereoSonar(pipeline, 10.0);
  const auto latest = sink.Latest();
  ASSERT_TRUE(latest.has_value());
  ASSERT_EQ(latest->target_tracks().tracks_size(), 1);
  EXPECT_LT(latest->data_age_ms(), 250.0);
  EXPECT_TRUE(latest->guidance_valid());
  EXPECT_EQ(latest->system_health().status(), uw::domain::HealthReport::STATUS_SUSPECT);
  EXPECT_EQ(latest->system_health().reason_code(), "dense_deadline_missed");
}
```

- [ ] **Step 2: Build and verify failure**

Run `cmake --build build --target application_tests -j2`.

Expected: compile failure because online pipeline and latest sink do not exist.

- [ ] **Step 3: Implement the online port and replace-latest output**

`OnlineAssistPipeline` implements every `PipelineInputPort` method. Images enter `AcousticOpticBuffer`; sonar
runs CFAR/extractor and triggers bundle association; vehicle state feeds interpolation; DVL and reference truth are
rejected for algorithm use; health messages update modality state. Publish after each accepted target update and
on every health transition. When the buffer reports a calibration-version transition, reset the associator/tracker,
clear any pending dense work, publish `recovering`, and require normal track confirmation under the new version.

Use these dense rules: run only when rectification and stereo quality pass, predicted completion fits a 100 ms
dense budget, and the previous dense task is not running. Otherwise increment a skip/deadline counter and continue
tracks. Sonar range/arc constraints may refine only the corresponding target/structure region and must retain both
raw source observation IDs. Keep `dense.enabled=false` in the competition default until paired scenario/pool evidence
shows better distance, path-offset or task success without violating realtime gates. `LatestAssistSink` stores one
state under a mutex; `Publish` replaces it atomically rather than queueing.

Degradation reasons are exact strings: `visual_unavailable`, `sonar_unavailable`, `stereo_depth_unavailable`,
`vehicle_state_stale`, `all_assist_unavailable`, `recovering`, and `dense_deadline_missed`.
Set `data_age_ms` from the newest contributing capture stamp to the publish clock; set `guidance_valid=false` when
all assist modalities are unavailable or the vehicle state is stale, and copy the active reason into
`degradation_reason`.

- [ ] **Step 4: Run tests**

```bash
cmake --build build --target application_tests -j2
ctest --test-dir build -R OnlineAssistPipeline --output-on-failure
```

Expected: normal, degraded, stale, dense-timeout and recovery tests pass.

- [ ] **Step 5: Commit**

```bash
git add include/runtime/config.hpp src/runtime/config.cpp configs/defaults/platform.yaml include/application/online_assist_pipeline.hpp src/application/online_assist_pipeline.cpp include/application/latest_assist_sink.hpp src/application/latest_assist_sink.cpp cmake/Libraries.cmake tests/runtime/config_test.cpp tests/application/online_assist_pipeline_test.cpp cmake/Tests.cmake
git commit -m "feat(application): assemble online acoustic optic assistance"
```

### Task 7: Render a non-blocking operator overlay

**Files:**
- Create: `adapters/opencv/include/adapters/operator_overlay_renderer.hpp`
- Create: `adapters/opencv/src/operator_overlay_renderer.cpp`
- Modify: `cmake/Libraries.cmake`
- Create: `tests/adapters/operator_overlay_renderer_test.cpp`
- Modify: `cmake/Tests.cmake`

- [ ] **Step 1: Write failing overlay tests**

```cpp
TEST(OperatorOverlayRenderer, MarksSourceAgeAndDegradedState) {
  OperatorOverlayRenderer renderer;
  const auto rendered = renderer.Render(MakeRgbFrame(), MakeSonarViewFrame(), MakeDegradedAssistState());
  ASSERT_TRUE(rendered.has_value());
  EXPECT_EQ(rendered->encoding(), uw::domain::ImageFrame::IMAGE_ENCODING_RGB8);
  EXPECT_NE(rendered->pixel_data(), MakeRgbFrame().pixel_data());
  EXPECT_EQ(renderer.LastLabelsForTest(),
            (std::vector<std::string>{"track_1 SONAR 4.0m c0.82 120ms", "DEGRADED visual_unavailable"}));
}
```

- [ ] **Step 2: Build and verify failure**

Run `cmake --build build --target adapters_tests -j2`.

Expected: compile failure because renderer does not exist.

- [ ] **Step 3: Implement headless rendering**

Accept an independent pilot RGB frame, an optional latest sonar frame and `OperatorAssistState`. Keep the pilot
image as the main panel, draw a labeled polar sonar side panel, and draw track ID, class, bearing, range, confidence,
source and age as projection-safe text/cues. Use green for acoustic-optic, cyan for visual, amber for sonar, red for
stale/unavailable. Draw a top health banner and path-offset arrow. Rendering returns a new RGB8 frame and
never calls `imshow`; ROS2 publishing and the operator display belong to the HoloOcean plan. Expose
`const std::vector<std::string>& LastLabelsForTest() const` as a read-only record of the exact labels drawn by the
last `Render` call, so the headless test checks operator-visible semantics rather than only changed pixels.

- [ ] **Step 4: Run tests and commit**

```bash
cmake --build build --target adapters_tests -j2
ctest --test-dir build -R OperatorOverlayRenderer --output-on-failure
git add adapters/opencv/include/adapters/operator_overlay_renderer.hpp adapters/opencv/src/operator_overlay_renderer.cpp cmake/Libraries.cmake tests/adapters/operator_overlay_renderer_test.cpp cmake/Tests.cmake
git commit -m "feat(hmi): render source-aware operator assistance"
```

### Task 8: Gate the in-process 20/10/50 Hz online assistance slice

**Files:**
- Create: `apps/online_assist_smoke.cpp`
- Modify: `cmake/Applications.cmake`
- Create: `tests/integration/online_assist_smoke_test.sh`
- Modify: `cmake/Tests.cmake`
- Modify: `docs/testing-and-verification-guide-2026-08-20.md`

- [ ] **Step 1: Write the failing gate script**

Require exact report keys:

```bash
output="$($1 --duration-s 5 --camera-hz 20 --sonar-hz 10 --state-hz 50)"
grep -Eq 'fused_tracks=[1-9][0-9]*' <<<"$output"
grep -q 'truth_delivered=0' <<<"$output"
grep -q 'stale_normal_tracks=0' <<<"$output"
grep -q 'queue_capacity_violations=0' <<<"$output"
grep -Eq 'result_age_p95_ms=([0-9]{1,2}|1[0-9]{2}|2[0-4][0-9])(\.[0-9]+)?$' <<<"$output"
```

- [ ] **Step 2: Register/build and verify failure**

Run `cmake --build build --target online_assist_smoke -j2`.

Expected: build failure because app does not exist.

- [ ] **Step 3: Implement normal and dropout profiles**

Feed generated stereo fixtures at 20 Hz, two-cluster sonar at 10 Hz and vehicle state at 50 Hz through
`LiveEventSource → PumpEvents → OnlineAssistPipeline`. Add `--drop-visual-at-s` and `--drop-sonar-at-s`; print
normal, degraded, recovery, age, queue and source counters. Never feed `/gt/state` to the pipeline.

- [ ] **Step 4: Run focused and full verification**

```bash
cmake --build build --target online_assist_smoke -j2
ctest --test-dir build -R integration.online_assist_smoke --output-on-failure
build/bin/online_assist_smoke --duration-s 30 --drop-visual-at-s 10 --camera-hz 20 --sonar-hz 10 --state-hz 50
ctest --test-dir build --output-on-failure
```

Expected: normal run has fused tracks below 250 ms P95; visual dropout switches to sonar-only within three camera
periods; no stale result remains normal; all tests pass.

- [ ] **Step 5: Commit**

```bash
git add apps/online_assist_smoke.cpp cmake/Applications.cmake tests/integration/online_assist_smoke_test.sh cmake/Tests.cmake docs/testing-and-verification-guide-2026-08-20.md
git commit -m "test(application): gate realtime acoustic optic assistance"
```
