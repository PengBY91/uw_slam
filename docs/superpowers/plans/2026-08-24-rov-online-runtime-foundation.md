# ROV Online Runtime Foundation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the canonical live ingestion substrate with semantic validation, bounded priority queues, observable overflow/age statistics, truth isolation and deterministic shutdown.

**Architecture:** Extend the wire contract with a raw `VehicleState`, validate every canonical event before enqueue, and implement `LiveEventSource` as four bounded lanes drained by a weighted priority scheduler. Preserve `EventSource → PumpEvents → PipelineInputPort`, so live and replay share the canonical dispatch boundary without forcing live data into the replay accumulator.

**Tech Stack:** C++17, Protobuf, GoogleTest, CMake, yaml-cpp

---

**Test fixture convention:** Every `Make*Event`, `MakeValid*`, and fake clock/port used below is implemented as a test-local helper in the same test file during that task. Valid builders must populate non-empty sensor/calibration/observation IDs, capture and receive stamps in the declared clock domain, consistent image/sonar payload dimensions, and normalized vehicle attitude; negative tests mutate only the field named by the case.

### Task 1: Add the raw vehicle-state contract and route it canonically

**Files:**
- Create: `schemas/proto/uw/domain/vehicle.proto`
- Modify: `schemas/proto/uw/domain/sonar.proto`
- Modify: `schemas/proto/uw/domain/time.proto`
- Modify: `schemas/proto/uw/domain/observation.proto`
- Modify: `include/domain/domain.hpp`
- Modify: `include/runtime/canonical_topics.hpp`
- Modify: `include/runtime/canonical_event.hpp`
- Modify: `include/application/pipeline_input_port.hpp`
- Modify: `src/application/event_pump.cpp`
- Modify: `include/application/replay_input_accumulator.hpp`
- Modify: `src/application/replay_input_accumulator.cpp`
- Modify: `src/runtime/mcap_event_source.cpp`
- Test: `tests/contracts/domain_contract_test.cpp`
- Test: `tests/runtime/canonical_event_test.cpp`
- Test: `tests/application/event_pump_test.cpp`

- [ ] **Step 1: Write failing domain and dispatch tests**

Add a round-trip test with the exact required fields:

```cpp
TEST(DomainContract, VehicleStateRoundTripsWithCanonicalHeader) {
  uw::domain::VehicleState state;
  state.mutable_header()->mutable_observation_id()->set_value("vehicle_7");
  state.mutable_header()->mutable_sensor_id()->set_value("bluerov_state");
  state.mutable_header()->mutable_sensor_frame()->set_value("base_link");
  state.mutable_header()->mutable_calibration_version()->set_value("rig_v1");
  state.mutable_header()->set_receive_clock_domain(uw::domain::CLOCK_DOMAIN_SYSTEM_MONOTONIC);
  state.add_orientation_xyzw(0.0);
  state.add_orientation_xyzw(0.0);
  state.add_orientation_xyzw(0.0);
  state.add_orientation_xyzw(1.0);
  state.add_angular_velocity_radps(0.1);
  state.add_angular_velocity_radps(0.2);
  state.add_angular_velocity_radps(0.3);
  state.set_depth_m(2.5);
  state.set_attitude_valid(true);
  state.set_depth_valid(true);
  state.set_leak_detected(false);
  state.set_supply_voltage_v(15.8);
  state.set_supply_current_a(8.2);
  state.set_link_quality(0.95);
  state.set_device_health_valid(true);

  std::string bytes;
  ASSERT_TRUE(state.SerializeToString(&bytes));
  uw::domain::VehicleState parsed;
  ASSERT_TRUE(parsed.ParseFromString(bytes));
  EXPECT_EQ(parsed.orientation_xyzw_size(), 4);
  EXPECT_DOUBLE_EQ(parsed.depth_m(), 2.5);
}
```

Add an event-pump test whose fake port increments `vehicle_state_count` in `OnVehicleState`, then assert one
`/raw/vehicle_state` event reaches that method and a `/gt/state` event reaches only `OnReferenceState`.

- [ ] **Step 2: Run the focused tests and verify failure**

Run:

```bash
cmake --build build --target contract_tests application_tests runtime_tests -j2
```

Expected: compile failure because `VehicleState`, `receive_clock_domain`, and `OnVehicleState` do not exist.

- [ ] **Step 3: Add the schema and canonical routing**

Create `vehicle.proto`:

```proto
syntax = "proto3";
package uw.domain;

import "uw/domain/observation.proto";

message VehicleState {
  ObservationHeader header = 1;
  repeated double orientation_xyzw = 2;       // exactly 4, body in reference frame
  repeated double angular_velocity_radps = 3; // exactly 3, body frame
  double depth_m = 4;                         // positive down
  repeated double covariance_7x7_row_major = 5;
  bool attitude_valid = 6;
  bool depth_valid = 7;
  bool leak_detected = 8;
  double supply_voltage_v = 9;
  double supply_current_a = 10;
  double link_quality = 11;                  // normalized [0, 1]
  bool device_health_valid = 12;
}
```

Add `double operating_frequency_hz = 15` to `SonarFrame`; this completes the per-frame acoustic contract together
with its existing gain and sound-speed fields. Extend the contract test to round-trip a nonzero frequency.

Add `CLOCK_DOMAIN_SYSTEM_REALTIME = 4` to `ClockDomain` and add this field to `ObservationHeader`:

```proto
ClockDomain receive_clock_domain = 11;
```

Add `VehicleState` to `domain.hpp`, `CanonicalPayload`, `CanonicalEventKind`, the topic registry under
`/raw/vehicle_state`, `McapEventSource`, and `PumpEvents`. Extend `PipelineInputPort` with:

```cpp
virtual bool OnVehicleState(const uw::runtime::CanonicalEvent& event) = 0;
```

`ReplayInputAccumulator` must retain vehicle states in a new `vehicle_states` vector; `/gt/state` remains in
`reference_states` and must never share the same callback.

- [ ] **Step 4: Regenerate/build and run focused tests**

Run:

```bash
cmake -S . -B build -DUW_BUILD_TESTS=ON
cmake --build build --target contract_tests application_tests runtime_tests -j2
ctest --test-dir build -R 'contract|unit.application|unit.runtime' --output-on-failure
```

Expected: all selected tests pass; the event-pump test proves vehicle state and truth use different methods.

- [ ] **Step 5: Commit**

```bash
git add schemas/proto/uw/domain/vehicle.proto schemas/proto/uw/domain/sonar.proto schemas/proto/uw/domain/time.proto schemas/proto/uw/domain/observation.proto include/domain/domain.hpp include/runtime/canonical_topics.hpp include/runtime/canonical_event.hpp include/application/pipeline_input_port.hpp src/application/event_pump.cpp include/application/replay_input_accumulator.hpp src/application/replay_input_accumulator.cpp src/runtime/mcap_event_source.cpp tests/contracts/domain_contract_test.cpp tests/runtime/canonical_event_test.cpp tests/application/event_pump_test.cpp
git commit -m "feat(domain): add canonical vehicle state input"
```

### Task 2: Reject semantically invalid live events before enqueue

**Files:**
- Create: `include/runtime/canonical_event_validation.hpp`
- Create: `src/runtime/canonical_event_validation.cpp`
- Modify: `include/domain/domain.hpp`
- Modify: `src/domain/domain.cpp`
- Modify: `cmake/Libraries.cmake`
- Create: `tests/runtime/canonical_event_validation_test.cpp`
- Modify: `cmake/Tests.cmake`

- [ ] **Step 1: Write failing validation tests**

Cover an empty calibration version, image payload mismatch, descending sonar azimuth, sonar tensor-size mismatch,
missing/non-finite sonar frequency, non-unit quaternion, missing attitude/depth/device-health validity, invalid link
quality and one fully valid event:

```cpp
TEST(CanonicalEventValidation, RejectsSonarTensorSizeMismatch) {
  auto frame = MakeValidSonarFrame();
  frame.set_intensity_tensor(std::string(3, '\0'));
  uw::runtime::CanonicalEvent event{uw::runtime::kTopicSonarFrame, 10, 1, frame};
  const auto result = uw::runtime::ValidateCanonicalEvent(event);
  EXPECT_EQ(result.code, uw::runtime::CanonicalEventValidationCode::kSonarPayloadSizeMismatch);
}

TEST(CanonicalEventValidation, RejectsNonUnitVehicleQuaternion) {
  auto state = MakeValidVehicleState();
  state.set_orientation_xyzw(3, 2.0);
  uw::runtime::CanonicalEvent event{uw::runtime::kTopicVehicleState, 10, 1, state};
  EXPECT_EQ(uw::runtime::ValidateCanonicalEvent(event).code,
            uw::runtime::CanonicalEventValidationCode::kVehicleQuaternionInvalid);
}
```

- [ ] **Step 2: Run the test and verify failure**

Run:

```bash
cmake --build build --target runtime_tests -j2
```

Expected: compile failure because `ValidateCanonicalEvent` and its result type do not exist.

- [ ] **Step 3: Implement complete event validation**

Define a closed result enum and dispatch by variant:

```cpp
enum class CanonicalEventValidationCode {
  kOk,
  kUnknownTopic,
  kTopicPayloadMismatch,
  kHeaderInvalid,
  kImageInvalid,
  kSonarPayloadSizeMismatch,
  kSonarGeometryInvalid,
  kVehicleVectorSizeInvalid,
  kVehicleQuaternionInvalid,
  kVehicleValueInvalid,
};

struct CanonicalEventValidationResult {
  CanonicalEventValidationCode code = CanonicalEventValidationCode::kOk;
  std::string message;
  bool ok() const { return code == CanonicalEventValidationCode::kOk; }
};
```

`ValidateCanonicalEvent` must first verify registry topic/type agreement, then validate every raw observation
header: non-empty observation/sensor/frame/calibration, non-unspecified capture and receive clock domains, normalized
stamp nanos, and `VALIDITY_OK` or `VALIDITY_DEGRADED`. For sonar, require `num_ranges*num_beams` bytes, exactly
`num_beams` ascending azimuths, `num_ranges` or `num_ranges+1` range bins, finite ordered range/FOV values. For
vehicle state, require sizes 4/3, finite values, quaternion norm within `1e-3`, positive-down finite depth when valid,
finite positive electrical values and link quality in `[0,1]` when device health is valid. Sonar operating frequency,
gain and nominal sound speed must be finite and positive.

- [ ] **Step 4: Run tests**

Run:

```bash
cmake --build build --target runtime_tests -j2
ctest --test-dir build -R unit.runtime.CanonicalEventValidation --output-on-failure
```

Expected: all validation cases pass.

- [ ] **Step 5: Commit**

```bash
git add include/runtime/canonical_event_validation.hpp src/runtime/canonical_event_validation.cpp include/domain/domain.hpp src/domain/domain.cpp cmake/Libraries.cmake tests/runtime/canonical_event_validation_test.cpp cmake/Tests.cmake
git commit -m "feat(runtime): validate canonical live events"
```

### Task 3: Make bounded-queue overflow explicit and observable

**Files:**
- Modify: `include/runtime/bounded_queue.hpp`
- Modify: `tests/runtime/runtime_test.cpp`

- [ ] **Step 1: Write failing queue-result and statistics tests**

```cpp
TEST(BoundedQueue, DropOldestReportsEvictionAndHighWatermark) {
  BoundedQueue<int> queue(2, OverflowPolicy::kDropOldest);
  EXPECT_EQ(queue.Push(1), PushResult::kEnqueued);
  EXPECT_EQ(queue.Push(2), PushResult::kEnqueued);
  EXPECT_EQ(queue.Push(3), PushResult::kDroppedOldestAndEnqueued);
  const auto stats = queue.Stats();
  EXPECT_EQ(stats.enqueued_count, 3u);
  EXPECT_EQ(stats.dropped_oldest_count, 1u);
  EXPECT_EQ(stats.high_watermark, 2u);
}

TEST(BoundedQueue, RejectNeverPretendsToEnqueue) {
  BoundedQueue<int> queue(1, OverflowPolicy::kReject);
  EXPECT_EQ(queue.Push(1), PushResult::kEnqueued);
  EXPECT_EQ(queue.Push(2), PushResult::kRejected);
  EXPECT_EQ(queue.Stats().rejected_count, 1u);
}
```

- [ ] **Step 2: Run and verify failure**

Run `cmake --build build --target runtime_tests -j2`.

Expected: compile failure because `PushResult` and `Stats()` do not exist.

- [ ] **Step 3: Implement result and statistics contracts**

```cpp
enum class PushResult { kEnqueued, kDroppedOldestAndEnqueued, kDroppedNewest, kRejected };

struct QueueStats {
  uint64_t enqueued_count = 0;
  uint64_t dequeued_count = 0;
  uint64_t dropped_oldest_count = 0;
  uint64_t dropped_newest_count = 0;
  uint64_t rejected_count = 0;
  std::size_t current_depth = 0;
  std::size_t high_watermark = 0;
};
```

Reject capacity zero in the constructor. Change `Push` to return `PushResult`; make `kDropNewest` return
`kDroppedNewest`, not success. Increment dequeue count in `TryPop`, and return a locked snapshot from `Stats()`.

- [ ] **Step 4: Run tests**

Run:

```bash
cmake --build build --target runtime_tests -j2
ctest --test-dir build -R unit.runtime.BoundedQueue --output-on-failure
```

Expected: all bounded queue tests pass.

- [ ] **Step 5: Commit**

```bash
git add include/runtime/bounded_queue.hpp tests/runtime/runtime_test.cpp
git commit -m "feat(runtime): expose bounded queue overflow statistics"
```

### Task 4: Implement the four-lane live event source

**Files:**
- Create: `include/runtime/live_event_source.hpp`
- Create: `src/runtime/live_event_source.cpp`
- Modify: `include/runtime/event_source.hpp`
- Modify: `cmake/Libraries.cmake`
- Create: `tests/runtime/live_event_source_test.cpp`
- Modify: `cmake/Tests.cmake`

- [ ] **Step 1: Write failing source tests**

Tests must prove weighted priority, image drop-oldest, vehicle-state reject, semantic rejection, reference-topic
rejection, per-sensor duplicate/out-of-order sequence rejection, sequence-gap accounting, blocked `Run()` wake-up on
`Close()`, and exactly-once delivery for accepted events:

```cpp
TEST(LiveEventSource, RejectsReferenceTruthBeforeAlgorithmConsumer) {
  LiveEventSource source(LiveSourceConfig::ForTest());
  EXPECT_EQ(source.Submit(MakeGroundTruthEvent()), LiveSubmitStatus::kReferenceRejected);
  source.Close();
  int delivered = 0;
  const auto report = source.Run([&](const CanonicalEvent&) { ++delivered; return true; });
  EXPECT_EQ(delivered, 0);
  EXPECT_EQ(report.reference_rejected_count, 1u);
}

TEST(LiveEventSource, CameraOverflowKeepsNewest) {
  auto config = LiveSourceConfig::ForTest();
  config.mapping = {2, OverflowPolicy::kDropOldest};
  LiveEventSource source(config);
  source.Submit(MakeImageEvent(1));
  source.Submit(MakeImageEvent(2));
  EXPECT_EQ(source.Submit(MakeImageEvent(3)), LiveSubmitStatus::kAcceptedAfterDroppingOldest);
  source.Close();
  std::vector<uint64_t> sequences;
  source.Run([&](const CanonicalEvent& e) { sequences.push_back(e.source_sequence); return true; });
  EXPECT_EQ(sequences, (std::vector<uint64_t>{2, 3}));
}
```

- [ ] **Step 2: Run and verify failure**

Run `cmake --build build --target runtime_tests -j2`.

Expected: compile failure because `LiveEventSource` does not exist.

- [ ] **Step 3: Implement live source API and scheduler**

Use this public API:

```cpp
struct LaneQueueConfig { std::size_t capacity; OverflowPolicy overflow_policy; };
struct LiveSourceConfig {
  LaneQueueConfig localization{64, OverflowPolicy::kReject};
  LaneQueueConfig correction{32, OverflowPolicy::kDropOldest};
  LaneQueueConfig mapping{16, OverflowPolicy::kDropOldest};
  LaneQueueConfig evidence{256, OverflowPolicy::kDropOldest};
  static LiveSourceConfig ForTest();
};

enum class LiveSubmitStatus {
  kAccepted,
  kAcceptedAfterDroppingOldest,
  kDroppedNewest,
  kOverflowRejected,
  kSemanticRejected,
  kDuplicateOrOutOfOrderRejected,
  kReferenceRejected,
  kClosed,
};

class LiveEventSource final : public EventSource {
 public:
  explicit LiveEventSource(LiveSourceConfig config);
  LiveSubmitStatus Submit(CanonicalEvent event);
  void Close();
  EventSourceReport Run(const EventConsumer& consumer) override;
  LiveSourceStats Stats() const;
};
```

Map vehicle state/IMU to localization, sonar to correction, images to mapping, and diagnostics/evidence to evidence.
Use a repeated schedule `{L,L,L,L,L,L,L,L,C,C,C,C,M,M,E}` so low lanes make progress without outranking
localization. `Submit` must call `ValidateCanonicalEvent` and reject `CanonicalTopicRole::kReferenceOnly` before
queueing. Track `ObservationHeader.sequence_id` per sensor: reject duplicate or decreasing values, accept a forward
gap while incrementing `sequence_gap_count` by the missing count, and reset that sensor's sequence baseline on an
explicit calibration-version change. A condition variable wakes `Run`; `Close` wakes all waiters and allows queued
events to drain before returning `kCompleted`.

- [ ] **Step 4: Run tests**

Run:

```bash
cmake --build build --target runtime_tests -j2
ctest --test-dir build -R unit.runtime.LiveEventSource --output-on-failure
```

Expected: all source, overflow, truth-isolation and close tests pass.

- [ ] **Step 5: Commit**

```bash
git add include/runtime/live_event_source.hpp src/runtime/live_event_source.cpp include/runtime/event_source.hpp cmake/Libraries.cmake tests/runtime/live_event_source_test.cpp cmake/Tests.cmake
git commit -m "feat(runtime): add bounded priority live event source"
```

### Task 5: Add rolling latency, age and health reporting

**Files:**
- Modify: `schemas/proto/uw/domain/health.proto`
- Create: `include/runtime/rolling_latency.hpp`
- Create: `src/runtime/rolling_latency.cpp`
- Modify: `include/runtime/live_event_source.hpp`
- Modify: `src/runtime/live_event_source.cpp`
- Modify: `cmake/Libraries.cmake`
- Create: `tests/runtime/rolling_latency_test.cpp`
- Modify: `tests/runtime/live_event_source_test.cpp`
- Modify: `cmake/Tests.cmake`

- [ ] **Step 1: Write failing percentile and health tests**

```cpp
TEST(RollingLatency, ComputesNearestRankPercentilesOverBoundedWindow) {
  RollingLatency stats(5);
  for (double value : {10.0, 50.0, 20.0, 40.0, 30.0}) stats.ObserveMs(value);
  EXPECT_DOUBLE_EQ(stats.Snapshot().p50_ms, 30.0);
  EXPECT_DOUBLE_EQ(stats.Snapshot().p95_ms, 50.0);
  stats.ObserveMs(5.0);
  EXPECT_EQ(stats.Snapshot().sample_count, 5u);
}
```

Extend the live-source health test to assert queue high-water mark, overflow rejects, semantic rejects, sequence
gaps, oldest message age, last valid capture/receive time and last processed time are populated.

- [ ] **Step 2: Run and verify failure**

Run `cmake --build build --target runtime_tests -j2`.

Expected: compile failure because the rolling statistics and health fields do not exist.

- [ ] **Step 3: Implement bounded rolling metrics**

Add these wire fields without renumbering existing fields:

```proto
uint64 rejected_frame_count = 15;
uint64 expired_frame_count = 16;
uint32 queue_high_watermark = 17;
double oldest_message_age_ms = 18;
uint64 deadline_miss_count = 19;
Stamp last_valid_capture_time = 20;
Stamp last_valid_receive_time = 21;
Stamp last_processed_time = 22;
uint64 synchronization_rejected_count = 23;
uint64 sequence_gap_count = 24;
uint64 stale_result_count = 25;
```

`RollingLatency` must keep a fixed-size ring and compute nearest-rank P50/P95/P99 on snapshot. `LiveEventSource`
must expose one health report per lane; queue residence time uses local monotonic ingress/pop time and must not
subtract unrelated capture clock domains.

- [ ] **Step 4: Run tests**

Run:

```bash
cmake -S . -B build -DUW_BUILD_TESTS=ON
cmake --build build --target runtime_tests -j2
ctest --test-dir build -R 'RollingLatency|LiveEventSource' --output-on-failure
```

Expected: all selected tests pass.

- [ ] **Step 5: Commit**

```bash
git add schemas/proto/uw/domain/health.proto include/runtime/rolling_latency.hpp src/runtime/rolling_latency.cpp include/runtime/live_event_source.hpp src/runtime/live_event_source.cpp cmake/Libraries.cmake tests/runtime/rolling_latency_test.cpp tests/runtime/live_event_source_test.cpp cmake/Tests.cmake
git commit -m "feat(runtime): report live queue latency and health"
```

### Task 6: Add a mixed-rate live-ingress executable and 30-minute gate

**Files:**
- Create: `apps/live_ingress_smoke.cpp`
- Modify: `cmake/Applications.cmake`
- Create: `tests/integration/live_ingress_smoke_test.sh`
- Modify: `cmake/Tests.cmake`
- Modify: `docs/testing-and-verification-guide-2026-08-20.md`

- [ ] **Step 1: Write the failing integration script**

The script must run a short CI profile and require explicit counters:

```bash
output="$($1 --duration-s 3 --camera-hz 20 --sonar-hz 10 --state-hz 50)"
grep -q 'reference_delivered=0' <<<"$output"
grep -q 'semantic_rejected=1' <<<"$output"
grep -q 'queue_capacity_violations=0' <<<"$output"
grep -q 'flush_count=1' <<<"$output"
```

- [ ] **Step 2: Register and run the test to verify failure**

Run:

```bash
cmake -S . -B build -DUW_BUILD_TESTS=ON
cmake --build build --target live_ingress_smoke -j2
```

Expected: build failure because the executable does not exist.

- [ ] **Step 3: Implement the smoke producer and counting port**

The app must start `PumpEvents` on a consumer thread, produce valid stereo frames at 20 Hz, sonar at 10 Hz and
vehicle state at 50 Hz using `steady_clock` deadlines, inject one invalid image and one reference-state event, then
close and join. Its final line must include:

```text
reference_delivered=0 semantic_rejected=1 queue_capacity_violations=0 flush_count=1
```

Support `--duration-s`; CI uses 3 seconds and the manual stability gate uses 1800 seconds.

- [ ] **Step 4: Run CI and manual stability profiles**

Run:

```bash
cmake --build build --target live_ingress_smoke -j2
ctest --test-dir build -R integration.live_ingress_smoke --output-on-failure
build/bin/live_ingress_smoke --duration-s 1800 --camera-hz 20 --sonar-hz 10 --state-hz 50
```

Expected: CI test passes. Manual output reports no queue capacity violation, no delivered truth, exactly one
semantic reject and one flush; RSS is externally sampled for the master-plan G1 evidence.

- [ ] **Step 5: Run the full regression suite and commit**

```bash
ctest --test-dir build --output-on-failure
git add apps/live_ingress_smoke.cpp cmake/Applications.cmake tests/integration/live_ingress_smoke_test.sh cmake/Tests.cmake docs/testing-and-verification-guide-2026-08-20.md
git commit -m "test(runtime): gate mixed-rate live ingestion"
```
