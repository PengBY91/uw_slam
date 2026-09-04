#include "application/online_assist_pipeline.hpp"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "application/latest_assist_sink.hpp"
#include "domain/domain.hpp"
#include "runtime/canonical_topics.hpp"
#include "sensor_models/geometry.hpp"

namespace {

using uw::application::DenseDepthProvider;
using uw::application::LatestAssistSink;
using uw::application::OnlineAssistPipeline;
using uw::application::OnlineAssistPipelineDependencies;
using uw::runtime::CanonicalEvent;

constexpr char kRigVersion[] = "assist-rig-v1";

// ---------------------------------------------------------------------
// Rig fixture: satisfies both AcousticOpticBuffer::ValidateRig (two
// cameras, one enabled sonar, one vehicle-state source, measured time
// offsets for every sensor) and TargetAssociator's frame resolution
// (frame_tree edges named exactly sensor_id + "_link", except sonar which
// resolves through the legacy canonical "sonar_link" name).
// ---------------------------------------------------------------------

void AddIdentityEdge(uw::domain::RigCalibrationSnapshot* rig, const std::string& child) {
  auto* edge = rig->add_frame_tree();
  edge->mutable_parent_frame()->set_value("base_link");
  edge->mutable_child_frame()->set_value(child);
  *edge->mutable_transform() = uw::sensor_models::Pose3::Identity().ToProto();
}

uw::domain::RigCalibrationSnapshot TestRig() {
  uw::domain::RigCalibrationSnapshot rig;
  rig.mutable_calibration_version()->set_value(kRigVersion);
  for (const std::string sensor : {"camera_left", "camera_right"}) {
    rig.add_cameras()->mutable_sensor_id()->set_value(sensor);
  }
  AddIdentityEdge(&rig, "camera_left_link");
  AddIdentityEdge(&rig, "camera_right_link");
  AddIdentityEdge(&rig, "sonar_link");
  auto* sonar = rig.add_sonar_beam_models();
  sonar->mutable_sensor_id()->set_value("sonar0");
  sonar->set_sonar_enabled(true);
  rig.add_vehicle_state_sensors()->set_value("rov-state");
  for (const auto& item : std::vector<std::pair<std::string, double>>{
           {"camera_left", 0.0}, {"camera_right", 0.0}, {"sonar0", 0.0}, {"rov-state", 0.0}}) {
    (*rig.mutable_time_offset_seconds())[item.first] = item.second;
    (*rig.mutable_time_offset_provenance())[item.first] = "measured:test";
  }
  return rig;
}

void SetHeader(uw::domain::ObservationHeader* header, const std::string& sensor_id,
              const std::string& sensor_frame, const std::string& observation_id,
              double capture_time_s, uint64_t sequence) {
  header->mutable_sensor_id()->set_value(sensor_id);
  header->mutable_sensor_frame()->set_value(sensor_frame);
  header->mutable_observation_id()->set_value(observation_id);
  header->mutable_sequence_id()->set_value(sequence);
  *header->mutable_capture_time() = uw::domain::FromSeconds(capture_time_s);
  *header->mutable_receive_time() = uw::domain::FromSeconds(capture_time_s);
  header->set_clock_domain(uw::domain::CLOCK_DOMAIN_SIMULATION);
  header->set_receive_clock_domain(uw::domain::CLOCK_DOMAIN_SIMULATION);
  header->mutable_calibration_version()->set_value(kRigVersion);
  header->set_validity(uw::domain::ObservationHeader::VALIDITY_OK);
}

CanonicalEvent MakeImageEvent(const std::string& sensor_id, const std::string& sensor_frame,
                              const char* topic, double capture_time_s, uint64_t sequence) {
  uw::domain::ImageFrame frame;
  // A stereo pair shares ONE observation_id (AcousticOpticBuffer's stereo
  // integrity check matches left/right by identical observation_id, not by
  // per-camera identity) -- see MakeImage's callers in
  // acoustic_optic_buffer_test.cpp for the same convention.
  SetHeader(frame.mutable_header(), sensor_id, sensor_frame, "frame-" + std::to_string(sequence),
           capture_time_s, sequence);
  frame.set_width(4);
  frame.set_height(4);
  frame.set_encoding(uw::domain::ImageFrame::IMAGE_ENCODING_MONO8);
  frame.set_pixel_data(std::string(16, '\x40'));
  frame.set_is_rectified(true);
  CanonicalEvent event;
  event.topic = topic;
  event.log_time_ns = static_cast<uint64_t>(capture_time_s * 1.0e9);
  event.source_sequence = sequence;
  event.payload = frame;
  return event;
}

CanonicalEvent MakeSonarEvent(double capture_time_s, uint64_t sequence) {
  uw::domain::SonarFrame frame;
  SetHeader(frame.mutable_header(), "sonar0", "sonar_link",
           "sonar-" + std::to_string(sequence), capture_time_s, sequence);
  frame.set_num_ranges(1);
  frame.set_num_beams(1);
  frame.set_encoding(uw::domain::SonarFrame::ENCODING_UINT8_GRAY);
  CanonicalEvent event;
  event.topic = uw::runtime::kTopicSonarFrame;
  event.log_time_ns = static_cast<uint64_t>(capture_time_s * 1.0e9);
  event.source_sequence = sequence;
  event.payload = frame;
  return event;
}

CanonicalEvent MakeVehicleStateEvent(double capture_time_s, uint64_t sequence) {
  uw::domain::VehicleState state;
  SetHeader(state.mutable_header(), "rov-state", "rov-state_link",
           "state-" + std::to_string(sequence), capture_time_s, sequence);
  state.add_orientation_xyzw(0.0);
  state.add_orientation_xyzw(0.0);
  state.add_orientation_xyzw(0.0);
  state.add_orientation_xyzw(1.0);
  state.add_angular_velocity_radps(0.0);
  state.add_angular_velocity_radps(0.0);
  state.add_angular_velocity_radps(0.0);
  state.set_depth_m(2.0);
  state.set_attitude_valid(true);
  state.set_depth_valid(true);
  state.set_device_health_valid(true);
  state.set_link_quality(0.9);
  state.set_supply_voltage_v(15.0);
  state.set_supply_current_a(2.0);
  for (int i = 0; i < 49; ++i) state.add_covariance_7x7_row_major(0.0);
  CanonicalEvent event;
  event.topic = uw::runtime::kTopicVehicleState;
  event.log_time_ns = static_cast<uint64_t>(capture_time_s * 1.0e9);
  event.source_sequence = sequence;
  event.payload = state;
  return event;
}

// ---------------------------------------------------------------------
// Fakes
// ---------------------------------------------------------------------

class FakeVisualAssistFrontend final : public uw::measurement_api::VisualAssistFrontend {
 public:
  uw::measurement_api::VisualAssistResult Process(
      const uw::domain::ImageFrame& left_rectified,
      const std::optional<uw::domain::OpticalDepthPriorMeasurement>& depth,
      const uw::domain::CameraIntrinsics& intrinsics) override {
    (void)depth;
    (void)intrinsics;
    ++call_count;
    uw::measurement_api::VisualAssistResult result;
    if (produce_detection) {
      uw::domain::TargetDetection detection;
      *detection.mutable_source_observation() = left_rectified.header().observation_id();
      *detection.mutable_capture_time() = left_rectified.header().capture_time();
      detection.set_class_label("target");
      detection.set_confidence(0.9);
      // Boresight (0.0) is the one bearing value whose real-world direction
      // is the same regardless of a sensor's own bearing-axis convention --
      // see the identical choice in target_fusion_binding_test.cpp's
      // VisualMeasurement(). A nonzero value here would need the camera's
      // and sonar's bearing conventions reconciled through real intrinsics
      // to land on the same 3D ray, which this fixture's identity-extrinsic,
      // no-intrinsics rig does not attempt to model.
      detection.set_bearing_rad(0.0);
      detection.set_has_range(false);
      detection.add_covariance_2x2_row_major(1.0e-4);
      detection.add_covariance_2x2_row_major(0.0);
      detection.add_covariance_2x2_row_major(0.0);
      detection.add_covariance_2x2_row_major(1.0e6);
      detection.set_source(uw::domain::ASSIST_SOURCE_VISUAL);
      result.targets.push_back(std::move(detection));
    }
    result.health.set_component_id("fake_visual");
    result.health.set_status(uw::domain::HealthReport::STATUS_HEALTHY);
    return result;
  }

  bool produce_detection = true;
  int call_count = 0;
};

class FakeSonarFrontend final : public uw::measurement_api::SonarFrontend {
 public:
  uw::domain::HypothesisSet ProcessSonarFrame(const uw::domain::SonarFrame& frame) override {
    ++call_count;
    uw::domain::HypothesisSet hypotheses;
    if (produce_detection) {
      uw::domain::SonarRangeBearing measurement;
      measurement.set_bearing_rad(0.0);  // boresight -- see FakeVisualAssistFrontend comment
      measurement.set_range_m(4.0);
      measurement.set_bearing_sigma_rad(0.01);
      measurement.set_range_sigma_m(0.05);
      uw::domain::EvidenceId id;
      id.set_value("fake-sonar-evidence-" + std::to_string(call_count));
      auto evidence = uw::domain::MakeEvidence(id, {frame.header().observation_id()}, measurement,
                                               1.0, "fake_sonar_v1");
      (*evidence.mutable_quality_features())["cfar_score"] = 2.0;
      *hypotheses.add_candidates() = evidence;
    }
    return hypotheses;
  }

  uw::domain::HealthReport Health() const override {
    uw::domain::HealthReport health;
    health.set_component_id("fake_sonar");
    health.set_status(report_status);
    health.set_reason_code(reason_code);
    return health;
  }

  bool produce_detection = true;
  uw::domain::HealthReport::Status report_status = uw::domain::HealthReport::STATUS_HEALTHY;
  std::string reason_code;
  int call_count = 0;
};

enum class DenseBehavior { kSuccess, kTimeout };

class FakeDenseDepthProvider final : public DenseDepthProvider {
 public:
  explicit FakeDenseDepthProvider(DenseBehavior behavior) : behavior_(behavior) {}

  std::optional<uw::domain::OpticalDepthPriorMeasurement> RunBounded(
      const uw::measurement_api::CameraFrameBundle& images,
      const uw::domain::RigCalibrationSnapshot& rig, double budget_ms) override {
    (void)images;
    (void)rig;
    (void)budget_ms;
    ++call_count;
    if (behavior_ == DenseBehavior::kTimeout) return std::nullopt;
    uw::domain::OpticalDepthPriorMeasurement depth;
    depth.set_width(1);
    depth.set_height(1);
    depth.add_depth_m(4.0f);
    depth.add_variance_m2(0.01f);
    depth.set_valid_mask(std::string(1, static_cast<char>(1)));
    return depth;
  }

  DenseBehavior behavior_;
  int call_count = 0;
};

class FakeClock {
 public:
  uw::domain::Stamp operator()() const { return uw::domain::FromSeconds(seconds_); }
  void Set(double seconds_value) { seconds_ = seconds_value; }

 private:
  double seconds_ = 0.0;
};

OnlineAssistPipelineDependencies TestPipelineDependencies(
    FakeClock& clock, LatestAssistSink& sink, uw::measurement_api::VisualAssistFrontend& visual,
    uw::measurement_api::SonarFrontend& sonar, DenseDepthProvider* dense_provider,
    bool dense_enabled) {
  OnlineAssistPipelineDependencies deps;
  deps.visual_frontend = &visual;
  deps.sonar_frontend = &sonar;
  deps.dense_depth_provider = dense_provider;
  deps.sink = &sink;
  deps.rig = TestRig();
  // FeedSynchronizedVehicleStereoSonar's 20 Hz stereo / 10 Hz sonar cadence
  // puts an "off" stereo tick exactly 0.05s from its nearest sonar
  // neighbor -- right on TargetAssociationConfig's default
  // max_corrected_time_delta_s boundary. Widen it to comfortably cover that
  // worst case so pairing is deterministic rather than a floating-point
  // coin flip; this is a fixture timing choice, not a claim about the
  // production default (covered by target_associator's own tests).
  deps.target_association.max_corrected_time_delta_s = 0.1;
  deps.pipeline.dense.enabled = dense_enabled;
  deps.pipeline.dense.budget_ms = 100.0;
  deps.pipeline.vehicle_state_stale_after_s = 0.5;
  deps.pipeline.modality_stale_after_s = 1.0;
  deps.now = [&clock] { return clock(); };
  return deps;
}

// 50 Hz vehicle state, 20 Hz stereo, 10 Hz sonar, merged into time order --
// matches the plan's "FeedSynchronizedVehicleStereoSonar" fixture.
void FeedSynchronizedVehicleStereoSonar(OnlineAssistPipeline& pipeline, FakeClock& clock,
                                        double duration_s) {
  struct Tick {
    double time_s;
    int kind;  // 0 = vehicle state, 1 = stereo, 2 = sonar
  };
  std::vector<Tick> ticks;
  for (double t = 0.0; t < duration_s; t += 1.0 / 50.0) ticks.push_back({t, 0});
  for (double t = 0.0; t < duration_s; t += 1.0 / 20.0) ticks.push_back({t, 1});
  for (double t = 0.0; t < duration_s; t += 1.0 / 10.0) ticks.push_back({t, 2});
  std::stable_sort(ticks.begin(), ticks.end(),
                   [](const Tick& a, const Tick& b) { return a.time_s < b.time_s; });

  uint64_t sequence = 0;
  for (const auto& tick : ticks) {
    ++sequence;
    clock.Set(tick.time_s);
    if (tick.kind == 0) {
      pipeline.OnVehicleState(MakeVehicleStateEvent(tick.time_s, sequence));
    } else if (tick.kind == 1) {
      pipeline.OnImageFrame(
          MakeImageEvent("camera_left", "camera_left_link", uw::runtime::kTopicCameraLeft,
                         tick.time_s, sequence));
      pipeline.OnImageFrame(
          MakeImageEvent("camera_right", "camera_right_link", uw::runtime::kTopicCameraRight,
                         tick.time_s, sequence));
    } else {
      pipeline.OnSonarFrame(MakeSonarEvent(tick.time_s, sequence));
    }
  }
}

}  // namespace

TEST(OnlineAssistPipeline, DenseTimeoutDoesNotBlockFreshTracks) {
  FakeClock clock;
  LatestAssistSink sink;
  FakeVisualAssistFrontend visual;
  FakeSonarFrontend sonar;
  FakeDenseDepthProvider dense(DenseBehavior::kTimeout);
  OnlineAssistPipeline pipeline(
      TestPipelineDependencies(clock, sink, visual, sonar, &dense, /*dense_enabled=*/true));

  FeedSynchronizedVehicleStereoSonar(pipeline, clock, 10.0);

  const auto latest = sink.Latest();
  ASSERT_TRUE(latest.has_value());
  ASSERT_EQ(latest->target_tracks().tracks_size(), 1);
  EXPECT_LT(latest->data_age_ms(), 250.0);
  EXPECT_TRUE(latest->guidance_valid());
  EXPECT_EQ(latest->system_health().status(), uw::domain::HealthReport::STATUS_SUSPECT);
  EXPECT_EQ(latest->system_health().reason_code(), "dense_deadline_missed");
  EXPECT_GT(dense.call_count, 0);
}

TEST(OnlineAssistPipeline, DenseSuccessStaysHealthy) {
  FakeClock clock;
  LatestAssistSink sink;
  FakeVisualAssistFrontend visual;
  FakeSonarFrontend sonar;
  FakeDenseDepthProvider dense(DenseBehavior::kSuccess);
  OnlineAssistPipeline pipeline(
      TestPipelineDependencies(clock, sink, visual, sonar, &dense, /*dense_enabled=*/true));

  FeedSynchronizedVehicleStereoSonar(pipeline, clock, 10.0);

  const auto latest = sink.Latest();
  ASSERT_TRUE(latest.has_value());
  ASSERT_EQ(latest->target_tracks().tracks_size(), 1);
  EXPECT_TRUE(latest->guidance_valid());
  EXPECT_EQ(latest->system_health().status(), uw::domain::HealthReport::STATUS_HEALTHY);
  EXPECT_EQ(latest->system_health().reason_code(), "");
  EXPECT_GT(dense.call_count, 0);
}

TEST(OnlineAssistPipeline, DenseDisabledByDefaultNeverInvokesProvider) {
  FakeClock clock;
  LatestAssistSink sink;
  FakeVisualAssistFrontend visual;
  FakeSonarFrontend sonar;
  FakeDenseDepthProvider dense(DenseBehavior::kSuccess);
  OnlineAssistPipeline pipeline(
      TestPipelineDependencies(clock, sink, visual, sonar, &dense, /*dense_enabled=*/false));

  FeedSynchronizedVehicleStereoSonar(pipeline, clock, 2.0);

  EXPECT_EQ(dense.call_count, 0);
  const auto latest = sink.Latest();
  ASSERT_TRUE(latest.has_value());
  EXPECT_EQ(latest->system_health().status(), uw::domain::HealthReport::STATUS_HEALTHY);
}

// visual_unavailable/sonar_unavailable are liveness signals (no frames from
// that modality at all -- e.g. a disconnected camera), not "the frontend
// looked and found nothing this frame" (that is a normal, healthy outcome
// already covered by result.health). So these two tests genuinely withhold
// one raw modality's CanonicalEvents rather than just configuring the fake
// frontend to return empty results.
TEST(OnlineAssistPipeline, SonarOnlyStillProducesTracks) {
  FakeClock clock;
  LatestAssistSink sink;
  FakeVisualAssistFrontend visual;
  FakeSonarFrontend sonar;
  OnlineAssistPipeline pipeline(TestPipelineDependencies(clock, sink, visual, sonar,
                                                         /*dense_provider=*/nullptr,
                                                         /*dense_enabled=*/false));

  double t = 0.0;
  for (int i = 0; i < 40; ++i) {  // 4s of 10 Hz sonar + matching state, no stereo at all
    const auto sequence = static_cast<uint64_t>(i + 1);
    clock.Set(t);
    pipeline.OnVehicleState(MakeVehicleStateEvent(t, sequence));
    pipeline.OnSonarFrame(MakeSonarEvent(t, sequence));
    t += 0.1;
  }

  const auto latest = sink.Latest();
  ASSERT_TRUE(latest.has_value());
  ASSERT_EQ(latest->target_tracks().tracks_size(), 1);
  EXPECT_TRUE(latest->guidance_valid());
  EXPECT_EQ(latest->system_health().reason_code(), "visual_unavailable");
  EXPECT_EQ(visual.call_count, 0);
}

TEST(OnlineAssistPipeline, VisualOnlyStillProducesTracks) {
  FakeClock clock;
  LatestAssistSink sink;
  FakeVisualAssistFrontend visual;
  FakeSonarFrontend sonar;
  OnlineAssistPipeline pipeline(TestPipelineDependencies(clock, sink, visual, sonar,
                                                         /*dense_provider=*/nullptr,
                                                         /*dense_enabled=*/false));

  double t = 0.0;
  for (int i = 0; i < 80; ++i) {  // 4s of 20 Hz stereo + matching state, no sonar at all
    const auto sequence = static_cast<uint64_t>(i + 1);
    clock.Set(t);
    pipeline.OnVehicleState(MakeVehicleStateEvent(t, sequence));
    pipeline.OnImageFrame(MakeImageEvent("camera_left", "camera_left_link",
                                        uw::runtime::kTopicCameraLeft, t, sequence));
    pipeline.OnImageFrame(MakeImageEvent("camera_right", "camera_right_link",
                                        uw::runtime::kTopicCameraRight, t, sequence));
    t += 0.05;
  }

  const auto latest = sink.Latest();
  ASSERT_TRUE(latest.has_value());
  ASSERT_EQ(latest->target_tracks().tracks_size(), 1);
  EXPECT_TRUE(latest->guidance_valid());
  EXPECT_EQ(latest->system_health().reason_code(), "sonar_unavailable");
  EXPECT_EQ(sonar.call_count, 0);
}

// Sonar frames keep arriving on schedule (sonar_live stays true) but the
// sonar frontend itself reports a genuine quality degradation -- this must
// surface into system_health() the same way an equivalent visual-frontend
// degradation would (symmetric with the visual_frontend.health check).
TEST(OnlineAssistPipeline, SonarFrontendDegradationSurfacesInSystemHealth) {
  FakeClock clock;
  LatestAssistSink sink;
  FakeVisualAssistFrontend visual;
  FakeSonarFrontend sonar;
  sonar.report_status = uw::domain::HealthReport::STATUS_SUSPECT;
  sonar.reason_code = "background_noise_high";
  OnlineAssistPipeline pipeline(TestPipelineDependencies(clock, sink, visual, sonar,
                                                         /*dense_provider=*/nullptr,
                                                         /*dense_enabled=*/false));

  FeedSynchronizedVehicleStereoSonar(pipeline, clock, 2.0);

  const auto latest = sink.Latest();
  ASSERT_TRUE(latest.has_value());
  EXPECT_TRUE(latest->guidance_valid());
  EXPECT_EQ(latest->system_health().status(), uw::domain::HealthReport::STATUS_SUSPECT);
  EXPECT_EQ(latest->system_health().reason_code(), "background_noise_high");
}

TEST(OnlineAssistPipeline, StaleVehicleStateInvalidatesGuidance) {
  FakeClock clock;
  LatestAssistSink sink;
  FakeVisualAssistFrontend visual;
  FakeSonarFrontend sonar;
  OnlineAssistPipeline pipeline(TestPipelineDependencies(clock, sink, visual, sonar,
                                                         /*dense_provider=*/nullptr,
                                                         /*dense_enabled=*/false));

  pipeline.OnVehicleState(MakeVehicleStateEvent(0.0, 1));
  clock.Set(5.0);
  pipeline.OnImageFrame(
      MakeImageEvent("camera_left", "camera_left_link", uw::runtime::kTopicCameraLeft, 5.0, 2));

  const auto latest = sink.Latest();
  ASSERT_TRUE(latest.has_value());
  EXPECT_FALSE(latest->guidance_valid());
  EXPECT_EQ(latest->system_health().reason_code(), "vehicle_state_stale");
  EXPECT_EQ(latest->system_health().status(), uw::domain::HealthReport::STATUS_UNAVAILABLE);
}

TEST(OnlineAssistPipeline, AllModalitiesSilentReportsAllUnavailable) {
  FakeClock clock;
  LatestAssistSink sink;
  FakeVisualAssistFrontend visual;
  FakeSonarFrontend sonar;
  OnlineAssistPipeline pipeline(TestPipelineDependencies(clock, sink, visual, sonar,
                                                         /*dense_provider=*/nullptr,
                                                         /*dense_enabled=*/false));

  pipeline.Flush();

  const auto latest = sink.Latest();
  ASSERT_TRUE(latest.has_value());
  EXPECT_FALSE(latest->guidance_valid());
  EXPECT_EQ(latest->system_health().reason_code(), "all_assist_unavailable");
  EXPECT_EQ(latest->system_health().status(), uw::domain::HealthReport::STATUS_UNAVAILABLE);
}

// docs/archive/rov-realtime-closed-loop-code-review-2026-08-27.md finding C1: every
// OnVehicleState call used to call PublishNow() unconditionally, driving a
// full HMI render + JSON rebuild on every single message -- up to ~145/sec
// at overload. min_publish_interval_s throttles the actual publish while
// internal tracking state keeps updating on every event regardless.
TEST(OnlineAssistPipeline, PublishIsThrottledButPublishedStateStaysCurrent) {
  FakeClock clock;
  LatestAssistSink sink;
  FakeVisualAssistFrontend visual;
  FakeSonarFrontend sonar;
  auto deps = TestPipelineDependencies(clock, sink, visual, sonar, /*dense_provider=*/nullptr,
                                       /*dense_enabled=*/false);
  deps.pipeline.min_publish_interval_s = 0.1;
  OnlineAssistPipeline pipeline(std::move(deps));

  // 50 vehicle-state events 10ms apart (0.5s total) -- ten times the
  // throttle's own cadence -- must NOT produce 50 publishes.
  double t = 0.0;
  for (int i = 0; i < 50; ++i) {
    clock.Set(t);
    pipeline.OnVehicleState(MakeVehicleStateEvent(t, static_cast<uint64_t>(i + 1)));
    t += 0.01;
  }

  const auto published = pipeline.Diagnostics().published_count;
  EXPECT_GE(published, 4u);   // ~0.5s / 0.1s interval, plus the always-published first call
  EXPECT_LE(published, 10u);  // must be far fewer than the 50 events fed

  // The throttle only gates the publish call, not internal bookkeeping --
  // the most recent capture time must still be reflected once something
  // does publish (verified here via Flush(), which always force-publishes).
  pipeline.Flush();
  const auto latest = sink.Latest();
  ASSERT_TRUE(latest.has_value());
  EXPECT_NEAR(latest->data_age_ms(), 0.0, 1.0);
}

TEST(OnlineAssistPipeline, PublishThrottlingCanBeDisabledViaZeroInterval) {
  FakeClock clock;
  LatestAssistSink sink;
  FakeVisualAssistFrontend visual;
  FakeSonarFrontend sonar;
  auto deps = TestPipelineDependencies(clock, sink, visual, sonar, /*dense_provider=*/nullptr,
                                       /*dense_enabled=*/false);
  deps.pipeline.min_publish_interval_s = 0.0;
  OnlineAssistPipeline pipeline(std::move(deps));

  double t = 0.0;
  for (int i = 0; i < 20; ++i) {
    clock.Set(t);
    pipeline.OnVehicleState(MakeVehicleStateEvent(t, static_cast<uint64_t>(i + 1)));
    t += 0.01;
  }

  EXPECT_EQ(pipeline.Diagnostics().published_count, 20u);
}

TEST(OnlineAssistPipeline, FlushAlwaysPublishesRegardlessOfThrottleWindow) {
  FakeClock clock;
  LatestAssistSink sink;
  FakeVisualAssistFrontend visual;
  FakeSonarFrontend sonar;
  auto deps = TestPipelineDependencies(clock, sink, visual, sonar, /*dense_provider=*/nullptr,
                                       /*dense_enabled=*/false);
  deps.pipeline.min_publish_interval_s = 10.0;  // effectively "never throttle-elapses" for this test
  OnlineAssistPipeline pipeline(std::move(deps));

  clock.Set(0.0);
  pipeline.OnVehicleState(MakeVehicleStateEvent(0.0, 1));  // first call always publishes
  const auto after_first = pipeline.Diagnostics().published_count;
  pipeline.Flush();  // zero elapsed wall time since the first publish -- must still publish
  EXPECT_EQ(pipeline.Diagnostics().published_count, after_first + 1);
}

TEST(OnlineAssistPipeline, CalibrationChangeResetsAndRecovers) {
  FakeClock clock;
  LatestAssistSink sink;
  FakeVisualAssistFrontend visual;
  FakeSonarFrontend sonar;
  OnlineAssistPipeline pipeline(TestPipelineDependencies(clock, sink, visual, sonar,
                                                         /*dense_provider=*/nullptr,
                                                         /*dense_enabled=*/false));

  FeedSynchronizedVehicleStereoSonar(pipeline, clock, 2.0);
  ASSERT_TRUE(sink.Latest().has_value());
  ASSERT_EQ(sink.Latest()->target_tracks().tracks_size(), 1);

  auto new_rig = TestRig();
  new_rig.mutable_calibration_version()->set_value("assist-rig-v2");
  pipeline.UpdateRig(new_rig);

  const auto recovering = sink.Latest();
  ASSERT_TRUE(recovering.has_value());
  EXPECT_FALSE(recovering->guidance_valid());
  EXPECT_EQ(recovering->system_health().reason_code(), "recovering");
  EXPECT_EQ(recovering->system_health().status(), uw::domain::HealthReport::STATUS_RECOVERING);
  EXPECT_EQ(recovering->target_tracks().tracks_size(), 0);
  EXPECT_EQ(pipeline.Diagnostics().calibration_reset_count, 1u);

  // Header calibration_version must match the new rig for detections to be
  // accepted again -- rebuild the feed with the new version stamped in.
  const std::string old_version = kRigVersion;
  (void)old_version;
  double t = 2.0;
  for (int i = 0; i < 40; ++i) {
    uint64_t sequence = static_cast<uint64_t>(200 + i);
    clock.Set(t);
    auto left = MakeImageEvent("camera_left", "camera_left_link", uw::runtime::kTopicCameraLeft, t,
                               sequence);
    auto sonar_event = MakeSonarEvent(t, sequence);
    auto state_event = MakeVehicleStateEvent(t, sequence);
    std::get<uw::domain::ImageFrame>(left.payload)
        .mutable_header()
        ->mutable_calibration_version()
        ->set_value("assist-rig-v2");
    std::get<uw::domain::SonarFrame>(sonar_event.payload)
        .mutable_header()
        ->mutable_calibration_version()
        ->set_value("assist-rig-v2");
    std::get<uw::domain::VehicleState>(state_event.payload)
        .mutable_header()
        ->mutable_calibration_version()
        ->set_value("assist-rig-v2");
    pipeline.OnVehicleState(state_event);
    pipeline.OnImageFrame(left);
    pipeline.OnSonarFrame(sonar_event);
    t += 0.1;
  }

  const auto recovered = sink.Latest();
  ASSERT_TRUE(recovered.has_value());
  EXPECT_EQ(recovered->target_tracks().tracks_size(), 1);
  EXPECT_TRUE(recovered->guidance_valid());
  EXPECT_NE(recovered->system_health().reason_code(), "recovering");
}

// docs/archive/rov-realtime-closed-loop-code-review-2026-08-27.md finding B5:
// FUS-HEALTH-002's "recovery must not silently reuse pre-fault cached
// state" was only ever enforced on a calibration-version change
// (UpdateRig). A plain sensor dropout+recovery had no equivalent -- this
// extends the SAME recovering_ gate to visual/sonar modality recovery,
// without the full associator/tracker reset UpdateRig does (a modality
// blip doesn't invalidate rig geometry the way a calibration change does).
//
// This checks modality_recovery_count rather than trying to catch
// sink.Latest() showing STATUS_RECOVERING at some exact intermediate tick:
// TargetTracker's hits counter never resets on a miss/staleness (only
// consecutive_misses does), and neither Predict() nor the stale/retention
// pruning in finding C2 ever run for a modality that receives literally no
// frames (they're driven by TargetTracker::Update(), only reachable via
// FlushAssociation(), only called from RunVisualDetection/
// RunSonarDetection) -- so a track that was CONFIRMED before a total
// silence gap can legitimately re-associate and reconfirm on the very
// first post-gap detection (one call, same tick) with these fixture
// frontends, which always report a fixed boresight bearing. That is not a
// bug: recovering_ still did its job (gating guidance on that first,
// immediately-reconfirmed hit rather than trusting it unconditionally);
// it is just not reliably observable as a separate published tick from
// outside. The counter is what reliably proves the trigger itself fired.
// docs/archive/rov-realtime-closed-loop-code-review-2026-08-27.md finding A2:
// realtime_gate.py's acceptance evidence needs non-zero sonar/visual
// detection counts and a non-zero fused track count, which nothing
// tracked cumulatively before -- these three counters are what the C++
// gateway-side metrics report (still being built out) will read.
TEST(OnlineAssistPipeline, TracksCumulativeDetectionAndFusedTrackCounts) {
  FakeClock clock;
  LatestAssistSink sink;
  FakeVisualAssistFrontend visual;
  FakeSonarFrontend sonar;
  OnlineAssistPipeline pipeline(TestPipelineDependencies(clock, sink, visual, sonar,
                                                         /*dense_provider=*/nullptr,
                                                         /*dense_enabled=*/false));

  EXPECT_EQ(pipeline.Diagnostics().sonar_detection_count, 0u);
  EXPECT_EQ(pipeline.Diagnostics().visual_detection_count, 0u);
  EXPECT_EQ(pipeline.Diagnostics().fused_track_publish_count, 0u);

  FeedSynchronizedVehicleStereoSonar(pipeline, clock, 2.0);

  const auto diagnostics = pipeline.Diagnostics();
  // FakeVisualAssistFrontend/FakeSonarFrontend each emit exactly one
  // detection per call when produce_detection is true (the default) --
  // 20 Hz stereo and 10 Hz sonar over 2.0s.
  EXPECT_GT(diagnostics.visual_detection_count, 0u);
  EXPECT_GT(diagnostics.sonar_detection_count, 0u);
  // Both fakes report boresight (bearing 0.0), so visual and sonar
  // detections associate into the same track -- it carries both sources
  // and should be counted as fused on every publish once confirmed.
  EXPECT_GT(diagnostics.fused_track_publish_count, 0u);
}

TEST(OnlineAssistPipeline, DetectionCountsStayZeroWhenFrontendsProduceNothing) {
  FakeClock clock;
  LatestAssistSink sink;
  FakeVisualAssistFrontend visual;
  FakeSonarFrontend sonar;
  visual.produce_detection = false;
  sonar.produce_detection = false;
  OnlineAssistPipeline pipeline(TestPipelineDependencies(clock, sink, visual, sonar,
                                                         /*dense_provider=*/nullptr,
                                                         /*dense_enabled=*/false));

  FeedSynchronizedVehicleStereoSonar(pipeline, clock, 1.0);

  const auto diagnostics = pipeline.Diagnostics();
  EXPECT_EQ(diagnostics.sonar_detection_count, 0u);
  EXPECT_EQ(diagnostics.visual_detection_count, 0u);
  EXPECT_EQ(diagnostics.fused_track_publish_count, 0u);
}

TEST(OnlineAssistPipeline, ModalityDropoutRecoveryIncrementsRecoveryCounter) {
  FakeClock clock;
  LatestAssistSink sink;
  FakeVisualAssistFrontend visual;
  FakeSonarFrontend sonar;
  OnlineAssistPipeline pipeline(TestPipelineDependencies(clock, sink, visual, sonar,
                                                         /*dense_provider=*/nullptr,
                                                         /*dense_enabled=*/false));

  FeedSynchronizedVehicleStereoSonar(pipeline, clock, 2.0);
  ASSERT_TRUE(sink.Latest().has_value());
  ASSERT_EQ(sink.Latest()->target_tracks().tracks_size(), 1);
  EXPECT_TRUE(sink.Latest()->guidance_valid());
  EXPECT_EQ(pipeline.Diagnostics().modality_recovery_count, 0u);

  // Both visual and sonar go silent for well over modality_stale_after_s
  // (1.0s) -- vehicle state keeps flowing throughout so vehicle_state_stale
  // doesn't mask what this test is checking.
  double t = 2.0;
  uint64_t sequence = 100000;
  for (int i = 0; i < 100; ++i) {  // 2.0s of state-only silence
    clock.Set(t);
    pipeline.OnVehicleState(MakeVehicleStateEvent(t, sequence));
    ++sequence;
    t += 0.02;
  }

  // Resume both modalities together -- each independently exceeded its own
  // stale threshold, so both triggers should fire (visual via
  // RunVisualDetection, sonar via RunSonarDetection).
  clock.Set(t);
  pipeline.OnVehicleState(MakeVehicleStateEvent(t, sequence));
  pipeline.OnImageFrame(
      MakeImageEvent("camera_left", "camera_left_link", uw::runtime::kTopicCameraLeft, t, sequence));
  pipeline.OnImageFrame(
      MakeImageEvent("camera_right", "camera_right_link", uw::runtime::kTopicCameraRight, t, sequence));
  pipeline.OnSonarFrame(MakeSonarEvent(t, sequence));

  EXPECT_EQ(pipeline.Diagnostics().modality_recovery_count, 2u);

  // The system must not be left permanently gated off by this -- guidance
  // resumes once the (in this fixture, immediately) reconfirmed track
  // publishes.
  pipeline.Flush();
  const auto recovered = sink.Latest();
  ASSERT_TRUE(recovered.has_value());
  EXPECT_TRUE(recovered->guidance_valid());
}

TEST(OnlineAssistPipeline, ShortModalityGapBelowStaleThresholdDoesNotEnterRecovering) {
  FakeClock clock;
  LatestAssistSink sink;
  FakeVisualAssistFrontend visual;
  FakeSonarFrontend sonar;
  OnlineAssistPipeline pipeline(TestPipelineDependencies(clock, sink, visual, sonar,
                                                         /*dense_provider=*/nullptr,
                                                         /*dense_enabled=*/false));

  FeedSynchronizedVehicleStereoSonar(pipeline, clock, 2.0);
  ASSERT_TRUE(sink.Latest().has_value());
  EXPECT_TRUE(sink.Latest()->guidance_valid());

  // A gap comfortably under modality_stale_after_s (1.0s) is routine
  // scheduling jitter, not a dropout -- must never trip recovering_.
  const double t = 2.3;
  clock.Set(t);
  pipeline.OnSonarFrame(MakeSonarEvent(t, 99999));

  EXPECT_EQ(pipeline.Diagnostics().modality_recovery_count, 0u);
  const auto latest = sink.Latest();
  ASSERT_TRUE(latest.has_value());
  EXPECT_NE(latest->system_health().status(), uw::domain::HealthReport::STATUS_RECOVERING);
  EXPECT_NE(latest->system_health().reason_code(), "recovering");
}
