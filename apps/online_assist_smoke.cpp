// Gates the real 20/10/50 Hz online acoustic-optic assistance slice: real
// stereo/sonar detection frontends and OnlineAssistPipeline behind the same
// LiveEventSource -> PumpEvents wiring live_ingress_smoke already exercises
// for raw ingestion, so this is where the fused-track output actually gets
// produced and checked. See docs/superpowers/plans/2026-08-24-acoustic-
// optic-online-tracking.md Task 8.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <future>
#include <iostream>
#include <limits>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "adapters/opencv_visual_assist_frontend.hpp"
#include "application/event_pump.hpp"
#include "application/online_assist_pipeline.hpp"
#include "application/pipeline_input_port.hpp"
#include "domain/domain.hpp"
#include "frontends/sonar_cfar_frontend.hpp"
#include "runtime/canonical_topics.hpp"
#include "runtime/live_event_source.hpp"
#include "runtime/synthetic_sonar.hpp"
#include "sensor_models/geometry.hpp"

namespace {

using SteadyClock = std::chrono::steady_clock;
using uw::application::OnlineAssistPipeline;
using uw::application::OnlineAssistPipelineDependencies;
using uw::domain::OperatorAssistState;
using uw::runtime::CanonicalEvent;
using uw::runtime::EventSourceReport;
using uw::runtime::EventSourceStatus;
using uw::runtime::LiveEventSource;
using uw::runtime::LiveSourceConfig;
using uw::runtime::LiveSubmitStatus;

constexpr char kCalibrationVersion[] = "online-assist-smoke-v1";
constexpr char kLeftCamera[] = "camera-left";
constexpr char kRightCamera[] = "camera-right";
constexpr char kSonarSensor[] = "sonar-forward";
constexpr char kStateSensor[] = "rov-state";
constexpr uint32_t kImageWidth = 320;
constexpr uint32_t kImageHeight = 240;

// ---------------------------------------------------------------------
// Option parsing (mirrors apps/live_ingress_smoke.cpp's conventions).
// ---------------------------------------------------------------------

struct Options {
  double duration_s = 5.0;
  double camera_hz = 20.0;
  double sonar_hz = 10.0;
  double state_hz = 50.0;
  std::optional<double> drop_visual_at_s;
  std::optional<double> drop_sonar_at_s;
};

double ParsePositiveFinite(const char* option, const char* text) {
  std::size_t parsed = 0;
  double value = 0.0;
  try {
    value = std::stod(text, &parsed);
  } catch (const std::exception&) {
    throw std::invalid_argument(std::string(option) + " requires a numeric value");
  }
  if (parsed != std::string(text).size() || !std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(option) + " must be positive and finite");
  }
  return value;
}

Options ParseOptions(int argc, char** argv) {
  Options options;
  for (int index = 1; index < argc; index += 2) {
    const std::string option = argv[index];
    const bool known_option = option == "--duration-s" || option == "--camera-hz" ||
                              option == "--sonar-hz" || option == "--state-hz" ||
                              option == "--drop-visual-at-s" || option == "--drop-sonar-at-s";
    if (!known_option) throw std::invalid_argument("unknown argument: " + option);
    if (index + 1 >= argc) throw std::invalid_argument(option + " requires a value");
    const char* value_text = argv[index + 1];
    if (option == "--duration-s") {
      options.duration_s = ParsePositiveFinite(option.c_str(), value_text);
    } else if (option == "--camera-hz") {
      options.camera_hz = ParsePositiveFinite(option.c_str(), value_text);
    } else if (option == "--sonar-hz") {
      options.sonar_hz = ParsePositiveFinite(option.c_str(), value_text);
    } else if (option == "--state-hz") {
      options.state_hz = ParsePositiveFinite(option.c_str(), value_text);
    } else if (option == "--drop-visual-at-s") {
      options.drop_visual_at_s = ParsePositiveFinite(option.c_str(), value_text);
    } else if (option == "--drop-sonar-at-s") {
      options.drop_sonar_at_s = ParsePositiveFinite(option.c_str(), value_text);
    }
  }
  return options;
}

// ---------------------------------------------------------------------
// Steady-clock scheduling helpers (identical to live_ingress_smoke.cpp;
// duplicated rather than shared since neither app exposes these via a
// header -- see that file for the "no burst catch-up" scheduling contract).
// ---------------------------------------------------------------------

SteadyClock::duration PeriodForRate(const char* option, double rate_hz) {
  const long double reciprocal_s = 1.0L / static_cast<long double>(rate_hz);
  const long double maximum_s =
      std::chrono::duration<long double>(SteadyClock::duration::max()).count();
  if (!std::isfinite(reciprocal_s) || reciprocal_s > maximum_s) {
    throw std::invalid_argument(std::string(option) + " is slower than the steady clock range");
  }
  const auto period =
      std::chrono::duration_cast<SteadyClock::duration>(std::chrono::duration<long double>(reciprocal_s));
  if (period <= SteadyClock::duration::zero()) {
    throw std::invalid_argument(std::string(option) + " is faster than the steady clock resolution");
  }
  return period;
}

SteadyClock::duration RunDuration(double duration_s) {
  const long double requested_s = static_cast<long double>(duration_s);
  const long double maximum_s =
      std::chrono::duration<long double>(SteadyClock::duration::max()).count();
  if (!std::isfinite(requested_s) || requested_s > maximum_s) {
    throw std::invalid_argument("--duration-s exceeds the steady clock range");
  }
  const auto duration =
      std::chrono::duration_cast<SteadyClock::duration>(std::chrono::duration<long double>(requested_s));
  if (duration <= SteadyClock::duration::zero()) {
    throw std::invalid_argument("--duration-s is shorter than the steady clock resolution");
  }
  return duration;
}

uint64_t ExpectedTicks(SteadyClock::duration duration, SteadyClock::duration period) {
  const auto complete_periods = duration / period;
  const auto remainder = duration % period;
  return static_cast<uint64_t>(complete_periods) +
         (remainder == SteadyClock::duration::zero() ? 0U : 1U);
}

void AdvanceDeadline(SteadyClock::time_point* deadline, SteadyClock::duration period, uint64_t steps,
                     SteadyClock::time_point end) {
  if (*deadline >= end || steps == 0) return;
  const uint64_t steps_to_end = ExpectedTicks(end - *deadline, period);
  if (steps >= steps_to_end) {
    *deadline = end;
    return;
  }
  *deadline += period * static_cast<SteadyClock::duration::rep>(steps);
}

struct DeadlineStats {
  uint64_t misses = 0;
};

bool PrepareDueDeadline(SteadyClock::time_point now, SteadyClock::time_point end,
                        SteadyClock::duration period, SteadyClock::time_point* deadline,
                        DeadlineStats* stats) {
  if (*deadline >= end || *deadline > now) return false;
  if (now >= end) {
    stats->misses += ExpectedTicks(end - *deadline, period);
    *deadline = end;
    return false;
  }
  const uint64_t misses = static_cast<uint64_t>((now - *deadline) / period);
  stats->misses += misses;
  AdvanceDeadline(deadline, period, misses, end);
  return *deadline < end && *deadline <= now;
}

uw::domain::Stamp SteadyStamp(SteadyClock::time_point time) {
  const auto since_epoch = time.time_since_epoch();
  const auto seconds = std::chrono::floor<std::chrono::seconds>(since_epoch);
  const auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(since_epoch - seconds);
  uw::domain::Stamp stamp;
  stamp.set_seconds(seconds.count());
  stamp.set_nanos(static_cast<int32_t>(nanos.count()));
  return stamp;
}

uint64_t SteadyNanoseconds(SteadyClock::time_point time) {
  const auto count = std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
  return count < 0 ? 0 : static_cast<uint64_t>(count);
}

// ---------------------------------------------------------------------
// Rig, real-frontend-detectable fixtures.
// ---------------------------------------------------------------------

void AddIdentityEdge(uw::domain::RigCalibrationSnapshot* rig, const std::string& child) {
  auto* edge = rig->add_frame_tree();
  edge->mutable_parent_frame()->set_value("base_link");
  edge->mutable_child_frame()->set_value(child);
  *edge->mutable_transform() = uw::sensor_models::Pose3::Identity().ToProto();
}

uw::domain::RigCalibrationSnapshot BuildRig() {
  uw::domain::RigCalibrationSnapshot rig;
  rig.mutable_calibration_version()->set_value(kCalibrationVersion);
  for (const std::string sensor : {kLeftCamera, kRightCamera}) {
    auto* camera = rig.add_cameras();
    camera->mutable_sensor_id()->set_value(sensor);
    camera->set_width(kImageWidth);
    camera->set_height(kImageHeight);
    for (double v : {300.0, 0.0, 160.0, 0.0, 300.0, 120.0, 0.0, 0.0, 1.0}) {
      camera->add_k_matrix_row_major(v);
    }
    AddIdentityEdge(&rig, sensor + "_link");
  }
  AddIdentityEdge(&rig, std::string(kSonarSensor) + "_link");
  auto* sonar = rig.add_sonar_beam_models();
  sonar->mutable_sensor_id()->set_value(kSonarSensor);
  sonar->set_sonar_enabled(true);
  rig.add_vehicle_state_sensors()->set_value(kStateSensor);
  for (const std::string sensor : {kLeftCamera, kRightCamera, kSonarSensor, kStateSensor}) {
    (*rig.mutable_time_offset_seconds())[sensor] = 0.0;
    (*rig.mutable_time_offset_provenance())[sensor] = "measured:online_assist_smoke";
  }
  return rig;
}

void PopulateHeader(uw::domain::ObservationHeader* header, const std::string& sensor_id,
                    uint64_t sequence_id, const uw::domain::Stamp& capture_time,
                    const uw::domain::Stamp& receive_time) {
  header->mutable_observation_id()->set_value(sensor_id + "-" + std::to_string(sequence_id));
  header->mutable_sensor_id()->set_value(sensor_id);
  header->mutable_sequence_id()->set_value(sequence_id);
  *header->mutable_capture_time() = capture_time;
  *header->mutable_receive_time() = receive_time;
  header->set_clock_domain(uw::domain::CLOCK_DOMAIN_SYSTEM_REALTIME);
  header->set_receive_clock_domain(uw::domain::CLOCK_DOMAIN_SYSTEM_MONOTONIC);
  header->mutable_sensor_frame()->set_value(sensor_id + "_link");
  header->mutable_calibration_version()->set_value(kCalibrationVersion);
  header->set_validity(uw::domain::ObservationHeader::VALIDITY_OK);
  header->set_provenance("online_assist_smoke");
}

// A single green rectangle, centered on the intrinsics' cx so its detected
// bearing lands on boresight (0 rad) -- deliberately matching the sonar
// target's own bearing below. A camera's and a sonar's bearing conventions
// are only guaranteed to agree at boresight without real, jointly-solved
// intrinsics/extrinsics; see online_assist_pipeline_test.cpp's identical
// choice and its comment for why.
uw::domain::ImageFrame MakePilotImage(const std::string& sensor_id, uint64_t sequence_id,
                                      const uw::domain::Stamp& capture_time,
                                      const uw::domain::Stamp& receive_time) {
  uw::domain::ImageFrame image;
  PopulateHeader(image.mutable_header(), sensor_id, sequence_id, capture_time, receive_time);
  image.set_width(kImageWidth);
  image.set_height(kImageHeight);
  image.set_row_stride_bytes(kImageWidth * 3);
  image.set_encoding(uw::domain::ImageFrame::IMAGE_ENCODING_RGB8);
  image.set_is_rectified(true);

  constexpr std::array<uint8_t, 3> kBackground{35, 35, 35};
  constexpr std::array<uint8_t, 3> kTargetColor{20, 220, 20};  // hue ~120, satisfies aquaculture HSV gate
  constexpr uint32_t kRectX = 130, kRectY = 80, kRectW = 60, kRectH = 50;  // centered on cx=160
  std::string pixels(static_cast<std::size_t>(image.row_stride_bytes()) * kImageHeight, '\0');
  for (uint32_t y = 0; y < kImageHeight; ++y) {
    for (uint32_t x = 0; x < kImageWidth; ++x) {
      const bool inside_rect = x >= kRectX && x < kRectX + kRectW && y >= kRectY && y < kRectY + kRectH;
      const auto& color = inside_rect ? kTargetColor : kBackground;
      const std::size_t offset = static_cast<std::size_t>(y) * image.row_stride_bytes() + x * 3;
      pixels[offset] = static_cast<char>(color[0]);
      pixels[offset + 1] = static_cast<char>(color[1]);
      pixels[offset + 2] = static_cast<char>(color[2]);
    }
  }
  image.set_pixel_data(std::move(pixels));
  return image;
}

// A single sonar target, on boresight -- pairs with the pilot image's
// target above. Deliberately NOT two clusters from one frame:
// SonarTargetExtractor stamps every cluster it extracts from a frame with
// that frame's own observation_id (Task 2's own design -- see
// tests/frontends/sonar_target_extractor_test.cpp), so two clusters in the
// same frame would reach TargetAssociator::Associate() as two detections
// sharing one observation_id in the same batch. TargetAssociator rejects
// the WHOLE batch on any duplicate id within a call (a deliberate
// provenance-uniqueness invariant, not a bug) -- confirmed by running this
// generator two-cluster and watching every association in the run come
// back kInvalidInput, visual detections included. SonarTargetExtractor's
// own "keeps every cluster" behavior is already covered at the unit level;
// this integration smoke only needs one real target to prove online
// association end-to-end.
uw::domain::SonarFrame MakeSonarFrame(uint64_t sequence_id, const uw::domain::Stamp& capture_time,
                                      const uw::domain::Stamp& receive_time) {
  uw::runtime::SyntheticSonarFrameSpec spec;
  spec.num_ranges = 120;
  spec.num_beams = 90;
  spec.min_range_m = 0.0;
  spec.max_range_m = 6.0;
  spec.horizontal_fov_rad = 1.4;
  spec.background_intensity = 5;
  spec.target_intensity = 220;
  spec.target_half_width_beams = 2;
  spec.observation_id = std::string(kSonarSensor) + "-" + std::to_string(sequence_id);
  spec.sensor_id = kSonarSensor;
  spec.sensor_frame = std::string(kSonarSensor) + "_link";
  spec.provenance = "online_assist_smoke";
  // RenderSyntheticSonarFrame derives capture_time from spec.timestamp_ns
  // (a raw nanosecond count, not tied to any particular clock); overridden
  // below with the real system_clock-based capture_time this app uses
  // everywhere else, so degradation staleness checks compare like with
  // like.

  auto rendered = uw::runtime::RenderSyntheticSonarFrame(spec, /*target_range_m=*/4.0,
                                                         /*target_bearing_rad=*/0.0);
  auto& paired = rendered;

  // RenderSyntheticSonarFrame does not populate the beam-model/environment
  // fields ValidateCanonicalEvent requires (they belong to the rig/mission
  // layer, not the range-bearing geometry this generator focuses on) --
  // fill in physically reasonable constants, matching
  // apps/live_ingress_smoke.cpp's own synthetic sonar fixture.
  paired.frame.set_elevation_aperture(0.2F);
  paired.frame.set_operating_frequency_hz(750000.0);
  paired.frame.mutable_gain_metadata()->set_gain(2.0F);
  paired.frame.mutable_gain_metadata()->set_mode(1);
  paired.frame.mutable_sound_speed_assumption()->set_speed_of_sound_mps(1500.0F);
  paired.frame.mutable_sound_speed_assumption()->set_salinity_ppt(35.0F);
  paired.frame.mutable_sound_speed_assumption()->set_is_measured(false);

  auto* header = paired.frame.mutable_header();
  header->mutable_sequence_id()->set_value(sequence_id);
  *header->mutable_capture_time() = capture_time;
  *header->mutable_receive_time() = receive_time;
  header->set_clock_domain(uw::domain::CLOCK_DOMAIN_SYSTEM_REALTIME);
  header->set_receive_clock_domain(uw::domain::CLOCK_DOMAIN_SYSTEM_MONOTONIC);
  header->mutable_calibration_version()->set_value(kCalibrationVersion);
  header->set_provenance("online_assist_smoke");
  return paired.frame;
}

uw::domain::VehicleState MakeVehicleState(uint64_t sequence_id, const uw::domain::Stamp& capture_time,
                                          const uw::domain::Stamp& receive_time) {
  uw::domain::VehicleState state;
  PopulateHeader(state.mutable_header(), kStateSensor, sequence_id, capture_time, receive_time);
  for (double v : {0.0, 0.0, 0.0, 1.0}) state.add_orientation_xyzw(v);
  for (double v : {0.0, 0.0, 0.0}) state.add_angular_velocity_radps(v);
  state.set_depth_m(2.0);
  state.set_attitude_valid(true);
  state.set_depth_valid(true);
  state.set_device_health_valid(true);
  state.set_supply_voltage_v(15.2);
  state.set_supply_current_a(2.1);
  state.set_link_quality(0.95);
  for (int i = 0; i < 49; ++i) state.add_covariance_7x7_row_major(0.0);
  return state;
}

// ---------------------------------------------------------------------
// Report collection.
// ---------------------------------------------------------------------

// Observes every published state (unlike LatestAssistSink, which only
// keeps the most recent one) to answer whole-run questions: did an
// acoustic-optic fused track ever confirm, did any track go stale while
// nothing was deliberately dropped, what is the P95 publish-to-capture
// age. Single-writer (the pipeline's own consumer thread) then read after
// that thread joins -- no locking needed, unlike the general-purpose
// LatestAssistSink this app deliberately does not reuse here.
class ReportSink final : public uw::application::AssistOutputSink {
 public:
  ReportSink(SteadyClock::time_point run_start, double first_drop_s)
      : run_start_(run_start), first_drop_s_(first_drop_s) {}

  void Publish(const OperatorAssistState& state) override {
    ++publish_count_;
    age_samples_ms_.push_back(state.data_age_ms());

    const double elapsed_s =
        std::chrono::duration<double>(SteadyClock::now() - run_start_).count();
    const bool normal_window = elapsed_s < first_drop_s_;

    for (const auto& track : state.target_tracks().tracks()) {
      const bool has_visual = std::find(track.sources().begin(), track.sources().end(),
                                        uw::domain::ASSIST_SOURCE_VISUAL) != track.sources().end();
      const bool has_sonar = std::find(track.sources().begin(), track.sources().end(),
                                       uw::domain::ASSIST_SOURCE_SONAR) != track.sources().end();
      if (has_visual && has_sonar && track.status() == uw::domain::TARGET_TRACK_STATUS_CONFIRMED) {
        fused_track_ids_.insert(track.track_id().value());
      }
      if (normal_window && track.status() == uw::domain::TARGET_TRACK_STATUS_STALE) {
        stale_normal_track_ids_.insert(track.track_id().value());
      }
    }
  }

  uint64_t PublishCount() const { return publish_count_; }
  uint64_t FusedTrackCount() const { return fused_track_ids_.size(); }
  uint64_t StaleNormalTrackCount() const { return stale_normal_track_ids_.size(); }

  double AgeP95Ms() const {
    if (age_samples_ms_.empty()) return 0.0;
    std::vector<double> sorted = age_samples_ms_;
    std::sort(sorted.begin(), sorted.end());
    const std::size_t rank =
        std::min(sorted.size() - 1,
                 static_cast<std::size_t>(std::ceil(0.95 * static_cast<double>(sorted.size()))) - 1);
    return sorted[rank];
  }

 private:
  SteadyClock::time_point run_start_;
  double first_drop_s_;
  uint64_t publish_count_ = 0;
  std::vector<double> age_samples_ms_;
  std::set<std::string> fused_track_ids_;
  std::set<std::string> stale_normal_track_ids_;
};

// Wraps OnlineAssistPipeline to prove /gt/state is never actually delivered
// to it -- this app never submits a reference-plane event in the first
// place, but the counter makes that a checked fact rather than an
// assumption a future edit could silently break.
class ReferenceCountingPort final : public uw::application::PipelineInputPort {
 public:
  explicit ReferenceCountingPort(OnlineAssistPipeline& pipeline) : pipeline_(pipeline) {}

  bool OnImageFrame(const CanonicalEvent& e) override { return pipeline_.OnImageFrame(e); }
  bool OnSonarFrame(const CanonicalEvent& e) override { return pipeline_.OnSonarFrame(e); }
  bool OnImuSample(const CanonicalEvent& e) override { return pipeline_.OnImuSample(e); }
  bool OnDvlSample(const CanonicalEvent& e) override { return pipeline_.OnDvlSample(e); }
  bool OnVehicleState(const CanonicalEvent& e) override { return pipeline_.OnVehicleState(e); }
  bool OnKeyframeBoundary(const CanonicalEvent& e) override {
    return pipeline_.OnKeyframeBoundary(e);
  }
  bool OnMeasurementEvidence(const CanonicalEvent& e) override {
    return pipeline_.OnMeasurementEvidence(e);
  }
  bool OnReferenceState(const CanonicalEvent& e) override {
    ++reference_delivered_;
    return pipeline_.OnReferenceState(e);
  }
  bool OnHealthReport(const CanonicalEvent& e) override { return pipeline_.OnHealthReport(e); }
  bool OnMapEvidence(const CanonicalEvent& e) override { return pipeline_.OnMapEvidence(e); }
  bool Flush() override { return pipeline_.Flush(); }

  uint64_t ReferenceDelivered() const { return reference_delivered_; }

 private:
  OnlineAssistPipeline& pipeline_;
  uint64_t reference_delivered_ = 0;
};

class PumpThreadGuard {
 public:
  PumpThreadGuard(LiveEventSource& source, std::thread& thread) : source_(source), thread_(thread) {}
  ~PumpThreadGuard() {
    source_.Close();
    if (thread_.joinable()) thread_.join();
  }

 private:
  LiveEventSource& source_;
  std::thread& thread_;
};

struct SubmitCounts {
  uint64_t normal_submitted = 0;
  uint64_t capacity_status_violations = 0;
  uint64_t unexpected_statuses = 0;
};

void SubmitNormal(LiveEventSource& source, CanonicalEvent event, SubmitCounts* counts) {
  ++counts->normal_submitted;
  const LiveSubmitStatus status = source.Submit(std::move(event));
  if (status == LiveSubmitStatus::kAccepted) return;
  if (status == LiveSubmitStatus::kAcceptedAfterDroppingOldest ||
      status == LiveSubmitStatus::kDroppedNewest || status == LiveSubmitStatus::kOverflowRejected) {
    ++counts->capacity_status_violations;
  } else {
    ++counts->unexpected_statuses;
  }
  if (status == LiveSubmitStatus::kClosed) {
    throw std::runtime_error("live source closed while the producer was active");
  }
}

uint64_t QueueCapacityViolations(const uw::runtime::QueueStats& stats, std::size_t capacity) {
  uint64_t violations = 0;
  if (stats.current_depth > capacity) ++violations;
  if (stats.high_watermark > capacity) ++violations;
  violations += stats.dropped_oldest_count;
  violations += stats.dropped_newest_count;
  violations += stats.rejected_count;
  return violations;
}

int Run(const Options& options) {
  const auto camera_period = PeriodForRate("--camera-hz", options.camera_hz);
  const auto sonar_period = PeriodForRate("--sonar-hz", options.sonar_hz);
  const auto state_period = PeriodForRate("--state-hz", options.state_hz);
  const auto run_duration = RunDuration(options.duration_s);

  const LiveSourceConfig live_config;
  LiveEventSource source(live_config);

  OnlineAssistPipelineDependencies deps;
  deps.rig = BuildRig();
  uw::opencv_adapters::OpenCvVisualAssistFrontend visual_frontend(
      (uw::opencv_adapters::VisualAssistParams{}));
  uw::frontends::SonarCfarFrontend sonar_frontend((uw::frontends::SonarCfarFrontendParams{}));
  deps.visual_frontend = &visual_frontend;
  deps.sonar_frontend = &sonar_frontend;
  deps.dense_depth_provider = nullptr;  // dense stays disabled -- see Task 6
  deps.pipeline.dense.enabled = false;
  // Tighter than the production default (1s/0.5s in configs/defaults/
  // platform.yaml) so a --drop-*-at-s run visibly degrades within a few
  // sensor periods, matching this task's own "within three camera periods"
  // framing -- not a claim about what production should use.
  deps.pipeline.modality_stale_after_s = 0.3;
  deps.pipeline.vehicle_state_stale_after_s = 0.3;
  const auto run_start = SteadyClock::now();
  const double first_drop_s =
      std::min(options.drop_visual_at_s.value_or(std::numeric_limits<double>::infinity()),
              options.drop_sonar_at_s.value_or(std::numeric_limits<double>::infinity()));
  ReportSink report_sink(run_start, first_drop_s);
  deps.sink = &report_sink;
  deps.now = [] { return uw::domain::ToStamp(std::chrono::system_clock::now()); };

  OnlineAssistPipeline pipeline(std::move(deps));
  ReferenceCountingPort port(pipeline);

  std::promise<EventSourceReport> report_promise;
  std::future<EventSourceReport> report_future = report_promise.get_future();
  std::thread consumer([&] {
    try {
      report_promise.set_value(uw::application::PumpEvents(source, port));
    } catch (...) {
      const auto error = std::current_exception();
      source.Close();
      try {
        report_promise.set_exception(error);
      } catch (...) {
      }
    }
  });
  PumpThreadGuard guard(source, consumer);

  uint64_t source_sequence = 0;
  uint64_t camera_sequence = 0;
  uint64_t sonar_sequence = 0;
  uint64_t state_sequence = 0;
  SubmitCounts submit_counts;

  const auto start = SteadyClock::now();
  if (run_duration > SteadyClock::time_point::max() - start) {
    throw std::invalid_argument("--duration-s exceeds the remaining steady clock range");
  }
  const auto end = start + run_duration;
  auto next_camera = start;
  auto next_sonar = start;
  auto next_state = start;
  DeadlineStats camera_deadlines;
  DeadlineStats sonar_deadlines;
  DeadlineStats state_deadlines;

  while (true) {
    const auto next_deadline = std::min({next_camera, next_sonar, next_state});
    if (next_deadline >= end) break;
    std::this_thread::sleep_until(next_deadline);
    const auto now = SteadyClock::now();
    const double elapsed_s = std::chrono::duration<double>(now - start).count();

    if (PrepareDueDeadline(now, end, camera_period, &next_camera, &camera_deadlines)) {
      ++camera_sequence;
      if (!options.drop_visual_at_s.has_value() || elapsed_s < *options.drop_visual_at_s) {
        const auto capture_time = uw::domain::ToStamp(std::chrono::system_clock::now());
        const auto receive_time = SteadyStamp(now);
        SubmitNormal(source,
                     {uw::runtime::kTopicCameraLeft, SteadyNanoseconds(now), ++source_sequence,
                      MakePilotImage(kLeftCamera, camera_sequence, capture_time, receive_time)},
                     &submit_counts);
        SubmitNormal(source,
                     {uw::runtime::kTopicCameraRight, SteadyNanoseconds(now), ++source_sequence,
                      MakePilotImage(kRightCamera, camera_sequence, capture_time, receive_time)},
                     &submit_counts);
      }
      AdvanceDeadline(&next_camera, camera_period, 1, end);
    }
    if (PrepareDueDeadline(now, end, sonar_period, &next_sonar, &sonar_deadlines)) {
      ++sonar_sequence;
      if (!options.drop_sonar_at_s.has_value() || elapsed_s < *options.drop_sonar_at_s) {
        const auto capture_time = uw::domain::ToStamp(std::chrono::system_clock::now());
        const auto receive_time = SteadyStamp(now);
        SubmitNormal(source,
                     {uw::runtime::kTopicSonarFrame, SteadyNanoseconds(now), ++source_sequence,
                      MakeSonarFrame(sonar_sequence, capture_time, receive_time)},
                     &submit_counts);
      }
      AdvanceDeadline(&next_sonar, sonar_period, 1, end);
    }
    if (PrepareDueDeadline(now, end, state_period, &next_state, &state_deadlines)) {
      ++state_sequence;
      const auto capture_time = uw::domain::ToStamp(std::chrono::system_clock::now());
      const auto receive_time = SteadyStamp(now);
      SubmitNormal(source,
                   {uw::runtime::kTopicVehicleState, SteadyNanoseconds(now), ++source_sequence,
                    MakeVehicleState(state_sequence, capture_time, receive_time)},
                   &submit_counts);
      AdvanceDeadline(&next_state, state_period, 1, end);
    }
  }

  source.Close();
  if (consumer.joinable()) consumer.join();
  const EventSourceReport event_report = report_future.get();
  const auto stats = source.Stats();

  uint64_t queue_capacity_violations = submit_counts.capacity_status_violations;
  queue_capacity_violations +=
      QueueCapacityViolations(stats.localization, live_config.localization.capacity);
  queue_capacity_violations +=
      QueueCapacityViolations(stats.correction, live_config.correction.capacity);
  queue_capacity_violations += QueueCapacityViolations(stats.mapping, live_config.mapping.capacity);
  queue_capacity_violations += QueueCapacityViolations(stats.evidence, live_config.evidence.capacity);

  bool ok = true;
  ok = ok && event_report.status == EventSourceStatus::kCompleted;
  ok = ok && submit_counts.unexpected_statuses == 0;
  ok = ok && stats.semantic_rejected_count == 0;
  ok = ok && stats.duplicate_or_out_of_order_rejected_count == 0;
  ok = ok && stats.closed_rejected_count == 0;
  ok = ok && stats.sequence_gap_count == 0;
  ok = ok && stats.accepted_count == submit_counts.normal_submitted;
  ok = ok && port.ReferenceDelivered() == 0;
  ok = ok && queue_capacity_violations == 0;
  ok = ok && report_sink.StaleNormalTrackCount() == 0;
  ok = ok && report_sink.FusedTrackCount() > 0;
  ok = ok && report_sink.AgeP95Ms() < 250.0;

  std::cout << "fused_tracks=" << report_sink.FusedTrackCount()
            << " truth_delivered=" << port.ReferenceDelivered()
            << " stale_normal_tracks=" << report_sink.StaleNormalTrackCount()
            << " queue_capacity_violations=" << queue_capacity_violations
            << " result_age_p95_ms=" << report_sink.AgeP95Ms()
            << " publish_count=" << report_sink.PublishCount()
            << " camera_submitted=" << camera_sequence << " sonar_submitted=" << sonar_sequence
            << " state_submitted=" << state_sequence
            << " camera_deadline_misses=" << camera_deadlines.misses
            << " sonar_deadline_misses=" << sonar_deadlines.misses
            << " state_deadline_misses=" << state_deadlines.misses << '\n';
  std::cout.flush();
  if (!std::cout) throw std::runtime_error("failed to write online assist smoke summary");
  return ok ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    return Run(ParseOptions(argc, argv));
  } catch (const std::exception& error) {
    std::cerr << "online_assist_smoke: " << error.what() << '\n';
    return 2;
  } catch (...) {
    std::cerr << "online_assist_smoke: unknown failure\n";
    return 2;
  }
}
