#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <future>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "application/event_pump.hpp"
#include "application/pipeline_input_port.hpp"
#include "domain/domain.hpp"
#include "runtime/canonical_topics.hpp"
#include "runtime/live_event_source.hpp"

namespace {

using SteadyClock = std::chrono::steady_clock;
using uw::runtime::CanonicalEvent;
using uw::runtime::EventSourceReport;
using uw::runtime::EventSourceStatus;
using uw::runtime::LiveEventSource;
using uw::runtime::LiveSourceConfig;
using uw::runtime::LiveSubmitStatus;

struct Options {
  double duration_s = 3.0;
  double camera_hz = 20.0;
  double sonar_hz = 10.0;
  double state_hz = 50.0;
  int64_t inject_stall_ms = 0;
};

int64_t ParseNonnegativeInteger(const char* option, const char* text) {
  std::size_t parsed = 0;
  long long value = 0;
  try {
    value = std::stoll(text, &parsed);
  } catch (const std::exception&) {
    throw std::invalid_argument(std::string(option) + " requires an integer value");
  }
  if (parsed != std::string(text).size() || value < 0) {
    throw std::invalid_argument(std::string(option) + " must be nonnegative");
  }
  return static_cast<int64_t>(value);
}

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
                              option == "--inject-stall-ms";
    if (!known_option) {
      throw std::invalid_argument("unknown argument: " + option);
    }
    if (index + 1 >= argc) {
      throw std::invalid_argument(option + " requires a value");
    }
    if (option == "--duration-s") {
      options.duration_s = ParsePositiveFinite(option.c_str(), argv[index + 1]);
    } else if (option == "--camera-hz") {
      options.camera_hz = ParsePositiveFinite(option.c_str(), argv[index + 1]);
    } else if (option == "--sonar-hz") {
      options.sonar_hz = ParsePositiveFinite(option.c_str(), argv[index + 1]);
    } else if (option == "--state-hz") {
      options.state_hz = ParsePositiveFinite(option.c_str(), argv[index + 1]);
    } else if (option == "--inject-stall-ms") {
      options.inject_stall_ms = ParseNonnegativeInteger(option.c_str(), argv[index + 1]);
    }
  }
  return options;
}

SteadyClock::duration PeriodForRate(const char* option, double rate_hz) {
  const long double reciprocal_s = 1.0L / static_cast<long double>(rate_hz);
  const long double maximum_s =
      std::chrono::duration<long double>(SteadyClock::duration::max()).count();
  if (!std::isfinite(reciprocal_s) || reciprocal_s > maximum_s) {
    throw std::invalid_argument(std::string(option) +
                                " is slower than the steady clock range");
  }
  const auto period = std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<long double>(reciprocal_s));
  if (period <= SteadyClock::duration::zero()) {
    throw std::invalid_argument(std::string(option) +
                                " is faster than the steady clock resolution");
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
  const auto duration = std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<long double>(requested_s));
  if (duration <= SteadyClock::duration::zero()) {
    throw std::invalid_argument("--duration-s is shorter than the steady clock resolution");
  }
  return duration;
}

uw::domain::Stamp SteadyStamp(SteadyClock::time_point time) {
  const auto since_epoch = time.time_since_epoch();
  const auto seconds = std::chrono::floor<std::chrono::seconds>(since_epoch);
  const auto nanos =
      std::chrono::duration_cast<std::chrono::nanoseconds>(since_epoch - seconds);
  uw::domain::Stamp stamp;
  stamp.set_seconds(seconds.count());
  stamp.set_nanos(static_cast<int32_t>(nanos.count()));
  return stamp;
}

uint64_t SteadyNanoseconds(SteadyClock::time_point time) {
  const auto count =
      std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
  return count < 0 ? 0 : static_cast<uint64_t>(count);
}

uint64_t DurationNanoseconds(SteadyClock::duration duration) {
  if (duration <= SteadyClock::duration::zero()) return 0;
  const auto count = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
  return count < 0 ? 0 : static_cast<uint64_t>(count);
}

uint64_t ExpectedTicks(SteadyClock::duration duration, SteadyClock::duration period) {
  const auto complete_periods = duration / period;
  const auto remainder = duration % period;
  return static_cast<uint64_t>(complete_periods) +
         (remainder == SteadyClock::duration::zero() ? 0U : 1U);
}

void AdvanceDeadline(SteadyClock::time_point* deadline, SteadyClock::duration period,
                     uint64_t steps, SteadyClock::time_point end) {
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
  SteadyClock::duration max_lateness = SteadyClock::duration::zero();
};

bool PrepareDueDeadline(SteadyClock::time_point now, SteadyClock::time_point end,
                        SteadyClock::duration period, SteadyClock::time_point* deadline,
                        DeadlineStats* stats) {
  if (*deadline >= end || *deadline > now) return false;
  stats->max_lateness = std::max(stats->max_lateness, now - *deadline);
  if (now >= end) {
    const uint64_t misses = ExpectedTicks(end - *deadline, period);
    stats->misses += misses;
    *deadline = end;
    return false;
  }
  const uint64_t misses = static_cast<uint64_t>((now - *deadline) / period);
  stats->misses += misses;
  AdvanceDeadline(deadline, period, misses, end);
  return *deadline < end && *deadline <= now;
}

void PopulateHeader(uw::domain::ObservationHeader* header, const std::string& sensor_id,
                    uint64_t sequence_id, const uw::domain::Stamp& capture_time,
                    const uw::domain::Stamp& receive_time) {
  header->mutable_observation_id()->set_value(sensor_id + "-" +
                                               std::to_string(sequence_id));
  header->mutable_sensor_id()->set_value(sensor_id);
  header->mutable_sequence_id()->set_value(sequence_id);
  header->mutable_capture_time()->CopyFrom(capture_time);
  header->mutable_receive_time()->CopyFrom(receive_time);
  header->set_clock_domain(uw::domain::CLOCK_DOMAIN_SYSTEM_REALTIME);
  header->set_receive_clock_domain(uw::domain::CLOCK_DOMAIN_SYSTEM_MONOTONIC);
  header->mutable_sensor_frame()->set_value(sensor_id + "-frame");
  header->mutable_calibration_version()->set_value("live-smoke-calibration-v1");
  header->set_validity(uw::domain::ObservationHeader::VALIDITY_OK);
  header->set_provenance("live_ingress_smoke");
}

CanonicalEvent MakeImageEvent(const char* topic, const std::string& sensor_id,
                              uint64_t sequence_id, uint64_t source_sequence,
                              const uw::domain::Stamp& capture_time,
                              const uw::domain::Stamp& receive_time, uint64_t log_time_ns) {
  uw::domain::ImageFrame frame;
  PopulateHeader(frame.mutable_header(), sensor_id, sequence_id, capture_time, receive_time);
  frame.set_width(4);
  frame.set_height(3);
  frame.set_row_stride_bytes(4);
  frame.set_encoding(uw::domain::ImageFrame::IMAGE_ENCODING_MONO8);
  frame.set_pixel_data(std::string(12, static_cast<char>(sequence_id & 0xffU)));
  frame.set_is_rectified(true);
  frame.set_exposure_seconds(0.005);
  return {topic, log_time_ns, source_sequence, std::move(frame)};
}

CanonicalEvent MakeSonarEvent(uint64_t sequence_id, uint64_t source_sequence,
                              const uw::domain::Stamp& capture_time,
                              const uw::domain::Stamp& receive_time, uint64_t log_time_ns) {
  uw::domain::SonarFrame frame;
  PopulateHeader(frame.mutable_header(), "sonar-forward", sequence_id, capture_time,
                 receive_time);
  frame.set_num_ranges(2);
  frame.set_num_beams(3);
  frame.set_encoding(uw::domain::SonarFrame::ENCODING_UINT8_GRAY);
  frame.set_intensity_tensor(std::string(6, static_cast<char>(sequence_id & 0xffU)));
  for (float range : {0.5F, 1.0F, 1.5F}) frame.add_range_bins(range);
  for (float azimuth : {-0.5F, 0.0F, 0.5F}) frame.add_azimuth_angles(azimuth);
  frame.set_min_range(0.5F);
  frame.set_max_range(1.5F);
  frame.set_range_resolution(0.5F);
  frame.set_horizontal_fov(1.0F);
  frame.set_elevation_aperture(0.2F);
  frame.mutable_gain_metadata()->set_gain(2.0F);
  frame.mutable_gain_metadata()->set_mode(1);
  frame.mutable_sound_speed_assumption()->set_speed_of_sound_mps(1500.0F);
  frame.mutable_sound_speed_assumption()->set_salinity_ppt(35.0F);
  frame.mutable_sound_speed_assumption()->set_is_measured(false);
  frame.set_operating_frequency_hz(750000.0);
  return {uw::runtime::kTopicSonarFrame, log_time_ns, source_sequence, std::move(frame)};
}

CanonicalEvent MakeVehicleStateEvent(uint64_t sequence_id, uint64_t source_sequence,
                                     const uw::domain::Stamp& capture_time,
                                     const uw::domain::Stamp& receive_time,
                                     uint64_t log_time_ns) {
  uw::domain::VehicleState state;
  PopulateHeader(state.mutable_header(), "rov-state", sequence_id, capture_time,
                 receive_time);
  for (double value : {0.0, 0.0, 0.0, 1.0}) state.add_orientation_xyzw(value);
  for (double value : {0.0, 0.0, 0.01}) state.add_angular_velocity_radps(value);
  state.set_depth_m(5.0);
  state.set_attitude_valid(true);
  state.set_depth_valid(true);
  state.set_leak_detected(false);
  state.set_supply_voltage_v(15.2);
  state.set_supply_current_a(2.1);
  state.set_link_quality(0.95);
  state.set_device_health_valid(true);
  return {uw::runtime::kTopicVehicleState, log_time_ns, source_sequence,
          std::move(state)};
}

CanonicalEvent MakeReferenceEvent(uint64_t source_sequence, uint64_t log_time_ns) {
  uw::domain::StateSnapshot state;
  return {uw::runtime::kTopicGtState, log_time_ns, source_sequence, std::move(state)};
}

class CountingPort final : public uw::application::PipelineInputPort {
 public:
  bool OnImageFrame(const CanonicalEvent& event) override {
    ++image_count;
    if (event.topic == uw::runtime::kTopicCameraLeft) {
      ++left_image_count;
    } else if (event.topic == uw::runtime::kTopicCameraRight) {
      ++right_image_count;
    } else {
      ++unexpected_count;
    }
    return true;
  }

  bool OnSonarFrame(const CanonicalEvent&) override {
    ++sonar_count;
    return true;
  }

  bool OnImuSample(const CanonicalEvent&) override {
    ++imu_count;
    return true;
  }

  bool OnDvlSample(const CanonicalEvent&) override {
    ++dvl_count;
    return true;
  }

  bool OnVehicleState(const CanonicalEvent&) override {
    ++vehicle_state_count;
    return true;
  }

  bool OnKeyframeBoundary(const CanonicalEvent&) override {
    ++keyframe_boundary_count;
    return true;
  }

  bool OnMeasurementEvidence(const CanonicalEvent&) override {
    ++measurement_evidence_count;
    return true;
  }

  bool OnReferenceState(const CanonicalEvent&) override {
    ++reference_count;
    return true;
  }

  bool OnHealthReport(const CanonicalEvent&) override {
    ++health_count;
    return true;
  }

  bool OnMapEvidence(const CanonicalEvent&) override {
    ++map_evidence_count;
    return true;
  }

  bool Flush() override {
    ++flush_count;
    return true;
  }

  uint64_t RawCount() const {
    return image_count + sonar_count + imu_count + dvl_count + vehicle_state_count +
           keyframe_boundary_count;
  }

  uint64_t left_image_count = 0;
  uint64_t right_image_count = 0;
  uint64_t image_count = 0;
  uint64_t sonar_count = 0;
  uint64_t imu_count = 0;
  uint64_t dvl_count = 0;
  uint64_t vehicle_state_count = 0;
  uint64_t keyframe_boundary_count = 0;
  uint64_t measurement_evidence_count = 0;
  uint64_t reference_count = 0;
  uint64_t health_count = 0;
  uint64_t map_evidence_count = 0;
  uint64_t unexpected_count = 0;
  uint64_t flush_count = 0;
};

class PumpThreadGuard {
 public:
  PumpThreadGuard(LiveEventSource& source, std::thread& thread)
      : source_(source), thread_(thread) {}

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
      status == LiveSubmitStatus::kDroppedNewest ||
      status == LiveSubmitStatus::kOverflowRejected) {
    ++counts->capacity_status_violations;
  } else {
    ++counts->unexpected_statuses;
  }
  if (status == LiveSubmitStatus::kClosed) {
    throw std::runtime_error("live source closed while the producer was active");
  }
}

uint64_t QueueCapacityViolations(const uw::runtime::QueueStats& stats,
                                 std::size_t capacity) {
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

  const LiveSourceConfig config;
  LiveEventSource source(config);
  CountingPort port;
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
  SubmitCounts submit_counts;

  const auto injection_receive = SteadyClock::now();
  const auto injection_capture = std::chrono::system_clock::now();
  auto invalid_image = MakeImageEvent(
      uw::runtime::kTopicCameraLeft, "camera-invalid", 1, ++source_sequence,
      uw::domain::ToStamp(injection_capture), SteadyStamp(injection_receive),
      SteadyNanoseconds(injection_receive));
  std::get<uw::domain::ImageFrame>(invalid_image.payload)
      .mutable_header()
      ->clear_calibration_version();
  const LiveSubmitStatus invalid_status = source.Submit(std::move(invalid_image));
  const LiveSubmitStatus reference_status = source.Submit(
      MakeReferenceEvent(++source_sequence, SteadyNanoseconds(SteadyClock::now())));

  uint64_t camera_sequence = 0;
  uint64_t sonar_sequence = 0;
  uint64_t state_sequence = 0;
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
  if (options.inject_stall_ms > 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(options.inject_stall_ms));
  }

  while (true) {
    const auto next_deadline = std::min({next_camera, next_sonar, next_state});
    if (next_deadline >= end) break;
    std::this_thread::sleep_until(next_deadline);
    const auto now = SteadyClock::now();

    if (PrepareDueDeadline(now, end, camera_period, &next_camera, &camera_deadlines)) {
      ++camera_sequence;
      const auto capture_time = uw::domain::ToStamp(std::chrono::system_clock::now());
      const auto receive_time = SteadyStamp(now);
      const auto log_time_ns = SteadyNanoseconds(now);
      SubmitNormal(source,
                   MakeImageEvent(uw::runtime::kTopicCameraLeft, "camera-left",
                                  camera_sequence, ++source_sequence, capture_time,
                                  receive_time, log_time_ns),
                   &submit_counts);
      SubmitNormal(source,
                   MakeImageEvent(uw::runtime::kTopicCameraRight, "camera-right",
                                  camera_sequence, ++source_sequence, capture_time,
                                  receive_time, log_time_ns),
                   &submit_counts);
      AdvanceDeadline(&next_camera, camera_period, 1, end);
    }
    if (PrepareDueDeadline(now, end, sonar_period, &next_sonar, &sonar_deadlines)) {
      ++sonar_sequence;
      const auto capture_time = uw::domain::ToStamp(std::chrono::system_clock::now());
      const auto receive_time = SteadyStamp(now);
      SubmitNormal(source,
                   MakeSonarEvent(sonar_sequence, ++source_sequence, capture_time,
                                  receive_time, SteadyNanoseconds(now)),
                   &submit_counts);
      AdvanceDeadline(&next_sonar, sonar_period, 1, end);
    }
    if (PrepareDueDeadline(now, end, state_period, &next_state, &state_deadlines)) {
      ++state_sequence;
      const auto capture_time = uw::domain::ToStamp(std::chrono::system_clock::now());
      const auto receive_time = SteadyStamp(now);
      SubmitNormal(source,
                   MakeVehicleStateEvent(state_sequence, ++source_sequence, capture_time,
                                         receive_time, SteadyNanoseconds(now)),
                   &submit_counts);
      AdvanceDeadline(&next_state, state_period, 1, end);
    }
  }

  source.Close();
  if (consumer.joinable()) consumer.join();
  const EventSourceReport report = report_future.get();
  const auto stats = source.Stats();

  uint64_t queue_capacity_violations = submit_counts.capacity_status_violations;
  queue_capacity_violations += QueueCapacityViolations(stats.localization,
                                                       config.localization.capacity);
  queue_capacity_violations +=
      QueueCapacityViolations(stats.correction, config.correction.capacity);
  queue_capacity_violations += QueueCapacityViolations(stats.mapping, config.mapping.capacity);
  queue_capacity_violations += QueueCapacityViolations(stats.evidence, config.evidence.capacity);

  const uint64_t other_delivered =
      port.imu_count + port.dvl_count + port.measurement_evidence_count +
      port.health_count + port.map_evidence_count + port.unexpected_count;
  const uint64_t camera_expected = ExpectedTicks(run_duration, camera_period);
  const uint64_t sonar_expected = ExpectedTicks(run_duration, sonar_period);
  const uint64_t state_expected = ExpectedTicks(run_duration, state_period);
  const uint64_t rate_count_violations =
      static_cast<uint64_t>(port.left_image_count != camera_expected) +
      static_cast<uint64_t>(port.right_image_count != camera_expected) +
      static_cast<uint64_t>(port.sonar_count != sonar_expected) +
      static_cast<uint64_t>(port.vehicle_state_count != state_expected);
  const uint64_t deadline_misses =
      camera_deadlines.misses * 2 + sonar_deadlines.misses + state_deadlines.misses;
  bool ok = true;
  ok = ok && invalid_status == LiveSubmitStatus::kSemanticRejected;
  ok = ok && reference_status == LiveSubmitStatus::kReferenceRejected;
  ok = ok && stats.semantic_rejected_count == 1;
  ok = ok && stats.reference_rejected_count == 1;
  ok = ok && report.reference_rejected_count == 1;
  ok = ok && report.status == EventSourceStatus::kCompleted;
  ok = ok && port.flush_count == 1;
  ok = ok && port.reference_count == 0;
  ok = ok && queue_capacity_violations == 0;
  ok = ok && submit_counts.unexpected_statuses == 0;
  ok = ok && stats.accepted_after_dropping_oldest_count == 0;
  ok = ok && stats.dropped_newest_count == 0;
  ok = ok && stats.overflow_rejected_count == 0;
  ok = ok && stats.duplicate_or_out_of_order_rejected_count == 0;
  ok = ok && stats.closed_rejected_count == 0;
  ok = ok && stats.sequence_gap_count == 0;
  ok = ok && stats.accepted_count == submit_counts.normal_submitted;
  ok = ok && stats.submit_attempt_count == submit_counts.normal_submitted + 2;
  ok = ok && report.messages_seen == submit_counts.normal_submitted;
  ok = ok && report.events_emitted == submit_counts.normal_submitted;
  ok = ok && port.RawCount() == submit_counts.normal_submitted;
  ok = ok && port.left_image_count > 0 && port.left_image_count == port.right_image_count;
  ok = ok && port.sonar_count > 0 && port.vehicle_state_count > 0;
  ok = ok && other_delivered == 0;
  ok = ok && rate_count_violations == 0;
  ok = ok && deadline_misses == 0;

  std::cout << "reference_delivered=" << port.reference_count
            << " semantic_rejected=" << stats.semantic_rejected_count
            << " queue_capacity_violations=" << queue_capacity_violations
            << " flush_count=" << port.flush_count
            << " reference_rejected=" << stats.reference_rejected_count
            << " submitted=" << submit_counts.normal_submitted
            << " delivered=" << port.RawCount()
            << " left_delivered=" << port.left_image_count
            << " right_delivered=" << port.right_image_count
            << " sonar_delivered=" << port.sonar_count
            << " state_delivered=" << port.vehicle_state_count
            << " left_expected=" << camera_expected
            << " left_actual=" << port.left_image_count
            << " right_expected=" << camera_expected
            << " right_actual=" << port.right_image_count
            << " sonar_expected=" << sonar_expected
            << " sonar_actual=" << port.sonar_count
            << " state_expected=" << state_expected
            << " state_actual=" << port.vehicle_state_count
            << " rate_count_violations=" << rate_count_violations
            << " deadline_misses=" << deadline_misses
            << " left_max_lateness_ns=" << DurationNanoseconds(camera_deadlines.max_lateness)
            << " right_max_lateness_ns=" << DurationNanoseconds(camera_deadlines.max_lateness)
            << " sonar_max_lateness_ns=" << DurationNanoseconds(sonar_deadlines.max_lateness)
            << " state_max_lateness_ns=" << DurationNanoseconds(state_deadlines.max_lateness)
            << " high_water_localization=" << stats.localization.high_watermark
            << " high_water_correction=" << stats.correction.high_watermark
            << " high_water_mapping=" << stats.mapping.high_watermark << '\n';
  std::cout.flush();
  if (!std::cout) {
    throw std::runtime_error("failed to write live ingress summary");
  }
  return ok ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    return Run(ParseOptions(argc, argv));
  } catch (const std::exception& error) {
    std::cerr << "live_ingress_smoke: " << error.what() << '\n';
    return 2;
  } catch (...) {
    std::cerr << "live_ingress_smoke: unknown failure\n";
    return 2;
  }
}
