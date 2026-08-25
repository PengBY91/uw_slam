#include "runtime/live_event_source.hpp"

#include <chrono>
#include <future>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "domain/domain.hpp"
#include "runtime/canonical_topics.hpp"

namespace {

using uw::runtime::CanonicalEvent;
using uw::runtime::EventSourceStatus;
using uw::runtime::LiveEventSource;
using uw::runtime::LiveSourceConfig;
using uw::runtime::LiveSubmitStatus;
using uw::runtime::OverflowPolicy;

void PopulateValidHeader(uw::domain::ObservationHeader* header,
                         const std::string& sensor_id, uint64_t sequence_id,
                         const std::string& calibration_version = "calibration-v1") {
  header->mutable_observation_id()->set_value(sensor_id + "-" + std::to_string(sequence_id));
  header->mutable_sensor_id()->set_value(sensor_id);
  header->mutable_sequence_id()->set_value(sequence_id);
  header->mutable_sensor_frame()->set_value(sensor_id + "-frame");
  header->mutable_calibration_version()->set_value(calibration_version);
  header->mutable_capture_time()->set_seconds(10);
  header->mutable_capture_time()->set_nanos(100);
  header->mutable_receive_time()->set_seconds(10);
  header->mutable_receive_time()->set_nanos(200);
  header->set_clock_domain(uw::domain::CLOCK_DOMAIN_SENSOR_HARDWARE);
  header->set_receive_clock_domain(uw::domain::CLOCK_DOMAIN_SYSTEM_MONOTONIC);
  header->set_validity(uw::domain::ObservationHeader::VALIDITY_OK);
}

CanonicalEvent MakeImageEvent(uint64_t source_sequence, uint64_t sequence_id,
                              const std::string& sensor_id = "camera-left",
                              const std::string& calibration_version = "calibration-v1") {
  uw::domain::ImageFrame frame;
  PopulateValidHeader(frame.mutable_header(), sensor_id, sequence_id, calibration_version);
  frame.set_width(2);
  frame.set_height(2);
  frame.set_row_stride_bytes(2);
  frame.set_encoding(uw::domain::ImageFrame::IMAGE_ENCODING_MONO8);
  frame.set_pixel_data(std::string(4, '\0'));
  return {uw::runtime::kTopicCameraLeft, source_sequence, source_sequence, std::move(frame)};
}

CanonicalEvent MakeSonarEvent(uint64_t source_sequence, uint64_t sequence_id,
                              const std::string& sensor_id = "sonar") {
  uw::domain::SonarFrame frame;
  PopulateValidHeader(frame.mutable_header(), sensor_id, sequence_id);
  frame.set_num_ranges(2);
  frame.set_num_beams(3);
  frame.set_encoding(uw::domain::SonarFrame::ENCODING_UINT8_GRAY);
  frame.set_intensity_tensor(std::string(6, '\0'));
  for (float range : {0.5f, 1.0f, 1.5f}) frame.add_range_bins(range);
  for (float azimuth : {-0.5f, 0.0f, 0.5f}) frame.add_azimuth_angles(azimuth);
  frame.set_min_range(0.5f);
  frame.set_max_range(1.5f);
  frame.set_range_resolution(0.5f);
  frame.set_horizontal_fov(1.0f);
  frame.set_elevation_aperture(0.2f);
  frame.mutable_gain_metadata()->set_gain(2.0f);
  frame.mutable_sound_speed_assumption()->set_speed_of_sound_mps(1500.0f);
  frame.set_operating_frequency_hz(750000.0);
  return {uw::runtime::kTopicSonarFrame, source_sequence, source_sequence, std::move(frame)};
}

CanonicalEvent MakeVehicleEvent(uint64_t source_sequence, uint64_t sequence_id,
                                const std::string& sensor_id = "vehicle",
                                const std::string& calibration_version = "calibration-v1") {
  uw::domain::VehicleState state;
  PopulateValidHeader(state.mutable_header(), sensor_id, sequence_id, calibration_version);
  for (double value : {0.0, 0.0, 0.0, 1.0}) state.add_orientation_xyzw(value);
  for (double value : {0.1, 0.2, 0.3}) state.add_angular_velocity_radps(value);
  state.set_attitude_valid(true);
  state.set_depth_valid(true);
  state.set_depth_m(0.0);
  state.set_device_health_valid(true);
  state.set_supply_voltage_v(15.0);
  state.set_supply_current_a(2.0);
  state.set_link_quality(0.75);
  return {uw::runtime::kTopicVehicleState, source_sequence, source_sequence, std::move(state)};
}

CanonicalEvent MakeHealthEvent(uint64_t source_sequence) {
  uw::domain::HealthReport report;
  report.set_component_id("component");
  report.set_status(uw::domain::HealthReport::STATUS_HEALTHY);
  return {uw::runtime::kTopicHealth, source_sequence, source_sequence, std::move(report)};
}

CanonicalEvent MakeGroundTruthEvent() {
  uw::domain::StateSnapshot snapshot;
  return {uw::runtime::kTopicGtState, 1, 1, std::move(snapshot)};
}

TEST(LiveEventSource, UsesTheFullWeightedScheduleWhileEveryLaneHasBacklog) {
  auto config = LiveSourceConfig::ForTest();
  config.localization = {32, OverflowPolicy::kReject};
  config.correction = {16, OverflowPolicy::kReject};
  config.mapping = {8, OverflowPolicy::kReject};
  config.evidence = {4, OverflowPolicy::kReject};
  LiveEventSource source(config);

  for (uint64_t sequence = 1; sequence <= 16; ++sequence) {
    ASSERT_EQ(source.Submit(MakeVehicleEvent(100 + sequence, sequence)),
              LiveSubmitStatus::kAccepted);
  }
  for (uint64_t sequence = 1; sequence <= 8; ++sequence) {
    ASSERT_EQ(source.Submit(MakeSonarEvent(200 + sequence, sequence)),
              LiveSubmitStatus::kAccepted);
  }
  for (uint64_t sequence = 1; sequence <= 4; ++sequence) {
    ASSERT_EQ(source.Submit(MakeImageEvent(300 + sequence, sequence)),
              LiveSubmitStatus::kAccepted);
  }
  ASSERT_EQ(source.Submit(MakeHealthEvent(401)), LiveSubmitStatus::kAccepted);
  ASSERT_EQ(source.Submit(MakeHealthEvent(402)), LiveSubmitStatus::kAccepted);
  source.Close();

  std::vector<uint64_t> delivered;
  const auto report = source.Run([&](const CanonicalEvent& event) {
    delivered.push_back(event.source_sequence);
    return true;
  });

  const std::vector<uint64_t> expected{
      101, 102, 103, 104, 105, 106, 107, 108, 201, 202,
      203, 204, 301, 302, 401, 109, 110, 111, 112, 113,
      114, 115, 116, 205, 206, 207, 208, 303, 304, 402};
  EXPECT_EQ(delivered, expected);
  EXPECT_EQ(report.status, EventSourceStatus::kCompleted);
}

TEST(LiveEventSource, RoutesEveryAlgorithmInputKindToAnExplicitLane) {
  auto config = LiveSourceConfig::ForTest();
  LiveEventSource source(config);

  uw::domain::ImuSample imu;
  PopulateValidHeader(imu.mutable_header(), "imu", 1);
  EXPECT_EQ(source.Submit({uw::runtime::kTopicImu, 1, 1, std::move(imu)}),
            LiveSubmitStatus::kAccepted);

  uw::domain::DvlSample dvl;
  PopulateValidHeader(dvl.mutable_header(), "dvl", 1);
  EXPECT_EQ(source.Submit({uw::runtime::kTopicDvl, 2, 2, std::move(dvl)}),
            LiveSubmitStatus::kAccepted);
  EXPECT_EQ(source.Submit(MakeVehicleEvent(3, 1)), LiveSubmitStatus::kAccepted);
  EXPECT_EQ(source.Submit(MakeSonarEvent(4, 1)), LiveSubmitStatus::kAccepted);
  EXPECT_EQ(source.Submit(MakeImageEvent(5, 1)), LiveSubmitStatus::kAccepted);

  auto right_image = MakeImageEvent(6, 1, "camera-right");
  right_image.topic = uw::runtime::kTopicCameraRight;
  EXPECT_EQ(source.Submit(std::move(right_image)), LiveSubmitStatus::kAccepted);

  uw::domain::MeasurementEvidence depth;
  EXPECT_EQ(source.Submit({uw::runtime::kTopicEvidenceDepth, 7, 7, std::move(depth)}),
            LiveSubmitStatus::kAccepted);
  uw::domain::MeasurementEvidence relative_pose;
  EXPECT_EQ(source.Submit(
                {uw::runtime::kTopicEvidenceRelativePose, 8, 8, std::move(relative_pose)}),
            LiveSubmitStatus::kAccepted);
  EXPECT_EQ(source.Submit(MakeHealthEvent(9)), LiveSubmitStatus::kAccepted);
  uw::domain::MapEvidence map;
  EXPECT_EQ(source.Submit({uw::runtime::kTopicEvidenceMap, 10, 10, std::move(map)}),
            LiveSubmitStatus::kAccepted);

  const auto stats = source.Stats();
  EXPECT_EQ(stats.localization.enqueued_count, 3u);
  EXPECT_EQ(stats.correction.enqueued_count, 1u);
  EXPECT_EQ(stats.mapping.enqueued_count, 2u);
  EXPECT_EQ(stats.evidence.enqueued_count, 4u);
  source.Close();
}

TEST(LiveEventSource, CameraDropOldestKeepsTheTwoNewestEvents) {
  auto config = LiveSourceConfig::ForTest();
  config.mapping = {2, OverflowPolicy::kDropOldest};
  LiveEventSource source(config);

  EXPECT_EQ(source.Submit(MakeImageEvent(1, 1)), LiveSubmitStatus::kAccepted);
  EXPECT_EQ(source.Submit(MakeImageEvent(2, 2)), LiveSubmitStatus::kAccepted);
  EXPECT_EQ(source.Submit(MakeImageEvent(3, 3)),
            LiveSubmitStatus::kAcceptedAfterDroppingOldest);
  source.Close();

  std::vector<uint64_t> sequences;
  source.Run([&](const CanonicalEvent& event) {
    sequences.push_back(event.source_sequence);
    return true;
  });
  EXPECT_EQ(sequences, (std::vector<uint64_t>{2, 3}));
  const auto stats = source.Stats();
  EXPECT_EQ(stats.accepted_count, 3u);
  EXPECT_EQ(stats.accepted_after_dropping_oldest_count, 1u);
  EXPECT_EQ(stats.mapping.dropped_oldest_count, 1u);
}

TEST(LiveEventSource, VehicleStateOverflowIsRejected) {
  auto config = LiveSourceConfig::ForTest();
  config.localization = {1, OverflowPolicy::kReject};
  LiveEventSource source(config);

  EXPECT_EQ(source.Submit(MakeVehicleEvent(1, 1)), LiveSubmitStatus::kAccepted);
  EXPECT_EQ(source.Submit(MakeVehicleEvent(2, 2)), LiveSubmitStatus::kOverflowRejected);
  EXPECT_EQ(source.Stats().overflow_rejected_count, 1u);
  EXPECT_EQ(source.Stats().localization.rejected_count, 1u);
  source.Close();
}

TEST(LiveEventSource, RejectsInvalidRawHeaderAndUnknownTopicAsSemanticErrors) {
  LiveEventSource source(LiveSourceConfig::ForTest());
  auto invalid = MakeImageEvent(1, 1);
  std::get<uw::domain::ImageFrame>(invalid.payload)
      .mutable_header()->mutable_sensor_id()->clear_value();
  EXPECT_EQ(source.Submit(std::move(invalid)), LiveSubmitStatus::kSemanticRejected);

  uw::domain::HealthReport report;
  EXPECT_EQ(source.Submit({"/unknown", 2, 2, std::move(report)}),
            LiveSubmitStatus::kSemanticRejected);
  EXPECT_EQ(source.Stats().semantic_rejected_count, 2u);
  source.Close();
}

TEST(LiveEventSource, RejectsReferenceTruthBeforeAlgorithmConsumer) {
  LiveEventSource source(LiveSourceConfig::ForTest());
  EXPECT_EQ(source.Submit(MakeGroundTruthEvent()), LiveSubmitStatus::kReferenceRejected);
  source.Close();

  int delivered = 0;
  const auto report = source.Run([&](const CanonicalEvent&) {
    ++delivered;
    return true;
  });
  EXPECT_EQ(delivered, 0);
  EXPECT_EQ(report.reference_rejected_count, 1u);
  EXPECT_EQ(source.Stats().reference_rejected_count, 1u);
}

TEST(LiveEventSource, RejectsDuplicateAndDecreasingSequencesPerSensor) {
  LiveEventSource source(LiveSourceConfig::ForTest());
  EXPECT_EQ(source.Submit(MakeImageEvent(1, 10, "camera-a")), LiveSubmitStatus::kAccepted);
  EXPECT_EQ(source.Submit(MakeImageEvent(2, 10, "camera-a")),
            LiveSubmitStatus::kDuplicateOrOutOfOrderRejected);
  EXPECT_EQ(source.Submit(MakeImageEvent(3, 9, "camera-a")),
            LiveSubmitStatus::kDuplicateOrOutOfOrderRejected);
  EXPECT_EQ(source.Submit(MakeImageEvent(4, 10, "camera-b")), LiveSubmitStatus::kAccepted);
  EXPECT_EQ(source.Submit(MakeImageEvent(5, 9, "camera-b")),
            LiveSubmitStatus::kDuplicateOrOutOfOrderRejected);
  EXPECT_EQ(source.Stats().duplicate_or_out_of_order_rejected_count, 3u);
  source.Close();
}

TEST(LiveEventSource, CountsForwardGapsAndCalibrationChangeResetsBaseline) {
  LiveEventSource source(LiveSourceConfig::ForTest());
  EXPECT_EQ(source.Submit(MakeImageEvent(1, 2, "camera", "cal-v1")),
            LiveSubmitStatus::kAccepted);
  EXPECT_EQ(source.Submit(MakeImageEvent(2, 5, "camera", "cal-v1")),
            LiveSubmitStatus::kAccepted);
  EXPECT_EQ(source.Stats().sequence_gap_count, 2u);
  EXPECT_EQ(source.Submit(MakeImageEvent(3, 1, "camera", "cal-v2")),
            LiveSubmitStatus::kAccepted);
  EXPECT_EQ(source.Stats().sequence_gap_count, 2u);
  source.Close();
}

TEST(LiveEventSource, RejectedOverflowDoesNotCommitSequenceBaselineOrGap) {
  auto config = LiveSourceConfig::ForTest();
  config.localization = {1, OverflowPolicy::kReject};
  LiveEventSource source(config);
  ASSERT_EQ(source.Submit(MakeVehicleEvent(1, 1, "blocker")), LiveSubmitStatus::kAccepted);

  EXPECT_EQ(source.Submit(MakeVehicleEvent(2, 10, "target")),
            LiveSubmitStatus::kOverflowRejected);
  EXPECT_EQ(source.Submit(MakeVehicleEvent(3, 10, "target")),
            LiveSubmitStatus::kOverflowRejected);
  EXPECT_EQ(source.Submit(MakeVehicleEvent(4, 11, "target")),
            LiveSubmitStatus::kOverflowRejected);
  EXPECT_EQ(source.Stats().duplicate_or_out_of_order_rejected_count, 0u);
  EXPECT_EQ(source.Stats().sequence_gap_count, 0u);
  source.Close();
}

TEST(LiveEventSource, DroppedNewestDoesNotCommitSequenceBaselineOrGap) {
  auto config = LiveSourceConfig::ForTest();
  config.mapping = {1, OverflowPolicy::kDropNewest};
  LiveEventSource source(config);
  ASSERT_EQ(source.Submit(MakeImageEvent(1, 1, "blocker")), LiveSubmitStatus::kAccepted);

  EXPECT_EQ(source.Submit(MakeImageEvent(2, 10, "target")),
            LiveSubmitStatus::kDroppedNewest);
  EXPECT_EQ(source.Submit(MakeImageEvent(3, 10, "target")),
            LiveSubmitStatus::kDroppedNewest);
  EXPECT_EQ(source.Submit(MakeImageEvent(4, 11, "target")),
            LiveSubmitStatus::kDroppedNewest);
  EXPECT_EQ(source.Stats().dropped_newest_count, 3u);
  EXPECT_EQ(source.Stats().duplicate_or_out_of_order_rejected_count, 0u);
  EXPECT_EQ(source.Stats().sequence_gap_count, 0u);
  source.Close();
}

TEST(LiveEventSource, BlockedRunIsWokenByClose) {
  LiveEventSource source(LiveSourceConfig::ForTest());
  auto future = std::async(std::launch::async, [&] {
    return source.Run([](const CanonicalEvent&) { return true; });
  });

  EXPECT_EQ(future.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);
  source.Close();
  ASSERT_EQ(future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(future.get().status, EventSourceStatus::kCompleted);
}

TEST(LiveEventSource, CloseDrainsAcceptedEventsExactlyOnceAndRejectsLaterSubmit) {
  LiveEventSource source(LiveSourceConfig::ForTest());
  EXPECT_EQ(source.Submit(MakeVehicleEvent(1, 1)), LiveSubmitStatus::kAccepted);
  EXPECT_EQ(source.Submit(MakeSonarEvent(2, 1)), LiveSubmitStatus::kAccepted);
  EXPECT_EQ(source.Submit(MakeImageEvent(3, 1)), LiveSubmitStatus::kAccepted);
  EXPECT_EQ(source.Submit(MakeHealthEvent(4)), LiveSubmitStatus::kAccepted);
  source.Close();
  EXPECT_EQ(source.Submit(MakeHealthEvent(5)), LiveSubmitStatus::kClosed);

  std::vector<uint64_t> delivered;
  const auto report = source.Run([&](const CanonicalEvent& event) {
    delivered.push_back(event.source_sequence);
    return true;
  });
  EXPECT_EQ(report.status, EventSourceStatus::kCompleted);
  EXPECT_EQ(report.messages_seen, 4u);
  EXPECT_EQ(report.events_emitted, 4u);
  EXPECT_EQ(delivered.size(), 4u);
  EXPECT_EQ(std::set<uint64_t>(delivered.begin(), delivered.end()).size(), 4u);
  EXPECT_EQ(source.Stats().closed_rejected_count, 1u);
}

TEST(LiveEventSource, ConsumerFalseStopsImmediatelyWithoutDrainingLaterEvents) {
  LiveEventSource source(LiveSourceConfig::ForTest());
  ASSERT_EQ(source.Submit(MakeVehicleEvent(1, 1)), LiveSubmitStatus::kAccepted);
  ASSERT_EQ(source.Submit(MakeVehicleEvent(2, 2)), LiveSubmitStatus::kAccepted);
  ASSERT_EQ(source.Submit(MakeVehicleEvent(3, 3)), LiveSubmitStatus::kAccepted);
  source.Close();

  const auto report = source.Run([](const CanonicalEvent&) { return false; });
  EXPECT_EQ(report.status, EventSourceStatus::kStoppedByConsumer);
  EXPECT_EQ(report.messages_seen, 1u);
  EXPECT_EQ(report.events_emitted, 1u);
  EXPECT_EQ(source.Stats().localization.current_depth, 2u);
}

TEST(LiveEventSource, RunIsSingleShot) {
  LiveEventSource source(LiveSourceConfig::ForTest());
  source.Close();
  EXPECT_EQ(source.Run([](const CanonicalEvent&) { return true; }).status,
            EventSourceStatus::kCompleted);
  EXPECT_THROW(source.Run([](const CanonicalEvent&) { return true; }), std::logic_error);
}

TEST(LiveEventSource, ZeroCapacityIsRejectedAtConstruction) {
  auto config = LiveSourceConfig::ForTest();
  config.evidence.capacity = 0;
  EXPECT_THROW(LiveEventSource source(config), std::runtime_error);
}

}  // namespace
