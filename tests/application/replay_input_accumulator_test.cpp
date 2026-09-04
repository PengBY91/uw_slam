#include "application/replay_input_accumulator.hpp"

#include <gtest/gtest.h>

#include "domain/domain.hpp"
#include "runtime/canonical_topics.hpp"

namespace {

using uw::application::ReplayInputAccumulator;
using uw::runtime::CanonicalEvent;

CanonicalEvent MakeImageEvent(const std::string& sensor_id, const std::string& observation_id,
                              double capture_time_s, uint64_t log_time_ns, uint64_t seq) {
  uw::domain::ImageFrame frame;
  frame.mutable_header()->mutable_sensor_id()->set_value(sensor_id);
  frame.mutable_header()->mutable_observation_id()->set_value(observation_id);
  *frame.mutable_header()->mutable_capture_time() = uw::domain::FromSeconds(capture_time_s);
  CanonicalEvent event;
  event.topic = uw::runtime::kTopicCameraLeft;
  event.log_time_ns = log_time_ns;
  event.source_sequence = seq;
  event.payload = frame;
  return event;
}

CanonicalEvent MakeImuEvent(const std::string& observation_id, uint64_t log_time_ns, uint64_t seq) {
  uw::domain::ImuSample sample;
  sample.mutable_header()->mutable_sensor_id()->set_value("imu");
  sample.mutable_header()->mutable_observation_id()->set_value(observation_id);
  CanonicalEvent event;
  event.topic = uw::runtime::kTopicImu;
  event.log_time_ns = log_time_ns;
  event.source_sequence = seq;
  event.payload = sample;
  return event;
}

CanonicalEvent MakeSonarEvent(const std::string& sensor_id, const std::string& observation_id,
                              uint64_t log_time_ns, uint64_t seq) {
  uw::domain::SonarFrame frame;
  frame.mutable_header()->mutable_sensor_id()->set_value(sensor_id);
  frame.mutable_header()->mutable_observation_id()->set_value(observation_id);
  CanonicalEvent event;
  event.topic = uw::runtime::kTopicSonarFrame;
  event.log_time_ns = log_time_ns;
  event.source_sequence = seq;
  event.payload = frame;
  return event;
}

CanonicalEvent MakeDepthEvidenceEvent(const std::string& evidence_id,
                                      const std::string& source_observation_id, uint64_t log_time_ns,
                                      uint64_t seq) {
  uw::domain::PressureDepthMeasurement depth;
  depth.set_depth_m(1.0);
  depth.set_sigma_m(0.05);
  uw::domain::EvidenceId eid;
  eid.set_value(evidence_id);
  uw::domain::ObservationId oid;
  oid.set_value(source_observation_id);
  const auto evidence = uw::domain::MakeEvidence(eid, {oid}, depth, /*noise_scale=*/1.0, "test_depth_v1");
  CanonicalEvent event;
  event.topic = uw::runtime::kTopicEvidenceDepth;
  event.log_time_ns = log_time_ns;
  event.source_sequence = seq;
  event.payload = evidence;
  return event;
}

CanonicalEvent MakeKeyframeBoundaryEvent(const std::string& observation_id,
                                         const std::string& keyframe_id,
                                         double capture_time_s, uint64_t log_time_ns,
                                         uint64_t seq) {
  uw::domain::KeyframeBoundary boundary;
  boundary.mutable_header()->mutable_sensor_id()->set_value("keyframe-selector");
  boundary.mutable_header()->mutable_observation_id()->set_value(observation_id);
  *boundary.mutable_header()->mutable_capture_time() = uw::domain::FromSeconds(capture_time_s);
  boundary.mutable_keyframe_id()->set_value(keyframe_id);
  boundary.set_source("test-selector");
  return {uw::runtime::kTopicKeyframeBoundary, log_time_ns, seq, std::move(boundary)};
}

TEST(ReplayInputAccumulator, StoresValidKeyframeBoundariesInEventOrder) {
  ReplayInputAccumulator accumulator;
  EXPECT_TRUE(accumulator.OnKeyframeBoundary(
      MakeKeyframeBoundaryEvent("boundary-1", "kf-1", 2.0, 9000, 0)));
  EXPECT_TRUE(accumulator.OnKeyframeBoundary(
      MakeKeyframeBoundaryEvent("boundary-2", "kf-2", 3.0, 1000, 1)));

  ASSERT_EQ(accumulator.Data().keyframe_boundaries.size(), 2u);
  EXPECT_EQ(accumulator.Data().keyframe_boundaries[0].keyframe_id().value(), "kf-1");
  EXPECT_EQ(accumulator.Data().keyframe_boundaries[1].keyframe_id().value(), "kf-2");
  EXPECT_FALSE(accumulator.Diagnostics().HasErrors());
}

TEST(ReplayInputAccumulator, RejectsEmptyAndDuplicateKeyframeIdsWithoutStoringThem) {
  ReplayInputAccumulator accumulator;
  EXPECT_TRUE(accumulator.OnKeyframeBoundary(
      MakeKeyframeBoundaryEvent("boundary-empty", "", 1.0, 1, 0)));
  EXPECT_TRUE(accumulator.OnKeyframeBoundary(
      MakeKeyframeBoundaryEvent("boundary-1", "kf-1", 2.0, 2, 1)));
  EXPECT_TRUE(accumulator.OnKeyframeBoundary(
      MakeKeyframeBoundaryEvent("boundary-2", "kf-1", 3.0, 3, 2)));

  EXPECT_EQ(accumulator.Diagnostics().empty_keyframe_id_count, 1u);
  EXPECT_EQ(accumulator.Diagnostics().duplicate_keyframe_id_count, 1u);
  EXPECT_EQ(accumulator.Diagnostics().non_increasing_keyframe_capture_time_count, 0u);
  ASSERT_EQ(accumulator.Data().keyframe_boundaries.size(), 1u);
  EXPECT_EQ(accumulator.Data().keyframe_boundaries[0].keyframe_id().value(), "kf-1");
  EXPECT_TRUE(accumulator.Diagnostics().HasErrors());
}

TEST(ReplayInputAccumulator, RejectsNonIncreasingBoundaryHeaderCaptureTimeNotLogTime) {
  ReplayInputAccumulator accumulator;
  EXPECT_TRUE(accumulator.OnKeyframeBoundary(
      MakeKeyframeBoundaryEvent("boundary-1", "kf-1", 10.0, 300, 0)));
  EXPECT_TRUE(accumulator.OnKeyframeBoundary(
      MakeKeyframeBoundaryEvent("boundary-2", "kf-2", 10.0, 400, 1)));
  EXPECT_TRUE(accumulator.OnKeyframeBoundary(
      MakeKeyframeBoundaryEvent("boundary-3", "kf-3", 9.0, 500, 2)));
  EXPECT_TRUE(accumulator.OnKeyframeBoundary(
      MakeKeyframeBoundaryEvent("boundary-4", "kf-4", 11.0, 100, 3)));

  EXPECT_EQ(accumulator.Diagnostics().non_increasing_keyframe_capture_time_count, 2u);
  ASSERT_EQ(accumulator.Data().keyframe_boundaries.size(), 2u);
  EXPECT_EQ(accumulator.Data().keyframe_boundaries[1].keyframe_id().value(), "kf-4");
  EXPECT_TRUE(accumulator.Diagnostics().HasErrors());
}

TEST(ReplayInputAccumulator, PreservesJitteredNonKeyframeAlignedObservationIds) {
  ReplayInputAccumulator accumulator;
  ASSERT_TRUE(accumulator.OnImageFrame(MakeImageEvent("camera_left", "frame-A", 0.0, 0, 0)));
  ASSERT_TRUE(
      accumulator.OnImageFrame(MakeImageEvent("camera_left", "frame-B", 0.173, 173000000ULL, 1)));
  ASSERT_TRUE(accumulator.OnMeasurementEvidence(MakeDepthEvidenceEvent("depth_a", "frame-A", 0, 2)));
  ASSERT_TRUE(
      accumulator.OnMeasurementEvidence(MakeDepthEvidenceEvent("depth_b", "frame-B", 173000000ULL, 3)));
  ASSERT_TRUE(accumulator.Flush());

  EXPECT_FALSE(accumulator.Diagnostics().HasErrors());
  ASSERT_EQ(accumulator.Data().images.size(), 2u);
  // Identities preserved verbatim -- never rewritten to "kf0"/"kf1", and
  // association does not depend on 173ms/241ms being a multiple of 0.2s.
  EXPECT_EQ(accumulator.Data().images[0].header().observation_id().value(), "frame-A");
  EXPECT_EQ(accumulator.Data().images[1].header().observation_id().value(), "frame-B");
  ASSERT_EQ(accumulator.Data().evidence.size(), 2u);
}

TEST(ReplayInputAccumulator, SameCaptureTimeDifferentObservationIdsAreNotOverwritten) {
  ReplayInputAccumulator accumulator;
  ASSERT_TRUE(accumulator.OnImageFrame(MakeImageEvent("camera_left", "frame-A", 5.0, 5000000000ULL, 0)));
  ASSERT_TRUE(accumulator.OnImageFrame(MakeImageEvent("camera_left", "frame-B", 5.0, 5000000000ULL, 1)));
  ASSERT_TRUE(accumulator.Flush());

  EXPECT_FALSE(accumulator.Diagnostics().HasErrors());
  ASSERT_EQ(accumulator.Data().images.size(), 2u);
}

TEST(ReplayInputAccumulator, EmptyObservationIdIsRecordedAsDiagnosticAndDropped) {
  ReplayInputAccumulator accumulator;
  EXPECT_TRUE(accumulator.OnImageFrame(MakeImageEvent("camera_left", "", 0.0, 0, 0)));
  accumulator.Flush();

  EXPECT_TRUE(accumulator.Diagnostics().HasErrors());
  EXPECT_EQ(accumulator.Diagnostics().empty_observation_id_count, 1u);
  EXPECT_TRUE(accumulator.Data().images.empty());
}

TEST(ReplayInputAccumulator, DuplicateSensorObservationIdIsRecordedAsDiagnosticAndDropped) {
  ReplayInputAccumulator accumulator;
  ASSERT_TRUE(accumulator.OnImageFrame(MakeImageEvent("camera_left", "frame-A", 0.0, 0, 0)));
  EXPECT_TRUE(accumulator.OnImageFrame(MakeImageEvent("camera_left", "frame-A", 1.0, 1000000000ULL, 1)));
  accumulator.Flush();

  EXPECT_TRUE(accumulator.Diagnostics().HasErrors());
  EXPECT_EQ(accumulator.Diagnostics().duplicate_observation_count, 1u);
  EXPECT_EQ(accumulator.Data().images.size(), 1u);
}

TEST(ReplayInputAccumulator, SameObservationIdDifferentSensorIsNotADuplicate) {
  // camera_left/camera_right legitimately share one observation_id per
  // stereo pair (see synth_bag_gen.cpp's BuildStereoPair) -- (sensor_id,
  // observation_id) is the identity key, not observation_id alone.
  ReplayInputAccumulator accumulator;
  ASSERT_TRUE(accumulator.OnImageFrame(MakeImageEvent("camera_left", "kf0", 0.0, 0, 0)));
  ASSERT_TRUE(accumulator.OnImageFrame(MakeImageEvent("camera_right", "kf0", 0.0, 0, 1)));
  accumulator.Flush();

  EXPECT_FALSE(accumulator.Diagnostics().HasErrors());
  EXPECT_EQ(accumulator.Data().images.size(), 2u);
}

TEST(ReplayInputAccumulator, SonarFrameSharingOneObservationIdAcrossMultiplePingsIsNotADuplicate) {
  // synth_bag_gen (and real multi-target scenes) legitimately emit one
  // SonarFrame per in-range target under the SAME (sensor_id,
  // observation_id) -- this is documented v1 behavior (replay_pipeline.cpp
  // keeps only the first one), not a data-integrity bug like a duplicate
  // ImageFrame would be.
  ReplayInputAccumulator accumulator;
  EXPECT_TRUE(accumulator.OnSonarFrame(MakeSonarEvent("sonar0", "kf0", 0, 0)));
  EXPECT_TRUE(accumulator.OnSonarFrame(MakeSonarEvent("sonar0", "kf0", 0, 1)));
  accumulator.Flush();

  EXPECT_FALSE(accumulator.Diagnostics().HasErrors());
  EXPECT_EQ(accumulator.Diagnostics().duplicate_observation_count, 0u);
  EXPECT_EQ(accumulator.Data().sonar_frames.size(), 2u);
}

TEST(ReplayInputAccumulator, EvidenceReferencingUnknownObservationIsRecordedAsDiagnostic) {
  ReplayInputAccumulator accumulator;
  ASSERT_TRUE(accumulator.OnMeasurementEvidence(MakeDepthEvidenceEvent("depth_x", "does-not-exist", 0, 0)));
  accumulator.Flush();

  EXPECT_TRUE(accumulator.Diagnostics().HasErrors());
  EXPECT_EQ(accumulator.Diagnostics().dangling_evidence_reference_count, 1u);
  // The evidence itself is still retained for diagnosability -- only the
  // reference is flagged, not silently dropped along with the evidence.
  EXPECT_EQ(accumulator.Data().evidence.size(), 1u);
}

TEST(ReplayInputAccumulator, EvidenceLogTimeNsIsIndexAlignedWithEvidence) {
  ReplayInputAccumulator accumulator;
  ASSERT_TRUE(accumulator.OnMeasurementEvidence(MakeDepthEvidenceEvent("depth_a", "frame-A", 111, 0)));
  ASSERT_TRUE(accumulator.OnMeasurementEvidence(MakeDepthEvidenceEvent("depth_b", "frame-B", 222, 1)));

  ASSERT_EQ(accumulator.Data().evidence.size(), 2u);
  ASSERT_EQ(accumulator.EvidenceLogTimeNs().size(), 2u);
  EXPECT_EQ(accumulator.EvidenceLogTimeNs()[0], 111u);
  EXPECT_EQ(accumulator.EvidenceLogTimeNs()[1], 222u);
}

TEST(ReplayInputAccumulator, ImuSamplesPreserveArrivalOrderWithoutKeyframeAssociation) {
  ReplayInputAccumulator accumulator;
  ASSERT_TRUE(accumulator.OnImuSample(MakeImuEvent("imu0", 0, 0)));
  ASSERT_TRUE(accumulator.OnImuSample(MakeImuEvent("imu1", 1000, 1)));
  accumulator.Flush();

  ASSERT_EQ(accumulator.Data().imu_samples.size(), 2u);
  EXPECT_EQ(accumulator.Data().imu_samples[0].header().observation_id().value(), "imu0");
  EXPECT_EQ(accumulator.Data().imu_samples[1].header().observation_id().value(), "imu1");
}

}  // namespace
