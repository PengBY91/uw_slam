#include "runtime/bag_audit_checks.hpp"

#include <gtest/gtest.h>

#include <array>
#include <limits>

using uw::runtime::AccumulateHeader;
using uw::runtime::FrameResolves;
using uw::runtime::HeaderStats;

namespace {

uw::domain::ObservationHeader MakeHeader(int64_t capture_s, int64_t receive_s,
                                          uw::domain::ClockDomain domain = uw::domain::CLOCK_DOMAIN_SIMULATION,
                                          const std::string& sensor_frame = "sonar_link") {
  uw::domain::ObservationHeader header;
  header.mutable_capture_time()->set_seconds(capture_s);
  header.mutable_receive_time()->set_seconds(receive_s);
  header.set_clock_domain(domain);
  header.mutable_sensor_frame()->set_value(sensor_frame);
  return header;
}

}  // namespace

TEST(BagAuditChecks, AccumulateHeaderDetectsMonotonicCaptureAndReceiveTime) {
  HeaderStats stats;
  AccumulateHeader(stats, MakeHeader(1, 1));
  AccumulateHeader(stats, MakeHeader(2, 2));
  AccumulateHeader(stats, MakeHeader(3, 3));
  EXPECT_EQ(stats.count, 3u);
  EXPECT_TRUE(stats.capture_time_ever_populated);
  EXPECT_TRUE(stats.receive_time_ever_populated);
  EXPECT_TRUE(stats.capture_time_monotonic);
  EXPECT_TRUE(stats.receive_time_monotonic);
}

TEST(BagAuditChecks, AccumulateHeaderDetectsNonMonotonicCaptureTime) {
  HeaderStats stats;
  AccumulateHeader(stats, MakeHeader(5, 1));
  AccumulateHeader(stats, MakeHeader(3, 2));  // capture_time went backwards
  EXPECT_FALSE(stats.capture_time_monotonic);
  EXPECT_TRUE(stats.receive_time_monotonic);  // receive_time still fine
}

TEST(BagAuditChecks, AccumulateHeaderDetectsUnpopulatedTimestamps) {
  HeaderStats stats;
  uw::domain::ObservationHeader header;  // capture_time/receive_time left at proto default (all-zero Stamp)
  header.mutable_sensor_frame()->set_value("sonar_link");
  AccumulateHeader(stats, header);
  EXPECT_EQ(stats.count, 1u);
  EXPECT_FALSE(stats.capture_time_ever_populated);
  EXPECT_FALSE(stats.receive_time_ever_populated);
  // A never-populated stamp must not be misreported as "monotonic" evidence
  // of anything — it just never contributed a comparison.
  EXPECT_TRUE(stats.capture_time_monotonic);
}

TEST(BagAuditChecks, AccumulateHeaderCollectsMixedClockDomains) {
  HeaderStats stats;
  AccumulateHeader(stats, MakeHeader(1, 1, uw::domain::CLOCK_DOMAIN_SIMULATION));
  AccumulateHeader(stats, MakeHeader(2, 2, uw::domain::CLOCK_DOMAIN_SYSTEM_MONOTONIC));
  EXPECT_EQ(stats.clock_domains.size(), 2u)
      << "a bag mixing simulation and system-monotonic clock domains must be detectable from stats alone";
}

TEST(BagAuditChecks, AccumulateHeaderSingleClockDomainStaysSingle) {
  HeaderStats stats;
  AccumulateHeader(stats, MakeHeader(1, 1, uw::domain::CLOCK_DOMAIN_SIMULATION));
  AccumulateHeader(stats, MakeHeader(2, 2, uw::domain::CLOCK_DOMAIN_SIMULATION));
  EXPECT_EQ(stats.clock_domains.size(), 1u);
}

TEST(BagAuditChecks, AccumulateHeaderCollectsDistinctSensorFrames) {
  HeaderStats stats;
  AccumulateHeader(stats, MakeHeader(1, 1, uw::domain::CLOCK_DOMAIN_SIMULATION, "sonar_link"));
  AccumulateHeader(stats, MakeHeader(2, 2, uw::domain::CLOCK_DOMAIN_SIMULATION, "sonar_link"));
  ASSERT_EQ(stats.sensor_frames.size(), 1u);
  EXPECT_EQ(*stats.sensor_frames.begin(), "sonar_link");
}

namespace {

uw::domain::RigCalibrationSnapshot MakeRigWithEdges(
    const std::vector<std::pair<std::string, std::string>>& edges) {
  uw::domain::RigCalibrationSnapshot rig;
  for (const auto& [parent, child] : edges) {
    auto* edge = rig.add_frame_tree();
    edge->mutable_parent_frame()->set_value(parent);
    edge->mutable_child_frame()->set_value(child);
  }
  return rig;
}

}  // namespace

TEST(BagAuditChecks, FrameResolvesForBaseLinkItself) {
  const auto rig = MakeRigWithEdges({});
  EXPECT_TRUE(FrameResolves(rig, "base_link"));
}

TEST(BagAuditChecks, FrameResolvesForDirectChild) {
  const auto rig = MakeRigWithEdges({{"base_link", "sonar_link"}});
  EXPECT_TRUE(FrameResolves(rig, "sonar_link"));
}

TEST(BagAuditChecks, FrameResolvesTransitivelyThroughMultipleEdges) {
  const auto rig = MakeRigWithEdges({{"base_link", "mount_link"}, {"mount_link", "camera_left_link"}});
  EXPECT_TRUE(FrameResolves(rig, "camera_left_link"));
}

TEST(BagAuditChecks, FrameResolutionFailsForFrameWithNoEdge) {
  // Mirrors the real, already-known gap this check is meant to catch:
  // configs/rig/example_auv_real_camera.yaml has camera_left_link/
  // camera_right_link/sonar_link/imu_link edges but no dvl_link edge.
  const auto rig = MakeRigWithEdges({{"base_link", "camera_left_link"},
                                      {"base_link", "camera_right_link"},
                                      {"base_link", "sonar_link"},
                                      {"base_link", "imu_link"}});
  EXPECT_TRUE(FrameResolves(rig, "sonar_link"));
  EXPECT_FALSE(FrameResolves(rig, "dvl_link"));
}

// --- capture-time span / rate -----------------------------------------

TEST(BagAuditChecks, AccumulateHeaderRecordsCaptureSpanAndMeanRate) {
  HeaderStats stats;
  // t = 0 is a real instant in a simulation clock domain (synth_bag_gen
  // stamps the first IMU sample of a stationary pre-roll there), so it is
  // part of the span — dropping it would report the stream as starting one
  // sample late and inflate the rate.
  AccumulateHeader(stats, MakeHeader(0, 0));
  AccumulateHeader(stats, MakeHeader(1, 1));
  AccumulateHeader(stats, MakeHeader(2, 2));
  EXPECT_EQ(stats.first_capture_ns, 0);
  EXPECT_EQ(stats.last_capture_ns, 2'000'000'000LL);
  EXPECT_DOUBLE_EQ(uw::runtime::MeanCaptureRateHz(stats), 1.0);
  // ...while "was this topic ever stamped at all" stays a whole-topic
  // property, which is what the unpopulated-timestamp failure keys on.
  EXPECT_TRUE(stats.capture_time_ever_populated);
}

TEST(BagAuditChecks, AllZeroStampTopicStaysUnpopulatedAndRateless) {
  HeaderStats stats;
  AccumulateHeader(stats, MakeHeader(0, 0));
  AccumulateHeader(stats, MakeHeader(0, 0));
  EXPECT_FALSE(stats.capture_time_ever_populated);
  EXPECT_FALSE(stats.receive_time_ever_populated);
  EXPECT_DOUBLE_EQ(uw::runtime::MeanCaptureRateHz(stats), 0.0);
}

TEST(BagAuditChecks, DetectsATopicThatMixesStampedAndUnstampedMessages) {
  // t = 0 is a real instant, so an all-zero Stamp can only be told apart
  // from it by whether the rest of the topic is stamped. A topic carrying
  // both has no usable span, rate or ordering -- and must be reported as
  // that, not as a bare non-monotonicity.
  HeaderStats mixed;
  AccumulateHeader(mixed, MakeHeader(5, 5));
  AccumulateHeader(mixed, MakeHeader(0, 0));  // left at the proto default
  AccumulateHeader(mixed, MakeHeader(7, 7));
  EXPECT_TRUE(uw::runtime::HasMixedStampPopulation(mixed));
  EXPECT_EQ(mixed.unpopulated_capture_time_after_populated_count, 1u);
  EXPECT_EQ(mixed.unpopulated_receive_time_after_populated_count, 1u);

  // Neither of the two legitimate cases is flagged: a fully stamped topic
  // (t = 0 included) or a topic that was never stamped at all.
  HeaderStats stamped_from_zero;
  AccumulateHeader(stamped_from_zero, MakeHeader(0, 0));
  AccumulateHeader(stamped_from_zero, MakeHeader(1, 1));
  EXPECT_FALSE(uw::runtime::HasMixedStampPopulation(stamped_from_zero))
      << "a stream that genuinely starts at t = 0 -- which is what the synthetic stationary "
         "pre-roll produces -- must not be called a defect";
  HeaderStats never_stamped;
  AccumulateHeader(never_stamped, MakeHeader(0, 0));
  AccumulateHeader(never_stamped, MakeHeader(0, 0));
  EXPECT_FALSE(uw::runtime::HasMixedStampPopulation(never_stamped));
}

TEST(BagAuditChecks, MeanCaptureRateIsZeroWithoutASpan) {
  HeaderStats empty;
  EXPECT_DOUBLE_EQ(uw::runtime::MeanCaptureRateHz(empty), 0.0);
  HeaderStats single;
  AccumulateHeader(single, MakeHeader(3, 3));
  EXPECT_DOUBLE_EQ(uw::runtime::MeanCaptureRateHz(single), 0.0);
  // Two messages at the same instant have a span of zero, not a rate of
  // infinity.
  HeaderStats repeated;
  AccumulateHeader(repeated, MakeHeader(3, 3));
  AccumulateHeader(repeated, MakeHeader(3, 3));
  EXPECT_DOUBLE_EQ(uw::runtime::MeanCaptureRateHz(repeated), 0.0);
}

TEST(BagAuditChecks, AccumulateHeaderSeparatesStrictlyIncreasingFromMonotonic) {
  HeaderStats stats;
  AccumulateHeader(stats, MakeHeader(1, 1));
  AccumulateHeader(stats, MakeHeader(1, 1));  // repeated instant
  // Several sonar pings written at one keyframe tick is legitimate, so the
  // shared monotonicity flag must stay true...
  EXPECT_TRUE(stats.capture_time_monotonic);
  // ...while the stronger property keyframe boundaries need is false.
  EXPECT_FALSE(stats.capture_time_strictly_increasing);
}

// --- keyframe boundaries ----------------------------------------------

namespace {

uw::domain::KeyframeBoundary MakeBoundary(const std::string& keyframe_id, int64_t capture_s,
                                          const std::string& source = "synthetic_fixed_interval_v1") {
  uw::domain::KeyframeBoundary boundary;
  *boundary.mutable_header() = MakeHeader(capture_s, capture_s, uw::domain::CLOCK_DOMAIN_SIMULATION,
                                          "base_link");
  boundary.mutable_keyframe_id()->set_value(keyframe_id);
  boundary.set_source(source);
  return boundary;
}

}  // namespace

TEST(BagAuditChecks, AccumulateKeyframeBoundaryAcceptsAWellFormedStream) {
  uw::runtime::KeyframeBoundaryStats stats;
  uw::runtime::AccumulateKeyframeBoundary(stats, MakeBoundary("kf0", 1));
  uw::runtime::AccumulateKeyframeBoundary(stats, MakeBoundary("kf1", 2));
  EXPECT_EQ(stats.header.count, 2u);
  EXPECT_TRUE(stats.keyframe_ids_unique);
  EXPECT_TRUE(stats.header.capture_time_strictly_increasing);
  EXPECT_FALSE(stats.keyframe_id_ever_empty);
  EXPECT_FALSE(stats.source_ever_empty);
  EXPECT_EQ(stats.keyframe_ids.size(), 2u);
}

TEST(BagAuditChecks, AccumulateKeyframeBoundaryDetectsDuplicateIds) {
  uw::runtime::KeyframeBoundaryStats stats;
  uw::runtime::AccumulateKeyframeBoundary(stats, MakeBoundary("kf0", 1));
  uw::runtime::AccumulateKeyframeBoundary(stats, MakeBoundary("kf0", 2));
  EXPECT_FALSE(stats.keyframe_ids_unique);
  EXPECT_TRUE(stats.header.capture_time_strictly_increasing);
}

TEST(BagAuditChecks, AccumulateKeyframeBoundaryDetectsRepeatedInstant) {
  uw::runtime::KeyframeBoundaryStats stats;
  uw::runtime::AccumulateKeyframeBoundary(stats, MakeBoundary("kf0", 1));
  uw::runtime::AccumulateKeyframeBoundary(stats, MakeBoundary("kf1", 1));
  EXPECT_TRUE(stats.keyframe_ids_unique);
  EXPECT_FALSE(stats.header.capture_time_strictly_increasing);
}

TEST(BagAuditChecks, AccumulateKeyframeBoundaryDetectsEmptyIdentityAndSource) {
  uw::runtime::KeyframeBoundaryStats stats;
  uw::runtime::AccumulateKeyframeBoundary(stats, MakeBoundary("", 1, ""));
  EXPECT_TRUE(stats.keyframe_id_ever_empty);
  EXPECT_TRUE(stats.source_ever_empty);
}

// --- IMU window statistics --------------------------------------------

namespace {

uw::domain::ImuSample MakeImuSample(const std::array<double, 3>& accel,
                                    const std::array<double, 3>& gyro) {
  uw::domain::ImuSample sample;
  for (int i = 0; i < 3; ++i) {
    sample.add_linear_acceleration_mps2(accel[i]);
    sample.add_angular_velocity_radps(gyro[i]);
  }
  return sample;
}

}  // namespace

TEST(BagAuditChecks, ImuWindowMeansCancelZeroMeanNoise) {
  uw::runtime::ImuWindowStats stats;
  // Two samples straddling a stationary truth of (0, 0, g) / zero rate by
  // +-0.1: every individual sample is well outside the 0.05 m/s^2
  // stationary threshold, the mean is exactly on it. This is the whole
  // reason the statistic is a mean and not a per-sample check.
  uw::runtime::AccumulateImuSample(stats, MakeImuSample({0.1, 0.0, 9.90665}, {0.05, 0.0, 0.0}));
  uw::runtime::AccumulateImuSample(stats, MakeImuSample({-0.1, 0.0, 9.70665}, {-0.05, 0.0, 0.0}));
  EXPECT_EQ(stats.sample_count, 2u);
  EXPECT_EQ(stats.malformed_sample_count, 0u);
  EXPECT_NEAR(uw::runtime::MeanAccelNormMps2(stats), 9.80665, 1e-12);
  EXPECT_NEAR(uw::runtime::MeanGyroNormRadps(stats), 0.0, 1e-12);
}

TEST(BagAuditChecks, ImuWindowSkipsMalformedSamplesInsteadOfPoisoningTheMean) {
  uw::runtime::ImuWindowStats stats;
  uw::runtime::AccumulateImuSample(stats, MakeImuSample({0.0, 0.0, 9.80665}, {0.0, 0.0, 0.0}));
  uw::domain::ImuSample short_sample;  // two entries, not three
  short_sample.add_linear_acceleration_mps2(1.0);
  short_sample.add_linear_acceleration_mps2(2.0);
  uw::runtime::AccumulateImuSample(stats, short_sample);
  uw::runtime::AccumulateImuSample(
      stats, MakeImuSample({std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}, {0.0, 0.0, 0.0}));
  EXPECT_EQ(stats.sample_count, 1u);
  EXPECT_EQ(stats.malformed_sample_count, 2u);
  EXPECT_NEAR(uw::runtime::MeanAccelNormMps2(stats), 9.80665, 1e-12);
}

TEST(BagAuditChecks, ImuWindowMeansAreZeroWithoutSamples) {
  uw::runtime::ImuWindowStats stats;
  EXPECT_DOUBLE_EQ(uw::runtime::MeanAccelNormMps2(stats), 0.0);
  EXPECT_DOUBLE_EQ(uw::runtime::MeanGyroNormRadps(stats), 0.0);
}

// --- payload digest ----------------------------------------------------

TEST(BagAuditChecks, PayloadDigestIsOrderSensitiveAndContentSensitive) {
  using uw::runtime::Fnv1a64Update;
  using uw::runtime::kFnv1a64OffsetBasis;
  const uint64_t ab = Fnv1a64Update(Fnv1a64Update(kFnv1a64OffsetBasis, "a"), "b");
  const uint64_t ba = Fnv1a64Update(Fnv1a64Update(kFnv1a64OffsetBasis, "b"), "a");
  const uint64_t ab_again = Fnv1a64Update(Fnv1a64Update(kFnv1a64OffsetBasis, "a"), "b");
  EXPECT_EQ(ab, ab_again);
  EXPECT_NE(ab, ba) << "two bags whose IMU messages differ only in order must digest differently";
  EXPECT_NE(ab, Fnv1a64Update(Fnv1a64Update(kFnv1a64OffsetBasis, "a"), "c"));
}

TEST(BagAuditChecks, ToHex64IsFixedWidthLowercase) {
  EXPECT_EQ(uw::runtime::ToHex64(0), "0000000000000000");
  EXPECT_EQ(uw::runtime::ToHex64(0xcbf29ce484222325ULL), "cbf29ce484222325");
}
