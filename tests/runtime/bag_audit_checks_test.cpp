#include "runtime/bag_audit_checks.hpp"

#include <gtest/gtest.h>

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
