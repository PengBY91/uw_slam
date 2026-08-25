#include "runtime/rolling_latency.hpp"

#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "domain/domain.hpp"

namespace {

using uw::runtime::RollingLatency;

TEST(RollingLatency, ReportsNearestRankPercentilesForCurrentWindow) {
  RollingLatency latency(5);
  for (double sample : {10.0, 50.0, 20.0, 40.0, 30.0}) {
    latency.ObserveMs(sample);
  }

  const auto snapshot = latency.Snapshot();
  EXPECT_EQ(snapshot.sample_count, 5u);
  EXPECT_DOUBLE_EQ(snapshot.p50_ms, 30.0);
  EXPECT_DOUBLE_EQ(snapshot.p95_ms, 50.0);
  EXPECT_DOUBLE_EQ(snapshot.p99_ms, 50.0);
}

TEST(RollingLatency, EvictsTheOldestSampleWhenWindowIsFull) {
  RollingLatency latency(5);
  for (double sample : {10.0, 50.0, 20.0, 40.0, 30.0, 5.0}) {
    latency.ObserveMs(sample);
  }

  const auto snapshot = latency.Snapshot();
  EXPECT_EQ(snapshot.sample_count, 5u);
  EXPECT_DOUBLE_EQ(snapshot.p50_ms, 30.0);
  EXPECT_DOUBLE_EQ(snapshot.p95_ms, 50.0);
  EXPECT_DOUBLE_EQ(snapshot.p99_ms, 50.0);
}

TEST(RollingLatency, EmptyWindowHasFiniteZeroPercentiles) {
  const auto snapshot = RollingLatency(3).Snapshot();
  EXPECT_EQ(snapshot.sample_count, 0u);
  EXPECT_DOUBLE_EQ(snapshot.p50_ms, 0.0);
  EXPECT_DOUBLE_EQ(snapshot.p95_ms, 0.0);
  EXPECT_DOUBLE_EQ(snapshot.p99_ms, 0.0);
}

TEST(RollingLatency, RejectsZeroWindowNegativeAndNonFiniteSamples) {
  EXPECT_THROW(RollingLatency(0), std::invalid_argument);

  RollingLatency latency(2);
  EXPECT_THROW(latency.ObserveMs(-0.001), std::invalid_argument);
  EXPECT_THROW(latency.ObserveMs(std::numeric_limits<double>::quiet_NaN()),
               std::invalid_argument);
  EXPECT_THROW(latency.ObserveMs(std::numeric_limits<double>::infinity()),
               std::invalid_argument);
  EXPECT_THROW(latency.ObserveMs(-std::numeric_limits<double>::infinity()),
               std::invalid_argument);
  EXPECT_EQ(latency.Snapshot().sample_count, 0u);
}

TEST(HealthReportContract, AppendedRuntimeFieldsHaveStableWireNumbersAndRoundTrip) {
  const auto* descriptor = uw::domain::HealthReport::descriptor();
  ASSERT_NE(descriptor, nullptr);
  EXPECT_EQ(descriptor->FindFieldByName("rejected_frame_count")->number(), 15);
  EXPECT_EQ(descriptor->FindFieldByName("expired_frame_count")->number(), 16);
  EXPECT_EQ(descriptor->FindFieldByName("queue_high_watermark")->number(), 17);
  EXPECT_EQ(descriptor->FindFieldByName("oldest_message_age_ms")->number(), 18);
  EXPECT_EQ(descriptor->FindFieldByName("deadline_miss_count")->number(), 19);
  EXPECT_EQ(descriptor->FindFieldByName("last_valid_capture_time")->number(), 20);
  EXPECT_EQ(descriptor->FindFieldByName("last_valid_receive_time")->number(), 21);
  EXPECT_EQ(descriptor->FindFieldByName("last_processed_time")->number(), 22);
  EXPECT_EQ(descriptor->FindFieldByName("synchronization_rejected_count")->number(), 23);
  EXPECT_EQ(descriptor->FindFieldByName("sequence_gap_count")->number(), 24);
  EXPECT_EQ(descriptor->FindFieldByName("stale_result_count")->number(), 25);

  uw::domain::HealthReport source;
  source.set_rejected_frame_count(11);
  source.set_expired_frame_count(12);
  source.set_queue_high_watermark(13);
  source.set_oldest_message_age_ms(14.5);
  source.set_deadline_miss_count(15);
  *source.mutable_last_valid_capture_time() = uw::domain::FromSeconds(16.25);
  *source.mutable_last_valid_receive_time() = uw::domain::FromSeconds(17.5);
  *source.mutable_last_processed_time() = uw::domain::FromSeconds(18.75);
  source.set_synchronization_rejected_count(19);
  source.set_sequence_gap_count(20);
  source.set_stale_result_count(21);

  uw::domain::HealthReport decoded;
  ASSERT_TRUE(decoded.ParseFromString(source.SerializeAsString()));
  EXPECT_EQ(decoded.rejected_frame_count(), 11u);
  EXPECT_EQ(decoded.expired_frame_count(), 12u);
  EXPECT_EQ(decoded.queue_high_watermark(), 13u);
  EXPECT_DOUBLE_EQ(decoded.oldest_message_age_ms(), 14.5);
  EXPECT_EQ(decoded.deadline_miss_count(), 15u);
  EXPECT_DOUBLE_EQ(uw::domain::ToSeconds(decoded.last_valid_capture_time()), 16.25);
  EXPECT_DOUBLE_EQ(uw::domain::ToSeconds(decoded.last_valid_receive_time()), 17.5);
  EXPECT_DOUBLE_EQ(uw::domain::ToSeconds(decoded.last_processed_time()), 18.75);
  EXPECT_EQ(decoded.synchronization_rejected_count(), 19u);
  EXPECT_EQ(decoded.sequence_gap_count(), 20u);
  EXPECT_EQ(decoded.stale_result_count(), 21u);
}

}  // namespace
