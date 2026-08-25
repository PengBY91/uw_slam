#include <atomic>
#include <stdexcept>
#include <thread>

#include <gtest/gtest.h>

#include "runtime/bounded_queue.hpp"
#include "runtime/run_manifest.hpp"
#include "runtime/state_machines.hpp"

using namespace uw::runtime;

TEST(RunManifest, SerializesCalibrationAndDerivedCalibrationHashesAsIndependentFields) {
  RunManifest manifest;
  manifest.calibration_hash = "raw_hash_123";
  manifest.derived_calibration_hash = "derived_hash_456";
  const std::string json = manifest.ToJson();
  EXPECT_NE(json.find("\"calibration_hash\": \"raw_hash_123\""), std::string::npos);
  EXPECT_NE(json.find("\"derived_calibration_hash\": \"derived_hash_456\""), std::string::npos);
}

TEST(RunManifest, DerivedCalibrationHashDefaultsToEmpty) {
  RunManifest manifest;
  EXPECT_TRUE(manifest.derived_calibration_hash.empty());
  EXPECT_NE(manifest.ToJson().find("\"derived_calibration_hash\": \"\""), std::string::npos);
}

TEST(BoundedQueue, DropOldestReportsOverflowAndKeepsMostRecent) {
  BoundedQueue<int> queue(2, OverflowPolicy::kDropOldest);

  EXPECT_EQ(queue.Push(1), PushResult::kEnqueued);
  EXPECT_EQ(queue.Push(2), PushResult::kEnqueued);
  EXPECT_EQ(queue.Push(3), PushResult::kDroppedOldestAndEnqueued);

  const QueueStats stats = queue.Stats();
  EXPECT_EQ(stats.enqueued_count, 3u);
  EXPECT_EQ(stats.dropped_oldest_count, 1u);
  EXPECT_EQ(stats.dropped_newest_count, 0u);
  EXPECT_EQ(stats.rejected_count, 0u);
  EXPECT_EQ(stats.current_depth, 2u);
  EXPECT_EQ(stats.high_watermark, 2u);
  EXPECT_EQ(queue.TryPop().value(), 2);
  EXPECT_EQ(queue.TryPop().value(), 3);
  EXPECT_FALSE(queue.TryPop().has_value());
}

TEST(BoundedQueue, RejectPolicyReportsRejectedItem) {
  BoundedQueue<int> queue(1, OverflowPolicy::kReject);

  EXPECT_EQ(queue.Push(1), PushResult::kEnqueued);
  EXPECT_EQ(queue.Push(2), PushResult::kRejected);

  const QueueStats stats = queue.Stats();
  EXPECT_EQ(stats.enqueued_count, 1u);
  EXPECT_EQ(stats.rejected_count, 1u);
  EXPECT_EQ(stats.current_depth, 1u);
  EXPECT_EQ(stats.high_watermark, 1u);
  EXPECT_EQ(queue.TryPop().value(), 1);
}

TEST(BoundedQueue, DropNewestReportsDropWithoutCountingEnqueue) {
  BoundedQueue<int> queue(1, OverflowPolicy::kDropNewest);

  EXPECT_EQ(queue.Push(1), PushResult::kEnqueued);
  EXPECT_EQ(queue.Push(2), PushResult::kDroppedNewest);

  const QueueStats stats = queue.Stats();
  EXPECT_EQ(stats.enqueued_count, 1u);
  EXPECT_EQ(stats.dropped_newest_count, 1u);
  EXPECT_EQ(stats.current_depth, 1u);
  EXPECT_EQ(stats.high_watermark, 1u);
  EXPECT_EQ(queue.TryPop().value(), 1);
}

TEST(BoundedQueue, TryPopUpdatesDequeuedCountAndCurrentDepth) {
  BoundedQueue<int> queue(2, OverflowPolicy::kReject);
  ASSERT_EQ(queue.Push(1), PushResult::kEnqueued);
  ASSERT_EQ(queue.Push(2), PushResult::kEnqueued);

  EXPECT_EQ(queue.TryPop().value(), 1);
  QueueStats stats = queue.Stats();
  EXPECT_EQ(stats.dequeued_count, 1u);
  EXPECT_EQ(stats.current_depth, 1u);
  EXPECT_EQ(stats.high_watermark, 2u);

  EXPECT_EQ(queue.TryPop().value(), 2);
  EXPECT_FALSE(queue.TryPop().has_value());
  stats = queue.Stats();
  EXPECT_EQ(stats.dequeued_count, 2u);
  EXPECT_EQ(stats.current_depth, 0u);
  EXPECT_EQ(stats.high_watermark, 2u);
}

TEST(BoundedQueue, RejectsZeroCapacity) {
  EXPECT_THROW((BoundedQueue<int>(0, OverflowPolicy::kReject)), std::runtime_error);
}

TEST(BoundedQueue, StatsSnapshotRemainsConsistentDuringConcurrentPushes) {
  constexpr int kItemsPerProducer = 1000;
  BoundedQueue<int> queue(2 * kItemsPerProducer, OverflowPolicy::kReject);
  std::atomic<bool> start{false};
  std::atomic<bool> producers_done{false};
  std::atomic<bool> snapshots_consistent{true};

  auto producer = [&](int offset) {
    while (!start.load()) std::this_thread::yield();
    for (int i = 0; i < kItemsPerProducer; ++i) {
      if (queue.Push(offset + i) != PushResult::kEnqueued) snapshots_consistent.store(false);
    }
  };

  std::thread first_producer(producer, 0);
  std::thread second_producer(producer, kItemsPerProducer);
  std::thread observer([&] {
    while (!start.load()) std::this_thread::yield();
    while (!producers_done.load()) {
      const QueueStats stats = queue.Stats();
      if (stats.enqueued_count != stats.current_depth ||
          stats.high_watermark < stats.current_depth) {
        snapshots_consistent.store(false);
      }
    }
  });

  start.store(true);
  first_producer.join();
  second_producer.join();
  producers_done.store(true);
  observer.join();

  EXPECT_TRUE(snapshots_consistent.load());
  const QueueStats stats = queue.Stats();
  EXPECT_EQ(stats.enqueued_count, 2u * kItemsPerProducer);
  EXPECT_EQ(stats.dequeued_count, 0u);
  EXPECT_EQ(stats.current_depth, 2u * kItemsPerProducer);
  EXPECT_EQ(stats.high_watermark, 2u * kItemsPerProducer);
}

TEST(SystemStateMachine, RespectsMinimumHoldTime) {
  // Construct with a long initial hold so the very first Request() below is
  // deterministically rejected regardless of scheduler jitter.
  SystemStateMachine sm(SystemState::kTracking, std::chrono::milliseconds(200));
  EXPECT_FALSE(sm.Request(SystemState::kDegraded, "visual_dropout"))
      << "immediate transition within the hold window must be rejected";
  EXPECT_EQ(sm.state(), SystemState::kTracking);

  std::this_thread::sleep_for(std::chrono::milliseconds(220));
  EXPECT_TRUE(sm.Request(SystemState::kDegraded, "visual_dropout"));
  EXPECT_EQ(sm.state(), SystemState::kDegraded);

  EXPECT_FALSE(sm.Request(SystemState::kLost, "still_degraded"))
      << "immediate second transition within the new hold window must be rejected";
  EXPECT_EQ(sm.state(), SystemState::kDegraded);
}

TEST(SystemStateMachine, ForceBypassesHoldTime) {
  SystemStateMachine sm(SystemState::kTracking, std::chrono::milliseconds(10000));
  EXPECT_TRUE(sm.Request(SystemState::kLost, "fatal_sensor_fault", /*force=*/true));
  EXPECT_EQ(sm.state(), SystemState::kLost);
}
