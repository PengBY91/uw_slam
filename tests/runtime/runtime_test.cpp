#include <atomic>
#include <stdexcept>
#include <thread>

#include <gtest/gtest.h>

#include "runtime/bounded_queue.hpp"
#include "runtime/run_manifest.hpp"
#include "runtime/state_machines.hpp"

using namespace uw::runtime;

namespace {

struct ThrowOnMove {
  ThrowOnMove(int value_in, bool* throw_on_move_in)
      : value(value_in), throw_on_move(throw_on_move_in) {}

  ThrowOnMove(const ThrowOnMove&) = delete;
  ThrowOnMove& operator=(const ThrowOnMove&) = delete;

  ThrowOnMove(ThrowOnMove&& other)
      : value(other.value), throw_on_move(other.throw_on_move) {
    if (*throw_on_move) throw std::runtime_error("move failed");
  }

  ThrowOnMove& operator=(ThrowOnMove&&) = delete;

  int value;
  bool* throw_on_move;
};

}  // namespace

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

TEST(BoundedQueue, FailedDropOldestEnqueuePreservesQueueAndStats) {
  bool throw_on_move = false;
  BoundedQueue<ThrowOnMove> queue(2, OverflowPolicy::kDropOldest);
  ASSERT_EQ(queue.Push(ThrowOnMove{1, &throw_on_move}), PushResult::kEnqueued);
  ASSERT_EQ(queue.Push(ThrowOnMove{2, &throw_on_move}), PushResult::kEnqueued);
  const QueueStats before = queue.Stats();

  throw_on_move = true;
  EXPECT_THROW(queue.Push(ThrowOnMove{3, &throw_on_move}), std::runtime_error);
  throw_on_move = false;

  const QueueStats after = queue.Stats();
  EXPECT_EQ(after.enqueued_count, before.enqueued_count);
  EXPECT_EQ(after.dequeued_count, before.dequeued_count);
  EXPECT_EQ(after.dropped_oldest_count, before.dropped_oldest_count);
  EXPECT_EQ(after.dropped_newest_count, before.dropped_newest_count);
  EXPECT_EQ(after.rejected_count, before.rejected_count);
  EXPECT_EQ(after.current_depth, before.current_depth);
  EXPECT_EQ(after.high_watermark, before.high_watermark);

  ASSERT_EQ(queue.Size(), 2u);
  const auto first = queue.TryPop();
  const auto second = queue.TryPop();
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(first->value, 1);
  EXPECT_EQ(second->value, 2);
}

TEST(BoundedQueue, FailedUnderCapacityEnqueuePreservesEmptyQueueAndStats) {
  bool throw_on_move = true;
  BoundedQueue<ThrowOnMove> queue(2, OverflowPolicy::kReject);

  EXPECT_THROW(queue.Push(ThrowOnMove{1, &throw_on_move}), std::runtime_error);
  throw_on_move = false;

  const QueueStats stats = queue.Stats();
  EXPECT_EQ(stats.enqueued_count, 0u);
  EXPECT_EQ(stats.dequeued_count, 0u);
  EXPECT_EQ(stats.dropped_oldest_count, 0u);
  EXPECT_EQ(stats.dropped_newest_count, 0u);
  EXPECT_EQ(stats.rejected_count, 0u);
  EXPECT_EQ(stats.current_depth, 0u);
  EXPECT_EQ(stats.high_watermark, 0u);
  EXPECT_FALSE(queue.TryPop().has_value());
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
