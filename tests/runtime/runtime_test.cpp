#include <thread>

#include <gtest/gtest.h>

#include "runtime/bounded_queue.hpp"
#include "runtime/state_machines.hpp"

using namespace uw::runtime;

TEST(BoundedQueue, DropOldestKeepsMostRecent) {
  BoundedQueue<int> queue(2, OverflowPolicy::kDropOldest);
  EXPECT_TRUE(queue.Push(1));
  EXPECT_TRUE(queue.Push(2));
  EXPECT_TRUE(queue.Push(3));  // evicts 1
  EXPECT_EQ(queue.DroppedCount(), 1u);
  EXPECT_EQ(queue.Size(), 2u);
  EXPECT_EQ(queue.TryPop().value(), 2);
  EXPECT_EQ(queue.TryPop().value(), 3);
  EXPECT_FALSE(queue.TryPop().has_value());
}

TEST(BoundedQueue, RejectPolicyReturnsFalseOnOverflow) {
  BoundedQueue<int> queue(1, OverflowPolicy::kReject);
  EXPECT_TRUE(queue.Push(1));
  EXPECT_FALSE(queue.Push(2));
  EXPECT_EQ(queue.Size(), 1u);
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
