// docs/archive/rov-realtime-closed-loop-code-review-2026-08-27.md finding B6:
// docs/traceability/rov-realtime-closed-loop.csv's FUS-OUT-002 row cited
// "tests/application/*" (a glob, not a real test) for LatestAssistSink's
// replace-latest guarantee -- no test actually exercised it directly. This
// file closes that gap.
#include "application/latest_assist_sink.hpp"

#include <gtest/gtest.h>

using uw::application::LatestAssistSink;

namespace {

uw::domain::OperatorAssistState MakeState(double data_age_ms) {
  uw::domain::OperatorAssistState state;
  state.set_data_age_ms(data_age_ms);
  return state;
}

}  // namespace

TEST(LatestAssistSink, StartsEmpty) {
  LatestAssistSink sink;
  EXPECT_FALSE(sink.Latest().has_value());
}

TEST(LatestAssistSink, PublishOverwritesRatherThanQueueing) {
  LatestAssistSink sink;
  sink.Publish(MakeState(10.0));
  sink.Publish(MakeState(20.0));
  sink.Publish(MakeState(30.0));

  // FUS-OUT-002: "must overwrite old state with the latest, not display a
  // backlog to guarantee every result is delivered" -- only the most
  // recent Publish() call's state may ever be visible.
  const auto latest = sink.Latest();
  ASSERT_TRUE(latest.has_value());
  EXPECT_DOUBLE_EQ(latest->data_age_ms(), 30.0);
}

TEST(LatestAssistSink, RepeatedReadsSeeTheSameLatestValueUntilPublishedAgain) {
  LatestAssistSink sink;
  sink.Publish(MakeState(5.0));

  EXPECT_DOUBLE_EQ(sink.Latest()->data_age_ms(), 5.0);
  EXPECT_DOUBLE_EQ(sink.Latest()->data_age_ms(), 5.0);
}
