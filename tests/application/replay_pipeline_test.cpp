#include "application/replay_pipeline.hpp"

#include <gtest/gtest.h>

namespace {

TEST(ReplayPipeline, RejectsEmptyBagPath) {
  uw::application::ReplayOptions options;
  EXPECT_EQ(uw::application::RunReplayPipeline(options, "test-commit"), 1);
}

}  // namespace
