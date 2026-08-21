#include "runtime/mcap_io.hpp"

#include <cstdio>
#include <vector>

#include <gtest/gtest.h>

#include "domain/domain.hpp"

using uw::runtime::McapProtobufWriter;
using uw::runtime::ReadMcapMessages;

TEST(McapIo, RoundTripsProtobufMessages) {
  const std::string path = "uw_runtime_mcap_io_test.mcap";

  {
    McapProtobufWriter writer;
    ASSERT_TRUE(writer.Open(path));
    for (int i = 0; i < 3; ++i) {
      uw::domain::HealthReport report;
      report.set_component_id("test_component");
      report.set_status(uw::domain::HealthReport::STATUS_HEALTHY);
      report.set_queue_depth(static_cast<uint32_t>(i));
      ASSERT_TRUE(writer.WriteMessage("/health", static_cast<uint64_t>(i) * 1000, report));
    }
    writer.Close();
  }

  std::vector<uint32_t> queue_depths;
  const bool ok = ReadMcapMessages<uw::domain::HealthReport>(
      path, "/health",
      [&](uint64_t /*log_time_ns*/, const uw::domain::HealthReport& report) {
        EXPECT_EQ(report.component_id(), "test_component");
        queue_depths.push_back(report.queue_depth());
      });

  ASSERT_TRUE(ok);
  ASSERT_EQ(queue_depths.size(), 3u);
  EXPECT_EQ(queue_depths[0], 0u);
  EXPECT_EQ(queue_depths[1], 1u);
  EXPECT_EQ(queue_depths[2], 2u);

  std::remove(path.c_str());
}
