// docs/archive/rov-realtime-closed-loop-code-review-2026-08-27.md finding A2:
// realtime_gate.py's evaluate_gate() needs ~15 runtime metrics that
// nothing in the C++ gateway ever collected. This tests the collector in
// isolation, with fake RSS/CPU readers standing in for real /proc access
// (ReadProcessRssMib/ReadSystemCpuJiffies themselves are Linux-specific
// and exercised only implicitly via a live run, matching this repo's
// established pattern for OS-boundary code).
#include "application/runtime_metrics_collector.hpp"

#include <optional>
#include <string>

#include <gtest/gtest.h>

using uw::application::CpuJiffies;
using uw::application::OnlineAssistPipelineDiagnostics;
using uw::application::RuntimeMetricsCollector;
using uw::application::RuntimeMetricsConfig;

namespace {

uw::domain::OperatorAssistState MakeState(bool guidance_valid, double data_age_ms,
                                          uw::domain::HealthReport::Status status =
                                              uw::domain::HealthReport::STATUS_HEALTHY) {
  uw::domain::OperatorAssistState state;
  state.set_guidance_valid(guidance_valid);
  state.set_data_age_ms(data_age_ms);
  state.mutable_system_health()->set_status(status);
  return state;
}

uw::domain::ObservationHeader MakeHeader(double capture_s) {
  uw::domain::ObservationHeader header;
  *header.mutable_capture_time() = uw::domain::FromSeconds(capture_s);
  return header;
}

uw::domain::HealthReport MakeQueueLane(uint32_t watermark, uint64_t dropped, uint64_t rejected) {
  uw::domain::HealthReport report;
  report.set_queue_high_watermark(watermark);
  report.set_dropped_frame_count(dropped);
  report.set_rejected_frame_count(rejected);
  return report;
}

// Finds "field":value in a flat JSON object string -- this repo has no
// JSON parsing library available to tests (see
// tests/application/holoocean_status_json_test.cpp's own doc comment for
// the same constraint), so assertions are substring/structural checks.
bool ContainsField(const std::string& json, const std::string& field) {
  return json.find("\"" + field + "\":") != std::string::npos;
}

}  // namespace

TEST(RuntimeMetricsCollector, TracksResultAgePercentilesAndDeadlineMissFraction) {
  RuntimeMetricsCollector collector(RuntimeMetricsConfig{/*deadline_ms=*/250.0});
  collector.ObservePublish(MakeState(true, 100.0), 1.0);
  collector.ObservePublish(MakeState(true, 200.0), 2.0);
  collector.ObservePublish(MakeState(true, 300.0), 3.0);  // exceeds 250ms deadline

  const auto json = collector.BuildReportJson();
  EXPECT_TRUE(ContainsField(json, "result_age_p50_ms"));
  // 1 miss out of 3 publishes.
  EXPECT_NE(json.find("\"deadline_miss_fraction\":0.333"), std::string::npos);
}

TEST(RuntimeMetricsCollector, GuidanceMarkedStaleWhenOverdueRequiresBothConditions) {
  RuntimeMetricsConfig config;
  config.stale_guidance_threshold_ms = 500.0;
  RuntimeMetricsCollector collector(config);

  // Overdue but STILL marked valid -- the mechanism has NOT been proven
  // correct yet (this would actually be a bug if it happened for real).
  collector.ObservePublish(MakeState(/*guidance_valid=*/true, 600.0), 1.0);
  EXPECT_NE(collector.BuildReportJson().find("\"guidance_marked_stale_when_overdue\":false"),
            std::string::npos);

  // Overdue AND correctly marked invalid -- now proven.
  collector.ObservePublish(MakeState(/*guidance_valid=*/false, 600.0), 2.0);
  EXPECT_NE(collector.BuildReportJson().find("\"guidance_marked_stale_when_overdue\":true"),
            std::string::npos);
}

TEST(RuntimeMetricsCollector, NotOverdueNeverSetsStaleWhenOverdueFlag) {
  RuntimeMetricsCollector collector(RuntimeMetricsConfig{});
  collector.ObservePublish(MakeState(true, 50.0), 1.0);
  EXPECT_NE(collector.BuildReportJson().find("\"guidance_marked_stale_when_overdue\":false"),
            std::string::npos);
}

TEST(RuntimeMetricsCollector, TracksMaxRecoveryDurationAcrossOneEpisode) {
  RuntimeMetricsCollector collector(RuntimeMetricsConfig{});
  collector.ObservePublish(MakeState(true, 10.0, uw::domain::HealthReport::STATUS_HEALTHY), 0.0);
  collector.ObservePublish(MakeState(false, 10.0, uw::domain::HealthReport::STATUS_RECOVERING), 1.0);
  collector.ObservePublish(MakeState(false, 10.0, uw::domain::HealthReport::STATUS_RECOVERING), 2.5);
  collector.ObservePublish(MakeState(true, 10.0, uw::domain::HealthReport::STATUS_HEALTHY), 4.0);
  // Recovering held from wall_s=1.0 to wall_s=4.0 -> 3.0s.

  EXPECT_NE(collector.BuildReportJson().find("\"recovery_duration_s_max\":3"), std::string::npos);
}

TEST(RuntimeMetricsCollector, RecoveryDurationTracksTheLongestOfMultipleEpisodes) {
  RuntimeMetricsCollector collector(RuntimeMetricsConfig{});
  collector.ObservePublish(MakeState(false, 10.0, uw::domain::HealthReport::STATUS_RECOVERING), 0.0);
  collector.ObservePublish(MakeState(true, 10.0, uw::domain::HealthReport::STATUS_HEALTHY), 1.0);  // 1.0s episode
  collector.ObservePublish(MakeState(false, 10.0, uw::domain::HealthReport::STATUS_RECOVERING), 5.0);
  collector.ObservePublish(MakeState(true, 10.0, uw::domain::HealthReport::STATUS_HEALTHY), 10.0);  // 5.0s episode

  EXPECT_NE(collector.BuildReportJson().find("\"recovery_duration_s_max\":5"), std::string::npos);
}

TEST(RuntimeMetricsCollector, TracksStateAgeFromVehicleStateHeaders) {
  RuntimeMetricsCollector collector(RuntimeMetricsConfig{});
  collector.ObserveVehicleState(MakeHeader(10.0), /*wall_now_s=*/10.05);  // 50ms age

  const auto json = collector.BuildReportJson();
  EXPECT_TRUE(ContainsField(json, "state_age_p50_ms"));
  EXPECT_NE(json.find("\"state_age_p50_ms\":50"), std::string::npos);
}

TEST(RuntimeMetricsCollector, ComputesRtfFromConsecutiveSimTimeObservations) {
  RuntimeMetricsCollector collector(RuntimeMetricsConfig{});
  collector.ObserveSimTime(/*capture_sim_s=*/0.0, /*wall_now_s=*/100.0);
  collector.ObserveSimTime(/*capture_sim_s=*/1.0, /*wall_now_s=*/101.0);  // 1.0s sim / 1.0s wall = RTF 1.0

  EXPECT_NE(collector.BuildReportJson().find("\"rtf_p50\":1"), std::string::npos);
}

TEST(RuntimeMetricsCollector, FirstSimTimeObservationProducesNoRtfSampleYet) {
  RuntimeMetricsCollector collector(RuntimeMetricsConfig{});
  collector.ObserveSimTime(0.0, 100.0);

  // No RollingLatency sample yet -- Snapshot() on an empty window reports
  // sample_count 0 and zeroed percentiles, which would misleadingly read
  // as "RTF is exactly 0" if this weren't distinguished; this test just
  // pins down that a single observation alone doesn't fabricate a sample.
  EXPECT_NE(collector.BuildReportJson().find("\"rtf_p50\":0"), std::string::npos);
}

TEST(RuntimeMetricsCollector, QueueHealthTracksMaxWatermarkAndCumulativeDropsRejects) {
  RuntimeMetricsCollector collector(RuntimeMetricsConfig{});
  collector.ObserveQueueHealth({MakeQueueLane(3, 0, 0), MakeQueueLane(1, 2, 0), MakeQueueLane(0, 0, 0),
                                MakeQueueLane(0, 0, 0)});
  collector.ObserveQueueHealth({MakeQueueLane(5, 0, 1), MakeQueueLane(1, 4, 0), MakeQueueLane(2, 0, 0),
                                MakeQueueLane(0, 0, 0)});

  const auto json = collector.BuildReportJson();
  EXPECT_NE(json.find("\"queue_high_watermark\":5"), std::string::npos);
  EXPECT_NE(json.find("\"queue_drops\":4"), std::string::npos);  // last snapshot's cumulative value, not summed
  EXPECT_NE(json.find("\"queue_rejects\":1"), std::string::npos);
  EXPECT_NE(json.find("\"queue_capacity_violations\":0"), std::string::npos);
}

TEST(RuntimeMetricsCollector, DetectsQueueCapacityViolationWhenWatermarkReachesConfiguredCapacity) {
  RuntimeMetricsConfig config;
  config.queue_lane_capacities = {64, 32, 16, 256};
  RuntimeMetricsCollector collector(config);
  collector.ObserveQueueHealth({MakeQueueLane(64, 0, 0), MakeQueueLane(10, 0, 0), MakeQueueLane(0, 0, 0),
                                MakeQueueLane(0, 0, 0)});

  EXPECT_NE(collector.BuildReportJson().find("\"queue_capacity_violations\":1"), std::string::npos);
}

// All three RSS tests below explicitly stub read_cpu_jiffies to nullopt --
// otherwise the default real ReadSystemCpuJiffies would read this test
// process's actual /proc/stat, which doesn't affect these assertions but
// would make the test's behavior depend on real host state rather than
// being fully hermetic.
TEST(RuntimeMetricsCollector, RssGrowthFieldOmittedUntilAnySampleObserved) {
  RuntimeMetricsCollector collector(
      RuntimeMetricsConfig{}, /*read_rss_mib=*/[] { return std::optional<double>(100.0); },
      /*read_cpu_jiffies=*/[] { return std::optional<CpuJiffies>(); });
  // Only ONE sample so far -- establishes the warmup baseline, produces no
  // growth figure yet.
  collector.SampleResourceUsage(/*process_uptime_s=*/700.0, /*warmup_s=*/600.0);

  EXPECT_FALSE(ContainsField(collector.BuildReportJson(), "rss_growth_after_warmup_mib"));
}

TEST(RuntimeMetricsCollector, RssGrowthIsMaxAboveThePostWarmupBaseline) {
  double rss = 100.0;
  RuntimeMetricsCollector collector(
      RuntimeMetricsConfig{}, /*read_rss_mib=*/[&rss] { return std::optional<double>(rss); },
      /*read_cpu_jiffies=*/[] { return std::optional<CpuJiffies>(); });
  collector.SampleResourceUsage(700.0, 600.0);  // baseline = 100.0
  rss = 130.0;
  collector.SampleResourceUsage(800.0, 600.0);  // +30
  rss = 120.0;
  collector.SampleResourceUsage(900.0, 600.0);  // +20 -- must not overwrite the max with a smaller value

  EXPECT_NE(collector.BuildReportJson().find("\"rss_growth_after_warmup_mib\":30"), std::string::npos);
}

TEST(RuntimeMetricsCollector, SamplesBeforeWarmupNeverEstablishBaseline) {
  RuntimeMetricsCollector collector(
      RuntimeMetricsConfig{}, /*read_rss_mib=*/[] { return std::optional<double>(100.0); },
      /*read_cpu_jiffies=*/[] { return std::optional<CpuJiffies>(); });
  collector.SampleResourceUsage(/*process_uptime_s=*/10.0, /*warmup_s=*/600.0);

  EXPECT_FALSE(ContainsField(collector.BuildReportJson(), "rss_growth_after_warmup_mib"));
}

TEST(RuntimeMetricsCollector, CpuHeadroomAveragesUtilizationAcrossSamples) {
  std::vector<CpuJiffies> samples = {
      {/*idle=*/800, /*total=*/1000},   // anchor only
      {/*idle=*/1600, /*total=*/2000},  // delta: idle+800/total+1000 -> util 20% -> headroom 80%
      {/*idle=*/2200, /*total=*/3000},  // delta: idle+600/total+1000 -> util 40% -> headroom 60%
  };
  std::size_t index = 0;
  RuntimeMetricsCollector collector(
      RuntimeMetricsConfig{}, /*read_rss_mib=*/[] { return std::optional<double>(); },
      /*read_cpu_jiffies=*/[&]() -> std::optional<CpuJiffies> { return samples[index++]; });

  collector.SampleResourceUsage(0.0);
  collector.SampleResourceUsage(1.0);
  collector.SampleResourceUsage(2.0);

  // Average of 0.80 and 0.60 -> 0.70.
  EXPECT_NE(collector.BuildReportJson().find("\"cpu_headroom_fraction_avg\":0.7"), std::string::npos);
}

TEST(RuntimeMetricsCollector, GpuHeadroomIsNeverEmittedRegardlessOfWhatElseWasObserved) {
  RuntimeMetricsCollector collector(RuntimeMetricsConfig{});
  collector.ObservePublish(MakeState(true, 10.0), 1.0);
  collector.SampleResourceUsage(1000.0);

  // Not measured, ever -- see the header's own doc comment for why this
  // is a deliberate omission, not an oversight.
  EXPECT_FALSE(ContainsField(collector.BuildReportJson(), "gpu_headroom_fraction_avg"));
}

TEST(RuntimeMetricsCollector, ReportCarriesTheMostRecentlyObservedDiagnostics) {
  RuntimeMetricsCollector collector(RuntimeMetricsConfig{});
  OnlineAssistPipelineDiagnostics diagnostics;
  diagnostics.sonar_detection_count = 42;
  diagnostics.visual_detection_count = 17;
  diagnostics.fused_track_publish_count = 3;
  collector.ObserveDiagnostics(diagnostics);

  const auto json = collector.BuildReportJson();
  EXPECT_NE(json.find("\"sonar_detection_count\":42"), std::string::npos);
  EXPECT_NE(json.find("\"visual_detection_count\":17"), std::string::npos);
  EXPECT_NE(json.find("\"fused_track_count\":3"), std::string::npos);
}
