#include "frontends/sonar_cfar_frontend.hpp"

#include <chrono>
#include <cmath>
#include <locale>

#include <gtest/gtest.h>

using uw::frontends::SonarCfarFrontend;
using uw::frontends::SonarCfarFrontendParams;

namespace {

uw::domain::SonarFrame MakeSyntheticFrame(int num_ranges, int num_beams, double range_resolution,
                                           int target_row, int target_col, uint8_t background = 5,
                                           uint8_t foreground = 200, int column_half_width = 1) {
  uw::domain::SonarFrame frame;
  frame.mutable_header()->mutable_sensor_frame()->set_value("sonar_link");
  frame.mutable_header()->mutable_observation_id()->set_value("obs_1");
  frame.set_num_ranges(static_cast<uint32_t>(num_ranges));
  frame.set_num_beams(static_cast<uint32_t>(num_beams));
  frame.set_range_resolution(static_cast<float>(range_resolution));
  frame.set_min_range(0.0f);
  frame.set_max_range(static_cast<float>(num_ranges * range_resolution));

  for (int r = 0; r <= num_ranges; ++r) frame.add_range_bins(static_cast<float>(r * range_resolution));
  const double fov = 1.0;  // radians, total
  for (int c = 0; c < num_beams; ++c) {
    frame.add_azimuth_angles(static_cast<float>(-fov / 2.0 + fov * c / (num_beams - 1)));
  }

  std::string bytes(static_cast<std::size_t>(num_ranges) * num_beams,
                    static_cast<char>(background));
  // A target with a little angular extent: bright at the same range across
  // a few adjacent bearing columns. First-contact extraction yields at most
  // ONE detection per column (nearest range with a hit), so a target that
  // only varies across RANGE within a single column would produce just one
  // point total — DBSCAN (min_samples=2) can never cluster a lone point.
  // Real targets have finite angular extent, so spreading across columns
  // here matches physical reality, not just what the pipeline needs.
  for (int dc = -column_half_width; dc <= column_half_width; ++dc) {
    const int c = target_col + dc;
    if (c < 0 || c >= num_beams) continue;
    if (target_row >= 0 && target_row < num_ranges) {
      bytes[static_cast<std::size_t>(target_row) * num_beams + c] =
          static_cast<char>(foreground);
    }
  }
  frame.set_intensity_tensor(bytes);
  frame.set_encoding(uw::domain::SonarFrame::ENCODING_UINT8_GRAY);
  return frame;
}

SonarCfarFrontendParams TestCfarParams() {
  SonarCfarFrontendParams params;
  params.cfar.num_training_cells = 16;
  params.cfar.num_guard_cells = 4;
  params.cfar.probability_false_alarm = 1e-2;
  params.detector_threshold = 50;
  return params;
}

class GroupHexDigits : public std::numpunct<char> {
 protected:
  char do_thousands_sep() const override { return '_'; }
  std::string do_grouping() const override { return "\3"; }
};

}  // namespace

TEST(SonarCfarFrontend, DetectsSyntheticTargetNearExpectedRangeBearing) {
  constexpr int kNumRanges = 64;
  constexpr int kNumBeams = 32;
  constexpr double kRangeResolution = 0.1;
  constexpr int kTargetRow = 40;
  constexpr int kTargetCol = 16;

  const auto frame = MakeSyntheticFrame(kNumRanges, kNumBeams, kRangeResolution, kTargetRow, kTargetCol);

  auto params = TestCfarParams();

  SonarCfarFrontend frontend(params);
  const auto hypotheses = frontend.ProcessSonarFrame(frame);

  ASSERT_GT(hypotheses.candidates_size(), 0) << "expected at least one detected cluster";

  ASSERT_TRUE(uw::domain::HasPayload<uw::domain::SonarRangeBearing>(hypotheses.candidates(0)));
  const auto& first =
      uw::domain::GetPayload<uw::domain::SonarRangeBearing>(hypotheses.candidates(0));

  const double expected_range = kTargetRow * kRangeResolution;
  const double expected_bearing = frame.azimuth_angles(kTargetCol);
  EXPECT_NEAR(first.range_m(), expected_range, kRangeResolution * 2);
  EXPECT_NEAR(first.bearing_rad(), expected_bearing, 0.05);
}

// docs/archive/rov-realtime-closed-loop-code-review-2026-08-27.md finding D3: every
// detection used to report the same fixed default_bearing_sigma_rad/
// default_range_sigma_m regardless of cluster spread, over-trusting sonar
// relative to vision in TargetAssociator::Fuse() whenever a cluster was
// genuinely more spread out (coarser beam resolution, low SNR) than the
// default assumed.
TEST(SonarCfarFrontend, WiderClusterReportsLargerSigmaThanDefault) {
  constexpr int kNumRanges = 64;
  constexpr int kNumBeams = 32;
  constexpr double kRangeResolution = 0.1;
  constexpr int kTargetRow = 40;
  constexpr int kTargetCol = 16;

  // 7 columns wide (half_width=3) vs the narrow 3-column baseline --
  // clearly more angular spread than a tight, well-resolved detection.
  const auto frame = MakeSyntheticFrame(kNumRanges, kNumBeams, kRangeResolution, kTargetRow,
                                        kTargetCol, /*background=*/5, /*foreground=*/200,
                                        /*column_half_width=*/3);

  auto params = TestCfarParams();  // default_bearing_sigma_rad=0.01, default_range_sigma_m=0.05
  SonarCfarFrontend frontend(params);
  const auto hypotheses = frontend.ProcessSonarFrame(frame);

  ASSERT_GT(hypotheses.candidates_size(), 0);
  const auto& measurement =
      uw::domain::GetPayload<uw::domain::SonarRangeBearing>(hypotheses.candidates(0));
  EXPECT_GT(measurement.bearing_sigma_rad(), params.default_bearing_sigma_rad);
}

TEST(SonarCfarFrontend, WiderClusterReportsLargerSigmaThanNarrowerCluster) {
  constexpr int kNumRanges = 64;
  constexpr int kNumBeams = 32;
  constexpr double kRangeResolution = 0.1;
  constexpr int kTargetRow = 40;
  constexpr int kTargetCol = 16;

  const auto narrow_frame = MakeSyntheticFrame(kNumRanges, kNumBeams, kRangeResolution, kTargetRow,
                                               kTargetCol, 5, 200, /*column_half_width=*/1);
  const auto wide_frame = MakeSyntheticFrame(kNumRanges, kNumBeams, kRangeResolution, kTargetRow,
                                             kTargetCol, 5, 200, /*column_half_width=*/4);

  auto params = TestCfarParams();
  SonarCfarFrontend narrow_frontend(params);
  SonarCfarFrontend wide_frontend(params);
  const auto narrow_hypotheses = narrow_frontend.ProcessSonarFrame(narrow_frame);
  const auto wide_hypotheses = wide_frontend.ProcessSonarFrame(wide_frame);

  ASSERT_GT(narrow_hypotheses.candidates_size(), 0);
  ASSERT_GT(wide_hypotheses.candidates_size(), 0);
  const auto& narrow =
      uw::domain::GetPayload<uw::domain::SonarRangeBearing>(narrow_hypotheses.candidates(0));
  const auto& wide =
      uw::domain::GetPayload<uw::domain::SonarRangeBearing>(wide_hypotheses.candidates(0));
  EXPECT_GT(wide.bearing_sigma_rad(), narrow.bearing_sigma_rad());
}

TEST(SonarCfarFrontend, RejectsNonAscendingAzimuth) {
  auto frame = MakeSyntheticFrame(32, 16, 0.1, 10, 8);
  // Corrupt azimuth ordering.
  frame.set_azimuth_angles(0, frame.azimuth_angles(frame.azimuth_angles_size() - 1));

  SonarCfarFrontendParams params;
  SonarCfarFrontend frontend(params);
  const auto hypotheses = frontend.ProcessSonarFrame(frame);

  EXPECT_TRUE(hypotheses.out_of_distribution());
  EXPECT_EQ(hypotheses.candidates_size(), 0);
}

TEST(SonarCfarFrontend, ReportsElevatedBackgroundNoiseWithExactReason) {
  auto params = TestCfarParams();
  params.health.max_background_noise_mean = 20.0;
  params.health.max_false_alarm_density = 1.0;
  params.health.min_valid_measurements = 0;
  SonarCfarFrontend frontend(params);
  frontend.ProcessSonarFrame(MakeSyntheticFrame(64, 32, 0.1, 40, 16, 80, 240));

  const auto health = frontend.Health();
  EXPECT_GT(health.background_noise_mean(), 20.0);
  EXPECT_EQ(health.status(), uw::domain::HealthReport::STATUS_SUSPECT);
  EXPECT_EQ(health.reason_code(), "sonar_background_noise_high");
}

TEST(SonarCfarFrontend, ReportsExcessiveFalseAlarmDensityWithExactReason) {
  auto frame = MakeSyntheticFrame(64, 32, 0.1, -1, -1);
  std::string bytes = frame.intensity_tensor();
  for (int col = 2; col < 30; col += 4) {
    bytes[static_cast<std::size_t>(20 + col % 3) * 32 + col] = static_cast<char>(220);
  }
  frame.set_intensity_tensor(bytes);

  auto params = TestCfarParams();
  params.dbscan_eps_m = 0.001;
  params.dbscan_min_samples = 2;
  params.health.max_background_noise_mean = 255.0;
  params.health.max_false_alarm_density = 0.5;
  params.health.min_valid_measurements = 0;
  SonarCfarFrontend frontend(params);
  frontend.ProcessSonarFrame(frame);

  const auto health = frontend.Health();
  EXPECT_GT(health.false_alarm_density(), 0.5);
  EXPECT_EQ(health.status(), uw::domain::HealthReport::STATUS_SUSPECT);
  EXPECT_EQ(health.reason_code(), "sonar_false_alarm_density_high");
}

TEST(SonarCfarFrontend, ReportsNoValidMeasurementsWithExactReason) {
  auto params = TestCfarParams();
  params.health.max_background_noise_mean = 255.0;
  params.health.max_false_alarm_density = 1.0;
  params.health.min_valid_measurements = 1;
  SonarCfarFrontend frontend(params);
  frontend.ProcessSonarFrame(MakeSyntheticFrame(64, 32, 0.1, -1, -1));

  const auto health = frontend.Health();
  EXPECT_EQ(health.valid_measurement_count(), 0u);
  EXPECT_EQ(health.status(), uw::domain::HealthReport::STATUS_SUSPECT);
  EXPECT_EQ(health.reason_code(), "sonar_no_valid_measurements");
}

TEST(SonarCfarFrontend, ReportsSlowProcessingWithExactReason) {
  auto params = TestCfarParams();
  params.health.max_background_noise_mean = 255.0;
  params.health.max_false_alarm_density = 1.0;
  params.health.max_processing_latency_ms = 10.0;
  int call_count = 0;
  const auto epoch = std::chrono::steady_clock::time_point{};
  SonarCfarFrontend frontend(params, [&] {
    return epoch + std::chrono::milliseconds(call_count++ == 0 ? 0 : 25);
  });
  frontend.ProcessSonarFrame(MakeSyntheticFrame(64, 32, 0.1, 40, 16));

  const auto health = frontend.Health();
  EXPECT_DOUBLE_EQ(health.latency_p95_ms(), 25.0);
  EXPECT_EQ(health.status(), uw::domain::HealthReport::STATUS_SUSPECT);
  EXPECT_EQ(health.reason_code(), "sonar_processing_latency_high");
}

TEST(SonarCfarFrontend, ReportsConfigHashMismatchWithExactReason) {
  auto params = TestCfarParams();
  params.health.max_background_noise_mean = 255.0;
  params.health.max_false_alarm_density = 1.0;
  params.expected_config_hash = "deployment_expected_hash";
  SonarCfarFrontend frontend(params);
  frontend.ProcessSonarFrame(MakeSyntheticFrame(64, 32, 0.1, 40, 16));

  const auto health = frontend.Health();
  EXPECT_FALSE(health.config_hash_consistent());
  EXPECT_NE(frontend.ActiveConfigHash(), params.expected_config_hash);
  EXPECT_EQ(health.status(), uw::domain::HealthReport::STATUS_SUSPECT);
  EXPECT_EQ(health.reason_code(), "sonar_config_hash_mismatch");
}

TEST(SonarCfarFrontend, ActiveConfigHashCoversEveryBehaviorAndHealthField) {
  const auto baseline = TestCfarParams();
  const std::string baseline_hash = SonarCfarFrontend(baseline).ActiveConfigHash();
  EXPECT_EQ(baseline_hash, "c6d02b68cef98b7b");
  const auto expect_changed = [&](const SonarCfarFrontendParams& changed) {
    EXPECT_NE(SonarCfarFrontend(changed).ActiveConfigHash(), baseline_hash);
  };

  auto changed = baseline;
  ++changed.cfar.num_training_cells;
  expect_changed(changed);
  changed = baseline;
  ++changed.cfar.num_guard_cells;
  expect_changed(changed);
  changed = baseline;
  changed.cfar.probability_false_alarm *= 0.5;
  expect_changed(changed);
  changed = baseline;
  ++changed.cfar.rank;
  expect_changed(changed);
  changed = baseline;
  changed.cfar_variant = uw::frontends::CfarVariant::kCA;
  expect_changed(changed);
  changed = baseline;
  ++changed.detector_threshold;
  expect_changed(changed);
  changed = baseline;
  changed.dbscan_eps_m += 0.01;
  expect_changed(changed);
  changed = baseline;
  ++changed.dbscan_min_samples;
  expect_changed(changed);
  changed = baseline;
  changed.default_range_sigma_m += 0.01;
  expect_changed(changed);
  changed = baseline;
  changed.default_bearing_sigma_rad += 0.01;
  expect_changed(changed);
  changed = baseline;
  changed.health.max_background_noise_mean += 1.0;
  expect_changed(changed);
  changed = baseline;
  changed.health.max_false_alarm_density += 0.01;
  expect_changed(changed);
  changed = baseline;
  ++changed.health.min_valid_measurements;
  expect_changed(changed);
  changed = baseline;
  changed.health.max_processing_latency_ms += 1.0;
  expect_changed(changed);
}

TEST(SonarCfarFrontend, ActiveConfigHashIsIndependentOfProcessLocale) {
  const auto previous_locale = std::locale();
  std::locale::global(std::locale(std::locale::classic(), new GroupHexDigits));
  const std::string grouped_locale_hash = SonarCfarFrontend(TestCfarParams()).ActiveConfigHash();
  std::locale::global(previous_locale);

  EXPECT_EQ(grouped_locale_hash, "c6d02b68cef98b7b");
}

TEST(SonarCfarFrontend, EvictsOldProcessingLatencySamplesFromBoundedWindow) {
  auto params = TestCfarParams();
  params.health.max_background_noise_mean = 255.0;
  params.health.max_false_alarm_density = 1.0;
  params.health.max_processing_latency_ms = 10.0;
  std::size_t clock_call = 0;
  const auto epoch = std::chrono::steady_clock::time_point{};
  SonarCfarFrontend frontend(params, [&] {
    const std::size_t sample_index = clock_call / 2;
    const bool is_end = (clock_call++ % 2) == 1;
    const auto sample_start = epoch + std::chrono::seconds(sample_index);
    if (!is_end) return sample_start;
    return sample_start +
           (sample_index == 0 ? std::chrono::milliseconds(100) : std::chrono::milliseconds(1));
  });
  const auto frame = MakeSyntheticFrame(64, 32, 0.1, 40, 16);

  frontend.ProcessSonarFrame(frame);
  EXPECT_EQ(frontend.Health().status(), uw::domain::HealthReport::STATUS_SUSPECT);
  EXPECT_EQ(frontend.Health().reason_code(), "sonar_processing_latency_high");

  for (int i = 0; i < 32; ++i) frontend.ProcessSonarFrame(frame);

  EXPECT_DOUBLE_EQ(frontend.Health().latency_p99_ms(), 1.0);
  EXPECT_EQ(frontend.Health().status(), uw::domain::HealthReport::STATUS_HEALTHY);
  EXPECT_TRUE(frontend.Health().reason_code().empty());
}
