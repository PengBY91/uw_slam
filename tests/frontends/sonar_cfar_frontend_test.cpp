#include "frontends/sonar_cfar_frontend.hpp"

#include <cmath>

#include <gtest/gtest.h>

using uw::frontends::SonarCfarFrontend;
using uw::frontends::SonarCfarFrontendParams;

namespace {

uw::domain::SonarFrame MakeSyntheticFrame(int num_ranges, int num_beams, double range_resolution,
                                           int target_row, int target_col) {
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

  std::string bytes(static_cast<std::size_t>(num_ranges) * num_beams, static_cast<char>(5));
  // A target with a little angular extent: bright at the same range across
  // a few adjacent bearing columns. First-contact extraction yields at most
  // ONE detection per column (nearest range with a hit), so a target that
  // only varies across RANGE within a single column would produce just one
  // point total — DBSCAN (min_samples=2) can never cluster a lone point.
  // Real targets have finite angular extent, so spreading across columns
  // here matches physical reality, not just what the pipeline needs.
  for (int dc = -1; dc <= 1; ++dc) {
    const int c = target_col + dc;
    if (c < 0 || c >= num_beams) continue;
    bytes[static_cast<std::size_t>(target_row) * num_beams + c] = static_cast<char>(200);
  }
  frame.set_intensity_tensor(bytes);
  frame.set_encoding(uw::domain::SonarFrame::ENCODING_UINT8_GRAY);
  return frame;
}

}  // namespace

TEST(SonarCfarFrontend, DetectsSyntheticTargetNearExpectedRangeBearing) {
  constexpr int kNumRanges = 64;
  constexpr int kNumBeams = 32;
  constexpr double kRangeResolution = 0.1;
  constexpr int kTargetRow = 40;
  constexpr int kTargetCol = 16;

  const auto frame = MakeSyntheticFrame(kNumRanges, kNumBeams, kRangeResolution, kTargetRow, kTargetCol);

  SonarCfarFrontendParams params;
  params.cfar.num_training_cells = 16;
  params.cfar.num_guard_cells = 4;
  params.cfar.probability_false_alarm = 1e-2;
  params.detector_threshold = 50;

  SonarCfarFrontend frontend(params);
  const auto hypotheses = frontend.ProcessSonarFrame(frame);

  ASSERT_GT(hypotheses.candidates_size(), 0) << "expected at least one detected cluster";

  const auto top = uw::domain::TopCandidate<uw::domain::SonarRangeBearing>(hypotheses);
  ASSERT_TRUE(top.has_value());

  const double expected_range = kTargetRow * kRangeResolution;
  const double expected_bearing = frame.azimuth_angles(kTargetCol);
  EXPECT_NEAR(top->range_m(), expected_range, kRangeResolution * 2);
  EXPECT_NEAR(top->bearing_rad(), expected_bearing, 0.05);
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
