#include "runtime/synthetic_sonar.hpp"

#include <cstddef>

#include <gtest/gtest.h>

namespace {

TEST(SyntheticSonar, RendersThreeBeamTargetAndCanonicalMetadata) {
  uw::runtime::SyntheticSonarFrameSpec spec;
  spec.observation_id = "kf7";
  spec.sensor_id = "sonar0";
  spec.provenance = "unit_test";
  spec.timestamp_ns = 2'500'000'123ULL;

  const auto result = uw::runtime::RenderSyntheticSonarFrame(spec, 4.0, 0.0);

  EXPECT_TRUE(result.target_rendered);
  EXPECT_EQ(result.frame.header().observation_id().value(), "kf7");
  EXPECT_EQ(result.frame.header().sensor_id().value(), "sonar0");
  EXPECT_EQ(result.frame.header().sensor_frame().value(), "sonar_link");
  EXPECT_EQ(result.frame.header().capture_time().seconds(), 2);
  EXPECT_EQ(result.frame.header().capture_time().nanos(), 500'000'123);
  // Synthetic generation has no real transport delay — receive_time must
  // equal capture_time, not be left at the zero-Stamp default (a prior gap;
  // see this file's git history / apps/bag_audit's findings).
  EXPECT_EQ(result.frame.header().receive_time().seconds(), 2);
  EXPECT_EQ(result.frame.header().receive_time().nanos(), 500'000'123);
  EXPECT_EQ(result.frame.header().clock_domain(), uw::domain::CLOCK_DOMAIN_SIMULATION);
  EXPECT_EQ(result.frame.header().provenance(), "unit_test");
  EXPECT_EQ(result.frame.range_bins_size(), static_cast<int>(spec.num_ranges + 1));
  EXPECT_EQ(result.frame.azimuth_angles_size(), static_cast<int>(spec.num_beams));
  EXPECT_EQ(result.frame.intensity_tensor().size(),
            static_cast<std::size_t>(spec.num_ranges) * spec.num_beams);

  std::size_t bright_pixels = 0;
  for (unsigned char value : result.frame.intensity_tensor()) {
    if (value == spec.target_intensity) ++bright_pixels;
  }
  EXPECT_EQ(bright_pixels, 3u);
}

TEST(SyntheticSonar, LeavesBackgroundOnlyWhenTargetFallsOutsideFrame) {
  uw::runtime::SyntheticSonarFrameSpec spec;
  const auto result = uw::runtime::RenderSyntheticSonarFrame(
      spec, spec.max_range_m + 1.0, spec.horizontal_fov_rad);

  EXPECT_FALSE(result.target_rendered);
  for (unsigned char value : result.frame.intensity_tensor()) {
    EXPECT_EQ(value, spec.background_intensity);
  }
}

}  // namespace
