#include "uw/adapters/holoocean_ros_bridge_sonar_frame_provider.hpp"

#include <gtest/gtest.h>

using uw::adapters::HoloOceanImagingSonarParams;
using uw::adapters::HoloOceanRosBridgeSonarFrameProvider;

namespace {

HoloOceanImagingSonarParams MakeParams() {
  HoloOceanImagingSonarParams params;
  params.horizontal_fov_rad = 1.0f;  // matches sonar_cfar_frontend_test's synthetic fixture
  params.min_range_m = 0.0f;
  params.max_range_m = 6.4f;
  return params;
}

}  // namespace

TEST(HoloOceanRosBridgeSonarFrameProvider, DropsMalformedFrameAndReportsIt) {
  HoloOceanRosBridgeSonarFrameProvider provider(MakeParams());
  std::vector<float> too_short(10, 0.5f);
  provider.PushImagingSonar(0, 4, 4, too_short, "obs_bad");

  EXPECT_FALSE(provider.PollSonarFrame().has_value());
  EXPECT_EQ(provider.Health().dropped_frame_count(), 1u);
  EXPECT_EQ(provider.Health().status(), uw::domain::HealthReport::STATUS_UNSPECIFIED);
}

TEST(HoloOceanRosBridgeSonarFrameProvider, ConvertsFieldsAndProducesAscendingBearing) {
  HoloOceanRosBridgeSonarFrameProvider provider(MakeParams());

  constexpr uint32_t kNumRanges = 4;
  constexpr uint32_t kNumBeams = 3;
  // Row-major [num_ranges, num_beams], values in [0, 1] per ImagingSonar's
  // documented convention; row 2 carries a bright cell in the last column
  // (pre-mirror) to check the mirror-flip lands it in the first column.
  std::vector<float> image_range(kNumRanges * kNumBeams, 0.1f);
  image_range[2 * kNumBeams + (kNumBeams - 1)] = 0.9f;

  constexpr int64_t kTimestampNs = 12'345'000'000;  // 12.345s sim time
  provider.PushImagingSonar(kTimestampNs, kNumRanges, kNumBeams, image_range, "obs_1");
  EXPECT_EQ(provider.Health().queue_depth(), 1u);

  auto frame = provider.PollSonarFrame();
  ASSERT_TRUE(frame.has_value());
  EXPECT_FALSE(provider.PollSonarFrame().has_value());

  EXPECT_EQ(frame->header().observation_id().value(), "obs_1");
  EXPECT_EQ(frame->header().clock_domain(), uw::domain::CLOCK_DOMAIN_SIMULATION);
  EXPECT_EQ(frame->header().capture_time().seconds(), 12);
  EXPECT_EQ(frame->header().capture_time().nanos(), 345'000'000);
  EXPECT_EQ(frame->header().validity(), uw::domain::ObservationHeader::VALIDITY_OK);

  EXPECT_EQ(frame->num_ranges(), kNumRanges);
  EXPECT_EQ(frame->num_beams(), kNumBeams);
  EXPECT_EQ(frame->range_bins_size(), static_cast<int>(kNumRanges) + 1);
  EXPECT_EQ(frame->azimuth_angles_size(), static_cast<int>(kNumBeams));
  EXPECT_TRUE(uw::domain::IsAzimuthAscending(*frame));

  // Mirror-flip: the bright cell was at row 2, last column pre-flip -> first
  // column post-flip.
  ASSERT_EQ(frame->intensity_tensor().size(), kNumRanges * kNumBeams);
  const unsigned char mirrored_cell =
      static_cast<unsigned char>(frame->intensity_tensor()[2 * kNumBeams + 0]);
  const unsigned char untouched_cell =
      static_cast<unsigned char>(frame->intensity_tensor()[2 * kNumBeams + 1]);
  EXPECT_GT(mirrored_cell, untouched_cell);
  EXPECT_NEAR(mirrored_cell, 229, 1);  // round(0.9 * 255)

  EXPECT_EQ(provider.Health().status(), uw::domain::HealthReport::STATUS_HEALTHY);
}
