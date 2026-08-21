#include "uw/frontends/harris_corner_detector.hpp"

#include <cmath>
#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

using uw::frontends::HarrisCornerDetector;
using uw::frontends::HarrisCornerDetectorParams;

namespace {

// A single, strong Harris corner: the whole image is split into four
// quadrants around (cx, cv), diagonal quadrants sharing a color — the
// classic "chessboard corner" test pattern, with exactly one point where
// gradient direction genuinely varies in every direction.
std::vector<uint8_t> MakeCheckerCorner(uint32_t width, uint32_t height, int cx, int cy, uint8_t dark,
                                        uint8_t bright) {
  std::vector<uint8_t> image(static_cast<std::size_t>(width) * height, dark);
  for (uint32_t v = 0; v < height; ++v) {
    const bool top = static_cast<int>(v) < cy;
    for (uint32_t u = 0; u < width; ++u) {
      const bool left = static_cast<int>(u) < cx;
      if (left == top) image[static_cast<std::size_t>(v) * width + u] = bright;
    }
  }
  return image;
}

// A grid of isolated bright squares, spaced far enough apart that each
// square's own corners never fall inside another square's nms_radius.
std::vector<uint8_t> MakeSquareGrid(uint32_t width, uint32_t height, int cols, int rows, int spacing,
                                     int half_size, uint8_t dark, uint8_t bright) {
  std::vector<uint8_t> image(static_cast<std::size_t>(width) * height, dark);
  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < cols; ++col) {
      const int cx = spacing / 2 + col * spacing;
      const int cy = spacing / 2 + row * spacing;
      for (int dv = -half_size; dv <= half_size; ++dv) {
        const int v = cy + dv;
        if (v < 0 || v >= static_cast<int>(height)) continue;
        for (int du = -half_size; du <= half_size; ++du) {
          const int u = cx + du;
          if (u < 0 || u >= static_cast<int>(width)) continue;
          image[static_cast<std::size_t>(v) * width + u] = bright;
        }
      }
    }
  }
  return image;
}

}  // namespace

TEST(HarrisCornerDetector, FlatImageProducesNoCorners) {
  HarrisCornerDetectorParams params;
  HarrisCornerDetector detector(params);
  std::vector<uint8_t> image(64 * 64, 100);
  EXPECT_TRUE(detector.Detect(image.data(), 64, 64, 64).empty());
}

TEST(HarrisCornerDetector, FindsAKnownCheckerboardCornerAndExtractsItsPatch) {
  HarrisCornerDetectorParams params;
  params.window_radius = 2;
  params.quality_level = 0.1;
  params.nms_radius = 5;
  params.max_corners = 10;
  params.patch_half_size = 3;
  HarrisCornerDetector detector(params);

  const auto image = MakeCheckerCorner(60, 60, /*cx=*/30, /*cy=*/30, /*dark=*/10, /*bright=*/220);
  const auto corners = detector.Detect(image.data(), 60, 60, 60);

  ASSERT_FALSE(corners.empty());
  // The strongest response (returned first, see the detector's descending
  // sort) should land within a couple pixels of the true corner — Sobel
  // smoothing over a hard binary edge keeps the peak from landing on the
  // exact pixel.
  EXPECT_NEAR(corners[0].centroid_u, 30.0, 2.0);
  EXPECT_NEAR(corners[0].centroid_v, 30.0, 2.0);

  const int patch_size = 2 * params.patch_half_size + 1;
  EXPECT_EQ(corners[0].patch.size(), static_cast<std::size_t>(patch_size * patch_size));
  EXPECT_EQ(corners[0].pixel_count, 1);  // point feature, see header comment
}

TEST(HarrisCornerDetector, NmsRadiusControlsHowManyNearbyResponsesSurvive) {
  // 9 isolated squares, 4 genuine corners each: nms_radius small enough to
  // stay within one square's corners (~7px apart) keeps all 4 per square;
  // large enough to span a whole square but not reach a neighbor (squares
  // are 20px apart) collapses each down to 1; large enough to span the
  // whole grid collapses everything to the single strongest corner.
  const auto image = MakeSquareGrid(70, 70, /*cols=*/3, /*rows=*/3, /*spacing=*/20, /*half_size=*/3,
                                     /*dark=*/10, /*bright=*/220);

  HarrisCornerDetectorParams loose;
  loose.quality_level = 0.1;
  loose.nms_radius = 2;
  loose.max_corners = 1000;
  const auto loose_corners = HarrisCornerDetector(loose).Detect(image.data(), 70, 70, 70);

  HarrisCornerDetectorParams strict = loose;
  strict.nms_radius = 8;
  const auto strict_corners = HarrisCornerDetector(strict).Detect(image.data(), 70, 70, 70);

  HarrisCornerDetectorParams strictest = loose;
  strictest.nms_radius = 30;
  const auto strictest_corners = HarrisCornerDetector(strictest).Detect(image.data(), 70, 70, 70);

  EXPECT_EQ(strictest_corners.size(), 1u);
  EXPECT_GT(strict_corners.size(), strictest_corners.size());
  EXPECT_GT(loose_corners.size(), strict_corners.size());
}

TEST(HarrisCornerDetector, MaxCornersCapsOutputToTheStrongestResponses) {
  HarrisCornerDetectorParams params;
  params.window_radius = 2;
  params.quality_level = 0.1;
  params.nms_radius = 3;
  params.patch_half_size = 2;
  params.max_corners = 1000;
  const auto image = MakeSquareGrid(70, 70, /*cols=*/3, /*rows=*/3, /*spacing=*/20, /*half_size=*/3,
                                     /*dark=*/10, /*bright=*/220);

  const auto uncapped = HarrisCornerDetector(params).Detect(image.data(), 70, 70, 70);
  ASSERT_GT(uncapped.size(), 4u);  // 9 squares, each with 4 corners: plenty to cap

  params.max_corners = 4;
  const auto capped = HarrisCornerDetector(params).Detect(image.data(), 70, 70, 70);
  EXPECT_EQ(capped.size(), 4u);
}
