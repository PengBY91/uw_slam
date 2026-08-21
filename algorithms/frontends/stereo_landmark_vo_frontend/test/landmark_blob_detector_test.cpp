#include "uw/frontends/landmark_blob_detector.hpp"

#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

using uw::frontends::LandmarkBlob;
using uw::frontends::LandmarkBlobDetector;
using uw::frontends::LandmarkBlobDetectorParams;

namespace {

// A width*height MONO8 image filled with `background`, with a
// (2*half+1)x(2*half+1) square of `foreground` centered at (cu, cv).
std::vector<uint8_t> MakeImageWithSquare(uint32_t width, uint32_t height, int cu, int cv, int half,
                                          uint8_t background, uint8_t foreground) {
  std::vector<uint8_t> image(static_cast<std::size_t>(width) * height, background);
  for (int dv = -half; dv <= half; ++dv) {
    const int v = cv + dv;
    if (v < 0 || v >= static_cast<int>(height)) continue;
    for (int du = -half; du <= half; ++du) {
      const int u = cu + du;
      if (u < 0 || u >= static_cast<int>(width)) continue;
      image[static_cast<std::size_t>(v) * width + u] = foreground;
    }
  }
  return image;
}

}  // namespace

TEST(LandmarkBlobDetector, FindsCentroidOfAnIsolatedBrightSquare) {
  LandmarkBlobDetectorParams params;
  params.intensity_threshold = 100;
  params.min_blob_pixels = 4;
  params.max_blob_pixels = 400;
  params.patch_half_size = 3;
  LandmarkBlobDetector detector(params);

  const auto image = MakeImageWithSquare(64, 64, 20, 30, 4, /*background=*/10, /*foreground=*/200);
  const auto blobs = detector.Detect(image.data(), 64, 64, 64);

  ASSERT_EQ(blobs.size(), 1u);
  EXPECT_NEAR(blobs[0].centroid_u, 20.0, 1e-9);
  EXPECT_NEAR(blobs[0].centroid_v, 30.0, 1e-9);
  EXPECT_EQ(blobs[0].pixel_count, 81);  // (2*4+1)^2
}

TEST(LandmarkBlobDetector, ExtractsAppearancePatchFromRawImageAroundCentroid) {
  LandmarkBlobDetectorParams params;
  params.intensity_threshold = 100;
  params.min_blob_pixels = 1;
  params.max_blob_pixels = 400;
  params.patch_half_size = 1;  // 3x3 patch
  LandmarkBlobDetector detector(params);

  // A single bright pixel; the surrounding background pixels (below
  // threshold) should still show up in the extracted patch, since the
  // patch samples the raw image, not the threshold mask.
  std::vector<uint8_t> image(10 * 10, 5);
  image[5 * 10 + 5] = 250;

  const auto blobs = detector.Detect(image.data(), 10, 10, 10);
  ASSERT_EQ(blobs.size(), 1u);
  ASSERT_EQ(blobs[0].patch.size(), 9u);
  // Center of the 3x3 patch is the bright pixel itself.
  EXPECT_EQ(blobs[0].patch[4], 250);
  // Corners of the patch are background.
  EXPECT_EQ(blobs[0].patch[0], 5);
  EXPECT_EQ(blobs[0].patch[8], 5);
}

TEST(LandmarkBlobDetector, RejectsBlobsOutsideSizeBounds) {
  LandmarkBlobDetectorParams params;
  params.intensity_threshold = 100;
  params.min_blob_pixels = 5;
  params.max_blob_pixels = 20;
  params.patch_half_size = 2;
  LandmarkBlobDetector detector(params);

  // A single bright pixel: too small (below min_blob_pixels).
  std::vector<uint8_t> noise_image(20 * 20, 5);
  noise_image[10 * 20 + 10] = 200;
  EXPECT_TRUE(detector.Detect(noise_image.data(), 20, 20, 20).empty());

  // A large bright square: too big (above max_blob_pixels).
  const auto huge_image = MakeImageWithSquare(40, 40, 20, 20, 10, /*background=*/5, /*foreground=*/200);
  EXPECT_TRUE(detector.Detect(huge_image.data(), 40, 40, 40).empty());
}

TEST(LandmarkBlobDetector, SeparatesTwoDistinctBlobs) {
  LandmarkBlobDetectorParams params;
  params.intensity_threshold = 100;
  params.min_blob_pixels = 1;
  params.max_blob_pixels = 100;
  params.patch_half_size = 1;
  LandmarkBlobDetector detector(params);

  std::vector<uint8_t> image(50 * 50, 5);
  image[10 * 50 + 10] = 200;
  image[40 * 50 + 40] = 210;

  const auto blobs = detector.Detect(image.data(), 50, 50, 50);
  ASSERT_EQ(blobs.size(), 2u);
  EXPECT_NEAR(blobs[0].centroid_u, 10.0, 1e-9);
  EXPECT_NEAR(blobs[0].centroid_v, 10.0, 1e-9);
  EXPECT_NEAR(blobs[1].centroid_u, 40.0, 1e-9);
  EXPECT_NEAR(blobs[1].centroid_v, 40.0, 1e-9);
}

TEST(LandmarkBlobDetector, EmptyImageProducesNoBlobs) {
  LandmarkBlobDetectorParams params;
  LandmarkBlobDetector detector(params);
  std::vector<uint8_t> image(32 * 32, 0);
  EXPECT_TRUE(detector.Detect(image.data(), 32, 32, 32).empty());
}
