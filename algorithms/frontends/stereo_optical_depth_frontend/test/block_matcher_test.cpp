#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "uw/frontends/block_matcher.hpp"

namespace {

// Deterministic, non-repeating-enough texture for exact block matching —
// not meant to resemble a real image, only to give each block a unique
// intensity fingerprint so SAD has a clean global minimum at the true
// disparity.
uint8_t Texture(int u, int v) { return static_cast<uint8_t>((u * 31 + v * 17 + 7) % 256); }

}  // namespace

TEST(BlockMatcher, RecoversConstantDisparityOnCleanTexture) {
  constexpr int kWidth = 20;
  constexpr int kHeight = 3;
  constexpr int kTrueDisparity = 4;

  std::vector<uint8_t> left(kWidth * kHeight);
  std::vector<uint8_t> right(kWidth * kHeight);
  for (int v = 0; v < kHeight; ++v) {
    for (int u = 0; u < kWidth; ++u) {
      left[v * kWidth + u] = Texture(u, v);
      right[v * kWidth + u] = Texture(u + kTrueDisparity, v);
    }
  }

  uw::frontends::BlockMatcherParams params;
  params.window_radius = 1;
  params.min_disparity = 0;
  params.max_disparity = 6;
  uw::frontends::BlockMatcher matcher(params);

  const auto result = matcher.Compute(left.data(), right.data(), kWidth, kHeight, kWidth);
  ASSERT_EQ(result.width, static_cast<uint32_t>(kWidth));
  ASSERT_EQ(result.height, static_cast<uint32_t>(kHeight));

  // Valid region: u in [radius+max_disparity, width-1-radius], v in [radius, height-1-radius].
  const int v = 1;
  for (int u = 1 + 6; u <= kWidth - 1 - 1; ++u) {
    const std::size_t idx = static_cast<std::size_t>(v) * kWidth + u;
    ASSERT_EQ(result.valid[idx], 1) << "u=" << u;
    EXPECT_FLOAT_EQ(result.disparity_px[idx], static_cast<float>(kTrueDisparity)) << "u=" << u;
  }
  // Outside the valid region (borders, and the excluded top/bottom rows).
  EXPECT_EQ(result.valid[0 * kWidth + 0], 0);
  EXPECT_EQ(result.valid[2 * kWidth + 10], 0);
}
