#include <array>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "frontends/block_matcher.hpp"

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
  params.min_disparity = 1;  // positive disparity only (0 => infinite depth, excluded by construction)
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

TEST(BlockMatcher, RejectsCompletelyFlatTexture) {
  constexpr int kWidth = 14;
  constexpr int kHeight = 3;
  std::vector<uint8_t> left(kWidth * kHeight, 100);
  std::vector<uint8_t> right(kWidth * kHeight, 100);

  uw::frontends::BlockMatcherParams params;
  params.window_radius = 1;
  params.min_disparity = 1;
  params.max_disparity = 3;
  uw::frontends::BlockMatcher matcher(params);

  const auto result = matcher.Compute(left.data(), right.data(), kWidth, kHeight, kWidth);
  for (uint8_t v : result.valid) EXPECT_EQ(v, 0);
}

TEST(BlockMatcher, RejectsRepeatedPeriodicTextureOnUniquenessMargin) {
  // Period-2 pattern: every odd disparity is an exact (SAD=0) match, so
  // best and second-best tie -- ambiguous despite high variance and a
  // perfect SAD score.
  constexpr int kWidth = 14;
  constexpr int kHeight = 3;
  std::vector<uint8_t> left(kWidth * kHeight);
  std::vector<uint8_t> right(kWidth * kHeight);
  for (int v = 0; v < kHeight; ++v) {
    for (int u = 0; u < kWidth; ++u) {
      left[v * kWidth + u] = (u % 2 == 0) ? 32 : 130;
      right[v * kWidth + u] = (u % 2 == 0) ? 130 : 32;  // left shifted by 1
    }
  }

  uw::frontends::BlockMatcherParams params;
  params.window_radius = 1;
  params.min_disparity = 1;
  params.max_disparity = 3;
  uw::frontends::BlockMatcher matcher(params);

  const auto result = matcher.Compute(left.data(), right.data(), kWidth, kHeight, kWidth);
  const int v = 1;
  const int u_test = 7;
  EXPECT_EQ(result.valid[static_cast<std::size_t>(v) * kWidth + u_test], 0);
}

TEST(BlockMatcher, RejectsLeftRightInconsistentMatch) {
  // Hand-picked (search-verified) 1D profiles, replicated across all rows:
  // the left->right search at u=7 finds a clean, well-textured,
  // unambiguous match at d=1 (mean SAD well under the default cap, margin
  // well over it) -- but the INDEPENDENT right->left search from that same
  // right-image position disagrees (d=3), the signature of an occlusion/
  // wrong-match that only looks good from one direction.
  constexpr int kWidth = 14;
  constexpr int kHeight = 3;
  constexpr std::array<uint8_t, kWidth> kLeftRow = {91, 254, 94, 214, 132, 115, 202,
                                                    57, 190, 135, 242, 146, 238, 61};
  constexpr std::array<uint8_t, kWidth> kRightRow = {33, 204, 105, 129, 229, 232, 138,
                                                     195, 35, 100, 152, 213, 29, 205};
  std::vector<uint8_t> left(kWidth * kHeight);
  std::vector<uint8_t> right(kWidth * kHeight);
  for (int v = 0; v < kHeight; ++v) {
    for (int u = 0; u < kWidth; ++u) {
      left[v * kWidth + u] = kLeftRow[u];
      right[v * kWidth + u] = kRightRow[u];
    }
  }

  uw::frontends::BlockMatcherParams params;
  params.window_radius = 1;
  params.min_disparity = 1;
  params.max_disparity = 3;
  params.left_right_max_diff_px = 1.0;
  uw::frontends::BlockMatcher matcher(params);

  const auto result = matcher.Compute(left.data(), right.data(), kWidth, kHeight, kWidth);
  const int v = 1;
  const int u_test = 7;
  EXPECT_EQ(result.valid[static_cast<std::size_t>(v) * kWidth + u_test], 0);
}

TEST(BlockMatcher, RecoversDisparityAtTheMinDisparityBoundary) {
  constexpr int kWidth = 20;
  constexpr int kHeight = 3;
  constexpr int kTrueDisparity = 1;  // == min_disparity itself

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
  params.min_disparity = 1;
  params.max_disparity = 6;
  uw::frontends::BlockMatcher matcher(params);

  const auto result = matcher.Compute(left.data(), right.data(), kWidth, kHeight, kWidth);
  const int v = 1;
  const int u = 1 + 6;  // inside the valid region regardless of true disparity
  const std::size_t idx = static_cast<std::size_t>(v) * kWidth + u;
  ASSERT_EQ(result.valid[idx], 1);
  EXPECT_FLOAT_EQ(result.disparity_px[idx], static_cast<float>(kTrueDisparity));
}

TEST(BlockMatcher, RecoversDisparityAtTheMaxDisparityBoundary) {
  constexpr int kWidth = 20;
  constexpr int kHeight = 3;
  constexpr int kTrueDisparity = 6;  // == max_disparity itself

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
  params.min_disparity = 1;
  params.max_disparity = 6;
  uw::frontends::BlockMatcher matcher(params);

  const auto result = matcher.Compute(left.data(), right.data(), kWidth, kHeight, kWidth);
  const int v = 1;
  const int u = 1 + 6;
  const std::size_t idx = static_cast<std::size_t>(v) * kWidth + u;
  ASSERT_EQ(result.valid[idx], 1);
  EXPECT_FLOAT_EQ(result.disparity_px[idx], static_cast<float>(kTrueDisparity));
}
