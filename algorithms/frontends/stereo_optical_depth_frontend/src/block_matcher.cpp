#include "uw/frontends/block_matcher.hpp"

#include <cstdlib>
#include <limits>

namespace uw::frontends {

BlockMatcher::BlockMatcher(BlockMatcherParams params) : params_(params) {}

DisparityResult BlockMatcher::Compute(const uint8_t* left, const uint8_t* right, uint32_t width,
                                      uint32_t height, uint32_t stride_px) const {
  DisparityResult result;
  result.width = width;
  result.height = height;
  result.disparity_px.assign(static_cast<std::size_t>(width) * height, 0.0f);
  result.valid.assign(static_cast<std::size_t>(width) * height, 0);

  const int r = params_.window_radius;
  const int window_pixels = (2 * r + 1) * (2 * r + 1);
  const int u_min = r + params_.max_disparity;
  const int u_max = static_cast<int>(width) - 1 - r;
  const int v_min = r;
  const int v_max = static_cast<int>(height) - 1 - r;

  auto at = [&](const uint8_t* image, int u, int v) -> int {
    return image[static_cast<std::size_t>(v) * stride_px + static_cast<std::size_t>(u)];
  };

  for (int v = v_min; v <= v_max; ++v) {
    for (int u = u_min; u <= u_max; ++u) {
      int best_disparity = -1;
      long best_sad = std::numeric_limits<long>::max();
      for (int d = params_.min_disparity; d <= params_.max_disparity; ++d) {
        long sad = 0;
        for (int dy = -r; dy <= r; ++dy) {
          for (int dx = -r; dx <= r; ++dx) {
            sad += std::abs(at(left, u + dx, v + dy) - at(right, u + dx - d, v + dy));
          }
        }
        if (sad < best_sad) {
          best_sad = sad;
          best_disparity = d;
        }
      }
      const double mean_abs_diff = static_cast<double>(best_sad) / window_pixels;
      const std::size_t idx = static_cast<std::size_t>(v) * width + static_cast<std::size_t>(u);
      if (best_disparity >= 0 && mean_abs_diff <= params_.max_mean_abs_diff) {
        result.disparity_px[idx] = static_cast<float>(best_disparity);
        result.valid[idx] = 1;
      }
    }
  }
  return result;
}

}  // namespace uw::frontends
