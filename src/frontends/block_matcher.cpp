#include "frontends/block_matcher.hpp"

#include <cmath>
#include <cstdlib>
#include <limits>

namespace uw::frontends {

namespace {

int At(const uint8_t* image, uint32_t stride_px, int u, int v) {
  return image[static_cast<std::size_t>(v) * stride_px + static_cast<std::size_t>(u)];
}

}  // namespace

BlockMatcher::BlockMatcher(BlockMatcherParams params) : params_(params) {}

BlockMatcher::DirectionalMatch BlockMatcher::MatchOneDirection(const uint8_t* reference,
                                                                const uint8_t* target, int search_sign,
                                                                int u, int v, uint32_t width,
                                                                uint32_t stride_px) const {
  DirectionalMatch result;
  const int r = params_.window_radius;
  const int window_pixels = (2 * r + 1) * (2 * r + 1);

  if (u - r < 0 || u + r > static_cast<int>(width) - 1) return result;  // reference block out of bounds

  double sum = 0.0;
  double sum_sq = 0.0;
  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      const double value = At(reference, stride_px, u + dx, v + dy);
      sum += value;
      sum_sq += value * value;
    }
  }
  const double mean = sum / window_pixels;
  result.variance = sum_sq / window_pixels - mean * mean;

  long best_sad = std::numeric_limits<long>::max();
  long second_best_sad = std::numeric_limits<long>::max();
  for (int d = params_.min_disparity; d <= params_.max_disparity; ++d) {
    const int target_u = u + search_sign * d;
    if (target_u - r < 0 || target_u + r > static_cast<int>(width) - 1) continue;

    long sad = 0;
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        sad += std::abs(At(reference, stride_px, u + dx, v + dy) -
                        At(target, stride_px, target_u + dx, v + dy));
      }
    }
    if (sad < best_sad) {
      second_best_sad = best_sad;
      result.has_second_best = second_best_sad != std::numeric_limits<long>::max();
      best_sad = sad;
      result.best_disparity = d;
    } else if (sad < second_best_sad) {
      second_best_sad = sad;
      result.has_second_best = true;
    }
  }

  result.valid_search = result.best_disparity != -1;
  if (result.valid_search) {
    result.best_mean_sad = static_cast<double>(best_sad) / window_pixels;
    if (result.has_second_best) {
      result.second_best_mean_sad = static_cast<double>(second_best_sad) / window_pixels;
    }
  }
  return result;
}

DisparityResult BlockMatcher::Compute(const uint8_t* left, const uint8_t* right, uint32_t width,
                                      uint32_t height, uint32_t stride_px) const {
  DisparityResult result;
  result.width = width;
  result.height = height;
  result.disparity_px.assign(static_cast<std::size_t>(width) * height, 0.0f);
  result.valid.assign(static_cast<std::size_t>(width) * height, 0);

  const int r = params_.window_radius;
  const int u_min = r + params_.max_disparity;
  const int u_max = static_cast<int>(width) - 1 - r;
  const int v_min = r;
  const int v_max = static_cast<int>(height) - 1 - r;

  for (int v = v_min; v <= v_max; ++v) {
    for (int u = u_min; u <= u_max; ++u) {
      // Filter order is fixed (see BlockMatcherParams' own field comments
      // for what each one catches): texture -> search range/positive
      // disparity -> max mean SAD -> uniqueness margin -> LR consistency.
      const auto lr = MatchOneDirection(left, right, -1, u, v, width, stride_px);
      if (lr.variance < params_.min_texture_variance) continue;
      if (!lr.valid_search) continue;
      if (lr.best_mean_sad > params_.max_mean_abs_diff) continue;
      const double margin = lr.has_second_best
                                ? (lr.second_best_mean_sad - lr.best_mean_sad)
                                : std::numeric_limits<double>::infinity();  // no competing candidate
      if (margin < params_.min_uniqueness_margin) continue;

      const int u_right = u - lr.best_disparity;
      const auto rl = MatchOneDirection(right, left, 1, u_right, v, width, stride_px);
      if (!rl.valid_search) continue;
      if (std::abs(rl.best_disparity - lr.best_disparity) > params_.left_right_max_diff_px) continue;

      const std::size_t idx = static_cast<std::size_t>(v) * width + static_cast<std::size_t>(u);
      result.disparity_px[idx] = static_cast<float>(lr.best_disparity);
      result.valid[idx] = 1;
    }
  }
  return result;
}

}  // namespace uw::frontends
