// Original implementation, not ported from any third party (same
// precedent as sonar_cfar_frontend's dbscan.hpp — see NOTICE) — this repo
// has no OpenCV/vendor image dependency and this MVP does not need one.
#pragma once

#include <cstdint>
#include <vector>

namespace uw::frontends {

struct BlockMatcherParams {
  int window_radius = 3;         // block is (2r+1)x(2r+1)
  int min_disparity = 1;         // >=1: disparity 0 implies infinite depth, excluded by construction
  int max_disparity = 32;
  double max_mean_abs_diff = 40.0;  // reject a match if best SAD / window_pixel_count exceeds this
};

struct DisparityResult {
  uint32_t width = 0;
  uint32_t height = 0;
  std::vector<float> disparity_px;  // row-major width*height; meaningless where valid[i] == 0
  std::vector<uint8_t> valid;       // 1 = confident match found inside the search range
};

// Fixed-window sum-of-absolute-differences stereo matcher over rectified
// MONO8 pairs. Deterministic, single-threaded, fixed iteration order.
// Sign convention: for a pixel (u, v) in `left`, the matcher searches
// `right` at (u - d, v) for d in [min_disparity, max_disparity] — i.e. it
// assumes right(u, v) == left(u + d_true, v) for the true disparity
// d_true at that point (see stereo_optical_depth_frontend_test.cpp and
// apps/synth_stereo_gen.cpp for the synthetic data generated under this
// exact convention).
class BlockMatcher {
 public:
  explicit BlockMatcher(BlockMatcherParams params);

  DisparityResult Compute(const uint8_t* left, const uint8_t* right, uint32_t width,
                          uint32_t height, uint32_t stride_px) const;

 private:
  BlockMatcherParams params_;
};

}  // namespace uw::frontends
