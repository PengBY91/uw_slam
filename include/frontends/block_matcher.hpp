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
  // Reject a block whose LEFT-image intensity variance falls below this --
  // a flat/textureless block cannot support a real match regardless of how
  // low its SAD happens to score against a candidate disparity.
  double min_texture_variance = 25.0;
  // Reject a match whose second-best mean SAD is not at least this much
  // worse than its best mean SAD (mean SAD units, i.e. already divided by
  // window pixel count) -- repeated/periodic texture ties multiple
  // disparities together with no real winner.
  double min_uniqueness_margin = 2.0;
  // Reject a match whose independently-computed right-to-left disparity
  // disagrees with the left-to-right disparity by more than this many
  // pixels -- catches occlusion boundaries and wrong matches that still
  // happened to score well in one direction alone.
  double left_right_max_diff_px = 1.0;
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
  // One block-matching pass in ONE direction: correlates the (2r+1)x(2r+1)
  // block centered at (u,v) in `reference` against `target` at
  // (u + search_sign*d, v) for d in [min_disparity, max_disparity],
  // returning the reference block's own variance alongside the best/
  // second-best mean SAD and the winning disparity. Used twice per output
  // pixel -- once left->right (search_sign=-1, the primary match) and once
  // right->left (search_sign=+1, the LR-consistency check) -- without ever
  // recursing into Compute() itself (see block_matcher.cpp for why: that
  // would redo the full image's work and make border semantics ambiguous).
  struct DirectionalMatch {
    bool valid_search = false;  // at least one candidate d stayed within `target`'s bounds
    double variance = 0.0;
    int best_disparity = -1;
    double best_mean_sad = 0.0;
    double second_best_mean_sad = 0.0;  // meaningless if only one candidate d was in range
    bool has_second_best = false;
  };
  DirectionalMatch MatchOneDirection(const uint8_t* reference, const uint8_t* target, int search_sign,
                                     int u, int v, uint32_t width, uint32_t stride_px) const;

  BlockMatcherParams params_;
};

}  // namespace uw::frontends
