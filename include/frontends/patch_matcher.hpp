#pragma once

#include <cstddef>
#include <vector>

#include "frontends/landmark_blob_detector.hpp"

namespace uw::frontends {

struct PatchMatch {
  std::size_t index_a = 0;
  std::size_t index_b = 0;
  double score = 0.0;  // normalized cross-correlation in [-1, 1], higher is better
};

struct PatchMatcherParams {
  double min_ncc_score = 0.6;  // a candidate pair below this score is never matched

  // Restricts candidate pairs to |a.centroid_v - b.centroid_v| <=
  // max_row_diff_px — an epipolar/row constraint for a parallel stereo
  // pair, mirroring BlockMatcher's own same-row disparity search. Negative
  // (default) disables the constraint entirely: needed for temporal
  // matching, where the "a" side (StereoLandmarkVoFrontend's
  // previous_as_blobs) only carries `.patch`, not a meaningful position.
  double max_row_diff_px = -1.0;

  // Rejects a candidate match unless it is clearly the best explanation
  // for BOTH its "a" and "b" side: requires (best score - second-best
  // score) >= min_score_margin, checked independently from each side. 0
  // (default) disables the check, preserving prior behavior exactly.
  // Real, texture-repetitive imagery (unlike synth_bag_gen's deliberately
  // unique per-landmark patterns) can produce several near-equally-good
  // NCC candidates for the same patch; RANSAC alone does not help once a
  // wrong match is just as "confident" as the right one, so this rejects
  // the ambiguous case outright rather than picking one arbitrarily.
  double min_score_margin = 0.0;
};

// Greedy best-match assignment between two sets of LandmarkBlob patches via
// normalized cross-correlation (NCC) on their fixed-size appearance
// patches — genuine appearance-based correspondence (position plays no
// role beyond the optional max_row_diff_px gate above), the same
// principle a real feature matcher uses, just without a learned/vendor
// descriptor. Used both for stereo (left<->right) and temporal
// (previous-left<->current-left) matching by StereoLandmarkVoFrontend —
// same algorithm either way, only the input blob sets and params differ.
//
// Deterministic: all candidate pairs are scored once in fixed (a-index,
// b-index) order, then the match set is built by repeatedly taking the
// single highest-scoring remaining pair (ties broken by (a-index,
// b-index) order) until no pair scores >= min_ncc_score or one side is
// exhausted — never depends on hash/map iteration order.
class PatchMatcher {
 public:
  explicit PatchMatcher(PatchMatcherParams params);

  std::vector<PatchMatch> Match(const std::vector<LandmarkBlob>& a,
                                 const std::vector<LandmarkBlob>& b) const;

 private:
  PatchMatcherParams params_;
};

}  // namespace uw::frontends
