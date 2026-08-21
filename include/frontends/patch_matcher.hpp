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
};

// Greedy best-match assignment between two sets of LandmarkBlob patches via
// normalized cross-correlation (NCC) on their fixed-size appearance
// patches — genuine appearance-based correspondence (position plays no
// role), the same principle a real feature matcher uses, just without a
// learned/vendor descriptor. Used both for stereo (left<->right) and
// temporal (previous-left<->current-left) matching by
// StereoLandmarkVoFrontend — same algorithm either way, only the input
// blob sets differ.
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
