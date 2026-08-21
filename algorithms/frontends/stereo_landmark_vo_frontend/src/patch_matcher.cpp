#include "uw/frontends/patch_matcher.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace uw::frontends {
namespace {

// Normalized cross-correlation over two equal-length patches. Returns 0 if
// either patch is perfectly flat (zero variance) unless both are flat and
// bit-identical, in which case they're a trivial perfect match.
double NormalizedCrossCorrelation(const std::vector<uint8_t>& patch_a, const std::vector<uint8_t>& patch_b) {
  if (patch_a.size() != patch_b.size() || patch_a.empty()) return -1.0;

  double sum_a = 0.0, sum_b = 0.0;
  for (std::size_t i = 0; i < patch_a.size(); ++i) {
    sum_a += patch_a[i];
    sum_b += patch_b[i];
  }
  const double mean_a = sum_a / static_cast<double>(patch_a.size());
  const double mean_b = sum_b / static_cast<double>(patch_b.size());

  double numerator = 0.0, var_a = 0.0, var_b = 0.0;
  for (std::size_t i = 0; i < patch_a.size(); ++i) {
    const double da = patch_a[i] - mean_a;
    const double db = patch_b[i] - mean_b;
    numerator += da * db;
    var_a += da * da;
    var_b += db * db;
  }

  if (var_a <= 0.0 || var_b <= 0.0) {
    return patch_a == patch_b ? 1.0 : 0.0;
  }
  return numerator / std::sqrt(var_a * var_b);
}

}  // namespace

PatchMatcher::PatchMatcher(PatchMatcherParams params) : params_(params) {}

std::vector<PatchMatch> PatchMatcher::Match(const std::vector<LandmarkBlob>& a,
                                             const std::vector<LandmarkBlob>& b) const {
  std::vector<PatchMatch> candidates;
  candidates.reserve(a.size() * b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    for (std::size_t j = 0; j < b.size(); ++j) {
      const double score = NormalizedCrossCorrelation(a[i].patch, b[j].patch);
      if (score >= params_.min_ncc_score) {
        candidates.push_back(PatchMatch{i, j, score});
      }
    }
  }

  // Stable sort by descending score, ties broken by (index_a, index_b) —
  // the initial candidate order is already (a-index, b-index) ascending,
  // so a stable sort preserves that as the tie-break.
  std::stable_sort(candidates.begin(), candidates.end(),
                    [](const PatchMatch& lhs, const PatchMatch& rhs) { return lhs.score > rhs.score; });

  std::vector<uint8_t> used_a(a.size(), 0), used_b(b.size(), 0);
  std::vector<PatchMatch> matches;
  for (const auto& candidate : candidates) {
    if (used_a[candidate.index_a] || used_b[candidate.index_b]) continue;
    used_a[candidate.index_a] = 1;
    used_b[candidate.index_b] = 1;
    matches.push_back(candidate);
  }

  return matches;
}

}  // namespace uw::frontends
