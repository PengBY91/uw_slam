#include "uw/frontends/patch_matcher.hpp"

#include <utility>
#include <vector>

#include <gtest/gtest.h>

using uw::frontends::LandmarkBlob;
using uw::frontends::PatchMatcher;
using uw::frontends::PatchMatcherParams;

namespace {

LandmarkBlob MakeBlob(double u, double v, std::vector<uint8_t> patch) {
  LandmarkBlob blob;
  blob.centroid_u = u;
  blob.centroid_v = v;
  blob.patch = std::move(patch);
  return blob;
}

}  // namespace

TEST(PatchMatcher, MatchesByAppearanceRegardlessOfPosition) {
  // Two distinctive patches; blob positions are deliberately scrambled
  // relative to which pattern they carry, so a position-based matcher
  // would get this wrong but an appearance-based one gets it right.
  const std::vector<uint8_t> pattern_a = {10, 200, 10, 200, 10, 200, 10, 200, 10};
  const std::vector<uint8_t> pattern_b = {5, 5, 250, 5, 5, 250, 5, 5, 250};

  std::vector<LandmarkBlob> left = {MakeBlob(100.0, 100.0, pattern_a), MakeBlob(5.0, 5.0, pattern_b)};
  std::vector<LandmarkBlob> right = {MakeBlob(5.0, 5.0, pattern_b), MakeBlob(100.0, 100.0, pattern_a)};

  PatchMatcherParams params;
  params.min_ncc_score = 0.9;
  PatchMatcher matcher(params);

  const auto matches = matcher.Match(left, right);
  ASSERT_EQ(matches.size(), 2u);

  for (const auto& match : matches) {
    if (match.index_a == 0) {
      EXPECT_EQ(match.index_b, 1u);  // left[0] (pattern_a) matches right[1] (pattern_a)
    } else {
      EXPECT_EQ(match.index_a, 1u);
      EXPECT_EQ(match.index_b, 0u);
    }
    EXPECT_GT(match.score, 0.9);
  }
}

TEST(PatchMatcher, RejectsDissimilarPatches) {
  const std::vector<uint8_t> pattern_a = {10, 200, 10, 200, 10, 200, 10, 200, 10};
  const std::vector<uint8_t> pattern_flat = {50, 50, 50, 50, 50, 50, 50, 50, 50};

  std::vector<LandmarkBlob> left = {MakeBlob(0, 0, pattern_a)};
  std::vector<LandmarkBlob> right = {MakeBlob(0, 0, pattern_flat)};

  PatchMatcherParams params;
  params.min_ncc_score = 0.6;
  PatchMatcher matcher(params);

  EXPECT_TRUE(matcher.Match(left, right).empty());
}

TEST(PatchMatcher, GreedyAssignmentIsOneToOne) {
  // Three left blobs, all carrying the SAME pattern (maximally ambiguous)
  // against three right blobs with the same pattern — every candidate
  // pair scores a perfect match, so the result must still be a valid
  // one-to-one assignment (no blob matched twice).
  const std::vector<uint8_t> pattern = {10, 200, 10, 200, 10, 200, 10, 200, 10};

  std::vector<LandmarkBlob> left = {MakeBlob(0, 0, pattern), MakeBlob(1, 1, pattern), MakeBlob(2, 2, pattern)};
  std::vector<LandmarkBlob> right = {MakeBlob(0, 0, pattern), MakeBlob(1, 1, pattern), MakeBlob(2, 2, pattern)};

  PatchMatcherParams params;
  params.min_ncc_score = 0.9;
  PatchMatcher matcher(params);

  const auto matches = matcher.Match(left, right);
  ASSERT_EQ(matches.size(), 3u);

  std::vector<bool> used_a(3, false), used_b(3, false);
  for (const auto& match : matches) {
    EXPECT_FALSE(used_a[match.index_a]);
    EXPECT_FALSE(used_b[match.index_b]);
    used_a[match.index_a] = true;
    used_b[match.index_b] = true;
  }
}

TEST(PatchMatcher, EmptyInputsProduceNoMatches) {
  PatchMatcherParams params;
  PatchMatcher matcher(params);
  EXPECT_TRUE(matcher.Match({}, {}).empty());

  std::vector<LandmarkBlob> one = {MakeBlob(0, 0, {1, 2, 3})};
  EXPECT_TRUE(matcher.Match(one, {}).empty());
  EXPECT_TRUE(matcher.Match({}, one).empty());
}
