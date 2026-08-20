#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>

namespace uw::evaluation {

struct DepthGrid {
  uint32_t width = 0;
  uint32_t height = 0;
  std::vector<float> depth_m;      // row-major width*height
  std::vector<uint8_t> valid_mask; // row-major width*height, 1 = valid
};

struct DepthMetricsResult {
  double rmse_m = 0.0;
  double mae_m = 0.0;
  // Fraction of ground-truth-valid pixels that `estimated` also marks
  // valid. NOT the design spec's sonar-covered/degraded-region split
  // (section 12.1) — that needs scenario masks and sonar association,
  // both introduced in later plans.
  double valid_coverage_fraction = 0.0;
  std::size_t num_compared_pixels = 0;
};

// Compares only pixels valid in BOTH grids. v1 limitation, documented
// rather than hidden (same pattern as ComputeAte's v1 note): no
// full-image/sonar-covered/visually-degraded split yet.
DepthMetricsResult ComputeDepthMetrics(const DepthGrid& estimated, const DepthGrid& ground_truth);

}  // namespace uw::evaluation
