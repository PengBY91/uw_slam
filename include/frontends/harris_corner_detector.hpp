#pragma once

#include <cstdint>
#include <vector>

#include "frontends/landmark_blob_detector.hpp"

namespace uw::frontends {

struct HarrisCornerDetectorParams {
  double k = 0.04;              // Harris free parameter, standard range 0.04-0.06
  int window_radius = 2;        // structure-tensor summation window is (2r+1)x(2r+1)
  double quality_level = 0.01;  // a corner must score >= quality_level * (the strongest response in
                                 // this image) — relative, not an absolute magnitude, since raw Harris
                                 // response is in gradient^4 units and has no fixed meaningful scale
                                 // across different exposures/content (unlike LandmarkBlobDetector's
                                 // fixed intensity_threshold, which is tuned for one known synthetic
                                 // brightness range)
  int nms_radius = 5;           // a candidate is kept only if it's the strongest response within this
                                 // Chebyshev radius — caps how densely corners cluster
  int max_corners = 60;         // strongest-response-first cap on output count, to bound downstream
                                 // O(n*m) patch-matching cost
  int patch_half_size = 6;      // appearance patch is (2r+1)x(2r+1); matches
                                 // LandmarkBlobDetectorParams' default so both detectors are drop-in
                                 // compatible with PatchMatcher
};

// Harris corner detection over a single MONO8 image: Sobel gradients ->
// windowed structure tensor -> R = det(M) - k*trace(M)^2 -> a pixel is a
// corner if R clears the adaptive quality_level threshold AND is the
// strongest response within its own nms_radius neighborhood (non-maximum
// suppression) -> capped to the max_corners strongest survivors -> each
// reduced to a centroid + fixed-size appearance patch, same shape as
// LandmarkBlobDetector's output (LandmarkBlob's pixel_count is meaningless
// for a point feature and always set to 1 — kept only so both detectors
// share one output type PatchMatcher/StereoLandmarkVoFrontend already
// consume). Deterministic, single-threaded, row-major scan order.
//
// Unlike LandmarkBlobDetector (tuned for apps/synth_bag_gen.cpp's bright
// hash-patterned synthetic squares — see that class's own header comment),
// this is the detector meant for real camera imagery, where there is no
// reason to expect isolated bright blobs. Original implementation, not a
// port — same precedent as landmark_blob_detector.hpp/sonar_cfar_frontend's
// dbscan.hpp (see NOTICE).
class HarrisCornerDetector {
 public:
  explicit HarrisCornerDetector(HarrisCornerDetectorParams params);

  std::vector<LandmarkBlob> Detect(const uint8_t* image, uint32_t width, uint32_t height,
                                    uint32_t stride_px) const;

 private:
  HarrisCornerDetectorParams params_;
};

}  // namespace uw::frontends
