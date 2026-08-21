#pragma once

#include <cstdint>
#include <vector>

namespace uw::frontends {

struct LandmarkBlobDetectorParams {
  uint8_t intensity_threshold = 140;  // a pixel above this counts as a candidate landmark pixel
  int min_blob_pixels = 4;            // rejects single-pixel noise
  int max_blob_pixels = 400;          // rejects large saturated regions (not a plausible point landmark)
  int patch_half_size = 6;            // appearance patch is (2r+1)x(2r+1), sampled around the centroid
};

struct LandmarkBlob {
  double centroid_u = 0.0;
  double centroid_v = 0.0;
  int pixel_count = 0;
  std::vector<uint8_t> patch;  // (2*patch_half_size+1)^2 row-major; out-of-bounds samples are 0
};

// Fixed-threshold blob detection over a single MONO8 image: pixels above
// `intensity_threshold` are grouped by 4-connected flood fill, then
// reduced to a centroid + a fixed-size appearance patch (sampled from the
// raw, non-thresholded image) per component. Deterministic, single-
// threaded, row-major scan order, so output ordering never depends on
// hash/map iteration.
//
// Original implementation, not a port — same precedent as
// sonar_cfar_frontend's dbscan.hpp (see NOTICE). Plain connected-
// components is enough here (unlike DBSCAN, which sonar_cfar_frontend
// needs for CFAR's noisy multi-pixel detections): the landmark patches
// this is tuned for (apps/tools/synth_bag_gen's BuildVisualLandmarks, or a
// real camera's bright point features) are small solid blobs, not a noisy
// point cloud.
class LandmarkBlobDetector {
 public:
  explicit LandmarkBlobDetector(LandmarkBlobDetectorParams params);

  std::vector<LandmarkBlob> Detect(const uint8_t* image, uint32_t width, uint32_t height,
                                    uint32_t stride_px) const;

 private:
  LandmarkBlobDetectorParams params_;
};

}  // namespace uw::frontends
