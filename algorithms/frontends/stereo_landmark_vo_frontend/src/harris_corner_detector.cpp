#include "uw/frontends/harris_corner_detector.hpp"

#include <algorithm>
#include <cstddef>

namespace uw::frontends {

namespace {
struct Candidate {
  int u = 0;
  int v = 0;
  double response = 0.0;
};
}  // namespace

HarrisCornerDetector::HarrisCornerDetector(HarrisCornerDetectorParams params) : params_(params) {}

std::vector<LandmarkBlob> HarrisCornerDetector::Detect(const uint8_t* image, uint32_t width, uint32_t height,
                                                         uint32_t stride_px) const {
  const int w = static_cast<int>(width);
  const int h = static_cast<int>(height);
  // One pixel of margin for the 3x3 Sobel kernel, plus window_radius for
  // the structure-tensor summation window around each candidate pixel.
  const int margin = 1 + params_.window_radius;
  if (w <= 2 * margin || h <= 2 * margin) return {};

  auto at = [&](int u, int v) -> double {
    return static_cast<double>(image[static_cast<std::size_t>(v) * stride_px + static_cast<std::size_t>(u)]);
  };

  std::vector<double> gx(static_cast<std::size_t>(w) * h, 0.0);
  std::vector<double> gy(static_cast<std::size_t>(w) * h, 0.0);
  for (int v = 1; v < h - 1; ++v) {
    for (int u = 1; u < w - 1; ++u) {
      gx[static_cast<std::size_t>(v) * w + u] = (at(u + 1, v - 1) + 2.0 * at(u + 1, v) + at(u + 1, v + 1)) -
                                                 (at(u - 1, v - 1) + 2.0 * at(u - 1, v) + at(u - 1, v + 1));
      gy[static_cast<std::size_t>(v) * w + u] = (at(u - 1, v + 1) + 2.0 * at(u, v + 1) + at(u + 1, v + 1)) -
                                                 (at(u - 1, v - 1) + 2.0 * at(u, v - 1) + at(u + 1, v - 1));
    }
  }

  std::vector<double> response(static_cast<std::size_t>(w) * h, 0.0);
  double max_response = 0.0;
  for (int v = margin; v < h - margin; ++v) {
    for (int u = margin; u < w - margin; ++u) {
      double sxx = 0.0, syy = 0.0, sxy = 0.0;
      for (int dv = -params_.window_radius; dv <= params_.window_radius; ++dv) {
        for (int du = -params_.window_radius; du <= params_.window_radius; ++du) {
          const double ix = gx[static_cast<std::size_t>(v + dv) * w + (u + du)];
          const double iy = gy[static_cast<std::size_t>(v + dv) * w + (u + du)];
          sxx += ix * ix;
          syy += iy * iy;
          sxy += ix * iy;
        }
      }
      const double det = sxx * syy - sxy * sxy;
      const double trace = sxx + syy;
      const double r = det - params_.k * trace * trace;
      response[static_cast<std::size_t>(v) * w + u] = r;
      max_response = std::max(max_response, r);
    }
  }
  if (max_response <= 0.0) return {};
  const double threshold = params_.quality_level * max_response;

  // A pixel survives only if its response clears the adaptive threshold
  // AND it is the (row-major-tie-broken) strongest response within its own
  // nms_radius neighborhood — standard non-maximum suppression, computed
  // directly off the response map rather than an O(n^2) pass over
  // already-thresholded candidates so a low quality_level on a noisy image
  // still stays bounded (O(width*height*nms_radius^2)).
  std::vector<Candidate> candidates;
  for (int v = margin; v < h - margin; ++v) {
    for (int u = margin; u < w - margin; ++u) {
      const double r = response[static_cast<std::size_t>(v) * w + u];
      if (r < threshold) continue;

      bool is_local_max = true;
      for (int dv = -params_.nms_radius; dv <= params_.nms_radius && is_local_max; ++dv) {
        const int nv = v + dv;
        if (nv < margin || nv >= h - margin) continue;
        for (int du = -params_.nms_radius; du <= params_.nms_radius; ++du) {
          if (du == 0 && dv == 0) continue;
          const int nu = u + du;
          if (nu < margin || nu >= w - margin) continue;
          const double neighbor = response[static_cast<std::size_t>(nv) * w + nu];
          if (neighbor > r || (neighbor == r && (nv < v || (nv == v && nu < u)))) {
            is_local_max = false;
            break;
          }
        }
      }
      if (is_local_max) candidates.push_back({u, v, r});
    }
  }

  std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
    if (a.response != b.response) return a.response > b.response;
    if (a.v != b.v) return a.v < b.v;
    return a.u < b.u;
  });
  if (static_cast<int>(candidates.size()) > params_.max_corners) {
    candidates.resize(static_cast<std::size_t>(params_.max_corners));
  }

  const int patch_size = 2 * params_.patch_half_size + 1;
  std::vector<LandmarkBlob> corners;
  corners.reserve(candidates.size());
  for (const auto& c : candidates) {
    LandmarkBlob blob;
    blob.centroid_u = c.u;
    blob.centroid_v = c.v;
    blob.pixel_count = 1;  // meaningless for a point feature; see header comment
    blob.patch.assign(static_cast<std::size_t>(patch_size) * patch_size, 0);
    for (int dv = -params_.patch_half_size; dv <= params_.patch_half_size; ++dv) {
      const int pv = c.v + dv;
      if (pv < 0 || pv >= h) continue;
      for (int du = -params_.patch_half_size; du <= params_.patch_half_size; ++du) {
        const int pu = c.u + du;
        if (pu < 0 || pu >= w) continue;
        const std::size_t patch_idx = static_cast<std::size_t>(dv + params_.patch_half_size) * patch_size +
                                       static_cast<std::size_t>(du + params_.patch_half_size);
        blob.patch[patch_idx] = static_cast<uint8_t>(at(pu, pv));
      }
    }
    corners.push_back(std::move(blob));
  }
  return corners;
}

}  // namespace uw::frontends
