#include "frontends/landmark_blob_detector.hpp"

#include <cmath>
#include <cstddef>
#include <deque>

namespace uw::frontends {

LandmarkBlobDetector::LandmarkBlobDetector(LandmarkBlobDetectorParams params) : params_(params) {}

std::vector<LandmarkBlob> LandmarkBlobDetector::Detect(const uint8_t* image, uint32_t width,
                                                         uint32_t height, uint32_t stride_px) const {
  std::vector<LandmarkBlob> blobs;
  if (width == 0 || height == 0) return blobs;

  const std::size_t num_pixels = static_cast<std::size_t>(width) * height;
  std::vector<uint8_t> visited(num_pixels, 0);
  const int patch_size = 2 * params_.patch_half_size + 1;

  auto at = [&](int u, int v) -> uint8_t { return image[static_cast<std::size_t>(v) * stride_px + u]; };

  for (uint32_t v0 = 0; v0 < height; ++v0) {
    for (uint32_t u0 = 0; u0 < width; ++u0) {
      const std::size_t idx0 = static_cast<std::size_t>(v0) * width + u0;
      if (visited[idx0] || at(u0, v0) <= params_.intensity_threshold) continue;

      // 4-connected flood fill, row-major queue order for determinism.
      std::deque<std::pair<int, int>> queue;
      queue.emplace_back(static_cast<int>(u0), static_cast<int>(v0));
      visited[idx0] = 1;
      long sum_u = 0, sum_v = 0;
      int count = 0;

      while (!queue.empty()) {
        const auto [u, v] = queue.front();
        queue.pop_front();
        sum_u += u;
        sum_v += v;
        ++count;

        const int neighbors[4][2] = {{u - 1, v}, {u + 1, v}, {u, v - 1}, {u, v + 1}};
        for (const auto& n : neighbors) {
          const int nu = n[0], nv = n[1];
          if (nu < 0 || nu >= static_cast<int>(width) || nv < 0 || nv >= static_cast<int>(height)) continue;
          const std::size_t nidx = static_cast<std::size_t>(nv) * width + nu;
          if (visited[nidx] || at(nu, nv) <= params_.intensity_threshold) continue;
          visited[nidx] = 1;
          queue.emplace_back(nu, nv);
        }
      }

      if (count < params_.min_blob_pixels || count > params_.max_blob_pixels) continue;

      LandmarkBlob blob;
      blob.pixel_count = count;
      blob.centroid_u = static_cast<double>(sum_u) / count;
      blob.centroid_v = static_cast<double>(sum_v) / count;

      const int center_u = static_cast<int>(std::lround(blob.centroid_u));
      const int center_v = static_cast<int>(std::lround(blob.centroid_v));
      blob.patch.resize(static_cast<std::size_t>(patch_size) * patch_size, 0);
      for (int dv = -params_.patch_half_size; dv <= params_.patch_half_size; ++dv) {
        const int pv = center_v + dv;
        if (pv < 0 || pv >= static_cast<int>(height)) continue;
        for (int du = -params_.patch_half_size; du <= params_.patch_half_size; ++du) {
          const int pu = center_u + du;
          if (pu < 0 || pu >= static_cast<int>(width)) continue;
          const std::size_t patch_idx =
              static_cast<std::size_t>(dv + params_.patch_half_size) * patch_size +
              static_cast<std::size_t>(du + params_.patch_half_size);
          blob.patch[patch_idx] = at(pu, pv);
        }
      }

      blobs.push_back(std::move(blob));
    }
  }

  return blobs;
}

}  // namespace uw::frontends
