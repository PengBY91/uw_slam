#include "sensor_models/camera_rectifier.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "sensor_models/camera_model.hpp"

namespace uw::sensor_models {

bool PlumbBobDistortion::IsIdentity() const {
  return k1 == 0.0 && k2 == 0.0 && p1 == 0.0 && p2 == 0.0 && k3 == 0.0;
}

std::optional<PlumbBobDistortion> PlumbBobDistortion::FromIntrinsics(
    const uw::domain::CameraIntrinsics& intrinsics) {
  if (!intrinsics.distortion_model().empty() && intrinsics.distortion_model() != "plumb_bob") {
    return std::nullopt;
  }
  const int n = intrinsics.distortion_size();
  if (n != 0 && n != 4 && n != 5) return std::nullopt;

  PlumbBobDistortion distortion;
  if (n >= 1) distortion.k1 = intrinsics.distortion(0);
  if (n >= 2) distortion.k2 = intrinsics.distortion(1);
  if (n >= 3) distortion.p1 = intrinsics.distortion(2);
  if (n >= 4) distortion.p2 = intrinsics.distortion(3);
  if (n >= 5) distortion.k3 = intrinsics.distortion(4);
  return distortion;
}

Eigen::Vector2d ApplyPlumbBobDistortion(const PlumbBobDistortion& distortion,
                                         const Eigen::Vector2d& normalized_undistorted) {
  const double x = normalized_undistorted.x();
  const double y = normalized_undistorted.y();
  const double r2 = x * x + y * y;
  const double radial = 1.0 + distortion.k1 * r2 + distortion.k2 * r2 * r2 + distortion.k3 * r2 * r2 * r2;
  const double x_distorted = x * radial + 2.0 * distortion.p1 * x * y + distortion.p2 * (r2 + 2.0 * x * x);
  const double y_distorted = y * radial + distortion.p1 * (r2 + 2.0 * y * y) + 2.0 * distortion.p2 * x * y;
  return Eigen::Vector2d(x_distorted, y_distorted);
}

namespace {

std::optional<uint32_t> BytesPerPixel(uw::domain::ImageFrame::ImageEncoding encoding) {
  switch (encoding) {
    case uw::domain::ImageFrame::IMAGE_ENCODING_MONO8:
      return 1;
    case uw::domain::ImageFrame::IMAGE_ENCODING_RGB8:
    case uw::domain::ImageFrame::IMAGE_ENCODING_BGR8:
      return 3;
    default:
      return std::nullopt;
  }
}

}  // namespace

std::optional<uw::domain::ImageFrame> UndistortImage(const uw::domain::ImageFrame& raw,
                                                       const uw::domain::CameraIntrinsics& intrinsics) {
  if (!uw::domain::ValidateImageFrame(raw).ok()) return std::nullopt;
  if (raw.width() != intrinsics.width() || raw.height() != intrinsics.height()) return std::nullopt;

  const auto distortion = PlumbBobDistortion::FromIntrinsics(intrinsics);
  if (!distortion.has_value()) return std::nullopt;
  if (distortion->IsIdentity()) return raw;

  const auto bytes_per_pixel = BytesPerPixel(raw.encoding());
  if (!bytes_per_pixel.has_value()) return std::nullopt;  // ValidateImageFrame already rejects this

  const PinholeCamera camera = PinholeCamera::FromIntrinsics(intrinsics);
  if (camera.fx == 0.0 || camera.fy == 0.0) return std::nullopt;  // degenerate intrinsics

  const uint32_t width = raw.width();
  const uint32_t height = raw.height();
  const uint32_t src_stride = raw.row_stride_bytes();
  const uint32_t dst_stride = width * *bytes_per_pixel;
  const auto* src = reinterpret_cast<const unsigned char*>(raw.pixel_data().data());

  std::string dst_pixels(static_cast<std::size_t>(dst_stride) * height, '\0');

  const auto sample = [&](int u, int v, uint32_t channel) -> double {
    return static_cast<double>(
        src[static_cast<std::size_t>(v) * src_stride + static_cast<std::size_t>(u) * *bytes_per_pixel + channel]);
  };

  // For each destination (undistorted) pixel, apply the FORWARD distortion
  // model to its normalized coordinate to find where it came from in the
  // source (distorted) image — the standard remap approach; no inversion of
  // the distortion polynomial is needed. See this file's header comment for
  // why the destination uses the same K as the source (v1 scope).
  for (uint32_t v = 0; v < height; ++v) {
    for (uint32_t u = 0; u < width; ++u) {
      const double x = (static_cast<double>(u) - camera.cx) / camera.fx;
      const double y = (static_cast<double>(v) - camera.cy) / camera.fy;
      const Eigen::Vector2d distorted_normalized = ApplyPlumbBobDistortion(*distortion, Eigen::Vector2d(x, y));
      const double src_u = distorted_normalized.x() * camera.fx + camera.cx;
      const double src_v = distorted_normalized.y() * camera.fy + camera.cy;

      unsigned char* dst_pixel = reinterpret_cast<unsigned char*>(
          &dst_pixels[static_cast<std::size_t>(v) * dst_stride + static_cast<std::size_t>(u) * *bytes_per_pixel]);
      if (src_u < 0.0 || src_v < 0.0 || src_u > static_cast<double>(width) - 1.0 ||
          src_v > static_cast<double>(height) - 1.0) {
        continue;  // off the source image; leave black (already zero-initialized)
      }

      const int u0 = static_cast<int>(std::floor(src_u));
      const int v0 = static_cast<int>(std::floor(src_v));
      const int u1 = std::min(u0 + 1, static_cast<int>(width) - 1);
      const int v1 = std::min(v0 + 1, static_cast<int>(height) - 1);
      const double fu = src_u - u0;
      const double fv = src_v - v0;

      for (uint32_t c = 0; c < *bytes_per_pixel; ++c) {
        const double top = sample(u0, v0, c) * (1.0 - fu) + sample(u1, v0, c) * fu;
        const double bottom = sample(u0, v1, c) * (1.0 - fu) + sample(u1, v1, c) * fu;
        const double value = top * (1.0 - fv) + bottom * fv;
        dst_pixel[c] = static_cast<unsigned char>(std::lround(std::clamp(value, 0.0, 255.0)));
      }
    }
  }

  uw::domain::ImageFrame result = raw;
  result.set_row_stride_bytes(dst_stride);
  result.set_pixel_data(dst_pixels);
  result.set_is_rectified(true);
  return result;
}

}  // namespace uw::sensor_models
