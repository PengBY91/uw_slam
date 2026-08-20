#include "uw/domain/domain.hpp"

#include <cmath>
#include <cstdint>
#include <utility>

#include <google/protobuf/repeated_field.h>

namespace uw::domain {

Stamp ToStamp(std::chrono::system_clock::time_point tp) {
  const auto since_epoch = tp.time_since_epoch();
  const auto secs = std::chrono::duration_cast<std::chrono::seconds>(since_epoch);
  const auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(since_epoch - secs);
  Stamp stamp;
  stamp.set_seconds(secs.count());
  stamp.set_nanos(static_cast<int32_t>(nanos.count()));
  return stamp;
}

std::chrono::system_clock::time_point ToTimePoint(const Stamp& stamp) {
  return std::chrono::system_clock::time_point{std::chrono::seconds(stamp.seconds()) +
                                                 std::chrono::nanoseconds(stamp.nanos())};
}

double ToSeconds(const Stamp& stamp) {
  return static_cast<double>(stamp.seconds()) + static_cast<double>(stamp.nanos()) * 1e-9;
}

Stamp FromSeconds(double seconds) {
  Stamp stamp;
  const int64_t whole = static_cast<int64_t>(std::floor(seconds));
  stamp.set_seconds(whole);
  stamp.set_nanos(static_cast<int32_t>((seconds - static_cast<double>(whole)) * 1e9));
  return stamp;
}

bool IsAzimuthAscending(const SonarFrame& frame) {
  for (int i = 1; i < frame.azimuth_angles_size(); ++i) {
    if (frame.azimuth_angles(i) <= frame.azimuth_angles(i - 1)) return false;
  }
  return true;
}

namespace {

ValidationResult Invalid(ValidationCode code, std::string message) {
  return ValidationResult{code, std::move(message)};
}

std::size_t PixelCount(uint32_t width, uint32_t height) {
  return static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
}

ValidationResult ValidateDepthValues(const google::protobuf::RepeatedField<float>& depth,
                                     const google::protobuf::RepeatedField<float>& variance,
                                     const std::string& valid_mask) {
  for (int i = 0; i < depth.size(); ++i) {
    if (static_cast<unsigned char>(valid_mask[static_cast<std::size_t>(i)]) == 0) continue;
    if (!std::isfinite(depth.Get(i)) || depth.Get(i) <= 0.0f) {
      return Invalid(ValidationCode::kInvalidDepthValue,
                     "valid depth values must be finite and positive");
    }
    if (!std::isfinite(variance.Get(i)) || variance.Get(i) <= 0.0f) {
      return Invalid(ValidationCode::kInvalidVarianceValue,
                     "valid variance values must be finite and positive");
    }
  }
  return {};
}

}  // namespace

ValidationResult ValidateImageFrame(const ImageFrame& frame) {
  if (frame.width() == 0 || frame.height() == 0) {
    return Invalid(ValidationCode::kMissingDimensions, "image dimensions must be non-zero");
  }
  uint32_t bytes_per_pixel = 0;
  switch (frame.encoding()) {
    case ImageFrame::IMAGE_ENCODING_MONO8:
      bytes_per_pixel = 1;
      break;
    case ImageFrame::IMAGE_ENCODING_RGB8:
    case ImageFrame::IMAGE_ENCODING_BGR8:
      bytes_per_pixel = 3;
      break;
    default:
      return Invalid(ValidationCode::kUnsupportedImageEncoding, "unsupported image encoding");
  }
  const uint32_t minimum_stride = frame.width() * bytes_per_pixel;
  if (frame.row_stride_bytes() < minimum_stride) {
    return Invalid(ValidationCode::kInvalidImageStride, "image stride is smaller than one row");
  }
  const std::size_t expected =
      static_cast<std::size_t>(frame.height()) * frame.row_stride_bytes();
  if (frame.pixel_data().size() != expected) {
    return Invalid(ValidationCode::kImagePayloadSizeMismatch, "image payload size does not match shape");
  }
  return {};
}

ValidationResult ValidateOpticalDepthPrior(const OpticalDepthPriorMeasurement& prior) {
  const std::size_t pixels = PixelCount(prior.width(), prior.height());
  if (pixels == 0) {
    return Invalid(ValidationCode::kMissingDimensions, "depth dimensions must be non-zero");
  }
  if (prior.depth_m_size() != static_cast<int>(pixels) ||
      prior.variance_m2_size() != static_cast<int>(pixels)) {
    return Invalid(ValidationCode::kDepthGridSizeMismatch, "depth and variance must match shape");
  }
  if (prior.valid_mask().size() != pixels) {
    return Invalid(ValidationCode::kDepthMaskSizeMismatch, "valid mask must match shape");
  }
  if (prior.scale_status() == OPTICAL_DEPTH_SCALE_STATUS_UNSPECIFIED) {
    return Invalid(ValidationCode::kInvalidScaleStatus, "optical scale status must be explicit");
  }
  return ValidateDepthValues(prior.depth_m(), prior.variance_m2(), prior.valid_mask());
}

ValidationResult ValidateFusedDepth(const FusedDepthMeasurement& fused) {
  const std::size_t pixels = PixelCount(fused.width(), fused.height());
  if (pixels == 0) {
    return Invalid(ValidationCode::kMissingDimensions, "fused depth dimensions must be non-zero");
  }
  if (fused.depth_m_size() != static_cast<int>(pixels) ||
      fused.variance_m2_size() != static_cast<int>(pixels)) {
    return Invalid(ValidationCode::kDepthGridSizeMismatch, "fused depth and variance must match shape");
  }
  if (fused.valid_mask().size() != pixels) {
    return Invalid(ValidationCode::kDepthMaskSizeMismatch, "fused valid mask must match shape");
  }
  if (fused.contribution_mask().size() != pixels) {
    return Invalid(ValidationCode::kContributionMaskSizeMismatch,
                   "contribution mask must match shape");
  }
  return ValidateDepthValues(fused.depth_m(), fused.variance_m2(), fused.valid_mask());
}

}  // namespace uw::domain
