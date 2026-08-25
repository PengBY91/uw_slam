#include "domain/domain.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
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
  for (int i = 0; i < frame.azimuth_angles_size(); ++i) {
    if (!std::isfinite(frame.azimuth_angles(i))) return false;
    if (i > 0 && frame.azimuth_angles(i) <= frame.azimuth_angles(i - 1)) return false;
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

bool CheckedMultiply(std::size_t lhs, std::size_t rhs, std::size_t* product) {
  if (lhs != 0 && rhs > std::numeric_limits<std::size_t>::max() / lhs) return false;
  *product = lhs * rhs;
  return true;
}

// Enforces the OpticalDepthPriorMeasurement/FusedDepthMeasurement convention
// (camera-optical-frame, positive z-forward range — see those messages'
// field comments in schemas/proto/uw/domain/measurement.proto). This is a
// DIFFERENT convention from PressureDepthMeasurement.depth_m (world-frame,
// positive-down); PressureDepthMeasurement has no equivalent Validate*
// function and passes through unchecked.
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
  std::size_t minimum_stride = 0;
  if (!CheckedMultiply(frame.width(), bytes_per_pixel, &minimum_stride) ||
      minimum_stride > std::numeric_limits<uint32_t>::max() ||
      frame.row_stride_bytes() < minimum_stride) {
    return Invalid(ValidationCode::kInvalidImageStride, "image stride is smaller than one row");
  }
  std::size_t expected = 0;
  if (!CheckedMultiply(frame.height(), frame.row_stride_bytes(), &expected)) {
    return Invalid(ValidationCode::kImagePayloadSizeMismatch,
                   "image payload size overflows the addressable range");
  }
  if (frame.pixel_data().size() != expected) {
    return Invalid(ValidationCode::kImagePayloadSizeMismatch, "image payload size does not match shape");
  }
  return {};
}

ValidationResult ValidateObservationHeader(const ObservationHeader& header) {
  if (!header.has_sequence_id() || !header.has_capture_time() ||
      !header.has_receive_time()) {
    return Invalid(ValidationCode::kInvalidObservationHeader,
                   "sequence, capture time, and receive time must be present");
  }
  if (header.observation_id().value().empty() || header.sensor_id().value().empty() ||
      header.sensor_frame().value().empty() || header.calibration_version().value().empty()) {
    return Invalid(ValidationCode::kInvalidObservationHeader,
                   "observation, sensor, frame, and calibration identifiers must be non-empty");
  }
  if (header.clock_domain() == CLOCK_DOMAIN_UNSPECIFIED ||
      header.receive_clock_domain() == CLOCK_DOMAIN_UNSPECIFIED) {
    return Invalid(ValidationCode::kInvalidObservationHeader,
                   "capture and receive clock domains must be explicit");
  }
  constexpr int32_t kNanosPerSecond = 1000000000;
  if (header.capture_time().nanos() < 0 ||
      header.capture_time().nanos() >= kNanosPerSecond ||
      header.receive_time().nanos() < 0 ||
      header.receive_time().nanos() >= kNanosPerSecond) {
    return Invalid(ValidationCode::kInvalidObservationHeader,
                   "capture and receive timestamp nanos must be normalized");
  }
  if (header.validity() != ObservationHeader::VALIDITY_OK &&
      header.validity() != ObservationHeader::VALIDITY_DEGRADED) {
    return Invalid(ValidationCode::kInvalidObservationHeader,
                   "observation validity must be OK or DEGRADED");
  }
  return {};
}

std::optional<ImageFrame> ConvertToMono8(const ImageFrame& frame) {
  if (!ValidateImageFrame(frame).ok()) return std::nullopt;
  if (frame.encoding() == ImageFrame::IMAGE_ENCODING_MONO8) return frame;
  if (frame.encoding() != ImageFrame::IMAGE_ENCODING_RGB8 &&
      frame.encoding() != ImageFrame::IMAGE_ENCODING_BGR8) {
    return std::nullopt;
  }
  const bool is_rgb = frame.encoding() == ImageFrame::IMAGE_ENCODING_RGB8;
  const uint32_t width = frame.width();
  const uint32_t height = frame.height();
  const uint32_t src_stride = frame.row_stride_bytes();
  const uint32_t dst_stride = width;

  std::string gray(static_cast<std::size_t>(dst_stride) * height, '\0');
  const auto* src = reinterpret_cast<const unsigned char*>(frame.pixel_data().data());
  for (uint32_t y = 0; y < height; ++y) {
    const unsigned char* row = src + static_cast<std::size_t>(y) * src_stride;
    char* out_row = gray.data() + static_cast<std::size_t>(y) * dst_stride;
    for (uint32_t x = 0; x < width; ++x) {
      const unsigned char c0 = row[x * 3 + 0];
      const unsigned char c1 = row[x * 3 + 1];
      const unsigned char c2 = row[x * 3 + 2];
      const unsigned char r = is_rgb ? c0 : c2;
      const unsigned char b = is_rgb ? c2 : c0;
      const double luminance = 0.299 * r + 0.587 * c1 + 0.114 * b;
      out_row[x] = static_cast<char>(static_cast<unsigned char>(std::lround(luminance)));
    }
  }

  ImageFrame result = frame;
  result.set_encoding(ImageFrame::IMAGE_ENCODING_MONO8);
  result.set_row_stride_bytes(dst_stride);
  result.set_pixel_data(gray);
  return result;
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
