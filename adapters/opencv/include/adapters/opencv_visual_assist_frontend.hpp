// OpenCV is isolated behind this repo-native interface. This public header
// intentionally exposes no cv:: types or OpenCV headers.
#pragma once

#include <cstddef>
#include <string>

#include "measurement_api/target_frontend.hpp"

namespace uw::opencv_adapters {

struct VisualAssistParams {
  int hsv_hue_min = 45;
  int hsv_hue_max = 85;
  int hsv_saturation_min = 100;
  int hsv_value_min = 80;
  int minimum_component_area_px = 500;

  double minimum_brightness = 10.0;
  double minimum_contrast = 5.0;
  double minimum_laplacian_variance = 1.0;
  double minimum_texture_support = 0.0001;

  double canny_low_threshold = 50.0;
  double canny_high_threshold = 150.0;
  int hough_vote_threshold = 30;
  double hough_min_line_length_px = 40.0;
  double hough_max_line_gap_px = 12.0;
  double minimum_structure_vertical_span_px = 80.0;
  std::size_t minimum_structure_line_count = 2;
  double structure_reference_range_m = 4.0;
  double path_offset_sigma_m = 0.15;

  std::size_t minimum_valid_depth_samples = 9;
  double depth_mad_scale = 1.4826;
  double minimum_range_sigma_m = 0.02;
  double bearing_sigma_rad = 0.01;
  double unobserved_range_variance_m2 = 1000000.0;
  std::string class_label = "aquaculture_zone";
};

class OpenCvVisualAssistFrontend final : public uw::measurement_api::VisualAssistFrontend {
 public:
  explicit OpenCvVisualAssistFrontend(VisualAssistParams params);

  uw::measurement_api::VisualAssistResult Process(
      const uw::domain::ImageFrame& left_rectified,
      const std::optional<uw::domain::OpticalDepthPriorMeasurement>& depth,
      const uw::domain::CameraIntrinsics& intrinsics) override;

 private:
  VisualAssistParams params_;
};

}  // namespace uw::opencv_adapters
