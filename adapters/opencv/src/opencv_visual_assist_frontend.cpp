#include "opencv_visual_assist_frontend.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace uw::opencv_adapters {
namespace {

constexpr char kImplementationLabel[] = "sim_fixture_detector_v1";
constexpr double kPi = 3.14159265358979323846;

bool FiniteNonNegative(double value) { return std::isfinite(value) && value >= 0.0; }
bool FinitePositive(double value) { return std::isfinite(value) && value > 0.0; }
bool HasFiniteSquare(double value) {
  return FiniteNonNegative(value) && std::isfinite(value * value);
}

void ValidateParams(const VisualAssistParams& params) {
  const bool valid_hsv = params.hsv_hue_min >= 0 && params.hsv_hue_min <= 179 &&
                         params.hsv_hue_max >= params.hsv_hue_min &&
                         params.hsv_hue_max <= 179 && params.hsv_saturation_min >= 0 &&
                         params.hsv_saturation_min <= 255 && params.hsv_value_min >= 0 &&
                         params.hsv_value_min <= 255;
  const bool valid_quality = FiniteNonNegative(params.minimum_brightness) &&
                             params.minimum_brightness <= 255.0 &&
                             FiniteNonNegative(params.minimum_contrast) &&
                             FiniteNonNegative(params.minimum_laplacian_variance) &&
                             FiniteNonNegative(params.minimum_texture_support) &&
                             params.minimum_texture_support <= 1.0;
  const bool valid_structure = FiniteNonNegative(params.canny_low_threshold) &&
                               FinitePositive(params.canny_high_threshold) &&
                               params.canny_high_threshold > params.canny_low_threshold &&
                               params.hough_vote_threshold > 0 &&
                               FinitePositive(params.hough_min_line_length_px) &&
                               FiniteNonNegative(params.hough_max_line_gap_px) &&
                               FinitePositive(params.minimum_structure_vertical_span_px) &&
                               FinitePositive(params.structure_orientation_cluster_tolerance_rad) &&
                               params.structure_orientation_cluster_tolerance_rad <= kPi * 0.5 &&
                               FinitePositive(params.structure_normal_distance_cluster_px) &&
                               params.minimum_structure_line_count > 0 &&
                               FinitePositive(params.structure_reference_range_m) &&
                               FinitePositive(params.path_offset_sigma_m);
  const bool valid_uncertainty = params.minimum_valid_depth_samples >= 9 &&
                                 FinitePositive(params.depth_mad_scale) &&
                                 FinitePositive(params.minimum_range_sigma_m) &&
                                 HasFiniteSquare(params.minimum_range_sigma_m) &&
                                 FinitePositive(params.bearing_sigma_rad) &&
                                 HasFiniteSquare(params.bearing_sigma_rad) &&
                                 FinitePositive(params.unobserved_range_variance_m2);
  if (!valid_hsv || params.minimum_component_area_px <= 0 || !valid_quality ||
      !valid_structure || !valid_uncertainty || params.class_label.empty()) {
    throw std::invalid_argument("invalid visual-assist frontend parameters");
  }
}

bool ValidIntrinsics(const uw::domain::CameraIntrinsics& intrinsics,
                     const uw::domain::ImageFrame& image) {
  if (intrinsics.sensor_id().value() != image.header().sensor_id().value() ||
      intrinsics.width() != image.width() || intrinsics.height() != image.height() ||
      intrinsics.k_matrix_row_major_size() != 9) {
    return false;
  }
  for (double value : intrinsics.k_matrix_row_major()) {
    if (!std::isfinite(value)) return false;
  }
  for (double value : intrinsics.distortion()) {
    if (!std::isfinite(value)) return false;
  }
  const double fx = intrinsics.k_matrix_row_major(0);
  const double fy = intrinsics.k_matrix_row_major(4);
  const double cx = intrinsics.k_matrix_row_major(2);
  const double cy = intrinsics.k_matrix_row_major(5);
  return fx > 0.0 && fy > 0.0 && cx >= 0.0 && cy >= 0.0 &&
         cx < static_cast<double>(image.width()) && cy < static_cast<double>(image.height());
}

bool IsSupportedColorEncoding(uw::domain::ImageFrame::ImageEncoding encoding) {
  return encoding == uw::domain::ImageFrame::IMAGE_ENCODING_RGB8 ||
         encoding == uw::domain::ImageFrame::IMAGE_ENCODING_BGR8;
}

cv::Mat ColorView(const uw::domain::ImageFrame& image) {
  return cv::Mat(static_cast<int>(image.height()), static_cast<int>(image.width()), CV_8UC3,
                 const_cast<char*>(image.pixel_data().data()), image.row_stride_bytes());
}

bool DepthGridCompatible(const uw::domain::OpticalDepthPriorMeasurement& depth,
                         const uw::domain::ImageFrame& image) {
  const std::size_t pixel_count = static_cast<std::size_t>(image.width()) * image.height();
  return pixel_count <= static_cast<std::size_t>(std::numeric_limits<int>::max()) &&
         depth.width() == image.width() && depth.height() == image.height() &&
         depth.depth_m_size() == static_cast<int>(pixel_count) &&
         depth.variance_m2_size() == static_cast<int>(pixel_count) &&
         depth.valid_mask().size() == pixel_count &&
         depth.scale_status() == uw::domain::OPTICAL_DEPTH_SCALE_STATUS_METRIC &&
         depth.reference_camera_frame().value() == image.header().sensor_frame().value();
}

double Median(std::vector<double>* values) {
  std::sort(values->begin(), values->end());
  const std::size_t middle = values->size() / 2;
  if (values->size() % 2 == 1) return (*values)[middle];
  return ((*values)[middle - 1] + (*values)[middle]) * 0.5;
}

struct TargetDepthStatistics {
  std::size_t central_pixel_count = 0;
  std::vector<double> samples;
};

TargetDepthStatistics CollectTargetDepth(
    const uw::domain::OpticalDepthPriorMeasurement& depth,
    const uw::domain::TargetDetection& target) {
  const uint32_t x_begin = target.bbox_x() + target.bbox_width() / 4;
  const uint32_t x_end = target.bbox_x() + (3 * target.bbox_width()) / 4;
  const uint32_t y_begin = target.bbox_y() + target.bbox_height() / 4;
  const uint32_t y_end = target.bbox_y() + (3 * target.bbox_height()) / 4;

  TargetDepthStatistics statistics;
  statistics.central_pixel_count =
      static_cast<std::size_t>(x_end - x_begin) * static_cast<std::size_t>(y_end - y_begin);
  statistics.samples.reserve(statistics.central_pixel_count);
  for (uint32_t y = y_begin; y < y_end; ++y) {
    for (uint32_t x = x_begin; x < x_end; ++x) {
      const std::size_t index = static_cast<std::size_t>(y) * depth.width() + x;
      if (static_cast<unsigned char>(depth.valid_mask()[index]) == 0) continue;
      const double sample = depth.depth_m(static_cast<int>(index));
      const double variance = depth.variance_m2(static_cast<int>(index));
      if (std::isfinite(sample) && sample > 0.0 && std::isfinite(variance) && variance > 0.0) {
        statistics.samples.push_back(sample);
      }
    }
  }
  return statistics;
}

void PopulatePathOffset(const cv::Mat& edges, const VisualAssistParams& params,
                        const uw::domain::CameraIntrinsics& intrinsics,
                        uw::measurement_api::VisualAssistResult* result) {
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(edges, lines, 1.0, CV_PI / 180.0, params.hough_vote_threshold,
                  params.hough_min_line_length_px, params.hough_max_line_gap_px);

  struct StructureLine {
    double angle_rad = 0.0;
    double midpoint_x = 0.0;
    double midpoint_y = 0.0;
    double center_u = 0.0;
    double length = 0.0;
  };
  std::vector<StructureLine> structure_lines;
  for (const cv::Vec4i& line : lines) {
    const double dx = static_cast<double>(line[2] - line[0]);
    const double dy = static_cast<double>(line[3] - line[1]);
    if (std::abs(dy) < params.minimum_structure_vertical_span_px) continue;
    const double length = std::hypot(dx, dy);
    if (!FinitePositive(length)) continue;
    double angle = std::atan2(dy, dx);
    if (angle < 0.0) angle += kPi;
    if (angle >= kPi) angle -= kPi;
    structure_lines.push_back(
        StructureLine{angle, 0.5 * static_cast<double>(line[0] + line[2]),
                      0.5 * static_cast<double>(line[1] + line[3]),
                      0.5 * static_cast<double>(line[0] + line[2]), length});
  }

  std::sort(structure_lines.begin(), structure_lines.end(),
            [](const StructureLine& lhs, const StructureLine& rhs) {
              if (lhs.angle_rad != rhs.angle_rad) return lhs.angle_rad < rhs.angle_rad;
              if (lhs.midpoint_x != rhs.midpoint_x) return lhs.midpoint_x < rhs.midpoint_x;
              return lhs.midpoint_y < rhs.midpoint_y;
            });

  struct StructureSupport {
    double angle_rad = 0.0;
    double reference_midpoint_x = 0.0;
    double reference_midpoint_y = 0.0;
    double weighted_center_sum = 0.0;
    double total_length = 0.0;
  };
  std::vector<StructureSupport> supports;
  for (const StructureLine& line : structure_lines) {
    StructureSupport* matched_support = nullptr;
    for (auto& support : supports) {
      const double direct_angle_difference = std::abs(line.angle_rad - support.angle_rad);
      const double angle_difference =
          std::min(direct_angle_difference, kPi - direct_angle_difference);
      if (angle_difference > params.structure_orientation_cluster_tolerance_rad) continue;
      const double normal_x = -std::sin(support.angle_rad);
      const double normal_y = std::cos(support.angle_rad);
      const double normal_distance =
          std::abs((line.midpoint_x - support.reference_midpoint_x) * normal_x +
                   (line.midpoint_y - support.reference_midpoint_y) * normal_y);
      if (normal_distance <= params.structure_normal_distance_cluster_px) {
        matched_support = &support;
        break;
      }
    }
    if (matched_support == nullptr) {
      supports.push_back(StructureSupport{line.angle_rad, line.midpoint_x, line.midpoint_y,
                                          0.0, 0.0});
      matched_support = &supports.back();
    }
    matched_support->weighted_center_sum += line.center_u * line.length;
    matched_support->total_length += line.length;
  }
  if (supports.size() < params.minimum_structure_line_count) return;

  double structure_center_sum = 0.0;
  for (const StructureSupport& support : supports) {
    if (!FinitePositive(support.total_length)) return;
    structure_center_sum += support.weighted_center_sum / support.total_length;
  }

  const double structure_center_u = structure_center_sum / static_cast<double>(supports.size());
  const double fx = intrinsics.k_matrix_row_major(0);
  const double cx = intrinsics.k_matrix_row_major(2);
  result->path_lateral_offset_m =
      (structure_center_u - cx) * params.structure_reference_range_m / fx;
  result->path_offset_sigma_m = params.path_offset_sigma_m;
}

bool TargetOrder(const uw::domain::TargetDetection& lhs,
                 const uw::domain::TargetDetection& rhs) {
  if (lhs.confidence() != rhs.confidence()) return lhs.confidence() > rhs.confidence();
  if (lhs.bbox_x() != rhs.bbox_x()) return lhs.bbox_x() < rhs.bbox_x();
  if (lhs.bbox_y() != rhs.bbox_y()) return lhs.bbox_y() < rhs.bbox_y();
  if (lhs.bbox_width() != rhs.bbox_width()) return lhs.bbox_width() < rhs.bbox_width();
  return lhs.bbox_height() < rhs.bbox_height();
}

}  // namespace

OpenCvVisualAssistFrontend::OpenCvVisualAssistFrontend(VisualAssistParams params)
    : params_(std::move(params)) {
  ValidateParams(params_);
}

uw::measurement_api::VisualAssistResult OpenCvVisualAssistFrontend::Process(
    const uw::domain::ImageFrame& left_rectified,
    const std::optional<uw::domain::OpticalDepthPriorMeasurement>& depth,
    const uw::domain::CameraIntrinsics& intrinsics) {
  uw::measurement_api::VisualAssistResult result;
  result.health.set_component_id(kImplementationLabel);

  if (!uw::domain::ValidateImageFrame(left_rectified).ok() ||
      !uw::domain::ValidateObservationHeader(left_rectified.header()).ok() ||
      left_rectified.width() > static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
      left_rectified.height() > static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
      !left_rectified.is_rectified() || !IsSupportedColorEncoding(left_rectified.encoding()) ||
      !ValidIntrinsics(intrinsics, left_rectified)) {
    result.health.set_status(uw::domain::HealthReport::STATUS_UNAVAILABLE);
    result.health.set_reason_code("visual_unavailable");
    return result;
  }

  const cv::Mat color = ColorView(left_rectified);
  const bool is_rgb = left_rectified.encoding() == uw::domain::ImageFrame::IMAGE_ENCODING_RGB8;
  cv::Mat gray;
  cv::cvtColor(color, gray, is_rgb ? cv::COLOR_RGB2GRAY : cv::COLOR_BGR2GRAY);

  cv::Scalar mean;
  cv::Scalar standard_deviation;
  cv::meanStdDev(gray, mean, standard_deviation);
  const double brightness = mean[0];
  const double contrast = standard_deviation[0];

  cv::Mat laplacian;
  cv::Laplacian(gray, laplacian, CV_64F);
  cv::Scalar laplacian_mean;
  cv::Scalar laplacian_standard_deviation;
  cv::meanStdDev(laplacian, laplacian_mean, laplacian_standard_deviation);
  const double laplacian_variance = laplacian_standard_deviation[0] *
                                    laplacian_standard_deviation[0];

  cv::Mat edges;
  cv::Canny(gray, edges, params_.canny_low_threshold, params_.canny_high_threshold);
  const double texture_support = static_cast<double>(cv::countNonZero(edges)) /
                                 static_cast<double>(left_rectified.width()) /
                                 static_cast<double>(left_rectified.height());

  if (brightness < params_.minimum_brightness) {
    result.health.set_status(uw::domain::HealthReport::STATUS_SUSPECT);
    result.health.set_reason_code("visual_low_light");
    return result;
  }
  if (contrast < params_.minimum_contrast) {
    result.health.set_status(uw::domain::HealthReport::STATUS_SUSPECT);
    result.health.set_reason_code("visual_low_contrast");
    return result;
  }
  if (laplacian_variance < params_.minimum_laplacian_variance) {
    result.health.set_status(uw::domain::HealthReport::STATUS_SUSPECT);
    result.health.set_reason_code("visual_blurred");
    return result;
  }
  if (texture_support < params_.minimum_texture_support) {
    result.health.set_status(uw::domain::HealthReport::STATUS_SUSPECT);
    result.health.set_reason_code("visual_low_contrast");
    return result;
  }

  cv::Mat hsv;
  cv::cvtColor(color, hsv, is_rgb ? cv::COLOR_RGB2HSV : cv::COLOR_BGR2HSV);
  cv::Mat mask;
  cv::inRange(hsv,
              cv::Scalar(params_.hsv_hue_min, params_.hsv_saturation_min,
                         params_.hsv_value_min),
              cv::Scalar(params_.hsv_hue_max, 255, 255), mask);

  cv::Mat labels;
  cv::Mat stats;
  cv::Mat centroids;
  const int component_count = cv::connectedComponentsWithStats(mask, labels, stats, centroids);
  for (int component = 1; component < component_count; ++component) {
    const int area = stats.at<int>(component, cv::CC_STAT_AREA);
    if (area < params_.minimum_component_area_px) continue;

    const int x = stats.at<int>(component, cv::CC_STAT_LEFT);
    const int y = stats.at<int>(component, cv::CC_STAT_TOP);
    const int width = stats.at<int>(component, cv::CC_STAT_WIDTH);
    const int height = stats.at<int>(component, cv::CC_STAT_HEIGHT);
    const double center_u = centroids.at<double>(component, 0);
    const double fx = intrinsics.k_matrix_row_major(0);
    const double cx = intrinsics.k_matrix_row_major(2);

    uw::domain::TargetDetection target;
    *target.mutable_source_observation() = left_rectified.header().observation_id();
    *target.mutable_capture_time() = left_rectified.header().capture_time();
    target.set_class_label(params_.class_label);
    target.set_confidence(std::clamp(static_cast<double>(area) /
                                         static_cast<double>(width * height),
                                     0.0, 1.0));
    target.set_bearing_rad(std::atan2(center_u - cx, fx));
    target.set_has_range(false);
    target.add_covariance_2x2_row_major(params_.bearing_sigma_rad * params_.bearing_sigma_rad);
    target.add_covariance_2x2_row_major(0.0);
    target.add_covariance_2x2_row_major(0.0);
    target.add_covariance_2x2_row_major(params_.unobserved_range_variance_m2);
    target.set_bbox_x(static_cast<uint32_t>(x));
    target.set_bbox_y(static_cast<uint32_t>(y));
    target.set_bbox_width(static_cast<uint32_t>(width));
    target.set_bbox_height(static_cast<uint32_t>(height));
    target.set_source(uw::domain::ASSIST_SOURCE_VISUAL);
    target.set_angular_extent_rad(2.0 * std::atan2(0.5 * width, fx));
    target.set_intensity_score(target.confidence());
    (*target.mutable_quality_metrics())["brightness"] = brightness;
    (*target.mutable_quality_metrics())["contrast"] = contrast;
    (*target.mutable_quality_metrics())["laplacian_blur_score"] = laplacian_variance;
    (*target.mutable_quality_metrics())["texture_support"] = texture_support;
    (*target.mutable_quality_metrics())["valid_depth_ratio"] = 0.0;
    result.targets.push_back(std::move(target));
  }

  bool depth_degraded =
      !depth.has_value() || !DepthGridCompatible(*depth, left_rectified);
  if (!depth_degraded) {
    for (auto& target : result.targets) {
      TargetDepthStatistics statistics = CollectTargetDepth(*depth, target);
      const double ratio = statistics.central_pixel_count == 0
                               ? 0.0
                               : static_cast<double>(statistics.samples.size()) /
                                     static_cast<double>(statistics.central_pixel_count);
      (*target.mutable_quality_metrics())["valid_depth_ratio"] = ratio;
      if (statistics.samples.size() < params_.minimum_valid_depth_samples) {
        depth_degraded = true;
        continue;
      }

      const double median = Median(&statistics.samples);
      std::vector<double> deviations;
      deviations.reserve(statistics.samples.size());
      for (double sample : statistics.samples) deviations.push_back(std::abs(sample - median));
      const double mad = Median(&deviations);
      const double scaled_mad = params_.depth_mad_scale * mad;
      const double sigma = std::max(params_.minimum_range_sigma_m, scaled_mad);
      const double range_variance = sigma * sigma;
      const double range_extent = 2.0 * sigma;
      if (!std::isfinite(median) || !std::isfinite(scaled_mad) || !std::isfinite(sigma) ||
          !std::isfinite(range_variance) || !std::isfinite(range_extent)) {
        depth_degraded = true;
        continue;
      }
      target.set_has_range(true);
      target.set_range_m(median);
      target.set_covariance_2x2_row_major(3, range_variance);
      target.set_range_extent_m(range_extent);
    }
  }

  std::sort(result.targets.begin(), result.targets.end(), TargetOrder);
  PopulatePathOffset(edges, params_, intrinsics, &result);

  if (depth_degraded) {
    result.health.set_status(uw::domain::HealthReport::STATUS_SUSPECT);
    result.health.set_reason_code("stereo_depth_unavailable");
  } else {
    result.health.set_status(uw::domain::HealthReport::STATUS_HEALTHY);
  }
  return result;
}

}  // namespace uw::opencv_adapters
