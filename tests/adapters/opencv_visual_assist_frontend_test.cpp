#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "adapters/opencv_visual_assist_frontend.hpp"

namespace {

using uw::opencv_adapters::OpenCvVisualAssistFrontend;
using uw::opencv_adapters::VisualAssistParams;

uw::domain::ImageFrame MakeRgbImage(uint32_t width, uint32_t height,
                                    const std::array<uint8_t, 3>& fill) {
  uw::domain::ImageFrame image;
  auto* header = image.mutable_header();
  header->mutable_observation_id()->set_value("camera_left_obs_17");
  header->mutable_sensor_id()->set_value("camera_left");
  header->mutable_sequence_id()->set_value(17);
  header->mutable_capture_time()->set_seconds(123);
  header->mutable_capture_time()->set_nanos(456000000);
  header->mutable_receive_time()->set_seconds(123);
  header->mutable_receive_time()->set_nanos(457000000);
  header->set_clock_domain(uw::domain::CLOCK_DOMAIN_SIMULATION);
  header->set_receive_clock_domain(uw::domain::CLOCK_DOMAIN_SIMULATION);
  header->mutable_sensor_frame()->set_value("camera_left_optical");
  header->mutable_calibration_version()->set_value("test_calibration_v1");
  header->set_validity(uw::domain::ObservationHeader::VALIDITY_OK);

  image.set_width(width);
  image.set_height(height);
  image.set_row_stride_bytes(width * 3);
  image.set_encoding(uw::domain::ImageFrame::IMAGE_ENCODING_RGB8);
  image.set_is_rectified(true);
  std::string pixels(static_cast<std::size_t>(image.row_stride_bytes()) * height, '\0');
  for (uint32_t y = 0; y < height; ++y) {
    for (uint32_t x = 0; x < width; ++x) {
      const std::size_t offset = static_cast<std::size_t>(y) * image.row_stride_bytes() + x * 3;
      pixels[offset] = static_cast<char>(fill[0]);
      pixels[offset + 1] = static_cast<char>(fill[1]);
      pixels[offset + 2] = static_cast<char>(fill[2]);
    }
  }
  image.set_pixel_data(std::move(pixels));
  return image;
}

uw::domain::ImageFrame MakeRgbImageWithRectangle(
    uint32_t width, uint32_t height, uint32_t x, uint32_t y, uint32_t rectangle_width,
    uint32_t rectangle_height, const std::array<uint8_t, 3>& color) {
  auto image = MakeRgbImage(width, height, {35, 35, 35});
  std::string pixels = image.pixel_data();
  for (uint32_t v = y; v < y + rectangle_height; ++v) {
    for (uint32_t u = x; u < x + rectangle_width; ++u) {
      const std::size_t offset = static_cast<std::size_t>(v) * image.row_stride_bytes() + u * 3;
      pixels[offset] = static_cast<char>(color[0]);
      pixels[offset + 1] = static_cast<char>(color[1]);
      pixels[offset + 2] = static_cast<char>(color[2]);
    }
  }
  image.set_pixel_data(std::move(pixels));
  return image;
}

void PaintRectangle(uw::domain::ImageFrame* image, uint32_t x, uint32_t y,
                    uint32_t rectangle_width, uint32_t rectangle_height,
                    const std::array<uint8_t, 3>& color) {
  std::string pixels = image->pixel_data();
  for (uint32_t v = y; v < y + rectangle_height; ++v) {
    for (uint32_t u = x; u < x + rectangle_width; ++u) {
      const std::size_t offset = static_cast<std::size_t>(v) * image->row_stride_bytes() + u * 3;
      pixels[offset] = static_cast<char>(color[0]);
      pixels[offset + 1] = static_cast<char>(color[1]);
      pixels[offset + 2] = static_cast<char>(color[2]);
    }
  }
  image->set_pixel_data(std::move(pixels));
}

void PaintRgbPixel(uw::domain::ImageFrame* image, int x, int y,
                   const std::array<uint8_t, 3>& color) {
  if (x < 0 || y < 0 || x >= static_cast<int>(image->width()) ||
      y >= static_cast<int>(image->height())) {
    return;
  }
  std::string* pixels = image->mutable_pixel_data();
  const std::size_t offset = static_cast<std::size_t>(y) * image->row_stride_bytes() +
                             static_cast<std::size_t>(x) * 3;
  (*pixels)[offset] = static_cast<char>(color[0]);
  (*pixels)[offset + 1] = static_cast<char>(color[1]);
  (*pixels)[offset + 2] = static_cast<char>(color[2]);
}

void PaintLine(uw::domain::ImageFrame* image, int x0, int y0, int x1, int y1,
               const std::array<uint8_t, 3>& color, int thickness) {
  const int dx = std::abs(x1 - x0);
  const int sx = x0 < x1 ? 1 : -1;
  const int dy = -std::abs(y1 - y0);
  const int sy = y0 < y1 ? 1 : -1;
  int error = dx + dy;
  while (true) {
    for (int oy = -thickness; oy <= thickness; ++oy) {
      for (int ox = -thickness; ox <= thickness; ++ox) PaintRgbPixel(image, x0 + ox, y0 + oy, color);
    }
    if (x0 == x1 && y0 == y1) break;
    const int twice_error = 2 * error;
    if (twice_error >= dy) {
      error += dy;
      x0 += sx;
    }
    if (twice_error <= dx) {
      error += dx;
      y0 += sy;
    }
  }
}

uw::domain::ImageFrame MakeSupportedStructureImage() {
  auto image = MakeRgbImage(320, 240, {45, 45, 45});
  PaintLine(&image, 125, 239, 165, 20, {230, 230, 230}, 1);
  PaintLine(&image, 245, 239, 205, 20, {230, 230, 230}, 1);
  return image;
}

uw::domain::ImageFrame MakeSingleStructureLineImage() {
  auto image = MakeRgbImage(320, 240, {45, 45, 45});
  PaintLine(&image, 125, 239, 165, 20, {230, 230, 230}, 1);
  return image;
}

uw::domain::ImageFrame MakeSmoothGradientImage() {
  auto image = MakeRgbImage(320, 240, {0, 0, 0});
  std::string pixels = image.pixel_data();
  for (uint32_t y = 0; y < image.height(); ++y) {
    for (uint32_t x = 0; x < image.width(); ++x) {
      const uint8_t value = static_cast<uint8_t>(30 + x / 2);
      const std::size_t offset = static_cast<std::size_t>(y) * image.row_stride_bytes() + x * 3;
      pixels[offset] = static_cast<char>(value);
      pixels[offset + 1] = static_cast<char>(value);
      pixels[offset + 2] = static_cast<char>(value);
    }
  }
  image.set_pixel_data(std::move(pixels));
  return image;
}

uw::domain::ImageFrame MakeUniformRgbImage() { return MakeRgbImage(320, 240, {80, 80, 80}); }

uw::domain::CameraIntrinsics TestCameraIntrinsics() {
  uw::domain::CameraIntrinsics intrinsics;
  intrinsics.mutable_sensor_id()->set_value("camera_left");
  intrinsics.set_width(320);
  intrinsics.set_height(240);
  for (double value : {240.0, 0.0, 160.0, 0.0, 240.0, 120.0, 0.0, 0.0, 1.0}) {
    intrinsics.add_k_matrix_row_major(value);
  }
  intrinsics.set_distortion_model("plumb_bob");
  return intrinsics;
}

uw::domain::OpticalDepthPriorMeasurement MakeEmptyMetricDepth(uint32_t width = 320,
                                                               uint32_t height = 240) {
  uw::domain::OpticalDepthPriorMeasurement depth;
  depth.mutable_reference_camera_frame()->set_value("camera_left_optical");
  depth.set_width(width);
  depth.set_height(height);
  depth.mutable_depth_m()->Reserve(static_cast<int>(width * height));
  depth.mutable_variance_m2()->Reserve(static_cast<int>(width * height));
  for (std::size_t i = 0; i < static_cast<std::size_t>(width) * height; ++i) {
    depth.add_depth_m(0.0f);
    depth.add_variance_m2(0.0f);
  }
  depth.set_valid_mask(std::string(static_cast<std::size_t>(width) * height, '\0'));
  depth.set_scale_status(uw::domain::OPTICAL_DEPTH_SCALE_STATUS_METRIC);
  depth.set_producer_type("test_stereo");
  return depth;
}

void SetDepthPixel(uw::domain::OpticalDepthPriorMeasurement* depth, uint32_t x, uint32_t y,
                   float value) {
  const int index = static_cast<int>(static_cast<std::size_t>(y) * depth->width() + x);
  depth->set_depth_m(index, value);
  depth->set_variance_m2(index, 0.01f);
  (*depth->mutable_valid_mask())[static_cast<std::size_t>(index)] = '\1';
}

uw::domain::OpticalDepthPriorMeasurement MakeDepthWithNineCentralSamples() {
  auto depth = MakeEmptyMetricDepth();
  const std::array<float, 9> values = {2.0f, 3.0f, 3.0f, 4.0f, 4.0f,
                                       5.0f, 5.0f, 6.0f, 8.0f};
  for (std::size_t i = 0; i < values.size(); ++i) {
    SetDepthPixel(&depth, 145 + static_cast<uint32_t>(i), 92, values[i]);
  }
  return depth;
}

VisualAssistParams TestVisualAssistParams() {
  VisualAssistParams params;
  params.hsv_hue_min = 45;
  params.hsv_hue_max = 85;
  params.hsv_saturation_min = 100;
  params.hsv_value_min = 80;
  params.minimum_component_area_px = 500;
  params.minimum_brightness = 10.0;
  params.minimum_contrast = 5.0;
  params.minimum_laplacian_variance = 1.0;
  params.minimum_texture_support = 0.0001;
  params.minimum_structure_line_count = 2;
  params.canny_low_threshold = 30.0;
  params.canny_high_threshold = 90.0;
  params.hough_vote_threshold = 20;
  params.hough_min_line_length_px = 50.0;
  params.hough_max_line_gap_px = 8.0;
  return params;
}

TEST(OpenCvVisualAssistFrontend, DetectsConfiguredAquacultureColor) {
  auto image = MakeRgbImageWithRectangle(320, 240, 130, 80, 60, 50, {20, 220, 20});
  OpenCvVisualAssistFrontend frontend(TestVisualAssistParams());
  const auto result = frontend.Process(image, std::nullopt, TestCameraIntrinsics());
  ASSERT_EQ(result.targets.size(), 1u);
  EXPECT_EQ(result.targets[0].class_label(), "aquaculture_zone");
  EXPECT_EQ(result.targets[0].source(), uw::domain::ASSIST_SOURCE_VISUAL);
}

TEST(OpenCvVisualAssistFrontend, RejectsStructureOffsetWhenLineSupportIsWeak) {
  OpenCvVisualAssistFrontend frontend(TestVisualAssistParams());
  const auto result = frontend.Process(MakeUniformRgbImage(), std::nullopt,
                                       TestCameraIntrinsics());
  EXPECT_FALSE(result.path_lateral_offset_m.has_value());
}

TEST(OpenCvVisualAssistFrontend, RejectsTwoCannyEdgesFromOnePhysicalStructureLine) {
  OpenCvVisualAssistFrontend frontend(TestVisualAssistParams());
  const auto result = frontend.Process(MakeSingleStructureLineImage(),
                                       MakeEmptyMetricDepth(), TestCameraIntrinsics());
  EXPECT_FALSE(result.path_lateral_offset_m.has_value());
  EXPECT_FALSE(result.path_offset_sigma_m.has_value());
}

TEST(OpenCvVisualAssistFrontend, ReportsExactVisualQualityReasons) {
  auto params = TestVisualAssistParams();
  params.hsv_value_min = 10;
  OpenCvVisualAssistFrontend frontend(params);

  auto low_light = MakeRgbImage(320, 240, {1, 1, 1});
  PaintRectangle(&low_light, 130, 80, 60, 50, {0, 20, 0});
  EXPECT_EQ(frontend.Process(low_light, std::nullopt, TestCameraIntrinsics())
                .health.reason_code(),
            "visual_low_light");

  const auto low_contrast = frontend.Process(MakeUniformRgbImage(), std::nullopt,
                                              TestCameraIntrinsics());
  EXPECT_EQ(low_contrast.health.reason_code(), "visual_low_contrast");
  EXPECT_TRUE(low_contrast.targets.empty());
  EXPECT_FALSE(low_contrast.path_lateral_offset_m.has_value());

  params.minimum_laplacian_variance = 50.0;
  params.minimum_texture_support = 0.0;
  OpenCvVisualAssistFrontend blur_frontend(params);
  EXPECT_EQ(blur_frontend.Process(MakeSmoothGradientImage(), std::nullopt,
                                  TestCameraIntrinsics())
                .health.reason_code(),
            "visual_blurred");

  params = TestVisualAssistParams();
  params.minimum_texture_support = 0.5;
  OpenCvVisualAssistFrontend texture_frontend(params);
  const auto textured_target =
      MakeRgbImageWithRectangle(320, 240, 130, 80, 60, 50, {20, 220, 20});
  EXPECT_EQ(texture_frontend.Process(textured_target, std::nullopt, TestCameraIntrinsics())
                .health.reason_code(),
            "visual_low_contrast");
}

TEST(OpenCvVisualAssistFrontend, KeepsMonocularTargetWhenStereoDepthIsUnavailable) {
  OpenCvVisualAssistFrontend frontend(TestVisualAssistParams());
  const auto image = MakeRgbImageWithRectangle(320, 240, 130, 80, 60, 50, {20, 220, 20});
  const auto result = frontend.Process(image, std::nullopt, TestCameraIntrinsics());

  ASSERT_EQ(result.targets.size(), 1u);
  EXPECT_FALSE(result.targets[0].has_range());
  EXPECT_DOUBLE_EQ(result.targets[0].range_m(), 0.0);
  EXPECT_EQ(result.health.component_id(), "sim_fixture_detector_v1");
  EXPECT_EQ(result.health.status(), uw::domain::HealthReport::STATUS_SUSPECT);
  EXPECT_EQ(result.health.reason_code(), "stereo_depth_unavailable");
  EXPECT_EQ(result.targets[0].quality_metrics().at("valid_depth_ratio"), 0.0);
}

TEST(OpenCvVisualAssistFrontend, CopiesCanonicalMetadataAndUsesBearingRangeCovariance) {
  OpenCvVisualAssistFrontend frontend(TestVisualAssistParams());
  const auto image = MakeRgbImageWithRectangle(320, 240, 130, 80, 60, 50, {20, 220, 20});
  const auto result = frontend.Process(image, std::nullopt, TestCameraIntrinsics());

  ASSERT_EQ(result.targets.size(), 1u);
  const auto& target = result.targets[0];
  EXPECT_EQ(target.source_observation().value(), image.header().observation_id().value());
  EXPECT_EQ(target.capture_time().SerializeAsString(),
            image.header().capture_time().SerializeAsString());
  EXPECT_TRUE(std::isfinite(target.confidence()));
  EXPECT_TRUE(std::isfinite(target.bearing_rad()));
  EXPECT_NEAR(target.bearing_rad(), std::atan2(159.5 - 160.0, 240.0), 1e-12);
  ASSERT_EQ(target.covariance_2x2_row_major_size(), 4);
  EXPECT_DOUBLE_EQ(target.covariance_2x2_row_major(0),
                   TestVisualAssistParams().bearing_sigma_rad *
                       TestVisualAssistParams().bearing_sigma_rad);
  EXPECT_DOUBLE_EQ(target.covariance_2x2_row_major(1), 0.0);
  EXPECT_DOUBLE_EQ(target.covariance_2x2_row_major(2), 0.0);
  for (double covariance : target.covariance_2x2_row_major()) EXPECT_TRUE(std::isfinite(covariance));
  EXPECT_FALSE(target.has_range());
  EXPECT_EQ(target.bbox_x(), 130u);
  EXPECT_EQ(target.bbox_y(), 80u);
  EXPECT_EQ(target.bbox_width(), 60u);
  EXPECT_EQ(target.bbox_height(), 50u);
  EXPECT_LE(target.bbox_x() + target.bbox_width(), image.width());
  EXPECT_LE(target.bbox_y() + target.bbox_height(), image.height());
  for (const char* metric : {"brightness", "contrast", "laplacian_blur_score",
                             "texture_support", "valid_depth_ratio"}) {
    ASSERT_NE(target.quality_metrics().find(metric), target.quality_metrics().end());
    EXPECT_TRUE(std::isfinite(target.quality_metrics().at(metric)));
  }
}

TEST(OpenCvVisualAssistFrontend, ComputesAngularExtentFromOffAxisBoundingBoxEdges) {
  OpenCvVisualAssistFrontend frontend(TestVisualAssistParams());
  const auto image =
      MakeRgbImageWithRectangle(320, 240, 240, 80, 60, 50, {20, 220, 20});
  const auto result = frontend.Process(image, std::nullopt, TestCameraIntrinsics());

  ASSERT_EQ(result.targets.size(), 1u);
  const auto& target = result.targets[0];
  const double expected_extent =
      std::atan2(300.0 - 160.0, 240.0) - std::atan2(240.0 - 160.0, 240.0);
  EXPECT_NEAR(target.angular_extent_rad(), expected_extent, 1e-12);
  EXPECT_TRUE(std::isfinite(target.angular_extent_rad()));
  EXPECT_GE(target.angular_extent_rad(), 0.0);
}

TEST(OpenCvVisualAssistFrontend, UsesCentralHalfRowMajorDepthMedianAndMad) {
  OpenCvVisualAssistFrontend frontend(TestVisualAssistParams());
  const auto image = MakeRgbImageWithRectangle(320, 240, 130, 80, 60, 50, {20, 220, 20});
  const auto result = frontend.Process(image, MakeDepthWithNineCentralSamples(),
                                       TestCameraIntrinsics());

  ASSERT_EQ(result.targets.size(), 1u);
  const auto& target = result.targets[0];
  EXPECT_TRUE(target.has_range());
  EXPECT_DOUBLE_EQ(target.range_m(), 4.0);
  ASSERT_EQ(target.covariance_2x2_row_major_size(), 4);
  EXPECT_NEAR(target.covariance_2x2_row_major(3), 1.4826 * 1.4826, 1e-6);
  EXPECT_NEAR(target.quality_metrics().at("valid_depth_ratio"), 9.0 / 750.0, 1e-12);
  EXPECT_EQ(result.health.status(), uw::domain::HealthReport::STATUS_HEALTHY);
  EXPECT_TRUE(result.health.reason_code().empty());
}

TEST(OpenCvVisualAssistFrontend, IgnoresNonPositiveAndNonFiniteDepthSamples) {
  auto depth = MakeDepthWithNineCentralSamples();
  SetDepthPixel(&depth, 145, 93, 0.0f);
  SetDepthPixel(&depth, 146, 93, -1.0f);
  SetDepthPixel(&depth, 147, 93, std::numeric_limits<float>::quiet_NaN());
  SetDepthPixel(&depth, 148, 93, std::numeric_limits<float>::infinity());
  SetDepthPixel(&depth, 149, 93, 4.0f);
  const int invalid_variance_index = 93 * static_cast<int>(depth.width()) + 149;
  depth.set_variance_m2(invalid_variance_index, std::numeric_limits<float>::quiet_NaN());
  OpenCvVisualAssistFrontend frontend(TestVisualAssistParams());
  const auto image = MakeRgbImageWithRectangle(320, 240, 130, 80, 60, 50, {20, 220, 20});
  const auto result = frontend.Process(image, depth, TestCameraIntrinsics());

  ASSERT_EQ(result.targets.size(), 1u);
  EXPECT_TRUE(result.targets[0].has_range());
  EXPECT_DOUBLE_EQ(result.targets[0].range_m(), 4.0);
  EXPECT_NEAR(result.targets[0].quality_metrics().at("valid_depth_ratio"), 9.0 / 750.0,
              1e-12);
}

TEST(OpenCvVisualAssistFrontend, RequiresNineUsableCentralDepthSamples) {
  auto depth = MakeEmptyMetricDepth();
  for (uint32_t x = 145; x < 153; ++x) SetDepthPixel(&depth, x, 92, 4.0f);
  for (uint32_t x = 130; x < 150; ++x) SetDepthPixel(&depth, x, 80, 9.0f);
  OpenCvVisualAssistFrontend frontend(TestVisualAssistParams());
  const auto image = MakeRgbImageWithRectangle(320, 240, 130, 80, 60, 50, {20, 220, 20});
  const auto result = frontend.Process(image, depth, TestCameraIntrinsics());

  ASSERT_EQ(result.targets.size(), 1u);
  EXPECT_FALSE(result.targets[0].has_range());
  EXPECT_EQ(result.health.reason_code(), "stereo_depth_unavailable");
}

TEST(OpenCvVisualAssistFrontend, UsesPositiveSigmaFloorWhenDepthMadIsZero) {
  auto depth = MakeEmptyMetricDepth();
  for (uint32_t x = 145; x < 154; ++x) SetDepthPixel(&depth, x, 92, 4.0f);
  OpenCvVisualAssistFrontend frontend(TestVisualAssistParams());
  const auto image = MakeRgbImageWithRectangle(320, 240, 130, 80, 60, 50, {20, 220, 20});
  const auto result = frontend.Process(image, depth, TestCameraIntrinsics());

  ASSERT_EQ(result.targets.size(), 1u);
  ASSERT_TRUE(result.targets[0].has_range());
  EXPECT_GT(result.targets[0].covariance_2x2_row_major(3), 0.0);
  EXPECT_TRUE(std::isfinite(result.targets[0].covariance_2x2_row_major(3)));
}

TEST(OpenCvVisualAssistFrontend, FailsClosedForMalformedInputs) {
  OpenCvVisualAssistFrontend frontend(TestVisualAssistParams());
  const auto valid_image = MakeRgbImageWithRectangle(320, 240, 130, 80, 60, 50, {20, 220, 20});

  std::vector<uw::domain::ImageFrame> malformed_images;
  auto bad_payload = valid_image;
  bad_payload.mutable_pixel_data()->pop_back();
  malformed_images.push_back(bad_payload);
  auto bad_header = valid_image;
  bad_header.mutable_header()->mutable_observation_id()->clear_value();
  malformed_images.push_back(bad_header);
  auto mono = valid_image;
  mono.set_encoding(uw::domain::ImageFrame::IMAGE_ENCODING_MONO8);
  mono.set_row_stride_bytes(mono.width());
  mono.set_pixel_data(std::string(static_cast<std::size_t>(mono.width()) * mono.height(), '\0'));
  malformed_images.push_back(mono);
  auto unrectified = valid_image;
  unrectified.set_is_rectified(false);
  malformed_images.push_back(unrectified);

  for (const auto& image : malformed_images) {
    uw::measurement_api::VisualAssistResult result;
    EXPECT_NO_THROW(result = frontend.Process(image, std::nullopt, TestCameraIntrinsics()));
    EXPECT_TRUE(result.targets.empty());
    EXPECT_FALSE(result.path_lateral_offset_m.has_value());
    EXPECT_EQ(result.health.status(), uw::domain::HealthReport::STATUS_UNAVAILABLE);
  }

  for (double bad_fx : {0.0, std::numeric_limits<double>::quiet_NaN(),
                        std::numeric_limits<double>::infinity()}) {
    auto intrinsics = TestCameraIntrinsics();
    intrinsics.set_k_matrix_row_major(0, bad_fx);
    const auto result = frontend.Process(valid_image, std::nullopt, intrinsics);
    EXPECT_TRUE(result.targets.empty());
    EXPECT_EQ(result.health.status(), uw::domain::HealthReport::STATUS_UNAVAILABLE);
  }

  auto nonfinite_distortion = TestCameraIntrinsics();
  nonfinite_distortion.add_distortion(std::numeric_limits<double>::quiet_NaN());
  const auto distortion_result =
      frontend.Process(valid_image, std::nullopt, nonfinite_distortion);
  EXPECT_TRUE(distortion_result.targets.empty());
  EXPECT_EQ(distortion_result.health.status(), uw::domain::HealthReport::STATUS_UNAVAILABLE);

  auto bad_depth = MakeDepthWithNineCentralSamples();
  bad_depth.set_width(319);
  const auto depth_result = frontend.Process(valid_image, bad_depth, TestCameraIntrinsics());
  ASSERT_EQ(depth_result.targets.size(), 1u);
  EXPECT_FALSE(depth_result.targets[0].has_range());
  EXPECT_EQ(depth_result.health.reason_code(), "stereo_depth_unavailable");
}

TEST(OpenCvVisualAssistFrontend, ConvertsCanonicalBgrInputConsistently) {
  auto image = MakeRgbImageWithRectangle(320, 240, 130, 80, 60, 50, {20, 220, 20});
  std::string pixels = image.pixel_data();
  for (std::size_t offset = 0; offset < pixels.size(); offset += 3) {
    std::swap(pixels[offset], pixels[offset + 2]);
  }
  image.set_encoding(uw::domain::ImageFrame::IMAGE_ENCODING_BGR8);
  image.set_pixel_data(std::move(pixels));

  OpenCvVisualAssistFrontend frontend(TestVisualAssistParams());
  const auto result = frontend.Process(image, std::nullopt, TestCameraIntrinsics());
  ASSERT_EQ(result.targets.size(), 1u);
  EXPECT_EQ(result.targets[0].bbox_x(), 130u);
}

TEST(OpenCvVisualAssistFrontend, OrdersEqualComponentsByBoundingBoxPosition) {
  auto image = MakeRgbImage(320, 240, {35, 35, 35});
  PaintRectangle(&image, 220, 20, 30, 30, {20, 220, 20});
  PaintRectangle(&image, 40, 140, 30, 30, {20, 220, 20});
  auto params = TestVisualAssistParams();
  params.minimum_component_area_px = 400;
  OpenCvVisualAssistFrontend frontend(params);
  const auto result = frontend.Process(image, std::nullopt, TestCameraIntrinsics());

  ASSERT_EQ(result.targets.size(), 2u);
  EXPECT_EQ(result.targets[0].bbox_x(), 40u);
  EXPECT_EQ(result.targets[1].bbox_x(), 220u);
}

TEST(OpenCvVisualAssistFrontend, EmitsStructureOffsetOnlyWithAdequateLineSupport) {
  OpenCvVisualAssistFrontend frontend(TestVisualAssistParams());
  const auto result = frontend.Process(MakeSupportedStructureImage(),
                                       MakeEmptyMetricDepth(), TestCameraIntrinsics());

  ASSERT_TRUE(result.path_lateral_offset_m.has_value());
  ASSERT_TRUE(result.path_offset_sigma_m.has_value());
  EXPECT_GT(*result.path_lateral_offset_m, 0.2);
  EXPECT_LT(*result.path_lateral_offset_m, 0.7);
  EXPECT_GT(*result.path_offset_sigma_m, 0.0);
}

TEST(OpenCvVisualAssistFrontend, EmitsNoPathPairWhenFinitePathArithmeticOverflows) {
  auto params = TestVisualAssistParams();
  params.structure_reference_range_m = std::numeric_limits<double>::max();
  OpenCvVisualAssistFrontend frontend(params);
  auto intrinsics = TestCameraIntrinsics();
  intrinsics.set_k_matrix_row_major(0, std::numeric_limits<double>::min());
  auto depth = MakeEmptyMetricDepth();
  SetDepthPixel(&depth, 0, 0, 4.0f);

  const auto result =
      frontend.Process(MakeSupportedStructureImage(), depth, intrinsics);

  EXPECT_FALSE(result.path_lateral_offset_m.has_value());
  EXPECT_FALSE(result.path_offset_sigma_m.has_value());
  EXPECT_EQ(result.health.status(), uw::domain::HealthReport::STATUS_HEALTHY);
  EXPECT_TRUE(result.health.reason_code().empty());
}

TEST(OpenCvVisualAssistFrontend, ReportsUnavailableDepthWithoutTargetsWhenFrameHasNoUsableSamples) {
  OpenCvVisualAssistFrontend frontend(TestVisualAssistParams());
  const auto result = frontend.Process(MakeSupportedStructureImage(),
                                       MakeEmptyMetricDepth(), TestCameraIntrinsics());

  EXPECT_TRUE(result.targets.empty());
  EXPECT_TRUE(result.path_lateral_offset_m.has_value());
  EXPECT_TRUE(result.path_offset_sigma_m.has_value());
  EXPECT_EQ(result.health.status(), uw::domain::HealthReport::STATUS_SUSPECT);
  EXPECT_EQ(result.health.reason_code(), "stereo_depth_unavailable");
}

TEST(OpenCvVisualAssistFrontend, RecoversDeterministicallyAfterVisualQualityFailure) {
  OpenCvVisualAssistFrontend frontend(TestVisualAssistParams());
  EXPECT_EQ(frontend.Process(MakeRgbImage(320, 240, {1, 1, 1}), std::nullopt,
                             TestCameraIntrinsics())
                .health.reason_code(),
            "visual_low_light");

  const auto valid_image = MakeRgbImageWithRectangle(320, 240, 130, 80, 60, 50,
                                                      {20, 220, 20});
  const auto recovered = frontend.Process(valid_image, MakeDepthWithNineCentralSamples(),
                                          TestCameraIntrinsics());
  EXPECT_EQ(recovered.health.status(), uw::domain::HealthReport::STATUS_HEALTHY);
  EXPECT_TRUE(recovered.health.reason_code().empty());
}

TEST(OpenCvVisualAssistFrontend, RejectsInvalidParamsAtConstruction) {
  auto params = TestVisualAssistParams();
  params.minimum_brightness = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(OpenCvVisualAssistFrontend frontend(params), std::invalid_argument);

  params = TestVisualAssistParams();
  params.hsv_hue_max = 180;
  EXPECT_THROW(OpenCvVisualAssistFrontend frontend(params), std::invalid_argument);

  params = TestVisualAssistParams();
  params.canny_high_threshold = params.canny_low_threshold;
  EXPECT_THROW(OpenCvVisualAssistFrontend frontend(params), std::invalid_argument);
}

TEST(OpenCvVisualAssistFrontend, RejectsParamsWhoseFixedCovarianceDerivationsOverflow) {
  auto params = TestVisualAssistParams();
  params.bearing_sigma_rad = std::numeric_limits<double>::max();
  EXPECT_THROW(OpenCvVisualAssistFrontend frontend(params), std::invalid_argument);

  params = TestVisualAssistParams();
  params.minimum_range_sigma_m = std::numeric_limits<double>::max();
  EXPECT_THROW(OpenCvVisualAssistFrontend frontend(params), std::invalid_argument);
}

TEST(OpenCvVisualAssistFrontend, FailsRangeClosedWhenDepthUncertaintyDerivationOverflows) {
  auto params = TestVisualAssistParams();
  params.depth_mad_scale = std::numeric_limits<double>::max();
  OpenCvVisualAssistFrontend frontend(params);
  const auto image =
      MakeRgbImageWithRectangle(320, 240, 130, 80, 60, 50, {20, 220, 20});
  const auto result = frontend.Process(image, MakeDepthWithNineCentralSamples(),
                                       TestCameraIntrinsics());

  ASSERT_EQ(result.targets.size(), 1u);
  const auto& target = result.targets[0];
  EXPECT_FALSE(target.has_range());
  EXPECT_DOUBLE_EQ(target.range_m(), 0.0);
  ASSERT_EQ(target.covariance_2x2_row_major_size(), 4);
  for (double covariance : target.covariance_2x2_row_major()) {
    EXPECT_TRUE(std::isfinite(covariance));
  }
  EXPECT_EQ(result.health.status(), uw::domain::HealthReport::STATUS_SUSPECT);
  EXPECT_EQ(result.health.reason_code(), "stereo_depth_unavailable");
}

}  // namespace
