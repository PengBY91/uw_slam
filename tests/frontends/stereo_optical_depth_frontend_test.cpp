#include <cmath>
#include <cstdint>
#include <string>

#include <gtest/gtest.h>

#include "frontends/stereo_optical_depth_frontend.hpp"

namespace {

uint8_t Texture(int u, int v) { return static_cast<uint8_t>((u * 31 + v * 17 + 7) % 256); }

uw::domain::RigCalibrationSnapshot MakeRig(double fx, double baseline_half) {
  uw::domain::RigCalibrationSnapshot rig;
  auto add_camera = [&](const std::string& id, uint32_t w, uint32_t h) {
    auto* camera = rig.add_cameras();
    camera->mutable_sensor_id()->set_value(id);
    camera->set_width(w);
    camera->set_height(h);
    for (double v : {fx, 0.0, 0.0, 0.0, fx, 0.0, 0.0, 0.0, 1.0}) camera->add_k_matrix_row_major(v);
  };
  add_camera("camera_left", 20, 3);
  add_camera("camera_right", 20, 3);
  auto add_edge = [&](const std::string& child, double y) {
    auto* edge = rig.add_frame_tree();
    edge->mutable_parent_frame()->set_value("base_link");
    edge->mutable_child_frame()->set_value(child);
    for (double v : {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, y, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0}) {
      edge->mutable_transform()->add_matrix_row_major(v);
    }
  };
  add_edge("camera_left_link", baseline_half);
  add_edge("camera_right_link", -baseline_half);
  return rig;
}

uw::domain::ImageFrame MakeImage(const std::string& frame, int width, int height, int shift) {
  uw::domain::ImageFrame image;
  image.mutable_header()->mutable_sensor_frame()->set_value(frame);
  image.set_width(width);
  image.set_height(height);
  image.set_row_stride_bytes(width);
  image.set_encoding(uw::domain::ImageFrame::IMAGE_ENCODING_MONO8);
  std::string pixels(static_cast<std::size_t>(width) * height, '\0');
  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      pixels[static_cast<std::size_t>(v) * width + u] = static_cast<char>(Texture(u + shift, v));
    }
  }
  image.set_pixel_data(pixels);
  return image;
}

}  // namespace

TEST(StereoOpticalDepthFrontend, RecoversMetricDepthFromConstantDisparity) {
  // fx=100, baseline=0.5m, true disparity=4px -> depth = fx*baseline/d = 12.5m.
  const auto rig = MakeRig(/*fx=*/100.0, /*baseline_half=*/0.25);
  uw::measurement_api::CameraFrameBundle bundle;
  bundle.primary = MakeImage("camera_left_link", 20, 3, /*shift=*/0);
  bundle.secondary = MakeImage("camera_right_link", 20, 3, /*shift=*/4);

  uw::frontends::StereoOpticalDepthFrontendParams params;
  params.left_sensor_id = "camera_left";
  params.left_frame = "camera_left_link";
  params.right_sensor_id = "camera_right";
  params.right_frame = "camera_right_link";
  params.matcher.window_radius = 1;
  params.matcher.min_disparity = 1;
  params.matcher.max_disparity = 6;
  uw::frontends::StereoOpticalDepthFrontend frontend(params);

  const auto evidence = frontend.Process(bundle, rig);
  ASSERT_TRUE(evidence.has_value());
  const auto& prior = uw::domain::GetPayload<uw::domain::OpticalDepthPriorMeasurement>(*evidence);
  EXPECT_EQ(prior.scale_status(), uw::domain::OPTICAL_DEPTH_SCALE_STATUS_METRIC);
  EXPECT_EQ(prior.producer_type(), "stereo");
  ASSERT_EQ(uw::domain::ValidateOpticalDepthPrior(prior).code, uw::domain::ValidationCode::kOk);

  const int u = 10, v = 1;  // inside the matcher's valid region for this fixture
  const std::size_t idx = static_cast<std::size_t>(v) * 20 + u;
  ASSERT_EQ(static_cast<unsigned char>(prior.valid_mask()[idx]), 1);
  EXPECT_NEAR(prior.depth_m(idx), 12.5, 1e-6);
  const double expected_variance = std::pow(12.5 * 12.5 / (100.0 * 0.5) * params.disparity_sigma_px, 2);
  EXPECT_NEAR(prior.variance_m2(idx), expected_variance, 1e-6);
}

TEST(StereoOpticalDepthFrontend, RejectsBundleWithoutSecondaryFrame) {
  const auto rig = MakeRig(100.0, 0.25);
  uw::measurement_api::CameraFrameBundle bundle;
  bundle.primary = MakeImage("camera_left_link", 20, 3, 0);

  uw::frontends::StereoOpticalDepthFrontendParams params;
  params.left_sensor_id = "camera_left";
  params.left_frame = "camera_left_link";
  params.right_sensor_id = "camera_right";
  params.right_frame = "camera_right_link";
  uw::frontends::StereoOpticalDepthFrontend frontend(params);

  EXPECT_FALSE(frontend.Process(bundle, rig).has_value());
}

TEST(StereoOpticalDepthFrontend, RejectsUnresolvableRigGeometry) {
  uw::domain::RigCalibrationSnapshot empty_rig;
  uw::measurement_api::CameraFrameBundle bundle;
  bundle.primary = MakeImage("camera_left_link", 20, 3, 0);
  bundle.secondary = MakeImage("camera_right_link", 20, 3, 4);

  uw::frontends::StereoOpticalDepthFrontendParams params;
  params.left_sensor_id = "camera_left";
  params.left_frame = "camera_left_link";
  params.right_sensor_id = "camera_right";
  params.right_frame = "camera_right_link";
  uw::frontends::StereoOpticalDepthFrontend frontend(params);

  EXPECT_FALSE(frontend.Process(bundle, empty_rig).has_value());
}
