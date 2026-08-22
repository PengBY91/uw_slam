#include "sensor_models/camera_rectifier.hpp"

#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

using uw::sensor_models::ApplyPlumbBobDistortion;
using uw::sensor_models::PlumbBobDistortion;
using uw::sensor_models::UndistortImage;

namespace {

uw::domain::CameraIntrinsics MakeIntrinsics(uint32_t width, uint32_t height, double fx, double fy, double cx,
                                             double cy, std::vector<double> distortion,
                                             const std::string& distortion_model = "plumb_bob") {
  uw::domain::CameraIntrinsics intrinsics;
  intrinsics.set_width(width);
  intrinsics.set_height(height);
  for (double v : {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0}) intrinsics.add_k_matrix_row_major(v);
  for (double v : distortion) intrinsics.add_distortion(v);
  intrinsics.set_distortion_model(distortion_model);
  return intrinsics;
}

}  // namespace

TEST(PlumbBobDistortionTest, FromIntrinsicsAcceptsZeroFourAndFiveCoefficients) {
  const auto zero = PlumbBobDistortion::FromIntrinsics(MakeIntrinsics(64, 64, 100, 100, 32, 32, {}));
  ASSERT_TRUE(zero.has_value());
  EXPECT_TRUE(zero->IsIdentity());

  const auto four =
      PlumbBobDistortion::FromIntrinsics(MakeIntrinsics(64, 64, 100, 100, 32, 32, {0.1, -0.05, 0.001, -0.002}));
  ASSERT_TRUE(four.has_value());
  EXPECT_DOUBLE_EQ(four->k1, 0.1);
  EXPECT_DOUBLE_EQ(four->k2, -0.05);
  EXPECT_DOUBLE_EQ(four->p1, 0.001);
  EXPECT_DOUBLE_EQ(four->p2, -0.002);
  EXPECT_DOUBLE_EQ(four->k3, 0.0);
  EXPECT_FALSE(four->IsIdentity());

  const auto five = PlumbBobDistortion::FromIntrinsics(
      MakeIntrinsics(64, 64, 100, 100, 32, 32, {0.017961, -0.042131, -0.000550, 0.001529, 0.023008}));
  ASSERT_TRUE(five.has_value());
  EXPECT_DOUBLE_EQ(five->k3, 0.023008);
}

TEST(PlumbBobDistortionTest, FromIntrinsicsRejectsUnsupportedLengthOrModel) {
  EXPECT_EQ(PlumbBobDistortion::FromIntrinsics(MakeIntrinsics(64, 64, 100, 100, 32, 32, {0.1, 0.2})), std::nullopt);
  EXPECT_EQ(PlumbBobDistortion::FromIntrinsics(
                MakeIntrinsics(64, 64, 100, 100, 32, 32, {0.1, 0.2, 0.3, 0.4}, "fisheye")),
            std::nullopt);
  // Empty distortion_model string (proto3 default) is treated as plumb_bob.
  EXPECT_NE(PlumbBobDistortion::FromIntrinsics(MakeIntrinsics(64, 64, 100, 100, 32, 32, {}, "")), std::nullopt);
}

TEST(PlumbBobDistortionTest, ApplyIsIdentityForZeroCoefficients) {
  const PlumbBobDistortion identity;
  const Eigen::Vector2d p(0.3, -0.2);
  const Eigen::Vector2d out = ApplyPlumbBobDistortion(identity, p);
  EXPECT_NEAR(out.x(), p.x(), 1e-12);
  EXPECT_NEAR(out.y(), p.y(), 1e-12);
}

TEST(PlumbBobDistortionTest, ApplyMatchesHandComputedRadialValue) {
  // Pure radial (p1=p2=k3=0): x_d = x * (1 + k1*r2 + k2*r2^2).
  PlumbBobDistortion d;
  d.k1 = 0.1;
  d.k2 = 0.02;
  const Eigen::Vector2d p(0.4, 0.0);
  const double r2 = 0.4 * 0.4;
  const double expected_x = 0.4 * (1.0 + 0.1 * r2 + 0.02 * r2 * r2);
  const Eigen::Vector2d out = ApplyPlumbBobDistortion(d, p);
  EXPECT_NEAR(out.x(), expected_x, 1e-12);
  EXPECT_NEAR(out.y(), 0.0, 1e-12);
}

namespace {

// A MONO8 frame whose pixel value is a known LINEAR function of the pixel's
// normalized-x coordinate under `intrinsics`: value(u, v) = round(128 + 80 *
// (u - cx) / fx). Because this is defined directly in normalized-camera
// space (not warped through any distortion), UndistortImage's output at
// pixel (u, v) is independently predictable as round(128 + 80 *
// ApplyPlumbBobDistortion(distortion, normalize(u,v)).x()) — letting the
// test check the actual image-warping code against the same closed-form
// formula ApplyPlumbBobDistortion is already tested against above, without
// needing to invert the distortion polynomial or depend on OpenCV as an
// oracle.
uw::domain::ImageFrame MakeLinearRampFrame(uint32_t width, uint32_t height, double fx, double cx) {
  uw::domain::ImageFrame frame;
  frame.set_width(width);
  frame.set_height(height);
  frame.set_row_stride_bytes(width);
  frame.set_encoding(uw::domain::ImageFrame::IMAGE_ENCODING_MONO8);
  std::string pixels(static_cast<std::size_t>(width) * height, '\0');
  for (uint32_t v = 0; v < height; ++v) {
    for (uint32_t u = 0; u < width; ++u) {
      const double x_norm = (static_cast<double>(u) - cx) / fx;
      const double value = 128.0 + 80.0 * x_norm;
      pixels[static_cast<std::size_t>(v) * width + u] = static_cast<char>(static_cast<unsigned char>(std::lround(value)));
    }
  }
  frame.set_pixel_data(pixels);
  return frame;
}

}  // namespace

TEST(UndistortImageTest, MatchesClosedFormForLinearRampUnderRadialDistortion) {
  constexpr uint32_t kSize = 100;
  constexpr double kFx = 100.0, kFy = 100.0, kCx = 50.0, kCy = 50.0;
  const auto intrinsics = MakeIntrinsics(kSize, kSize, kFx, kFy, kCx, kCy, {0.05, 0.0, 0.0, 0.0});
  const auto distortion = PlumbBobDistortion::FromIntrinsics(intrinsics);
  ASSERT_TRUE(distortion.has_value());

  const auto raw = MakeLinearRampFrame(kSize, kSize, kFx, kCx);
  const auto result = UndistortImage(raw, intrinsics);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->is_rectified());
  ASSERT_EQ(result->width(), kSize);
  ASSERT_EQ(result->height(), kSize);

  // Check an interior region only, well away from the border, so the
  // forward-distorted source lookup never runs off the source image.
  for (uint32_t v = 20; v < 80; v += 5) {
    for (uint32_t u = 20; u < 80; u += 5) {
      const double x_norm = (static_cast<double>(u) - kCx) / kFx;
      const double y_norm = (static_cast<double>(v) - kCy) / kFy;
      const Eigen::Vector2d distorted = ApplyPlumbBobDistortion(*distortion, Eigen::Vector2d(x_norm, y_norm));
      const double expected = 128.0 + 80.0 * distorted.x();
      const unsigned char actual =
          static_cast<unsigned char>(result->pixel_data()[static_cast<std::size_t>(v) * kSize + u]);
      EXPECT_NEAR(static_cast<double>(actual), expected, 2.0) << "u=" << u << " v=" << v;
    }
  }
}

TEST(UndistortImageTest, IdentityDistortionReturnsFrameUnchanged) {
  const auto intrinsics = MakeIntrinsics(100, 100, 100.0, 100.0, 50.0, 50.0, {0.0, 0.0, 0.0, 0.0});
  auto raw = MakeLinearRampFrame(100, 100, 100.0, 50.0);
  raw.set_is_rectified(false);
  const auto result = UndistortImage(raw, intrinsics);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->pixel_data(), raw.pixel_data());
  EXPECT_FALSE(result->is_rectified());  // unchanged — nothing was actually rectified
}

TEST(UndistortImageTest, RejectsDimensionMismatch) {
  const auto intrinsics = MakeIntrinsics(64, 64, 100.0, 100.0, 32.0, 32.0, {0.1, 0.0, 0.0, 0.0});
  const auto raw = MakeLinearRampFrame(100, 100, 100.0, 50.0);  // 100x100, intrinsics say 64x64
  EXPECT_EQ(UndistortImage(raw, intrinsics), std::nullopt);
}

TEST(UndistortImageTest, RejectsInvalidFrame) {
  const auto intrinsics = MakeIntrinsics(100, 100, 100.0, 100.0, 50.0, 50.0, {0.1, 0.0, 0.0, 0.0});
  uw::domain::ImageFrame raw;  // width/height default to 0 -> fails ValidateImageFrame
  EXPECT_EQ(UndistortImage(raw, intrinsics), std::nullopt);
}

TEST(UndistortImageTest, RejectsUnsupportedDistortionModel) {
  const auto intrinsics =
      MakeIntrinsics(100, 100, 100.0, 100.0, 50.0, 50.0, {0.1, 0.0, 0.0, 0.0}, "fisheye");
  const auto raw = MakeLinearRampFrame(100, 100, 100.0, 50.0);
  EXPECT_EQ(UndistortImage(raw, intrinsics), std::nullopt);
}
