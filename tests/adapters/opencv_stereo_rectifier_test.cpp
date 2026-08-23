#include <cmath>
#include <limits>
#include <string>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "opencv_adapters/stereo_rectifier.hpp"
#include "sensor_models/camera_model.hpp"
#include "sensor_models/camera_rectifier.hpp"
#include "sensor_models/geometry.hpp"

namespace {

using uw::opencv_adapters::RectificationCropPolicy;
using uw::opencv_adapters::StereoRectificationContext;
using uw::opencv_adapters::StereoRectificationParams;

void AddCamera(uw::domain::RigCalibrationSnapshot& rig, const std::string& id, uint32_t w,
               uint32_t h, double fx, double fy, double cx, double cy,
               const std::vector<double>& distortion = {},
               const std::string& model = "plumb_bob") {
  auto* camera = rig.add_cameras();
  camera->mutable_sensor_id()->set_value(id);
  camera->set_width(w);
  camera->set_height(h);
  for (double v : {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0}) camera->add_k_matrix_row_major(v);
  for (double v : distortion) camera->add_distortion(v);
  camera->set_distortion_model(model);
}

void AddEdge(uw::domain::RigCalibrationSnapshot& rig, const std::string& child, double x,
             double y, double z) {
  auto* edge = rig.add_frame_tree();
  edge->mutable_parent_frame()->set_value("base_link");
  edge->mutable_child_frame()->set_value(child);
  for (double v : {1.0, 0.0, 0.0, x, 0.0, 1.0, 0.0, y, 0.0, 0.0, 1.0, z, 0.0, 0.0, 0.0, 1.0}) {
    edge->mutable_transform()->add_matrix_row_major(v);
  }
}

// A rig that already satisfies the identity fast path: zero distortion,
// identical left/right K, identical camera_link orientation, baseline purely
// along the body x/optical-x-equivalent axis (see AddEdge's y-only offset,
// matching camera_model_test.cpp's ResolvesBaselineForParallelRig fixture).
uw::domain::RigCalibrationSnapshot MakeIdentityEligibleRig() {
  uw::domain::RigCalibrationSnapshot rig;
  AddCamera(rig, "camera_left", 640, 480, 420.0, 420.0, 320.0, 240.0);
  AddCamera(rig, "camera_right", 640, 480, 420.0, 420.0, 320.0, 240.0);
  AddEdge(rig, "camera_left_link", 0.15, 0.06, 0.0);
  AddEdge(rig, "camera_right_link", 0.15, -0.06, 0.0);
  rig.mutable_calibration_version()->set_value("raw_v1");
  return rig;
}

StereoRectificationParams DefaultParams() {
  StereoRectificationParams params;
  params.left_sensor_id = "camera_left";
  params.left_frame = "camera_left_link";
  params.right_sensor_id = "camera_right";
  params.right_frame = "camera_right_link";
  return params;
}

uw::domain::ImageFrame MakeMono8Image(const std::string& frame, const std::string& sensor_id,
                                       uint32_t width, uint32_t height, uint8_t fill) {
  uw::domain::ImageFrame image;
  image.mutable_header()->mutable_sensor_frame()->set_value(frame);
  image.mutable_header()->mutable_sensor_id()->set_value(sensor_id);
  image.mutable_header()->mutable_observation_id()->set_value(sensor_id + "_obs_1");
  image.mutable_header()->mutable_capture_time()->set_seconds(10);
  image.mutable_header()->mutable_capture_time()->set_nanos(500);
  image.mutable_header()->mutable_receive_time()->set_seconds(10);
  image.mutable_header()->mutable_receive_time()->set_nanos(600);
  image.set_width(width);
  image.set_height(height);
  image.set_row_stride_bytes(width);
  image.set_encoding(uw::domain::ImageFrame::IMAGE_ENCODING_MONO8);
  image.set_pixel_data(std::string(static_cast<std::size_t>(width) * height,
                                    static_cast<char>(fill)));
  return image;
}

void AddEdgeRotated(uw::domain::RigCalibrationSnapshot& rig, const std::string& child,
                     const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation) {
  auto* edge = rig.add_frame_tree();
  edge->mutable_parent_frame()->set_value("base_link");
  edge->mutable_child_frame()->set_value(child);
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
  m.topLeftCorner<3, 3>() = rotation;
  m.topRightCorner<3, 1>() = translation;
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) edge->mutable_transform()->add_matrix_row_major(m(row, col));
  }
}

uw::sensor_models::Pose3 EdgeToPose(const uw::domain::RigCalibrationSnapshot& rig,
                                     const std::string& child) {
  for (const auto& edge : rig.frame_tree()) {
    if (edge.child_frame().value() == child) return uw::sensor_models::Pose3::FromProto(edge.transform());
  }
  return uw::sensor_models::Pose3::Identity();
}

struct GeneralRigFixture {
  uw::domain::RigCalibrationSnapshot rig;
  uw::sensor_models::PlumbBobDistortion left_distortion;
  uw::sensor_models::PlumbBobDistortion right_distortion;
  uw::sensor_models::PinholeCamera left_camera;
  uw::sensor_models::PinholeCamera right_camera;
};

// Non-parallel rig: different K per camera, nonzero 5-param plumb-bob
// distortion, and a right camera with yaw+pitch plus small forward/vertical
// translation on top of the lateral baseline.
GeneralRigFixture MakeGeneralRig() {
  GeneralRigFixture fixture;
  auto& rig = fixture.rig;
  AddCamera(rig, "camera_left", 640, 480, 500.0, 500.0, 320.0, 240.0,
            {-0.08, 0.02, 0.001, -0.0007, 0.0005});
  AddCamera(rig, "camera_right", 640, 480, 520.0, 510.0, 330.0, 250.0,
            {-0.06, 0.015, 0.0008, -0.0005, 0.0003});
  AddEdgeRotated(rig, "camera_left_link", Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, 0.06, 0.0));
  const Eigen::Matrix3d right_rotation =
      (Eigen::AngleAxisd(3.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(2.0 * M_PI / 180.0, Eigen::Vector3d::UnitY()))
          .toRotationMatrix();
  AddEdgeRotated(rig, "camera_right_link", right_rotation, Eigen::Vector3d(0.02, -0.06, 0.01));
  rig.mutable_calibration_version()->set_value("raw_general_v1");

  fixture.left_camera = uw::sensor_models::PinholeCamera::FromIntrinsics(rig.cameras(0));
  fixture.right_camera = uw::sensor_models::PinholeCamera::FromIntrinsics(rig.cameras(1));
  fixture.left_distortion = *uw::sensor_models::PlumbBobDistortion::FromIntrinsics(rig.cameras(0));
  fixture.right_distortion = *uw::sensor_models::PlumbBobDistortion::FromIntrinsics(rig.cameras(1));
  return fixture;
}

// Forward-projects a known world (base_link-frame) point through a raw,
// distorted camera model -- the inverse of what StereoRectificationContext
// is supposed to undo.
Eigen::Vector2d ProjectToRawPixel(const uw::sensor_models::Pose3& body_T_link,
                                   const uw::sensor_models::PinholeCamera& camera,
                                   const uw::sensor_models::PlumbBobDistortion& distortion,
                                   const Eigen::Vector3d& world_point) {
  const Eigen::Vector3d point_link = body_T_link.Inverse().Apply(world_point);
  const Eigen::Vector3d point_optical = uw::sensor_models::OpticalFromBodyRotation() * point_link;
  const Eigen::Vector2d normalized_undistorted(point_optical.x() / point_optical.z(),
                                                point_optical.y() / point_optical.z());
  const Eigen::Vector2d normalized_distorted =
      uw::sensor_models::ApplyPlumbBobDistortion(distortion, normalized_undistorted);
  return Eigen::Vector2d(camera.fx * normalized_distorted.x() + camera.cx,
                          camera.fy * normalized_distorted.y() + camera.cy);
}

uw::domain::ImageFrame MakeMarkerImage(const std::string& frame, const std::string& sensor_id,
                                        uint32_t width, uint32_t height, double marker_u,
                                        double marker_v) {
  uw::domain::ImageFrame image = MakeMono8Image(frame, sensor_id, width, height, 40);
  std::string pixels = image.pixel_data();
  const int cu = static_cast<int>(std::lround(marker_u));
  const int cv = static_cast<int>(std::lround(marker_v));
  for (int dv = -2; dv <= 2; ++dv) {
    for (int du = -2; du <= 2; ++du) {
      const int u = cu + du;
      const int v = cv + dv;
      if (u < 0 || v < 0 || u >= static_cast<int>(width) || v >= static_cast<int>(height)) continue;
      pixels[static_cast<std::size_t>(v) * width + static_cast<std::size_t>(u)] =
          static_cast<char>(220);
    }
  }
  image.set_pixel_data(pixels);
  return image;
}

std::optional<Eigen::Vector2d> FindMarkerCentroid(const uw::domain::ImageFrame& image,
                                                   uint8_t threshold) {
  double sum_u = 0.0;
  double sum_v = 0.0;
  double sum_w = 0.0;
  const auto& pixels = image.pixel_data();
  for (uint32_t v = 0; v < image.height(); ++v) {
    for (uint32_t u = 0; u < image.width(); ++u) {
      const auto value = static_cast<uint8_t>(
          pixels[static_cast<std::size_t>(v) * image.row_stride_bytes() + u]);
      if (value <= threshold) continue;
      const double w = static_cast<double>(value);
      sum_u += w * u;
      sum_v += w * v;
      sum_w += w;
    }
  }
  if (sum_w <= 0.0) return std::nullopt;
  return Eigen::Vector2d(sum_u / sum_w, sum_v / sum_w);
}

}  // namespace

TEST(StereoRectificationContext, RejectsMissingCamera) {
  auto rig = MakeIdentityEligibleRig();
  rig.mutable_cameras()->DeleteSubrange(1, 1);  // drop camera_right
  std::string error;
  const auto context = StereoRectificationContext::Create(rig, DefaultParams(), &error);
  EXPECT_FALSE(context.has_value());
  EXPECT_FALSE(error.empty());
}

TEST(StereoRectificationContext, RejectsIllegalIntrinsics) {
  auto rig = MakeIdentityEligibleRig();
  rig.mutable_cameras(0)->set_k_matrix_row_major(0, 0.0);  // fx == 0
  std::string error;
  const auto context = StereoRectificationContext::Create(rig, DefaultParams(), &error);
  EXPECT_FALSE(context.has_value());
  EXPECT_FALSE(error.empty());
}

TEST(StereoRectificationContext, RejectsNonFiniteIntrinsics) {
  auto rig = MakeIdentityEligibleRig();
  rig.mutable_cameras(0)->set_k_matrix_row_major(4, std::numeric_limits<double>::quiet_NaN());
  std::string error;
  const auto context = StereoRectificationContext::Create(rig, DefaultParams(), &error);
  EXPECT_FALSE(context.has_value());
  EXPECT_FALSE(error.empty());
}

TEST(StereoRectificationContext, RejectsMissingFrameEdge) {
  auto rig = MakeIdentityEligibleRig();
  rig.mutable_frame_tree()->DeleteSubrange(1, 1);  // drop camera_right_link edge
  std::string error;
  const auto context = StereoRectificationContext::Create(rig, DefaultParams(), &error);
  EXPECT_FALSE(context.has_value());
  EXPECT_FALSE(error.empty());
}

TEST(StereoRectificationContext, RejectsMismatchedLeftRightImageSize) {
  auto rig = MakeIdentityEligibleRig();
  rig.mutable_cameras(1)->set_width(320);
  std::string error;
  const auto context = StereoRectificationContext::Create(rig, DefaultParams(), &error);
  EXPECT_FALSE(context.has_value());
  EXPECT_FALSE(error.empty());
}

TEST(StereoRectificationContext, RejectsNonPlumbBobDistortionModel) {
  auto rig = MakeIdentityEligibleRig();
  rig.mutable_cameras(0)->set_distortion_model("rational_polynomial");
  std::string error;
  const auto context = StereoRectificationContext::Create(rig, DefaultParams(), &error);
  EXPECT_FALSE(context.has_value());
  EXPECT_FALSE(error.empty());
}

TEST(StereoRectificationContext, RejectsAlphaOutOfRange) {
  auto rig = MakeIdentityEligibleRig();
  auto params = DefaultParams();
  params.alpha = 1.5;
  std::string error;
  const auto context = StereoRectificationContext::Create(rig, params, &error);
  EXPECT_FALSE(context.has_value());
  EXPECT_FALSE(error.empty());
}

TEST(StereoRectificationContext, RejectsEmptySensorOrFrameIdentifiers) {
  auto rig = MakeIdentityEligibleRig();
  auto params = DefaultParams();
  params.left_sensor_id = "";
  std::string error;
  const auto context = StereoRectificationContext::Create(rig, params, &error);
  EXPECT_FALSE(context.has_value());
  EXPECT_FALSE(error.empty());
}

TEST(StereoRectificationContext, RejectsSameLeftAndRightSensorId) {
  auto rig = MakeIdentityEligibleRig();
  auto params = DefaultParams();
  params.right_sensor_id = params.left_sensor_id;
  std::string error;
  const auto context = StereoRectificationContext::Create(rig, params, &error);
  EXPECT_FALSE(context.has_value());
  EXPECT_FALSE(error.empty());
}

TEST(StereoRectificationContext, IdentityFastPathSucceedsAndProducesDerivedRig) {
  const auto rig = MakeIdentityEligibleRig();
  std::string error;
  const auto context = StereoRectificationContext::Create(rig, DefaultParams(), &error);
  ASSERT_TRUE(context.has_value()) << error;

  EXPECT_EQ(context->LeftRectifiedFrame(), "camera_left_link_rectified");
  EXPECT_EQ(context->RightRectifiedFrame(), "camera_right_link_rectified");

  const auto& derived = context->DerivedRig();
  EXPECT_NE(derived.calibration_version().value(), rig.calibration_version().value());
  EXPECT_FALSE(derived.calibration_version().value().empty());

  bool found_left_edge = false;
  bool found_right_edge = false;
  for (const auto& edge : derived.frame_tree()) {
    if (edge.child_frame().value() == "camera_left_link_rectified") found_left_edge = true;
    if (edge.child_frame().value() == "camera_right_link_rectified") found_right_edge = true;
  }
  EXPECT_TRUE(found_left_edge);
  EXPECT_TRUE(found_right_edge);

  // Original frame edges/cameras are still present (derived rig is additive,
  // not a replacement of the raw rig's own frame tree).
  bool found_original_left = false;
  for (const auto& edge : derived.frame_tree()) {
    if (edge.child_frame().value() == "camera_left_link") found_original_left = true;
  }
  EXPECT_TRUE(found_original_left);
}

TEST(StereoRectificationContext, IdentityFastPathIsDeterministic) {
  const auto rig = MakeIdentityEligibleRig();
  std::string error1;
  std::string error2;
  const auto context1 = StereoRectificationContext::Create(rig, DefaultParams(), &error1);
  const auto context2 = StereoRectificationContext::Create(rig, DefaultParams(), &error2);
  ASSERT_TRUE(context1.has_value());
  ASSERT_TRUE(context2.has_value());
  EXPECT_EQ(context1->DerivedRig().calibration_version().value(),
            context2->DerivedRig().calibration_version().value());
}

TEST(StereoRectificationContext, ProcessPreservesHeaderFieldsAndMarksRectified) {
  const auto rig = MakeIdentityEligibleRig();
  std::string error;
  const auto context = StereoRectificationContext::Create(rig, DefaultParams(), &error);
  ASSERT_TRUE(context.has_value()) << error;

  const auto left = MakeMono8Image("camera_left_link", "camera_left", 640, 480, 42);
  const auto right = MakeMono8Image("camera_right_link", "camera_right", 640, 480, 99);

  const auto rectified = context->Process(left, right, &error);
  ASSERT_TRUE(rectified.has_value()) << error;

  const auto& primary = rectified->images.primary;
  EXPECT_TRUE(primary.is_rectified());
  EXPECT_EQ(primary.header().sensor_frame().value(), "camera_left_link_rectified");
  EXPECT_EQ(primary.header().sensor_id().value(), "camera_left");
  EXPECT_EQ(primary.header().observation_id().value(), "camera_left_obs_1");
  EXPECT_EQ(primary.header().capture_time().seconds(), 10);
  EXPECT_EQ(primary.header().capture_time().nanos(), 500);
  EXPECT_EQ(primary.header().receive_time().nanos(), 600);
  EXPECT_EQ(primary.pixel_data(), left.pixel_data());

  ASSERT_TRUE(rectified->images.secondary.has_value());
  const auto& secondary = *rectified->images.secondary;
  EXPECT_TRUE(secondary.is_rectified());
  EXPECT_EQ(secondary.header().sensor_frame().value(), "camera_right_link_rectified");
  EXPECT_EQ(secondary.pixel_data(), right.pixel_data());
}

TEST(StereoRectificationContext, ProcessRejectsWrongImageSize) {
  const auto rig = MakeIdentityEligibleRig();
  std::string error;
  const auto context = StereoRectificationContext::Create(rig, DefaultParams(), &error);
  ASSERT_TRUE(context.has_value()) << error;

  const auto left = MakeMono8Image("camera_left_link", "camera_left", 320, 240, 42);
  const auto right = MakeMono8Image("camera_right_link", "camera_right", 640, 480, 99);

  const auto rectified = context->Process(left, right, &error);
  EXPECT_FALSE(rectified.has_value());
  EXPECT_FALSE(error.empty());
}

TEST(StereoRectificationContext, GeneralRigRecoversDepthAndAlignsRows) {
  const auto fixture = MakeGeneralRig();
  std::string error;
  const auto context = StereoRectificationContext::Create(fixture.rig, DefaultParams(), &error);
  ASSERT_TRUE(context.has_value()) << error;

  const Eigen::Vector3d world_point(3.0, 0.0, 0.15);
  const auto left_pose = EdgeToPose(fixture.rig, "camera_left_link");
  const auto right_pose = EdgeToPose(fixture.rig, "camera_right_link");
  const Eigen::Vector2d left_raw_px =
      ProjectToRawPixel(left_pose, fixture.left_camera, fixture.left_distortion, world_point);
  const Eigen::Vector2d right_raw_px =
      ProjectToRawPixel(right_pose, fixture.right_camera, fixture.right_distortion, world_point);

  const auto left_image =
      MakeMarkerImage("camera_left_link", "camera_left", 640, 480, left_raw_px.x(), left_raw_px.y());
  const auto right_image = MakeMarkerImage("camera_right_link", "camera_right", 640, 480,
                                            right_raw_px.x(), right_raw_px.y());

  const auto rectified = context->Process(left_image, right_image, &error);
  ASSERT_TRUE(rectified.has_value()) << error;
  EXPECT_TRUE(rectified->images.primary.is_rectified());
  ASSERT_TRUE(rectified->images.secondary.has_value());
  EXPECT_TRUE(rectified->images.secondary->is_rectified());

  const auto geometry = uw::sensor_models::StereoGeometry::Resolve(
      context->DerivedRig(), "camera_left", context->LeftRectifiedFrame(), "camera_right",
      context->RightRectifiedFrame());
  ASSERT_TRUE(geometry.valid);

  const auto left_centroid = FindMarkerCentroid(rectified->images.primary, 100);
  const auto right_centroid = FindMarkerCentroid(*rectified->images.secondary, 100);
  ASSERT_TRUE(left_centroid.has_value());
  ASSERT_TRUE(right_centroid.has_value());

  EXPECT_LT(std::abs(left_centroid->y() - right_centroid->y()), 0.6);

  const double disparity_px = left_centroid->x() - right_centroid->x();
  ASSERT_GT(disparity_px, 0.5);

  // Expected depth computed from OUR OWN derived rig, not from OpenCV
  // directly: the z-coordinate of the world point expressed in the
  // rectified left camera's optical frame is what stereo disparity actually
  // recovers, so this checks that DerivedRig()'s virtual extrinsics are
  // consistent with the pixels cv::remap() actually produced.
  const auto left_rectified_pose = EdgeToPose(context->DerivedRig(), context->LeftRectifiedFrame());
  const Eigen::Vector3d point_in_rectified_left_link =
      left_rectified_pose.Inverse().Apply(world_point);
  const Eigen::Vector3d point_in_rectified_left_optical =
      uw::sensor_models::OpticalFromBodyRotation() * point_in_rectified_left_link;
  const double expected_depth_m = point_in_rectified_left_optical.z();

  const double recovered_depth_m = geometry.left.fx * geometry.baseline_m / disparity_px;
  EXPECT_NEAR(recovered_depth_m, expected_depth_m, 0.05);
}

TEST(StereoRectificationContext, CommonValidRoiProducesMatchingCroppedSizes) {
  const auto fixture = MakeGeneralRig();
  auto params = DefaultParams();
  params.crop_policy = RectificationCropPolicy::kCommonValidRoi;

  std::string error;
  const auto context = StereoRectificationContext::Create(fixture.rig, params, &error);
  ASSERT_TRUE(context.has_value()) << error;

  const auto& derived = context->DerivedRig();
  const uw::domain::CameraIntrinsics* left_cam = nullptr;
  const uw::domain::CameraIntrinsics* right_cam = nullptr;
  for (const auto& camera : derived.cameras()) {
    if (camera.sensor_id().value() == "camera_left") left_cam = &camera;
    if (camera.sensor_id().value() == "camera_right") right_cam = &camera;
  }
  ASSERT_NE(left_cam, nullptr);
  ASSERT_NE(right_cam, nullptr);
  EXPECT_EQ(left_cam->width(), right_cam->width());
  EXPECT_EQ(left_cam->height(), right_cam->height());
  EXPECT_GT(left_cam->width(), 0u);
  EXPECT_GT(left_cam->height(), 0u);
  EXPECT_LE(left_cam->width(), 640u);
  EXPECT_LE(left_cam->height(), 480u);

  const Eigen::Vector3d world_point(3.0, 0.0, 0.15);
  const auto left_pose = EdgeToPose(fixture.rig, "camera_left_link");
  const auto right_pose = EdgeToPose(fixture.rig, "camera_right_link");
  const Eigen::Vector2d left_raw_px =
      ProjectToRawPixel(left_pose, fixture.left_camera, fixture.left_distortion, world_point);
  const Eigen::Vector2d right_raw_px =
      ProjectToRawPixel(right_pose, fixture.right_camera, fixture.right_distortion, world_point);
  const auto left_image =
      MakeMarkerImage("camera_left_link", "camera_left", 640, 480, left_raw_px.x(), left_raw_px.y());
  const auto right_image = MakeMarkerImage("camera_right_link", "camera_right", 640, 480,
                                            right_raw_px.x(), right_raw_px.y());
  const auto rectified = context->Process(left_image, right_image, &error);
  ASSERT_TRUE(rectified.has_value()) << error;
  EXPECT_EQ(rectified->images.primary.width(), left_cam->width());
  EXPECT_EQ(rectified->images.primary.height(), left_cam->height());
  EXPECT_EQ(rectified->images.primary.row_stride_bytes(), left_cam->width());
  ASSERT_TRUE(rectified->images.secondary.has_value());
  EXPECT_EQ(rectified->images.secondary->width(), rectified->images.primary.width());
  EXPECT_EQ(rectified->images.secondary->height(), rectified->images.primary.height());
}
