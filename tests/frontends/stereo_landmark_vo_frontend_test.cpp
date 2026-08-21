#include "frontends/stereo_landmark_vo_frontend.hpp"

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "sensor_models/camera_model.hpp"
#include "sensor_models/geometry.hpp"

using uw::sensor_models::PinholeCamera;
using uw::sensor_models::Pose3;
using uw::sensor_models::StereoGeometry;

namespace {

constexpr uint8_t kBackground = 10;
constexpr int kSquareHalf = 6;

uw::domain::RigCalibrationSnapshot MakeRig(double fx, double cx, double cy, double baseline_half,
                                            uint32_t width, uint32_t height) {
  uw::domain::RigCalibrationSnapshot rig;
  auto add_camera = [&](const std::string& id) {
    auto* camera = rig.add_cameras();
    camera->mutable_sensor_id()->set_value(id);
    camera->set_width(width);
    camera->set_height(height);
    for (double v : {fx, 0.0, cx, 0.0, fx, cy, 0.0, 0.0, 1.0}) camera->add_k_matrix_row_major(v);
  };
  add_camera("camera_left");
  add_camera("camera_right");
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

void PaintSquare(std::vector<uint8_t>& image, uint32_t width, uint32_t height, int cu, int cv, int half,
                  uint8_t value) {
  for (int dv = -half; dv <= half; ++dv) {
    const int v = cv + dv;
    if (v < 0 || v >= static_cast<int>(height)) continue;
    for (int du = -half; du <= half; ++du) {
      const int u = cu + du;
      if (u < 0 || u >= static_cast<int>(width)) continue;
      image[static_cast<std::size_t>(v) * width + u] = value;
    }
  }
}

uw::domain::ImageFrame MakeImageFrame(const std::string& frame, const std::string& observation_id,
                                       uint32_t width, uint32_t height, std::vector<uint8_t> pixels) {
  uw::domain::ImageFrame image;
  image.mutable_header()->mutable_sensor_frame()->set_value(frame);
  image.mutable_header()->mutable_observation_id()->set_value(observation_id);
  image.set_width(width);
  image.set_height(height);
  image.set_row_stride_bytes(width);
  image.set_encoding(uw::domain::ImageFrame::IMAGE_ENCODING_MONO8);
  image.set_pixel_data(std::string(reinterpret_cast<const char*>(pixels.data()), pixels.size()));
  return image;
}

// Paints `points` (already expressed in the LEFT camera's optical frame)
// as distinctly-intensity-valued squares into a synthetic stereo MONO8
// pair, with the right image correctly shifted by the stereo baseline —
// the same right(u - d) == left(u) convention BlockMatcher/
// StereoLandmarkVoFrontend assume.
std::pair<uw::domain::ImageFrame, uw::domain::ImageFrame> BuildStereoFrames(
    const PinholeCamera& left_cam, const PinholeCamera& right_cam, double baseline_m,
    const std::vector<Eigen::Vector3d>& points, const std::vector<uint8_t>& intensities,
    const std::string& observation_id, uint32_t width, uint32_t height) {
  std::vector<uint8_t> left_pixels(static_cast<std::size_t>(width) * height, kBackground);
  std::vector<uint8_t> right_pixels(static_cast<std::size_t>(width) * height, kBackground);

  for (std::size_t i = 0; i < points.size(); ++i) {
    const Eigen::Vector2d left_pixel = left_cam.Project(points[i]);
    PaintSquare(left_pixels, width, height, static_cast<int>(std::lround(left_pixel.x())),
                static_cast<int>(std::lround(left_pixel.y())), kSquareHalf, intensities[i]);

    const Eigen::Vector3d right_point = points[i] - Eigen::Vector3d(baseline_m, 0.0, 0.0);
    const Eigen::Vector2d right_pixel = right_cam.Project(right_point);
    PaintSquare(right_pixels, width, height, static_cast<int>(std::lround(right_pixel.x())),
                static_cast<int>(std::lround(right_pixel.y())), kSquareHalf, intensities[i]);
  }

  return {MakeImageFrame("camera_left_link", observation_id, width, height, left_pixels),
          MakeImageFrame("camera_right_link", observation_id, width, height, right_pixels)};
}

// Mirrors stereo_landmark_vo_frontend.cpp's private BodyFromCameraOptical
// helper (not exposed via the header — this is a from-scratch local
// re-derivation, same "test builds its own fixtures" pattern as MakeRig
// above), so the test can independently compute what the frontend is
// expected to do with the optical<->body conversion instead of just
// trusting its internals.
uw::sensor_models::Pose3 BodyFromCameraOpticalForTest(const uw::domain::RigCalibrationSnapshot& rig,
                                                       const std::string& camera_frame) {
  Pose3 camera_link_body_pose = Pose3::Identity();
  for (const auto& edge : rig.frame_tree()) {
    if (edge.child_frame().value() == camera_frame) {
      camera_link_body_pose = Pose3::FromProto(edge.transform());
      break;
    }
  }
  Pose3 optical_to_body_rotation;
  optical_to_body_rotation.rotation = Eigen::Quaterniond(uw::sensor_models::OpticalFromBodyRotation()).inverse();
  return camera_link_body_pose * optical_to_body_rotation;
}

uw::frontends::StereoLandmarkVoFrontendParams MakeParams() {
  uw::frontends::StereoLandmarkVoFrontendParams params;
  params.detector.intensity_threshold = 100;
  params.detector.min_blob_pixels = 4;
  params.detector.max_blob_pixels = 400;
  params.detector.patch_half_size = 2;
  params.stereo_matcher.min_ncc_score = 0.9;
  params.temporal_matcher.min_ncc_score = 0.9;
  params.min_landmarks_for_pose = 3;
  return params;
}

}  // namespace

TEST(StereoLandmarkVoFrontend, RecoversRelativePoseFromTwoSyntheticKeyframes) {
  // A high-resolution synthetic camera (fx=1000, wide baseline) keeps
  // per-pixel disparity quantization noise (painted squares snap to
  // integer pixel centers, same as any real rasterizer) well below the
  // relative motion this test recovers — at low resolution the two are
  // comparable and no fitter, however correct, can tell signal from
  // rounding noise. See this test's git history for a low-res version
  // that failed for exactly that reason, not an algorithm bug (confirmed
  // by RigidTransformFit's own unit test recovering an equivalent
  // transform to 1e-9 from un-quantized points).
  const uint32_t width = 1000, height = 1000;
  const auto rig = MakeRig(/*fx=*/1000.0, /*cx=*/500.0, /*cy=*/500.0, /*baseline_half=*/1.0, width, height);
  const auto geometry = StereoGeometry::Resolve(rig, "camera_left", "camera_left_link", "camera_right",
                                                 "camera_right_link");
  ASSERT_TRUE(geometry.valid);

  // The pose the frontend is expected to recover — a BODY-frame relative
  // motion (RelativePoseMeasurement's actual contract, consumed as such by
  // RelativePoseFactorBuilder/PoseGraphProblem elsewhere), NOT a
  // camera-optical-frame one. This distinction is the whole point of this
  // test: an earlier version of this frontend computed the pose fit
  // directly in camera-optical coordinates and returned it unconverted —
  // translation *magnitudes* looked fine (a rotation-only error doesn't
  // change vector norms) but landed on the wrong axes once composed into
  // a real pose graph, only caught by running the actual end-to-end demo,
  // not this test (which, before this fix, used an identity-rotation test
  // rig that happened to make optical and body frames coincide, masking
  // exactly this bug — see BodyFromCameraOpticalForTest's non-trivial
  // result below for why that's no longer possible to accidentally miss).
  Pose3 true_relative_body;
  true_relative_body.translation = Eigen::Vector3d(0.5, 0.3, 1.0);
  true_relative_body.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(0.1, Eigen::Vector3d(0.0, 0.0, 1.0)));
  true_relative_body.rotation.normalize();

  // The equivalent motion expressed in the LEFT camera's OPTICAL frame —
  // what the raw Kabsch fit over triangulated camera-frame points actually
  // computes, before the frontend converts it to body frame. Points_curr
  // below is built with THIS transform (not true_relative_body directly)
  // since it must be consistent with what a real stereo pair would show a
  // camera physically undergoing true_relative_body's body motion.
  const auto body_from_camera_optical = BodyFromCameraOpticalForTest(rig, "camera_left_link");
  const Pose3 true_relative_camera =
      body_from_camera_optical.Inverse() * true_relative_body * body_from_camera_optical;

  // Five well-separated pixel targets, unprojected to depth=8m so the
  // synthetic squares don't touch (a corner cluster + a center point).
  const double depth = 8.0;
  std::vector<Eigen::Vector3d> points_prev;
  for (auto [u, v] : std::vector<std::pair<double, double>>{
           {200, 200}, {800, 200}, {200, 800}, {800, 800}, {500, 500}}) {
    points_prev.push_back(geometry.left.Unproject(u, v, depth));
  }
  const std::vector<uint8_t> intensities = {130, 150, 170, 190, 210};

  std::vector<Eigen::Vector3d> points_curr;
  points_curr.reserve(points_prev.size());
  for (const auto& p : points_prev) points_curr.push_back(true_relative_camera.Inverse().Apply(p));

  const auto [prev_left, prev_right] = BuildStereoFrames(geometry.left, geometry.right, geometry.baseline_m,
                                                          points_prev, intensities, "kf0", width, height);
  const auto [curr_left, curr_right] = BuildStereoFrames(geometry.left, geometry.right, geometry.baseline_m,
                                                          points_curr, intensities, "kf1", width, height);

  uw::frontends::StereoLandmarkVoFrontend frontend(MakeParams());

  uw::measurement_api::CameraFrameBundle bundle1;
  bundle1.primary = prev_left;
  bundle1.secondary = prev_right;
  EXPECT_FALSE(frontend.Process(bundle1, rig).has_value());  // no previous frame to compare against yet

  uw::measurement_api::CameraFrameBundle bundle2;
  bundle2.primary = curr_left;
  bundle2.secondary = curr_right;
  const auto result = frontend.Process(bundle2, rig);
  ASSERT_TRUE(result.has_value());

  const auto& measurement = uw::domain::GetPayload<uw::domain::RelativePoseMeasurement>(*result);
  EXPECT_EQ(measurement.from_keyframe().value(), "kf0");
  EXPECT_EQ(measurement.to_keyframe().value(), "kf1");

  // Tolerances are loose relative to RigidTransformFit's own 1e-9 (see
  // that test): triangulated points here come from *detected pixel
  // centroids* of squares painted at rounded integer pixel coordinates,
  // so some residual quantization noise is expected and correct, not a
  // bug — this test's job is to confirm the whole detect->match->
  // triangulate->fit pipeline recovers the right pose to a sane
  // real-world tolerance, not bit-exactness.
  const auto recovered = Pose3::FromProto(measurement.relative_pose());
  EXPECT_NEAR((recovered.translation - true_relative_body.translation).norm(), 0.0, 0.05);
  EXPECT_NEAR(std::abs(recovered.rotation.dot(true_relative_body.rotation)), 1.0, 1e-3);
}

TEST(StereoLandmarkVoFrontend, RejectsWhenTooFewLandmarksMatchBetweenFrames) {
  const uint32_t width = 100, height = 100;
  const auto rig = MakeRig(100.0, 50.0, 50.0, 0.25, width, height);
  const auto geometry = StereoGeometry::Resolve(rig, "camera_left", "camera_left_link", "camera_right",
                                                 "camera_right_link");
  ASSERT_TRUE(geometry.valid);

  // Only two landmarks: below min_landmarks_for_pose=3.
  std::vector<Eigen::Vector3d> points = {geometry.left.Unproject(30, 30, 8.0),
                                          geometry.left.Unproject(70, 70, 8.0)};
  const std::vector<uint8_t> intensities = {130, 190};

  const auto [left1, right1] = BuildStereoFrames(geometry.left, geometry.right, geometry.baseline_m, points,
                                                  intensities, "kf0", width, height);
  const auto [left2, right2] = BuildStereoFrames(geometry.left, geometry.right, geometry.baseline_m, points,
                                                  intensities, "kf1", width, height);

  uw::frontends::StereoLandmarkVoFrontend frontend(MakeParams());
  uw::measurement_api::CameraFrameBundle bundle1;
  bundle1.primary = left1;
  bundle1.secondary = right1;
  frontend.Process(bundle1, rig);

  uw::measurement_api::CameraFrameBundle bundle2;
  bundle2.primary = left2;
  bundle2.secondary = right2;
  EXPECT_FALSE(frontend.Process(bundle2, rig).has_value());
}

TEST(StereoLandmarkVoFrontend, RejectsBundleWithoutSecondaryFrame) {
  const auto rig = MakeRig(100.0, 50.0, 50.0, 0.25, 100, 100);
  uw::frontends::StereoLandmarkVoFrontend frontend(MakeParams());

  uw::measurement_api::CameraFrameBundle bundle;
  bundle.primary = MakeImageFrame("camera_left_link", "kf0", 100, 100,
                                   std::vector<uint8_t>(100 * 100, kBackground));
  EXPECT_FALSE(frontend.Process(bundle, rig).has_value());
}
