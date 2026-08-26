#include "frontends/loop_closure_frontend.hpp"

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
constexpr int kSquareHalf = 15;
constexpr uint32_t kWidth = 400, kHeight = 400;

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
  image.set_is_rectified(true);
  image.set_pixel_data(std::string(reinterpret_cast<const char*>(pixels.data()), pixels.size()));
  return image;
}

// Paints `points` (LEFT camera optical frame) as distinctly-intensity-
// valued squares into a synthetic stereo MONO8 pair, right image shifted
// by the stereo baseline (same right(u-d)==left(u) convention
// BlockMatcher/StereoLandmarkVoFrontend assume). Each landmark's Harris
// corners are placed on a DIFFERENT image row (see MakeScenePositions'
// comment) -- combined with revisit_matcher.max_row_diff_px (see
// MakeParams), this is what lets PatchMatcher tell landmark i's corners
// apart from landmark j's: a plain square's 4 corners are locally
// indistinguishable, via NCC, from the SAME corner-type of any OTHER
// same-sized square (NCC is invariant to each patch's absolute intensity
// level) -- only genuine spatial separation (here, row separation) rules
// out a cross-landmark match, exactly the way BlockMatcher/PatchMatcher's
// row constraint already does for LEFT<->RIGHT stereo correspondence.
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

// A flat MONO8 stereo pair with no corners at all — used as filler
// keyframes to advance the frontend's archive insertion order (satisfying
// min_keyframe_index_gap) without introducing any landmark of its own.
std::pair<uw::domain::ImageFrame, uw::domain::ImageFrame> BuildBlankStereoFrames(
    const std::string& observation_id, uint32_t width, uint32_t height) {
  std::vector<uint8_t> pixels(static_cast<std::size_t>(width) * height, kBackground);
  return {MakeImageFrame("camera_left_link", observation_id, width, height, pixels),
          MakeImageFrame("camera_right_link", observation_id, width, height, pixels)};
}

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

uw::frontends::LoopClosureFrontendParams MakeParams() {
  uw::frontends::LoopClosureFrontendParams params;
  params.detector.window_radius = 2;
  params.detector.quality_level = 0.1;
  params.detector.nms_radius = 8;
  params.detector.max_corners = 40;
  params.detector.patch_half_size = 3;
  params.stereo_matcher.min_ncc_score = 0.9;
  params.revisit_matcher.min_ncc_score = 0.9;
  // See BuildStereoFrames' comment: rules out cross-landmark ties (same
  // corner-type, different square, both score a perfect 1.0 via NCC) by
  // requiring candidate pairs to be roughly row-aligned -- landmark rows
  // are always >=100px apart (MakeScenePositions), comfortably more than
  // both a single square's own ~2*kSquareHalf row spread and the small
  // test motions used below.
  params.revisit_matcher.max_row_diff_px = 45.0;
  params.min_landmarks_for_pose = 3;
  params.candidate_search_radius_m = 3.0;
  params.min_keyframe_index_gap = 3;
  params.max_accepted_translation_m = 5.0;
  params.max_accepted_rotation_rad = 0.6;
  params.max_loop_edges_per_keyframe = 2;
  return params;
}

// Four well-separated, row-distinct points (LEFT camera optical frame,
// depth=8m) -- the "scene" a revisit must recognize. See MakeParams'
// max_row_diff_px comment for why the rows differ.
std::vector<Eigen::Vector3d> MakeScenePositions(const StereoGeometry& geometry) {
  std::vector<Eigen::Vector3d> points;
  for (auto [u, v] : std::vector<std::pair<double, double>>{{100, 60}, {300, 160}, {150, 260}, {280, 360}}) {
    points.push_back(geometry.left.Unproject(u, v, /*depth=*/8.0));
  }
  return points;
}

std::vector<uint8_t> MakeSceneIntensities() { return {130, 150, 170, 190}; }

// A second, disjoint scene (different pixel positions) -- "some unrelated
// keyframe" that must never be confused with the primary scene.
std::vector<Eigen::Vector3d> MakeAlternateScenePositions(const StereoGeometry& geometry) {
  std::vector<Eigen::Vector3d> points;
  for (auto [u, v] : std::vector<std::pair<double, double>>{{200, 40}, {80, 140}, {320, 240}, {200, 340}}) {
    points.push_back(geometry.left.Unproject(u, v, /*depth=*/8.0));
  }
  return points;
}

}  // namespace

TEST(LoopClosureFrontend, AcceptsLoopClosureBetweenRevisitedKeyframes) {
  const auto rig = MakeRig(/*fx=*/400.0, /*cx=*/200.0, /*cy=*/200.0, /*baseline_half=*/1.0, kWidth, kHeight);
  const auto geometry =
      StereoGeometry::Resolve(rig, "camera_left", "camera_left_link", "camera_right", "camera_right_link");
  ASSERT_TRUE(geometry.valid);

  Pose3 true_relative_body;
  true_relative_body.translation = Eigen::Vector3d(0.4, 0.2, 0.5);
  true_relative_body.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(0.05, Eigen::Vector3d(0.0, 0.0, 1.0)));
  true_relative_body.rotation.normalize();
  const auto body_from_camera_optical = BodyFromCameraOpticalForTest(rig, "camera_left_link");
  const Pose3 true_relative_camera =
      body_from_camera_optical.Inverse() * true_relative_body * body_from_camera_optical;

  const auto points_a = MakeScenePositions(geometry);
  const auto intensities = MakeSceneIntensities();
  std::vector<Eigen::Vector3d> points_c;
  for (const auto& p : points_a) points_c.push_back(true_relative_camera.Inverse().Apply(p));

  const auto [a_left, a_right] = BuildStereoFrames(geometry.left, geometry.right, geometry.baseline_m, points_a,
                                                    intensities, "kf_A", kWidth, kHeight);
  const auto alt_points = MakeAlternateScenePositions(geometry);
  const auto [b_left, b_right] = BuildStereoFrames(geometry.left, geometry.right, geometry.baseline_m,
                                                    alt_points, intensities, "kf_B", kWidth, kHeight);
  const auto [c_left, c_right] = BuildStereoFrames(geometry.left, geometry.right, geometry.baseline_m, points_c,
                                                    intensities, "kf_C", kWidth, kHeight);

  uw::frontends::LoopClosureFrontend frontend(MakeParams());

  uw::measurement_api::CameraFrameBundle bundle_a{a_left, a_right};
  EXPECT_TRUE(frontend.Process(bundle_a, rig, "kf_A", Pose3::Identity()).empty());

  // kf_B: different scene, and its pose estimate is placed FAR outside
  // candidate_search_radius_m from kf_A's — must never be proposed as a
  // loop candidate for kf_C below (distance-gated, not just appearance).
  Pose3 pose_b = Pose3::Identity();
  pose_b.translation = Eigen::Vector3d(100.0, 0.0, 0.0);
  uw::measurement_api::CameraFrameBundle bundle_b{b_left, b_right};
  EXPECT_TRUE(frontend.Process(bundle_b, rig, "kf_B", pose_b).empty());

  // One filler keyframe so kf_A ends up >= min_keyframe_index_gap=3
  // archive slots behind kf_C (kf_A=0, kf_B=1, filler=2, kf_C=3).
  const auto [filler_left, filler_right] = BuildBlankStereoFrames("kf_filler", kWidth, kHeight);
  EXPECT_TRUE(frontend
                  .Process(uw::measurement_api::CameraFrameBundle{filler_left, filler_right}, rig, "kf_filler",
                          Pose3::Identity())
                  .empty());

  // kf_C: revisits kf_A's scene, close in position (within
  // candidate_search_radius_m of kf_A's pose_estimate) and far enough in
  // archive order (>= min_keyframe_index_gap=3) to be a legitimate loop
  // candidate.
  uw::measurement_api::CameraFrameBundle bundle_c{c_left, c_right};
  const auto loops = frontend.Process(bundle_c, rig, "kf_C", true_relative_body);
  ASSERT_EQ(loops.size(), 1u);

  const auto& measurement = uw::domain::GetPayload<uw::domain::RelativePoseMeasurement>(loops[0]);
  EXPECT_EQ(measurement.from_keyframe().value(), "kf_A");
  EXPECT_EQ(measurement.to_keyframe().value(), "kf_C");

  // Tolerances are loose relative to RigidTransformFit's own <1e-9 (see
  // that test) or even StereoLandmarkVoFrontend's 0.05m: a Harris CORNER
  // is a real image-edge artifact (unlike a blob detector's centroid,
  // which sits at the true landmark center), so this pipeline's job here
  // is to confirm the whole detect->match->fit->sanity-gate pipeline
  // recovers a plausible pose from the RIGHT keyframe, not bit-exactness.
  const auto recovered = Pose3::FromProto(measurement.relative_pose());
  EXPECT_NEAR((recovered.translation - true_relative_body.translation).norm(), 0.0, 0.3);
  EXPECT_GT(std::abs(recovered.rotation.dot(true_relative_body.rotation)), 0.98);
  EXPECT_EQ(measurement.covariance_6x6_row_major_size(), 36);
}

TEST(LoopClosureFrontend, RejectsTemporallyAdjacentKeyframesAsLoopCandidates) {
  const auto rig = MakeRig(/*fx=*/400.0, /*cx=*/200.0, /*cy=*/200.0, /*baseline_half=*/1.0, kWidth, kHeight);
  const auto geometry =
      StereoGeometry::Resolve(rig, "camera_left", "camera_left_link", "camera_right", "camera_right_link");
  ASSERT_TRUE(geometry.valid);

  const auto points = MakeScenePositions(geometry);
  const auto intensities = MakeSceneIntensities();
  const auto [a_left, a_right] = BuildStereoFrames(geometry.left, geometry.right, geometry.baseline_m, points,
                                                    intensities, "kf_A", kWidth, kHeight);
  // Same exact scene AND same pose — a perfect appearance+position match —
  // but archived only 1 slot before kf_B, below min_keyframe_index_gap=3.
  const auto [b_left, b_right] = BuildStereoFrames(geometry.left, geometry.right, geometry.baseline_m, points,
                                                    intensities, "kf_B", kWidth, kHeight);

  uw::frontends::LoopClosureFrontend frontend(MakeParams());
  EXPECT_TRUE(frontend.Process(uw::measurement_api::CameraFrameBundle{a_left, a_right}, rig, "kf_A",
                               Pose3::Identity())
                  .empty());
  EXPECT_TRUE(frontend.Process(uw::measurement_api::CameraFrameBundle{b_left, b_right}, rig, "kf_B",
                               Pose3::Identity())
                  .empty());
}

TEST(LoopClosureFrontend, RejectsCandidateOutsideSearchRadius) {
  const auto rig = MakeRig(/*fx=*/400.0, /*cx=*/200.0, /*cy=*/200.0, /*baseline_half=*/1.0, kWidth, kHeight);
  const auto geometry =
      StereoGeometry::Resolve(rig, "camera_left", "camera_left_link", "camera_right", "camera_right_link");
  ASSERT_TRUE(geometry.valid);

  const auto points = MakeScenePositions(geometry);
  const auto intensities = MakeSceneIntensities();
  const auto [a_left, a_right] = BuildStereoFrames(geometry.left, geometry.right, geometry.baseline_m, points,
                                                    intensities, "kf_A", kWidth, kHeight);
  const auto [z_left, z_right] = BuildStereoFrames(geometry.left, geometry.right, geometry.baseline_m, points,
                                                    intensities, "kf_Z", kWidth, kHeight);

  auto params = MakeParams();
  params.min_keyframe_index_gap = 1;  // isolate the distance gate, not the temporal one
  uw::frontends::LoopClosureFrontend frontend(params);

  EXPECT_TRUE(frontend.Process(uw::measurement_api::CameraFrameBundle{a_left, a_right}, rig, "kf_A",
                               Pose3::Identity())
                  .empty());

  // Filler keyframes so kf_Z is well past min_keyframe_index_gap from kf_A.
  for (int i = 0; i < 3; ++i) {
    const auto [blank_left, blank_right] =
        BuildBlankStereoFrames("kf_filler_" + std::to_string(i), kWidth, kHeight);
    frontend.Process(uw::measurement_api::CameraFrameBundle{blank_left, blank_right}, rig,
                     "kf_filler_" + std::to_string(i), Pose3::Identity());
  }

  // Same scene as kf_A (would match on appearance), but its pose estimate
  // is well outside candidate_search_radius_m=3.0 — this test isolates
  // the pose-proximity v1 scope boundary itself.
  Pose3 pose_z = Pose3::Identity();
  pose_z.translation = Eigen::Vector3d(10.0, 0.0, 0.0);
  EXPECT_TRUE(
      frontend.Process(uw::measurement_api::CameraFrameBundle{z_left, z_right}, rig, "kf_Z", pose_z).empty());
}

TEST(LoopClosureFrontend, RejectsWhenTooFewLandmarksInCurrentFrame) {
  const auto rig = MakeRig(/*fx=*/400.0, /*cx=*/200.0, /*cy=*/200.0, /*baseline_half=*/1.0, kWidth, kHeight);
  const auto geometry =
      StereoGeometry::Resolve(rig, "camera_left", "camera_left_link", "camera_right", "camera_right_link");
  ASSERT_TRUE(geometry.valid);

  const auto points = MakeScenePositions(geometry);
  const auto intensities = MakeSceneIntensities();
  const auto [a_left, a_right] = BuildStereoFrames(geometry.left, geometry.right, geometry.baseline_m, points,
                                                    intensities, "kf_A", kWidth, kHeight);

  // A completely blank frame -- zero landmarks, well below
  // min_landmarks_for_pose=3.
  const auto [z_left, z_right] = BuildBlankStereoFrames("kf_Z", kWidth, kHeight);

  auto params = MakeParams();
  params.min_keyframe_index_gap = 1;
  uw::frontends::LoopClosureFrontend frontend(params);

  EXPECT_TRUE(frontend.Process(uw::measurement_api::CameraFrameBundle{a_left, a_right}, rig, "kf_A",
                               Pose3::Identity())
                  .empty());
  EXPECT_TRUE(
      frontend.Process(uw::measurement_api::CameraFrameBundle{z_left, z_right}, rig, "kf_Z", Pose3::Identity())
          .empty());
}

TEST(LoopClosureFrontend, HealthAlwaysReportsHealthy) {
  uw::frontends::LoopClosureFrontend frontend(MakeParams());
  EXPECT_EQ(frontend.Health().status(), uw::domain::HealthReport::STATUS_HEALTHY);
  EXPECT_EQ(frontend.Health().component_id(), "loop_closure_frontend");
}
