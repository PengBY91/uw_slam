#include <gtest/gtest.h>

#include "sensor_models/camera_model.hpp"

TEST(CameraModel, ProjectUnprojectRoundTrip) {
  uw::domain::CameraIntrinsics intrinsics;
  intrinsics.set_width(640);
  intrinsics.set_height(480);
  for (double v : {420.0, 0.0, 320.0, 0.0, 420.0, 240.0, 0.0, 0.0, 1.0}) {
    intrinsics.add_k_matrix_row_major(v);
  }
  const auto camera = uw::sensor_models::PinholeCamera::FromIntrinsics(intrinsics);
  EXPECT_DOUBLE_EQ(camera.fx, 420.0);
  EXPECT_DOUBLE_EQ(camera.cx, 320.0);

  const Eigen::Vector3d point_camera(1.5, -0.5, 6.3);
  const Eigen::Vector2d pixel = camera.Project(point_camera);
  const Eigen::Vector3d recovered = camera.Unproject(pixel.x(), pixel.y(), point_camera.z());
  EXPECT_NEAR(recovered.x(), point_camera.x(), 1e-9);
  EXPECT_NEAR(recovered.y(), point_camera.y(), 1e-9);
  EXPECT_NEAR(recovered.z(), point_camera.z(), 1e-9);
}

TEST(StereoGeometryTest, ResolvesBaselineForParallelRig) {
  uw::domain::RigCalibrationSnapshot rig;
  auto add_camera = [&](const std::string& id) {
    auto* camera = rig.add_cameras();
    camera->mutable_sensor_id()->set_value(id);
    camera->set_width(640);
    camera->set_height(480);
    for (double v : {420.0, 0.0, 320.0, 0.0, 420.0, 240.0, 0.0, 0.0, 1.0}) {
      camera->add_k_matrix_row_major(v);
    }
  };
  add_camera("camera_left");
  add_camera("camera_right");

  auto add_edge = [&](const std::string& child, double y) {
    auto* edge = rig.add_frame_tree();
    edge->mutable_parent_frame()->set_value("base_link");
    edge->mutable_child_frame()->set_value(child);
    for (double v : {1.0, 0.0, 0.0, 0.15, 0.0, 1.0, 0.0, y, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0}) {
      edge->mutable_transform()->add_matrix_row_major(v);
    }
  };
  add_edge("camera_left_link", 0.06);
  add_edge("camera_right_link", -0.06);

  const auto geometry = uw::sensor_models::StereoGeometry::Resolve(
      rig, "camera_left", "camera_left_link", "camera_right", "camera_right_link");
  ASSERT_TRUE(geometry.valid);
  EXPECT_NEAR(geometry.baseline_m, 0.12, 1e-9);
  EXPECT_DOUBLE_EQ(geometry.left.fx, 420.0);
}

TEST(StereoGeometryTest, RejectsNonParallelOrientation) {
  uw::domain::RigCalibrationSnapshot rig;
  auto* left_cam = rig.add_cameras();
  left_cam->mutable_sensor_id()->set_value("camera_left");
  left_cam->set_width(640);
  left_cam->set_height(480);
  auto* right_cam = rig.add_cameras();
  right_cam->mutable_sensor_id()->set_value("camera_right");
  right_cam->set_width(640);
  right_cam->set_height(480);

  auto* left_edge = rig.add_frame_tree();
  left_edge->mutable_parent_frame()->set_value("base_link");
  left_edge->mutable_child_frame()->set_value("camera_left_link");
  for (double v : {1.0, 0.0, 0.0, 0.15, 0.0, 1.0, 0.0, 0.06, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0}) {
    left_edge->mutable_transform()->add_matrix_row_major(v);
  }
  // Right camera rotated 90 degrees about Z relative to left — not a valid
  // rectified pair for this v1's parallel-baseline assumption.
  auto* right_edge = rig.add_frame_tree();
  right_edge->mutable_parent_frame()->set_value("base_link");
  right_edge->mutable_child_frame()->set_value("camera_right_link");
  for (double v : {0.0, -1.0, 0.0, 0.15, 1.0, 0.0, 0.0, -0.06, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0}) {
    right_edge->mutable_transform()->add_matrix_row_major(v);
  }

  const auto geometry = uw::sensor_models::StereoGeometry::Resolve(
      rig, "camera_left", "camera_left_link", "camera_right", "camera_right_link");
  EXPECT_FALSE(geometry.valid);
}
