#include "runtime/config.hpp"

#include <cstdio>
#include <filesystem>
#include <fstream>

#include <gtest/gtest.h>

// UW_REPO_ROOT is injected by CMake (see runtime/CMakeLists.txt) so this
// test finds configs/ regardless of ctest's working directory.
#ifndef UW_REPO_ROOT
#define UW_REPO_ROOT "."
#endif

TEST(Config, LoadsExperimentConfigWithAllThreeLayers) {
  const auto config =
      uw::runtime::LoadExperimentConfig(std::string(UW_REPO_ROOT) + "/configs/experiment/synthetic_smoke.yaml");

  // defaults/platform.yaml
  EXPECT_EQ(config.defaults.solver, "gauss_newton_v1");
  EXPECT_EQ(config.defaults.max_iterations, 30);
  EXPECT_DOUBLE_EQ(config.defaults.default_sqrt_information.sonar_range, 15.0);
  EXPECT_DOUBLE_EQ(config.defaults.warmup_seconds, 0.0);

  // rig/example_auv.yaml
  EXPECT_EQ(config.rig.calibration_version().value(), "example_auv_v2");
  ASSERT_EQ(config.rig.frame_tree_size(), 4);
  EXPECT_EQ(config.rig.frame_tree(1).child_frame().value(), "camera_left_link");
  EXPECT_EQ(config.rig.frame_tree(2).child_frame().value(), "camera_right_link");
  EXPECT_EQ(config.rig.frame_tree(3).child_frame().value(), "sonar_link");

  ASSERT_EQ(config.rig.cameras_size(), 2);
  EXPECT_EQ(config.rig.cameras(0).sensor_id().value(), "camera_left");
  EXPECT_EQ(config.rig.cameras(0).width(), 640u);
  ASSERT_EQ(config.rig.cameras(0).k_matrix_row_major_size(), 9);
  EXPECT_DOUBLE_EQ(config.rig.cameras(0).k_matrix_row_major(0), 420.0);
  EXPECT_EQ(config.rig.cameras(1).sensor_id().value(), "camera_right");

  ASSERT_EQ(config.rig.sonar_beam_models_size(), 1);
  EXPECT_TRUE(config.rig.sonar_beam_models(0).sonar_enabled());
  EXPECT_DOUBLE_EQ(config.rig.time_offset_seconds().at("camera_left"), 0.0);
  EXPECT_DOUBLE_EQ(config.rig.time_offset_seconds().at("camera_right"), 0.0);
  EXPECT_DOUBLE_EQ(config.rig.time_offset_seconds().at("sonar0"), 0.0);

  // scenario/synthetic_smoke.yaml
  EXPECT_EQ(config.scenario.seed, 42u);
  EXPECT_EQ(config.scenario.num_keyframes, 12);
  EXPECT_DOUBLE_EQ(config.scenario.radius_m, 8.0);
  ASSERT_EQ(config.scenario.sonar_targets_world.size(), 3u);
  EXPECT_NEAR(config.scenario.sonar_targets_world[0].x(), 2.0, 1e-9);

  // experiment/synthetic_smoke.yaml itself
  EXPECT_EQ(config.sonar_frontend, "sonar_cfar_frontend_v1");
  EXPECT_EQ(config.optical_frontend, "stereo_depth_frontend_v1");
  EXPECT_EQ(config.landmark_detector, "bright_blob");  // not set in this experiment file; default
  EXPECT_EQ(config.estimator_mode, "black_box_vio");
  EXPECT_TRUE(config.write_run_manifest);
}

TEST(Config, MissingOptionalFieldsFallBackToDefaults) {
  // ScenarioConfig's built-in defaults should survive a config that only
  // sets a subset of fields.
  const auto scenario = uw::runtime::ScenarioConfig{};
  EXPECT_EQ(scenario.seed, 42u);
  EXPECT_DOUBLE_EQ(scenario.noise.relative_pose_noise_m, 0.02);
}

TEST(Config, ParsesNonDefaultWarmupSeconds) {
  const auto tmp_path = std::filesystem::temp_directory_path() / "uw_config_test_warmup.yaml";
  {
    std::ofstream out(tmp_path);
    out << "estimation:\n  warmup_seconds: 1.5\n";
  }
  const auto config = uw::runtime::LoadPlatformDefaultsConfig(tmp_path.string());
  EXPECT_DOUBLE_EQ(config.warmup_seconds, 1.5);
  std::remove(tmp_path.string().c_str());
}

TEST(Config, RejectsRigTransformThatIsNotFourByFour) {
  const auto path = std::filesystem::temp_directory_path() / "uw_bad_transform_rig.yaml";
  {
    std::ofstream out(path);
    out << "calibration_version: bad\n"
           "frame_tree:\n"
           "  - parent_frame: base_link\n"
           "    child_frame: camera_left_link\n"
           "    transform_row_major: [1, 0, 0]\n";
  }
  EXPECT_THROW(uw::runtime::LoadRigConfig(path.string()), std::runtime_error);
  std::remove(path.string().c_str());
}

TEST(Config, RejectsCameraIntrinsicMatrixThatIsNotThreeByThree) {
  const auto path = std::filesystem::temp_directory_path() / "uw_bad_camera_rig.yaml";
  {
    std::ofstream out(path);
    out << "calibration_version: bad\n"
           "cameras:\n"
           "  - sensor_id: camera_left\n"
           "    width: 640\n"
           "    height: 480\n"
           "    k_matrix_row_major: [420, 0, 320]\n";
  }
  EXPECT_THROW(uw::runtime::LoadRigConfig(path.string()), std::runtime_error);
  std::remove(path.string().c_str());
}
