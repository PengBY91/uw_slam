#include "runtime/config.hpp"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>

#include <gtest/gtest.h>

// UW_REPO_ROOT is injected by CMake (see runtime/CMakeLists.txt) so this
// test finds configs/ regardless of ctest's working directory.
#ifndef UW_REPO_ROOT
#define UW_REPO_ROOT "."
#endif

namespace {

std::string MinimalOnlineRigYaml(const std::string& offsets,
                                 const std::string& provenance,
                                 const std::string& state_sensors = "  - rov-state\n") {
  return "calibration_version: test_v1\n"
         "cameras:\n"
         "  - sensor_id: camera_left\n"
         "    width: 1\n"
         "    height: 1\n"
         "    k_matrix_row_major: [1,0,0,0,1,0,0,0,1]\n"
         "  - sensor_id: camera_right\n"
         "    width: 1\n"
         "    height: 1\n"
         "    k_matrix_row_major: [1,0,0,0,1,0,0,0,1]\n"
         "sonar_beam_models:\n"
         "  - sensor_id: sonar0\n"
         "    sonar_enabled: true\n"
         "vehicle_state_sensors:\n" + state_sensors +
         "time_offset_seconds:\n" + offsets +
         "time_offset_provenance:\n" + provenance;
}

std::filesystem::path WriteRig(const std::string& name, const std::string& yaml) {
  const auto path = std::filesystem::temp_directory_path() / name;
  std::ofstream(path) << yaml;
  return path;
}

}  // namespace

TEST(Config, LoadsExperimentConfigWithAllThreeLayers) {
  const auto config =
      uw::runtime::LoadExperimentConfig(std::string(UW_REPO_ROOT) + "/configs/experiment/synthetic_smoke.yaml");

  // defaults/platform.yaml
  EXPECT_EQ(config.defaults.solver, "gauss_newton_v1");
  EXPECT_EQ(config.defaults.max_iterations, 30);
  EXPECT_DOUBLE_EQ(config.defaults.default_sqrt_information.sonar_range, 15.0);
  EXPECT_DOUBLE_EQ(config.defaults.warmup_seconds, 0.0);
  EXPECT_EQ(config.defaults.sonar_frontend.training_cells, 16);
  EXPECT_EQ(config.defaults.sonar_frontend.guard_cells, 4);
  EXPECT_DOUBLE_EQ(config.defaults.sonar_frontend.probability_false_alarm, 0.01);
  EXPECT_EQ(config.defaults.sonar_frontend.detector_threshold, 50);
  EXPECT_DOUBLE_EQ(config.defaults.sonar_frontend.dbscan_eps_m, 0.20);
  EXPECT_EQ(config.defaults.sonar_frontend.dbscan_min_samples, 2);
  EXPECT_DOUBLE_EQ(config.defaults.sonar_frontend.default_range_sigma_m, 0.05);
  EXPECT_DOUBLE_EQ(config.defaults.sonar_frontend.default_bearing_sigma_rad, 0.01);

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
  ASSERT_EQ(config.rig.vehicle_state_sensors_size(), 1);
  EXPECT_EQ(config.rig.vehicle_state_sensors(0).value(), "rov-state");
  EXPECT_EQ(config.rig.time_offset_provenance().at("camera_left"), "measured:simulation");

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

TEST(Config, LoadsEveryCheckedInRigUnderOnlineV1CardinalityContract) {
  for (const std::string name : {"euroc_mh01.yaml", "example_auv.yaml",
                                 "example_auv_real_camera.yaml",
                                 "example_auv_sonar_only.yaml"}) {
    EXPECT_NO_THROW(uw::runtime::LoadRigConfig(
        std::string(UW_REPO_ROOT) + "/configs/rig/" + name))
        << "rig file: " << name;
  }

  const auto euroc = uw::runtime::LoadRigConfig(
      std::string(UW_REPO_ROOT) + "/configs/rig/euroc_mh01.yaml");
  EXPECT_EQ(euroc.sonar_beam_models_size(), 0);
  EXPECT_EQ(euroc.vehicle_state_sensors_size(), 0);

  const auto sonar_only = uw::runtime::LoadRigConfig(
      std::string(UW_REPO_ROOT) + "/configs/rig/example_auv_sonar_only.yaml");
  EXPECT_EQ(sonar_only.cameras_size(), 0);
  ASSERT_EQ(sonar_only.sonar_beam_models_size(), 1);
  EXPECT_TRUE(sonar_only.sonar_beam_models(0).sonar_enabled());
  EXPECT_EQ(sonar_only.vehicle_state_sensors_size(), 1);
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

TEST(Config, StereoRectificationDefaultsWhenSectionAbsent) {
  const auto config = uw::runtime::PlatformDefaultsConfig{};
  EXPECT_DOUBLE_EQ(config.stereo_rectification.alpha, 0.0);
  EXPECT_EQ(config.stereo_rectification.crop_policy, "full_canvas");
  EXPECT_EQ(config.stereo_rectification.frame_suffix, "_rectified");
}

TEST(Config, ParsesTypedSonarCfarOverrides) {
  const auto tmp_path = std::filesystem::temp_directory_path() / "uw_config_test_sonar_cfar.yaml";
  {
    std::ofstream out(tmp_path);
    out << "frontends:\n"
           "  sonar_cfar:\n"
           "    training_cells: 20\n"
           "    guard_cells: 6\n"
           "    probability_false_alarm: 0.005\n"
           "    detector_threshold: 60\n"
           "    dbscan_eps_m: 0.25\n"
           "    dbscan_min_samples: 3\n"
           "    default_range_sigma_m: 0.08\n"
           "    default_bearing_sigma_rad: 0.02\n";
  }
  const auto config = uw::runtime::LoadPlatformDefaultsConfig(tmp_path.string());
  EXPECT_EQ(config.sonar_frontend.training_cells, 20);
  EXPECT_EQ(config.sonar_frontend.guard_cells, 6);
  EXPECT_DOUBLE_EQ(config.sonar_frontend.probability_false_alarm, 0.005);
  EXPECT_EQ(config.sonar_frontend.detector_threshold, 60);
  EXPECT_DOUBLE_EQ(config.sonar_frontend.dbscan_eps_m, 0.25);
  EXPECT_EQ(config.sonar_frontend.dbscan_min_samples, 3);
  EXPECT_DOUBLE_EQ(config.sonar_frontend.default_range_sigma_m, 0.08);
  EXPECT_DOUBLE_EQ(config.sonar_frontend.default_bearing_sigma_rad, 0.02);
  std::remove(tmp_path.string().c_str());
}

TEST(Config, ParsesStereoRectificationOverrides) {
  const auto tmp_path = std::filesystem::temp_directory_path() / "uw_config_test_rectification.yaml";
  {
    std::ofstream out(tmp_path);
    out << "frontends:\n"
           "  stereo_rectification:\n"
           "    alpha: 0.3\n"
           "    crop_policy: common_valid_roi\n"
           "    frame_suffix: _rect2\n";
  }
  const auto config = uw::runtime::LoadPlatformDefaultsConfig(tmp_path.string());
  EXPECT_DOUBLE_EQ(config.stereo_rectification.alpha, 0.3);
  EXPECT_EQ(config.stereo_rectification.crop_policy, "common_valid_roi");
  EXPECT_EQ(config.stereo_rectification.frame_suffix, "_rect2");
  std::remove(tmp_path.string().c_str());
}

TEST(Config, RejectsStereoRectificationAlphaOutOfRange) {
  const auto tmp_path = std::filesystem::temp_directory_path() / "uw_config_test_rect_alpha.yaml";
  {
    std::ofstream out(tmp_path);
    out << "frontends:\n  stereo_rectification:\n    alpha: 1.5\n";
  }
  EXPECT_THROW(uw::runtime::LoadPlatformDefaultsConfig(tmp_path.string()), std::runtime_error);
  std::remove(tmp_path.string().c_str());
}

TEST(Config, RejectsStereoRectificationEmptyFrameSuffix) {
  const auto tmp_path = std::filesystem::temp_directory_path() / "uw_config_test_rect_suffix.yaml";
  {
    std::ofstream out(tmp_path);
    out << "frontends:\n  stereo_rectification:\n    frame_suffix: \"\"\n";
  }
  EXPECT_THROW(uw::runtime::LoadPlatformDefaultsConfig(tmp_path.string()), std::runtime_error);
  std::remove(tmp_path.string().c_str());
}

TEST(Config, RejectsStereoRectificationUnknownCropPolicy) {
  const auto tmp_path = std::filesystem::temp_directory_path() / "uw_config_test_rect_crop.yaml";
  {
    std::ofstream out(tmp_path);
    out << "frontends:\n  stereo_rectification:\n    crop_policy: full_canv\n";
  }
  EXPECT_THROW(uw::runtime::LoadPlatformDefaultsConfig(tmp_path.string()), std::runtime_error);
  std::remove(tmp_path.string().c_str());
}

TEST(Config, RejectsStereoRectificationUnknownKey) {
  const auto tmp_path = std::filesystem::temp_directory_path() / "uw_config_test_rect_unknown.yaml";
  {
    std::ofstream out(tmp_path);
    out << "frontends:\n  stereo_rectification:\n    bogus_key: 1\n";
  }
  EXPECT_THROW(uw::runtime::LoadPlatformDefaultsConfig(tmp_path.string()), std::runtime_error);
  std::remove(tmp_path.string().c_str());
}

TEST(Config, VisualOdometryMaxConsecutiveFailuresDefaultsToThree) {
  const auto config = uw::runtime::PlatformDefaultsConfig{};
  EXPECT_EQ(config.visual_odometry.max_consecutive_failures, 3);
}

TEST(Config, ParsesVisualOdometryMaxConsecutiveFailuresOverride) {
  const auto tmp_path = std::filesystem::temp_directory_path() / "uw_config_test_vo_failures.yaml";
  {
    std::ofstream out(tmp_path);
    out << "visual_odometry:\n  max_consecutive_failures: 5\n";
  }
  const auto config = uw::runtime::LoadPlatformDefaultsConfig(tmp_path.string());
  EXPECT_EQ(config.visual_odometry.max_consecutive_failures, 5);
  std::remove(tmp_path.string().c_str());
}

TEST(Config, RejectsVisualOdometryMaxConsecutiveFailuresOutOfRange) {
  const auto tmp_path = std::filesystem::temp_directory_path() / "uw_config_test_vo_failures_bad.yaml";
  {
    std::ofstream out(tmp_path);
    out << "visual_odometry:\n  max_consecutive_failures: 0\n";
  }
  EXPECT_THROW(uw::runtime::LoadPlatformDefaultsConfig(tmp_path.string()), std::runtime_error);
  std::remove(tmp_path.string().c_str());
}

TEST(Config, ParsesVisualOdometryCovarianceThresholdOverrides) {
  const auto tmp_path = std::filesystem::temp_directory_path() / "uw_config_test_vo_cov.yaml";
  {
    std::ofstream out(tmp_path);
    out << "visual_odometry:\n"
           "  max_condition_number: 1.0e6\n"
           "  residual_variance_floor_m2: 1.0e-6\n";
  }
  const auto config = uw::runtime::LoadPlatformDefaultsConfig(tmp_path.string());
  EXPECT_DOUBLE_EQ(config.visual_odometry.max_condition_number, 1.0e6);
  EXPECT_DOUBLE_EQ(config.visual_odometry.residual_variance_floor_m2, 1.0e-6);
  std::remove(tmp_path.string().c_str());
}

TEST(Config, RejectsVisualOdometryNonFiniteMaxConditionNumber) {
  const auto tmp_path = std::filesystem::temp_directory_path() / "uw_config_test_vo_cov_bad.yaml";
  {
    std::ofstream out(tmp_path);
    out << "visual_odometry:\n  max_condition_number: -1.0\n";
  }
  EXPECT_THROW(uw::runtime::LoadPlatformDefaultsConfig(tmp_path.string()), std::runtime_error);
  std::remove(tmp_path.string().c_str());
}

TEST(Config, RejectsVisualOdometryNonPositiveResidualVarianceFloor) {
  const auto tmp_path = std::filesystem::temp_directory_path() / "uw_config_test_vo_floor_bad.yaml";
  {
    std::ofstream out(tmp_path);
    out << "visual_odometry:\n  residual_variance_floor_m2: 0.0\n";
  }
  EXPECT_THROW(uw::runtime::LoadPlatformDefaultsConfig(tmp_path.string()), std::runtime_error);
  std::remove(tmp_path.string().c_str());
}

TEST(Config, ParsesVisualOdometryMaxInlierRmseOverride) {
  const auto tmp_path = std::filesystem::temp_directory_path() / "uw_config_test_vo_rmse.yaml";
  {
    std::ofstream out(tmp_path);
    out << "visual_odometry:\n  max_inlier_rmse_m: 0.12\n";
  }
  const auto config = uw::runtime::LoadPlatformDefaultsConfig(tmp_path.string());
  EXPECT_DOUBLE_EQ(config.visual_odometry.max_inlier_rmse_m, 0.12);
  std::remove(tmp_path.string().c_str());
}

TEST(Config, NonPositiveVisualOdometryMaxInlierRmseDisablesTheGate) {
  const auto tmp_path = std::filesystem::temp_directory_path() / "uw_config_test_vo_rmse_disabled.yaml";
  {
    std::ofstream out(tmp_path);
    out << "visual_odometry:\n  max_inlier_rmse_m: 0.0\n";
  }
  const auto config = uw::runtime::LoadPlatformDefaultsConfig(tmp_path.string());
  EXPECT_TRUE(std::isinf(config.visual_odometry.max_inlier_rmse_m));
  std::remove(tmp_path.string().c_str());
}

TEST(Config, RejectsVisualOdometryUnknownKey) {
  const auto tmp_path = std::filesystem::temp_directory_path() / "uw_config_test_vo_unknown.yaml";
  {
    std::ofstream out(tmp_path);
    out << "visual_odometry:\n  bogus_key: 1\n";
  }
  EXPECT_THROW(uw::runtime::LoadPlatformDefaultsConfig(tmp_path.string()), std::runtime_error);
  std::remove(tmp_path.string().c_str());
}

TEST(Config, ParsesRelativePoseSqrtInformationCapsIndependently) {
  const auto tmp_path = std::filesystem::temp_directory_path() / "uw_config_test_relpose_caps.yaml";
  {
    std::ofstream out(tmp_path);
    out << "reliability:\n"
           "  default_sqrt_information:\n"
           "    relative_pose:\n"
           "      translation: 15.0\n"
           "      rotation: 25.0\n"
           "    sonar_range: 15.0\n"
           "    depth: 20.0\n";
  }
  const auto config = uw::runtime::LoadPlatformDefaultsConfig(tmp_path.string());
  EXPECT_DOUBLE_EQ(config.default_sqrt_information.relative_pose.translation, 15.0);
  EXPECT_DOUBLE_EQ(config.default_sqrt_information.relative_pose.rotation, 25.0);
  std::remove(tmp_path.string().c_str());
}

TEST(Config, RejectsZeroRelativePoseTranslationCap) {
  const auto tmp_path = std::filesystem::temp_directory_path() / "uw_config_test_relpose_zero.yaml";
  {
    std::ofstream out(tmp_path);
    out << "reliability:\n  default_sqrt_information:\n    relative_pose:\n      translation: 0.0\n";
  }
  EXPECT_THROW(uw::runtime::LoadPlatformDefaultsConfig(tmp_path.string()), std::runtime_error);
  std::remove(tmp_path.string().c_str());
}

TEST(Config, RejectsNegativeRelativePoseRotationCap) {
  const auto tmp_path = std::filesystem::temp_directory_path() / "uw_config_test_relpose_neg.yaml";
  {
    std::ofstream out(tmp_path);
    out << "reliability:\n  default_sqrt_information:\n    relative_pose:\n      rotation: -1.0\n";
  }
  EXPECT_THROW(uw::runtime::LoadPlatformDefaultsConfig(tmp_path.string()), std::runtime_error);
  std::remove(tmp_path.string().c_str());
}

TEST(Config, RejectsOldScalarRelativePoseFormat) {
  const auto tmp_path = std::filesystem::temp_directory_path() / "uw_config_test_relpose_old.yaml";
  {
    std::ofstream out(tmp_path);
    out << "reliability:\n  default_sqrt_information:\n    relative_pose: 20.0\n";
  }
  EXPECT_THROW(uw::runtime::LoadPlatformDefaultsConfig(tmp_path.string()), std::runtime_error);
  std::remove(tmp_path.string().c_str());
}

TEST(Config, ParsesStereoMatchingOverrides) {
  const auto tmp_path = std::filesystem::temp_directory_path() / "uw_config_test_stereo_matching.yaml";
  {
    std::ofstream out(tmp_path);
    out << "stereo_matching:\n"
           "  min_texture_variance: 10.0\n"
           "  min_uniqueness_margin: 1.5\n"
           "  left_right_max_diff_px: 2.0\n";
  }
  const auto config = uw::runtime::LoadPlatformDefaultsConfig(tmp_path.string());
  EXPECT_DOUBLE_EQ(config.stereo_matching.min_texture_variance, 10.0);
  EXPECT_DOUBLE_EQ(config.stereo_matching.min_uniqueness_margin, 1.5);
  EXPECT_DOUBLE_EQ(config.stereo_matching.left_right_max_diff_px, 2.0);
  std::remove(tmp_path.string().c_str());
}

TEST(Config, StereoMatchingDefaultsWhenSectionAbsent) {
  const auto config = uw::runtime::PlatformDefaultsConfig{};
  EXPECT_DOUBLE_EQ(config.stereo_matching.min_texture_variance, 25.0);
  EXPECT_DOUBLE_EQ(config.stereo_matching.min_uniqueness_margin, 2.0);
  EXPECT_DOUBLE_EQ(config.stereo_matching.left_right_max_diff_px, 1.0);
}

TEST(Config, RejectsNegativeStereoMatchingTextureVariance) {
  const auto tmp_path = std::filesystem::temp_directory_path() / "uw_config_test_stereo_var_neg.yaml";
  {
    std::ofstream out(tmp_path);
    out << "stereo_matching:\n  min_texture_variance: -1.0\n";
  }
  EXPECT_THROW(uw::runtime::LoadPlatformDefaultsConfig(tmp_path.string()), std::runtime_error);
  std::remove(tmp_path.string().c_str());
}

TEST(Config, RejectsNegativeStereoMatchingUniquenessMargin) {
  const auto tmp_path = std::filesystem::temp_directory_path() / "uw_config_test_stereo_margin_neg.yaml";
  {
    std::ofstream out(tmp_path);
    out << "stereo_matching:\n  min_uniqueness_margin: -0.5\n";
  }
  EXPECT_THROW(uw::runtime::LoadPlatformDefaultsConfig(tmp_path.string()), std::runtime_error);
  std::remove(tmp_path.string().c_str());
}

TEST(Config, RejectsNonFiniteStereoMatchingLeftRightThreshold) {
  const auto tmp_path = std::filesystem::temp_directory_path() / "uw_config_test_stereo_lr_bad.yaml";
  {
    std::ofstream out(tmp_path);
    out << "stereo_matching:\n  left_right_max_diff_px: -1.0\n";
  }
  EXPECT_THROW(uw::runtime::LoadPlatformDefaultsConfig(tmp_path.string()), std::runtime_error);
  std::remove(tmp_path.string().c_str());
}

TEST(Config, RejectsStereoMatchingUnknownKey) {
  const auto tmp_path = std::filesystem::temp_directory_path() / "uw_config_test_stereo_unknown.yaml";
  {
    std::ofstream out(tmp_path);
    out << "stereo_matching:\n  bogus_key: 1\n";
  }
  EXPECT_THROW(uw::runtime::LoadPlatformDefaultsConfig(tmp_path.string()), std::runtime_error);
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

TEST(Config, AcceptsExplicitMeasuredZeroOffsetsForAllOnlineRoles) {
  const auto path = WriteRig(
      "uw_online_rig_zero.yaml",
      MinimalOnlineRigYaml("  camera_left: 0.0\n  camera_right: 0.0\n  sonar0: 0.0\n  rov-state: 0.0\n",
                           "  camera_left: measured:left\n  camera_right: measured:right\n"
                           "  sonar0: measured:sonar\n  rov-state: measured:state\n"));
  const auto rig = uw::runtime::LoadRigConfig(path.string());
  EXPECT_DOUBLE_EQ(rig.time_offset_seconds().at("rov-state"), 0.0);
  EXPECT_EQ(rig.vehicle_state_sensors(0).value(), "rov-state");
  std::remove(path.string().c_str());
}

TEST(Config, RejectsMissingOffsetOrProvenanceForDeclaredOnlineSensor) {
  const auto missing_offset = WriteRig(
      "uw_online_rig_missing_offset.yaml",
      MinimalOnlineRigYaml("  camera_left: 0.0\n  camera_right: 0.0\n  sonar0: 0.0\n",
                           "  camera_left: measured:left\n  camera_right: measured:right\n"
                           "  sonar0: measured:sonar\n  rov-state: measured:state\n"));
  EXPECT_THROW(uw::runtime::LoadRigConfig(missing_offset.string()), std::runtime_error);
  std::remove(missing_offset.string().c_str());

  const auto missing_provenance = WriteRig(
      "uw_online_rig_missing_provenance.yaml",
      MinimalOnlineRigYaml("  camera_left: 0.0\n  camera_right: 0.0\n  sonar0: 0.0\n  rov-state: 0.0\n",
                           "  camera_left: measured:left\n  camera_right: measured:right\n"
                           "  sonar0: measured:sonar\n"));
  EXPECT_THROW(uw::runtime::LoadRigConfig(missing_provenance.string()), std::runtime_error);
  std::remove(missing_provenance.string().c_str());
}

TEST(Config, RejectsDuplicateEmptyOrUnknownVehicleTimingRoles) {
  const std::string offsets =
      "  camera_left: 0.0\n  camera_right: 0.0\n  sonar0: 0.0\n  rov-state: 0.0\n";
  const std::string provenance =
      "  camera_left: measured:left\n  camera_right: measured:right\n"
      "  sonar0: measured:sonar\n  rov-state: measured:state\n";
  const auto duplicate = WriteRig("uw_online_rig_duplicate.yaml",
                                  MinimalOnlineRigYaml(offsets, provenance,
                                                       "  - rov-state\n  - rov-state\n"));
  EXPECT_THROW(uw::runtime::LoadRigConfig(duplicate.string()), std::runtime_error);
  std::remove(duplicate.string().c_str());

  const auto empty = WriteRig("uw_online_rig_empty.yaml",
                              MinimalOnlineRigYaml(offsets, provenance, "  - \"\"\n"));
  EXPECT_THROW(uw::runtime::LoadRigConfig(empty.string()), std::runtime_error);
  std::remove(empty.string().c_str());

  const auto unknown = WriteRig(
      "uw_online_rig_unknown_provenance.yaml",
      MinimalOnlineRigYaml(offsets,
                           provenance + "  ghost: measured:unknown\n"));
  EXPECT_THROW(uw::runtime::LoadRigConfig(unknown.string()), std::runtime_error);
  std::remove(unknown.string().c_str());
}

TEST(Config, RejectsNonFiniteOnlineOffsetAndEmptyProvenance) {
  const auto nonfinite = WriteRig(
      "uw_online_rig_nonfinite.yaml",
      MinimalOnlineRigYaml("  camera_left: .inf\n  camera_right: 0.0\n  sonar0: 0.0\n  rov-state: 0.0\n",
                           "  camera_left: measured:left\n  camera_right: measured:right\n"
                           "  sonar0: measured:sonar\n  rov-state: measured:state\n"));
  EXPECT_THROW(uw::runtime::LoadRigConfig(nonfinite.string()), std::runtime_error);
  std::remove(nonfinite.string().c_str());

  const auto empty = WriteRig(
      "uw_online_rig_empty_provenance.yaml",
      MinimalOnlineRigYaml("  camera_left: 0.0\n  camera_right: 0.0\n  sonar0: 0.0\n  rov-state: 0.0\n",
                           "  camera_left: \"\"\n  camera_right: measured:right\n"
                           "  sonar0: measured:sonar\n  rov-state: measured:state\n"));
  EXPECT_THROW(uw::runtime::LoadRigConfig(empty.string()), std::runtime_error);
  std::remove(empty.string().c_str());
}

TEST(Config, FullAcousticOpticRigRequiresExactlyOneVehicleStateSensor) {
  const std::string offsets =
      "  camera_left: 0.0\n  camera_right: 0.0\n  sonar0: 0.0\n  rov-state: 0.0\n"
      "  backup-state: 0.0\n";
  const std::string provenance =
      "  camera_left: measured:left\n  camera_right: measured:right\n"
      "  sonar0: measured:sonar\n  rov-state: measured:state\n"
      "  backup-state: measured:backup\n";
  const auto missing = WriteRig("uw_online_rig_no_state_role.yaml",
                                 MinimalOnlineRigYaml(offsets, provenance, ""));
  EXPECT_THROW(uw::runtime::LoadRigConfig(missing.string()), std::runtime_error);
  std::remove(missing.string().c_str());

  const auto multiple = WriteRig(
      "uw_online_rig_multiple_state_roles.yaml",
      MinimalOnlineRigYaml(offsets, provenance, "  - rov-state\n  - backup-state\n"));
  EXPECT_THROW(uw::runtime::LoadRigConfig(multiple.string()), std::runtime_error);
  std::remove(multiple.string().c_str());
}

TEST(Config, SonarRigsRequireExactlyOneEnabledSonarAndVehicleStateSource) {
  const std::string offsets =
      "  sonar0: 0.0\n  sonar1: 0.0\n  rov-state: 0.0\n  backup-state: 0.0\n";
  const std::string provenance =
      "  sonar0: measured:sonar\n  sonar1: measured:sonar\n"
      "  rov-state: measured:state\n  backup-state: measured:backup\n";
  const auto make_sonar_rig = [&](const std::string& name, const std::string& sonars,
                                  const std::string& states) {
    return WriteRig(name,
                    "calibration_version: sonar_v1\nsonar_beam_models:\n" + sonars +
                        "vehicle_state_sensors:\n" + states +
                        "time_offset_seconds:\n" + offsets +
                        "time_offset_provenance:\n" + provenance);
  };

  const auto no_state = make_sonar_rig(
      "uw_sonar_rig_no_state.yaml", "  - sensor_id: sonar0\n    sonar_enabled: true\n", "");
  EXPECT_THROW(uw::runtime::LoadRigConfig(no_state.string()), std::runtime_error);
  std::remove(no_state.string().c_str());

  const auto multiple_state = make_sonar_rig(
      "uw_sonar_rig_multiple_state.yaml", "  - sensor_id: sonar0\n    sonar_enabled: true\n",
      "  - rov-state\n  - backup-state\n");
  EXPECT_THROW(uw::runtime::LoadRigConfig(multiple_state.string()), std::runtime_error);
  std::remove(multiple_state.string().c_str());

  const auto multiple_sonar = make_sonar_rig(
      "uw_sonar_rig_multiple_sonar.yaml",
      "  - sensor_id: sonar0\n    sonar_enabled: true\n"
      "  - sensor_id: sonar1\n    sonar_enabled: true\n",
      "  - rov-state\n");
  EXPECT_THROW(uw::runtime::LoadRigConfig(multiple_sonar.string()), std::runtime_error);
  std::remove(multiple_sonar.string().c_str());
}

TEST(Config, RejectsHugeFiniteTimeOffset) {
  const auto path = WriteRig(
      "uw_online_rig_huge_offset.yaml",
      MinimalOnlineRigYaml(
          "  camera_left: 10.000001\n  camera_right: 0.0\n  sonar0: 0.0\n  rov-state: 0.0\n",
          "  camera_left: measured:left\n  camera_right: measured:right\n"
          "  sonar0: measured:sonar\n  rov-state: measured:state\n"));
  EXPECT_THROW(uw::runtime::LoadRigConfig(path.string()), std::runtime_error);
  std::remove(path.string().c_str());
}

TEST(Config, ValidateExperimentConfigSelectionsAcceptsDefaults) {
  // Default-constructed ExperimentConfig must always validate — every
  // configs/experiment/*.yaml either uses these defaults or overrides them
  // with an equally-recognized value (see the loaded-from-file test below).
  const uw::runtime::ExperimentConfig config;
  EXPECT_EQ(uw::runtime::ValidateExperimentConfigSelections(config), std::nullopt);
}

TEST(Config, ValidateExperimentConfigSelectionsAcceptsRealExperimentFiles) {
  for (const std::string name : {"synthetic_smoke.yaml", "synthetic_smoke_vo.yaml", "acoustic_optic_demo.yaml",
                                  "real_holoocean_vo.yaml"}) {
    const auto config =
        uw::runtime::LoadExperimentConfig(std::string(UW_REPO_ROOT) + "/configs/experiment/" + name);
    EXPECT_EQ(uw::runtime::ValidateExperimentConfigSelections(config), std::nullopt) << "experiment file: " << name;
  }
}

TEST(Config, ValidateExperimentConfigSelectionsRejectsUnknownSonarFrontend) {
  uw::runtime::ExperimentConfig config;
  config.sonar_frontend = "does_not_exist_v1";
  const auto error = uw::runtime::ValidateExperimentConfigSelections(config);
  ASSERT_NE(error, std::nullopt);
  EXPECT_NE(error->find("sonar_frontend"), std::string::npos);
}

TEST(Config, ValidateExperimentConfigSelectionsRejectsUnknownOpticalFrontend) {
  uw::runtime::ExperimentConfig config;
  config.optical_frontend = "does_not_exist_v1";
  const auto error = uw::runtime::ValidateExperimentConfigSelections(config);
  ASSERT_NE(error, std::nullopt);
  EXPECT_NE(error->find("optical_frontend"), std::string::npos);
}

TEST(Config, ValidateExperimentConfigSelectionsRejectsUnknownMapBackend) {
  uw::runtime::ExperimentConfig config;
  config.map_backend = "does_not_exist_v1";
  const auto error = uw::runtime::ValidateExperimentConfigSelections(config);
  ASSERT_NE(error, std::nullopt);
  EXPECT_NE(error->find("map_backend"), std::string::npos);
}

TEST(Config, ValidateExperimentConfigSelectionsRejectsUnknownEstimatorMode) {
  uw::runtime::ExperimentConfig config;
  config.estimator_mode = "does_not_exist";
  const auto error = uw::runtime::ValidateExperimentConfigSelections(config);
  ASSERT_NE(error, std::nullopt);
  EXPECT_NE(error->find("estimator_mode"), std::string::npos);
}

TEST(Config, ValidateExperimentConfigSelectionsRejectsUnknownLandmarkDetector) {
  uw::runtime::ExperimentConfig config;
  config.landmark_detector = "does_not_exist";
  const auto error = uw::runtime::ValidateExperimentConfigSelections(config);
  ASSERT_NE(error, std::nullopt);
  EXPECT_NE(error->find("landmark_detector"), std::string::npos);
}

TEST(Config, ValidateExperimentConfigSelectionsRejectsUnknownSolver) {
  uw::runtime::ExperimentConfig config;
  config.defaults.solver = "does_not_exist";
  const auto error = uw::runtime::ValidateExperimentConfigSelections(config);
  ASSERT_NE(error, std::nullopt);
  EXPECT_NE(error->find("solver"), std::string::npos);
}

TEST(Config, ValidateExperimentConfigSelectionsAcceptsCeresV1Solver) {
  uw::runtime::ExperimentConfig config;
  config.defaults.solver = "ceres_v1";
  EXPECT_EQ(uw::runtime::ValidateExperimentConfigSelections(config), std::nullopt);
}

TEST(Config, LoadsAssociationAndTrackerDefaultsUsedByFusion) {
  const auto config = uw::runtime::LoadPlatformDefaultsConfig(
      std::string(UW_REPO_ROOT) + "/configs/defaults/platform.yaml");
  EXPECT_DOUBLE_EQ(config.target_association.max_corrected_time_delta_s, 0.05);
  EXPECT_DOUBLE_EQ(config.target_association.max_bearing_mahalanobis_sq, 9.0);
  EXPECT_DOUBLE_EQ(config.target_association.max_range_mahalanobis_sq, 9.0);
  EXPECT_DOUBLE_EQ(config.target_tracker.stale_after_s, 0.5);
  EXPECT_EQ(config.target_tracker.confirm_hits, 2);
  EXPECT_EQ(config.target_tracker.degraded_misses, 3);
  EXPECT_NE(config.target_fusion_manifest_parameters().find("stale_after_s=0.5"),
            std::string::npos);
}

TEST(Config, RejectsUnsafeAssociationAndTrackerThresholds) {
  const auto path = std::filesystem::temp_directory_path() / "uw_bad_target_fusion.yaml";
  {
    std::ofstream out(path);
    out << "frontends:\n"
           "  target_association:\n"
           "    max_corrected_time_delta_s: .nan\n"
           "  target_tracker:\n"
           "    stale_after_s: 0.0\n";
  }
  EXPECT_THROW(uw::runtime::LoadPlatformDefaultsConfig(path.string()), std::runtime_error);
  std::remove(path.string().c_str());
}

// estimation.solver at the experiment-file level (not nested under a
// separate defaults/*.yaml) — see docs/superpowers/specs/2026-08-23-solver-
// and-mapping-oss-adoption.md §7 and configs/README.md's "第三个例外".
TEST(Config, ExperimentLevelEstimationSolverOverridesTheDefault) {
  const auto path = std::filesystem::temp_directory_path() / "uw_solver_override_experiment.yaml";
  {
    std::ofstream out(path);
    out << "estimation:\n"
           "  solver: ceres_v1\n";
  }
  const auto config = uw::runtime::LoadExperimentConfig(path.string());
  EXPECT_EQ(config.defaults.solver, "ceres_v1");
  std::remove(path.string().c_str());
}
