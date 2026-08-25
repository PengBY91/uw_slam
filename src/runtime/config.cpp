#include "runtime/config.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <filesystem>
#include <set>
#include <stdexcept>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace uw::runtime {

namespace {

void RejectUnknownKeys(const YAML::Node& node, const std::vector<std::string>& allowed,
                       const std::string& context) {
  if (!node || !node.IsMap()) return;
  for (const auto& entry : node) {
    const std::string key = entry.first.as<std::string>();
    if (std::find(allowed.begin(), allowed.end(), key) == allowed.end()) {
      throw std::runtime_error("unknown key '" + key + "' in " + context);
    }
  }
}

std::string ResolveRelative(const std::string& base_dir, const std::string& maybe_relative) {
  std::filesystem::path p(maybe_relative);
  if (p.is_absolute()) return maybe_relative;
  return (std::filesystem::path(base_dir) / p).string();
}

template <typename T>
T GetOr(const YAML::Node& node, const char* key, T fallback) {
  if (!node || !node[key] || node[key].IsNull()) return fallback;
  return node[key].as<T>();
}

void RequireSequenceLength(const YAML::Node& node, const char* field,
                           std::size_t expected, const std::string& path) {
  if (!node[field] || !node[field].IsSequence() || node[field].size() != expected) {
    throw std::runtime_error(std::string(field) + " must contain exactly " +
                             std::to_string(expected) + " values: " + path);
  }
}

}  // namespace

PlatformDefaultsConfig LoadPlatformDefaultsConfig(const std::string& path) {
  const YAML::Node root = YAML::LoadFile(path);
  PlatformDefaultsConfig config;

  const auto estimation = root["estimation"];
  config.solver = GetOr<std::string>(estimation, "solver", config.solver);
  config.max_iterations = GetOr<int>(estimation, "max_iterations", config.max_iterations);
  config.initial_lambda = GetOr<double>(estimation, "initial_lambda", config.initial_lambda);
  config.warmup_seconds = GetOr<double>(estimation, "warmup_seconds", config.warmup_seconds);

  const auto gates = root["gates"];
  config.require_converged = GetOr<bool>(gates, "require_converged", config.require_converged);
  config.max_ate_rmse_m = GetOr<double>(gates, "max_ate_rmse_m", config.max_ate_rmse_m);
  config.min_matched_ate_poses =
      GetOr<int>(gates, "min_matched_ate_poses", config.min_matched_ate_poses);
  config.require_nonempty_map =
      GetOr<bool>(gates, "require_nonempty_map", config.require_nonempty_map);
  config.min_acoustic_optic_accepted =
      GetOr<int>(gates, "min_acoustic_optic_accepted", config.min_acoustic_optic_accepted);
  config.min_acoustic_optic_map_points =
      GetOr<int>(gates, "min_acoustic_optic_map_points", config.min_acoustic_optic_map_points);

  if (root["frontends"] && root["frontends"]["stereo_rectification"]) {
    const auto sr = root["frontends"]["stereo_rectification"];
    RejectUnknownKeys(sr, {"alpha", "crop_policy", "frame_suffix"}, "frontends.stereo_rectification");
    config.stereo_rectification.alpha = GetOr<double>(sr, "alpha", config.stereo_rectification.alpha);
    config.stereo_rectification.crop_policy =
        GetOr<std::string>(sr, "crop_policy", config.stereo_rectification.crop_policy);
    config.stereo_rectification.frame_suffix =
        GetOr<std::string>(sr, "frame_suffix", config.stereo_rectification.frame_suffix);

    if (!std::isfinite(config.stereo_rectification.alpha) || config.stereo_rectification.alpha < -1.0 ||
        config.stereo_rectification.alpha > 1.0) {
      throw std::runtime_error("frontends.stereo_rectification.alpha must be finite and in [-1, 1]");
    }
    if (config.stereo_rectification.crop_policy != "full_canvas" &&
        config.stereo_rectification.crop_policy != "common_valid_roi") {
      throw std::runtime_error(
          "frontends.stereo_rectification.crop_policy must be 'full_canvas' or 'common_valid_roi'");
    }
    if (config.stereo_rectification.frame_suffix.empty()) {
      throw std::runtime_error("frontends.stereo_rectification.frame_suffix must not be empty");
    }
  }

  if (root["frontends"] && root["frontends"]["sonar_cfar"]) {
    const auto sonar = root["frontends"]["sonar_cfar"];
    RejectUnknownKeys(sonar,
                      {"training_cells", "guard_cells", "probability_false_alarm",
                       "detector_threshold", "dbscan_eps_m", "dbscan_min_samples",
                       "default_range_sigma_m", "default_bearing_sigma_rad"},
                      "frontends.sonar_cfar");
    config.sonar_frontend.training_cells =
        GetOr<int>(sonar, "training_cells", config.sonar_frontend.training_cells);
    config.sonar_frontend.guard_cells =
        GetOr<int>(sonar, "guard_cells", config.sonar_frontend.guard_cells);
    config.sonar_frontend.probability_false_alarm = GetOr<double>(
        sonar, "probability_false_alarm", config.sonar_frontend.probability_false_alarm);
    config.sonar_frontend.detector_threshold =
        GetOr<int>(sonar, "detector_threshold", config.sonar_frontend.detector_threshold);
    config.sonar_frontend.dbscan_eps_m =
        GetOr<double>(sonar, "dbscan_eps_m", config.sonar_frontend.dbscan_eps_m);
    config.sonar_frontend.dbscan_min_samples =
        GetOr<int>(sonar, "dbscan_min_samples", config.sonar_frontend.dbscan_min_samples);
    config.sonar_frontend.default_range_sigma_m = GetOr<double>(
        sonar, "default_range_sigma_m", config.sonar_frontend.default_range_sigma_m);
    config.sonar_frontend.default_bearing_sigma_rad = GetOr<double>(
        sonar, "default_bearing_sigma_rad", config.sonar_frontend.default_bearing_sigma_rad);

    if (config.sonar_frontend.training_cells <= 0 ||
        config.sonar_frontend.training_cells % 2 != 0) {
      throw std::runtime_error("frontends.sonar_cfar.training_cells must be positive and even");
    }
    if (config.sonar_frontend.guard_cells < 0 || config.sonar_frontend.guard_cells % 2 != 0) {
      throw std::runtime_error("frontends.sonar_cfar.guard_cells must be non-negative and even");
    }
    if (!std::isfinite(config.sonar_frontend.probability_false_alarm) ||
        config.sonar_frontend.probability_false_alarm <= 0.0 ||
        config.sonar_frontend.probability_false_alarm >= 1.0) {
      throw std::runtime_error(
          "frontends.sonar_cfar.probability_false_alarm must be finite and in (0, 1)");
    }
    if (config.sonar_frontend.detector_threshold < 0 ||
        config.sonar_frontend.detector_threshold > 255) {
      throw std::runtime_error("frontends.sonar_cfar.detector_threshold must be in [0, 255]");
    }
    if (!std::isfinite(config.sonar_frontend.dbscan_eps_m) ||
        config.sonar_frontend.dbscan_eps_m <= 0.0) {
      throw std::runtime_error("frontends.sonar_cfar.dbscan_eps_m must be finite and positive");
    }
    if (config.sonar_frontend.dbscan_min_samples <= 0) {
      throw std::runtime_error("frontends.sonar_cfar.dbscan_min_samples must be positive");
    }
    if (!std::isfinite(config.sonar_frontend.default_range_sigma_m) ||
        config.sonar_frontend.default_range_sigma_m <= 0.0) {
      throw std::runtime_error(
          "frontends.sonar_cfar.default_range_sigma_m must be finite and positive");
    }
    if (!std::isfinite(config.sonar_frontend.default_bearing_sigma_rad) ||
        config.sonar_frontend.default_bearing_sigma_rad <= 0.0) {
      throw std::runtime_error(
          "frontends.sonar_cfar.default_bearing_sigma_rad must be finite and positive");
    }
  }

  if (root["frontends"] && root["frontends"]["target_association"]) {
    const auto association = root["frontends"]["target_association"];
    RejectUnknownKeys(association,
                      {"max_corrected_time_delta_s", "max_bearing_mahalanobis_sq",
                       "max_range_mahalanobis_sq", "max_motion_bearing_delta_rad",
                       "max_motion_rate_rad_s", "max_bearing_variance_rad2",
                       "max_range_variance_m2"},
                      "frontends.target_association");
    auto& out = config.target_association;
    out.max_corrected_time_delta_s = GetOr<double>(
        association, "max_corrected_time_delta_s", out.max_corrected_time_delta_s);
    out.max_bearing_mahalanobis_sq = GetOr<double>(
        association, "max_bearing_mahalanobis_sq", out.max_bearing_mahalanobis_sq);
    out.max_range_mahalanobis_sq = GetOr<double>(
        association, "max_range_mahalanobis_sq", out.max_range_mahalanobis_sq);
    out.max_motion_bearing_delta_rad = GetOr<double>(
        association, "max_motion_bearing_delta_rad", out.max_motion_bearing_delta_rad);
    out.max_motion_rate_rad_s =
        GetOr<double>(association, "max_motion_rate_rad_s", out.max_motion_rate_rad_s);
    out.max_bearing_variance_rad2 = GetOr<double>(
        association, "max_bearing_variance_rad2", out.max_bearing_variance_rad2);
    out.max_range_variance_m2 = GetOr<double>(
        association, "max_range_variance_m2", out.max_range_variance_m2);
    const std::vector<std::pair<const char*, double>> positive = {
        {"max_corrected_time_delta_s", out.max_corrected_time_delta_s},
        {"max_bearing_mahalanobis_sq", out.max_bearing_mahalanobis_sq},
        {"max_range_mahalanobis_sq", out.max_range_mahalanobis_sq},
        {"max_motion_bearing_delta_rad", out.max_motion_bearing_delta_rad},
        {"max_bearing_variance_rad2", out.max_bearing_variance_rad2},
        {"max_range_variance_m2", out.max_range_variance_m2}};
    for (const auto& [name, value] : positive) {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::runtime_error(std::string("frontends.target_association.") + name +
                                 " must be finite and positive");
      }
    }
    if (!std::isfinite(out.max_motion_rate_rad_s) || out.max_motion_rate_rad_s < 0.0) {
      throw std::runtime_error(
          "frontends.target_association.max_motion_rate_rad_s must be finite and non-negative");
    }
  }

  if (root["frontends"] && root["frontends"]["target_tracker"]) {
    const auto tracker = root["frontends"]["target_tracker"];
    RejectUnknownKeys(tracker,
                      {"association_mahalanobis_sq", "confirm_hits", "degraded_misses",
                       "stale_after_s", "max_prediction_dt_s",
                       "bearing_acceleration_noise", "range_acceleration_noise",
                       "merge_bearing_threshold_rad", "merge_range_threshold_m"},
                      "frontends.target_tracker");
    auto& out = config.target_tracker;
    out.association_mahalanobis_sq = GetOr<double>(
        tracker, "association_mahalanobis_sq", out.association_mahalanobis_sq);
    out.confirm_hits = GetOr<int>(tracker, "confirm_hits", out.confirm_hits);
    out.degraded_misses = GetOr<int>(tracker, "degraded_misses", out.degraded_misses);
    out.stale_after_s = GetOr<double>(tracker, "stale_after_s", out.stale_after_s);
    out.max_prediction_dt_s =
        GetOr<double>(tracker, "max_prediction_dt_s", out.max_prediction_dt_s);
    out.bearing_acceleration_noise = GetOr<double>(
        tracker, "bearing_acceleration_noise", out.bearing_acceleration_noise);
    out.range_acceleration_noise = GetOr<double>(
        tracker, "range_acceleration_noise", out.range_acceleration_noise);
    out.merge_bearing_threshold_rad = GetOr<double>(
        tracker, "merge_bearing_threshold_rad", out.merge_bearing_threshold_rad);
    out.merge_range_threshold_m = GetOr<double>(
        tracker, "merge_range_threshold_m", out.merge_range_threshold_m);
    if (out.confirm_hits < 1 || out.degraded_misses < 1) {
      throw std::runtime_error(
          "frontends.target_tracker confirm_hits/degraded_misses must be positive");
    }
    const std::vector<std::pair<const char*, double>> positive = {
        {"association_mahalanobis_sq", out.association_mahalanobis_sq},
        {"stale_after_s", out.stale_after_s},
        {"max_prediction_dt_s", out.max_prediction_dt_s},
        {"bearing_acceleration_noise", out.bearing_acceleration_noise},
        {"range_acceleration_noise", out.range_acceleration_noise},
        {"merge_bearing_threshold_rad", out.merge_bearing_threshold_rad},
        {"merge_range_threshold_m", out.merge_range_threshold_m}};
    for (const auto& [name, value] : positive) {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::runtime_error(std::string("frontends.target_tracker.") + name +
                                 " must be finite and positive");
      }
    }
  }

  if (root["reliability"] && root["reliability"]["default_sqrt_information"]) {
    const auto info = root["reliability"]["default_sqrt_information"];
    if (info["relative_pose"]) {
      const auto relative_pose = info["relative_pose"];
      if (!relative_pose.IsMap()) {
        throw std::runtime_error(
            "reliability.default_sqrt_information.relative_pose must be a mapping with "
            "translation/rotation keys (e.g. 'relative_pose: {translation: 20.0, rotation: 20.0}'), "
            "not a single scalar -- translation and rotation now take independent caps");
      }
      RejectUnknownKeys(relative_pose, {"translation", "rotation"},
                        "reliability.default_sqrt_information.relative_pose");
      config.default_sqrt_information.relative_pose.translation = GetOr<double>(
          relative_pose, "translation", config.default_sqrt_information.relative_pose.translation);
      config.default_sqrt_information.relative_pose.rotation = GetOr<double>(
          relative_pose, "rotation", config.default_sqrt_information.relative_pose.rotation);
      if (!std::isfinite(config.default_sqrt_information.relative_pose.translation) ||
          config.default_sqrt_information.relative_pose.translation <= 0.0) {
        throw std::runtime_error(
            "reliability.default_sqrt_information.relative_pose.translation must be finite and positive");
      }
      if (!std::isfinite(config.default_sqrt_information.relative_pose.rotation) ||
          config.default_sqrt_information.relative_pose.rotation <= 0.0) {
        throw std::runtime_error(
            "reliability.default_sqrt_information.relative_pose.rotation must be finite and positive");
      }
    }
    config.default_sqrt_information.sonar_range =
        GetOr<double>(info, "sonar_range", config.default_sqrt_information.sonar_range);
    config.default_sqrt_information.depth =
        GetOr<double>(info, "depth", config.default_sqrt_information.depth);
  }

  if (root["visual_odometry"]) {
    const auto vo = root["visual_odometry"];
    RejectUnknownKeys(
        vo, {"max_consecutive_failures", "max_condition_number", "residual_variance_floor_m2", "max_inlier_rmse_m"},
        "visual_odometry");
    config.visual_odometry.max_consecutive_failures =
        GetOr<int>(vo, "max_consecutive_failures", config.visual_odometry.max_consecutive_failures);
    if (config.visual_odometry.max_consecutive_failures < 1 ||
        config.visual_odometry.max_consecutive_failures > 1000) {
      throw std::runtime_error("visual_odometry.max_consecutive_failures must be in [1, 1000]");
    }
    config.visual_odometry.max_condition_number =
        GetOr<double>(vo, "max_condition_number", config.visual_odometry.max_condition_number);
    if (!std::isfinite(config.visual_odometry.max_condition_number) ||
        config.visual_odometry.max_condition_number <= 0.0) {
      throw std::runtime_error("visual_odometry.max_condition_number must be finite and positive");
    }
    config.visual_odometry.residual_variance_floor_m2 = GetOr<double>(
        vo, "residual_variance_floor_m2", config.visual_odometry.residual_variance_floor_m2);
    if (!std::isfinite(config.visual_odometry.residual_variance_floor_m2) ||
        config.visual_odometry.residual_variance_floor_m2 <= 0.0) {
      throw std::runtime_error("visual_odometry.residual_variance_floor_m2 must be finite and positive");
    }
    config.visual_odometry.max_inlier_rmse_m =
        GetOr<double>(vo, "max_inlier_rmse_m", config.visual_odometry.max_inlier_rmse_m);
    if (std::isnan(config.visual_odometry.max_inlier_rmse_m)) {
      throw std::runtime_error("visual_odometry.max_inlier_rmse_m must not be NaN");
    }
    // <=0 means disabled (matches PlatformDefaultsConfig's gate convention)
    // -- map to +inf so CovarianceEstimationParams::max_inlier_rmse_m's
    // comparison never rejects.
    if (config.visual_odometry.max_inlier_rmse_m <= 0.0) {
      config.visual_odometry.max_inlier_rmse_m = std::numeric_limits<double>::infinity();
    }
  }

  if (root["stereo_matching"]) {
    const auto sm = root["stereo_matching"];
    RejectUnknownKeys(sm, {"min_texture_variance", "min_uniqueness_margin", "left_right_max_diff_px"},
                      "stereo_matching");
    config.stereo_matching.min_texture_variance =
        GetOr<double>(sm, "min_texture_variance", config.stereo_matching.min_texture_variance);
    if (!std::isfinite(config.stereo_matching.min_texture_variance) ||
        config.stereo_matching.min_texture_variance < 0.0) {
      throw std::runtime_error("stereo_matching.min_texture_variance must be finite and >= 0");
    }
    config.stereo_matching.min_uniqueness_margin =
        GetOr<double>(sm, "min_uniqueness_margin", config.stereo_matching.min_uniqueness_margin);
    if (!std::isfinite(config.stereo_matching.min_uniqueness_margin) ||
        config.stereo_matching.min_uniqueness_margin < 0.0) {
      throw std::runtime_error("stereo_matching.min_uniqueness_margin must be finite and >= 0");
    }
    config.stereo_matching.left_right_max_diff_px =
        GetOr<double>(sm, "left_right_max_diff_px", config.stereo_matching.left_right_max_diff_px);
    if (!std::isfinite(config.stereo_matching.left_right_max_diff_px) ||
        config.stereo_matching.left_right_max_diff_px < 0.0) {
      throw std::runtime_error("stereo_matching.left_right_max_diff_px must be finite and >= 0");
    }
  }
  return config;
}

uw::domain::RigCalibrationSnapshot LoadRigConfig(const std::string& path) {
  const YAML::Node root = YAML::LoadFile(path);
  uw::domain::RigCalibrationSnapshot snapshot;

  snapshot.mutable_calibration_version()->set_value(
      GetOr<std::string>(root, "calibration_version", std::string("unversioned")));

  if (root["frame_tree"]) {
    for (const auto& edge_node : root["frame_tree"]) {
      auto* edge = snapshot.add_frame_tree();
      edge->mutable_parent_frame()->set_value(edge_node["parent_frame"].as<std::string>());
      edge->mutable_child_frame()->set_value(edge_node["child_frame"].as<std::string>());
      RequireSequenceLength(edge_node, "transform_row_major", 16, path);
      auto* transform = edge->mutable_transform();
      for (const auto& value : edge_node["transform_row_major"]) {
        transform->add_matrix_row_major(value.as<double>());
      }
    }
  }

  if (root["imu_noise"]) {
    const auto imu = root["imu_noise"];
    auto* noise = snapshot.mutable_imu_noise();
    noise->set_sigma_gyro_c(GetOr<double>(imu, "sigma_gyro_c", 0.0));
    noise->set_sigma_accel_c(GetOr<double>(imu, "sigma_accel_c", 0.0));
    noise->set_sigma_gyro_bias(GetOr<double>(imu, "sigma_gyro_bias", 0.0));
    noise->set_sigma_accel_bias(GetOr<double>(imu, "sigma_accel_bias", 0.0));
    noise->set_sigma_gyro_bias_walk_c(GetOr<double>(imu, "sigma_gyro_bias_walk_c", 0.0));
    noise->set_sigma_accel_bias_walk_c(GetOr<double>(imu, "sigma_accel_bias_walk_c", 0.0));
    noise->set_rate_hz(GetOr<double>(imu, "rate_hz", 200.0));
    noise->set_gravity_mps2(GetOr<double>(imu, "gravity_mps2", 9.80665));
  }

  if (root["cameras"]) {
    for (const auto& camera_node : root["cameras"]) {
      auto* camera = snapshot.add_cameras();
      camera->mutable_sensor_id()->set_value(camera_node["sensor_id"].as<std::string>());
      camera->set_width(camera_node["width"].as<uint32_t>());
      camera->set_height(camera_node["height"].as<uint32_t>());
      RequireSequenceLength(camera_node, "k_matrix_row_major", 9, path);
      if (camera_node["width"].as<uint32_t>() == 0 ||
          camera_node["height"].as<uint32_t>() == 0) {
        throw std::runtime_error("camera width/height must be non-zero: " + path);
      }
      for (const auto& value : camera_node["k_matrix_row_major"]) {
        camera->add_k_matrix_row_major(value.as<double>());
      }
      if (camera_node["distortion"]) {
        for (const auto& value : camera_node["distortion"]) {
          camera->add_distortion(value.as<double>());
        }
      }
      camera->set_distortion_model(
          GetOr<std::string>(camera_node, "distortion_model", std::string("plumb_bob")));
    }
  }

  if (root["time_offset_seconds"]) {
    for (const auto& entry : root["time_offset_seconds"]) {
      (*snapshot.mutable_time_offset_seconds())[entry.first.as<std::string>()] =
          entry.second.as<double>();
    }
  }

  if (root["time_offset_provenance"]) {
    for (const auto& entry : root["time_offset_provenance"]) {
      (*snapshot.mutable_time_offset_provenance())[entry.first.as<std::string>()] =
          entry.second.as<std::string>();
    }
  }

  if (root["vehicle_state_sensors"]) {
    if (!root["vehicle_state_sensors"].IsSequence()) {
      throw std::runtime_error("vehicle_state_sensors must be a sequence: " + path);
    }
    std::set<std::string> state_sensor_ids;
    for (const auto& sensor_node : root["vehicle_state_sensors"]) {
      const std::string sensor_id = sensor_node.as<std::string>();
      if (sensor_id.empty() || !state_sensor_ids.insert(sensor_id).second) {
        throw std::runtime_error("vehicle_state_sensors entries must be non-empty and unique: " + path);
      }
      snapshot.add_vehicle_state_sensors()->set_value(sensor_id);
    }
  }

  if (root["sonar_beam_models"]) {
    for (const auto& sonar_node : root["sonar_beam_models"]) {
      auto* model = snapshot.add_sonar_beam_models();
      model->mutable_sensor_id()->set_value(sonar_node["sensor_id"].as<std::string>());
      model->set_horizontal_fov_rad(GetOr<double>(sonar_node, "horizontal_fov_rad", 0.0));
      model->set_elevation_aperture_rad(GetOr<double>(sonar_node, "elevation_aperture_rad", 0.0));
      model->set_range_resolution_m(GetOr<double>(sonar_node, "range_resolution_m", 0.0));
      model->set_nominal_speed_of_sound_mps(
          GetOr<double>(sonar_node, "nominal_speed_of_sound_mps", 1500.0));
      model->set_sonar_enabled(GetOr<bool>(sonar_node, "sonar_enabled", true));
    }
  }

  if (root["depth_models"]) {
    for (const auto& depth_node : root["depth_models"]) {
      auto* model = snapshot.add_depth_models();
      model->mutable_sensor_id()->set_value(depth_node["sensor_id"].as<std::string>());
      model->set_noise_sigma_m(GetOr<double>(depth_node, "noise_sigma_m", 0.05));
      model->set_depth_enabled(GetOr<bool>(depth_node, "depth_enabled", true));
    }
  }
  std::set<std::string> online_sensor_ids;
  for (const auto& camera : snapshot.cameras()) {
    if (camera.sensor_id().value().empty() ||
        !online_sensor_ids.insert(camera.sensor_id().value()).second) {
      throw std::runtime_error("camera sensor ids must be non-empty and unique: " + path);
    }
  }
  std::size_t enabled_sonar_count = 0;
  for (const auto& sonar : snapshot.sonar_beam_models()) {
    if (!sonar.sonar_enabled()) continue;
    ++enabled_sonar_count;
    if (sonar.sensor_id().value().empty() ||
        !online_sensor_ids.insert(sonar.sensor_id().value()).second) {
      throw std::runtime_error("enabled sonar sensor ids must be non-empty and unique: " + path);
    }
  }
  for (const auto& sensor : snapshot.vehicle_state_sensors()) {
    if (!online_sensor_ids.insert(sensor.value()).second) {
      throw std::runtime_error("online sensor roles must use unique sensor ids: " + path);
    }
  }
  if (enabled_sonar_count > 0 && enabled_sonar_count != 1) {
    throw std::runtime_error(
        "online v1 rigs require exactly one enabled sonar: " + path);
  }
  if (enabled_sonar_count > 0 && snapshot.vehicle_state_sensors_size() != 1) {
    throw std::runtime_error(
        "rigs with an enabled sonar require exactly one vehicle_state_sensor: " + path);
  }
  for (const std::string& sensor_id : online_sensor_ids) {
    const auto offset = snapshot.time_offset_seconds().find(sensor_id);
    const auto provenance = snapshot.time_offset_provenance().find(sensor_id);
    if (offset == snapshot.time_offset_seconds().end() || !std::isfinite(offset->second) ||
        std::abs(offset->second) > kMaxAbsoluteSensorTimeOffsetSeconds ||
        provenance == snapshot.time_offset_provenance().end() || provenance->second.empty()) {
      throw std::runtime_error(
          "online sensor requires a bounded finite time offset and non-empty provenance: " +
          sensor_id + " in " + path);
    }
  }
  for (const auto& entry : snapshot.time_offset_seconds()) {
    if (!std::isfinite(entry.second) ||
        std::abs(entry.second) > kMaxAbsoluteSensorTimeOffsetSeconds) {
      throw std::runtime_error(
          "absolute time offset must be finite and no greater than " +
          std::to_string(kMaxAbsoluteSensorTimeOffsetSeconds) + " seconds: " +
          entry.first + " in " + path);
    }
    const auto provenance = snapshot.time_offset_provenance().find(entry.first);
    if (provenance == snapshot.time_offset_provenance().end() || provenance->second.empty()) {
      throw std::runtime_error("every time offset requires non-empty provenance: " + entry.first +
                               " in " + path);
    }
  }
  for (const auto& entry : snapshot.time_offset_provenance()) {
    if (entry.second.empty() || snapshot.time_offset_seconds().count(entry.first) == 0) {
      throw std::runtime_error("time offset provenance requires a matching offset: " + entry.first +
                               " in " + path);
    }
  }

  return snapshot;
}

ScenarioConfig LoadScenarioConfig(const std::string& path) {
  const YAML::Node root = YAML::LoadFile(path);
  ScenarioConfig config;

  config.seed = GetOr<uint64_t>(root, "seed", config.seed);
  config.num_keyframes = GetOr<int>(root, "num_keyframes", config.num_keyframes);
  config.radius_m = GetOr<double>(root, "radius_m", config.radius_m);
  config.arc_radians = GetOr<double>(root, "arc_radians", config.arc_radians);
  config.depth_m = GetOr<double>(root, "depth_m", config.depth_m);

  if (root["noise"]) {
    const auto noise = root["noise"];
    config.noise.relative_pose_noise_m =
        GetOr<double>(noise, "relative_pose_noise_m", config.noise.relative_pose_noise_m);
    config.noise.sonar_range_noise_m =
        GetOr<double>(noise, "sonar_range_noise_m", config.noise.sonar_range_noise_m);
    config.noise.sonar_bearing_noise_rad =
        GetOr<double>(noise, "sonar_bearing_noise_rad", config.noise.sonar_bearing_noise_rad);
  }

  if (root["sonar_targets_world"]) {
    for (const auto& target_node : root["sonar_targets_world"]) {
      if (target_node.size() != 3) {
        throw std::runtime_error("sonar_targets_world entries must have exactly 3 components: " +
                                  path);
      }
      config.sonar_targets_world.emplace_back(target_node[0].as<double>(), target_node[1].as<double>(),
                                               target_node[2].as<double>());
    }
  }

  return config;
}

ExperimentConfig LoadExperimentConfig(const std::string& path) {
  const YAML::Node root = YAML::LoadFile(path);
  // Experiment files live at configs/experiment/*.yaml and reference their
  // defaults/rig/scenario layers as "defaults/x.yaml", "rig/y.yaml",
  // "scenario/z.yaml" — i.e. relative to configs/ (the common parent of
  // all four layer directories), not relative to configs/experiment/
  // itself. Go up two levels from the experiment file to get there.
  const std::string base_dir =
      std::filesystem::path(path).parent_path().parent_path().string();

  ExperimentConfig config;

  if (root["defaults"]) {
    config.defaults = LoadPlatformDefaultsConfig(ResolveRelative(base_dir, root["defaults"].as<std::string>()));
  }
  // Experiment-level override for the solver backend, same pattern as
  // frontends.landmark_detector below — lets a benchmark compare backends
  // via one small experiment file instead of forking a whole separate
  // defaults/*.yaml just to flip this one field.
  if (root["estimation"] && root["estimation"]["solver"]) {
    config.defaults.solver = root["estimation"]["solver"].as<std::string>();
  }
  if (root["rig"]) {
    config.rig = LoadRigConfig(ResolveRelative(base_dir, root["rig"].as<std::string>()));
  }
  if (root["scenario"]) {
    config.scenario = LoadScenarioConfig(ResolveRelative(base_dir, root["scenario"].as<std::string>()));
  }

  if (root["frontends"] && root["frontends"]["optical"]) {
    config.optical_frontend = root["frontends"]["optical"].as<std::string>();
  }
  if (root["frontends"] && root["frontends"]["sonar"]) {
    config.sonar_frontend = root["frontends"]["sonar"].as<std::string>();
  }
  if (root["frontends"] && root["frontends"]["landmark_detector"]) {
    config.landmark_detector = root["frontends"]["landmark_detector"].as<std::string>();
  }
  config.estimator_mode = GetOr<std::string>(root, "estimator_mode", config.estimator_mode);
  config.map_backend = GetOr<std::string>(root, "map_backend", config.map_backend);
  if (root["output"]) {
    config.write_run_manifest =
        GetOr<bool>(root["output"], "write_run_manifest", config.write_run_manifest);
  }

  // Experiment-level gate overrides: what counts as an acceptable run
  // differs sharply between a known-good synthetic scenario and a
  // real-data experiment still being brought up (docs/uw-slam-production-
  // readiness-and-roadmap-2026-08-21.md section 5.5). Rather than fork
  // defaults/platform.yaml per experiment, the experiment file — the most
  // specific layer — can override individual defaults.* gate fields here.
  if (root["gates"]) {
    const auto gates = root["gates"];
    config.defaults.require_converged =
        GetOr<bool>(gates, "require_converged", config.defaults.require_converged);
    config.defaults.max_ate_rmse_m = GetOr<double>(gates, "max_ate_rmse_m", config.defaults.max_ate_rmse_m);
    config.defaults.min_matched_ate_poses =
        GetOr<int>(gates, "min_matched_ate_poses", config.defaults.min_matched_ate_poses);
    config.defaults.require_nonempty_map =
        GetOr<bool>(gates, "require_nonempty_map", config.defaults.require_nonempty_map);
    config.defaults.min_acoustic_optic_accepted =
        GetOr<int>(gates, "min_acoustic_optic_accepted", config.defaults.min_acoustic_optic_accepted);
    config.defaults.min_acoustic_optic_map_points = GetOr<int>(
        gates, "min_acoustic_optic_map_points", config.defaults.min_acoustic_optic_map_points);
  }

  return config;
}

std::optional<std::string> ValidateExperimentConfigSelections(const ExperimentConfig& config) {
  // Known-good identifiers, one per field, matching the single
  // implementation each currently has: the algorithm_version literals
  // written by src/frontends/sonar_cfar_frontend.cpp and
  // src/frontends/stereo_optical_depth_frontend.cpp, and SubmapManager
  // (the only MapEvidence consumer — always MAP_REPRESENTATION_POINT_CLOUD).
  // estimator_mode and landmark_detector are genuinely dispatched on in
  // apps/replay_demo.cpp; the other three fields are not dispatched (there
  // is nothing else to switch to yet), but an unrecognized value must still
  // fail loudly instead of the app silently running its one pipeline anyway.
  if (config.sonar_frontend != "sonar_cfar_frontend_v1") {
    return "unrecognized sonar_frontend '" + config.sonar_frontend +
           "' (only sonar_cfar_frontend_v1 is implemented)";
  }
  if (config.optical_frontend != "stereo_depth_frontend_v1") {
    return "unrecognized optical_frontend '" + config.optical_frontend +
           "' (only stereo_depth_frontend_v1 is implemented)";
  }
  if (config.map_backend != "submap_point_cloud_v1") {
    return "unrecognized map_backend '" + config.map_backend +
           "' (only submap_point_cloud_v1 is implemented)";
  }
  if (config.estimator_mode != "black_box_vio" && config.estimator_mode != "stereo_landmark_vo") {
    return "unrecognized estimator_mode '" + config.estimator_mode +
           "' (must be black_box_vio or stereo_landmark_vo)";
  }
  if (config.landmark_detector != "bright_blob" && config.landmark_detector != "harris_corner") {
    return "unrecognized landmark_detector '" + config.landmark_detector +
           "' (must be bright_blob or harris_corner)";
  }
  // defaults.solver: genuinely dispatched in apps/replay_demo.cpp
  // (docs/superpowers/specs/2026-08-23-solver-and-mapping-oss-adoption.md
  // §7) — "ceres_v1" is a recognized value regardless of whether this
  // binary was built with UW_BUILD_CERES_SOLVER; that build-time capability
  // check happens separately, at the point of actually constructing the
  // solver, so the error message can say "not compiled in" rather than
  // "unrecognized" for a name that IS a real, just-not-linked backend.
  if (config.defaults.solver != "gauss_newton_v1" && config.defaults.solver != "ceres_v1") {
    return "unrecognized solver '" + config.defaults.solver + "' (must be gauss_newton_v1 or ceres_v1)";
  }
  return std::nullopt;
}

}  // namespace uw::runtime
