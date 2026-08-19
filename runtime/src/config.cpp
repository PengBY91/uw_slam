#include "uw/runtime/config.hpp"

#include <filesystem>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace uw::runtime {

namespace {

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

}  // namespace

PlatformDefaultsConfig LoadPlatformDefaultsConfig(const std::string& path) {
  const YAML::Node root = YAML::LoadFile(path);
  PlatformDefaultsConfig config;

  const auto estimation = root["estimation"];
  config.solver = GetOr<std::string>(estimation, "solver", config.solver);
  config.max_iterations = GetOr<int>(estimation, "max_iterations", config.max_iterations);
  config.initial_lambda = GetOr<double>(estimation, "initial_lambda", config.initial_lambda);
  config.warmup_seconds = GetOr<double>(estimation, "warmup_seconds", config.warmup_seconds);

  if (root["reliability"] && root["reliability"]["default_sqrt_information"]) {
    const auto info = root["reliability"]["default_sqrt_information"];
    config.default_sqrt_information.relative_pose =
        GetOr<double>(info, "relative_pose", config.default_sqrt_information.relative_pose);
    config.default_sqrt_information.sonar_range =
        GetOr<double>(info, "sonar_range", config.default_sqrt_information.sonar_range);
    config.default_sqrt_information.depth =
        GetOr<double>(info, "depth", config.default_sqrt_information.depth);
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
  if (root["rig"]) {
    config.rig = LoadRigConfig(ResolveRelative(base_dir, root["rig"].as<std::string>()));
  }
  if (root["scenario"]) {
    config.scenario = LoadScenarioConfig(ResolveRelative(base_dir, root["scenario"].as<std::string>()));
  }

  if (root["frontends"] && root["frontends"]["sonar"]) {
    config.sonar_frontend = root["frontends"]["sonar"].as<std::string>();
  }
  config.estimator_mode = GetOr<std::string>(root, "estimator_mode", config.estimator_mode);
  config.map_backend = GetOr<std::string>(root, "map_backend", config.map_backend);
  if (root["output"]) {
    config.write_run_manifest =
        GetOr<bool>(root["output"], "write_run_manifest", config.write_run_manifest);
  }

  return config;
}

}  // namespace uw::runtime
