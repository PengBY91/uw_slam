// Layered configuration loading (platform architecture section 14.2):
// defaults -> rig -> scenario -> experiment. Each layer is a plain YAML
// file under configs/; this loads them into typed structs (or, for rig,
// directly into the domain RigCalibrationSnapshot proto — see
// schemas/proto/uw/domain/calibration.proto — rather than a parallel
// hand-written struct, so there is exactly one place that defines what a
// rig calibration looks like).
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "uw/domain/domain.hpp"

namespace uw::runtime {

struct SqrtInformationDefaults {
  double relative_pose = 20.0;
  double sonar_range = 15.0;
  double depth = 20.0;
};

struct PlatformDefaultsConfig {
  std::string solver = "gauss_newton_v1";
  int max_iterations = 30;
  double initial_lambda = 1e-3;
  SqrtInformationDefaults default_sqrt_information;

  // Discard evidence from the first N seconds of a run before fusing it
  // (0 = disabled). Motivated by a specific deployment lesson from a
  // sibling ROS2 SVIn+HoloOcean bring-up (workfiles_02's merge_node), which
  // drops early frames while the VIO's IMU bias hasn't converged yet. We
  // don't run an online IMU filter here, so the analogue is: keyframes
  // before this cutoff are excluded from the pose graph entirely, and the
  // first surviving keyframe becomes the fixed anchor (see apps/replay_demo
  // for exactly how this is applied).
  double warmup_seconds = 0.0;
};

struct ScenarioNoiseConfig {
  double relative_pose_noise_m = 0.02;
  double sonar_range_noise_m = 0.03;
  double sonar_bearing_noise_rad = 0.01;
};

struct ScenarioConfig {
  uint64_t seed = 42;
  int num_keyframes = 12;
  double radius_m = 8.0;
  double arc_radians = 1.4;
  double depth_m = 12.0;
  ScenarioNoiseConfig noise;
  std::vector<Eigen::Vector3d> sonar_targets_world;
};

// The fully-resolved layer stack for one run: defaults + rig + scenario,
// as named by an experiment YAML (which also selects algorithm variants —
// those selections are read but only partially consumed in v1; see
// apps/replay_demo and apps/tools/synth_bag_gen for exactly what's wired
// up vs. still a documented no-op).
struct ExperimentConfig {
  PlatformDefaultsConfig defaults;
  uw::domain::RigCalibrationSnapshot rig;
  ScenarioConfig scenario;

  std::string sonar_frontend = "sonar_cfar_frontend_v1";
  std::string optical_frontend = "stereo_depth_frontend_v1";
  std::string estimator_mode = "black_box_vio";
  std::string map_backend = "submap_point_cloud_v1";
  bool write_run_manifest = true;
};

PlatformDefaultsConfig LoadPlatformDefaultsConfig(const std::string& path);
uw::domain::RigCalibrationSnapshot LoadRigConfig(const std::string& path);
ScenarioConfig LoadScenarioConfig(const std::string& path);

// Loads an experiment YAML and, following its `defaults:`/`rig:`/
// `scenario:` keys (paths resolved relative to the experiment file's
// grandparent directory — i.e. configs/, the common parent of
// configs/{defaults,rig,scenario,experiment}/ — matching how
// configs/experiment/*.yaml already write those keys as "defaults/x.yaml"
// etc.), the three layers underneath it.
ExperimentConfig LoadExperimentConfig(const std::string& path);

}  // namespace uw::runtime
