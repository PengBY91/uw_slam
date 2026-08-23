// Layered configuration loading (platform architecture section 14.2):
// defaults -> rig -> scenario -> experiment. Each layer is a plain YAML
// file under configs/; this loads them into typed structs (or, for rig,
// directly into the domain RigCalibrationSnapshot proto — see
// schemas/proto/uw/domain/calibration.proto — rather than a parallel
// hand-written struct, so there is exactly one place that defines what a
// rig calibration looks like).
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "domain/domain.hpp"

namespace uw::runtime {

struct SqrtInformationDefaults {
  double relative_pose = 20.0;
  double sonar_range = 15.0;
  double depth = 20.0;
};

struct PlatformDefaultsConfig {
  // "gauss_newton_v1" (default, uw::estimation::GaussNewtonSolver) or
  // "ceres_v1" (uw::adapters::ceres_solver::CeresPoseGraphSolver — only
  // usable if this binary was built with UW_BUILD_CERES_SOLVER=ON; selected
  // but not compiled in is a fatal startup error, not a silent fallback).
  // See docs/superpowers/specs/2026-08-23-solver-and-mapping-oss-adoption.md.
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

  // P0 non-void replay gates (docs/uw-slam-production-readiness-and-roadmap-
  // 2026-08-21.md section 5.5/7): apps/replay_demo checks these after
  // solving and exits non-zero when violated, instead of always returning 0
  // regardless of output quality. All threshold gates default to disabled
  // (negative/zero) so existing experiments that haven't had a threshold
  // deliberately chosen for them keep running unblocked; require_converged
  // defaults to true because a stalled solver is never an acceptable output
  // regardless of experiment.
  bool require_converged = true;
  double max_ate_rmse_m = -1.0;       // <0 = gate disabled
  int min_matched_ate_poses = 0;      // <=0 = gate disabled
  bool require_nonempty_map = false;  // landmarks discovered + map evidence points > 0
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
// see apps/replay_demo and apps/synth_bag_gen.cpp for exactly what's
// dispatched on vs. still a documented single-implementation field, and
// ValidateExperimentConfigSelections() below for what happens when a field
// names something this binary doesn't have).
struct ExperimentConfig {
  PlatformDefaultsConfig defaults;
  uw::domain::RigCalibrationSnapshot rig;
  ScenarioConfig scenario;

  std::string sonar_frontend = "sonar_cfar_frontend_v1";
  std::string optical_frontend = "stereo_depth_frontend_v1";
  // Only consumed when estimator_mode == "stereo_landmark_vo" — selects
  // uw::frontends::LandmarkDetectorKind ("bright_blob" or "harris_corner",
  // see that enum's comment). "bright_blob" is tuned for
  // apps/synth_bag_gen.cpp's synthetic scene; "harris_corner" is for
  // real camera imagery.
  std::string landmark_detector = "bright_blob";
  // Historical name: this selects the relative-pose evidence source, not the
  // optimizer. "stereo_landmark_vo" selects camera-computed evidence only
  // when the loaded rig contains cameras; otherwise replay falls back to
  // bag /evidence/relative_pose. Both paths feed the same GaussNewtonSolver.
  std::string estimator_mode = "black_box_vio";
  // Reserved map-implementation selector; v1 accepts only
  // "submap_point_cloud_v1".
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

// Checks that every algorithm-selection field in `config`
// (sonar_frontend/optical_frontend/map_backend/estimator_mode/
// landmark_detector/defaults.solver) names an implementation this binary
// actually has,
// instead of being silently ignored — see docs/uw-slam-production-
// readiness-and-roadmap-2026-08-21.md section 10's "配置存在但不驱动实现"
// risk. Returns the reason the first unrecognized field is invalid, or
// std::nullopt if every field is recognized. apps/replay_demo (the only
// consumer of these fields — apps/synth_bag_gen only reads
// config.scenario/config.rig, not the algorithm-selection fields) should
// treat a non-nullopt result as a fatal configuration error, not a warning.
std::optional<std::string> ValidateExperimentConfigSelections(const ExperimentConfig& config);

}  // namespace uw::runtime
