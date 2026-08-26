// Layered configuration loading (platform architecture section 14.2):
// defaults -> rig -> scenario -> experiment. Each layer is a plain YAML
// file under configs/; this loads them into typed structs (or, for rig,
// directly into the domain RigCalibrationSnapshot proto — see
// schemas/proto/uw/domain/calibration.proto — rather than a parallel
// hand-written struct, so there is exactly one place that defines what a
// rig calibration looks like).
#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "domain/domain.hpp"

namespace uw::runtime {

// Clock calibration is expected to correct sensor-scale skew, not bridge
// unrelated epochs. Bounding it also guarantees safe nanosecond
// quantization in corrected-time buffering.
inline constexpr double kMaxAbsoluteSensorTimeOffsetSeconds = 10.0;

// Bounded corrected-time queues used by AcousticOpticBuffer. Time windows
// are inclusive; capacities are hard limits and must be positive.
struct AcousticOpticBufferConfig {
  double max_stereo_delta_s = 0.002;
  double max_sonar_camera_delta_s = 0.050;
  double max_state_bracket_s = 0.100;
  double max_residence_s = 0.500;
  std::size_t max_images_per_camera = 32;
  std::size_t max_sonar_frames = 16;
  std::size_t max_vehicle_states = 128;
};

// Translation and rotation get INDEPENDENT sqrt-information caps (rather
// than one scalar for both) since RelativePoseFactorBuilder now whitens
// from the VO frontend's actual 6x6 covariance (Task 9) — a single shared
// cap would either over-constrain rotation to translation's noise scale
// or vice versa. See relative_pose_factor_builder.cpp for how these two
// caps combine with the covariance.
struct RelativePoseSqrtInformationCaps {
  double translation = 20.0;
  double rotation = 20.0;
};

struct SqrtInformationDefaults {
  RelativePoseSqrtInformationCaps relative_pose;
  double sonar_range = 15.0;
  double depth = 20.0;
};

// Config for opencv_adapters::StereoRectificationParams -- see that
// struct's own doc comment for what alpha/crop_policy/frame_suffix mean.
// crop_policy is validated to be exactly "full_canvas" or
// "common_valid_roi" (mapped 1:1 to RectificationCropPolicy); no
// near-spelling tolerance.
struct StereoRectificationConfig {
  double alpha = 0.0;
  std::string crop_policy = "full_canvas";
  std::string frame_suffix = "_rectified";
};

// Config for frontends::StereoLandmarkVoFrontendParams::max_consecutive_failures.
// Config for frontends::BlockMatcherParams's texture/uniqueness/LR-
// consistency filters -- see that struct's own field comments.
struct StereoMatchingConfig {
  double min_texture_variance = 25.0;
  double min_uniqueness_margin = 2.0;
  double left_right_max_diff_px = 1.0;
};

struct SonarFrontendConfig {
  int training_cells = 16;
  int guard_cells = 4;
  double probability_false_alarm = 0.01;
  int detector_threshold = 50;
  double dbscan_eps_m = 0.20;
  int dbscan_min_samples = 2;
  double default_range_sigma_m = 0.05;
  double default_bearing_sigma_rad = 0.01;
};

// Multi-sensor target association gates. These values are parsed from the
// platform defaults and intentionally mirror TargetAssociatorParams by name,
// allowing a caller that owns both layers to pass this object directly to the
// associator's constrained config constructor without a second source of
// defaults.
struct TargetAssociationConfig {
  double max_corrected_time_delta_s = 0.05;
  double max_bearing_mahalanobis_sq = 9.0;
  double max_range_mahalanobis_sq = 9.0;
  double max_motion_bearing_delta_rad = 0.25;
  double max_motion_rate_rad_s = 1.5;
  double max_bearing_variance_rad2 = 0.25;
  double max_range_variance_m2 = 4.0;
};

struct TargetTrackerConfig {
  double association_mahalanobis_sq = 16.0;
  int confirm_hits = 2;
  int degraded_misses = 3;
  double stale_after_s = 0.5;
  double max_prediction_dt_s = 0.5;
  double bearing_acceleration_noise = 0.05;
  double range_acceleration_noise = 0.5;
  double merge_bearing_threshold_rad = 0.03;
  double merge_range_threshold_m = 0.30;
};

// Gates OnlineAssistPipeline's local dense stereo depth completion (block
// matching, expensive). Disabled by default per docs/superpowers/plans/
// 2026-08-24-acoustic-optic-online-tracking.md Task 6: bearing/range from
// visual + sonar target detections already drive tracking without it, and
// dense's real distance/path-offset benefit needs paired scenario/pool
// evidence before it's worth the realtime budget risk.
struct OnlineAssistDenseConfig {
  bool enabled = false;
  double budget_ms = 100.0;
};

// Non-dense-specific timing gates for OnlineAssistPipeline's degradation
// reporting: how long a modality (visual detections, sonar detections) or
// the vehicle state feed may go without a fresh capture before the
// corresponding degradation reason (visual_unavailable/sonar_unavailable/
// vehicle_state_stale) takes effect.
struct OnlineAssistPipelineConfig {
  OnlineAssistDenseConfig dense;
  double vehicle_state_stale_after_s = 0.5;
  double modality_stale_after_s = 1.0;
};

struct VisualOdometryConfig {
  int max_consecutive_failures = 3;
  // See frontends::CovarianceEstimationParams (rigid_transform_fit.hpp)
  // for what these gate.
  double max_condition_number = 1.0e8;
  double residual_variance_floor_m2 = 1.0e-8;
  // <=0 (or omitted, defaulting to +inf) disables this gate. See
  // CovarianceEstimationParams::max_inlier_rmse_m's own doc comment for
  // why this catches a failure mode conditioning alone does not: a
  // spurious small "consensus" of coincidentally-consistent false
  // correspondences.
  double max_inlier_rmse_m = std::numeric_limits<double>::infinity();
};

// Loop-closure pose-graph edges (architecture-inspired by SVIn's pose_graph
// module, an ORIGINAL implementation -- see frontends::LoopClosureFrontend's
// own header comment for the v1 scope limits this config controls).
// Default-off (`enabled = false`): zero behavior change to any existing
// experiment unless a config explicitly opts in.
struct LoopClosureConfig {
  bool enabled = false;
  // Pose-proximity candidate retrieval -- see LoopClosureFrontend's header
  // comment for why this (not DBoW2/appearance-only retrieval) is v1's
  // deliberate scope boundary.
  double candidate_search_radius_m = 3.0;
  int min_keyframe_index_gap = 15;
  // Sanity gate on the RECOVERED relative pose (translation/rotation
  // magnitude), applied by the frontend AFTER RANSAC's own inlier-
  // count/rmse gates -- see LoopClosureFrontendParams' own field comments.
  double max_accepted_translation_m = 5.0;
  double max_accepted_rotation_rad = 0.6;
  int min_landmarks_for_pose = 3;
  int max_loop_edges_per_keyframe = 1;
  // Threaded into GaussNewtonOptions::huber_delta (gauss_newton_solver.hpp)
  // -- a threshold on a loop edge's WHITENED residual norm, only meaningful
  // when defaults.solver == "gauss_newton_v1" (the default); the Ceres
  // adapter does not yet read PoseGraphProblem::ResidualBinding::
  // robust_policy (see ceres_pose_graph_solver.cpp's own TODO).
  double huber_delta = 1.5;
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
  StereoRectificationConfig stereo_rectification;
  VisualOdometryConfig visual_odometry;
  LoopClosureConfig loop_closure;
  StereoMatchingConfig stereo_matching;
  SonarFrontendConfig sonar_frontend;
  TargetAssociationConfig target_association;
  TargetTrackerConfig target_tracker;
  OnlineAssistPipelineConfig online_assist;

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
  // Distinct from require_nonempty_map: that gate is satisfied by
  // optical-only depth alone. These two gate on ACOUSTIC-OPTIC
  // contribution specifically (contribution_mask ==
  // DEPTH_CONTRIBUTION_ACOUSTIC_OPTIC), counted BEFORE
  // BuildMapEvidenceFromFusedDepth (which does not preserve per-point
  // origin) — see application::CountDepthContributions. <=0 = disabled;
  // only meant for experiments that expect a visible acoustic-optic
  // target (e.g. configs/experiment/acoustic_optic_demo.yaml), not for
  // scenarios whose targets are outside the camera's narrow FOV.
  int min_acoustic_optic_accepted = 0;
  int min_acoustic_optic_map_points = 0;
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
