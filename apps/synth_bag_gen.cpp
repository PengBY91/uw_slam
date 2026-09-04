// Generates a synthetic MCAP bag with a known ground-truth trajectory and
// fabricated measurements (relative pose "VIO" evidence, sonar range-
// bearing to a few fixed targets, and depth). This exists because the
// current development machine has neither HoloOcean nor ROS2 installed
// (see the platform architecture's environment notes) — it lets
// apps/replay_demo exercise the full FactorBuilder -> PoseGraphProblem ->
// GaussNewtonSolver -> StateStore -> SubmapManager chain end-to-end without
// either of those.
//
// Topics written (all protobuf-encoded, see uw::runtime::McapProtobufWriter):
//   /gt/state                    uw.domain.StateSnapshot   (ground truth per keyframe)
//   /evidence/relative_pose      uw.domain.MeasurementEvidence (RelativePoseMeasurement)
//   /raw/sonar_frame             uw.domain.SonarFrame      (synthetic imaging-sonar ping; see
//                                 RenderSyntheticSonarFrame below — replay_pipeline runs this
//                                 through the real sonar_cfar_frontend (include/frontends,
//                                 src/frontends), it is
//                                 NOT pre-computed range-bearing evidence)
//   /evidence/depth              uw.domain.MeasurementEvidence (PressureDepthMeasurement)
//   /scenario/sonar_targets      uw.domain.MapEvidence     (known target positions, world frame,
//                                 packed float32 xyz — reused as a point-cloud payload; see
//                                 apps/replay_demo for how this stands in for a real
//                                 landmark/submap query that v1 does not yet have)
//   /raw/camera/left             uw.domain.ImageFrame      (only when --experiment loads a rig
//   /raw/camera/right            uw.domain.ImageFrame       with cameras; see BuildStereoPair —
//                                 synthetic stereo pair per keyframe, real per-keyframe geometry,
//                                 for apps/replay_demo's acoustic-optic pass)
//   /raw/imu                     uw.domain.ImuSample       (only when --experiment selects
//   /keyframe/boundary           uw.domain.KeyframeBoundary estimator_mode: imu_preintegration;
//                                 see "IMU fixture" below)
//
// Flags that exist only to test leakage, not to describe any real sensor:
// --omit-relative-pose drops /evidence/relative_pose; --omit-ground-truth,
// --ground-truth-time-offset-s and --ground-truth-pose-offset-m drop or
// corrupt /gt/state. All four leave every other topic bit-identical, so a
// pipeline whose output moves when one is used is reading something it is
// not allowed to read.
//
// IMU fixture (PREP-B-01, docs/imu-preintegration-design-2026-09-03.md §7-8).
// Only `estimator_mode: imu_preintegration` turns this on, so every other
// experiment's bag is byte-for-byte what it was before. Three things change
// together, and they only make sense together:
//
//   1. A STATIONARY PRE-ROLL is prepended (kImuPreRollS, 0.75 s): /raw/imu
//      starts at t = 0 and the vehicle does not move until then. This is what the
//      stationary initializer reads to estimate the initial gyro/accel
//      biases and the gravity direction — without it there is no legitimate
//      (non-ground-truth) source for those.
//   2. Every keyframe-anchored topic shifts by that same 0.5 s, and each
//      keyframe gets an explicit /keyframe/boundary event. The boundary
//      stream — not /gt/state, not relative-pose evidence — is the only
//      contract saying where a preintegration interval starts and ends, and
//      it deliberately carries no pose.
//   3. The arc is traversed with a QUINTIC SMOOTHSTEP angular profile
//      instead of a constant rate (see ArcFraction). The geometry is the
//      same circle; what changes is that dθ/dt and d²θ/dt² are zero at the
//      start, so the vehicle is genuinely at rest at the first keyframe. A
//      constant-rate arc would have the body jump from 0 to R·ω ≈ 5 m/s at
//      the instant the pre-roll ends, which no IMU stream can represent —
//      the stationary initializer's v₀ = 0 would then be wrong by that much
//      and no amount of estimator work could recover the trajectory. Other
//      estimator modes keep the constant-rate profile they always had.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <limits>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "domain/domain.hpp"
#include "runtime/canonical_topics.hpp"
#include "runtime/config.hpp"
#include "runtime/mcap_io.hpp"
#include "runtime/synthetic_sonar.hpp"
#include "sensor_models/camera_model.hpp"
#include "sensor_models/geometry.hpp"
#include "sensor_models/sonar_beam_model.hpp"

using uw::sensor_models::Pose3;

namespace {

struct ScenarioOptions {
  std::string out_path = "/tmp/synthetic.mcap";
  int num_keyframes = 12;
  double radius_m = 8.0;
  double arc_radians = 1.4;
  double depth_m = 12.0;
  double relative_pose_noise_m = 0.02;
  double sonar_range_noise_m = 0.03;
  double sonar_bearing_noise_rad = 0.01;
  uint64_t seed = 42;
  // Empty => BuildSonarTargets() falls back to its built-in defaults (kept
  // for standalone use without --experiment, and so the existing
  // determinism/demo commands in README.md keep working unchanged).
  std::vector<Eigen::Vector3d> sonar_targets_world;
};

// Layers configs/scenario/*.yaml (via an --experiment configs/experiment/*.yaml
// file, section 14.2's defaults->rig->scenario->experiment stack) onto
// ScenarioOptions's built-in defaults. Explicit CLI flags parsed AFTER this
// call (see main()) still win — this is the "scenario" layer, CLI flags are
// an even-more-specific ad hoc override on top.
void ApplyScenarioConfig(const uw::runtime::ScenarioConfig& scenario, ScenarioOptions& opt) {
  opt.num_keyframes = scenario.num_keyframes;
  opt.radius_m = scenario.radius_m;
  opt.arc_radians = scenario.arc_radians;
  opt.depth_m = scenario.depth_m;
  opt.relative_pose_noise_m = scenario.noise.relative_pose_noise_m;
  opt.sonar_range_noise_m = scenario.noise.sonar_range_noise_m;
  opt.sonar_bearing_noise_rad = scenario.noise.sonar_bearing_noise_rad;
  opt.seed = scenario.seed;
  if (!scenario.sonar_targets_world.empty()) {
    opt.sonar_targets_world = scenario.sonar_targets_world;
  }
}

// How far along the arc the vehicle is at normalized motion time s in
// [0, 1], as a fraction of arc_radians.
//
// `smooth_start` = false is the original constant-rate traversal every
// non-IMU experiment uses and must keep using. `smooth_start` = true is the
// quintic smoothstep 10s³ − 15s⁴ + 6s⁵, whose first AND second derivatives
// vanish at both ends: the body's angular rate and its specific force are
// therefore continuous across the pre-roll/motion seam, so "stationary
// until t = 0.5 s" is true of the trajectory and not just of the samples
// before it. See this file's header comment for why a constant-rate arc
// cannot be used behind a stationary pre-roll.
double ArcFraction(double s, bool smooth_start) {
  if (!smooth_start) return s;
  return s * s * s * (10.0 - 15.0 * s + 6.0 * s * s);
}

// d/ds and d²/ds² of ArcFraction, for deriving the IMU truth analytically
// rather than differencing poses.
double ArcFractionRate(double s, bool smooth_start) {
  if (!smooth_start) return 1.0;
  return s * s * (30.0 - 60.0 * s + 30.0 * s * s);
}

double ArcFractionAcceleration(double s, bool smooth_start) {
  if (!smooth_start) return 0.0;
  return s * (60.0 - 180.0 * s + 120.0 * s * s);
}

Pose3 ArcPose(const ScenarioOptions& opt, double theta) {
  Pose3 pose;
  pose.translation =
      Eigen::Vector3d(opt.radius_m * std::sin(theta), opt.radius_m * (1.0 - std::cos(theta)),
                      -opt.depth_m);
  pose.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()));
  return pose;
}

std::vector<Pose3> BuildGroundTruthTrajectory(const ScenarioOptions& opt, bool smooth_start) {
  std::vector<Pose3> trajectory;
  trajectory.reserve(opt.num_keyframes);
  for (int i = 0; i < opt.num_keyframes; ++i) {
    const double s = opt.num_keyframes > 1
                          ? static_cast<double>(i) / (opt.num_keyframes - 1)
                          : 0.0;
    trajectory.push_back(ArcPose(opt, opt.arc_radians * ArcFraction(s, smooth_start)));
  }
  return trajectory;
}

std::vector<Eigen::Vector3d> BuildSonarTargets(const ScenarioOptions& opt) {
  if (!opt.sonar_targets_world.empty()) return opt.sonar_targets_world;
  // Fallback for standalone use without --experiment: a handful of fixed
  // seabed-like features near the path, close enough that most keyframes
  // see at least one within a plausible sonar range.
  return {
      Eigen::Vector3d(2.0, 3.0, -opt.depth_m - 1.0),
      Eigen::Vector3d(6.0, 6.0, -opt.depth_m + 0.5),
      Eigen::Vector3d(-1.0, 8.0, -opt.depth_m - 0.5),
  };
}

std::string KeyframeId(int i) { return "kf" + std::to_string(i); }

// 5 Hz keyframes, unchanged from the original fixture.
constexpr double kKeyframePeriodS = 0.2;
// docs/imu-preintegration-design-2026-09-03.md section 7 requires AT LEAST
// 0.5 s of stationary IMU data before the first keyframe, so the stationary
// initializer has something to estimate the initial biases and the gravity
// direction from. Deliberately generated with margin rather than exactly at
// the minimum: at exactly 0.5 s the initializer's "window_duration_s >=
// min_stationary_duration_s" test passes only by exact floating-point
// equality, so any rate that does not divide 0.5 s evenly (150 Hz puts its
// last pre-roll sample at 0.4933 s) would silently drop the run to the wide
// velocity prior, with a worse ATE as the only symptom.
constexpr double kImuPreRollS = 0.75;

uint64_t SecondsToNanos(double seconds) {
  return static_cast<uint64_t>(std::llround(seconds * 1e9));
}

// The header convention every synthetic producer in this repo follows (see
// runtime/synthetic_sonar.cpp): receive_time equals capture_time because
// synthetic generation models no transport delay, and leaving it at the
// all-zero default would read as "never populated" in apps/bag_audit.
uw::domain::ObservationHeader MakeSyntheticHeader(const std::string& observation_id,
                                                   const std::string& sensor_id,
                                                   const std::string& sensor_frame,
                                                   uint64_t t_ns) {
  uw::domain::ObservationHeader header;
  header.mutable_observation_id()->set_value(observation_id);
  header.mutable_sensor_id()->set_value(sensor_id);
  header.mutable_sensor_frame()->set_value(sensor_frame);
  header.mutable_capture_time()->set_seconds(static_cast<int64_t>(t_ns / 1'000'000'000ULL));
  header.mutable_capture_time()->set_nanos(static_cast<int32_t>(t_ns % 1'000'000'000ULL));
  *header.mutable_receive_time() = header.capture_time();
  header.set_clock_domain(uw::domain::CLOCK_DOMAIN_SIMULATION);
  header.set_validity(uw::domain::ObservationHeader::VALIDITY_OK);
  header.set_provenance("synth_bag_gen_v1");
  return header;
}

// Analytic IMU truth in the BODY frame at motion-normalized time `s`.
//
// The body follows p(theta) = (R sin, R(1-cos), -depth) with yaw theta, so
// rotating the world acceleration back into the body frame collapses to
// exactly (R*theta_ddot, R*theta_dot^2, 0): tangential along +x, centripetal
// along +y, nothing along z. The accelerometer measures specific force,
// a_body - R_wb^T * g_w with g_w = (0, 0, -gravity), and the body has no
// roll/pitch here, so that adds a constant +gravity on body z. Deriving it
// this way (rather than differencing poses) is what makes the pre-roll seam
// exact: at s = 0 the smoothstep's first and second derivatives are zero,
// so this reduces to the stationary reading (0, 0, gravity) with no jump.
struct ImuTruth {
  Eigen::Vector3d specific_force_mps2;
  Eigen::Vector3d angular_velocity_radps;
  Eigen::Vector3d angular_acceleration_radps2;
};

ImuTruth StationaryImuTruth(double gravity_mps2) {
  return ImuTruth{Eigen::Vector3d(0.0, 0.0, gravity_mps2), Eigen::Vector3d::Zero(),
                  Eigen::Vector3d::Zero()};
}

ImuTruth BodyImuTruth(const ScenarioOptions& opt, double motion_duration_s, double s,
                      double gravity_mps2) {
  if (motion_duration_s <= 0.0) return StationaryImuTruth(gravity_mps2);
  const double theta_rate =
      opt.arc_radians * ArcFractionRate(s, /*smooth_start=*/true) / motion_duration_s;
  const double theta_acceleration = opt.arc_radians *
                                    ArcFractionAcceleration(s, /*smooth_start=*/true) /
                                    (motion_duration_s * motion_duration_s);
  ImuTruth truth;
  truth.specific_force_mps2 = Eigen::Vector3d(opt.radius_m * theta_acceleration,
                                              opt.radius_m * theta_rate * theta_rate,
                                              gravity_mps2);
  truth.angular_velocity_radps = Eigen::Vector3d(0.0, 0.0, theta_rate);
  truth.angular_acceleration_radps2 = Eigen::Vector3d(0.0, 0.0, theta_acceleration);
  return truth;
}

// base_link readings -> imu_link readings: the exact inverse of what
// frontends/imu_preintegration_frontend does when it maps a raw sample back
// into the body frame. An IMU mounted at r from the body origin also feels
// the lever-arm terms omega x (omega x r) and alpha x r. The frontend
// deliberately drops alpha x r as a documented approximation; with the
// identity imu_link edge in configs/rig/example_auv_sonar_only.yaml both
// terms are exactly zero, so the two sides agree bit for bit on this
// fixture and the term is here only so a rig with a real offset is not
// silently generated wrong.
ImuTruth ToImuFrame(const ImuTruth& body, const Pose3& base_link_T_imu_link) {
  const Eigen::Matrix3d rotation = base_link_T_imu_link.rotation.toRotationMatrix();
  const Eigen::Vector3d& r = base_link_T_imu_link.translation;
  const Eigen::Vector3d& omega = body.angular_velocity_radps;
  const Eigen::Vector3d lever_arm =
      omega.cross(omega.cross(r)) + body.angular_acceleration_radps2.cross(r);
  ImuTruth sensor;
  sensor.specific_force_mps2 = rotation.transpose() * (body.specific_force_mps2 + lever_arm);
  sensor.angular_velocity_radps = rotation.transpose() * omega;
  sensor.angular_acceleration_radps2 = rotation.transpose() * body.angular_acceleration_radps2;
  return sensor;
}

// Note what is NOT written: has_bias / bias_* stay unset. Those fields mean
// "the sensor reported its own internal bias estimate"; filling them with
// the simulator's exact bias truth would put a ground-truth channel on an
// algorithm-input topic, which is precisely what PREP-B-01 is closing off.
uw::domain::ImuSample MakeImuSample(uint64_t t_ns, int index, const Eigen::Vector3d& specific_force,
                                    const Eigen::Vector3d& angular_velocity) {
  uw::domain::ImuSample sample;
  *sample.mutable_header() =
      MakeSyntheticHeader("imu_" + std::to_string(index), "imu0", "imu_link", t_ns);
  for (int i = 0; i < 3; ++i) {
    sample.add_linear_acceleration_mps2(specific_force(i));
    sample.add_angular_velocity_radps(angular_velocity(i));
  }
  return sample;
}

// Carries the keyframe's identity and its instant, and nothing else. No
// pose field is filled from ground truth -- that is the entire point of the
// topic (docs/imu-preintegration-design-2026-09-03.md section 8).
uw::domain::KeyframeBoundary MakeKeyframeBoundary(uint64_t t_ns, const std::string& kf_id) {
  uw::domain::KeyframeBoundary boundary;
  *boundary.mutable_header() =
      MakeSyntheticHeader("boundary_" + kf_id, "keyframe_scheduler", "base_link", t_ns);
  boundary.mutable_keyframe_id()->set_value(kf_id);
  boundary.set_source("synthetic_fixed_interval_v1");
  return boundary;
}

// Derives an independent RNG stream from the scenario's top-level seed for
// one specific noise purpose, via a distinguishing salt (arbitrary splitmix64
// constants, chosen only to differ from each other). This is what keeps
// per-target sonar noise draws (whose count varies with how many
// sonar_targets_world entries are in range at each keyframe) from desyncing
// the pose-noise stream: each purpose gets its own std::mt19937_64 seeded
// once from {seed, salt}, so how many draws one stream consumes never
// shifts what another stream produces. Before this, all three purposes
// shared one rng, and scenario/acoustic_optic_demo.yaml's single sonar
// target (vs. synthetic_smoke.yaml's three) silently changed the "seed 42"
// relative-pose noise actually baked into the bag.
std::mt19937_64 MakeStreamRng(uint64_t seed, uint64_t salt) {
  std::seed_seq seq{static_cast<uint32_t>(seed), static_cast<uint32_t>(seed >> 32),
                     static_cast<uint32_t>(salt), static_cast<uint32_t>(salt >> 32)};
  return std::mt19937_64(seq);
}

// --- Optional per-keyframe stereo images, only emitted when --experiment
// loads a rig with cameras (see main()'s `rig` capture below). Mirrors
// apps/acoustic_optic_scenarios.cpp's proven
// paint-background-then-paste-target technique (see that file's
// MakeStereoPair header comment for why the naive per-pixel approach is
// wrong), simplified: no noise/degradation variants — this app has no
// depth-accuracy scoring to protect, unlike that scenario matrix, just
// needs a working, honest scene for apps/replay_demo's acoustic-optic pass
// to run on.
constexpr uint32_t kCameraWidth = 640;
constexpr uint32_t kCameraHeight = 480;

// Dimmed vs. the original full [0,255) range: keeps background strictly
// below kLandmarkPatchMinIntensity (see LandmarkPatchIntensity) so a
// threshold-based landmark detector can't pick up background texture as a
// false landmark.
uint8_t StereoTexture(int u, int v) { return static_cast<uint8_t>(15 + ((u * 131 + v * 67 + 19) % 110)); }

// One 3D landmark's projected pixel footprint for one keyframe, plus the
// (fixed, scenario-wide) id used to look up its patch pattern — see
// LandmarkPatchIntensity. Not the landmark's identity in any tracking
// sense from a consumer's point of view: a real frontend must recover
// correspondence from patch appearance/geometry alone, the same as it
// would from a real camera, not from this id.
struct VisibleLandmark {
  int id = 0;
  Eigen::Vector3d camera_optical;
};

constexpr int kLandmarksPerKeyframe = 10;
constexpr int kLandmarkPatchHalfSize = 6;  // 13x13 px, small enough that a keyframe's cluster rarely collides
constexpr int kLandmarkPatchMinIntensity = 160;  // > StereoTexture's max (124), so patches threshold cleanly

// Landmark cloud scattered along the trajectory corridor, `kLandmarksPerKeyframe`
// clustered near EACH keyframe's own arc position (radius/depth still
// jittered, so the cluster isn't coplanar — a coplanar point set
// degenerates a rigid Kabsch/Procrustes fit). This replaces an earlier
// version that scattered points uniformly across the whole arc: with a
// fixed total landmark count, uniform scatter thins out badly per unit of
// travel once arc_radians/radius_m grow, so a camera's (narrow, unlike the
// sonar's ~6 rad) field of view could lose overlap with the previous
// keyframe within just a few steps — confirmed by running
// stereo_landmark_vo_frontend end-to-end (not just its unit tests, which
// use hand-built fixtures) and watching visible-landmark counts collapse
// from ~18 to 1 by keyframe 7. Anchoring density to each keyframe directly
// guarantees neighboring keyframes share a healthy landmark overlap
// regardless of trajectory length. Jitter is drawn from the caller's
// seeded `rng`, once, up front — never reseeded, never global (CLAUDE.md's
// RNG discipline / the L2 determinism test).
std::vector<Eigen::Vector3d> BuildVisualLandmarks(const ScenarioOptions& opt, std::mt19937_64& rng) {
  std::uniform_real_distribution<double> radius_jitter(0.6, 1.2);
  std::uniform_real_distribution<double> depth_jitter(-2.5, 2.5);
  const double keyframe_step_rad =
      opt.num_keyframes > 1 ? opt.arc_radians / (opt.num_keyframes - 1) : opt.arc_radians;
  std::uniform_real_distribution<double> theta_offset_jitter(-1.5 * keyframe_step_rad, 1.5 * keyframe_step_rad);

  std::vector<Eigen::Vector3d> landmarks;
  landmarks.reserve(static_cast<std::size_t>(kLandmarksPerKeyframe) * opt.num_keyframes);
  for (int kf = 0; kf < opt.num_keyframes; ++kf) {
    const double kf_t =
        opt.num_keyframes > 1 ? static_cast<double>(kf) / (opt.num_keyframes - 1) : 0.0;
    const double base_theta = kf_t * opt.arc_radians;
    for (int j = 0; j < kLandmarksPerKeyframe; ++j) {
      const double theta = base_theta + theta_offset_jitter(rng);
      const double radius = opt.radius_m * radius_jitter(rng);
      const double depth = -opt.depth_m + depth_jitter(rng);
      landmarks.emplace_back(radius * std::sin(theta), radius * (1.0 - std::cos(theta)), depth);
    }
  }
  return landmarks;
}

// Deterministic per-landmark pattern (a small position-dependent hash, not
// a uniform blob): gives each landmark id a reproducible but visually
// distinctive footprint so a future frontend can actually tell landmarks
// apart by patch appearance (normalized cross-correlation or similar)
// instead of all "features" looking identical, which no real matcher could
// disambiguate. Not learned, not ported from anywhere — same precedent as
// sonar_cfar_frontend's dbscan.hpp and block_matcher.hpp (see NOTICE).
uint8_t LandmarkPatchIntensity(int landmark_id, int du, int dv) {
  uint32_t h = static_cast<uint32_t>(landmark_id) * 2654435761u;
  h ^= static_cast<uint32_t>((du + kLandmarkPatchHalfSize) * (2 * kLandmarkPatchHalfSize + 1) +
                              (dv + kLandmarkPatchHalfSize)) *
       2246822519u;
  h ^= h >> 13;
  h *= 3266489917u;
  h ^= h >> 16;
  return static_cast<uint8_t>(kLandmarkPatchMinIntensity + (h % (256 - kLandmarkPatchMinIntensity)));
}

Pose3 FindRigEdgePose(const uw::domain::RigCalibrationSnapshot& rig, const std::string& child_frame) {
  for (const auto& edge : rig.frame_tree()) {
    if (edge.child_frame().value() == child_frame) return Pose3::FromProto(edge.transform());
  }
  return Pose3::Identity();
}

const uw::domain::CameraIntrinsics* FindRigCamera(const uw::domain::RigCalibrationSnapshot& rig,
                                                   const std::string& sensor_id) {
  for (const auto& camera : rig.cameras()) {
    if (camera.sensor_id().value() == sensor_id) return &camera;
  }
  return nullptr;
}

std::pair<uw::domain::ImageFrame, uw::domain::ImageFrame> BuildStereoPair(
    const uw::sensor_models::StereoGeometry& stereo_geometry,
    const std::vector<VisibleLandmark>& visible_landmarks, uint64_t t_ns, const std::string& kf_id) {
  constexpr double kBackgroundDepthM = 15.0;
  const int background_disparity_px = std::max(
      1, static_cast<int>(std::lround(stereo_geometry.left.fx * stereo_geometry.baseline_m / kBackgroundDepthM)));

  std::string left_pixels(static_cast<std::size_t>(kCameraWidth) * kCameraHeight, '\0');
  std::string right_pixels(static_cast<std::size_t>(kCameraWidth) * kCameraHeight, '\0');
  for (uint32_t v = 0; v < kCameraHeight; ++v) {
    for (uint32_t u = 0; u < kCameraWidth; ++u) {
      left_pixels[static_cast<std::size_t>(v) * kCameraWidth + u] =
          static_cast<char>(StereoTexture(static_cast<int>(u), static_cast<int>(v)));
      right_pixels[static_cast<std::size_t>(v) * kCameraWidth + u] =
          static_cast<char>(StereoTexture(static_cast<int>(u) + background_disparity_px, static_cast<int>(v)));
    }
  }

  // Each landmark's patch is painted into BOTH images at its own
  // depth-derived disparity, with identical content in both — unlike the
  // single-target trick this replaced (which only warped `right`, relying
  // on `left` already holding the same background texture value by
  // construction). That trick gave a locally-consistent disparity for
  // dense block matching but no single-frame-salient blob a landmark
  // frontend could detect or track between keyframes; painting both sides
  // makes each landmark an actual detectable, matchable feature.
  for (const auto& landmark : visible_landmarks) {
    if (landmark.camera_optical.z() <= 0.5) continue;
    const Eigen::Vector2d pixel = stereo_geometry.left.Project(landmark.camera_optical);
    const int center_u = static_cast<int>(std::lround(pixel.x()));
    const int center_v = static_cast<int>(std::lround(pixel.y()));
    const int disparity_px = std::max(
        1, static_cast<int>(std::lround(stereo_geometry.left.fx * stereo_geometry.baseline_m /
                                        landmark.camera_optical.z())));
    for (int dv = -kLandmarkPatchHalfSize; dv <= kLandmarkPatchHalfSize; ++dv) {
      const int v = center_v + dv;
      if (v < 0 || v >= static_cast<int>(kCameraHeight)) continue;
      for (int du = -kLandmarkPatchHalfSize; du <= kLandmarkPatchHalfSize; ++du) {
        const int u_left = center_u + du;
        const int u_right = u_left - disparity_px;
        if (u_left < 0 || u_left >= static_cast<int>(kCameraWidth) || u_right < 0 ||
            u_right >= static_cast<int>(kCameraWidth)) {
          continue;
        }
        const char intensity = static_cast<char>(LandmarkPatchIntensity(landmark.id, du, dv));
        left_pixels[static_cast<std::size_t>(v) * kCameraWidth + static_cast<std::size_t>(u_left)] = intensity;
        right_pixels[static_cast<std::size_t>(v) * kCameraWidth + static_cast<std::size_t>(u_right)] = intensity;
      }
    }
  }

  auto make_frame = [&](const std::string& frame_name, std::string pixels) {
    uw::domain::ImageFrame image;
    image.mutable_header()->mutable_observation_id()->set_value(kf_id);
    image.mutable_header()->mutable_sensor_frame()->set_value(frame_name);
    image.mutable_header()->mutable_sensor_id()->set_value(
        frame_name == "camera_left_link" ? "camera_left" : "camera_right");
    image.mutable_header()->mutable_capture_time()->set_seconds(static_cast<int64_t>(t_ns / 1'000'000'000ULL));
    image.mutable_header()->mutable_capture_time()->set_nanos(static_cast<int32_t>(t_ns % 1'000'000'000ULL));
    // Synthetic generation has no real transport delay to model — receive_time
    // equals capture_time (not left at the zero-Stamp default, which
    // tools/bag_audit reads as "never populated" rather than "instantaneous").
    *image.mutable_header()->mutable_receive_time() = image.header().capture_time();
    image.mutable_header()->set_clock_domain(uw::domain::CLOCK_DOMAIN_SIMULATION);
    image.mutable_header()->set_validity(uw::domain::ObservationHeader::VALIDITY_OK);
    image.mutable_header()->set_provenance("synth_bag_gen_v1");
    image.set_width(kCameraWidth);
    image.set_height(kCameraHeight);
    image.set_row_stride_bytes(kCameraWidth);
    image.set_encoding(uw::domain::ImageFrame::IMAGE_ENCODING_MONO8);
    image.set_pixel_data(std::move(pixels));
    image.set_is_rectified(true);
    return image;
  };
  return {make_frame("camera_left_link", std::move(left_pixels)),
          make_frame("camera_right_link", std::move(right_pixels))};
}

}  // namespace

int main(int argc, char** argv) {
  ScenarioOptions opt;
  // Only set when --experiment loads a rig with cameras — gates the new
  // /raw/camera/left,right emission below so the no-experiment path (used
  // by tests/l2_replay/determinism_test.sh) is provably unchanged.
  std::optional<uw::domain::RigCalibrationSnapshot> rig;
  // Set instead when --experiment selects estimator_mode: imu_preintegration.
  // Kept separate from `rig` above because that one is deliberately only
  // populated for a rig WITH cameras (it gates the stereo path), while the
  // IMU path needs the imu_noise block and the base_link->imu_link edge of a
  // camera-less rig such as configs/rig/example_auv_sonar_only.yaml.
  std::optional<uw::domain::RigCalibrationSnapshot> imu_rig;
  // Suppresses /evidence/relative_pose. The IMU estimator must ignore that
  // topic entirely, and a bag generated with and without it is how
  // tests/integration/synthetic_imu_fixture_test.sh shows the IMU stream
  // does not depend on it (each noise purpose draws from its own RNG
  // stream, see MakeStreamRng).
  bool omit_relative_pose = false;
  // Reference-branch-only knobs (PREP-B-01 Task 6). These change what an
  // EVALUATOR would read and nothing else: /gt/state is the only topic they
  // touch, they consume no RNG draws, and they deliberately do NOT move the
  // ground-truth messages' MCAP log time. That combination is what makes
  // them a leak detector -- if any of them changes a single byte of the
  // estimated trajectory, ground truth reached the algorithm path
  // (tests/integration/imu_preintegration_smoke_test.sh).
  bool omit_ground_truth = false;
  double ground_truth_time_offset_s = 0.0;
  double ground_truth_pose_offset_m = 0.0;

  // First pass: find --experiment, if any, and layer its scenario config
  // onto opt before any explicit CLI override is applied.
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--experiment") {
      if (i + 1 >= argc) {
        std::cerr << "missing value for --experiment\n";
        return 1;
      }
      const auto config = uw::runtime::LoadExperimentConfig(argv[++i]);
      ApplyScenarioConfig(config.scenario, opt);
      if (config.rig.cameras_size() > 0) rig = config.rig;
      if (config.estimator_mode == "imu_preintegration") imu_rig = config.rig;
    }
  }

  // Second pass: explicit flags override whatever --experiment set.
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&](const char* flag) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "missing value for " << flag << "\n";
        std::exit(1);
      }
      return argv[++i];
    };
    if (arg == "--experiment") {
      next("--experiment");  // already consumed above; just skip its value here
    } else if (arg == "--out") {
      opt.out_path = next("--out");
    } else if (arg == "--num-keyframes") {
      opt.num_keyframes = std::stoi(next("--num-keyframes"));
    } else if (arg == "--seed") {
      opt.seed = std::stoull(next("--seed"));
    } else if (arg == "--omit-relative-pose") {
      omit_relative_pose = true;
    } else if (arg == "--omit-ground-truth") {
      omit_ground_truth = true;
    } else if (arg == "--ground-truth-time-offset-s") {
      ground_truth_time_offset_s = std::stod(next("--ground-truth-time-offset-s"));
    } else if (arg == "--ground-truth-pose-offset-m") {
      ground_truth_pose_offset_m = std::stod(next("--ground-truth-pose-offset-m"));
    } else {
      std::cerr << "unknown argument: " << arg << "\n";
      return 1;
    }
  }

  std::mt19937_64 pose_rng = MakeStreamRng(opt.seed, 0x9E3779B97F4A7C15ULL);
  std::mt19937_64 sonar_rng = MakeStreamRng(opt.seed, 0xC2B2AE3D27D4EB4FULL);
  std::mt19937_64 landmark_rng = MakeStreamRng(opt.seed, 0x165667B19E3779F9ULL);
  // Fourth independent stream, same reasoning as the three above: the IMU
  // runs at 200 Hz and draws a variable number of times relative to
  // everything else, so it must not share a stream with them in either
  // direction.
  std::mt19937_64 imu_rng = MakeStreamRng(opt.seed, 0xD6E8FEB86659FD93ULL);
  std::normal_distribution<double> pose_noise(0.0, opt.relative_pose_noise_m);
  std::normal_distribution<double> range_noise(0.0, opt.sonar_range_noise_m);
  std::normal_distribution<double> bearing_noise(0.0, opt.sonar_bearing_noise_rad);

  // The IMU fixture and everything that follows from it -- the stationary
  // pre-roll, the time shift, the explicit keyframe boundaries, and the
  // smooth-start angular profile -- are all gated on this single flag, so
  // every other experiment's bag stays byte-identical.
  const bool imu_mode = imu_rig.has_value();
  const double pre_roll_s = imu_mode ? kImuPreRollS : 0.0;
  const uint64_t pre_roll_ns = SecondsToNanos(pre_roll_s);
  const double motion_duration_s = std::max(0, opt.num_keyframes - 1) * kKeyframePeriodS;

  const auto trajectory = BuildGroundTruthTrajectory(opt, /*smooth_start=*/imu_mode);
  const auto targets = BuildSonarTargets(opt);
  uw::runtime::SyntheticSonarFrameSpec sonar_frame_spec;
  // Matches configs/rig/*.yaml's sonar_beam_models[0].sensor_id and
  // AcousticOpticAssociatorParams::sonar_sensor_id's default -- required
  // (not optional metadata) since SynchronizeAcousticOptic() rejects an
  // empty sensor_id as an invalid timestamp header.
  sonar_frame_spec.sensor_id = "sonar0";
  sonar_frame_spec.provenance = "synth_bag_gen_v1";
  // Only meaningful when a camera rig is loaded (see `rig` above); drawn from
  // its own stream (landmark_rng), so whether or how much of it gets
  // consumed can never perturb the pose/sonar streams above.
  const std::vector<Eigen::Vector3d> visual_landmarks =
      rig.has_value() ? BuildVisualLandmarks(opt, landmark_rng) : std::vector<Eigen::Vector3d>{};

  uw::runtime::McapProtobufWriter writer;
  if (!writer.Open(opt.out_path)) {
    std::cerr << "failed to open " << opt.out_path << " for writing\n";
    return 1;
  }

  // Scenario-level target list, once.
  {
    uw::domain::MapEvidence targets_evidence;
    targets_evidence.mutable_keyframe_id()->set_value("scenario");
    targets_evidence.mutable_local_frame()->set_value("world");
    targets_evidence.set_representation_type(uw::domain::MAP_REPRESENTATION_POINT_CLOUD);
    std::string bytes(targets.size() * 3 * sizeof(float), '\0');
    auto* raw = reinterpret_cast<float*>(bytes.data());
    for (std::size_t i = 0; i < targets.size(); ++i) {
      raw[i * 3 + 0] = static_cast<float>(targets[i].x());
      raw[i * 3 + 1] = static_cast<float>(targets[i].y());
      raw[i * 3 + 2] = static_cast<float>(targets[i].z());
    }
    targets_evidence.set_geometry_or_occupancy(bytes);
    writer.WriteMessage("/scenario/sonar_targets", 0, targets_evidence);
  }

  // --- IMU stream (estimator_mode: imu_preintegration only) -------------
  // Written in one pass ahead of the keyframe loop, spanning [0, last
  // keyframe] so every preintegration interval is fully covered on both
  // ends. Consumers read a bag in log-time order (McapEventSource sets
  // LogTimeOrder explicitly), so writing this block first does not put the
  // IMU ahead of the keyframe topics in time.
  if (imu_mode) {
    const auto& imu_noise = imu_rig->imu_noise();
    const double rate_hz = imu_noise.rate_hz();
    const double gravity_mps2 = imu_noise.gravity_mps2();
    if (!(rate_hz > 0.0) || !(gravity_mps2 > 0.0)) {
      std::cerr << "rig imu_noise.rate_hz and gravity_mps2 must be positive for "
                   "estimator_mode: imu_preintegration\n";
      return 1;
    }
    const double dt_s = 1.0 / rate_hz;
    const Pose3 base_link_T_imu_link = FindRigEdgePose(*imu_rig, "imu_link");

    // Continuous-time densities discretize to sigma_c * sqrt(rate); the bias
    // random walk uses sigma_walk_c * sqrt(dt). sigma_*_bias is the INITIAL
    // bias magnitude only -- the two are separate knobs by decision, see the
    // design note's section 9 item 3 -- so it seeds the constant offset and
    // never the walk.
    std::normal_distribution<double> accel_white(0.0, imu_noise.sigma_accel_c() * std::sqrt(rate_hz));
    std::normal_distribution<double> gyro_white(0.0, imu_noise.sigma_gyro_c() * std::sqrt(rate_hz));
    std::normal_distribution<double> accel_bias_initial(0.0, imu_noise.sigma_accel_bias());
    std::normal_distribution<double> gyro_bias_initial(0.0, imu_noise.sigma_gyro_bias());
    std::normal_distribution<double> accel_bias_step(
        0.0, imu_noise.sigma_accel_bias_walk_c() * std::sqrt(dt_s));
    std::normal_distribution<double> gyro_bias_step(
        0.0, imu_noise.sigma_gyro_bias_walk_c() * std::sqrt(dt_s));

    Eigen::Vector3d accel_bias(accel_bias_initial(imu_rng), accel_bias_initial(imu_rng),
                               accel_bias_initial(imu_rng));
    Eigen::Vector3d gyro_bias(gyro_bias_initial(imu_rng), gyro_bias_initial(imu_rng),
                              gyro_bias_initial(imu_rng));

    const double total_duration_s = pre_roll_s + motion_duration_s;
    const int imu_sample_count = static_cast<int>(std::llround(total_duration_s * rate_hz));
    for (int sample_index = 0; sample_index <= imu_sample_count; ++sample_index) {
      const double t_s = static_cast<double>(sample_index) / rate_hz;
      // Strictly inside the pre-roll the body is at rest. The sample landing
      // exactly on the first keyframe boundary is still a stationary one,
      // which is what makes the pre-roll a full 0.5 s of stationary data
      // rather than 0.5 s minus one sample.
      const ImuTruth body_truth =
          t_s <= pre_roll_s
              ? StationaryImuTruth(gravity_mps2)
              : BodyImuTruth(opt, motion_duration_s,
                             std::min(1.0, (t_s - pre_roll_s) / motion_duration_s), gravity_mps2);
      const ImuTruth sensor_truth = ToImuFrame(body_truth, base_link_T_imu_link);

      const Eigen::Vector3d measured_accel =
          sensor_truth.specific_force_mps2 + accel_bias +
          Eigen::Vector3d(accel_white(imu_rng), accel_white(imu_rng), accel_white(imu_rng));
      const Eigen::Vector3d measured_gyro =
          sensor_truth.angular_velocity_radps + gyro_bias +
          Eigen::Vector3d(gyro_white(imu_rng), gyro_white(imu_rng), gyro_white(imu_rng));
      writer.WriteMessage(uw::runtime::kTopicImu, SecondsToNanos(t_s),
                          MakeImuSample(SecondsToNanos(t_s), sample_index, measured_accel,
                                        measured_gyro));

      accel_bias += Eigen::Vector3d(accel_bias_step(imu_rng), accel_bias_step(imu_rng),
                                    accel_bias_step(imu_rng));
      gyro_bias += Eigen::Vector3d(gyro_bias_step(imu_rng), gyro_bias_step(imu_rng),
                                   gyro_bias_step(imu_rng));
    }
  }

  for (int i = 0; i < opt.num_keyframes; ++i) {
    // 5 Hz keyframes, offset by the stationary pre-roll in IMU mode (zero
    // in every other mode, so those bags keep their original timestamps).
    const uint64_t t_ns = pre_roll_ns + static_cast<uint64_t>(i) * 200'000'000ULL;
    const std::string kf_id = KeyframeId(i);

    // Keyframe boundary first: it is the event that declares this keyframe
    // exists, and everything below is evidence attached to it.
    if (imu_mode) {
      writer.WriteMessage(uw::runtime::kTopicKeyframeBoundary, t_ns,
                          MakeKeyframeBoundary(t_ns, kf_id));
    }

    // Ground truth. Reference-only: no algorithm input is derived from it
    // (see the /gt/state role in include/runtime/canonical_topics.hpp), and
    // the three --*-ground-truth-* flags below exist to prove that.
    if (!omit_ground_truth) {
      uw::domain::StateSnapshot gt;
      gt.mutable_state_id()->set_value(kf_id);
      // The tampering is applied to the CAPTURE TIMESTAMP only; the MCAP
      // log time stays t_ns. A reader that (incorrectly) reconstructed
      // keyframe times from log time would otherwise see nothing move.
      const double gt_time_s =
          static_cast<double>(t_ns) * 1e-9 + ground_truth_time_offset_s;
      *gt.mutable_capture_timestamp() = uw::domain::FromSeconds(gt_time_s);
      Pose3 gt_pose = trajectory[i];
      gt_pose.translation += Eigen::Vector3d::Constant(ground_truth_pose_offset_m);
      *gt.mutable_pose_wb() = gt_pose.ToProto();
      writer.WriteMessage("/gt/state", t_ns, gt);
    }

    // Relative pose evidence (black-box VIO mode, section 8.1) between
    // consecutive keyframes, with additive translation noise.
    if (i > 0 && !omit_relative_pose) {
      const Pose3 true_relative = trajectory[i - 1].Inverse() * trajectory[i];
      Pose3 noisy_relative = true_relative;
      noisy_relative.translation +=
          Eigen::Vector3d(pose_noise(pose_rng), pose_noise(pose_rng), pose_noise(pose_rng));

      uw::domain::RelativePoseMeasurement measurement;
      measurement.mutable_from_keyframe()->set_value(KeyframeId(i - 1));
      measurement.mutable_to_keyframe()->set_value(kf_id);
      *measurement.mutable_relative_pose() = noisy_relative.ToProto();

      uw::domain::EvidenceId evidence_id;
      evidence_id.set_value("relpose_" + kf_id);
      std::vector<uw::domain::ObservationId> sources;
      auto evidence = uw::domain::MakeEvidence<uw::domain::RelativePoseMeasurement>(
          evidence_id, sources, measurement, /*noise_scale=*/1.0, "synth_bag_gen_v1");
      writer.WriteMessage("/evidence/relative_pose", t_ns, evidence);
    }

    // Sonar: one synthetic ping per target within plausible range, run
    // through the real sonar_cfar_frontend by apps/replay_demo (not
    // pre-computed range-bearing evidence — see file header and
    // RenderSyntheticSonarFrame). One frame per target-in-range, rather than
    // one frame covering all of them, is a synthetic-demo simplification —
    // matches apps/replay_demo's documented landmark-association stand-in,
    // not a general multi-target-per-ping sensor model.
    for (const auto& target : targets) {
      const Eigen::Vector3d local = trajectory[i].Inverse().Apply(target);
      const double range = local.norm();
      if (range > 12.0) continue;  // out of sonar range for this keyframe
      const double bearing = std::atan2(local.y(), local.x());
      const double noisy_range = range + range_noise(sonar_rng);
      const double noisy_bearing = bearing + bearing_noise(sonar_rng);
      sonar_frame_spec.observation_id = kf_id;
      sonar_frame_spec.timestamp_ns = t_ns;
      auto rendered = uw::runtime::RenderSyntheticSonarFrame(
          sonar_frame_spec, noisy_range, noisy_bearing);
      if (!rendered.target_rendered) {
        std::cerr << "warning: target for " << kf_id
                  << " falls outside the synthetic sonar frame (range=" << noisy_range
                  << "m bearing=" << noisy_bearing
                  << "rad) — frame written background-only\n";
      }
      writer.WriteMessage("/raw/sonar_frame", t_ns, rendered.frame);
    }

    // Stereo images (only when a rig with cameras was loaded via
    // --experiment): find the nearest target actually inside the camera's
    // — narrower than the sonar's — field of view, and build a real
    // per-keyframe stereo pair via BuildStereoPair.
    if (rig.has_value()) {
      const auto* left_intrinsics = FindRigCamera(*rig, "camera_left");
      const auto* right_intrinsics = FindRigCamera(*rig, "camera_right");
      if (left_intrinsics != nullptr && right_intrinsics != nullptr) {
        const auto stereo_geometry = uw::sensor_models::StereoGeometry::Resolve(
            *rig, "camera_left", "camera_left_link", "camera_right", "camera_right_link");
        if (stereo_geometry.valid) {
          const Pose3 camera_pose = FindRigEdgePose(*rig, "camera_left_link");
          std::vector<VisibleLandmark> visible;
          for (std::size_t li = 0; li < visual_landmarks.size(); ++li) {
            const Eigen::Vector3d local_body = trajectory[i].Inverse().Apply(visual_landmarks[li]);
            const Eigen::Vector3d local_camera_body = camera_pose.Inverse().Apply(local_body);
            const Eigen::Vector3d local_optical =
                uw::sensor_models::OpticalFromBodyRotation() * local_camera_body;
            if (local_optical.z() <= 0.5) continue;  // behind or too close to the camera
            const Eigen::Vector2d pixel = stereo_geometry.left.Project(local_optical);
            if (pixel.x() < 0 || pixel.x() >= stereo_geometry.left.width || pixel.y() < 0 ||
                pixel.y() >= stereo_geometry.left.height) {
              continue;  // outside the camera's (narrower than sonar) field of view
            }
            visible.push_back(VisibleLandmark{static_cast<int>(li), local_optical});
          }
          // Also paint sonar targets that happen to fall inside the
          // camera's (narrower than sonar) FOV: without this, the stereo
          // pair's disparity at a sonar target's projected pixel always
          // reads back the flat kBackgroundDepthM plane (BuildStereoPair
          // never otherwise knows a target is there), so
          // AcousticOpticAssociator's range gate can never find a matching
          // optical candidate and every association is REJECTED/
          // NO_CANDIDATE regardless of geometry -- scenarios that exist
          // specifically to demonstrate a real ACCEPTED association (e.g.
          // configs/scenario/acoustic_optic_demo.yaml) need this. A large id
          // offset keeps each target's patch pattern (see
          // LandmarkPatchIntensity) distinct from any visual landmark's.
          for (std::size_t ti = 0; ti < targets.size(); ++ti) {
            const Eigen::Vector3d local_body = trajectory[i].Inverse().Apply(targets[ti]);
            const Eigen::Vector3d local_camera_body = camera_pose.Inverse().Apply(local_body);
            const Eigen::Vector3d local_optical =
                uw::sensor_models::OpticalFromBodyRotation() * local_camera_body;
            if (local_optical.z() <= 0.5) continue;
            const Eigen::Vector2d pixel = stereo_geometry.left.Project(local_optical);
            if (pixel.x() < 0 || pixel.x() >= stereo_geometry.left.width || pixel.y() < 0 ||
                pixel.y() >= stereo_geometry.left.height) {
              continue;
            }
            visible.push_back(VisibleLandmark{static_cast<int>(100000 + ti), local_optical});
          }
          auto stereo_pair = BuildStereoPair(stereo_geometry, visible, t_ns, kf_id);
          writer.WriteMessage("/raw/camera/left", t_ns, stereo_pair.first);
          writer.WriteMessage("/raw/camera/right", t_ns, stereo_pair.second);
        }
      }
    }

    // Depth.
    {
      uw::domain::PressureDepthMeasurement measurement;
      // depth_m is positive-down (world Z-up); negate the pose z to produce
      // it — see PressureDepthMeasurement's field comment in
      // schemas/proto/uw/domain/measurement.proto.
      measurement.set_depth_m(-trajectory[i].translation.z());
      measurement.set_sigma_m(0.05);
      uw::domain::EvidenceId evidence_id;
      evidence_id.set_value("depth_" + kf_id);
      uw::domain::ObservationId observation_id;
      observation_id.set_value(kf_id);
      std::vector<uw::domain::ObservationId> sources{observation_id};
      auto evidence = uw::domain::MakeEvidence<uw::domain::PressureDepthMeasurement>(
          evidence_id, sources, measurement, /*noise_scale=*/1.0, "synth_bag_gen_v1");
      writer.WriteMessage("/evidence/depth", t_ns, evidence);
    }
  }

  writer.Close();
  std::cout << "wrote " << opt.num_keyframes << " keyframes to " << opt.out_path << "\n";
  return 0;
}
