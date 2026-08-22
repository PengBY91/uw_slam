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

std::vector<Pose3> BuildGroundTruthTrajectory(const ScenarioOptions& opt) {
  std::vector<Pose3> trajectory;
  trajectory.reserve(opt.num_keyframes);
  for (int i = 0; i < opt.num_keyframes; ++i) {
    const double t = opt.num_keyframes > 1
                          ? static_cast<double>(i) / (opt.num_keyframes - 1)
                          : 0.0;
    const double theta = t * opt.arc_radians;
    Pose3 pose;
    pose.translation =
        Eigen::Vector3d(opt.radius_m * std::sin(theta), opt.radius_m * (1.0 - std::cos(theta)),
                        -opt.depth_m);
    pose.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()));
    trajectory.push_back(pose);
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
    const std::vector<VisibleLandmark>& visible_landmarks, uint64_t t_ns) {
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
    image.mutable_header()->mutable_sensor_frame()->set_value(frame_name);
    image.mutable_header()->mutable_sensor_id()->set_value(
        frame_name == "camera_left_link" ? "camera_left" : "camera_right");
    image.mutable_header()->mutable_capture_time()->set_seconds(static_cast<int64_t>(t_ns / 1'000'000'000ULL));
    image.mutable_header()->mutable_capture_time()->set_nanos(static_cast<int32_t>(t_ns % 1'000'000'000ULL));
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
    } else {
      std::cerr << "unknown argument: " << arg << "\n";
      return 1;
    }
  }

  std::mt19937_64 rng(opt.seed);
  std::normal_distribution<double> pose_noise(0.0, opt.relative_pose_noise_m);
  std::normal_distribution<double> range_noise(0.0, opt.sonar_range_noise_m);
  std::normal_distribution<double> bearing_noise(0.0, opt.sonar_bearing_noise_rad);

  const auto trajectory = BuildGroundTruthTrajectory(opt);
  const auto targets = BuildSonarTargets(opt);
  uw::runtime::SyntheticSonarFrameSpec sonar_frame_spec;
  sonar_frame_spec.provenance = "synth_bag_gen_v1";
  // Only meaningful when a camera rig is loaded (see `rig` above); drawn
  // here, after `rng` exists but before the per-keyframe loop below, so it
  // consumes a fixed slice of the seeded stream without perturbing the
  // per-keyframe pose/range/bearing draws that follow.
  const std::vector<Eigen::Vector3d> visual_landmarks =
      rig.has_value() ? BuildVisualLandmarks(opt, rng) : std::vector<Eigen::Vector3d>{};

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

  for (int i = 0; i < opt.num_keyframes; ++i) {
    const uint64_t t_ns = static_cast<uint64_t>(i) * 200'000'000ULL;  // 5 Hz keyframes
    const std::string kf_id = KeyframeId(i);

    // Ground truth.
    uw::domain::StateSnapshot gt;
    gt.mutable_state_id()->set_value(kf_id);
    gt.mutable_capture_timestamp()->set_seconds(static_cast<int64_t>(t_ns / 1'000'000'000ULL));
    gt.mutable_capture_timestamp()->set_nanos(static_cast<int32_t>(t_ns % 1'000'000'000ULL));
    *gt.mutable_pose_wb() = trajectory[i].ToProto();
    writer.WriteMessage("/gt/state", t_ns, gt);

    // Relative pose evidence (black-box VIO mode, section 8.1) between
    // consecutive keyframes, with additive translation noise.
    if (i > 0) {
      const Pose3 true_relative = trajectory[i - 1].Inverse() * trajectory[i];
      Pose3 noisy_relative = true_relative;
      noisy_relative.translation +=
          Eigen::Vector3d(pose_noise(rng), pose_noise(rng), pose_noise(rng));

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
      const double noisy_range = range + range_noise(rng);
      const double noisy_bearing = bearing + bearing_noise(rng);
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
          auto stereo_pair = BuildStereoPair(stereo_geometry, visible, t_ns);
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
