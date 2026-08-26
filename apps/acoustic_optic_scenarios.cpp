#include "acoustic_optic_scenarios.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <tuple>
#include <utility>

#include "runtime/synthetic_sonar.hpp"
#include "sensor_models/camera_model.hpp"
#include "sensor_models/sonar_arc_projector.hpp"

namespace uw::scenario_matrix {

namespace {

constexpr uint32_t kWidth = 640;
constexpr uint32_t kHeight = 480;

using uw::sensor_models::FindCamera;

uw::sensor_models::Pose3 FindEdgePose(const uw::domain::RigCalibrationSnapshot& rig,
                                      const std::string& child_frame) {
  for (const auto& edge : rig.frame_tree()) {
    if (edge.child_frame().value() == child_frame) return uw::sensor_models::Pose3::FromProto(edge.transform());
  }
  return uw::sensor_models::Pose3::Identity();
}

uint8_t BaseTexture(int u, int v, int block_size, int repeated_period) {
  int su = u;
  if (repeated_period > 0) su = u % repeated_period;
  const int bu = su / block_size, bv = v / block_size;
  return static_cast<uint8_t>((bu * 131 + bv * 67 + 19) % 256);
}

// Builds LEFT (the plain, unshifted reference canvas) and RIGHT (background
// disparity painted everywhere, THEN the target patch pasted on top,
// shifted by target_disparity_px, sourced from LEFT's own content at each
// pasted column) together, so RIGHT is indexed consistently by its own
// coordinate with content SOURCED from the correct LEFT column — standard
// foreground-occludes-background stereo compositing.
//
// An earlier version instead evaluated "which disparity applies" directly
// on the pixel being written in EACH image separately (RIGHT(u,v) =
// LEFT(u + disparity_at(u,v))). That is subtly wrong: for a LEFT pixel
// u_left near the patch center, the disparity-8 hypothesis reads RIGHT
// columns [u_left-8-radius, u_left-8+radius] — a range shifted by a further
// 8px from the patch's OWN boundary in the SAME coordinate space — so
// "clean" recoverable depth only started ~radius+target_disparity_px from
// the boundary, not ~radius as intended, corrupting depth at and near the
// patch center itself. Comparing the recovered depth against the analytic
// sonar range in the end-to-end scenario exposed the error.
std::pair<uw::domain::ImageFrame, uw::domain::ImageFrame> MakeStereoPair(
    int block_size, double noise_std, int repeated_period, int patch_center_u, int patch_center_v,
    int patch_half_size, int target_disparity_px, int background_disparity_px, std::mt19937_64& rng) {
  auto init_header = [](uw::domain::ImageFrame& image, const std::string& sensor_id,
                        const std::string& frame) {
    image.mutable_header()->mutable_sensor_id()->set_value(sensor_id);
    image.mutable_header()->mutable_sensor_frame()->set_value(frame);
    image.mutable_header()->set_clock_domain(uw::domain::CLOCK_DOMAIN_SIMULATION);
    image.mutable_header()->set_validity(uw::domain::ObservationHeader::VALIDITY_OK);
    image.mutable_header()->set_provenance("acoustic_optic_scenario_matrix_v1");
    image.set_width(kWidth);
    image.set_height(kHeight);
    image.set_row_stride_bytes(kWidth);
    image.set_encoding(uw::domain::ImageFrame::IMAGE_ENCODING_MONO8);
    image.set_is_rectified(true);
  };

  uw::domain::ImageFrame left, right;
  init_header(left, "camera_left", "camera_left_link");
  init_header(right, "camera_right", "camera_right_link");

  std::string left_pixels(static_cast<std::size_t>(kWidth) * kHeight, '\0');
  std::string right_pixels(static_cast<std::size_t>(kWidth) * kHeight, '\0');

  for (uint32_t v = 0; v < kHeight; ++v) {
    for (uint32_t u = 0; u < kWidth; ++u) {
      left_pixels[static_cast<std::size_t>(v) * kWidth + u] =
          static_cast<char>(static_cast<uint8_t>(BaseTexture(static_cast<int>(u), static_cast<int>(v), block_size, repeated_period)));
      right_pixels[static_cast<std::size_t>(v) * kWidth + u] = static_cast<char>(static_cast<uint8_t>(
          BaseTexture(static_cast<int>(u) + background_disparity_px, static_cast<int>(v), block_size, repeated_period)));
    }
  }
  for (int dv = -patch_half_size; dv <= patch_half_size; ++dv) {
    const int v = patch_center_v + dv;
    if (v < 0 || v >= static_cast<int>(kHeight)) continue;
    for (int du = -patch_half_size; du <= patch_half_size; ++du) {
      const int u_left = patch_center_u + du;
      const int u_right = u_left - target_disparity_px;
      if (u_left < 0 || u_left >= static_cast<int>(kWidth) || u_right < 0 || u_right >= static_cast<int>(kWidth)) {
        continue;
      }
      right_pixels[static_cast<std::size_t>(v) * kWidth + static_cast<std::size_t>(u_right)] =
          static_cast<char>(static_cast<uint8_t>(BaseTexture(u_left, v, block_size, repeated_period)));
    }
  }

  if (noise_std > 0.0) {
    std::normal_distribution<double> noise(0.0, noise_std);
    for (auto* pixels : {&left_pixels, &right_pixels}) {
      for (auto& c : *pixels) {
        const double value = std::clamp(static_cast<double>(static_cast<uint8_t>(c)) + noise(rng), 0.0, 255.0);
        c = static_cast<char>(static_cast<uint8_t>(value));
      }
    }
  }

  left.set_pixel_data(left_pixels);
  right.set_pixel_data(right_pixels);
  return {left, right};
}

}  // namespace

const std::vector<ScenarioSpec>& AllScenarios() {
  static const std::vector<ScenarioSpec> kScenarios = {
      {ScenarioKind::kCleanTextured, "clean_textured"},
      {ScenarioKind::kLowTextureSonarVisible, "low_texture_sonar_visible"},
      {ScenarioKind::kTurbidSonarVisible, "turbid_sonar_visible"},
      {ScenarioKind::kRepeatedStructure, "repeated_structure"},
      {ScenarioKind::kElevationStress, "elevation_stress"},
      {ScenarioKind::kTimeOffsetFault, "time_offset_fault"},
      {ScenarioKind::kExtrinsicPerturbation, "extrinsic_perturbation"},
      {ScenarioKind::kSonarDropout, "sonar_dropout"},
      {ScenarioKind::kOpticalInvalidRegion, "optical_invalid_region"},
  };
  return kScenarios;
}

SyntheticTrial BuildTrial(ScenarioKind kind, const uw::domain::RigCalibrationSnapshot& true_rig,
                          uint64_t trial_seed) {
  std::mt19937_64 rng(trial_seed);
  SyntheticTrial trial;
  trial.width = kWidth;
  trial.height = kHeight;
  trial.pipeline_rig = true_rig;

  const auto* camera_intrinsics = FindCamera(true_rig, "camera_left");
  const auto camera = uw::sensor_models::PinholeCamera::FromIntrinsics(*camera_intrinsics);
  const auto camera_pose = FindEdgePose(true_rig, "camera_left_link");
  const auto sonar_pose = FindEdgePose(true_rig, "sonar_link");
  const uw::sensor_models::Pose3 camera_T_sonar = camera_pose.Inverse() * sonar_pose;

  const auto stereo_geometry = uw::sensor_models::StereoGeometry::Resolve(
      true_rig, "camera_left", "camera_left_link", "camera_right", "camera_right_link");
  const double fx_baseline = stereo_geometry.left.fx * stereo_geometry.baseline_m;
  // Round to the nearest integer pixel disparity (the block matcher has no
  // sub-pixel refinement) so the GT depth used for scoring is EXACTLY what
  // the real matcher can recover on a clean, noiseless region — matching
  // synth_stereo_gen's convention (self-consistency caught a real bug
  // here: an earlier version of this scenario used a hand-picked GT depth
  // that didn't correspond to any achievable integer disparity, producing a
  // spurious ~0.3m systematic RMSE that looked like a pipeline defect).
  const int target_disparity_px = std::max(1, static_cast<int>(std::lround(fx_baseline / 6.0)));
  const int background_disparity_px = std::max(1, static_cast<int>(std::lround(fx_baseline / 10.0)));
  trial.gt_target_depth_m = fx_baseline / target_disparity_px;
  trial.gt_background_depth_m = fx_baseline / background_disparity_px;

  trial.patch_center_u = static_cast<int>(kWidth / 2);
  trial.patch_center_v = static_cast<int>(kHeight / 2);
  // Small: the arc projector samples elevation at ~5px/step near boresight
  // (aperture_rad/arc_samples * fx) — a patch much larger than that step
  // creates genuine, physically-real elevation ambiguity by containing
  // several equally-flat candidate pixels, which is a real FLS limitation,
  // not a scenario bug. Only
  // needs to clear the block matcher's own window_radius (3px default) as
  // margin now that MakeStereoPair sources pasted content directly from
  // LEFT's own column (see that function's header comment) — half_size=6
  // keeps 1-2 arc samples inside for a cleanly-resolvable single candidate.
  trial.patch_half_size = 6;

  int block_size = 1, repeated_period = -1;
  double noise_std = 0.0;
  bool omit_sonar = false;
  bool invalidate_target_pixel = false;
  // Which pixel the sonar-consistent 3D point is derived from — normally
  // the patch center, but shifted for elevation_stress so the true point
  // sits off the sonar's central elevation (nonzero phi) while remaining
  // inside the (re-centered) target patch.
  int sonar_pixel_u = trial.patch_center_u;
  int sonar_pixel_v = trial.patch_center_v;

  switch (kind) {
    case ScenarioKind::kCleanTextured:
      break;
    case ScenarioKind::kLowTextureSonarVisible:
      block_size = 8;
      noise_std = 8.0;
      break;
    case ScenarioKind::kTurbidSonarVisible:
      noise_std = 45.0;
      break;
    case ScenarioKind::kRepeatedStructure:
      repeated_period = target_disparity_px;  // aliasing hazard: one period == the true disparity
      break;
    case ScenarioKind::kElevationStress:
      sonar_pixel_v = trial.patch_center_v - 20;  // ~20px vertical offset -> nonzero sonar-frame elevation
      trial.patch_center_v = sonar_pixel_v;        // re-center the patch so the true point stays inside it
      break;
    case ScenarioKind::kTimeOffsetFault:
    case ScenarioKind::kExtrinsicPerturbation:
      break;
    case ScenarioKind::kSonarDropout:
      omit_sonar = true;
      break;
    case ScenarioKind::kOpticalInvalidRegion:
      invalidate_target_pixel = true;
      break;
  }

  std::tie(trial.left, trial.right) =
      MakeStereoPair(block_size, noise_std, repeated_period, trial.patch_center_u, trial.patch_center_v,
                     trial.patch_half_size, target_disparity_px, background_disparity_px, rng);
  if (invalidate_target_pixel) {
    // Overwrite the ENTIRE target patch (plus a margin, so no adjacent
    // correctly-depthed pixel survives at the edge) in both images with
    // flat, textureless intensity — the block matcher then has no basis to
    // report ANY disparity there (mirrors a real specular-highlight/
    // occlusion optical dropout), so the optical prior is genuinely
    // invalid everywhere the sonar target could land, not just at one
    // pixel (an earlier version invalidated a fixed 9x9 box, far smaller
    // than the target patch, leaving plenty of still-valid nearby pixels
    // and defeating the scenario's purpose).
    const int invalidate_half_size = trial.patch_half_size + 3;
    for (int dv = -invalidate_half_size; dv <= invalidate_half_size; ++dv) {
      for (int du = -invalidate_half_size; du <= invalidate_half_size; ++du) {
        const int u = trial.patch_center_u + du, v = trial.patch_center_v + dv;
        if (u < 0 || u >= static_cast<int>(kWidth) || v < 0 || v >= static_cast<int>(kHeight)) continue;
        const std::size_t idx = static_cast<std::size_t>(v) * kWidth + static_cast<std::size_t>(u);
        trial.left.mutable_pixel_data()->at(idx) = static_cast<char>(128);
        trial.right.mutable_pixel_data()->at(idx) = static_cast<char>(128);
      }
    }
  }

  const auto sonar_observation = uw::sensor_models::UnprojectPixelToSonarRangeBearing(
      sonar_pixel_u, sonar_pixel_v, trial.gt_target_depth_m, camera_T_sonar, camera);

  trial.expected_sonar_range_m = sonar_observation.range_m;
  trial.expected_sonar_bearing_rad = sonar_observation.bearing_rad;

  if (!omit_sonar) {
    uw::runtime::SyntheticSonarFrameSpec sonar_spec;
    sonar_spec.sensor_id = "sonar0";
    sonar_spec.provenance = "acoustic_optic_scenario_matrix_v1";
    trial.sonar = uw::runtime::RenderSyntheticSonarFrame(
                      sonar_spec, sonar_observation.range_m,
                      sonar_observation.bearing_rad)
                      .frame;
    if (kind == ScenarioKind::kTimeOffsetFault) {
      trial.sonar->mutable_header()->mutable_capture_time()->set_seconds(1);  // 1s after camera's default 0
    }
  }

  if (kind == ScenarioKind::kExtrinsicPerturbation) {
    for (auto& edge : *trial.pipeline_rig.mutable_frame_tree()) {
      if (edge.child_frame().value() != "sonar_link") continue;
      // Perturb the sonar's x-translation by 1.0m — a grossly wrong
      // calibration (not a subtle drift), chosen to unambiguously exceed
      // the associator's default range/bearing gates so this scenario
      // demonstrates fail-closed rejection rather than a borderline case.
      // (A 0.3m perturbation was tried first and passed the default gates
      // silently, producing 100% false-fusion — a real finding about gate
      // sensitivity to SMALL calibration errors, worth a documented
      // follow-up, but not what this scenario is meant to demonstrate.)
      edge.mutable_transform()->set_matrix_row_major(3, edge.transform().matrix_row_major(3) + 1.0);
    }
  }

  return trial;
}

}  // namespace uw::scenario_matrix
