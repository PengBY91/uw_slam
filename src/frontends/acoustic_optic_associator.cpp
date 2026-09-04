#include "frontends/acoustic_optic_associator.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>

#include "sensor_models/sonar_arc_projector.hpp"

namespace uw::frontends {

namespace {

using uw::sensor_models::FindCamera;
using uw::sensor_models::FindEdgePose;

const uw::domain::SonarBeamModel* FindBeamModel(const uw::domain::RigCalibrationSnapshot& rig,
                                                 const std::string& sensor_id) {
  for (const auto& model : rig.sonar_beam_models()) {
    if (model.sensor_id().value() == sensor_id) return &model;
  }
  return nullptr;
}

}  // namespace

AcousticOpticAssociator::AcousticOpticAssociator(AcousticOpticAssociatorParams params)
    : params_(std::move(params)) {}

AssociationAuditResult AcousticOpticAssociator::Associate(
    const uw::domain::HypothesisSet& sonar_hypotheses,
    const uw::domain::MeasurementEvidence& optical_evidence,
    const uw::domain::RigCalibrationSnapshot& rig, double time_delta_seconds) {
  AssociationAuditResult result;
  result.health.set_component_id("acoustic_optic_associator");

  if (sonar_hypotheses.candidates_size() == 0) return result;
  ++frames_processed_;

  const auto& top_evidence = sonar_hypotheses.candidates(0);
  if (!uw::domain::HasPayload<uw::domain::SonarRangeBearing>(top_evidence)) return result;
  const auto& top_sonar = uw::domain::GetPayload<uw::domain::SonarRangeBearing>(top_evidence);

  uw::domain::AcousticOpticAssociationRecord record;
  *record.mutable_sonar_evidence_id() = top_evidence.evidence_id();
  record.set_time_delta_seconds(time_delta_seconds);

  // First-checked, audit-first gate (see AcousticOpticAssociatorParams'
  // doc comment): reject on a bad time delta before any geometric
  // projection, not after -- a stale sonar/camera pairing has no business
  // being scored as if it were spatially consistent, regardless of how
  // well the numbers happen to line up.
  if (time_delta_seconds > params_.max_time_delta_s) {
    record.set_status(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_REJECTED);
    record.set_reason(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_TIME_DELTA);
    result.records.push_back(record);
    return result;
  }

  if (!uw::domain::HasPayload<uw::domain::OpticalDepthPriorMeasurement>(optical_evidence)) {
    record.set_status(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_REJECTED);
    record.set_reason(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_NO_CANDIDATE);
    result.records.push_back(record);
    return result;
  }
  const auto& prior = uw::domain::GetPayload<uw::domain::OpticalDepthPriorMeasurement>(optical_evidence);
  if (prior.scale_status() != uw::domain::OPTICAL_DEPTH_SCALE_STATUS_METRIC) {
    record.set_status(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_REJECTED);
    record.set_reason(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_SCALE);
    result.records.push_back(record);
    return result;
  }

  const auto* camera_intrinsics = FindCamera(rig, params_.camera_sensor_id);
  const auto* beam_model = FindBeamModel(rig, params_.sonar_sensor_id);
  const auto camera_pose = FindEdgePose(rig, params_.camera_frame);
  const auto sonar_pose = FindEdgePose(rig, params_.sonar_frame);
  if (camera_intrinsics == nullptr || beam_model == nullptr || !camera_pose.has_value() ||
      !sonar_pose.has_value()) {
    record.set_status(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_REJECTED);
    record.set_reason(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_CALIBRATION);
    result.records.push_back(record);
    return result;
  }

  const auto camera = uw::sensor_models::PinholeCamera::FromIntrinsics(*camera_intrinsics);
  const uw::sensor_models::Pose3 camera_T_sonar = camera_pose->Inverse() * (*sonar_pose);

  const auto arc_candidates = uw::sensor_models::ProjectSonarArcToCamera(
      top_sonar.range_m(), top_sonar.bearing_rad(), beam_model->elevation_aperture_rad(),
      camera_T_sonar, camera, params_.arc_samples);

  struct Scored {
    std::size_t pixel_index = 0;
    double score = 0.0;
    float depth_m = 0.0f;
    float variance_m2 = 0.0f;
  };
  std::vector<Scored> passed;
  const double range_sigma = top_sonar.range_sigma_m() > 0.0 ? top_sonar.range_sigma_m() : 1.0;
  const double bearing_sigma = top_sonar.bearing_sigma_rad() > 0.0 ? top_sonar.bearing_sigma_rad() : 1.0;

  for (const auto& candidate : arc_candidates) {
    const int u = static_cast<int>(std::lround(candidate.pixel_u));
    const int v = static_cast<int>(std::lround(candidate.pixel_v));
    if (u < 0 || u >= static_cast<int>(prior.width()) || v < 0 || v >= static_cast<int>(prior.height())) {
      continue;
    }
    const std::size_t idx = static_cast<std::size_t>(v) * prior.width() + static_cast<std::size_t>(u);
    if (idx >= prior.valid_mask().size() || prior.valid_mask()[idx] == 0) continue;

    const double depth_m = prior.depth_m(static_cast<int>(idx));
    const auto observed = uw::sensor_models::UnprojectPixelToSonarRangeBearing(
        candidate.pixel_u, candidate.pixel_v, depth_m, camera_T_sonar, camera);
    const double range_residual = observed.range_m - top_sonar.range_m();
    const double bearing_residual = observed.bearing_rad - top_sonar.bearing_rad();
    if (std::abs(range_residual) > params_.range_gate_m) continue;
    if (std::abs(bearing_residual) > params_.bearing_gate_rad) continue;

    const double score = (range_residual * range_residual) / (range_sigma * range_sigma) +
                         (bearing_residual * bearing_residual) / (bearing_sigma * bearing_sigma);
    passed.push_back(Scored{idx, score, static_cast<float>(depth_m), prior.variance_m2(static_cast<int>(idx))});
  }

  // Multiple arc samples can land on the same rounded pixel (e.g. a zero
  // or narrow aperture) — collapse to one entry per unique pixel (keeping
  // the best score) before ambiguity-margin scoring, so duplicate samples
  // of the SAME candidate never masquerade as a second, competing one.
  std::sort(passed.begin(), passed.end(),
            [](const Scored& a, const Scored& b) { return a.pixel_index < b.pixel_index; });
  std::vector<Scored> deduped;
  for (const auto& entry : passed) {
    if (!deduped.empty() && deduped.back().pixel_index == entry.pixel_index) {
      if (entry.score < deduped.back().score) deduped.back() = entry;
      continue;
    }
    deduped.push_back(entry);
  }
  passed = std::move(deduped);

  if (passed.empty()) {
    record.set_status(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_REJECTED);
    record.set_reason(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_NO_CANDIDATE);
    result.records.push_back(record);
    return result;
  }
  std::sort(passed.begin(), passed.end(), [](const Scored& a, const Scored& b) { return a.score < b.score; });
  for (std::size_t i = 0; i < passed.size() && static_cast<int>(i) < params_.max_candidates; ++i) {
    record.add_candidate_pixel_indices(static_cast<uint32_t>(passed[i].pixel_index));
  }
  record.set_best_score(passed[0].score);

  if (passed.size() > 1) {
    record.set_second_best_score(passed[1].score);
    if (passed[1].score - passed[0].score < params_.ambiguity_margin) {
      // Near boresight, elevation only weakly perturbs (range, bearing) —
      // bearing is exactly independent of elevation/depth, and range only
      // picks up a second-order sec(phi) correction — so on a locally flat
      // target, several arc-sample pixels legitimately tie on geometric
      // score even though there is no real competing hypothesis (see
      // docs/archive/uw-slam-production-readiness-and-roadmap-2026-08-21.md 2.3 for
      // the investigation that found this: clean_textured/elevation_stress
      // scenarios were rejecting 100% of associations as AMBIGUOUS despite
      // being the "should just work" cases). What actually matters for
      // downstream fusion is whether the CHOICE affects the resulting depth
      // estimate — if the tied candidates' depth_m values also agree, they
      // are redundant estimates of the same point, not competing
      // hypotheses, so fall through and accept the best-scoring one instead
      // of rejecting.
      const double depth_diff = static_cast<double>(passed[1].depth_m) - static_cast<double>(passed[0].depth_m);
      const double combined_variance =
          static_cast<double>(passed[0].variance_m2) + static_cast<double>(passed[1].variance_m2);
      const double agreement_threshold_sq =
          combined_variance * params_.depth_agreement_sigma * params_.depth_agreement_sigma;
      const bool depths_agree =
          combined_variance > 0.0 ? (depth_diff * depth_diff) <= agreement_threshold_sq : depth_diff == 0.0;
      if (!depths_agree) {
        record.set_status(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_AMBIGUOUS);
        record.set_reason(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_AMBIGUOUS_MARGIN);
        result.records.push_back(record);
        return result;
      }
    }
  }

  record.set_status(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_ACCEPTED);
  record.set_reason(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_NONE);
  record.set_has_selected_pixel(true);
  record.set_selected_pixel_index(static_cast<uint32_t>(passed[0].pixel_index));
  record.set_prior_depth_m(passed[0].depth_m);
  record.set_prior_variance_m2(passed[0].variance_m2);
  ++frames_accepted_;
  result.records.push_back(record);
  return result;
}

}  // namespace uw::frontends
