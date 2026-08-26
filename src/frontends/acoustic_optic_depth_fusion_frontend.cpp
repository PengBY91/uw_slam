#include "frontends/acoustic_optic_depth_fusion_frontend.hpp"

#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include "sensor_models/camera_model.hpp"
#include "sensor_models/geometry.hpp"

namespace uw::frontends {

namespace {

using uw::sensor_models::FindCamera;
using uw::sensor_models::FindEdgePose;

}  // namespace

AcousticOpticDepthFusionFrontend::AcousticOpticDepthFusionFrontend(AcousticOpticDepthFusionParams params)
    : params_(params), associator_(params.associator) {}

std::optional<FusedDepthResult> AcousticOpticDepthFusionFrontend::Fuse(
    const uw::domain::HypothesisSet& sonar_hypotheses,
    const uw::domain::MeasurementEvidence& optical_evidence,
    const uw::domain::RigCalibrationSnapshot& rig, double time_delta_seconds) {
  if (!uw::domain::HasPayload<uw::domain::OpticalDepthPriorMeasurement>(optical_evidence)) {
    return std::nullopt;
  }
  const auto& prior = uw::domain::GetPayload<uw::domain::OpticalDepthPriorMeasurement>(optical_evidence);

  uw::domain::FusedDepthMeasurement fused;
  *fused.mutable_reference_camera_frame() = prior.reference_camera_frame();
  fused.set_width(prior.width());
  fused.set_height(prior.height());
  const std::size_t pixels = static_cast<std::size_t>(prior.width()) * prior.height();
  std::string valid_mask(pixels, '\0');
  std::string contribution_mask(pixels, static_cast<char>(uw::domain::DEPTH_CONTRIBUTION_INVALID));
  for (std::size_t i = 0; i < pixels; ++i) {
    fused.add_depth_m(prior.depth_m(static_cast<int>(i)));
    fused.add_variance_m2(prior.variance_m2(static_cast<int>(i)));
    if (i < prior.valid_mask().size() && prior.valid_mask()[i] != 0) {
      valid_mask[i] = 1;
      contribution_mask[i] = static_cast<char>(uw::domain::DEPTH_CONTRIBUTION_OPTICAL_ONLY);
    }
  }

  auto audit = associator_.Associate(sonar_hypotheses, optical_evidence, rig, time_delta_seconds);

  if (!audit.records.empty()) {
    auto& record = audit.records[0];
    if (record.status() == uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_ACCEPTED &&
        record.has_selected_pixel()) {
      const auto* camera_intrinsics = FindCamera(rig, params_.associator.camera_sensor_id);
      const auto camera_pose = FindEdgePose(rig, params_.associator.camera_frame);
      const auto sonar_pose = FindEdgePose(rig, params_.associator.sonar_frame);

      if (camera_intrinsics != nullptr && camera_pose.has_value() && sonar_pose.has_value()) {
        const auto& top_sonar =
            uw::domain::GetPayload<uw::domain::SonarRangeBearing>(sonar_hypotheses.candidates(0));
        const auto camera = uw::sensor_models::PinholeCamera::FromIntrinsics(*camera_intrinsics);
        const uw::sensor_models::Pose3 camera_T_sonar = camera_pose->Inverse() * (*sonar_pose);
        const std::size_t idx = record.selected_pixel_index();
        const std::size_t v = idx / prior.width();
        const std::size_t u = idx % prior.width();

        const auto posterior = OptimizePosteriorDepth(
            static_cast<double>(u), static_cast<double>(v), record.prior_depth_m(),
            record.prior_variance_m2(), top_sonar.range_m(), top_sonar.range_sigma_m(),
            top_sonar.bearing_rad(), top_sonar.bearing_sigma_rad(), camera_T_sonar, camera,
            params_.optimizer);

        if (!posterior.valid) {
          record.set_status(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_REJECTED);
          record.set_reason(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_POSTERIOR_INVALID);
        } else if (posterior.variance_m2 >
                   record.prior_variance_m2() * (1.0 - params_.min_variance_improvement_fraction)) {
          record.set_status(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_REJECTED);
          record.set_reason(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_VARIANCE_NOT_IMPROVED);
        } else if (std::abs(posterior.range_residual_m) >
                       params_.innovation_gate_sigma * top_sonar.range_sigma_m() ||
                   std::abs(posterior.bearing_residual_rad) >
                       params_.innovation_gate_sigma * top_sonar.bearing_sigma_rad()) {
          record.set_status(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_CONFLICT);
          record.set_reason(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_CROSS_MODAL_CONFLICT);
        } else {
          record.set_posterior_depth_m(posterior.depth_m);
          record.set_posterior_variance_m2(posterior.variance_m2);
          fused.set_depth_m(static_cast<int>(idx), static_cast<float>(posterior.depth_m));
          fused.set_variance_m2(static_cast<int>(idx), static_cast<float>(posterior.variance_m2));
          contribution_mask[idx] = static_cast<char>(uw::domain::DEPTH_CONTRIBUTION_ACOUSTIC_OPTIC);
        }
      } else {
        record.set_status(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_REJECTED);
        record.set_reason(uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_CALIBRATION);
      }
    }
    *fused.add_associations() = record;
  }

  fused.set_valid_mask(valid_mask);
  fused.set_contribution_mask(contribution_mask);

  uw::domain::EvidenceId evidence_id;
  evidence_id.set_value("fused_depth_" + std::to_string(next_evidence_id_++));
  std::vector<uw::domain::ObservationId> sources(optical_evidence.source_observations().begin(),
                                                  optical_evidence.source_observations().end());

  FusedDepthResult result;
  result.fused_evidence = uw::domain::MakeEvidence(evidence_id, sources, fused, /*noise_scale=*/1.0,
                                                    "acoustic_optic_depth_fusion_v1");
  result.health.set_component_id("acoustic_optic_depth_fusion_frontend");
  return result;
}

}  // namespace uw::frontends
