#include "factor_builders/relative_pose_factor_builder.hpp"

#include "factor_builders/relative_pose_residual.hpp"

namespace uw::factor_builders {

bool RelativePoseFactorBuilder::CanBuild(const uw::domain::FactorCandidate& candidate) const {
  return candidate.residual_model() == kResidualModel;
}

std::unique_ptr<uw::measurement_api::ResidualBlock> RelativePoseFactorBuilder::Build(
    const uw::domain::FactorCandidate& candidate, const uw::domain::MeasurementEvidence& evidence,
    const uw::measurement_api::FactorBuildContext& /*context*/) const {
  if (!CanBuild(candidate)) return nullptr;
  if (!uw::domain::HasPayload<uw::domain::RelativePoseMeasurement>(evidence)) return nullptr;

  const auto& measurement = uw::domain::GetPayload<uw::domain::RelativePoseMeasurement>(evidence);
  const auto measured_pose = uw::sensor_models::Pose3::FromProto(measurement.relative_pose());
  const double sqrt_information = candidate.proposed_noise() > 0.0 ? candidate.proposed_noise() : 1.0;

  return std::make_unique<RelativePoseResidual>(measured_pose, sqrt_information, sqrt_information);
}

}  // namespace uw::factor_builders
