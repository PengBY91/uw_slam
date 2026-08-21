#include "factor_builders/depth_factor_builder.hpp"

#include "factor_builders/depth_residual.hpp"

namespace uw::factor_builders {

bool DepthFactorBuilder::CanBuild(const uw::domain::FactorCandidate& candidate) const {
  return candidate.residual_model() == kResidualModel;
}

std::unique_ptr<uw::measurement_api::ResidualBlock> DepthFactorBuilder::Build(
    const uw::domain::FactorCandidate& candidate, const uw::domain::MeasurementEvidence& evidence,
    const uw::measurement_api::FactorBuildContext& /*context*/) const {
  if (!CanBuild(candidate)) return nullptr;
  if (!uw::domain::HasPayload<uw::domain::PressureDepthMeasurement>(evidence)) return nullptr;

  const auto& measurement = uw::domain::GetPayload<uw::domain::PressureDepthMeasurement>(evidence);
  const double sqrt_information = candidate.proposed_noise() > 0.0 ? candidate.proposed_noise() : 1.0;
  return std::make_unique<DepthResidual>(measurement.depth_m(), sqrt_information);
}

}  // namespace uw::factor_builders
