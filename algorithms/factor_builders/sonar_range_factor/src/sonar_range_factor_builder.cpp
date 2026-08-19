#include "uw/factor_builders/sonar_range_factor_builder.hpp"

#include <cmath>

#include "uw/factor_builders/sonar_range_residual.hpp"

namespace uw::factor_builders {

bool SonarRangeFactorBuilder::CanBuild(const uw::domain::FactorCandidate& candidate) const {
  return candidate.residual_model() == kResidualModel;
}

std::unique_ptr<uw::measurement_api::ResidualBlock> SonarRangeFactorBuilder::Build(
    const uw::domain::FactorCandidate& candidate, const uw::domain::MeasurementEvidence& evidence,
    const uw::measurement_api::FactorBuildContext& context) const {
  if (!CanBuild(candidate)) return nullptr;
  if (!uw::domain::HasPayload<uw::domain::SonarRangeBearing>(evidence)) return nullptr;
  if (context.nearby_points_W.empty()) return nullptr;

  const auto& measurement = uw::domain::GetPayload<uw::domain::SonarRangeBearing>(evidence);

  // proposed_noise is expected to already be the final, capped sqrt
  // information (architecture section 8.4: the estimator/reliability layer
  // owns the cap, this builder does not second-guess it).
  const double sqrt_information = candidate.proposed_noise() > 0.0 ? candidate.proposed_noise() : 1.0;

  return std::make_unique<SonarRangeResidual>(measurement.range_m(), measurement.bearing_rad(),
                                               sqrt_information, context.nearby_points_W);
}

}  // namespace uw::factor_builders
