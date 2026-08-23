#include "factor_builders/sonar_range_factor_builder.hpp"

#include <algorithm>
#include <cmath>

#include "factor_builders/sonar_range_residual.hpp"

namespace uw::factor_builders {

namespace {

// candidate.proposed_noise() is the caller-configured UPPER BOUND/fallback
// (architecture section 8.4), not the final weight -- the sonar's own
// reported range_sigma_m wins whenever it's usable, capped by this bound.
// bearing_sigma_rad deliberately does not enter here: this is a range-only
// residual (see sonar_range_residual.hpp) and does not pretend to be a 2D
// bearing factor.
double CappedSqrtInformation(double sigma, double configured_cap) {
  const double cap = std::isfinite(configured_cap) && configured_cap > 0.0 ? configured_cap : 1.0;
  if (!std::isfinite(sigma) || sigma <= 0.0) return cap;
  return std::min(cap, 1.0 / sigma);
}

}  // namespace

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
  const double sqrt_information =
      CappedSqrtInformation(measurement.range_sigma_m(), candidate.proposed_noise());

  return std::make_unique<SonarRangeResidual>(measurement.range_m(), measurement.bearing_rad(),
                                               sqrt_information, context.nearby_points_W);
}

}  // namespace uw::factor_builders
