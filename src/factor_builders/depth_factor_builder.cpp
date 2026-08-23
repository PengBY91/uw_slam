#include "factor_builders/depth_factor_builder.hpp"

#include <algorithm>
#include <cmath>

#include "factor_builders/depth_residual.hpp"

namespace uw::factor_builders {

namespace {

// candidate.proposed_noise() is the caller-configured UPPER BOUND/fallback
// (architecture section 8.4), not the final weight -- the actual sensor's
// reported sigma_m wins whenever it's usable, capped by this bound so a
// suspiciously-confident sigma can never exceed what the caller allows.
double CappedSqrtInformation(double sigma, double configured_cap) {
  const double cap = std::isfinite(configured_cap) && configured_cap > 0.0 ? configured_cap : 1.0;
  if (!std::isfinite(sigma) || sigma <= 0.0) return cap;
  return std::min(cap, 1.0 / sigma);
}

}  // namespace

bool DepthFactorBuilder::CanBuild(const uw::domain::FactorCandidate& candidate) const {
  return candidate.residual_model() == kResidualModel;
}

std::unique_ptr<uw::measurement_api::ResidualBlock> DepthFactorBuilder::Build(
    const uw::domain::FactorCandidate& candidate, const uw::domain::MeasurementEvidence& evidence,
    const uw::measurement_api::FactorBuildContext& /*context*/) const {
  if (!CanBuild(candidate)) return nullptr;
  if (!uw::domain::HasPayload<uw::domain::PressureDepthMeasurement>(evidence)) return nullptr;

  const auto& measurement = uw::domain::GetPayload<uw::domain::PressureDepthMeasurement>(evidence);
  const double sqrt_information = CappedSqrtInformation(measurement.sigma_m(), candidate.proposed_noise());
  return std::make_unique<DepthResidual>(measurement.depth_m(), sqrt_information);
}

}  // namespace uw::factor_builders
