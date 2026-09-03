#include "factor_builders/imu_preintegration_factor_builder.hpp"

#include <cmath>
#include <string>

#include <Eigen/Cholesky>

#include "factor_builders/imu_preintegration_residual.hpp"
#include "sensor_models/imu_preintegration.hpp"

namespace uw::factor_builders {

namespace {

using Matrix15d = Eigen::Matrix<double, 15, 15>;

// The frontend's covariance is a genuine propagated quantity (not a
// configured guess), so unlike relative_pose_factor_builder there is no
// isotropic cap to clamp it against — the only knobs are the frontend's
// own `estimated_noise_scale` suggestion and the caller's
// `proposed_noise`, both of which INFLATE the covariance (weaken the
// factor) and neither of which can sharpen it below what was integrated.
double InflationScale(double estimated_noise_scale, double proposed_noise) {
  double scale = 1.0;
  if (std::isfinite(estimated_noise_scale) && estimated_noise_scale > 0.0) {
    scale *= estimated_noise_scale;
  }
  if (std::isfinite(proposed_noise) && proposed_noise > 0.0) scale *= proposed_noise;
  return scale;
}

}  // namespace

bool ImuPreintegrationFactorBuilder::CanBuild(const uw::domain::FactorCandidate& candidate) const {
  return candidate.residual_model() == kResidualModel;
}

std::unique_ptr<uw::measurement_api::ResidualBlock> ImuPreintegrationFactorBuilder::Build(
    const uw::domain::FactorCandidate& candidate, const uw::domain::MeasurementEvidence& evidence,
    const uw::measurement_api::FactorBuildContext& /*context*/) const {
  if (!CanBuild(candidate)) return nullptr;
  if (!uw::domain::HasPayload<uw::domain::ImuPreintegrationMeasurement>(evidence)) return nullptr;

  const auto& message = uw::domain::GetPayload<uw::domain::ImuPreintegrationMeasurement>(evidence);
  std::string error;
  const auto delta = uw::sensor_models::PreintegratedImuDelta::FromProto(message, &error);
  if (!delta.has_value()) return nullptr;

  const double scale = InflationScale(evidence.estimated_noise_scale(), candidate.proposed_noise());
  Matrix15d covariance = delta->covariance * (scale * scale);
  covariance = 0.5 * (covariance + covariance.transpose());

  // sqrt_information = L^-1 for covariance = L L^T, so
  // sqrt_information^T * sqrt_information == covariance^-1. LLT's own
  // failure flag is the positive-definiteness check: a zero row (e.g. a
  // producer that filled no bias random walk) makes it fail here rather
  // than silently produce an infinitely confident factor.
  const Eigen::LLT<Matrix15d> llt(covariance);
  if (llt.info() != Eigen::Success) return nullptr;
  const Matrix15d sqrt_information =
      llt.matrixL().solve(Matrix15d::Identity());
  if (!sqrt_information.allFinite()) return nullptr;

  return std::make_unique<ImuPreintegrationResidual>(*delta, sqrt_information);
}

}  // namespace uw::factor_builders
