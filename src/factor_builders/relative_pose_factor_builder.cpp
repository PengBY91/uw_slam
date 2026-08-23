#include "factor_builders/relative_pose_factor_builder.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

#include "factor_builders/relative_pose_residual.hpp"

namespace uw::factor_builders {

namespace {

Eigen::Matrix<double, 6, 6> DiagonalCap(double translation_cap, double rotation_cap) {
  Eigen::Matrix<double, 6, 6> d = Eigen::Matrix<double, 6, 6>::Zero();
  d.diagonal() << translation_cap, translation_cap, translation_cap, rotation_cap, rotation_cap,
      rotation_cap;
  return d;
}

}  // namespace

RelativePoseFactorBuilder::RelativePoseFactorBuilder(double translation_cap, double rotation_cap)
    : translation_cap_(translation_cap), rotation_cap_(rotation_cap) {}

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

  // candidate.proposed_noise() is a single wire scalar (FactorCandidate has
  // no typed translation/rotation split) representing the caller's
  // COMBINED compatibility upper bound on both caps -- see
  // src/application/replay_pipeline.cpp, which sets it to
  // max(config_translation_cap, config_rotation_cap) so a non-positive
  // value never tightens either cap below what this builder was
  // constructed with.
  const double candidate_cap = candidate.proposed_noise();
  const double effective_translation_cap =
      candidate_cap > 0.0 ? std::min(translation_cap_, candidate_cap) : translation_cap_;
  const double effective_rotation_cap =
      candidate_cap > 0.0 ? std::min(rotation_cap_, candidate_cap) : rotation_cap_;

  const Eigen::Matrix<double, 6, 6> d = DiagonalCap(effective_translation_cap, effective_rotation_cap);
  Eigen::Matrix<double, 6, 6> sqrt_information = d;  // fallback: isotropic caps, no covariance direction

  if (measurement.covariance_6x6_row_major_size() == 36) {
    Eigen::Matrix<double, 6, 6> covariance;
    bool finite = true;
    for (int row = 0; row < 6; ++row) {
      for (int col = 0; col < 6; ++col) {
        const double v = measurement.covariance_6x6_row_major(row * 6 + col);
        if (!std::isfinite(v)) finite = false;
        covariance(row, col) = v;
      }
    }
    if (finite) {
      covariance = 0.5 * (covariance + covariance.transpose());
      const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigensolver(covariance);
      if (eigensolver.info() == Eigen::Success && eigensolver.eigenvalues().minCoeff() > 0.0) {
        const Eigen::Matrix<double, 6, 6> w_raw = eigensolver.eigenvectors() *
                                                  eigensolver.eigenvalues().cwiseInverse().cwiseSqrt().asDiagonal() *
                                                  eigensolver.eigenvectors().transpose();
        Eigen::Matrix<double, 6, 6> d_inv = Eigen::Matrix<double, 6, 6>::Zero();
        d_inv.diagonal() << 1.0 / effective_translation_cap, 1.0 / effective_translation_cap,
            1.0 / effective_translation_cap, 1.0 / effective_rotation_cap, 1.0 / effective_rotation_cap,
            1.0 / effective_rotation_cap;

        // Cap the MAXIMUM gain W_raw*D^-1 can apply (per singular value)
        // to 1, so the covariance-derived weighting can only ever be as
        // aggressive as the isotropic caps, never more -- while still
        // preserving the covariance's actual DIRECTIONALITY (unlike
        // naively clamping to a diagonal, which would discard it).
        const Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svd(w_raw * d_inv,
                                                                Eigen::ComputeFullU | Eigen::ComputeFullV);
        const Eigen::Matrix<double, 6, 6> clamped = svd.singularValues().cwiseMin(1.0).asDiagonal();
        sqrt_information = svd.matrixU() * clamped * svd.matrixV().transpose() * d;
      }
      // else: not positive definite (or SVD failed) -- keep the diagonal-cap fallback already set.
    }
  }

  return std::make_unique<RelativePoseResidual>(measured_pose, sqrt_information);
}

}  // namespace uw::factor_builders
