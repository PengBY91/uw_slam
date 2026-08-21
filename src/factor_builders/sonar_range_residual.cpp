#include "factor_builders/sonar_range_residual.hpp"

namespace uw::factor_builders {

SonarRangeResidual::SonarRangeResidual(double range_m, double bearing_rad,
                                        double sqrt_information,
                                        std::vector<Eigen::Vector3d> landmark_subset_W)
    : range_m_(range_m),
      bearing_rad_(bearing_rad),
      sqrt_information_(sqrt_information),
      landmark_subset_W_(std::move(landmark_subset_W)) {}

bool SonarRangeResidual::Evaluate(const std::vector<const double*>& parameters,
                                   double* residuals, std::vector<double*>* jacobians) const {
  if (landmark_subset_W_.empty()) return false;

  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto& point : landmark_subset_W_) mean += point;
  mean /= static_cast<double>(landmark_subset_W_.size());

  const auto T_WS = uw::sensor_models::Pose3::FromParameterBlock(parameters[0]);

  const Eigen::Vector3d delta = T_WS.translation - mean;
  const double range_corrected = delta.norm();
  if (range_corrected < 1e-9) return false;  // degenerate: pose coincides with landmark mean

  residuals[0] = sqrt_information_ * (range_m_ - range_corrected);

  if (jacobians != nullptr && !jacobians->empty() && (*jacobians)[0] != nullptr) {
    // d(range_corrected)/d(translation) = delta / ||delta||;
    // d(residual)/d(translation) = -d(range_corrected)/d(translation).
    const Eigen::Vector3d d_residual_d_translation = -sqrt_information_ * delta / range_corrected;
    double* jac = (*jacobians)[0];
    jac[0] = d_residual_d_translation.x();
    jac[1] = d_residual_d_translation.y();
    jac[2] = d_residual_d_translation.z();
    // Orientation columns: this residual does not depend on T_WS's
    // rotation (only translation enters the distance-to-landmark
    // computation), so these are exactly zero, not merely approximated.
    jac[3] = 0.0;
    jac[4] = 0.0;
    jac[5] = 0.0;
    jac[6] = 0.0;
  }
  return true;
}

}  // namespace uw::factor_builders
