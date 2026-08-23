#include "factor_builders/relative_pose_residual.hpp"

#include <array>

namespace uw::factor_builders {

using uw::sensor_models::Pose3;

RelativePoseResidual::RelativePoseResidual(Pose3 measured_relative_pose,
                                            Eigen::Matrix<double, 6, 6> sqrt_information)
    : measured_relative_pose_(std::move(measured_relative_pose)),
      sqrt_information_(std::move(sqrt_information)) {}

Eigen::Matrix<double, 6, 1> RelativePoseResidual::ResidualOnly(const Pose3& pose_i,
                                                                const Pose3& pose_j) const {
  const Pose3 predicted = pose_i.Inverse() * pose_j;  // i_T_j
  const Eigen::Vector3d translation_error =
      predicted.translation - measured_relative_pose_.translation;

  const Eigen::Quaterniond rotation_error =
      measured_relative_pose_.rotation.conjugate() * predicted.rotation;
  // Small-angle quaternion-error approximation: 2 * vector part, sign-fixed
  // so it does not flip discontinuously with the quaternion's double cover.
  Eigen::Vector3d rotation_residual = 2.0 * rotation_error.vec();
  if (rotation_error.w() < 0.0) rotation_residual = -rotation_residual;

  Eigen::Matrix<double, 6, 1> raw_residual;
  raw_residual.head<3>() = translation_error;
  raw_residual.tail<3>() = rotation_residual;
  return sqrt_information_ * raw_residual;
}

bool RelativePoseResidual::Evaluate(const std::vector<const double*>& parameters,
                                     double* residuals, std::vector<double*>* jacobians) const {
  const Pose3 pose_i = Pose3::FromParameterBlock(parameters[0]);
  const Pose3 pose_j = Pose3::FromParameterBlock(parameters[1]);

  const auto residual_vec = ResidualOnly(pose_i, pose_j);
  for (int i = 0; i < 6; ++i) residuals[i] = residual_vec(i);

  if (jacobians == nullptr || jacobians->size() < 2) return true;

  constexpr double kEps = 1e-6;
  for (int block = 0; block < 2; ++block) {
    double* jac = (*jacobians)[block];
    if (jac == nullptr) continue;

    std::array<double, 7> perturbed{};
    const double* base = parameters[block];
    for (int p = 0; p < 7; ++p) perturbed[p] = base[p];

    for (int p = 0; p < 7; ++p) {
      const double original = perturbed[p];
      perturbed[p] = original + kEps;
      const Pose3 pose_plus = Pose3::FromParameterBlock(perturbed.data());
      const auto r_plus = (block == 0) ? ResidualOnly(pose_plus, pose_j) : ResidualOnly(pose_i, pose_plus);

      perturbed[p] = original - kEps;
      const Pose3 pose_minus = Pose3::FromParameterBlock(perturbed.data());
      const auto r_minus =
          (block == 0) ? ResidualOnly(pose_minus, pose_j) : ResidualOnly(pose_i, pose_minus);

      perturbed[p] = original;

      const Eigen::Matrix<double, 6, 1> column = (r_plus - r_minus) / (2.0 * kEps);
      // row-major 6x7: jac[row * 7 + col]
      for (int row = 0; row < 6; ++row) jac[row * 7 + p] = column(row);
    }
  }
  return true;
}

}  // namespace uw::factor_builders
