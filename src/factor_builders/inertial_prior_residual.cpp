#include "factor_builders/inertial_prior_residual.hpp"

#include <cmath>

namespace uw::factor_builders {

InertialPriorResidual::InertialPriorResidual(Eigen::Matrix<double, 9, 1> target,
                                             Eigen::Matrix<double, 9, 1> sqrt_information_diagonal)
    : target_(std::move(target)), sqrt_information_diagonal_(std::move(sqrt_information_diagonal)) {}

std::unique_ptr<InertialPriorResidual> InertialPriorResidual::Create(
    const Eigen::Matrix<double, 9, 1>& target, const Eigen::Matrix<double, 9, 1>& sigma) {
  Eigen::Matrix<double, 9, 1> sqrt_information_diagonal;
  for (int i = 0; i < 9; ++i) {
    if (!std::isfinite(target(i))) return nullptr;
    if (!std::isfinite(sigma(i)) || sigma(i) <= 0.0) return nullptr;
    sqrt_information_diagonal(i) = 1.0 / sigma(i);
  }
  return std::unique_ptr<InertialPriorResidual>(
      new InertialPriorResidual(target, sqrt_information_diagonal));
}

bool InertialPriorResidual::Evaluate(const std::vector<const double*>& parameters,
                                     double* residuals, std::vector<double*>* jacobians) const {
  if (parameters.size() < 1 || parameters[0] == nullptr || residuals == nullptr) return false;

  const Eigen::Map<const Eigen::Matrix<double, 9, 1>> inertial(parameters[0]);
  Eigen::Map<Eigen::Matrix<double, 9, 1>> residual(residuals);
  residual = sqrt_information_diagonal_.cwiseProduct(inertial - target_);

  if (jacobians != nullptr && !jacobians->empty() && (*jacobians)[0] != nullptr) {
    Eigen::Map<Eigen::Matrix<double, 9, 9, Eigen::RowMajor>> jacobian((*jacobians)[0]);
    jacobian.setZero();
    jacobian.diagonal() = sqrt_information_diagonal_;
  }
  return true;
}

}  // namespace uw::factor_builders
