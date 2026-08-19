#include "uw/factor_builders/depth_residual.hpp"

namespace uw::factor_builders {

DepthResidual::DepthResidual(double measured_depth_m, double sqrt_information)
    : measured_depth_m_(measured_depth_m), sqrt_information_(sqrt_information) {}

bool DepthResidual::Evaluate(const std::vector<const double*>& parameters, double* residuals,
                              std::vector<double*>* jacobians) const {
  const double tz = parameters[0][2];
  residuals[0] = sqrt_information_ * (measured_depth_m_ + tz);

  if (jacobians != nullptr && !jacobians->empty() && (*jacobians)[0] != nullptr) {
    double* jac = (*jacobians)[0];
    jac[0] = 0.0;
    jac[1] = 0.0;
    jac[2] = sqrt_information_;
    jac[3] = 0.0;
    jac[4] = 0.0;
    jac[5] = 0.0;
    jac[6] = 0.0;
  }
  return true;
}

}  // namespace uw::factor_builders
