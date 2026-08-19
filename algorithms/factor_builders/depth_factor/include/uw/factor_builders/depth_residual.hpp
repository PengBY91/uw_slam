#pragma once

#include "uw/measurement_api/residual_block.hpp"

namespace uw::factor_builders {

// New (not ported) — a 1-D depth/pressure prior, per platform architecture
// section 7.4/9.4.5 ("depth factor: 一维 z prior"). Convention: world frame
// is Z-up, so a measured depth (positive = below surface) corresponds to a
// negative pose Z. residual = measured_depth_m + translation.z().
//
// Parameter block[0]: 7-vector pose T_WB = [tx,ty,tz,qx,qy,qz,qw].
class DepthResidual : public uw::measurement_api::ResidualBlock {
 public:
  DepthResidual(double measured_depth_m, double sqrt_information);

  int ResidualDim() const override { return 1; }
  std::vector<int> ParameterBlockSizes() const override { return {7}; }

  bool Evaluate(const std::vector<const double*>& parameters, double* residuals,
               std::vector<double*>* jacobians) const override;

 private:
  double measured_depth_m_;
  double sqrt_information_;
};

}  // namespace uw::factor_builders
