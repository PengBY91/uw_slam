// measurement_api (part of the `core` library): abstract Frontend /
// FactorBuilder / provider interfaces (platform architecture section 6).
// Depends on `domain` and `sensor_models` only — no concrete algorithm, no ROS.
#pragma once

#include <cstddef>
#include <vector>

namespace uw::measurement_api {

// Backend-agnostic residual + analytic Jacobian interface. Every
// factor_builders/* implementation (include/factor_builders, src/factor_builders)
// produces one of these; the estimator's solver (a hand-rolled Gauss-Newton
// in v1, see include/estimation) evaluates it without knowing which factor type it
// is. This indirection is what lets us swap the solver for Ceres/GTSAM
// later (platform architecture section 20 "延后决策") without touching
// factor_builders.
class ResidualBlock {
 public:
  virtual ~ResidualBlock() = default;

  virtual int ResidualDim() const = 0;
  virtual std::vector<int> ParameterBlockSizes() const = 0;

  // parameters[i] points to ParameterBlockSizes()[i] doubles.
  // residuals points to ResidualDim() doubles (caller-owned).
  // jacobians, if non-null, has one entry per parameter block; a non-null
  // entry j points to ResidualDim() * ParameterBlockSizes()[i] doubles,
  // row-major (d residual / d parameter_i). A null entry means "don't
  // compute this block's Jacobian".
  virtual bool Evaluate(const std::vector<const double*>& parameters, double* residuals,
                        std::vector<double*>* jacobians) const = 0;
};

}  // namespace uw::measurement_api
