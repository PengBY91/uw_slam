// 9-dim prior on one keyframe's inertial state (PREP-B-01,
// docs/imu-preintegration-design-2026-09-03.md section 7).
//
// The anchor keyframe's POSE is fixed to remove the gauge freedom, but its
// inertial block must not be: velocity and the two biases are quantities the
// estimator is supposed to refine, and pinning them would push their whole
// error into the trajectory. What holds them instead is this one residual --
// a diagonal Gaussian prior around the values
// frontends/imu_stationary_initializer.hpp measured (or, when it could not,
// around zero under a deliberately wide velocity sigma).
//
// It is also what keeps the anchor's inertial block from being structurally
// unconstrained: an inertial state referenced by no IMU edge and no prior
// makes the normal equations singular, which LM damping would quietly hide
// as a plausible-looking but arbitrary answer (the design note's section 7
// last bullet).
//
// Parameter block, matching PoseGraphProblem::InertialState exactly:
//   [0] inertial 9  [vx,vy,vz, bgx,bgy,bgz, bax,bay,baz]
// world-frame velocity, body-frame biases.
//
// Residual = diag(1/sigma) * (inertial - target). Per-axis sigmas rather
// than one isotropic scale because the three groups differ by orders of
// magnitude (m/s vs rad/s vs m/s^2) and, in the wide-velocity fallback, the
// velocity block is deliberately far looser than the bias blocks.
#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>

#include "measurement_api/residual_block.hpp"

namespace uw::factor_builders {

class InertialPriorResidual : public uw::measurement_api::ResidualBlock {
 public:
  // Returns nullptr when the prior cannot be defined: any non-finite target
  // entry, or any sigma that is not finite and strictly positive. A zero
  // sigma would be an infinitely stiff constraint -- that is what fixing
  // the block is for, and it is a different decision made elsewhere -- and
  // an infinite one is not a prior at all.
  static std::unique_ptr<InertialPriorResidual> Create(const Eigen::Matrix<double, 9, 1>& target,
                                                        const Eigen::Matrix<double, 9, 1>& sigma);

  int ResidualDim() const override { return 9; }
  std::vector<int> ParameterBlockSizes() const override { return {9}; }

  bool Evaluate(const std::vector<const double*>& parameters, double* residuals,
                std::vector<double*>* jacobians) const override;

  const Eigen::Matrix<double, 9, 1>& target() const { return target_; }
  // 1 / sigma per axis, i.e. the diagonal of the sqrt information matrix.
  const Eigen::Matrix<double, 9, 1>& sqrt_information_diagonal() const {
    return sqrt_information_diagonal_;
  }

 private:
  InertialPriorResidual(Eigen::Matrix<double, 9, 1> target,
                        Eigen::Matrix<double, 9, 1> sqrt_information_diagonal);

  Eigen::Matrix<double, 9, 1> target_;
  Eigen::Matrix<double, 9, 1> sqrt_information_diagonal_;
};

}  // namespace uw::factor_builders
