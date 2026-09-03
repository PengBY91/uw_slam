// 15-dim on-manifold IMU preintegration residual (PREP-B-01 step 2,
// docs/imu-preintegration-design-2026-09-03.md section 4). New code, not
// ported: the residual and every Jacobian block below are derived from the
// delta definitions in include/sensor_models/imu_preintegration.hpp and
// cross-checked against central finite differences in
// tests/factor_builders/imu_preintegration_residual_test.cpp — the same
// rule sonar_range_residual.hpp follows (CLAUDE.md: derive, then verify;
// never copy an upstream Jacobian).
//
// Parameter blocks, in ParameterBlockSizes() order:
//   [0] pose_i     7  [tx,ty,tz,qx,qy,qz,qw]  (uw::sensor_models::Pose3)
//   [1] inertial_i 9  [vx,vy,vz, bgx,bgy,bgz, bax,bay,baz]
//   [2] pose_j     7
//   [3] inertial_j 9
// Velocity is world-frame, biases are body-frame; this is exactly
// PoseGraphProblem::InertialState's storage layout (option A of the design
// note section 6 — a separate 9-dim block, NOT a widened keyframe).
//
// Raw residual, ordered [r_R(3), r_v(3), r_p(3), r_bg(3), r_ba(3)] to match
// the covariance ordering on the wire (schemas/proto/uw/domain/
// measurement.proto ImuPreintegrationMeasurement), with the deltas
// first-order re-linearised for the current bias estimate
// (delta.Corrected*, paper eq. (44)) and g_W = (0, 0, -gravity_mps2)
// because world/body are Z-up in this repo:
//   r_R  = Log( dR~^T R_i^T R_j )
//   r_v  = R_i^T (v_j - v_i - g dt) - dv~
//   r_p  = R_i^T (p_j - p_i - v_i dt - 0.5 g dt^2) - dp~
//   r_bg = bg_j - bg_i
//   r_ba = ba_j - ba_i
// Whitened residual = sqrt_information * raw_residual, where
// sqrt_information is the 15x15 matrix built by
// ImuPreintegrationFactorBuilder from the evidence covariance.
//
// Jacobian convention. The residual is derived analytically against the
// MINIMAL perturbations (world-frame dp, right-multiplied dphi with
// R <- R Exp(dphi), additive dv/dbg/dba) and then chained onto the raw
// 4-parameter quaternion columns the solvers actually step. That chain is
// exact rather than approximate: the residual only ever sees
// Pose3::FromParameterBlock's NORMALISED quaternion, so it is invariant
// along q itself, and for the orthogonal directions the map is
//   d r / d q_raw = (d r / d dphi) * 2 Q(q_hat)^T / |q_raw|,
//   Q = [ w I + [v]_x ; -v^T ]  (4x3, Q^T Q = I, Q^T q = 0).
// Under Ceres's EigenQuaternionManifold (the adapter's PoseManifold) the
// ambient Jacobian is post-multiplied by dq/ddphi = 0.5 Q, which recovers
// the minimal Jacobian exactly; under the hand-rolled Gauss-Newton solver
// the along-q column is zero, which is correct (that direction genuinely
// does not change the residual) and harmless because LM damping keeps the
// normal equations positive-definite and the step is renormalised.
#pragma once

#include <vector>

#include <Eigen/Core>

#include "measurement_api/residual_block.hpp"
#include "sensor_models/imu_preintegration.hpp"

namespace uw::factor_builders {

class ImuPreintegrationResidual : public uw::measurement_api::ResidualBlock {
 public:
  ImuPreintegrationResidual(uw::sensor_models::PreintegratedImuDelta delta,
                             Eigen::Matrix<double, 15, 15> sqrt_information);

  int ResidualDim() const override { return 15; }
  std::vector<int> ParameterBlockSizes() const override { return {7, 9, 7, 9}; }

  bool Evaluate(const std::vector<const double*>& parameters, double* residuals,
                std::vector<double*>* jacobians) const override;

  const uw::sensor_models::PreintegratedImuDelta& delta() const { return delta_; }
  const Eigen::Matrix<double, 15, 15>& sqrt_information() const { return sqrt_information_; }

 private:
  uw::sensor_models::PreintegratedImuDelta delta_;
  Eigen::Matrix<double, 15, 15> sqrt_information_;
};

}  // namespace uw::factor_builders
