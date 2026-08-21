// Ported from SVIn (AutonomousFieldRoboticsLab/SVIn), repository license
// GPLv3; upstream file-level copyright OKVIS ASL/ETH Zurich (BSD-3-Clause)
// + Sharmin Rahman for SVIn's SonarError. See /NOTICE for full provenance.
//
// okvis_ros/okvis/okvis_ceres/{include/okvis/ceres/SonarError.hpp,
// src/SonarError.cpp}: given a pose T_WS and a subset of nearby landmarks,
// the sonar range residual is `measured_range - ||T_WS.t - mean(landmarks)||`
// — i.e. "sonar constrains distance to a landmark cluster", never a fixed
// elevation. That residual formulation is what's ported here.
//
// The Jacobian in this file is NOT copied from upstream: upstream computes
// it via an approximate direction (through the sonar beam's range/bearing
// endpoint rather than the landmark mean, and without the sign flip implied
// by residual = measured - computed) which does not equal the true
// derivative of the residual it implements. This file instead derives the
// Jacobian directly and exactly from the residual formula above, so it
// passes a finite-difference cross-check
// (tests/factor_builders/sonar_range_residual_test.cpp) — see NOTICE for the
// full comparison.
#pragma once

#include <vector>

#include <Eigen/Core>

#include "measurement_api/residual_block.hpp"
#include "sensor_models/geometry.hpp"

namespace uw::factor_builders {

// Parameter block[0]: 7-vector pose T_WS = [tx,ty,tz,qx,qy,qz,qw]
// (uw::sensor_models::Pose3::ToParameterBlock layout).
class SonarRangeResidual : public uw::measurement_api::ResidualBlock {
 public:
  // range_m/bearing_rad: the sonar return being constrained (kept only for
  // provenance/logging — the residual itself only uses `landmark_subset_W`
  // and `range_m`, matching upstream, which never uses bearing in the
  // residual/Jacobian either, only in an auxiliary point it no longer needs
  // once the Jacobian is re-derived).
  // sqrt_information: architecture section 8.4 — this must already be the
  // capped, calibrated value; this class does not apply any cap itself.
  // landmark_subset_W: fixed (non-optimized) nearby 3D points in world
  // frame, per FactorBuildContext::nearby_points_W.
  SonarRangeResidual(double range_m, double bearing_rad, double sqrt_information,
                      std::vector<Eigen::Vector3d> landmark_subset_W);

  int ResidualDim() const override { return 1; }
  std::vector<int> ParameterBlockSizes() const override { return {7}; }

  bool Evaluate(const std::vector<const double*>& parameters, double* residuals,
               std::vector<double*>* jacobians) const override;

 private:
  double range_m_;
  double bearing_rad_;
  double sqrt_information_;
  std::vector<Eigen::Vector3d> landmark_subset_W_;
};

}  // namespace uw::factor_builders
