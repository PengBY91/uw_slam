#pragma once

#include <Eigen/Core>

#include "measurement_api/residual_block.hpp"
#include "sensor_models/geometry.hpp"

namespace uw::factor_builders {

// New (not ported) — black-box VIO mode relative-pose factor between two
// keyframes (platform architecture section 8.1: SVIn/VIO adapter output ->
// RelativePoseEvidence -> this factor). Residual (6D):
//   translation: R_i^T (t_j - t_i) - measured_translation
//   rotation:    2 * vec( measured_q^{-1} * (q_i^{-1} * q_j) )   (small-angle
//                quaternion-error approximation, standard in VIO literature)
//
// Jacobian: computed by INTERNAL central finite differences over both
// parameter blocks rather than a hand-derived closed form. This is a
// deliberate v1 simplification (correct-by-construction, at the cost of 24
// extra residual evaluations per factor) — replacing it with a closed-form
// minimal-SO3 Jacobian is a natural follow-up once a real solver
// (Ceres/GTSAM, see platform architecture section 20) is adopted and
// analytic Jacobians start to matter for performance.
//
// Parameter blocks: [0] = T_WBi, [1] = T_WBj, both 7-vectors
// [tx,ty,tz,qx,qy,qz,qw].
class RelativePoseResidual : public uw::measurement_api::ResidualBlock {
 public:
  RelativePoseResidual(uw::sensor_models::Pose3 measured_relative_pose,
                        double sqrt_information_translation, double sqrt_information_rotation);

  int ResidualDim() const override { return 6; }
  std::vector<int> ParameterBlockSizes() const override { return {7, 7}; }

  bool Evaluate(const std::vector<const double*>& parameters, double* residuals,
               std::vector<double*>* jacobians) const override;

 private:
  Eigen::Matrix<double, 6, 1> ResidualOnly(const uw::sensor_models::Pose3& pose_i,
                                           const uw::sensor_models::Pose3& pose_j) const;

  uw::sensor_models::Pose3 measured_relative_pose_;
  double sqrt_information_translation_;
  double sqrt_information_rotation_;
};

}  // namespace uw::factor_builders
