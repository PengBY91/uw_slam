// IMU preintegration factor builder (PREP-B-01 step 2,
// docs/imu-preintegration-design-2026-09-03.md section 5): turns one
// ImuPreintegrationMeasurement evidence (produced by
// include/frontends/imu_preintegration_frontend.hpp) into the 15-dim
// ImuPreintegrationResidual.
//
// The residual's four parameter blocks are {pose_i, inertial_i, pose_j,
// inertial_j}; the caller (replay/online pipeline) is responsible for
// binding them in that order via
// PoseGraphProblem::AddResidualBlock(block, {PoseRef(from),
// InertialRef(from), PoseRef(to), InertialRef(to)}) and for having called
// AddInertialState() for both keyframes first.
//
// Fail-closed, per architecture invariant #1 (FactorBuilder owns the
// weighting): a malformed payload, a covariance that is not positive
// definite, or a non-finite whitening matrix all return nullptr rather
// than falling back to an arbitrary weight — an IMU edge with a
// made-up information matrix is worse than no edge, because the deltas it
// carries are strong enough to drag the whole window.
#pragma once

#include "measurement_api/factor_builder.hpp"

namespace uw::factor_builders {

class ImuPreintegrationFactorBuilder : public uw::measurement_api::FactorBuilder {
 public:
  static constexpr const char* kResidualModel = "imu_preintegration_v1";

  bool CanBuild(const uw::domain::FactorCandidate& candidate) const override;

  std::unique_ptr<uw::measurement_api::ResidualBlock> Build(
      const uw::domain::FactorCandidate& candidate,
      const uw::domain::MeasurementEvidence& evidence,
      const uw::measurement_api::FactorBuildContext& context) const override;
};

}  // namespace uw::factor_builders
