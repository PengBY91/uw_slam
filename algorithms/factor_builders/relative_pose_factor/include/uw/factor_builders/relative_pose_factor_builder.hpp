#pragma once

#include "uw/measurement_api/factor_builder.hpp"

namespace uw::factor_builders {

// v1 uses a single isotropic sqrt-information scalar (candidate.proposed_noise)
// for both translation and rotation blocks. This is a deliberate
// simplification tied to a real finding from the SVIn code audit
// (acoustic-optic-slam-platform-architecture section 22.4): SVIn's
// nav_msgs/Odometry output does not populate usable pose covariance, so a
// LocalOdometryProvider wrapper has to estimate/calibrate its own noise
// scale anyway rather than trusting an upstream covariance field — using it
// as one scalar here (rather than pretending we have anisotropic
// covariance we don't) is the honest v1 choice.
class RelativePoseFactorBuilder : public uw::measurement_api::FactorBuilder {
 public:
  static constexpr const char* kResidualModel = "relative_pose_v1";

  bool CanBuild(const uw::domain::FactorCandidate& candidate) const override;

  std::unique_ptr<uw::measurement_api::ResidualBlock> Build(
      const uw::domain::FactorCandidate& candidate,
      const uw::domain::MeasurementEvidence& evidence,
      const uw::measurement_api::FactorBuildContext& context) const override;
};

}  // namespace uw::factor_builders
