#pragma once

#include "measurement_api/factor_builder.hpp"

namespace uw::factor_builders {

// Whitens the relative-pose residual from the evidence's ACTUAL 6x6
// covariance (frontends::RigidTransformFitResult::covariance, propagated
// through StereoLandmarkVoFrontend -- see rigid_transform_fit.hpp) when
// one is present and well-formed, rather than a fixed isotropic scalar for
// every factor regardless of how well-conditioned the underlying fit was.
// `translation_cap`/`rotation_cap` (independent, per platform architecture
// section 8.4's "final_information = min(...)") bound the MAXIMUM gain the
// covariance-derived weighting can apply on each subspace; a missing,
// non-square, non-finite, asymmetric-beyond-tolerance, or non-positive-
// -definite covariance falls back to the isotropic caps themselves (never
// crashes, never silently trusts garbage). See Build() for the exact
// construction.
class RelativePoseFactorBuilder : public uw::measurement_api::FactorBuilder {
 public:
  static constexpr const char* kResidualModel = "relative_pose_v1";

  RelativePoseFactorBuilder(double translation_cap, double rotation_cap);

  bool CanBuild(const uw::domain::FactorCandidate& candidate) const override;

  std::unique_ptr<uw::measurement_api::ResidualBlock> Build(
      const uw::domain::FactorCandidate& candidate,
      const uw::domain::MeasurementEvidence& evidence,
      const uw::measurement_api::FactorBuildContext& context) const override;

 private:
  double translation_cap_;
  double rotation_cap_;
};

}  // namespace uw::factor_builders
