#pragma once

#include "measurement_api/factor_builder.hpp"

namespace uw::factor_builders {

// Wires SonarRangeResidual into the FactorBuilder contract. Matches
// FactorCandidate.residual_model == kResidualModel.
//
// v1 limitation (documented, not hidden): this repository's estimator does
// not yet jointly estimate 3D landmarks (the fixed-lag graph variables are
// keyframe poses only, per holoocean-to-acoustic-optic-slam-pipeline
// section 10.4.5), so `context.nearby_points_W` cannot yet come from a
// live, continuously-updated landmark/submap store the way upstream SVIn's
// VIO landmarks do. In v1 it is supplied externally (scenario config for
// synthetic replay, or a future submap_manager query) as a small set of
// known/tracked 3D points near the sonar bearing. Wiring this up to a real
// submap query is the natural extension point once
// include/mapping/submap_manager.hpp accumulates points online.
class SonarRangeFactorBuilder : public uw::measurement_api::FactorBuilder {
 public:
  static constexpr const char* kResidualModel = "sonar_range_v1";

  bool CanBuild(const uw::domain::FactorCandidate& candidate) const override;

  std::unique_ptr<uw::measurement_api::ResidualBlock> Build(
      const uw::domain::FactorCandidate& candidate,
      const uw::domain::MeasurementEvidence& evidence,
      const uw::measurement_api::FactorBuildContext& context) const override;
};

}  // namespace uw::factor_builders
