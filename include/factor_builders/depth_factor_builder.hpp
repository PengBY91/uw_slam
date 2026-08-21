#pragma once

#include "measurement_api/factor_builder.hpp"

namespace uw::factor_builders {

class DepthFactorBuilder : public uw::measurement_api::FactorBuilder {
 public:
  static constexpr const char* kResidualModel = "depth_v1";

  bool CanBuild(const uw::domain::FactorCandidate& candidate) const override;

  std::unique_ptr<uw::measurement_api::ResidualBlock> Build(
      const uw::domain::FactorCandidate& candidate,
      const uw::domain::MeasurementEvidence& evidence,
      const uw::measurement_api::FactorBuildContext& context) const override;
};

}  // namespace uw::factor_builders
