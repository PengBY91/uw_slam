#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>

#include "domain/domain.hpp"
#include "measurement_api/residual_block.hpp"

namespace uw::measurement_api {

// Optional spatial context a builder may need beyond the candidate/evidence
// pair — e.g. sonar_range_factor's landmark-cluster residual needs nearby
// already-estimated 3D points. Builders that don't need spatial context
// (relative_pose_factor, depth_factor) simply ignore this. In v1 this is
// populated from configuration or a submap query; see the sonar_range_factor
// (include/factor_builders/sonar_range_factor_builder.hpp) for the concrete
// usage and its documented v1 limitation (no joint landmark estimation yet).
struct FactorBuildContext {
  std::vector<Eigen::Vector3d> nearby_points_W;
};

// Per platform architecture section 7.6 / section 21 invariant #1:
// "FactorBuilder owns the mathematical model" — frontends propose
// FactorCandidate + evidence, only a FactorBuilder turns that into an
// actual residual in the graph. Frontends never inject arbitrary weights
// directly.
class FactorBuilder {
 public:
  virtual ~FactorBuilder() = default;

  // Does this builder know how to turn `candidate` into a ResidualBlock?
  // (matched on FactorCandidate.residual_model)
  virtual bool CanBuild(const uw::domain::FactorCandidate& candidate) const = 0;

  virtual std::unique_ptr<ResidualBlock> Build(const uw::domain::FactorCandidate& candidate,
                                                const uw::domain::MeasurementEvidence& evidence,
                                                const FactorBuildContext& context) const = 0;
};

}  // namespace uw::measurement_api
