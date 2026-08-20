#pragma once

#include <cstdint>
#include <optional>

#include "uw/frontends/acoustic_optic_associator.hpp"
#include "uw/frontends/posterior_depth_optimizer.hpp"

namespace uw::frontends {

struct AcousticOpticDepthFusionParams {
  AcousticOpticAssociatorParams associator;
  PosteriorDepthOptimizerParams optimizer;
  double min_variance_improvement_fraction = 0.05;
  double innovation_gate_sigma = 3.0;
};

struct FusedDepthResult {
  uw::domain::MeasurementEvidence fused_evidence;
  uw::domain::HealthReport health;
};

// AcousticOpticDepthFusionFrontend::Fuse (design spec section 7.2/8.4): runs
// AcousticOpticAssociator's geometric association, then — only for an
// ACCEPTED record — a bounded posterior depth optimization at the selected
// pixel. Fails closed at every stage (design spec section 9's "不能证明
// 一致，就不融合" policy): any geometric rejection, non-finite/un-improved
// posterior, or innovation-gate failure leaves that pixel's fused depth
// equal to the optical prior (DEPTH_CONTRIBUTION_OPTICAL_ONLY), never
// fabricates a corrected value. An empty HypothesisSet (sonar dropout,
// design spec section 10 scenario 8) degrades gracefully to a full
// optical-only passthrough — documented behavior, not an error. Returns
// nullopt only when the optical evidence itself has no
// OpticalDepthPriorMeasurement payload (nothing to build from).
class AcousticOpticDepthFusionFrontend {
 public:
  explicit AcousticOpticDepthFusionFrontend(AcousticOpticDepthFusionParams params);

  std::optional<FusedDepthResult> Fuse(const uw::domain::HypothesisSet& sonar_hypotheses,
                                       const uw::domain::MeasurementEvidence& optical_evidence,
                                       const uw::domain::RigCalibrationSnapshot& rig,
                                       double time_delta_seconds);

 private:
  AcousticOpticDepthFusionParams params_;
  AcousticOpticAssociator associator_;
  uint64_t next_evidence_id_ = 1;
};

}  // namespace uw::frontends
