// Cross-modal geometric association (design spec section 8.2/8.3) — NOT
// the posterior depth update (section 8.4, plan 4's
// AcousticOpticDepthFusionFrontend). Produces AcousticOpticAssociationRecords
// with prior_depth_m/prior_variance_m2 filled in and
// posterior_depth_m/posterior_variance_m2 left at zero; never sets
// POSTERIOR_INVALID or VARIANCE_NOT_IMPROVED. Inherits the repo-wide v1
// rule (hypothesis.proto) of consuming only the top-ranked HypothesisSet
// candidate — at most one record per call.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "domain/domain.hpp"
#include "sensor_models/camera_model.hpp"

namespace uw::frontends {

struct AcousticOpticAssociatorParams {
  std::string camera_sensor_id = "camera_left";
  std::string camera_frame = "camera_left_link";
  std::string sonar_sensor_id = "sonar0";
  std::string sonar_frame = "sonar_link";
  int arc_samples = 16;
  double range_gate_m = 0.5;
  double bearing_gate_rad = 0.1;
  double ambiguity_margin = 1.0;  // second_best_score - best_score must exceed this to accept
  int max_candidates = 8;
};

struct AssociationAuditResult {
  std::vector<uw::domain::AcousticOpticAssociationRecord> records;
  uw::domain::HealthReport health;
};

class AcousticOpticAssociator {
 public:
  explicit AcousticOpticAssociator(AcousticOpticAssociatorParams params);

  AssociationAuditResult Associate(const uw::domain::HypothesisSet& sonar_hypotheses,
                                   const uw::domain::MeasurementEvidence& optical_evidence,
                                   const uw::domain::RigCalibrationSnapshot& rig,
                                   double time_delta_seconds);

 private:
  AcousticOpticAssociatorParams params_;
  uint64_t frames_processed_ = 0;
  uint64_t frames_accepted_ = 0;
};

}  // namespace uw::frontends
