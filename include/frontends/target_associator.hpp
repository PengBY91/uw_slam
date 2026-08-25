#pragma once

#include <optional>
#include <limits>
#include <string>
#include <type_traits>
#include <vector>

#include <Eigen/Core>

#include "domain/domain.hpp"

namespace uw::frontends {

struct TargetAssociatorParams {
  double max_corrected_time_delta_s = 0.05;
  double max_bearing_mahalanobis_sq = 9.0;
  double max_range_mahalanobis_sq = 9.0;
  double max_motion_bearing_delta_rad = 0.25;
  double max_motion_rate_rad_s = 1.5;
  double max_bearing_variance_rad2 = 0.25;
  double max_range_variance_m2 = 4.0;
};

// TargetDetection deliberately contains measurement data only. This
// envelope supplies the sensor/frame/version provenance required to resolve
// a versioned rig and correct capture time without guessing from an ID.
struct SensorTargetDetection {
  uw::domain::TargetDetection detection;
  std::string sensor_id;
  std::string sensor_frame;
  std::string calibration_version;
};

// Canonical base_link measurement consumed by TargetTracker. range_m is
// absent for a bearing-only ray; covariance(1,1) is never interpreted in
// that case.
struct TargetMeasurement {
  double corrected_time_s = 0.0;
  std::string class_label;
  double confidence = 0.0;
  double bearing_rad = 0.0;
  std::optional<double> range_m;
  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
  std::vector<uw::domain::AssistSource> sources;
  std::vector<uw::domain::ObservationId> observation_ids;
};

enum class AssociationReason {
  kAccepted,
  kInvalidInput,
  kCalibrationMismatch,
  kFrameUnresolved,
  kCorrectedTimeDelta,
  kClassIncompatible,
  kUncertainty,
  kBearingMahalanobis,
  kRangeMahalanobis,
  kMotionContinuity,
  kPairConflict,
};

struct AssociationDiagnostic {
  std::string visual_observation_id;
  std::string sonar_observation_id;
  bool accepted = false;
  AssociationReason reason = AssociationReason::kInvalidInput;
  double value = 0.0;
  // NaN means the decision has no scalar configured threshold (accepted,
  // invalid boundary input, class mismatch, or deterministic pair conflict).
  double threshold = std::numeric_limits<double>::quiet_NaN();
};

struct TargetAssociationResult {
  std::vector<TargetMeasurement> measurements;
  std::vector<AssociationDiagnostic> diagnostics;
};

class TargetAssociator {
 public:
  explicit TargetAssociator(TargetAssociatorParams params = {});

  // Accepts runtime::TargetAssociationConfig (or another structurally
  // identical resolved config) without introducing a forbidden
  // frontends->runtime dependency.
  template <typename Config,
            typename = std::enable_if_t<!std::is_same_v<std::decay_t<Config>,
                                                       TargetAssociatorParams>>>
  explicit TargetAssociator(const Config& config)
      : TargetAssociator(TargetAssociatorParams{
            config.max_corrected_time_delta_s,
            config.max_bearing_mahalanobis_sq,
            config.max_range_mahalanobis_sq,
            config.max_motion_bearing_delta_rad,
            config.max_motion_rate_rad_s,
            config.max_bearing_variance_rad2,
            config.max_range_variance_m2}) {}

  TargetAssociationResult Associate(
      const std::vector<SensorTargetDetection>& visual,
      const std::vector<SensorTargetDetection>& sonar,
      const uw::domain::RigCalibrationSnapshot& rig) const;

 private:
  TargetAssociatorParams params_;
};

}  // namespace uw::frontends
