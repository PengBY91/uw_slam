#pragma once

#include <string>

#include "frontends/target_associator.hpp"
#include "frontends/target_tracker.hpp"

namespace uw::frontends {

// Production binding for target fusion. The normalized parameter values
// stored here are the exact values used to construct both components and to
// produce the manifest fragment. A pipeline should publish that fragment
// only after it actually creates and uses this bundle.
class TargetFusionComponents {
 public:
  TargetFusionComponents(TargetAssociatorParams association_params,
                         TargetTrackerParams tracker_params);

  template <typename AssociationConfig, typename TrackerConfig>
  TargetFusionComponents(const AssociationConfig& association,
                         const TrackerConfig& tracker)
      : TargetFusionComponents(
            TargetAssociatorParams{
                association.max_corrected_time_delta_s,
                association.max_bearing_mahalanobis_sq,
                association.max_range_mahalanobis_sq,
                association.max_motion_bearing_delta_rad,
                association.max_motion_rate_rad_s,
                association.max_bearing_variance_rad2,
                association.max_range_variance_m2},
            TargetTrackerParams{
                tracker.association_mahalanobis_sq,
                tracker.confirm_hits,
                tracker.degraded_misses,
                tracker.stale_after_s,
                tracker.max_prediction_dt_s,
                tracker.bearing_acceleration_noise,
                tracker.range_acceleration_noise,
                tracker.merge_bearing_threshold_rad,
                tracker.merge_range_threshold_m}) {}

  TargetAssociator& associator() { return associator_; }
  const TargetAssociator& associator() const { return associator_; }
  TargetTracker& tracker() { return tracker_; }
  const TargetTracker& tracker() const { return tracker_; }
  const std::string& manifest_parameters() const {
    return manifest_parameters_;
  }

 private:
  TargetAssociatorParams association_params_;
  TargetTrackerParams tracker_params_;
  TargetAssociator associator_;
  TargetTracker tracker_;
  std::string manifest_parameters_;
};

}  // namespace uw::frontends
