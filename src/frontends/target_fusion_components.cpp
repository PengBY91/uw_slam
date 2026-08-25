#include "frontends/target_fusion_components.hpp"

#include <locale>
#include <sstream>

namespace uw::frontends {
namespace {

std::string CanonicalParameters(const TargetAssociatorParams& association,
                                const TargetTrackerParams& tracker) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out.precision(17);
  out << "target_association.max_corrected_time_delta_s="
      << association.max_corrected_time_delta_s
      << ";max_bearing_mahalanobis_sq="
      << association.max_bearing_mahalanobis_sq
      << ";max_range_mahalanobis_sq="
      << association.max_range_mahalanobis_sq
      << ";max_motion_bearing_delta_rad="
      << association.max_motion_bearing_delta_rad
      << ";max_motion_rate_rad_s=" << association.max_motion_rate_rad_s
      << ";max_bearing_variance_rad2="
      << association.max_bearing_variance_rad2
      << ";max_range_variance_m2=" << association.max_range_variance_m2
      << ";target_tracker.association_mahalanobis_sq="
      << tracker.association_mahalanobis_sq
      << ";confirm_hits=" << tracker.confirm_hits
      << ";degraded_misses=" << tracker.degraded_misses
      << ";stale_after_s=" << tracker.stale_after_s
      << ";max_prediction_dt_s=" << tracker.max_prediction_dt_s
      << ";bearing_acceleration_noise="
      << tracker.bearing_acceleration_noise
      << ";range_acceleration_noise=" << tracker.range_acceleration_noise
      << ";merge_bearing_threshold_rad="
      << tracker.merge_bearing_threshold_rad
      << ";merge_range_threshold_m=" << tracker.merge_range_threshold_m;
  return out.str();
}

}  // namespace

TargetFusionComponents::TargetFusionComponents(
    TargetAssociatorParams association_params,
    TargetTrackerParams tracker_params)
    : association_params_(association_params),
      tracker_params_(tracker_params),
      associator_(association_params_),
      tracker_(tracker_params_),
      manifest_parameters_(
          CanonicalParameters(association_params_, tracker_params_)) {}

}  // namespace uw::frontends
