// MVP stereo implementation of uw::measurement_api::OpticalDepthFrontend
// (include/measurement_api/frontend.hpp, added in
// the contract-foundation plan). Consumes only geometry — no learned
// disparity, no semantic gating (architecture invariant: frontends own
// measurement evidence, not policy).
#pragma once

#include <cstdint>
#include <string>

#include "frontends/block_matcher.hpp"
#include "measurement_api/frontend.hpp"
#include "sensor_models/camera_model.hpp"

namespace uw::frontends {

struct StereoOpticalDepthFrontendParams {
  std::string left_sensor_id = "camera_left";
  std::string left_frame = "camera_left_link";
  std::string right_sensor_id = "camera_right";
  std::string right_frame = "camera_right_link";
  BlockMatcherParams matcher;
  double disparity_sigma_px = 0.5;  // assumed fixed matcher uncertainty, propagated to variance_m2
};

class StereoOpticalDepthFrontend : public uw::measurement_api::OpticalDepthFrontend {
 public:
  explicit StereoOpticalDepthFrontend(StereoOpticalDepthFrontendParams params);

  std::optional<uw::domain::MeasurementEvidence> Process(
      const uw::measurement_api::CameraFrameBundle& bundle,
      const uw::domain::RigCalibrationSnapshot& rig) override;
  uw::domain::HealthReport Health() const override;

 private:
  StereoOpticalDepthFrontendParams params_;
  BlockMatcher matcher_;
  uint64_t frames_processed_ = 0;
  uint64_t frames_rejected_ = 0;
  uint64_t next_evidence_id_ = 1;
};

}  // namespace uw::frontends
