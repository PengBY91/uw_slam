#pragma once

#include <optional>
#include <vector>

#include "domain/domain.hpp"

namespace uw::measurement_api {

struct VisualAssistResult {
  std::vector<uw::domain::TargetDetection> targets;
  std::optional<double> path_lateral_offset_m;
  std::optional<double> path_offset_sigma_m;
  uw::domain::HealthReport health;
};

class VisualAssistFrontend {
 public:
  virtual ~VisualAssistFrontend() = default;
  virtual VisualAssistResult Process(
      const uw::domain::ImageFrame& left_rectified,
      const std::optional<uw::domain::OpticalDepthPriorMeasurement>& depth,
      const uw::domain::CameraIntrinsics& intrinsics) = 0;
};

}  // namespace uw::measurement_api
