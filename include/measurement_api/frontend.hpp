#pragma once

#include <optional>

#include "domain/domain.hpp"

namespace uw::measurement_api {

// A sonar-measurement frontend: SonarFrame in, a HypothesisSet of
// range-bearing candidates out (never a collapsed 6DoF pose — section 7.5).
// Other frontend kinds (visual, stereo depth) get their own narrow
// interface here as they're implemented; this repo intentionally does not
// force every frontend through one generic templated interface, since their
// inputs/outputs are physically different (architecture section 7.4).
class SonarFrontend {
 public:
  virtual ~SonarFrontend() = default;
  virtual uw::domain::HypothesisSet ProcessSonarFrame(const uw::domain::SonarFrame& frame) = 0;
  virtual uw::domain::HealthReport Health() const = 0;
};

struct CameraFrameBundle {
  uw::domain::ImageFrame primary;
  std::optional<uw::domain::ImageFrame> secondary;
};

class OpticalDepthFrontend {
 public:
  virtual ~OpticalDepthFrontend() = default;
  virtual std::optional<uw::domain::MeasurementEvidence> Process(
      const CameraFrameBundle& bundle,
      const uw::domain::RigCalibrationSnapshot& calibration) = 0;
  virtual uw::domain::HealthReport Health() const = 0;
};

// Relative-pose-from-camera-frames evidence (a RelativePoseMeasurement
// payload), as an alternative to a black-box VIO's opaque odometry output
// — see include/frontends/stereo_landmark_vo_frontend.hpp, the first
// implementation. Same "one bundle in, one evidence out" shape as
// OpticalDepthFrontend; an implementation that needs cross-time state
// (comparing this call's frame to the previous one) holds that privately,
// same as StereoOpticalDepthFrontend's frame counters — the interface
// itself stays stateless-looking on purpose (section 7.4).
class VisualOdometryFrontend {
 public:
  virtual ~VisualOdometryFrontend() = default;
  virtual std::optional<uw::domain::MeasurementEvidence> Process(
      const CameraFrameBundle& bundle,
      const uw::domain::RigCalibrationSnapshot& calibration) = 0;
  virtual uw::domain::HealthReport Health() const = 0;
};

}  // namespace uw::measurement_api
