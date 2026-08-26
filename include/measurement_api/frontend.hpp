#pragma once

#include <optional>
#include <string>
#include <vector>

#include "domain/domain.hpp"
#include "sensor_models/geometry.hpp"

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

// A candidate revisit relative-pose evidence source: same RelativePoseMeasurement
// payload shape as VisualOdometryFrontend, but between two keyframes that may
// be arbitrarily far apart in time/keyframe-index (a "loop" edge, not a
// sequential dead-reckoning edge) — see include/frontends/
// loop_closure_frontend.hpp for the only implementation. Deliberately NOT
// shaped like VisualOdometryFrontend/OpticalDepthFrontend's "one bundle in,
// one evidence out": a loop-closure search legitimately may find zero,
// one, or several past keyframes worth proposing an edge against for a
// single current keyframe. `current_keyframe_id`/`current_pose_estimate`
// are what candidate retrieval searches against (see the implementation's
// own header comment for what "estimate" means at the point this is
// called).
class LoopClosureFrontend {
 public:
  virtual ~LoopClosureFrontend() = default;
  virtual std::vector<uw::domain::MeasurementEvidence> Process(
      const CameraFrameBundle& bundle, const uw::domain::RigCalibrationSnapshot& calibration,
      const std::string& current_keyframe_id,
      const uw::sensor_models::Pose3& current_pose_estimate) = 0;
  virtual uw::domain::HealthReport Health() const = 0;
};

}  // namespace uw::measurement_api
