#pragma once

#include <optional>
#include <vector>

#include "domain/domain.hpp"

namespace uw::measurement_api {

// Adapter-facing interfaces, per platform architecture section 19/22.5: a
// black-box VIO (SVIn) is wrapped as a LocalOdometryProvider, a mapping
// baseline (sonar_camera_reconstruction) as a MapObservationProvider.
// Concrete implementations live in adapters/, never in core/algorithms —
// core only knows these interfaces.
class LocalOdometryProvider {
 public:
  virtual ~LocalOdometryProvider() = default;
  // Non-blocking poll; returns nullopt if no new relative-pose evidence is
  // available yet. Evidence payload is RelativePoseMeasurement.
  virtual std::optional<uw::domain::MeasurementEvidence> PollRelativePose() = 0;
  virtual uw::domain::HealthReport Health() const = 0;
};

class MapObservationProvider {
 public:
  virtual ~MapObservationProvider() = default;
  virtual std::vector<uw::domain::MapEvidence> PollMapEvidence() = 0;
  virtual uw::domain::HealthReport Health() const = 0;
};

// A sensor-gateway-side source of SonarFrame (e.g. a ROS2/simulator bridge
// that converts a vendor imaging-sonar topic into the domain type). Feeds
// SonarFrontend::ProcessSonarFrame (frontend.hpp); kept as a poll-based
// provider, mirroring LocalOdometryProvider/MapObservationProvider, so a
// scheduler lane can drain it without blocking on the adapter thread.
class SonarFrameProvider {
 public:
  virtual ~SonarFrameProvider() = default;
  virtual std::optional<uw::domain::SonarFrame> PollSonarFrame() = 0;
  virtual uw::domain::HealthReport Health() const = 0;
};

// A sensor-gateway-side source of ImageFrame, mirroring SonarFrameProvider.
// Emits independent raw frames per physical camera; runtime pairing into a
// CameraFrameBundle (frontend.hpp) belongs to the later synchronizer plan.
class CameraFrameProvider {
 public:
  virtual ~CameraFrameProvider() = default;
  virtual std::optional<uw::domain::ImageFrame> PollImageFrame() = 0;
  virtual uw::domain::HealthReport Health() const = 0;
};

}  // namespace uw::measurement_api
