// Source-agnostic sink for CanonicalEvents. Both MCAP replay and a future
// vendor SDK live source feed the same PipelineInputPort implementation
// through PumpEvents (event_pump.hpp) -- this is what lets algorithm code
// stay unaware of where its input came from. Implementations must never
// leak MCAP/ROS2/vendor SDK types back out through this interface (see
// docs/superpowers/plans/2026-08-24-live-replay-unified-ingress.md section
// 4/1.1).
#pragma once

#include "runtime/canonical_event.hpp"

namespace uw::application {

class PipelineInputPort {
 public:
  virtual ~PipelineInputPort() = default;

  virtual bool OnImageFrame(const uw::runtime::CanonicalEvent& event) = 0;
  virtual bool OnSonarFrame(const uw::runtime::CanonicalEvent& event) = 0;
  virtual bool OnImuSample(const uw::runtime::CanonicalEvent& event) = 0;
  virtual bool OnDvlSample(const uw::runtime::CanonicalEvent& event) = 0;
  virtual bool OnVehicleState(const uw::runtime::CanonicalEvent& event) = 0;
  virtual bool OnMeasurementEvidence(const uw::runtime::CanonicalEvent& event) = 0;
  // Ground truth / reference-only data (currently just /gt/state) -- must
  // never be routed anywhere an online algorithm could read it as input.
  virtual bool OnReferenceState(const uw::runtime::CanonicalEvent& event) = 0;
  virtual bool OnHealthReport(const uw::runtime::CanonicalEvent& event) = 0;
  virtual bool OnMapEvidence(const uw::runtime::CanonicalEvent& event) = 0;

  // Called by PumpEvents exactly once, only when the underlying EventSource
  // finished normally (EventSourceStatus::kCompleted) -- never after the
  // source failed to open or was stopped early by this port's own On*
  // methods returning false.
  virtual bool Flush() = 0;
};

}  // namespace uw::application
