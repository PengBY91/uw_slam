// Dependency-inversion seam between the ROS2 realtime gateway
// (adapters/ros2/, the only place ROS2 headers may appear -- see
// tools/lint/check_layer_dependencies.py's ROS_VENDOR_PREFIXES check) and
// the real online acoustic-optic assistance pipeline (LiveEventSource +
// OnlineAssistPipeline, application/runtime role -- strictly off-limits to
// any ROS-header-including translation unit per that same lint's `ros2`
// role, whose ALLOWED set is only {adapters, measurement_api,
// sensor_models, domain, domain_proto}).
//
// This header declares two pure-domain-typed abstract interfaces plus a
// factory function; it must never grow an application/runtime/opencv_adapters
// include itself. adapters/ros2/include/adapters/ros2_holoocean_realtime_gateway.hpp
// depends only on this header (an "adapters"-role dependency, allowed) to
// reach the pipeline, and implements HoloOceanRealtimeOutput itself to
// publish results back onto ROS2 topics -- so the ROS2-owned translation
// unit never needs to see LiveEventSource/OnlineAssistPipeline/
// OperatorOverlayRenderer at all. See
// src/application/holoocean_realtime_sink.cpp for the concrete
// implementation this factory returns.
#pragma once

#include <memory>
#include <string>

#include "domain/domain.hpp"

namespace uw::adapters {

// Consumes already-converted realtime sensor readings (see
// holoocean_live_conversion.hpp for the raw-HoloOcean -> uw::domain
// conversion the gateway performs before calling these).
class HoloOceanRealtimeSink {
 public:
  virtual ~HoloOceanRealtimeSink() = default;
  virtual void OnLeftCamera(uw::domain::ImageFrame frame) = 0;
  virtual void OnRightCamera(uw::domain::ImageFrame frame) = 0;
  // Presentation-only -- an implementation must never route this into the
  // algorithm pipeline (the plan's "independent pilot path" requirement).
  virtual void OnPilotCamera(uw::domain::ImageFrame frame) = 0;
  virtual void OnSonar(uw::domain::SonarFrame frame) = 0;
  virtual void OnVehicleState(uw::domain::VehicleState state) = 0;
};

// The sink's own output: an operator overlay image and a compact JSON
// status string, both replace-latest (no unbounded buffering). Implemented
// by the ROS2 gateway node (publishing sensor_msgs/Image + std_msgs/String)
// -- the sink implementation never needs to know it is talking to ROS2.
class HoloOceanRealtimeOutput {
 public:
  virtual ~HoloOceanRealtimeOutput() = default;
  virtual void PublishOverlay(uw::domain::ImageFrame frame) = 0;
  virtual void PublishStatus(std::string json_status) = 0;
};

// Builds the real production sink: internally owns a LiveEventSource, a
// real OnlineAssistPipeline (OpenCvVisualAssistFrontend + SonarCfarFrontend,
// dense depth disabled -- matching every other app in this repo), a
// PumpEvents worker thread, and an OperatorOverlayRenderer-backed
// AssistOutputSink that publishes through `output`. Defined in
// src/application/holoocean_realtime_sink.cpp (the application role may
// depend on everything this needs); this header itself stays free of
// application/runtime/opencv_adapters includes.
std::unique_ptr<HoloOceanRealtimeSink> MakeOnlineAssistRealtimeSink(
    HoloOceanRealtimeOutput& output, uw::domain::RigCalibrationSnapshot rig);

}  // namespace uw::adapters
