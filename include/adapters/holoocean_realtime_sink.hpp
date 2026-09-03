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
  // PREP-A-03: the contract vehicle's single gimbal camera -- an algorithm
  // input (routed to /raw/camera/main), unlike OnPilotCamera below. The
  // online pipeline picks its visual camera by matching the frame's
  // sensor_id ("camera_main") against the rig's first camera entry, so a
  // mono rig must list camera_main first.
  virtual void OnMainCamera(uw::domain::ImageFrame frame) = 0;
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

// How the sink resolves its RigCalibrationSnapshot and algorithm parameters
// (sonar CFAR/clustering, target association/tracker gates, degradation
// timing). The `ros2` lint role (adapters/ros2/) may only depend on
// {adapters, measurement_api, sensor_models, domain, domain_proto} -- NOT
// `runtime`, where the YAML loaders (uw::runtime::LoadRigConfig,
// LoadPlatformDefaultsConfig) live -- so the ROS2-owned translation unit
// cannot load either YAML file itself. It passes paths instead; the
// application-role .cpp behind this header does the actual loading.
//
// See docs/rov-realtime-closed-loop-code-review-2026-08-27.md findings B1
// (rig) and D2 (platform defaults): the gateway used to always wire in a
// placeholder identity-extrinsic rig (FUS-CAL-001 forbids that reaching
// real-machine acceptance) AND always default-construct
// VisualAssistParams/SonarCfarFrontendParams/target association/tracker
// config in C++ instead of reading them from version-controlled YAML
// (FUS-AC-002 requires the latter). For both `rig_config_path` and
// `platform_config_path`: empty means the corresponding `fallback_*` is
// used as-is and a loud warning is logged -- acceptable for a dev/smoke run
// with no calibration/tuning file on hand, never for a run meant to produce
// acceptance evidence. A non-empty path that fails to load (missing file,
// malformed YAML) is a hard error (throws) rather than a silent fallback --
// an operator who explicitly asked for real config and got a broken one
// should find out immediately, not get an unnoticed placeholder instead.
//
// `platform_config_path`'s visual-detector parameters
// (opencv_adapters::VisualAssistParams) are NOT part of this: no YAML
// schema for that frontend exists anywhere in this repo yet (it is
// default-constructed even in the offline pipeline), and that detector's
// suitability is itself an open question (see finding D1) -- inventing a
// config schema for a likely-to-be-replaced HSV-threshold placeholder was
// judged not worth doing ahead of that larger decision.
struct HoloOceanRealtimeSinkConfig {
  std::string rig_config_path;
  uw::domain::RigCalibrationSnapshot fallback_rig;
  std::string platform_config_path;
  // If non-empty, the sink periodically (roughly once per second, from the
  // pump thread's own Publish() cadence -- see
  // src/application/runtime_metrics_collector.cpp's doc comment for why a
  // dedicated writer thread was judged not worth the added complexity)
  // writes a JSON runtime-metrics report to this path: result/state age
  // percentiles, deadline-miss fraction, queue backpressure stats, RTF,
  // RSS growth, CPU headroom, recovery duration, detection/fused-track
  // counts, and guidance-marked-stale-when-overdue. Empty (the default)
  // means no report is written -- see
  // docs/rov-realtime-closed-loop-code-review-2026-08-27.md finding A2.
  std::string run_report_path;
  // Result-age budget (ms) a published state must not exceed to count as
  // "on time" for deadline_miss_fraction -- realtime_gate.py should pass
  // the profile-appropriate value (250ms nominal, matching FUS-RT-002).
  double deadline_ms = 250.0;
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
    HoloOceanRealtimeOutput& output, HoloOceanRealtimeSinkConfig config);

}  // namespace uw::adapters
