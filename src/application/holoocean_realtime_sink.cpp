// Concrete implementation of uw::adapters::HoloOceanRealtimeSink (see that
// header's doc comment for why this seam exists). This file is the only
// place that actually owns a LiveEventSource + OnlineAssistPipeline for the
// HoloOcean realtime closed loop -- it deliberately includes no ROS2
// header anywhere, so tools/lint/check_layer_dependencies.py's ROS-vendor
// check never fires on it, and it is free to depend on the full
// application/runtime/opencv_adapters/frontends stack the `application`
// role allows.
#include "adapters/holoocean_realtime_sink.hpp"

#include <atomic>
#include <chrono>
#include <future>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
#include <thread>
#include <utility>

#include "adapters/opencv_visual_assist_frontend.hpp"
#include "adapters/operator_overlay_renderer.hpp"
#include "application/assist_output_sink.hpp"
#include "application/event_pump.hpp"
#include "application/online_assist_pipeline.hpp"
#include "application/pipeline_input_port.hpp"
#include "frontends/sonar_cfar_frontend.hpp"
#include "runtime/canonical_topics.hpp"
#include "runtime/live_event_source.hpp"

namespace uw::adapters {
namespace {

int64_t SteadyNowNs() {
  const auto count = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::steady_clock::now().time_since_epoch())
                         .count();
  return count < 0 ? 0 : count;
}

std::string JsonEscape(const std::string& text) {
  std::string escaped;
  escaped.reserve(text.size());
  for (const char c : text) {
    switch (c) {
      case '"':
        escaped += "\\\"";
        break;
      case '\\':
        escaped += "\\\\";
        break;
      case '\n':
        escaped += "\\n";
        break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          std::ostringstream oss;
          oss << "\\u" << std::hex << std::setw(4) << std::setfill('0')
              << static_cast<int>(static_cast<unsigned char>(c));
          escaped += oss.str();
        } else {
          escaped += c;
        }
    }
  }
  return escaped;
}

std::string HealthReportToJson(const uw::domain::HealthReport& health) {
  std::ostringstream oss;
  oss << "{\"component_id\":\"" << JsonEscape(health.component_id()) << "\",\"status\":"
      << static_cast<int>(health.status()) << ",\"reason_code\":\""
      << JsonEscape(health.reason_code()) << "\"}";
  return oss.str();
}

// Compact JSON status: target/path values, source, confidence, data age,
// discrete guidance state, every sensor's health and degradation reason --
// exactly the fields the plan's Task 4 text requires, drawn straight from
// uw.domain.OperatorAssistState (schemas/proto/uw/domain/target.proto).
std::string BuildStatusJson(const uw::domain::OperatorAssistState& state) {
  std::ostringstream oss;
  oss << "{";
  oss << "\"guidance_valid\":" << (state.guidance_valid() ? "true" : "false") << ",";
  oss << "\"degradation_reason\":\"" << JsonEscape(state.degradation_reason()) << "\",";
  oss << "\"data_age_ms\":" << state.data_age_ms() << ",";
  oss << "\"system_health\":" << HealthReportToJson(state.system_health()) << ",";
  oss << "\"has_path_lateral_offset\":" << (state.has_path_lateral_offset() ? "true" : "false") << ",";
  oss << "\"path_lateral_offset_m\":" << state.path_lateral_offset_m() << ",";
  oss << "\"path_offset_sigma_m\":" << state.path_offset_sigma_m() << ",";
  oss << "\"sensor_health\":[";
  for (int i = 0; i < state.sensor_health_size(); ++i) {
    if (i > 0) oss << ",";
    oss << HealthReportToJson(state.sensor_health(i));
  }
  oss << "],";
  oss << "\"target_tracks\":[";
  for (int i = 0; i < state.target_tracks().tracks_size(); ++i) {
    if (i > 0) oss << ",";
    const auto& track = state.target_tracks().tracks(i);
    oss << "{\"track_id\":\"" << JsonEscape(track.track_id().value()) << "\",\"class_label\":\""
        << JsonEscape(track.class_label()) << "\",\"confidence\":" << track.class_confidence()
        << ",\"bearing_rad\":" << track.bearing_rad()
        << ",\"has_range\":" << (track.has_range_m() ? "true" : "false")
        << ",\"range_m\":" << (track.has_range_m() ? track.range_m() : 0.0)
        << ",\"status\":" << static_cast<int>(track.status()) << ",\"sources\":[";
    for (int s = 0; s < track.sources_size(); ++s) {
      if (s > 0) oss << ",";
      oss << static_cast<int>(track.sources(s));
    }
    oss << "]}";
  }
  oss << "]";
  oss << "}";
  return oss.str();
}

// AssistOutputSink implementation feeding HoloOceanRealtimeOutput. Composes
// the latest pilot image + latest sonar frame (replace-latest, no
// unbounded buffering) with the OperatorAssistState via
// OperatorOverlayRenderer, then hands the result to `output` -- which is
// the ROS2 node itself, publishing sensor_msgs/Image + std_msgs/String.
class RealtimeAssistOutputSink final : public uw::application::AssistOutputSink {
 public:
  explicit RealtimeAssistOutputSink(HoloOceanRealtimeOutput& output) : output_(output) {}

  void Publish(const uw::domain::OperatorAssistState& state) override {
    std::optional<uw::domain::ImageFrame> pilot_image;
    std::optional<uw::domain::SonarFrame> sonar_frame;
    {
      std::lock_guard<std::mutex> lock(latest_mutex_);
      pilot_image = latest_pilot_image_;
      sonar_frame = latest_sonar_frame_;
    }
    if (pilot_image.has_value()) {
      const auto overlay = renderer_.Render(*pilot_image, sonar_frame, state);
      if (overlay.has_value()) output_.PublishOverlay(*overlay);
    }
    output_.PublishStatus(BuildStatusJson(state));
  }

  void SetLatestPilotImage(uw::domain::ImageFrame image) {
    std::lock_guard<std::mutex> lock(latest_mutex_);
    latest_pilot_image_ = std::move(image);
  }

  void SetLatestSonarFrame(uw::domain::SonarFrame frame) {
    std::lock_guard<std::mutex> lock(latest_mutex_);
    latest_sonar_frame_ = std::move(frame);
  }

 private:
  HoloOceanRealtimeOutput& output_;
  uw::opencv_adapters::OperatorOverlayRenderer renderer_;
  std::mutex latest_mutex_;
  std::optional<uw::domain::ImageFrame> latest_pilot_image_;
  std::optional<uw::domain::SonarFrame> latest_sonar_frame_;
};

// PipelineInputPort adapter forwarding straight to a real OnlineAssistPipeline
// -- same thin-forwarding role as apps/online_assist_smoke.cpp's
// ReferenceCountingPort, minus the truth-delivery counter (this sink is
// never handed a reference-plane event -- the ROS2 gateway that owns it
// never subscribes /uw/sim/ground_truth).
class ForwardingPort final : public uw::application::PipelineInputPort {
 public:
  explicit ForwardingPort(uw::application::OnlineAssistPipeline& pipeline) : pipeline_(pipeline) {}

  bool OnImageFrame(const uw::runtime::CanonicalEvent& e) override { return pipeline_.OnImageFrame(e); }
  bool OnSonarFrame(const uw::runtime::CanonicalEvent& e) override { return pipeline_.OnSonarFrame(e); }
  bool OnImuSample(const uw::runtime::CanonicalEvent& e) override { return pipeline_.OnImuSample(e); }
  bool OnDvlSample(const uw::runtime::CanonicalEvent& e) override { return pipeline_.OnDvlSample(e); }
  bool OnVehicleState(const uw::runtime::CanonicalEvent& e) override { return pipeline_.OnVehicleState(e); }
  bool OnMeasurementEvidence(const uw::runtime::CanonicalEvent& e) override {
    return pipeline_.OnMeasurementEvidence(e);
  }
  bool OnReferenceState(const uw::runtime::CanonicalEvent& e) override {
    return pipeline_.OnReferenceState(e);
  }
  bool OnHealthReport(const uw::runtime::CanonicalEvent& e) override { return pipeline_.OnHealthReport(e); }
  bool OnMapEvidence(const uw::runtime::CanonicalEvent& e) override { return pipeline_.OnMapEvidence(e); }
  bool Flush() override { return pipeline_.Flush(); }

 private:
  uw::application::OnlineAssistPipeline& pipeline_;
};

class OnlineAssistRealtimeSink final : public HoloOceanRealtimeSink {
 public:
  OnlineAssistRealtimeSink(HoloOceanRealtimeOutput& output, uw::domain::RigCalibrationSnapshot rig)
      : source_(uw::runtime::LiveSourceConfig{}),
        visual_frontend_((uw::opencv_adapters::VisualAssistParams{})),
        sonar_frontend_((uw::frontends::SonarCfarFrontendParams{})),
        output_sink_(output) {
    uw::application::OnlineAssistPipelineDependencies deps;
    deps.rig = std::move(rig);
    deps.visual_frontend = &visual_frontend_;
    deps.sonar_frontend = &sonar_frontend_;
    deps.dense_depth_provider = nullptr;  // dense stays disabled -- matches every app in this repo so far
    deps.pipeline.dense.enabled = false;
    deps.sink = &output_sink_;
    deps.now = [] { return uw::domain::ToStamp(std::chrono::system_clock::now()); };
    pipeline_ = std::make_unique<uw::application::OnlineAssistPipeline>(std::move(deps));
    port_ = std::make_unique<ForwardingPort>(*pipeline_);

    pump_thread_ = std::thread([this] {
      try {
        report_promise_.set_value(uw::application::PumpEvents(source_, *port_));
      } catch (const std::exception& error) {
        // Nothing currently calls report_promise_.get_future(), so a lost
        // exception here would otherwise be a silent pump-thread death with
        // no signal anywhere the node's assist output stopped updating --
        // log it so it is at least visible in the node's own stderr/log
        // output, matching this class's own ROS-free design (no rclcpp
        // logger available here).
        std::cerr << "holoocean_realtime_sink: PumpEvents thread terminated: " << error.what() << '\n';
        source_.Close();
        try {
          report_promise_.set_exception(std::current_exception());
        } catch (...) {
        }
      } catch (...) {
        std::cerr << "holoocean_realtime_sink: PumpEvents thread terminated: unknown exception\n";
        source_.Close();
        try {
          report_promise_.set_exception(std::current_exception());
        } catch (...) {
        }
      }
    });
  }

  ~OnlineAssistRealtimeSink() override {
    source_.Close();
    if (pump_thread_.joinable()) pump_thread_.join();
  }

  void OnLeftCamera(uw::domain::ImageFrame frame) override {
    Submit(uw::runtime::kTopicCameraLeft, std::move(frame));
  }
  void OnRightCamera(uw::domain::ImageFrame frame) override {
    Submit(uw::runtime::kTopicCameraRight, std::move(frame));
  }
  void OnPilotCamera(uw::domain::ImageFrame frame) override {
    // Presentation-only -- cached for the overlay compositor, NEVER
    // submitted to LiveEventSource/OnlineAssistPipeline (the plan's
    // explicit "independent pilot path" requirement).
    output_sink_.SetLatestPilotImage(std::move(frame));
  }
  void OnSonar(uw::domain::SonarFrame frame) override {
    output_sink_.SetLatestSonarFrame(frame);
    Submit(uw::runtime::kTopicSonarFrame, std::move(frame));
  }
  void OnVehicleState(uw::domain::VehicleState state) override {
    Submit(uw::runtime::kTopicVehicleState, std::move(state));
  }

 private:
  template <typename Payload>
  void Submit(const char* topic, Payload payload) {
    const auto status = source_.Submit(
        {topic, static_cast<uint64_t>(SteadyNowNs()), ++source_sequence_, std::move(payload)});
    if (status == uw::runtime::LiveSubmitStatus::kClosed) {
      // Node is shutting down (source_.Close() already called) -- nothing
      // further to do; every other status is a normal accept/degrade
      // outcome LiveEventSource itself already tracks in its own stats.
      return;
    }
  }

  uw::runtime::LiveEventSource source_;
  uw::opencv_adapters::OpenCvVisualAssistFrontend visual_frontend_;
  uw::frontends::SonarCfarFrontend sonar_frontend_;
  RealtimeAssistOutputSink output_sink_;
  std::unique_ptr<uw::application::OnlineAssistPipeline> pipeline_;
  std::unique_ptr<ForwardingPort> port_;
  std::atomic<uint64_t> source_sequence_{0};
  std::thread pump_thread_;
  std::promise<uw::runtime::EventSourceReport> report_promise_;
};

}  // namespace

std::unique_ptr<HoloOceanRealtimeSink> MakeOnlineAssistRealtimeSink(
    HoloOceanRealtimeOutput& output, uw::domain::RigCalibrationSnapshot rig) {
  return std::make_unique<OnlineAssistRealtimeSink>(output, std::move(rig));
}

}  // namespace uw::adapters
