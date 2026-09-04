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
#include <fstream>
#include <future>
#include <iostream>
#include <mutex>
#include <optional>
#include <thread>
#include <type_traits>
#include <utility>

#include "adapters/opencv_visual_assist_frontend.hpp"
#include "adapters/operator_overlay_renderer.hpp"
#include "adapters/sim_wall_clock_estimator.hpp"
#include "application/assist_output_sink.hpp"
#include "application/event_pump.hpp"
#include "application/holoocean_status_json.hpp"
#include "application/online_assist_pipeline.hpp"
#include "application/pipeline_input_port.hpp"
#include "application/replay_pipeline.hpp"
#include "application/runtime_metrics_collector.hpp"
#include "frontends/sonar_cfar_frontend.hpp"
#include "runtime/canonical_topics.hpp"
#include "runtime/config.hpp"
#include "runtime/live_event_source.hpp"

namespace uw::adapters {
namespace {

uw::domain::RigCalibrationSnapshot ResolveRig(const HoloOceanRealtimeSinkConfig& config) {
  if (config.rig_config_path.empty()) {
    std::cerr << "holoocean_realtime_sink: WARNING no rig_config_path parameter given -- using "
                 "a placeholder identity-extrinsic rig. Every bearing/range projection is "
                 "geometrically wrong until a real calibrated rig is supplied; this run must "
                 "not be treated as real-machine acceptance evidence (FUS-CAL-001).\n";
    return config.fallback_rig;
  }
  // Deliberately NOT caught here: an operator who explicitly set
  // rig_config_path and got a load failure should see the node fail to
  // start, not silently fall back to the (wrong) placeholder rig above.
  return uw::runtime::LoadRigConfig(config.rig_config_path);
}

// Same policy as ResolveRig above, for sonar CFAR/clustering + target
// association/tracker + degradation-timing parameters (FUS-AC-002): empty
// path -> hardcoded struct defaults + loud warning; set-but-broken path ->
// hard error, not a silent fallback.
uw::runtime::PlatformDefaultsConfig ResolvePlatformDefaults(const HoloOceanRealtimeSinkConfig& config) {
  if (config.platform_config_path.empty()) {
    std::cerr << "holoocean_realtime_sink: WARNING no platform_config_path parameter given -- "
                 "using hardcoded C++ defaults for sonar CFAR/clustering, target association/"
                 "tracker gates and degradation timing instead of a versioned config file "
                 "(FUS-AC-002). Editing configs/defaults/platform.yaml has no effect on this "
                 "run until platform_config_path is set.\n";
    return uw::runtime::PlatformDefaultsConfig{};
  }
  return uw::runtime::LoadPlatformDefaultsConfig(config.platform_config_path);
}

int64_t SteadyNowNs() {
  const auto count = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::steady_clock::now().time_since_epoch())
                         .count();
  return count < 0 ? 0 : count;
}

double WallNowSeconds() {
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

// RuntimeMetricsConfig::queue_lane_capacities must track source_'s own
// LaneQueueConfig::capacity fields below (localization/correction/mapping/
// evidence, LiveEventSource::HealthReports()'s documented fixed order) --
// source_ is always constructed with LiveSourceConfig{} defaults, so these
// are hardcoded to match rather than plumbed through twice.
uw::application::RuntimeMetricsConfig MakeMetricsConfig(const HoloOceanRealtimeSinkConfig& config) {
  uw::application::RuntimeMetricsConfig metrics_config;
  metrics_config.deadline_ms = config.deadline_ms;
  metrics_config.queue_lane_capacities = {64, 32, 16, 256};
  return metrics_config;
}

// AssistOutputSink implementation feeding HoloOceanRealtimeOutput. Composes
// the latest pilot image + latest sonar frame (replace-latest, no
// unbounded buffering) with the OperatorAssistState via
// OperatorOverlayRenderer, then hands the result to `output` -- which is
// the ROS2 node itself, publishing sensor_msgs/Image + std_msgs/String.
class RealtimeAssistOutputSink final : public uw::application::AssistOutputSink {
 public:
  // `source` and `metrics` outlive this sink: OnlineAssistRealtimeSink
  // declares source_ and metrics_ before output_sink_, so both are already
  // fully constructed here, and all three are destroyed in reverse
  // declaration order (output_sink_ first). `pipeline` is not available yet
  // at this point (it is a unique_ptr the outer constructor's BODY builds,
  // after output_sink_ already exists as its own deps.sink) -- SetPipeline
  // below is called once the outer constructor has it.
  RealtimeAssistOutputSink(HoloOceanRealtimeOutput& output, const uw::runtime::LiveEventSource& source,
                            uw::application::RuntimeMetricsCollector& metrics, std::string run_report_path)
      : output_(output), source_(source), metrics_(metrics), run_report_path_(std::move(run_report_path)) {}

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
    const auto queue_health = source_.HealthReports();
    output_.PublishStatus(uw::application::BuildOnlineAssistStatusJson(state, queue_health));

    // Publish() is only ever invoked synchronously from within the pump
    // thread's own OnlineAssistPipeline::PublishNow() (see that class's
    // sink_->Publish(state) call) -- the same thread that owns/mutates
    // pipeline_'s diagnostics_, so reading it here via SetPipeline's raw
    // pointer needs no extra synchronization beyond metrics_'s own mutex.
    const double wall_now_s = WallNowSeconds();
    metrics_.ObservePublish(state, wall_now_s);
    metrics_.ObserveQueueHealth(queue_health);
    if (pipeline_ != nullptr) metrics_.ObserveDiagnostics(pipeline_->Diagnostics());
    metrics_.SampleResourceUsage(wall_now_s - process_start_wall_s_);
    MaybeWriteReport(wall_now_s);
  }

  void SetLatestPilotImage(uw::domain::ImageFrame image) {
    std::lock_guard<std::mutex> lock(latest_mutex_);
    latest_pilot_image_ = std::move(image);
  }

  void SetLatestSonarFrame(uw::domain::SonarFrame frame) {
    std::lock_guard<std::mutex> lock(latest_mutex_);
    latest_sonar_frame_ = std::move(frame);
  }

  void SetPipeline(const uw::application::OnlineAssistPipeline& pipeline) { pipeline_ = &pipeline; }

 private:
  void MaybeWriteReport(double wall_now_s) {
    if (run_report_path_.empty()) return;
    // Piggybacks on Publish()'s own throttled cadence (C1's
    // min_publish_interval_s, default 100ms) rather than a dedicated
    // writer thread -- see HoloOceanRealtimeSinkConfig::run_report_path's
    // doc comment. This extra >=1s gate keeps the actual file write (an
    // fstream open plus full JSON serialize) off the common publish path,
    // since realtime_gate.py only needs a report fresh to within a second
    // or two, not one rewritten on every throttled publish.
    if (last_report_write_wall_s_.has_value() && wall_now_s - *last_report_write_wall_s_ < 1.0) return;
    last_report_write_wall_s_ = wall_now_s;
    std::ofstream out(run_report_path_, std::ios::trunc);
    if (!out.is_open()) {
      std::cerr << "holoocean_realtime_sink: WARNING failed to open run_report_path '" << run_report_path_
                << "' for writing\n";
      return;
    }
    out << metrics_.BuildReportJson();
  }

  HoloOceanRealtimeOutput& output_;
  const uw::runtime::LiveEventSource& source_;
  uw::application::RuntimeMetricsCollector& metrics_;
  std::string run_report_path_;
  const uw::application::OnlineAssistPipeline* pipeline_ = nullptr;
  const double process_start_wall_s_ = WallNowSeconds();
  std::optional<double> last_report_write_wall_s_;
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
  bool OnKeyframeBoundary(const uw::runtime::CanonicalEvent& e) override {
    return pipeline_.OnKeyframeBoundary(e);
  }
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
  OnlineAssistRealtimeSink(HoloOceanRealtimeOutput& output, HoloOceanRealtimeSinkConfig config)
      : platform_defaults_(ResolvePlatformDefaults(config)),
        source_(uw::runtime::LiveSourceConfig{}),
        visual_frontend_((uw::opencv_adapters::VisualAssistParams{})),
        sonar_frontend_(uw::application::BuildSonarCfarFrontendParams(platform_defaults_.sonar_frontend)),
        metrics_(MakeMetricsConfig(config)),
        output_sink_(output, source_, metrics_, config.run_report_path) {
    uw::application::OnlineAssistPipelineDependencies deps;
    deps.rig = ResolveRig(config);
    deps.visual_frontend = &visual_frontend_;
    deps.sonar_frontend = &sonar_frontend_;
    deps.dense_depth_provider = nullptr;  // dense stays disabled -- matches every app in this repo so far
    deps.target_association = platform_defaults_.target_association;
    deps.target_tracker = platform_defaults_.target_tracker;
    deps.pipeline = platform_defaults_.online_assist;
    deps.pipeline.dense.enabled = false;
    deps.sink = &output_sink_;
    // capture_time on every sensor header arriving through this sink is
    // CLOCK_DOMAIN_SIMULATION (see holoocean_live_conversion.cpp), not wall
    // time -- wiring `now` straight to system_clock::now() here would make
    // every staleness/degradation check compare a ~0s sim timestamp against
    // a ~1.7e9s Unix timestamp and report itself permanently unavailable
    // from the first tick. sim_clock_ bridges the two domains; see
    // include/adapters/sim_wall_clock_estimator.hpp.
    deps.now = [this] { return sim_clock_.EstimateNow(); };
    pipeline_ = std::make_unique<uw::application::OnlineAssistPipeline>(std::move(deps));
    output_sink_.SetPipeline(*pipeline_);
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
  void OnMainCamera(uw::domain::ImageFrame frame) override {
    Submit(uw::runtime::kTopicCameraMain, std::move(frame));
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
    // header is a reference into payload -- read everything needed from it
    // BEFORE payload is moved into source_.Submit below.
    const auto& header = payload.header();
    if (header.clock_domain() == uw::domain::CLOCK_DOMAIN_SIMULATION) {
      // RTF needs the RAW (capture_s, wall_s) pair, not sim_clock_'s own
      // anchored/extrapolated estimate -- feeding EstimateNow() back into
      // itself here would just measure "how close to RTF=1 we assumed",
      // not the actual ratio.
      metrics_.ObserveSimTime(uw::domain::ToSeconds(header.capture_time()), WallNowSeconds());
      if constexpr (std::is_same_v<Payload, uw::domain::VehicleState>) {
        // Uses the anchor from BEFORE this message updates it just below
        // (sim_clock_.Observe(header)) -- calling EstimateNow() after would
        // trivially self-anchor to ~0 age every time, since this exact
        // message would already be the anchor.
        metrics_.ObserveVehicleState(header, uw::domain::ToSeconds(sim_clock_.EstimateNow()));
      }
    }
    // Anchors sim_clock_ using this message's own capture_time before it is
    // moved into the queue -- ingestion time is exactly when we can pair a
    // fresh (sim capture, wall receipt) sample, and doing it here (not on
    // the pump thread that later processes the event) keeps the anchor as
    // current as possible.
    sim_clock_.Observe(header);
    const auto status = source_.Submit(
        {topic, static_cast<uint64_t>(SteadyNowNs()), ++source_sequence_, std::move(payload)});
    if (status == uw::runtime::LiveSubmitStatus::kClosed) {
      // Node is shutting down (source_.Close() already called) -- nothing
      // further to do; every other status is a normal accept/degrade
      // outcome LiveEventSource itself already tracks in its own stats.
      return;
    }
  }

  uw::adapters::SimWallClockEstimator sim_clock_;
  // Declared before sonar_frontend_ (member init order follows declaration
  // order, not initializer-list order) -- sonar_frontend_'s initializer
  // reads platform_defaults_.sonar_frontend.
  uw::runtime::PlatformDefaultsConfig platform_defaults_;
  uw::runtime::LiveEventSource source_;
  uw::opencv_adapters::OpenCvVisualAssistFrontend visual_frontend_;
  uw::frontends::SonarCfarFrontend sonar_frontend_;
  // Declared before output_sink_ -- its constructor takes a reference to
  // this (same established pattern as source_ above).
  uw::application::RuntimeMetricsCollector metrics_;
  RealtimeAssistOutputSink output_sink_;
  std::unique_ptr<uw::application::OnlineAssistPipeline> pipeline_;
  std::unique_ptr<ForwardingPort> port_;
  std::atomic<uint64_t> source_sequence_{0};
  std::thread pump_thread_;
  std::promise<uw::runtime::EventSourceReport> report_promise_;
};

}  // namespace

std::unique_ptr<HoloOceanRealtimeSink> MakeOnlineAssistRealtimeSink(
    HoloOceanRealtimeOutput& output, HoloOceanRealtimeSinkConfig config) {
  return std::make_unique<OnlineAssistRealtimeSink>(output, std::move(config));
}

}  // namespace uw::adapters
