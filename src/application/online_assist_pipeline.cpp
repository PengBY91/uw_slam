#include "application/online_assist_pipeline.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>
#include <variant>
#include <vector>

#include "frontends/sonar_target_extractor.hpp"
#include "frontends/stereo_optical_depth_frontend.hpp"
#include "frontends/target_fusion_components.hpp"
#include "runtime/acoustic_optic_buffer.hpp"
#include "sensor_models/camera_model.hpp"

namespace uw::application {

namespace {

using uw::sensor_models::FindCamera;

uw::domain::Stamp DefaultNow() {
  const double seconds =
      std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
  return uw::domain::FromSeconds(seconds);
}

uw::frontends::SensorTargetDetection Wrap(uw::domain::TargetDetection detection,
                                          const uw::domain::ObservationHeader& header) {
  uw::frontends::SensorTargetDetection wrapped;
  wrapped.sensor_id = header.sensor_id().value();
  wrapped.sensor_frame = header.sensor_frame().value();
  wrapped.calibration_version = header.calibration_version().value();
  wrapped.detection = std::move(detection);
  return wrapped;
}

}  // namespace

// ---------------------------------------------------------------------
// StereoBlockMatchDenseDepthProvider
// ---------------------------------------------------------------------

class StereoBlockMatchDenseDepthProvider::Impl {
 public:
  explicit Impl(uw::runtime::StereoMatchingConfig matcher_config) {
    uw::frontends::StereoOpticalDepthFrontendParams params;
    params.matcher.min_texture_variance = matcher_config.min_texture_variance;
    params.matcher.min_uniqueness_margin = matcher_config.min_uniqueness_margin;
    params.matcher.left_right_max_diff_px = matcher_config.left_right_max_diff_px;
    frontend_ = std::make_unique<uw::frontends::StereoOpticalDepthFrontend>(params);
  }

  std::optional<uw::domain::OpticalDepthPriorMeasurement> RunBounded(
      const uw::measurement_api::CameraFrameBundle& images,
      const uw::domain::RigCalibrationSnapshot& rig, double budget_ms) {
    const auto start = std::chrono::steady_clock::now();
    auto evidence = frontend_->Process(images, rig);
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    if (!evidence.has_value() || elapsed_ms > budget_ms || !evidence->has_optical_depth_prior()) {
      return std::nullopt;
    }
    return evidence->optical_depth_prior();
  }

 private:
  std::unique_ptr<uw::frontends::StereoOpticalDepthFrontend> frontend_;
};

StereoBlockMatchDenseDepthProvider::StereoBlockMatchDenseDepthProvider(
    uw::runtime::StereoMatchingConfig matcher_config)
    : impl_(std::make_unique<Impl>(matcher_config)) {}
StereoBlockMatchDenseDepthProvider::~StereoBlockMatchDenseDepthProvider() = default;

std::optional<uw::domain::OpticalDepthPriorMeasurement> StereoBlockMatchDenseDepthProvider::RunBounded(
    const uw::measurement_api::CameraFrameBundle& images,
    const uw::domain::RigCalibrationSnapshot& rig, double budget_ms) {
  return impl_->RunBounded(images, rig, budget_ms);
}

// ---------------------------------------------------------------------
// OnlineAssistPipeline
// ---------------------------------------------------------------------

struct DegradationDecision {
  uw::domain::HealthReport::Status status = uw::domain::HealthReport::STATUS_HEALTHY;
  std::string reason;
  bool guidance_valid = true;
};

struct ExternalHealthEntry {
  uw::domain::HealthReport report;
  double received_wall_s = 0.0;
};

class OnlineAssistPipeline::Impl {
 public:
  explicit Impl(OnlineAssistPipelineDependencies deps)
      : visual_frontend_(deps.visual_frontend),
        sonar_frontend_(deps.sonar_frontend),
        dense_provider_(deps.dense_depth_provider),
        sink_(deps.sink),
        rig_(deps.rig),
        association_config_(deps.target_association),
        tracker_config_(deps.target_tracker),
        pipeline_config_(deps.pipeline),
        now_(deps.now ? std::move(deps.now) : std::function<uw::domain::Stamp()>(&DefaultNow)),
        buffer_(deps.buffer, rig_) {
    if (!visual_frontend_ || !sonar_frontend_ || !sink_) {
      throw std::invalid_argument(
          "OnlineAssistPipeline requires non-null visual_frontend, sonar_frontend and sink");
    }
    fusion_.emplace(association_config_, tracker_config_);
  }

  bool OnImageFrame(const uw::runtime::CanonicalEvent& event) {
    const auto& image = std::get<uw::domain::ImageFrame>(event.payload);
    auto bundle = buffer_.AddImage(image);
    if (image.header().sensor_id().value() == rig_.cameras(0).sensor_id().value()) {
      RunVisualDetection(image);
    }
    if (bundle.has_value()) HandleBundle(*bundle);
    PublishNow();
    return true;
  }

  bool OnSonarFrame(const uw::runtime::CanonicalEvent& event) {
    const auto& sonar = std::get<uw::domain::SonarFrame>(event.payload);
    auto bundle = buffer_.AddSonar(sonar);
    RunSonarDetection(sonar);
    if (bundle.has_value()) HandleBundle(*bundle);
    PublishNow();
    return true;
  }

  bool OnVehicleState(const uw::runtime::CanonicalEvent& event) {
    const auto& state = std::get<uw::domain::VehicleState>(event.payload);
    if (state.header().sensor_id().value() == rig_.vehicle_state_sensors(0).value()) {
      last_vehicle_state_capture_s_ = uw::domain::ToSeconds(state.header().capture_time());
    }
    auto bundle = buffer_.AddVehicleState(state);
    if (bundle.has_value()) HandleBundle(*bundle);
    PublishNow();
    return true;
  }

  bool OnHealthReport(const uw::runtime::CanonicalEvent& event) {
    const auto& report = std::get<uw::domain::HealthReport>(event.payload);
    // Keyed by our own receipt time (now_()), not the reporter's own
    // capture_time -- consistent with how visual/sonar/vehicle-state
    // liveness is judged elsewhere in this class ("how long since we last
    // heard from it"), and robust to a reporter with a skewed clock.
    external_health_[report.component_id()] = {report, uw::domain::ToSeconds(now_())};
    PublishNow();
    return true;
  }

  // DVL, reference/ground-truth, generic measurement evidence and map
  // evidence are outside this pipeline's algorithmic inputs -- accepted
  // (never stalls the online loop) but never fed into tracking.
  bool OnImuSample(const uw::runtime::CanonicalEvent&) { return true; }
  bool OnDvlSample(const uw::runtime::CanonicalEvent&) { return true; }
  bool OnMeasurementEvidence(const uw::runtime::CanonicalEvent&) { return true; }
  bool OnReferenceState(const uw::runtime::CanonicalEvent&) { return true; }
  bool OnMapEvidence(const uw::runtime::CanonicalEvent&) { return true; }

  bool Flush() {
    PublishNow();
    return true;
  }

  void UpdateRig(uw::domain::RigCalibrationSnapshot rig) {
    const bool version_changed =
        rig.calibration_version().value() != rig_.calibration_version().value();
    buffer_.UpdateRig(rig);
    rig_ = std::move(rig);
    if (!version_changed) return;
    fusion_.emplace(association_config_, tracker_config_);
    pending_visual_.clear();
    pending_sonar_.clear();
    pending_dense_depth_.reset();
    pending_dense_depth_capture_s_.reset();
    recovering_ = true;
    ++diagnostics_.calibration_reset_count;
    PublishNow();
  }

  OnlineAssistPipelineDiagnostics Diagnostics() const { return diagnostics_; }

 private:
  void RunVisualDetection(const uw::domain::ImageFrame& left_image) {
    const double capture_s = uw::domain::ToSeconds(left_image.header().capture_time());
    last_visual_capture_s_ = capture_s;
    const auto* intrinsics = FindCamera(rig_, left_image.header().sensor_id().value());
    if (!intrinsics) return;

    const std::optional<uw::domain::OpticalDepthPriorMeasurement> depth =
        DenseCurrentlyFresh(capture_s) ? pending_dense_depth_ : std::nullopt;

    const auto result = visual_frontend_->Process(left_image, depth, *intrinsics);
    last_visual_health_ = result.health;
    latest_path_lateral_offset_m_ = result.path_lateral_offset_m;
    latest_path_offset_sigma_m_ = result.path_offset_sigma_m;

    // Replaced, not appended: pending_visual_ holds only the latest visual
    // frame's own detections. Accumulating detections from several visual
    // frames here would let two same-target re-detections reach the same
    // Associate()+Update() batch -- TargetTracker matches at most one
    // detection to each existing track per batch, so the second, unpaired
    // re-detection of a target already claimed by the first (paired)
    // measurement would spawn a spurious duplicate track instead of
    // reinforcing the real one. The accepted tradeoff is that a visual
    // frame between two sonar arrivals can be superseded before it is ever
    // associated -- acceptable since the tracker's own prediction/gating
    // still converges on one track from the surviving detections, and a
    // true sonar dropout (see SonarRecentlyLive below) stops replacing and
    // flushes every visual frame instead.
    pending_visual_.clear();
    for (const auto& target : result.targets) {
      pending_visual_.push_back(Wrap(target, left_image.header()));
    }
    // Sonar drives the association batch under normal operation (see
    // RunSonarDetection): a visual detection just stages into
    // pending_visual_ and waits for the next sonar arrival to pair with,
    // rather than flushing immediately and consuming its observation_id
    // before sonar has a chance to see it -- TargetTracker::Update rejects
    // a whole batch outright on any already-accepted observation_id, so an
    // eagerly-flushed-then-reused stale detection would poison every
    // following pairing attempt. Flush eagerly here only when sonar itself
    // looks unavailable, so a real sonar dropout still produces
    // visual-only tracks promptly instead of waiting forever.
    if (!SonarRecentlyLive(capture_s)) FlushAssociation(capture_s);
  }

  void RunSonarDetection(const uw::domain::SonarFrame& sonar) {
    const double capture_s = uw::domain::ToSeconds(sonar.header().capture_time());
    last_sonar_capture_s_ = capture_s;

    const auto hypotheses = sonar_frontend_->ProcessSonarFrame(sonar);
    last_sonar_health_ = sonar_frontend_->Health();
    const auto targets = sonar_extractor_.Extract(hypotheses, sonar);

    pending_sonar_.clear();
    for (const auto& target : targets) {
      pending_sonar_.push_back(Wrap(target, sonar.header()));
    }
    FlushAssociation(capture_s);
  }

  bool SonarRecentlyLive(double capture_s) const {
    return last_sonar_capture_s_.has_value() &&
          (capture_s - *last_sonar_capture_s_) <= pipeline_config_.modality_stale_after_s;
  }

  void FlushAssociation(double now_s) {
    const auto association = fusion_->associator().Associate(pending_visual_, pending_sonar_, rig_);
    if (!fusion_->tracker().Update(association.measurements, now_s)) {
      ++diagnostics_.association_reject_count;
    }
    // Every id just submitted (paired or singleton, accepted or rejected)
    // must not be resubmitted -- TargetTracker::Update's atomic batch
    // rejects outright on any id it has already accepted, and a rejected
    // batch's ids would otherwise linger here as a permanently-stale
    // pairing partner for the next tick.
    pending_visual_.clear();
    pending_sonar_.clear();
    if (!recovering_) return;
    const auto tracks = fusion_->tracker().Tracks(now_s);
    const bool any_confirmed = std::any_of(tracks.begin(), tracks.end(), [](const auto& track) {
      return track.status == uw::domain::TARGET_TRACK_STATUS_CONFIRMED;
    });
    if (any_confirmed) recovering_ = false;
  }

  void HandleBundle(const uw::runtime::OnlineAcousticOpticBundle& bundle) {
    if (!pipeline_config_.dense.enabled || dense_task_in_flight_) return;
    if (!dense_provider_) {
      ++diagnostics_.dense_attempt_count;
      ++diagnostics_.dense_deadline_missed_count;
      return;
    }
    if (!bundle.images.primary.is_rectified() || !bundle.images.secondary.has_value() ||
        !bundle.images.secondary->is_rectified()) {
      // Rectification gate not satisfied -- not attempted, not counted as a
      // fresh failure. ComputeDegradation derives the dense health signal
      // from pending_dense_depth_'s own freshness (see DenseCurrentlyFresh),
      // so an indefinitely-broken rectification pipeline still surfaces
      // dense_deadline_missed once the last successful result ages out,
      // without this gate needing to touch any latched reason itself.
      return;
    }

    dense_task_in_flight_ = true;
    ++diagnostics_.dense_attempt_count;
    auto result =
        dense_provider_->RunBounded(bundle.images, rig_, pipeline_config_.dense.budget_ms);
    dense_task_in_flight_ = false;

    if (result.has_value()) {
      pending_dense_depth_ = std::move(result);
      pending_dense_depth_capture_s_ = uw::domain::ToSeconds(bundle.images.primary.header().capture_time());
    } else {
      pending_dense_depth_.reset();
      pending_dense_depth_capture_s_.reset();
      ++diagnostics_.dense_deadline_missed_count;
    }
  }

  // Dense depth counts as currently contributing only while enabled and a
  // successful result is still within its freshness window -- reusing the
  // exact check RunVisualDetection applies before using it as a depth
  // prior. The two calls use different "now" values on purpose (this one's
  // caller, ComputeDegradation, is reporting overall state at publish
  // time; RunVisualDetection's caller is deciding whether to use the depth
  // prior for one specific, earlier frame) -- wall_s here is always >= that
  // frame's capture_s, so this health check is never more optimistic than
  // the usage check, only possibly a step more conservative in the narrow
  // window between the two, which is the safe direction to be wrong in.
  bool DenseCurrentlyFresh(double wall_s) const {
    return pipeline_config_.dense.enabled && pending_dense_depth_.has_value() &&
          pending_dense_depth_capture_s_.has_value() &&
          (wall_s - *pending_dense_depth_capture_s_) <= pipeline_config_.modality_stale_after_s;
  }

  bool VisualLive(double wall_s) const {
    return last_visual_capture_s_.has_value() &&
          (wall_s - *last_visual_capture_s_) <= pipeline_config_.modality_stale_after_s;
  }

  bool SonarLive(double wall_s) const {
    return last_sonar_capture_s_.has_value() &&
          (wall_s - *last_sonar_capture_s_) <= pipeline_config_.modality_stale_after_s;
  }

  DegradationDecision ComputeDegradation(double wall_s) const {
    if (recovering_) return {uw::domain::HealthReport::STATUS_RECOVERING, "recovering", false};

    const bool visual_live = VisualLive(wall_s);
    const bool sonar_live = SonarLive(wall_s);
    const bool vehicle_state_ok =
        last_vehicle_state_capture_s_.has_value() &&
        (wall_s - *last_vehicle_state_capture_s_) <= pipeline_config_.vehicle_state_stale_after_s;

    if (!visual_live && !sonar_live) {
      return {uw::domain::HealthReport::STATUS_UNAVAILABLE, "all_assist_unavailable", false};
    }
    if (!vehicle_state_ok) {
      return {uw::domain::HealthReport::STATUS_UNAVAILABLE, "vehicle_state_stale", false};
    }
    if (!sonar_live) {
      return {uw::domain::HealthReport::STATUS_SUSPECT, "sonar_unavailable", true};
    }
    if (!visual_live) {
      return {uw::domain::HealthReport::STATUS_SUSPECT, "visual_unavailable", true};
    }
    // Visual is checked ahead of sonar below (both live, both frontend-
    // reported): an intentional fixed priority, not a coincidence of
    // order, matching the fixed priority chain above it. If both a visual
    // and a sonar frontend report degraded at once, the operator-facing
    // top-line reason is the visual one; the sonar report is still visible
    // in full in sensor_health(), just not promoted to system_health().
    if (last_visual_health_.has_value() &&
        last_visual_health_->status() != uw::domain::HealthReport::STATUS_HEALTHY) {
      return {uw::domain::HealthReport::STATUS_SUSPECT, last_visual_health_->reason_code(), true};
    }
    if (last_sonar_health_.has_value() &&
        last_sonar_health_->status() != uw::domain::HealthReport::STATUS_HEALTHY) {
      return {uw::domain::HealthReport::STATUS_SUSPECT, last_sonar_health_->reason_code(), true};
    }
    if (pipeline_config_.dense.enabled && !DenseCurrentlyFresh(wall_s)) {
      return {uw::domain::HealthReport::STATUS_SUSPECT, "dense_deadline_missed", true};
    }
    return {uw::domain::HealthReport::STATUS_HEALTHY, "", true};
  }

  void PublishNow() {
    const double wall_s = uw::domain::ToSeconds(now_());
    const auto decision = ComputeDegradation(wall_s);

    uw::domain::OperatorAssistState state;
    const auto track_set = fusion_->tracker().ToProtoSet(wall_s);
    if (track_set.has_value()) *state.mutable_target_tracks() = *track_set;

    // Gated on current visual liveness, not just has_value(): without this,
    // a path offset computed from the last frame before a camera dropout
    // would keep republishing forever with no staleness signal of its own
    // (guidance_valid stays true off of the track/vehicle-state checks
    // alone, which say nothing about this specific field's age).
    if (latest_path_lateral_offset_m_.has_value() && VisualLive(wall_s)) {
      state.set_has_path_lateral_offset(true);
      state.set_path_lateral_offset_m(*latest_path_lateral_offset_m_);
      if (latest_path_offset_sigma_m_.has_value()) {
        state.set_path_offset_sigma_m(*latest_path_offset_sigma_m_);
      }
    }

    uw::domain::HealthReport health;
    health.set_component_id("online_assist_pipeline");
    health.set_status(decision.status);
    health.set_reason_code(decision.reason);
    *state.mutable_system_health() = health;

    double newest_capture_s = -std::numeric_limits<double>::infinity();
    for (const auto& capture : {last_visual_capture_s_, last_sonar_capture_s_, last_vehicle_state_capture_s_}) {
      if (capture.has_value()) newest_capture_s = std::max(newest_capture_s, *capture);
    }
    state.set_data_age_ms(std::isfinite(newest_capture_s)
                              ? std::max(0.0, (wall_s - newest_capture_s) * 1000.0)
                              : 0.0);
    state.set_guidance_valid(decision.guidance_valid);
    state.set_degradation_reason(decision.reason);

    if (last_visual_health_.has_value()) *state.add_sensor_health() = *last_visual_health_;
    if (last_sonar_health_.has_value()) *state.add_sensor_health() = *last_sonar_health_;
    // Expired externally-reported health is dropped rather than republished
    // forever -- unlike this pipeline's own visual/sonar liveness, an
    // external reporter that stops sending has no other signal marking its
    // last report stale.
    for (const auto& [component_id, entry] : external_health_) {
      (void)component_id;
      if ((wall_s - entry.received_wall_s) <= pipeline_config_.modality_stale_after_s) {
        *state.add_sensor_health() = entry.report;
      }
    }

    sink_->Publish(state);
    ++diagnostics_.published_count;
  }

  uw::measurement_api::VisualAssistFrontend* visual_frontend_;
  uw::measurement_api::SonarFrontend* sonar_frontend_;
  DenseDepthProvider* dense_provider_;
  AssistOutputSink* sink_;

  uw::domain::RigCalibrationSnapshot rig_;
  uw::runtime::TargetAssociationConfig association_config_;
  uw::runtime::TargetTrackerConfig tracker_config_;
  uw::runtime::OnlineAssistPipelineConfig pipeline_config_;
  std::function<uw::domain::Stamp()> now_;

  uw::runtime::AcousticOpticBuffer buffer_;
  std::optional<uw::frontends::TargetFusionComponents> fusion_;
  uw::frontends::SonarTargetExtractor sonar_extractor_;

  std::vector<uw::frontends::SensorTargetDetection> pending_visual_;
  std::vector<uw::frontends::SensorTargetDetection> pending_sonar_;

  std::optional<double> last_visual_capture_s_;
  std::optional<double> last_sonar_capture_s_;
  std::optional<double> last_vehicle_state_capture_s_;

  std::optional<uw::domain::HealthReport> last_visual_health_;
  std::optional<uw::domain::HealthReport> last_sonar_health_;
  std::map<std::string, ExternalHealthEntry> external_health_;

  std::optional<double> latest_path_lateral_offset_m_;
  std::optional<double> latest_path_offset_sigma_m_;

  std::optional<uw::domain::OpticalDepthPriorMeasurement> pending_dense_depth_;
  std::optional<double> pending_dense_depth_capture_s_;
  // Guards re-entrant dense dispatch. Always false when this call returns,
  // since dense work runs synchronously today (see DenseDepthProvider's own
  // doc comment); kept so a future async provider doesn't need a pipeline
  // change to be dispatched safely.
  bool dense_task_in_flight_ = false;

  bool recovering_ = false;

  OnlineAssistPipelineDiagnostics diagnostics_;
};

OnlineAssistPipeline::OnlineAssistPipeline(OnlineAssistPipelineDependencies deps)
    : impl_(std::make_unique<Impl>(std::move(deps))) {}
OnlineAssistPipeline::~OnlineAssistPipeline() = default;
OnlineAssistPipeline::OnlineAssistPipeline(OnlineAssistPipeline&&) noexcept = default;
OnlineAssistPipeline& OnlineAssistPipeline::operator=(OnlineAssistPipeline&&) noexcept = default;

bool OnlineAssistPipeline::OnImageFrame(const uw::runtime::CanonicalEvent& event) {
  return impl_->OnImageFrame(event);
}
bool OnlineAssistPipeline::OnSonarFrame(const uw::runtime::CanonicalEvent& event) {
  return impl_->OnSonarFrame(event);
}
bool OnlineAssistPipeline::OnImuSample(const uw::runtime::CanonicalEvent& event) {
  return impl_->OnImuSample(event);
}
bool OnlineAssistPipeline::OnDvlSample(const uw::runtime::CanonicalEvent& event) {
  return impl_->OnDvlSample(event);
}
bool OnlineAssistPipeline::OnVehicleState(const uw::runtime::CanonicalEvent& event) {
  return impl_->OnVehicleState(event);
}
bool OnlineAssistPipeline::OnMeasurementEvidence(const uw::runtime::CanonicalEvent& event) {
  return impl_->OnMeasurementEvidence(event);
}
bool OnlineAssistPipeline::OnReferenceState(const uw::runtime::CanonicalEvent& event) {
  return impl_->OnReferenceState(event);
}
bool OnlineAssistPipeline::OnHealthReport(const uw::runtime::CanonicalEvent& event) {
  return impl_->OnHealthReport(event);
}
bool OnlineAssistPipeline::OnMapEvidence(const uw::runtime::CanonicalEvent& event) {
  return impl_->OnMapEvidence(event);
}
bool OnlineAssistPipeline::Flush() { return impl_->Flush(); }

void OnlineAssistPipeline::UpdateRig(uw::domain::RigCalibrationSnapshot rig) {
  impl_->UpdateRig(std::move(rig));
}

OnlineAssistPipelineDiagnostics OnlineAssistPipeline::Diagnostics() const {
  return impl_->Diagnostics();
}

}  // namespace uw::application
