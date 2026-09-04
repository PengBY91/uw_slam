// Assembles the online acoustic-optic target-assist slice: images/sonar/
// vehicle state in through the canonical PipelineInputPort, source-aware
// TargetTracks and a HealthReport-backed degradation summary out through a
// replace-latest AssistOutputSink. See docs/archive/superpowers/plans/2026-08-24-
// acoustic-optic-online-tracking.md Task 6.
//
// Visual and sonar target detection each run independently of the other --
// a sonar dropout does not stop visual-only tracks from publishing, and
// vice versa (that resilience is the point of a degraded-mode assist
// system). Local dense stereo depth is the one path gated behind
// AcousticOpticBuffer's fully synchronized stereo+sonar+state bundle,
// since it is the one computation genuinely expensive enough to need a
// realtime budget decision; see DenseDepthProvider below.
#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "application/assist_output_sink.hpp"
#include "application/pipeline_input_port.hpp"
#include "domain/domain.hpp"
#include "measurement_api/frontend.hpp"
#include "measurement_api/target_frontend.hpp"
#include "runtime/config.hpp"

namespace uw::application {

// Injectable local dense stereo depth completion. A production
// implementation wraps frontends::StereoOpticalDepthFrontend with a
// wall-clock budget check (see StereoBlockMatchDenseDepthProvider below);
// tests inject a fake to exercise the pipeline's budget/quality/in-flight
// gating deterministically without paying for real block matching.
// RunBounded returning nullopt covers every outcome the pipeline treats as
// "no usable depth this cycle" -- quality rejection, budget overrun, or
// failure. All surface identically as reason_code "dense_deadline_missed":
// the operator-facing vocabulary does not distinguish why dense didn't
// deliver, only that it didn't.
class DenseDepthProvider {
 public:
  virtual ~DenseDepthProvider() = default;
  virtual std::optional<uw::domain::OpticalDepthPriorMeasurement> RunBounded(
      const uw::measurement_api::CameraFrameBundle& images,
      const uw::domain::RigCalibrationSnapshot& rig, double budget_ms) = 0;
};

// Default production dense depth provider: frontends::StereoOpticalDepthFrontend
// plus a post-hoc wall-clock budget check. Block matching is not
// preemptible, so an overrun is detected after the fact and the result
// discarded, rather than the call itself being interrupted.
class StereoBlockMatchDenseDepthProvider final : public DenseDepthProvider {
 public:
  explicit StereoBlockMatchDenseDepthProvider(uw::runtime::StereoMatchingConfig matcher_config);
  ~StereoBlockMatchDenseDepthProvider() override;

  std::optional<uw::domain::OpticalDepthPriorMeasurement> RunBounded(
      const uw::measurement_api::CameraFrameBundle& images,
      const uw::domain::RigCalibrationSnapshot& rig, double budget_ms) override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

// Every field is a caller-owned, non-owning pointer/reference except the
// plain config/data values -- OnlineAssistPipeline never takes ownership of
// visual_frontend/sonar_frontend/dense_depth_provider/sink, matching how the
// rest of this codebase injects frontends (see e.g. AcousticOpticDepthFusionFrontend
// callers). dense_depth_provider may be null: dense then never runs even if
// dense.enabled is true, which is treated the same as a permanent
// dense_deadline_missed outcome whenever a bundle would otherwise trigger it.
struct OnlineAssistPipelineDependencies {
  uw::measurement_api::VisualAssistFrontend* visual_frontend = nullptr;
  uw::measurement_api::SonarFrontend* sonar_frontend = nullptr;
  DenseDepthProvider* dense_depth_provider = nullptr;
  AssistOutputSink* sink = nullptr;

  uw::domain::RigCalibrationSnapshot rig;
  uw::runtime::AcousticOpticBufferConfig buffer;
  uw::runtime::TargetAssociationConfig target_association;
  uw::runtime::TargetTrackerConfig target_tracker;
  uw::runtime::OnlineAssistPipelineConfig pipeline;

  // Wall clock used for data_age_ms/staleness bookkeeping, in the same
  // domain Stamp frame as every sensor header's capture_time -- NOT a
  // steady_clock (dense's own internal budget timing uses steady_clock
  // separately, since that measures real CPU cost, not sensor time).
  // Defaults to system_clock; tests inject a FakeClock closure that they
  // advance in lockstep with the capture times they feed in.
  std::function<uw::domain::Stamp()> now;
};

struct OnlineAssistPipelineDiagnostics {
  uint64_t published_count = 0;
  uint64_t dense_attempt_count = 0;
  uint64_t dense_deadline_missed_count = 0;
  uint64_t calibration_reset_count = 0;
  // Incremented every time a visual or sonar detection arrives after a gap
  // longer than modality_stale_after_s since that modality's own previous
  // detection (FUS-HEALTH-002) -- independent of how quickly the
  // resulting "recovering" state actually clears again (which depends on
  // tracker reconfirmation timing and isn't reliably observable from
  // outside), this counter is the one thing that reliably proves the
  // dropout+recovery trigger itself fired.
  uint64_t modality_recovery_count = 0;
  // TargetTracker::Update's batch is atomic: it rejects the whole batch
  // (no mutation) on non-finite/out-of-order input or an already-accepted
  // observation_id. That is a silent no-op from FlushAssociation's own
  // return type, so this counter is the only signal that an association
  // batch was ever dropped this way.
  uint64_t association_reject_count = 0;

  // Cumulative raw frontend output counts (SIM-ACC-002/FUS-ACC-001: an
  // acceptance run must show non-zero sonar/visual detections and a
  // non-zero fused track, not just "the process stayed alive"). Counted at
  // the frontend-output boundary (every element of RunSonarDetection's/
  // RunVisualDetection's own `targets`), not re-derived from published
  // track state, so a detection that never ends up associated into any
  // track is still counted here.
  uint64_t sonar_detection_count = 0;
  uint64_t visual_detection_count = 0;
  // Number of PublishNow() calls whose exported track set contained at
  // least one track carrying both ASSIST_SOURCE_VISUAL and
  // ASSIST_SOURCE_SONAR -- an approximate, cheap-to-compute proxy for "a
  // genuine acoustic-optic fused track was visible," not a count of
  // distinct fused tracks (the same track re-counts on every publish it
  // remains fused in). Nonzero is all downstream acceptance gating
  // actually checks for.
  uint64_t fused_track_publish_count = 0;
};

class OnlineAssistPipeline final : public PipelineInputPort {
 public:
  explicit OnlineAssistPipeline(OnlineAssistPipelineDependencies deps);
  ~OnlineAssistPipeline() override;
  OnlineAssistPipeline(OnlineAssistPipeline&&) noexcept;
  OnlineAssistPipeline& operator=(OnlineAssistPipeline&&) noexcept;
  OnlineAssistPipeline(const OnlineAssistPipeline&) = delete;
  OnlineAssistPipeline& operator=(const OnlineAssistPipeline&) = delete;

  bool OnImageFrame(const uw::runtime::CanonicalEvent& event) override;
  bool OnSonarFrame(const uw::runtime::CanonicalEvent& event) override;
  bool OnImuSample(const uw::runtime::CanonicalEvent& event) override;
  bool OnDvlSample(const uw::runtime::CanonicalEvent& event) override;
  bool OnVehicleState(const uw::runtime::CanonicalEvent& event) override;
  bool OnKeyframeBoundary(const uw::runtime::CanonicalEvent& event) override;
  bool OnMeasurementEvidence(const uw::runtime::CanonicalEvent& event) override;
  bool OnReferenceState(const uw::runtime::CanonicalEvent& event) override;
  bool OnHealthReport(const uw::runtime::CanonicalEvent& event) override;
  bool OnMapEvidence(const uw::runtime::CanonicalEvent& event) override;
  bool Flush() override;

  // Explicit calibration-change entry point (rig calibration is not itself
  // a CanonicalEvent -- see include/runtime/config.hpp's rig layer). Resets
  // the associator/tracker and AcousticOpticBuffer, clears pending dense
  // work, and publishes a "recovering" state immediately. Guidance is
  // invalid again until a track is CONFIRMED under the new version.
  void UpdateRig(uw::domain::RigCalibrationSnapshot rig);

  OnlineAssistPipelineDiagnostics Diagnostics() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace uw::application
