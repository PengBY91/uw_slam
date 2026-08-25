#pragma once

#include <string>
#include <vector>

#include "domain/domain.hpp"
#include "estimation/gauss_newton_solver.hpp"
#include "evaluation/trajectory_metrics.hpp"
#include "frontends/sonar_cfar_frontend.hpp"
#include "runtime/config.hpp"
#include "sensor_models/geometry.hpp"

namespace uw::application {

struct ReplayOptions {
  std::string bag_path;
  std::string experiment_path;
  std::string out_prefix = "/tmp/replay_demo";
  int max_iterations = -1;
  bool align_ate = false;
};

int RunReplayPipeline(const ReplayOptions& options,
                      const std::string& git_commit);

// Application-layer conversion keeps runtime and frontends as peer layers
// while ensuring every typed sonar default actually configures the frontend.
uw::frontends::SonarCfarFrontendParams BuildSonarCfarFrontendParams(
    const uw::runtime::SonarFrontendConfig& config);

// Pure decision extracted for unit testing (no IO): what a keyframe's
// snapshot should report as tracking_status, given the batch solver's
// convergence and (when running estimator_mode == "stereo_landmark_vo")
// that keyframe's VO frontend health AT THE TIME it was processed --
// never the frontend's FINAL health applied retroactively to every
// historical keyframe (see RunReplayPipeline's vo_health_by_keyframe).
// LOST outranks DEGRADED outranks TRACKING: a stalled solver can never
// report TRACKING even if VO itself is healthy, and VO STATUS_UNAVAILABLE
// can never be downgraded to merely DEGRADED just because the solver
// happened to converge.
struct ReplayTrackingInputs {
  bool solver_converged = false;
  bool vo_enabled = false;
  uw::domain::HealthReport::Status vo_health = uw::domain::HealthReport::STATUS_UNSPECIFIED;
};

uw::domain::StateSnapshot::TrackingStatus DecideTrackingStatus(const ReplayTrackingInputs& inputs);

// Pure snapshot construction extracted for unit testing (no IO/StateStore
// dependency). contributing_evidence is sorted and deduplicated by
// evidence_id value before being written, so the same logical inputs
// always produce byte-identical output regardless of the order evidence
// happened to be collected in (determinism_test.sh's actual contract).
struct StateSnapshotInputs {
  std::string state_id;
  uint64_t state_version = 0;
  uw::sensor_models::Pose3 pose;
  uw::domain::Stamp capture_timestamp;
  std::string calibration_version;
  std::vector<uw::domain::EvidenceId> contributing_evidence;
  uw::domain::StateSnapshot::TrackingStatus tracking_status =
      uw::domain::StateSnapshot::TRACKING_STATUS_UNSPECIFIED;
};

uw::domain::StateSnapshot BuildStateSnapshot(const StateSnapshotInputs& inputs);

// Counts a fused-depth measurement's valid pixels by contribution_mask --
// must be called BEFORE mapping::BuildMapEvidenceFromFusedDepth, which
// does not preserve per-point origin (see that function's own points-only
// output). Invalid pixels (contribution_mask ==
// DEPTH_CONTRIBUTION_INVALID, or a valid_mask bit of 0) count as neither.
struct MapContributionCounts {
  uint64_t optical_only_points = 0;
  uint64_t acoustic_optic_points = 0;
};

MapContributionCounts CountDepthContributions(const uw::domain::FusedDepthMeasurement& fused);

// Pure gate evaluation extracted for unit testing (no IO/exit-code
// coupling): returns the list of human-readable gate failure messages for
// the given run outcome, or an empty vector if every enabled gate passed.
// A non-empty result should make the caller exit non-zero (see
// RunReplayPipeline's own use of this). require_converged defaults on
// (see PlatformDefaultsConfig); every other gate is opt-in via a
// zero/negative disabling value.
std::vector<std::string> EvaluateReplayGates(const uw::runtime::PlatformDefaultsConfig& defaults,
                                             const uw::estimation::GaussNewtonSummary& solver,
                                             const uw::evaluation::AteResult& ate, int num_landmarks,
                                             const MapContributionCounts& contributions,
                                             int num_acoustic_optic_accepted);

}  // namespace uw::application
