// Sparse control-point localisation error (PREP-B-06).
//
// Why this exists: `ComputeAte` (trajectory_metrics.hpp) needs a dense
// ground-truth trajectory, which a simulation has and a real test pool does
// not. What a real pool DOES have is a handful of surveyed markers on the
// floor and walls whose world positions were measured with a tape or a
// total station. Observing a marker from an estimated pose predicts where
// that marker is in the world; the distance between that prediction and
// the surveyed position is an error the estimator is directly responsible
// for, with no ground-truth trajectory needed anywhere.
//
//   predicted_W = T_WB(estimate at observation time) * point_B
//   error       = || predicted_W - surveyed_W ||
//
// What this metric is and is not:
//   * It is a LOCALISATION error, not a mapping error — the observation's
//     own range/bearing noise enters it, so a per-point error can never be
//     smaller than the sensor's own accuracy on that marker. Report
//     sensor sigma alongside; do not read a 5 cm RMSE against a sonar with
//     5 cm range resolution as "the estimator is 5 cm good".
//   * It is UNALIGNED by default, unlike ComputeAte's optional Umeyama
//     step. Control points are surveyed in the same world frame the
//     scenario's poses are expressed in; if the estimate's frame is not
//     that frame, the fix is to anchor the run properly, not to fit the
//     discrepancy away. `align_before_scoring` exists for the pool case
//     where the estimator's anchor convention puts kf0 at the origin (see
//     ComputeAte's own comment on the same problem) and is opt-in for the
//     same reason.
//   * A marker observed from several poses contributes several errors, and
//     that is deliberate: an estimator that drifts should be penalised
//     once per drifted observation, not once per marker.
//
// Association (which detection is which marker) is the CALLER's job. In
// v1 that is a human labelling a replay; later it is CFAR detection plus
// nearest-neighbour, per PREP-B-06 step 2. This header only scores an
// already-associated set, so the metric cannot quietly absorb an
// association heuristic's mistakes.
#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

#include "evaluation/trajectory_metrics.hpp"
#include "sensor_models/geometry.hpp"

namespace uw::evaluation {

// One surveyed marker, mirroring uw::runtime::ScenarioControlPoint without
// depending on the runtime layer (evaluation sits beside it, not above it).
struct ControlPoint {
  std::string tag;
  Eigen::Vector3d position_W = Eigen::Vector3d::Zero();
  double size_m = 0.3;
};

// One sighting of a marker: where it appeared, in the observing body's own
// frame, at a given time.
struct ControlPointObservation {
  std::string tag;
  double timestamp_s = 0.0;
  // Marker position in base_link at `timestamp_s`. Callers that detect in
  // a sensor frame must apply the rig extrinsic first — this struct takes
  // base_link so the metric never has to know which sensor saw it.
  Eigen::Vector3d position_B = Eigen::Vector3d::Zero();
};

// Per-observation row of the report. `matched_pose_time_s` is the estimated
// pose actually used, so a reader can see how much time-matching slack was
// taken up.
struct ControlPointError {
  std::string tag;
  double timestamp_s = 0.0;
  double matched_pose_time_s = 0.0;
  double error_m = 0.0;
  Eigen::Vector3d predicted_W = Eigen::Vector3d::Zero();
  Eigen::Vector3d surveyed_W = Eigen::Vector3d::Zero();
};

struct ControlPointResult {
  double rmse_m = 0.0;
  double mean_m = 0.0;
  double p95_m = 0.0;
  double max_m = 0.0;
  std::size_t num_scored_observations = 0;
  // Observations dropped because no estimated pose was within
  // max_time_diff_s, or because their tag is not in `control_points`.
  std::size_t num_unmatched_observations = 0;
  std::size_t num_unknown_tags = 0;
  // Distinct markers that contributed at least one scored observation.
  std::size_t num_covered_control_points = 0;
  // Per-observation table, in input order. The PREP-B-06 acceptance work
  // (correlating this RMSE against full-trajectory ATE on the pool level)
  // needs the individual rows, not just the summary.
  std::vector<ControlPointError> per_observation;
};

// `max_time_diff_s` matches ComputeAte's default: an observation with no
// estimated pose within that window is reported as unmatched rather than
// scored against a stale pose.
//
// `align_before_scoring` fits the same rigid (no-scale) Umeyama transform
// ComputeAte uses, but from the OBSERVATION side: it aligns the predicted
// marker positions onto the surveyed ones. It needs at least 3 scored
// observations spanning 3 distinct markers to be meaningful and silently
// returns the unaligned result otherwise (again matching ComputeAte's
// behaviour rather than failing a whole evaluation run).
ControlPointResult ComputeControlPointError(const std::vector<TrajectoryPose>& estimated,
                                            const std::vector<ControlPoint>& control_points,
                                            const std::vector<ControlPointObservation>& observations,
                                            double max_time_diff_s = 0.05,
                                            bool align_before_scoring = false);

}  // namespace uw::evaluation
