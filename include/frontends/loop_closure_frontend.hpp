#pragma once

#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "frontends/harris_corner_detector.hpp"
#include "frontends/landmark_blob_detector.hpp"
#include "frontends/patch_matcher.hpp"
#include "frontends/rigid_transform_fit.hpp"
#include "measurement_api/frontend.hpp"

namespace uw::frontends {

struct LoopClosureFrontendParams {
  std::string left_sensor_id = "camera_left";
  std::string left_frame = "camera_left_link";
  std::string right_sensor_id = "camera_right";
  std::string right_frame = "camera_right_link";
  // Real-imagery corner detector only (see HarrisCornerDetector's own
  // header comment) -- unlike StereoLandmarkVoFrontend, this frontend has
  // no bright-blob/kBrightBlob mode: loop closure only makes sense once
  // there is a real trajectory to revisit, which is the same "real
  // imagery" regime Harris targets.
  HarrisCornerDetectorParams detector;
  PatchMatcherParams stereo_matcher;   // left<->right, triangulates the CURRENT keyframe's landmarks
  PatchMatcherParams revisit_matcher;  // current-keyframe <-> archived-candidate landmark matching
  double min_disparity_px = 1.0;       // matches BlockMatcher's/StereoLandmarkVoFrontend's convention
  int min_landmarks_for_pose = 3;      // floor applied before RANSAC even runs
  RansacParams ransac;                 // outlier rejection for the revisit pose fit
  CovarianceEstimationParams covariance_estimation;
  // Candidate retrieval (pose-proximity, NOT appearance/DBoW2 -- v1 scope
  // limit, see class comment below): a past keyframe is a candidate only if
  // it is within this radius of current_pose_estimate.translation.
  double candidate_search_radius_m = 3.0;
  // Excludes near-neighbors (by ARCHIVE INSERTION ORDER, i.e. keyframe
  // sequence position, not wall-clock time) from ever self-triggering as a
  // "loop" -- consecutive/nearby keyframes are always spatially close but
  // are not a genuine revisit.
  int min_keyframe_index_gap = 15;
  // Sanity gate on the RECOVERED relative pose: rejects a
  // geometrically-plausible-looking but implausibly large "revisit" jump
  // that RANSAC's own inlier-count/rmse gates didn't catch -- this
  // repo's own named bound, not a value copied from any reference
  // implementation.
  double max_accepted_translation_m = 5.0;
  double max_accepted_rotation_rad = 0.6;
  // Bounds how many loop edges one keyframe can add to the graph in a
  // single Process() call -- only the closest (by candidate_search_radius_m
  // distance) candidates, up to this count, are geometrically verified.
  int max_loop_edges_per_keyframe = 1;
  // Seeds this instance's own RNG, used only for RANSAC's minimal-sample
  // draws -- never reseeded after construction (CLAUDE.md's RNG
  // discipline / the L2 determinism test).
  uint64_t rng_seed = 12345;
};

// Pose-graph loop-closure evidence, architecture-inspired by SVIn's
// pose_graph module (DBoW2 + PnP RANSAC + 4DOF pose graph) but an ORIGINAL
// implementation -- no code ported from SVIn (see /NOTICE; this file is not
// listed there because nothing here is a line-level port). Reuses this
// repo's own existing, dependency-free geometric-verification machinery
// (HarrisCornerDetector + PatchMatcher + FitRigidTransformRansac) -- the
// SAME pipeline StereoLandmarkVoFrontend already uses for frame-to-frame
// VO, just applied across this frontend's own full-run keyframe archive
// instead of just the previous frame.
//
// Architecturally simpler than SVIn's pose_graph: this repo's estimator is
// a single global BATCH graph (uw::estimation::PoseGraphProblem), solved
// exactly once (see src/application/replay_pipeline.cpp). A loop edge here
// is just one more residual block added before that one Solve() call -- no
// separate async optimizer thread, no incremental re-optimization, and no
// "propagate drift to not-yet-optimized future frames" trick, all of which
// SVIn needs only because its VIO is incremental/online.
//
// v1 scope limits (documented, not silently papered over):
//  - Candidate retrieval is POSE-PROXIMITY ONLY (brute-force scan over
//    already-dead-reckoned keyframe positions), not DBoW2/appearance-only
//    retrieval. A revisit whose accumulated drift before this frontend
//    sees it exceeds `candidate_search_radius_m` is never found. Given
//    problem sizes here are "single-digit to low hundreds of keyframes"
//    (see gauss_newton_solver.hpp's own v1-scale comment), this is
//    adequate and avoids vendoring DBoW2/Brisk or any new third-party
//    dependency.
//  - Batch-only: all archiving and candidate search happen before the
//    single PoseGraphProblem::Solve() call (see replay_pipeline.cpp's
//    wiring) -- there is no incremental/online loop closure.
//  - No joint landmark re-estimation from loop closure -- matches
//    pose_graph_problem.hpp's existing "pose-only graph, no jointly-
//    optimized 3D landmarks" v1 limitation.
//
// Unlike VisualOdometryFrontend's "one bundle in, one evidence out" (a
// failure to track is a genuine health problem), finding zero loop
// candidates on a given Process() call is the ordinary/expected case here,
// not a health problem -- Health() always reports STATUS_HEALTHY.
class LoopClosureFrontend : public uw::measurement_api::LoopClosureFrontend {
 public:
  explicit LoopClosureFrontend(LoopClosureFrontendParams params);

  std::vector<uw::domain::MeasurementEvidence> Process(
      const uw::measurement_api::CameraFrameBundle& bundle,
      const uw::domain::RigCalibrationSnapshot& rig, const std::string& current_keyframe_id,
      const uw::sensor_models::Pose3& current_pose_estimate) override;
  uw::domain::HealthReport Health() const override;

 private:
  struct TriangulatedLandmark {
    Eigen::Vector3d camera_point;
    std::vector<uint8_t> patch;
  };
  // One past keyframe this frontend has seen, archived unconditionally at
  // the end of every Process() call regardless of whether a loop was found
  // this call. This archive does not exist anywhere else in the codebase:
  // replay_pipeline.cpp's left_by_kf_raw/right_by_kf_raw/get_rectified
  // already keep every keyframe's RAW images for the whole run, but
  // nothing else archives EXTRACTED landmark descriptors across more than
  // the immediately-previous frame (StereoLandmarkVoFrontend only ever
  // retains its previous call's landmarks).
  struct ArchivedKeyframe {
    std::string keyframe_id;
    Eigen::Vector3d position_W;                // current_pose_estimate at insertion time
    std::vector<TriangulatedLandmark> landmarks;
    int insertion_order = 0;                   // this frontend's own archive sequence position
  };

  LoopClosureFrontendParams params_;
  HarrisCornerDetector detector_;
  PatchMatcher stereo_matcher_;
  PatchMatcher revisit_matcher_;
  std::mt19937_64 rng_;
  std::vector<ArchivedKeyframe> archive_;
  uint64_t next_evidence_id_ = 1;
  uint64_t frames_processed_ = 0;
  uint64_t accepted_loops_ = 0;
  uint64_t rejected_candidates_ = 0;
};

}  // namespace uw::frontends
