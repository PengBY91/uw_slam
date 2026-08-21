#pragma once

#include <cstdint>
#include <functional>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "uw/frontends/harris_corner_detector.hpp"
#include "uw/frontends/landmark_blob_detector.hpp"
#include "uw/frontends/patch_matcher.hpp"
#include "uw/frontends/rigid_transform_fit.hpp"
#include "uw/measurement_api/frontend.hpp"

namespace uw::frontends {

// Which landmark detector Process() runs on each image. kBrightBlob is
// tuned for apps/tools/synth_bag_gen's synthetic hash-patterned squares
// (see LandmarkBlobDetector's header comment); kHarrisCorner is the
// detector meant for real camera imagery, where there's no reason to
// expect isolated bright blobs (see HarrisCornerDetector's header
// comment). Defaults to kBrightBlob so existing callers (and the
// synthetic-scene demo/tests) are unaffected.
enum class LandmarkDetectorKind { kBrightBlob, kHarrisCorner };

struct StereoLandmarkVoFrontendParams {
  std::string left_sensor_id = "camera_left";
  std::string left_frame = "camera_left_link";
  std::string right_sensor_id = "camera_right";
  std::string right_frame = "camera_right_link";
  LandmarkDetectorKind detector_kind = LandmarkDetectorKind::kBrightBlob;
  LandmarkBlobDetectorParams detector;          // used when detector_kind == kBrightBlob
  HarrisCornerDetectorParams harris_detector;   // used when detector_kind == kHarrisCorner
  PatchMatcherParams stereo_matcher;    // left<->right correspondence, for triangulation
  PatchMatcherParams temporal_matcher;  // previous-left<->current-left correspondence, for the pose fit
  double min_disparity_px = 1.0;        // matches BlockMatcher's convention: disparity 0 => infinite depth
  int min_landmarks_for_pose = 3;       // floor applied before RANSAC even runs
  RansacParams ransac;                  // outlier rejection for the pose fit — see rigid_transform_fit.hpp
  // Seeds this instance's own RNG, used only for RANSAC's minimal-sample
  // draws — never reseeded after construction (CLAUDE.md's RNG
  // discipline / the L2 determinism test: same bag in, same output out).
  uint64_t rng_seed = 12345;
};

// Real (not ground-truth-derived) relative-pose evidence computed from
// stereo camera frames: detect bright landmark blobs in the left image
// (LandmarkBlobDetector), match each to a right-image blob by appearance
// (PatchMatcher, under an epipolar/row constraint) to triangulate a 3D
// point per landmark in the current camera frame, then match this frame's
// landmarks against the previous call's by appearance again (not by any
// externally-given id — a real frontend has no way to know landmark
// identity beyond what the image itself shows) and fit the rigid
// transform between the two matched 3D point sets — RANSAC over
// Kabsch/Procrustes SVD, see rigid_transform_fit.hpp, so one bad greedy
// NCC match doesn't corrupt the whole transition — to recover the
// relative pose between consecutive keyframes. This is the algorithm
// counterpart to
// apps/tools/synth_bag_gen's ground-truth+noise "black-box VIO" evidence
// stand-in — see that file's header comment and README's known
// boundaries section for why that stand-in existed.
//
// Stateful: holds the previous call's triangulated landmarks (3D point +
// appearance patch) as private members between Process() calls — the
// VisualOdometryFrontend interface itself (core/measurement_api/
// frontend.hpp) stays "one bundle in, one evidence out" on purpose, same
// pattern as StereoOpticalDepthFrontend's frame counters.
class StereoLandmarkVoFrontend : public uw::measurement_api::VisualOdometryFrontend {
 public:
  explicit StereoLandmarkVoFrontend(StereoLandmarkVoFrontendParams params);

  std::optional<uw::domain::MeasurementEvidence> Process(
      const uw::measurement_api::CameraFrameBundle& bundle,
      const uw::domain::RigCalibrationSnapshot& rig) override;
  uw::domain::HealthReport Health() const override;

 private:
  struct TriangulatedLandmark {
    Eigen::Vector3d camera_point;
    std::vector<uint8_t> patch;
  };

  StereoLandmarkVoFrontendParams params_;
  // Bound in the constructor to either detector per params_.detector_kind
  // — see that field's comment. Both detectors share LandmarkBlob as their
  // output type (PatchMatcher/Process() below don't care which produced
  // it), so a std::function is enough; no virtual interface needed.
  std::function<std::vector<LandmarkBlob>(const uint8_t*, uint32_t, uint32_t, uint32_t)> detect_;
  PatchMatcher stereo_matcher_;
  PatchMatcher temporal_matcher_;
  std::mt19937_64 rng_;

  bool has_previous_ = false;
  std::string previous_keyframe_id_;
  std::vector<TriangulatedLandmark> previous_landmarks_;

  uint64_t frames_processed_ = 0;
  uint64_t frames_rejected_ = 0;
  uint64_t next_evidence_id_ = 1;
};

}  // namespace uw::frontends
