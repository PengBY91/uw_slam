#include "frontends/stereo_landmark_vo_frontend.hpp"

#include "frontends/rigid_transform_fit.hpp"
#include "sensor_models/camera_model.hpp"

namespace uw::frontends {
namespace {

// Same lookup as apps/synth_bag_gen.cpp's local FindRigEdgePose (and
// several algorithms/ callers — see acoustic_optic_depth_fusion_frontend.cpp,
// acoustic_optic_associator.cpp, acoustic_optic_map_bridge.cpp for the same
// pattern; there is no shared public rig-lookup utility to call instead).
uw::sensor_models::Pose3 FindRigEdgePose(const uw::domain::RigCalibrationSnapshot& rig,
                                          const std::string& child_frame) {
  for (const auto& edge : rig.frame_tree()) {
    if (edge.child_frame().value() == child_frame) return uw::sensor_models::Pose3::FromProto(edge.transform());
  }
  return uw::sensor_models::Pose3::Identity();
}

// The Pose3 that maps a point already expressed in the LEFT camera's
// OPTICAL frame (PinholeCamera::Project/Unproject's convention) into the
// rig's body frame — i.e. body_T_camera_optical.Apply(p_optical) == p_body.
// Composes the frame_tree edge (base_link -> camera_left_link, itself in
// BODY convention) with the inverse of the fixed optical<->body axis
// rotation (see camera_model.hpp's OpticalFromBodyRotation doc comment,
// and synth_bag_gen's BuildStereoPair for the forward direction of this
// exact composition — world/body point down to optical, this is that
// same chain run backwards).
uw::sensor_models::Pose3 BodyFromCameraOptical(const uw::domain::RigCalibrationSnapshot& rig,
                                                const std::string& camera_frame) {
  const auto camera_link_body_pose = FindRigEdgePose(rig, camera_frame);
  uw::sensor_models::Pose3 optical_to_body_rotation;
  optical_to_body_rotation.rotation = Eigen::Quaterniond(uw::sensor_models::OpticalFromBodyRotation()).inverse();
  return camera_link_body_pose * optical_to_body_rotation;
}

}  // namespace

namespace {
std::function<std::vector<LandmarkBlob>(const uint8_t*, uint32_t, uint32_t, uint32_t)> MakeDetector(
    const StereoLandmarkVoFrontendParams& params) {
  if (params.detector_kind == LandmarkDetectorKind::kHarrisCorner) {
    return [d = HarrisCornerDetector(params.harris_detector)](const uint8_t* image, uint32_t width,
                                                                uint32_t height, uint32_t stride_px) {
      return d.Detect(image, width, height, stride_px);
    };
  }
  return [d = LandmarkBlobDetector(params.detector)](const uint8_t* image, uint32_t width, uint32_t height,
                                                       uint32_t stride_px) {
    return d.Detect(image, width, height, stride_px);
  };
}
}  // namespace

StereoLandmarkVoFrontend::StereoLandmarkVoFrontend(StereoLandmarkVoFrontendParams params)
    : params_(params),
      detect_(MakeDetector(params_)),
      stereo_matcher_(params.stereo_matcher),
      temporal_matcher_(params.temporal_matcher),
      rng_(params.rng_seed) {}

std::optional<uw::domain::MeasurementEvidence> StereoLandmarkVoFrontend::Process(
    const uw::measurement_api::CameraFrameBundle& bundle, const uw::domain::RigCalibrationSnapshot& rig) {
  ++frames_processed_;

  if (!bundle.secondary.has_value()) {
    ++frames_rejected_;
    return std::nullopt;
  }

  const auto geometry = uw::sensor_models::StereoGeometry::Resolve(
      rig, params_.left_sensor_id, params_.left_frame, params_.right_sensor_id, params_.right_frame);
  if (!geometry.valid) {
    ++frames_rejected_;
    return std::nullopt;
  }

  const auto& left_image = bundle.primary;
  const auto& right_image = *bundle.secondary;
  if (left_image.encoding() != uw::domain::ImageFrame::IMAGE_ENCODING_MONO8 ||
      right_image.encoding() != uw::domain::ImageFrame::IMAGE_ENCODING_MONO8 ||
      left_image.width() != right_image.width() || left_image.height() != right_image.height()) {
    ++frames_rejected_;
    return std::nullopt;
  }

  const auto left_blobs = detect_(
      reinterpret_cast<const uint8_t*>(left_image.pixel_data().data()), left_image.width(),
      left_image.height(), left_image.row_stride_bytes());
  const auto right_blobs = detect_(
      reinterpret_cast<const uint8_t*>(right_image.pixel_data().data()), right_image.width(),
      right_image.height(), right_image.row_stride_bytes());

  // Stereo correspondence: match left<->right blobs by appearance, then
  // triangulate each match via the same disparity->depth formula as
  // StereoOpticalDepthFrontend/BlockMatcher (right(u-d) == left(u)
  // convention). A match with non-positive or implausibly tiny disparity
  // is dropped, same reasoning as BlockMatcherParams::min_disparity.
  const auto stereo_matches = stereo_matcher_.Match(left_blobs, right_blobs);
  std::vector<TriangulatedLandmark> current_landmarks;
  current_landmarks.reserve(stereo_matches.size());
  for (const auto& match : stereo_matches) {
    const auto& left_blob = left_blobs[match.index_a];
    const auto& right_blob = right_blobs[match.index_b];
    const double disparity = left_blob.centroid_u - right_blob.centroid_u;
    if (disparity < params_.min_disparity_px) continue;

    const double depth_m = geometry.left.fx * geometry.baseline_m / disparity;
    const Eigen::Vector3d camera_point =
        geometry.left.Unproject(left_blob.centroid_u, left_blob.centroid_v, depth_m);
    current_landmarks.push_back(TriangulatedLandmark{camera_point, left_blob.patch});
  }

  std::optional<uw::domain::MeasurementEvidence> result;
  if (has_previous_ && !current_landmarks.empty()) {
    std::vector<LandmarkBlob> previous_as_blobs, current_as_blobs;
    previous_as_blobs.reserve(previous_landmarks_.size());
    for (const auto& lm : previous_landmarks_) {
      LandmarkBlob blob;
      blob.patch = lm.patch;
      previous_as_blobs.push_back(std::move(blob));
    }
    current_as_blobs.reserve(current_landmarks.size());
    for (const auto& lm : current_landmarks) {
      LandmarkBlob blob;
      blob.patch = lm.patch;
      current_as_blobs.push_back(std::move(blob));
    }

    const auto temporal_matches = temporal_matcher_.Match(previous_as_blobs, current_as_blobs);
    if (static_cast<int>(temporal_matches.size()) >= params_.min_landmarks_for_pose) {
      std::vector<Eigen::Vector3d> points_current, points_previous;
      points_current.reserve(temporal_matches.size());
      points_previous.reserve(temporal_matches.size());
      for (const auto& match : temporal_matches) {
        points_previous.push_back(previous_landmarks_[match.index_a].camera_point);
        points_current.push_back(current_landmarks[match.index_b].camera_point);
      }

      // FitRigidTransformRansac(a, b, ...) returns T with b ~= T.Apply(a)
      // over its inlier set. With a = current-frame points and b =
      // previous-frame points, T is the pose of the current camera
      // expressed in the previous camera's OPTICAL frame. RANSAC-
      // robustified (not plain FitRigidTransform) because
      // temporal_matcher_'s greedy NCC matching can produce an occasional
      // wrong correspondence that would otherwise corrupt the whole fit —
      // see this module's own header comment and the memory of running
      // this end-to-end for why that turned out to matter in practice,
      // not just in theory.
      const auto fit = FitRigidTransformRansac(points_current, points_previous, params_.ransac, rng_);
      if (fit.has_value()) {
        // Convert camera-OPTICAL-frame relative pose into the rig's BODY
        // frame — RelativePoseMeasurement.relative_pose ("from_T_to") is
        // consumed everywhere else in the pipeline (RelativePoseFactorBuilder,
        // PoseGraphProblem's keyframe poses, synth_bag_gen's ground-truth
        // generator) as a BODY-frame transform, not a camera-optical one.
        // Skipping this conjugation was a real bug found by running the
        // actual end-to-end demo (not caught by this module's own unit
        // tests, which build synthetic points directly in "the" camera
        // frame and never exercise a body/optical mismatch): translation
        // norms looked plausible (~1m/step, matching the true per-step
        // motion) but landed almost entirely on the optical z axis
        // (forward) instead of the body x/y plane the vehicle actually
        // moves in — a rotation-only error, invisible in magnitude,
        // catastrophic once composed into the pose graph. The extrinsic
        // (camera_optical -> body) is the SAME fixed rig calibration at
        // both keyframes, so conjugating by it here is exactly right:
        // body_T_camera_optical * cam_from_T_cam_to * (body_T_camera_optical)^-1.
        const auto body_from_camera_optical = BodyFromCameraOptical(rig, params_.left_frame);
        const auto body_relative = body_from_camera_optical * (*fit) * body_from_camera_optical.Inverse();

        uw::domain::RelativePoseMeasurement measurement;
        measurement.mutable_from_keyframe()->set_value(previous_keyframe_id_);
        if (left_image.header().has_observation_id()) {
          measurement.mutable_to_keyframe()->set_value(left_image.header().observation_id().value());
        }
        *measurement.mutable_relative_pose() = body_relative.ToProto();

        uw::domain::EvidenceId evidence_id;
        evidence_id.set_value("stereo_landmark_vo_" + std::to_string(next_evidence_id_++));
        std::vector<uw::domain::ObservationId> sources;
        if (left_image.header().has_observation_id()) sources.push_back(left_image.header().observation_id());
        if (right_image.header().has_observation_id()) sources.push_back(right_image.header().observation_id());

        result = uw::domain::MakeEvidence(evidence_id, sources, measurement, /*noise_scale=*/1.0,
                                          "stereo_landmark_vo_frontend_v1");
      }
    }
  }

  if (!result.has_value()) ++frames_rejected_;

  has_previous_ = true;
  previous_keyframe_id_ =
      left_image.header().has_observation_id() ? left_image.header().observation_id().value() : "";
  previous_landmarks_ = std::move(current_landmarks);

  return result;
}

uw::domain::HealthReport StereoLandmarkVoFrontend::Health() const {
  uw::domain::HealthReport report;
  report.set_component_id("stereo_landmark_vo_frontend");
  report.set_status(frames_processed_ > 0 && frames_rejected_ == frames_processed_
                        ? uw::domain::HealthReport::STATUS_SUSPECT
                        : uw::domain::HealthReport::STATUS_HEALTHY);
  return report;
}

}  // namespace uw::frontends
