#include "frontends/loop_closure_frontend.hpp"

#include <algorithm>
#include <utility>

#include <Eigen/Geometry>

#include "frontends/camera_body_conjugation.hpp"
#include "sensor_models/camera_model.hpp"

namespace uw::frontends {

LoopClosureFrontend::LoopClosureFrontend(LoopClosureFrontendParams params)
    : params_(params),
      detector_(params_.detector),
      stereo_matcher_(params.stereo_matcher),
      revisit_matcher_(params.revisit_matcher),
      rng_(params.rng_seed) {}

std::vector<uw::domain::MeasurementEvidence> LoopClosureFrontend::Process(
    const uw::measurement_api::CameraFrameBundle& bundle, const uw::domain::RigCalibrationSnapshot& rig,
    const std::string& current_keyframe_id, const uw::sensor_models::Pose3& current_pose_estimate) {
  ++frames_processed_;
  std::vector<uw::domain::MeasurementEvidence> result;

  if (!bundle.secondary.has_value()) return result;

  const auto geometry = uw::sensor_models::StereoGeometry::Resolve(
      rig, params_.left_sensor_id, params_.left_frame, params_.right_sensor_id, params_.right_frame);
  if (!geometry.valid) return result;

  const auto& left_image = bundle.primary;
  const auto& right_image = *bundle.secondary;
  if (left_image.encoding() != uw::domain::ImageFrame::IMAGE_ENCODING_MONO8 ||
      right_image.encoding() != uw::domain::ImageFrame::IMAGE_ENCODING_MONO8 ||
      left_image.width() != right_image.width() || left_image.height() != right_image.height()) {
    return result;
  }
  // Same raw-vs-rectified contract as StereoLandmarkVoFrontend — see that
  // file's own comment on why an unrectified pair must never reach here.
  if (!left_image.is_rectified() || !right_image.is_rectified() ||
      left_image.header().sensor_frame().value() != params_.left_frame ||
      right_image.header().sensor_frame().value() != params_.right_frame ||
      left_image.width() != geometry.left.width || left_image.height() != geometry.left.height) {
    return result;
  }

  const auto left_corners = detector_.Detect(
      reinterpret_cast<const uint8_t*>(left_image.pixel_data().data()), left_image.width(),
      left_image.height(), left_image.row_stride_bytes());
  const auto right_corners = detector_.Detect(
      reinterpret_cast<const uint8_t*>(right_image.pixel_data().data()), right_image.width(),
      right_image.height(), right_image.row_stride_bytes());

  const auto stereo_matches = stereo_matcher_.Match(left_corners, right_corners);
  std::vector<TriangulatedLandmark> current_landmarks;
  current_landmarks.reserve(stereo_matches.size());
  for (const auto& match : stereo_matches) {
    const auto& left_corner = left_corners[match.index_a];
    const auto& right_corner = right_corners[match.index_b];
    const double disparity = left_corner.centroid_u - right_corner.centroid_u;
    if (disparity < params_.min_disparity_px) continue;

    const double depth_m = geometry.left.fx * geometry.baseline_m / disparity;
    const Eigen::Vector3d camera_point =
        geometry.left.Unproject(left_corner.centroid_u, left_corner.centroid_v, depth_m);
    current_landmarks.push_back(TriangulatedLandmark{camera_point, left_corner.patch});
  }

  // Candidate retrieval: pose-proximity, brute-force O(N) scan (see class
  // header comment for why this is adequate at v1 problem sizes and what
  // its documented scope limit is), sorted nearest-first so
  // max_loop_edges_per_keyframe keeps the strongest candidates.
  const int current_insertion_order = static_cast<int>(archive_.size());
  std::vector<std::pair<double, std::size_t>> candidates;
  for (std::size_t i = 0; i < archive_.size(); ++i) {
    const auto& entry = archive_[i];
    if (current_insertion_order - entry.insertion_order < params_.min_keyframe_index_gap) continue;
    const double distance_m = (entry.position_W - current_pose_estimate.translation).norm();
    if (distance_m > params_.candidate_search_radius_m) continue;
    candidates.emplace_back(distance_m, i);
  }
  std::sort(candidates.begin(), candidates.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });

  if (static_cast<int>(current_landmarks.size()) >= params_.min_landmarks_for_pose) {
    for (const auto& [distance_m, archive_index] : candidates) {
      if (static_cast<int>(result.size()) >= params_.max_loop_edges_per_keyframe) break;
      const auto& candidate_kf = archive_[archive_index];

      std::vector<LandmarkBlob> candidate_as_blobs, current_as_blobs;
      candidate_as_blobs.reserve(candidate_kf.landmarks.size());
      for (const auto& landmark : candidate_kf.landmarks) {
        LandmarkBlob blob;
        blob.patch = landmark.patch;
        candidate_as_blobs.push_back(std::move(blob));
      }
      current_as_blobs.reserve(current_landmarks.size());
      for (const auto& landmark : current_landmarks) {
        LandmarkBlob blob;
        blob.patch = landmark.patch;
        current_as_blobs.push_back(std::move(blob));
      }

      const auto matches = revisit_matcher_.Match(candidate_as_blobs, current_as_blobs);
      if (static_cast<int>(matches.size()) < params_.min_landmarks_for_pose) {
        ++rejected_candidates_;
        continue;
      }

      std::vector<Eigen::Vector3d> points_current, points_candidate;
      points_current.reserve(matches.size());
      points_candidate.reserve(matches.size());
      for (const auto& match : matches) {
        points_candidate.push_back(candidate_kf.landmarks[match.index_a].camera_point);
        points_current.push_back(current_landmarks[match.index_b].camera_point);
      }

      // FitRigidTransformRansac(a, b, ...) returns T with b ~= T.Apply(a)
      // over its inlier set. With a = current-frame points and b =
      // candidate(archived)-frame points, T is the pose of the current
      // camera expressed in the candidate camera's OPTICAL frame — same
      // convention StereoLandmarkVoFrontend uses for its temporal fit.
      const auto fit = FitRigidTransformRansac(points_current, points_candidate, params_.ransac, rng_,
                                               params_.covariance_estimation);
      if (!fit.has_value()) {
        ++rejected_candidates_;
        continue;
      }

      // Convert camera-OPTICAL-frame relative pose into the rig's BODY
      // frame — RelativePoseMeasurement.relative_pose is consumed
      // everywhere downstream as a BODY-frame transform (see
      // camera_body_conjugation.hpp / StereoLandmarkVoFrontend's own
      // comment on why skipping this conjugation is a real, previously-hit
      // bug, not a hypothetical one).
      const auto body_from_camera_optical = BodyFromCameraOptical(rig, params_.left_frame);
      const auto body_relative =
          body_from_camera_optical * fit->pose * body_from_camera_optical.Inverse();

      // Sanity gate on the RECOVERED relative pose: RANSAC's own inlier-
      // count/rmse gates catch a badly-inconsistent correspondence set, but
      // not a geometrically-plausible-looking match to the WRONG place —
      // an implausibly large "revisit" jump is rejected outright rather
      // than handed to the solver's Huber down-weighting to sort out.
      const double rotation_angle_rad = Eigen::AngleAxisd(body_relative.rotation).angle();
      if (body_relative.translation.norm() > params_.max_accepted_translation_m ||
          rotation_angle_rad > params_.max_accepted_rotation_rad) {
        ++rejected_candidates_;
        continue;
      }

      const auto body_covariance =
          TransformCovarianceForConjugation(fit->pose, body_from_camera_optical, fit->covariance);

      uw::domain::RelativePoseMeasurement measurement;
      measurement.mutable_from_keyframe()->set_value(candidate_kf.keyframe_id);
      if (!current_keyframe_id.empty()) {
        measurement.mutable_to_keyframe()->set_value(current_keyframe_id);
      }
      *measurement.mutable_relative_pose() = body_relative.ToProto();
      for (int row = 0; row < 6; ++row) {
        for (int col = 0; col < 6; ++col) {
          measurement.add_covariance_6x6_row_major(body_covariance(row, col));
        }
      }

      uw::domain::EvidenceId evidence_id;
      evidence_id.set_value("loop_closure_" + std::to_string(next_evidence_id_++));
      std::vector<uw::domain::ObservationId> sources;
      if (left_image.header().has_observation_id()) sources.push_back(left_image.header().observation_id());
      if (right_image.header().has_observation_id()) sources.push_back(right_image.header().observation_id());

      auto evidence = uw::domain::MakeEvidence(evidence_id, sources, measurement, /*noise_scale=*/1.0,
                                               "loop_closure_frontend_v1");
      auto& quality_features = *evidence.mutable_quality_features();
      quality_features["correspondence_count"] = static_cast<double>(fit->correspondence_count);
      quality_features["inlier_count"] = static_cast<double>(fit->inlier_indices.size());
      quality_features["inlier_ratio"] = fit->inlier_ratio;
      quality_features["inlier_rmse_m"] = fit->inlier_rmse_m;
      quality_features["candidate_distance_m"] = distance_m;

      result.push_back(std::move(evidence));
      ++accepted_loops_;
    }
  }

  // Archived unconditionally — even a keyframe that found no loop this
  // call, or had too few landmarks to search with, still needs to be
  // available as a CANDIDATE for some later keyframe's search.
  archive_.push_back(ArchivedKeyframe{current_keyframe_id, current_pose_estimate.translation,
                                      std::move(current_landmarks), current_insertion_order});

  return result;
}

uw::domain::HealthReport LoopClosureFrontend::Health() const {
  uw::domain::HealthReport report;
  report.set_component_id("loop_closure_frontend");
  // Finding zero loop candidates is the ordinary/expected case (most
  // keyframes never revisit anywhere) — unlike VisualOdometryFrontend,
  // that is not a tracking-loss signal, so there is no SUSPECT/UNAVAILABLE
  // state to report here.
  report.set_status(uw::domain::HealthReport::STATUS_HEALTHY);
  return report;
}

}  // namespace uw::frontends
