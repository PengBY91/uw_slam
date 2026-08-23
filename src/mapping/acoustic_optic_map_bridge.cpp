#include "mapping/acoustic_optic_map_bridge.hpp"

#include <optional>
#include <vector>

#include "sensor_models/camera_model.hpp"
#include "sensor_models/geometry.hpp"

namespace uw::mapping {

namespace {

const uw::domain::CameraIntrinsics* FindCamera(const uw::domain::RigCalibrationSnapshot& rig,
                                                const std::string& sensor_id) {
  for (const auto& camera : rig.cameras()) {
    if (camera.sensor_id().value() == sensor_id) return &camera;
  }
  return nullptr;
}

std::optional<uw::sensor_models::Pose3> FindEdgePose(const uw::domain::RigCalibrationSnapshot& rig,
                                                      const std::string& child_frame) {
  for (const auto& edge : rig.frame_tree()) {
    if (edge.child_frame().value() != child_frame) continue;
    return uw::sensor_models::Pose3::FromProto(edge.transform());
  }
  return std::nullopt;
}

}  // namespace

std::optional<uw::domain::MapEvidence> BuildMapEvidenceFromFusedDepth(
    const uw::domain::MeasurementEvidence& fused_evidence,
    const uw::domain::RigCalibrationSnapshot& rig, const AcousticOpticMapBridgeParams& params,
    const std::string& keyframe_id, uint64_t state_version) {
  if (!uw::domain::HasPayload<uw::domain::FusedDepthMeasurement>(fused_evidence)) {
    return std::nullopt;
  }
  const auto& fused = uw::domain::GetPayload<uw::domain::FusedDepthMeasurement>(fused_evidence);

  const auto* camera_intrinsics = FindCamera(rig, params.camera_sensor_id);
  const auto camera_pose = FindEdgePose(rig, params.camera_frame);
  if (camera_intrinsics == nullptr || !camera_pose.has_value()) return std::nullopt;
  const auto camera = uw::sensor_models::PinholeCamera::FromIntrinsics(*camera_intrinsics);

  std::vector<float> points_xyz;
  const std::size_t pixels = static_cast<std::size_t>(fused.width()) * fused.height();
  std::vector<double> uncertainty;
  for (std::size_t i = 0; i < pixels; ++i) {
    if (i >= fused.contribution_mask().size() ||
        static_cast<unsigned char>(fused.contribution_mask()[i]) == uw::domain::DEPTH_CONTRIBUTION_INVALID) {
      continue;
    }
    if (i >= fused.valid_mask().size() || fused.valid_mask()[i] == 0) continue;
    const double depth_m = fused.depth_m(static_cast<int>(i));
    // FusedDepthMeasurement.depth_m is camera-optical-frame, positive
    // z-forward range (see its field comment in
    // schemas/proto/uw/domain/measurement.proto) — unrelated to
    // PressureDepthMeasurement's world-frame positive-down convention.
    if (!(depth_m > 0.0)) continue;

    const double u = static_cast<double>(i % fused.width());
    const double v = static_cast<double>(i / fused.width());
    const Eigen::Vector3d point_optical = camera.Unproject(u, v, depth_m);
    const Eigen::Vector3d point_camera_body =
        uw::sensor_models::OpticalFromBodyRotation().transpose() * point_optical;
    const Eigen::Vector3d point_base_link = camera_pose->Apply(point_camera_body);

    points_xyz.push_back(static_cast<float>(point_base_link.x()));
    points_xyz.push_back(static_cast<float>(point_base_link.y()));
    points_xyz.push_back(static_cast<float>(point_base_link.z()));
    uncertainty.push_back(fused.variance_m2(static_cast<int>(i)));
  }

  uw::domain::MapEvidence evidence;
  evidence.mutable_evidence_id()->set_value(fused_evidence.evidence_id().value() + "_map");
  evidence.mutable_keyframe_id()->set_value(keyframe_id);
  evidence.mutable_state_version()->set_value(state_version);
  evidence.mutable_local_frame()->set_value("base_link");
  evidence.set_representation_type(uw::domain::MAP_REPRESENTATION_POINT_CLOUD);
  evidence.set_geometry_or_occupancy(
      std::string(reinterpret_cast<const char*>(points_xyz.data()), points_xyz.size() * sizeof(float)));
  for (double u : uncertainty) evidence.add_uncertainty(u);
  *evidence.mutable_source_observations() = fused_evidence.source_observations();
  evidence.set_reintegration_policy(uw::domain::MapEvidence::REINTEGRATION_POLICY_TRANSFORM_ONLY);
  return evidence;
}

namespace {

// True when pixel `i` (row-major, width x height) is usable: contribution
// mask says it isn't invalid, valid_mask agrees, and depth is a sane
// positive optical-frame range — the same three checks
// BuildMapEvidenceFromFusedDepth already applies per-pixel above, factored
// out so FuseDepthIntoSurfels can reuse them for both a pixel and its grid
// neighbors without duplicating the logic three times.
bool PixelIsUsable(const uw::domain::FusedDepthMeasurement& fused, std::size_t i) {
  if (i >= fused.contribution_mask().size() ||
      static_cast<unsigned char>(fused.contribution_mask()[i]) == uw::domain::DEPTH_CONTRIBUTION_INVALID) {
    return false;
  }
  if (i >= fused.valid_mask().size() || fused.valid_mask()[i] == 0) return false;
  return fused.depth_m(static_cast<int>(i)) > 0.0;
}

// Unprojects pixel `i` into the camera's OPTICAL frame (not yet rotated
// into base_link/world) — the frame grid-neighbor differences below need to
// be taken in, since the two tangent vectors must share one consistent
// frame before their cross product means anything.
Eigen::Vector3d UnprojectOptical(const uw::sensor_models::PinholeCamera& camera,
                                  const uw::domain::FusedDepthMeasurement& fused, std::size_t i) {
  const double u = static_cast<double>(i % fused.width());
  const double v = static_cast<double>(i / fused.width());
  return camera.Unproject(u, v, static_cast<double>(fused.depth_m(static_cast<int>(i))));
}

}  // namespace

int FuseDepthIntoSurfels(const uw::domain::MeasurementEvidence& fused_evidence,
                         const uw::domain::RigCalibrationSnapshot& rig,
                         const AcousticOpticMapBridgeParams& params, const uw::sensor_models::Pose3& pose_WB,
                         SurfelMap& surfels) {
  if (!uw::domain::HasPayload<uw::domain::FusedDepthMeasurement>(fused_evidence)) return 0;
  const auto& fused = uw::domain::GetPayload<uw::domain::FusedDepthMeasurement>(fused_evidence);

  const auto* camera_intrinsics = FindCamera(rig, params.camera_sensor_id);
  const auto camera_pose = FindEdgePose(rig, params.camera_frame);
  if (camera_intrinsics == nullptr || !camera_pose.has_value()) return 0;
  const auto camera = uw::sensor_models::PinholeCamera::FromIntrinsics(*camera_intrinsics);

  // Direction-only transform optical -> base_link -> world: rotation alone
  // (no translation), since a surface normal is a direction, not a point —
  // point transforms below additionally apply each Pose3's translation via
  // Apply(), normals must not.
  const Eigen::Matrix3d optical_to_world_rotation =
      pose_WB.rotation.toRotationMatrix() * camera_pose->rotation.toRotationMatrix() *
      uw::sensor_models::OpticalFromBodyRotation().transpose();

  const std::size_t width = fused.width();
  const std::size_t pixels = static_cast<std::size_t>(width) * fused.height();
  int added = 0;
  for (std::size_t i = 0; i < pixels; ++i) {
    if (!PixelIsUsable(fused, i)) continue;
    const double variance_m2 = fused.variance_m2(static_cast<int>(i));
    if (!(variance_m2 > 0.0)) continue;  // fail-closed on degenerate/unset variance, not divide-by-zero
    const double confidence = 1.0 / variance_m2;

    const Eigen::Vector3d point_optical = UnprojectOptical(camera, fused, i);
    const Eigen::Vector3d point_base_link =
        uw::sensor_models::OpticalFromBodyRotation().transpose() * point_optical;
    const Eigen::Vector3d point_world = pose_WB.Apply(camera_pose->Apply(point_base_link));

    // Grid-neighbor normal estimate: needs a right (u+1) and a down (v+1)
    // neighbor, both usable. (P_down - P) x (P_right - P), in that order,
    // faces back toward the camera for a frontal surface (optical +Z is
    // "into the scene," so this cross-product order yields -Z there) —
    // this is a sensor-facing convention, not viewpoint-canonicalized
    // against multiple observers of the same surfel; good enough for a
    // first local estimate, not attempted to be more than that here.
    std::optional<Eigen::Vector3d> normal_world;
    const std::size_t u = i % width;
    const std::size_t v = i / width;
    const std::size_t right = i + 1;
    const std::size_t down = i + width;
    if (u + 1 < width && v + 1 < fused.height() && PixelIsUsable(fused, right) && PixelIsUsable(fused, down)) {
      const Eigen::Vector3d tangent_right = UnprojectOptical(camera, fused, right) - point_optical;
      const Eigen::Vector3d tangent_down = UnprojectOptical(camera, fused, down) - point_optical;
      const Eigen::Vector3d normal_optical = tangent_down.cross(tangent_right);
      const double norm = normal_optical.norm();
      if (norm > 1e-12) {  // guards a degenerate (near-collinear/duplicate-point) neighborhood
        normal_world = optical_to_world_rotation * (normal_optical / norm);
      }
    }

    if (normal_world.has_value()) {
      surfels.AddPointWithNormal(point_world, *normal_world, confidence);
    } else {
      surfels.AddPoint(point_world, confidence);
    }
    ++added;
  }
  return added;
}

}  // namespace uw::mapping
