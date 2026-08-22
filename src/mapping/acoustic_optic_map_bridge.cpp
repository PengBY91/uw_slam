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

}  // namespace uw::mapping
