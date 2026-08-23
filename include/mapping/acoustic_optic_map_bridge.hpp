// Converts FusedDepthMeasurement into local-frame MapEvidence. Every valid pixel whose
// contribution_mask != DEPTH_CONTRIBUTION_INVALID is unprojected and
// expressed in BASE_LINK frame — not camera-optical, not world — because
// that is the frame include/mapping/submap_manager.hpp composes with a keyframe's pose_WB via
// pose_WB.Apply(local). Points are unprojected via the same fixed
// body/optical rotation (`uw::sensor_models::OpticalFromBodyRotation`) and
// the same rig-derived camera extrinsic as the other cross-modal geometry.
#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "domain/domain.hpp"
#include "mapping/surfel_map.hpp"
#include "sensor_models/geometry.hpp"

namespace uw::mapping {

struct AcousticOpticMapBridgeParams {
  std::string camera_sensor_id = "camera_left";
  std::string camera_frame = "camera_left_link";
};

// Returns nullopt if fused_evidence has no FusedDepthMeasurement payload,
// or if the rig cannot resolve the named camera's intrinsics/extrinsic
// (fail-closed, matching the other cross-modal calibration checks).
std::optional<uw::domain::MapEvidence> BuildMapEvidenceFromFusedDepth(
    const uw::domain::MeasurementEvidence& fused_evidence,
    const uw::domain::RigCalibrationSnapshot& rig, const AcousticOpticMapBridgeParams& params,
    const std::string& keyframe_id, uint64_t state_version);

// P3 roadmap item 2 ("visual-only 和 sonar-grounded 两条局部几何路径"): routes
// a FusedDepthMeasurement's per-pixel data into `surfels`, using
// DepthContribution (measurement.proto) to weight each pixel by how it was
// produced — confidence = 1/variance_m2 (Surfel::confidence's documented
// convention), and DEPTH_CONTRIBUTION_ACOUSTIC_OPTIC pixels are only ever
// marked as such by AcousticOpticDepthFusionFrontend when their posterior
// variance is PROVABLY lower than the optical prior (see
// acoustic_optic_depth_fusion_frontend.cpp's min_variance_improvement_fraction
// gate) — so a sonar-grounded observation naturally dominates a
// confidence-weighted merge against an optical-only observation of the same
// physical point, with no special-casing needed in SurfelMap itself. This is
// one unified path, not two separate entry points: the two "local geometry
// paths" the roadmap names differ only in which confidence value a pixel
// carries, which SurfelMap's existing merge logic already handles generically.
//
// Also estimates a per-pixel surface normal from grid neighbors (fused's
// row-major width x height layout: unprojects the pixel's right and down
// neighbors, when both are valid, and takes their cross product) and calls
// SurfelMap::AddKeyframeObservationWithNormal instead of
// AddKeyframeObservation when one is available — this is the "local
// geometry" half: without it, every surfel's normal stays unknown (Zero())
// forever, since nothing else in this codebase computes one.
//
// P3 roadmap item 4 (pose-correction reintegration, see
// SurfelMap::ReintegrateKeyframe's doc comment): points/normals are
// unprojected into BASE_LINK frame (camera extrinsic already applied, same
// "local" convention BuildMapEvidenceFromFusedDepth uses) and handed to
// SurfelMap via AddKeyframeObservation(WithNormal) under `keyframe_id`, with
// `pose_WB` as the local-to-world transform — NOT the raw world-frame
// AddPoint/AddPointWithNormal path this function used before. This means a
// caller that later corrects `keyframe_id`'s pose can call
// SurfelMap::ReintegrateKeyframe directly to get a correctly re-fused
// result, instead of this fusion being silently stale forever (this file's
// own prior note — "SurfelMap has no deferred-reintegration concept yet" —
// is the gap this closes).
//
// Returns the number of points actually added to `surfels` (0 if
// fused_evidence has no FusedDepthMeasurement payload, or the rig cannot
// resolve the named camera — same fail-closed contract as
// BuildMapEvidenceFromFusedDepth).
int FuseDepthIntoSurfels(const uw::domain::MeasurementEvidence& fused_evidence,
                         const uw::domain::RigCalibrationSnapshot& rig,
                         const AcousticOpticMapBridgeParams& params, const std::string& keyframe_id,
                         const uw::sensor_models::Pose3& pose_WB, SurfelMap& surfels);

}  // namespace uw::mapping
