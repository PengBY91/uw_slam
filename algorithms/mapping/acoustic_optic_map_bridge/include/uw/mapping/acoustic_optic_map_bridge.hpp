// Converts plan 4's FusedDepthMeasurement into local-frame MapEvidence
// (design spec section 7/16, "mapping handoff"). Every valid pixel whose
// contribution_mask != DEPTH_CONTRIBUTION_INVALID is unprojected and
// expressed in BASE_LINK frame — not camera-optical, not world — because
// that is the frame algorithms/mapping/submap_manager (pre-existing, NOT
// modified by this plan) composes with a keyframe's pose_WB via
// pose_WB.Apply(local). Points are unprojected via the same fixed
// body/optical rotation used since plan 3
// (uw::sensor_models::OpticalFromBodyRotation) and the same rig-derived
// camera extrinsic used throughout plans 2-4 — no new geometry primitive.
#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "uw/domain/domain.hpp"

namespace uw::mapping {

struct AcousticOpticMapBridgeParams {
  std::string camera_sensor_id = "camera_left";
  std::string camera_frame = "camera_left_link";
};

// Returns nullopt if fused_evidence has no FusedDepthMeasurement payload,
// or if the rig cannot resolve the named camera's intrinsics/extrinsic
// (fail-closed, matching plan 3/4's CALIBRATION-rejection precedent).
std::optional<uw::domain::MapEvidence> BuildMapEvidenceFromFusedDepth(
    const uw::domain::MeasurementEvidence& fused_evidence,
    const uw::domain::RigCalibrationSnapshot& rig, const AcousticOpticMapBridgeParams& params,
    const std::string& keyframe_id, uint64_t state_version);

}  // namespace uw::mapping
