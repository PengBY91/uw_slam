#pragma once

#include <string>

#include <Eigen/Core>

#include "domain/domain.hpp"
#include "sensor_models/geometry.hpp"

namespace uw::frontends {

// Looks up the rig's frame_tree edge whose child_frame is `child_frame` and
// returns its transform (Identity if not found). Same lookup pattern used
// independently by several algorithms/ callers (acoustic_optic_depth_fusion_
// frontend.cpp, acoustic_optic_associator.cpp, acoustic_optic_map_bridge.cpp,
// apps/synth_bag_gen.cpp's local FindRigEdgePose) — there is no shared
// public rig-lookup utility beyond this one, extracted here only because
// BodyFromCameraOptical below needs it and is itself now shared between
// StereoLandmarkVoFrontend and LoopClosureFrontend.
uw::sensor_models::Pose3 FindRigEdgePose(const uw::domain::RigCalibrationSnapshot& rig,
                                          const std::string& child_frame);

// The Pose3 that maps a point already expressed in a camera's OPTICAL frame
// (PinholeCamera::Project/Unproject's convention) into the rig's body frame
// — i.e. body_T_camera_optical.Apply(p_optical) == p_body. Composes the
// frame_tree edge (base_link -> camera_frame, itself in BODY convention)
// with the inverse of the fixed optical<->body axis rotation (see
// camera_model.hpp's OpticalFromBodyRotation doc comment). Extracted from
// stereo_landmark_vo_frontend.cpp (original home) so LoopClosureFrontend
// can reuse it unchanged instead of duplicating this math.
uw::sensor_models::Pose3 BodyFromCameraOptical(const uw::domain::RigCalibrationSnapshot& rig,
                                                const std::string& camera_frame);

// Transforms a 6x6 covariance of `original_pose`'s LEFT [dt(3);dtheta(3)]
// perturbation (rigid_transform_fit.hpp's convention: pose_perturbed =
// Exp(dtheta)*pose, translation += dt — decoupled, NOT the fully-coupled
// SE(3) exponential) into the covariance of (conjugator * original_pose *
// conjugator^-1)'s perturbation under the SAME conjugator applied on both
// sides. Derived by first-order expansion of that conjugation (verified
// numerically against a central-difference Jacobian during development,
// max abs error ~1e-10):
//   dt'     = R_C * dt + R_C * [w]_x * dtheta
//   dtheta' = R_C * dtheta
// where R_C is conjugator's rotation and w = original_pose.rotation *
// (conjugator.rotation^-1 * conjugator.translation).
Eigen::Matrix<double, 6, 6> TransformCovarianceForConjugation(
    const uw::sensor_models::Pose3& original_pose, const uw::sensor_models::Pose3& conjugator,
    const Eigen::Matrix<double, 6, 6>& covariance);

}  // namespace uw::frontends
