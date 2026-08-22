#pragma once

#include <vector>

#include <Eigen/Core>

#include "sensor_models/camera_model.hpp"
#include "sensor_models/geometry.hpp"

namespace uw::sensor_models {

struct ArcCandidate {
  double phi_rad = 0.0;                          // elevation angle sampled within the aperture
  Eigen::Vector3d point_sonar = Eigen::Vector3d::Zero();  // 3D point in the sonar's frame
  double pixel_u = 0.0;
  double pixel_v = 0.0;
};

// Samples the ideal FLS arc p_S(phi) = rho * [cos(phi)cos(theta), cos(phi)sin(theta), sin(phi)],
// phi in [-aperture/2, +aperture/2], transforms each sample through
// `camera_T_sonar` (a rig-derived Pose3 in this platform's BODY convention — e.g.
// camera_left_edge.Inverse() * sonar_edge, both straight from RigCalibrationSnapshot's
// frame_tree) into the target camera's frame, applies the fixed body->optical rotation
// (OpticalFromBodyRotation(), camera_model.hpp), then projects with `camera`. Keeps only
// samples with positive optical-frame depth that land inside [0,width)x[0,height) — does not
// clamp or extrapolate out-of-image samples.
std::vector<ArcCandidate> ProjectSonarArcToCamera(double range_m, double bearing_rad,
                                                   double elevation_aperture_rad,
                                                   const Pose3& camera_T_sonar,
                                                   const PinholeCamera& camera, int num_samples);

struct SonarFrameObservation {
  double range_m = 0.0;
  double bearing_rad = 0.0;
};

// Inverse direction: given a pixel + a depth already resolved at that pixel (e.g. from an
// OpticalDepthPriorMeasurement), express the corresponding 3D point in the sonar's frame and
// reduce to range/bearing. Elevation is intentionally discarded on the way out — matches
// SonarRangeBearing's own contract (architecture invariant: FLS is 2D range-bearing only).
SonarFrameObservation UnprojectPixelToSonarRangeBearing(double pixel_u, double pixel_v,
                                                         double depth_m,
                                                         const Pose3& camera_T_sonar,
                                                         const PinholeCamera& camera);

}  // namespace uw::sensor_models
