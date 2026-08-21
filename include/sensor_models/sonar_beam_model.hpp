#pragma once

#include <vector>

#include <Eigen/Core>

namespace uw::sensor_models {

// A 2D FLS/imaging sonar return gives range + bearing; elevation within the
// vertical aperture is NOT observed. This returns the point on the z=0
// plane of the sonar frame — the only thing a range-bearing return actually
// tells you (used by include/factor_builders/sonar_range_residual.hpp).
Eigen::Vector3d SonarRangeBearingToPlanePoint(double range_m, double bearing_rad);

// Expands a range-bearing return across the sonar's vertical aperture into
// a fan of candidate 3D points. This is a MAPPING-ONLY dense-evidence
// generator (never feeds a pose factor) — ported from
// sonar_camera_reconstruction's imaging_sonar.py get_extended_coordinates.
std::vector<Eigen::Vector3d> ExpandElevationFan(double range_m, double bearing_rad,
                                                 double elevation_aperture_rad,
                                                 int num_elevation_samples);

}  // namespace uw::sensor_models
