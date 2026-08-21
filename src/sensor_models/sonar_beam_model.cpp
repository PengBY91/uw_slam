#include "sensor_models/sonar_beam_model.hpp"

#include <cmath>

namespace uw::sensor_models {

Eigen::Vector3d SonarRangeBearingToPlanePoint(double range_m, double bearing_rad) {
  return Eigen::Vector3d(range_m * std::cos(bearing_rad), range_m * std::sin(bearing_rad), 0.0);
}

std::vector<Eigen::Vector3d> ExpandElevationFan(double range_m, double bearing_rad,
                                                 double elevation_aperture_rad,
                                                 int num_elevation_samples) {
  std::vector<Eigen::Vector3d> points;
  if (num_elevation_samples <= 0) return points;
  points.reserve(static_cast<size_t>(num_elevation_samples));

  const double phi_max = elevation_aperture_rad / 2.0;
  const double cos_theta = std::cos(bearing_rad);
  const double sin_theta = std::sin(bearing_rad);

  for (int i = 0; i < num_elevation_samples; ++i) {
    const double t = num_elevation_samples == 1
                          ? 0.0
                          : static_cast<double>(i) / (num_elevation_samples - 1);
    const double phi = phi_max - t * 2.0 * phi_max;  // phi_max down to -phi_max
    const double cos_phi = std::cos(phi);
    const double sin_phi = std::sin(phi);
    points.emplace_back(range_m * cos_phi * cos_theta, range_m * cos_phi * sin_theta,
                         range_m * sin_phi);
  }
  return points;
}

}  // namespace uw::sensor_models
