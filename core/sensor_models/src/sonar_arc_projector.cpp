#include "uw/sensor_models/sonar_arc_projector.hpp"

#include <cmath>

namespace uw::sensor_models {

std::vector<ArcCandidate> ProjectSonarArcToCamera(double range_m, double bearing_rad,
                                                   double elevation_aperture_rad,
                                                   const Pose3& camera_T_sonar,
                                                   const PinholeCamera& camera, int num_samples) {
  std::vector<ArcCandidate> candidates;
  if (num_samples <= 0) return candidates;

  for (int i = 0; i < num_samples; ++i) {
    const double phi = num_samples > 1
                            ? -elevation_aperture_rad / 2.0 +
                                  elevation_aperture_rad * static_cast<double>(i) / (num_samples - 1)
                            : 0.0;
    const Eigen::Vector3d point_sonar(range_m * std::cos(phi) * std::cos(bearing_rad),
                                      range_m * std::cos(phi) * std::sin(bearing_rad),
                                      range_m * std::sin(phi));
    const Eigen::Vector3d point_camera_body = camera_T_sonar.Apply(point_sonar);
    const Eigen::Vector3d point_optical = OpticalFromBodyRotation() * point_camera_body;
    if (point_optical.z() <= 0.0) continue;

    const Eigen::Vector2d pixel = camera.Project(point_optical);
    if (pixel.x() < 0.0 || pixel.x() >= static_cast<double>(camera.width) || pixel.y() < 0.0 ||
        pixel.y() >= static_cast<double>(camera.height)) {
      continue;
    }
    candidates.push_back(ArcCandidate{phi, point_sonar, pixel.x(), pixel.y()});
  }
  return candidates;
}

SonarFrameObservation UnprojectPixelToSonarRangeBearing(double pixel_u, double pixel_v,
                                                         double depth_m,
                                                         const Pose3& camera_T_sonar,
                                                         const PinholeCamera& camera) {
  const Eigen::Vector3d point_optical = camera.Unproject(pixel_u, pixel_v, depth_m);
  const Eigen::Vector3d point_camera_body = OpticalFromBodyRotation().transpose() * point_optical;
  const Eigen::Vector3d point_sonar = camera_T_sonar.Inverse().Apply(point_camera_body);

  SonarFrameObservation observation;
  observation.range_m = point_sonar.norm();
  observation.bearing_rad = std::atan2(point_sonar.y(), point_sonar.x());
  return observation;
}

}  // namespace uw::sensor_models
