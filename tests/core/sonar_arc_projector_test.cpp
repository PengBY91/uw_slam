#include <cmath>

#include <gtest/gtest.h>

#include "sensor_models/sonar_arc_projector.hpp"

namespace {

uw::sensor_models::PinholeCamera MakeCamera() {
  uw::sensor_models::PinholeCamera camera;
  camera.fx = 100.0;
  camera.fy = 100.0;
  camera.cx = 10.0;
  camera.cy = 5.0;
  camera.width = 20;
  camera.height = 10;
  return camera;
}

}  // namespace

TEST(SonarArcProjector, ProjectsBoresightPointToImageCenter) {
  // range=5m straight ahead (bearing=0), zero-aperture single sample
  // (phi=0) — with camera co-located and co-oriented with the sonar in
  // BODY convention, the fixed body->optical rotation puts this point on
  // the camera's optical axis, landing exactly at (cx, cy).
  const auto candidates = uw::sensor_models::ProjectSonarArcToCamera(
      /*range_m=*/5.0, /*bearing_rad=*/0.0, /*elevation_aperture_rad=*/0.0,
      uw::sensor_models::Pose3::Identity(), MakeCamera(), /*num_samples=*/1);
  ASSERT_EQ(candidates.size(), 1u);
  EXPECT_NEAR(candidates[0].pixel_u, 10.0, 1e-9);
  EXPECT_NEAR(candidates[0].pixel_v, 5.0, 1e-9);
  EXPECT_NEAR(candidates[0].point_sonar.x(), 5.0, 1e-9);
  EXPECT_NEAR(candidates[0].point_sonar.y(), 0.0, 1e-9);
  EXPECT_NEAR(candidates[0].point_sonar.z(), 0.0, 1e-9);
}

TEST(SonarArcProjector, UnprojectIsExactInverseOfProjectAtBoresight) {
  const auto camera = MakeCamera();
  const auto candidates = uw::sensor_models::ProjectSonarArcToCamera(
      5.0, 0.0, 0.0, uw::sensor_models::Pose3::Identity(), camera, 1);
  ASSERT_EQ(candidates.size(), 1u);

  const auto observed = uw::sensor_models::UnprojectPixelToSonarRangeBearing(
      candidates[0].pixel_u, candidates[0].pixel_v, /*depth_m=*/5.0,
      uw::sensor_models::Pose3::Identity(), camera);
  EXPECT_NEAR(observed.range_m, 5.0, 1e-9);
  EXPECT_NEAR(observed.bearing_rad, 0.0, 1e-9);
}

TEST(SonarArcProjector, RoundTripsForNonZeroBearing) {
  const auto camera = MakeCamera();
  const double range = 4.0;
  const double bearing = 0.05;  // small enough to stay inside the 20x10 test image
  const auto candidates = uw::sensor_models::ProjectSonarArcToCamera(
      range, bearing, 0.0, uw::sensor_models::Pose3::Identity(), camera, 1);
  ASSERT_EQ(candidates.size(), 1u);

  // depth_m must be the camera OPTICAL-frame z (point_optical.z()), which only equals the
  // sonar range at boresight (bearing=0) — off-boresight it's range*cos(bearing) (phi=0 here,
  // single sample) since Unproject/Project work in optical convention, not sonar-frame range.
  const double depth_m = range * std::cos(bearing);
  const auto observed = uw::sensor_models::UnprojectPixelToSonarRangeBearing(
      candidates[0].pixel_u, candidates[0].pixel_v, depth_m, uw::sensor_models::Pose3::Identity(),
      camera);
  EXPECT_NEAR(observed.range_m, range, 1e-9);
  EXPECT_NEAR(observed.bearing_rad, bearing, 1e-9);
}

TEST(SonarArcProjector, SamplesFullApertureAndSkipsOutOfImageSamples) {
  // A wide aperture pushes some samples off-camera (small image, cx/cy
  // near center) — the projector must drop those, not clamp or fabricate.
  const auto candidates = uw::sensor_models::ProjectSonarArcToCamera(
      2.0, 0.0, /*elevation_aperture_rad=*/2.0, uw::sensor_models::Pose3::Identity(),
      MakeCamera(), /*num_samples=*/9);
  EXPECT_LT(candidates.size(), 9u);
  for (const auto& c : candidates) {
    EXPECT_GE(c.pixel_u, 0.0);
    EXPECT_LT(c.pixel_u, 20.0);
    EXPECT_GE(c.pixel_v, 0.0);
    EXPECT_LT(c.pixel_v, 10.0);
  }
}
