#include <cmath>
#include <random>

#include <gtest/gtest.h>

#include "sensor_models/ned_conversion.hpp"

using uw::sensor_models::BodyDeltaFromFrd;
using uw::sensor_models::BodyDeltaToFrd;
using uw::sensor_models::BodyVectorToFrd;
using uw::sensor_models::FrdFromFluRotation;
using uw::sensor_models::Matrix6d;
using uw::sensor_models::NedFromEnuRotation;
using uw::sensor_models::Pose3;
using uw::sensor_models::RotateCovariance6;
using uw::sensor_models::RotationVector;
using uw::sensor_models::WorldPoseFromNed;
using uw::sensor_models::WorldPoseToNed;
using uw::sensor_models::WorldVectorToNed;

namespace {

constexpr double kTol = 1e-12;

void ExpectVec(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
  EXPECT_NEAR(a.x(), b.x(), kTol);
  EXPECT_NEAR(a.y(), b.y(), kTol);
  EXPECT_NEAR(a.z(), b.z(), kTol);
}

void ExpectPose(const Pose3& a, const Pose3& b) {
  ExpectVec(a.translation, b.translation);
  // q and -q are the same rotation.
  const double dot = std::abs(a.rotation.coeffs().dot(b.rotation.coeffs()));
  EXPECT_NEAR(dot, 1.0, 1e-12);
}

Pose3 RandomPose(std::mt19937_64& rng) {
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  Pose3 pose;
  pose.translation = Eigen::Vector3d(u(rng), u(rng), u(rng)) * 5.0;
  pose.rotation = Eigen::Quaterniond(u(rng), u(rng), u(rng), u(rng)).normalized();
  return pose;
}

}  // namespace

TEST(NedConversion, BasisChangesAreProperRotationsAndInvolutions) {
  const Eigen::Matrix3d ne = NedFromEnuRotation();
  const Eigen::Matrix3d fl = FrdFromFluRotation();
  EXPECT_NEAR(ne.determinant(), 1.0, kTol);
  EXPECT_NEAR(fl.determinant(), 1.0, kTol);
  EXPECT_TRUE((ne * ne).isApprox(Eigen::Matrix3d::Identity(), kTol));
  EXPECT_TRUE((fl * fl).isApprox(Eigen::Matrix3d::Identity(), kTol));
}

TEST(NedConversion, WorldAxesMapEastNorthUpToNorthEastDown) {
  ExpectVec(WorldVectorToNed(Eigen::Vector3d(1, 0, 0)), Eigen::Vector3d(0, 1, 0));   // east  -> E component
  ExpectVec(WorldVectorToNed(Eigen::Vector3d(0, 1, 0)), Eigen::Vector3d(1, 0, 0));   // north -> N component
  ExpectVec(WorldVectorToNed(Eigen::Vector3d(0, 0, 1)), Eigen::Vector3d(0, 0, -1));  // up    -> -D
}

TEST(NedConversion, BodyAxesMapForwardLeftUpToForwardRightDown) {
  ExpectVec(BodyVectorToFrd(Eigen::Vector3d(1, 0, 0)), Eigen::Vector3d(1, 0, 0));
  ExpectVec(BodyVectorToFrd(Eigen::Vector3d(0, 1, 0)), Eigen::Vector3d(0, -1, 0));
  ExpectVec(BodyVectorToFrd(Eigen::Vector3d(0, 0, 1)), Eigen::Vector3d(0, 0, -1));
}

TEST(NedConversion, DepthBelowSurfaceBecomesPositiveDown) {
  Pose3 T;
  T.translation = Eigen::Vector3d(2.0, 3.0, -4.0);  // 4 m below the surface, z-up
  const Pose3 ned = WorldPoseToNed(T);
  ExpectVec(ned.translation, Eigen::Vector3d(3.0, 2.0, 4.0));
}

TEST(NedConversion, BodyForwardAxisIsConsistentBetweenConventions) {
  // Whatever the body attitude, the world direction of "forward" must be the
  // same physical direction in both conventions.
  std::mt19937_64 rng(7);
  for (int i = 0; i < 20; ++i) {
    const Pose3 T = RandomPose(rng);
    const Pose3 ned = WorldPoseToNed(T);
    const Eigen::Vector3d forward_enu = T.rotation * Eigen::Vector3d(1, 0, 0);
    const Eigen::Vector3d forward_ned = ned.rotation * Eigen::Vector3d(1, 0, 0);
    ExpectVec(forward_ned, WorldVectorToNed(forward_enu));
    const Eigen::Vector3d up_enu = T.rotation * Eigen::Vector3d(0, 0, 1);
    const Eigen::Vector3d down_ned = ned.rotation * Eigen::Vector3d(0, 0, 1);
    ExpectVec(down_ned, -WorldVectorToNed(up_enu));
  }
}

TEST(NedConversion, HeadingEastInEnuIsYawPlusNinetyInNed) {
  // Identity attitude in ENU/FLU means forward == east. In NED, forward ==
  // east is a +90 deg yaw about D (clockwise from above).
  const Pose3 ned = WorldPoseToNed(Pose3::Identity());
  const Eigen::Vector3d rv = RotationVector(ned.rotation);
  EXPECT_NEAR(rv.x(), 0.0, kTol);
  EXPECT_NEAR(rv.y(), 0.0, kTol);
  EXPECT_NEAR(rv.z(), M_PI / 2.0, kTol);

  // Heading north in ENU (yaw +90 about up) is yaw 0 in NED.
  Pose3 north;
  north.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
  const Pose3 ned_north = WorldPoseToNed(north);
  EXPECT_NEAR(RotationVector(ned_north.rotation).norm(), 0.0, 1e-12);
}

TEST(NedConversion, WorldPoseRoundTrips) {
  std::mt19937_64 rng(11);
  for (int i = 0; i < 50; ++i) {
    const Pose3 T = RandomPose(rng);
    ExpectPose(WorldPoseFromNed(WorldPoseToNed(T)), T);
  }
}

TEST(NedConversion, BodyDeltaRoundTripsAndFlipsLateralSign) {
  std::mt19937_64 rng(13);
  for (int i = 0; i < 50; ++i) {
    const Pose3 d = RandomPose(rng);
    ExpectPose(BodyDeltaFromFrd(BodyDeltaToFrd(d)), d);
  }
  Pose3 left_step;
  left_step.translation = Eigen::Vector3d(1.0, 0.5, 0.2);  // forward 1, left 0.5, up 0.2
  const Pose3 frd = BodyDeltaToFrd(left_step);
  ExpectVec(frd.translation, Eigen::Vector3d(1.0, -0.5, -0.2));

  // A left (counter-clockwise from above) yaw of +30 deg in FLU is a -30 deg
  // yaw about the down axis in FRD.
  Pose3 yaw_left;
  yaw_left.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 6.0, Eigen::Vector3d::UnitZ()));
  const Eigen::Vector3d rv = RotationVector(BodyDeltaToFrd(yaw_left).rotation);
  EXPECT_NEAR(rv.x(), 0.0, kTol);
  EXPECT_NEAR(rv.y(), 0.0, kTol);
  EXPECT_NEAR(rv.z(), -M_PI / 6.0, kTol);
}

TEST(NedConversion, BodyDeltaConversionIsConsistentWithWorldConversion) {
  // delta_frd computed from converted world poses must equal the direct
  // body-delta conversion -- this is the identity the MAVLink adapter relies
  // on when it sends VISION_POSITION_DELTA between two SLAM keyframes.
  std::mt19937_64 rng(17);
  for (int i = 0; i < 20; ++i) {
    const Pose3 Ta = RandomPose(rng);
    const Pose3 Tb = RandomPose(rng);
    const Pose3 delta_flu = Ta.Inverse() * Tb;
    const Pose3 delta_frd_direct = BodyDeltaToFrd(delta_flu);
    const Pose3 delta_frd_via_world = WorldPoseToNed(Ta).Inverse() * WorldPoseToNed(Tb);
    ExpectPose(delta_frd_direct, delta_frd_via_world);
  }
}

TEST(NedConversion, RotationVectorUsesShortestArc) {
  const Eigen::Quaterniond q(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()));
  Eigen::Quaterniond negated = q;
  negated.coeffs() *= -1.0;
  ExpectVec(RotationVector(q), Eigen::Vector3d(0.0, 0.3, 0.0));
  ExpectVec(RotationVector(negated), Eigen::Vector3d(0.0, 0.3, 0.0));
}

TEST(NedConversion, CovarianceRotatesBothBlocksAndStaysSymmetric) {
  Matrix6d cov = Matrix6d::Zero();
  cov.diagonal() << 1.0, 2.0, 3.0, 0.1, 0.2, 0.3;
  cov(0, 1) = cov(1, 0) = 0.5;  // x-y correlation
  const Matrix6d out = RotateCovariance6(cov, FrdFromFluRotation());
  EXPECT_TRUE(out.isApprox(out.transpose(), kTol));
  // Diagonal variances are invariant under an axis flip; the x-y
  // correlation flips sign because y is negated.
  EXPECT_NEAR(out(0, 0), 1.0, kTol);
  EXPECT_NEAR(out(1, 1), 2.0, kTol);
  EXPECT_NEAR(out(2, 2), 3.0, kTol);
  EXPECT_NEAR(out(4, 4), 0.2, kTol);
  EXPECT_NEAR(out(0, 1), -0.5, kTol);
}
