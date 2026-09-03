#include "sensor_models/so3.hpp"

#include <cmath>
#include <random>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

namespace so3 = uw::sensor_models::so3;

namespace {

Eigen::Vector3d RandomVector(std::mt19937_64& rng, double scale) {
  std::uniform_real_distribution<double> dist(-scale, scale);
  return Eigen::Vector3d(dist(rng), dist(rng), dist(rng));
}

TEST(So3, HatMatchesCrossProductAndVeeInvertsIt) {
  const Eigen::Vector3d v(0.3, -1.2, 2.5);
  const Eigen::Vector3d w(-0.7, 0.4, 0.9);
  EXPECT_LT((so3::Hat(v) * w - v.cross(w)).norm(), 1e-15);
  EXPECT_LT((so3::Vee(so3::Hat(v)) - v).norm(), 1e-15);
}

TEST(So3, ExpMatchesAngleAxisAndQuaternionForm) {
  std::mt19937_64 rng(7);
  for (int i = 0; i < 50; ++i) {
    const Eigen::Vector3d phi = RandomVector(rng, 2.0);
    const Eigen::Matrix3d expected = Eigen::AngleAxisd(phi.norm(), phi.normalized()).toRotationMatrix();
    EXPECT_LT((so3::Exp(phi) - expected).norm(), 1e-12);
    EXPECT_LT((so3::ExpQuaternion(phi).toRotationMatrix() - expected).norm(), 1e-12);
  }
}

TEST(So3, ExpLogRoundTripIncludingSmallAndNearPiAngles) {
  std::mt19937_64 rng(11);
  for (int i = 0; i < 100; ++i) {
    const Eigen::Vector3d dir = RandomVector(rng, 1.0).normalized();
    std::uniform_real_distribution<double> angle_dist(0.0, M_PI - 1e-3);
    const Eigen::Vector3d phi = angle_dist(rng) * dir;
    EXPECT_LT((so3::Log(so3::Exp(phi)) - phi).norm(), 1e-9) << "phi=" << phi.transpose();
    EXPECT_LT((so3::Log(so3::ExpQuaternion(phi)) - phi).norm(), 1e-9);
  }
  // Small angle branch.
  const Eigen::Vector3d tiny(1e-9, -2e-9, 3e-9);
  EXPECT_LT((so3::Log(so3::Exp(tiny)) - tiny).norm(), 1e-15);
  EXPECT_LT((so3::Exp(Eigen::Vector3d::Zero()) - Eigen::Matrix3d::Identity()).norm(), 0.0 + 1e-16);
  // Near pi: the rotation is recovered up to the sign ambiguity of the axis.
  const Eigen::Vector3d near_pi = (M_PI - 1e-7) * Eigen::Vector3d(0.0, 1.0, 0.0);
  const Eigen::Vector3d recovered = so3::Log(so3::Exp(near_pi));
  EXPECT_LT((so3::Exp(recovered) - so3::Exp(near_pi)).norm(), 1e-6);
}

TEST(So3, RightJacobianMatchesFiniteDifferenceOfExp) {
  // Exp(phi + d) ~= Exp(phi) Exp(Jr(phi) d)  =>  Log(Exp(phi)^T Exp(phi + d)) ~= Jr(phi) d
  std::mt19937_64 rng(3);
  for (int i = 0; i < 30; ++i) {
    const Eigen::Vector3d phi = RandomVector(rng, 1.5);
    const Eigen::Matrix3d jr = so3::RightJacobian(phi);
    const double eps = 1e-6;
    for (int axis = 0; axis < 3; ++axis) {
      Eigen::Vector3d d = Eigen::Vector3d::Zero();
      d(axis) = eps;
      const Eigen::Vector3d numeric = so3::Log(so3::Exp(phi).transpose() * so3::Exp(phi + d)) / eps;
      EXPECT_LT((numeric - jr.col(axis)).norm(), 1e-5) << "phi=" << phi.transpose() << " axis=" << axis;
    }
  }
}

TEST(So3, RightJacobianInverseIsTheInverse) {
  std::mt19937_64 rng(5);
  for (int i = 0; i < 30; ++i) {
    const Eigen::Vector3d phi = RandomVector(rng, 1.5);
    const Eigen::Matrix3d product = so3::RightJacobian(phi) * so3::RightJacobianInverse(phi);
    EXPECT_LT((product - Eigen::Matrix3d::Identity()).norm(), 1e-10);
  }
  const Eigen::Vector3d tiny(1e-8, 2e-8, -1e-8);
  EXPECT_LT((so3::RightJacobian(tiny) * so3::RightJacobianInverse(tiny) - Eigen::Matrix3d::Identity()).norm(),
            1e-12);
}

}  // namespace
