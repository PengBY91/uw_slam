#include "frontends/rigid_transform_fit.hpp"

#include <random>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

using uw::frontends::FitRigidTransform;
using uw::frontends::FitRigidTransformRansac;
using uw::frontends::RansacParams;
using uw::sensor_models::Pose3;

TEST(RigidTransformFit, RecoversAKnownTransformFromNonCollinearPoints) {
  Pose3 true_transform;
  true_transform.translation = Eigen::Vector3d(0.5, -1.2, 0.3);
  true_transform.rotation =
      Eigen::Quaterniond(Eigen::AngleAxisd(0.4, Eigen::Vector3d(0.2, 0.8, 0.3).normalized()));
  true_transform.rotation.normalize();

  const std::vector<Eigen::Vector3d> points_a = {
      {1.0, 0.0, 5.0}, {0.0, 1.0, 4.0}, {-1.0, -1.0, 6.0}, {2.0, -0.5, 3.0}, {0.5, 2.0, 5.5},
  };
  std::vector<Eigen::Vector3d> points_b;
  points_b.reserve(points_a.size());
  for (const auto& p : points_a) points_b.push_back(true_transform.Apply(p));

  const auto fit = FitRigidTransform(points_a, points_b);
  ASSERT_TRUE(fit.has_value());

  EXPECT_NEAR((fit->translation - true_transform.translation).norm(), 0.0, 1e-9);
  EXPECT_NEAR(std::abs(fit->rotation.dot(true_transform.rotation)), 1.0, 1e-9);

  for (const auto& p : points_a) {
    EXPECT_NEAR((fit->Apply(p) - true_transform.Apply(p)).norm(), 0.0, 1e-9);
  }
}

TEST(RigidTransformFit, RejectsFewerThanThreePoints) {
  const std::vector<Eigen::Vector3d> two_points = {{0, 0, 1}, {1, 0, 1}};
  EXPECT_FALSE(FitRigidTransform(two_points, two_points).has_value());
  EXPECT_FALSE(FitRigidTransform({}, {}).has_value());
}

TEST(RigidTransformFit, RejectsMismatchedSizes) {
  const std::vector<Eigen::Vector3d> a = {{0, 0, 1}, {1, 0, 1}, {0, 1, 1}};
  const std::vector<Eigen::Vector3d> b = {{0, 0, 1}, {1, 0, 1}};
  EXPECT_FALSE(FitRigidTransform(a, b).has_value());
}

TEST(RigidTransformFit, IdentityInputGivesIdentityTransform) {
  const std::vector<Eigen::Vector3d> points = {{1, 0, 5}, {0, 1, 4}, {-1, -1, 6}, {2, -0.5, 3}};
  const auto fit = FitRigidTransform(points, points);
  ASSERT_TRUE(fit.has_value());
  EXPECT_NEAR(fit->translation.norm(), 0.0, 1e-9);
  EXPECT_NEAR(std::abs(fit->rotation.dot(Eigen::Quaterniond::Identity())), 1.0, 1e-9);
}

namespace {

// 8 points that are exact under `transform` ("inliers") plus 3 points
// whose `b` side is a fixed, wildly-inconsistent value ("outliers" — as
// if temporal_matcher_ had greedily matched the wrong blob for those
// three). Plain FitRigidTransform over all 11 would be pulled badly off
// by the outliers; FitRigidTransformRansac should still recover
// `transform` from the 8-point inlier majority.
struct OutlierFixture {
  std::vector<Eigen::Vector3d> a;
  std::vector<Eigen::Vector3d> b;
};

OutlierFixture MakeOutlierFixture(const Pose3& transform) {
  OutlierFixture fixture;
  const std::vector<Eigen::Vector3d> inlier_a = {
      {1.0, 0.0, 5.0},  {0.0, 1.0, 4.0},  {-1.0, -1.0, 6.0}, {2.0, -0.5, 3.0},
      {0.5, 2.0, 5.5},  {-2.0, 0.5, 4.5}, {1.5, -1.5, 6.5},  {-0.5, -2.0, 3.5},
  };
  for (const auto& p : inlier_a) {
    fixture.a.push_back(p);
    fixture.b.push_back(transform.Apply(p));
  }
  const std::vector<Eigen::Vector3d> outlier_a = {{3.0, 3.0, 7.0}, {-3.0, 3.0, 7.0}, {3.0, -3.0, 7.0}};
  for (const auto& p : outlier_a) {
    fixture.a.push_back(p);
    fixture.b.push_back(Eigen::Vector3d(100.0, -100.0, 200.0));  // consistent with no plausible transform
  }
  return fixture;
}

}  // namespace

TEST(RigidTransformFitRansac, RecoversTrueTransformDespiteAMinorityOfOutliers) {
  Pose3 true_transform;
  true_transform.translation = Eigen::Vector3d(0.3, -0.6, 0.15);
  true_transform.rotation =
      Eigen::Quaterniond(Eigen::AngleAxisd(0.2, Eigen::Vector3d(0.1, 0.9, 0.2).normalized()));
  true_transform.rotation.normalize();

  const auto fixture = MakeOutlierFixture(true_transform);

  RansacParams params;
  params.inlier_threshold_m = 0.05;
  params.max_iterations = 300;
  params.min_inliers = 3;
  std::mt19937_64 rng(42);

  const auto fit = FitRigidTransformRansac(fixture.a, fixture.b, params, rng);
  ASSERT_TRUE(fit.has_value());
  EXPECT_NEAR((fit->translation - true_transform.translation).norm(), 0.0, 1e-6);
  EXPECT_NEAR(std::abs(fit->rotation.dot(true_transform.rotation)), 1.0, 1e-6);
}

TEST(RigidTransformFitRansac, PlainFitIsPulledOffByTheSameOutliersRansacIgnores) {
  // Same fixture as above — demonstrates *why* RANSAC is needed, not just
  // that it works: the un-robustified fit over the same 11 points (8
  // inliers + 3 outliers) should NOT recover the true transform.
  Pose3 true_transform;
  true_transform.translation = Eigen::Vector3d(0.3, -0.6, 0.15);
  true_transform.rotation =
      Eigen::Quaterniond(Eigen::AngleAxisd(0.2, Eigen::Vector3d(0.1, 0.9, 0.2).normalized()));
  true_transform.rotation.normalize();

  const auto fixture = MakeOutlierFixture(true_transform);
  const auto plain_fit = FitRigidTransform(fixture.a, fixture.b);
  ASSERT_TRUE(plain_fit.has_value());
  EXPECT_GT((plain_fit->translation - true_transform.translation).norm(), 0.5);
}

TEST(RigidTransformFitRansac, RejectsWhenTooFewPoints) {
  RansacParams params;
  std::mt19937_64 rng(1);
  const std::vector<Eigen::Vector3d> two_points = {{0, 0, 1}, {1, 0, 1}};
  EXPECT_FALSE(FitRigidTransformRansac(two_points, two_points, params, rng).has_value());
}

TEST(RigidTransformFitRansac, RejectsWhenNoConsistentInlierSetExists) {
  // All-noise correspondences: no 3-point sample should find a transform
  // that many other points also agree with.
  std::mt19937_64 seed_rng(7);
  std::uniform_real_distribution<double> spread(-10.0, 10.0);
  std::vector<Eigen::Vector3d> a, b;
  for (int i = 0; i < 20; ++i) {
    a.emplace_back(spread(seed_rng), spread(seed_rng), spread(seed_rng) + 15.0);
    b.emplace_back(spread(seed_rng), spread(seed_rng), spread(seed_rng) + 15.0);
  }

  RansacParams params;
  params.inlier_threshold_m = 0.01;  // tight enough that random agreement is implausible
  params.max_iterations = 100;
  params.min_inliers = 6;
  std::mt19937_64 rng(99);
  EXPECT_FALSE(FitRigidTransformRansac(a, b, params, rng).has_value());
}

TEST(RigidTransformFitRansac, IsDeterministicGivenTheSameSeed) {
  Pose3 true_transform;
  true_transform.translation = Eigen::Vector3d(0.1, 0.2, -0.1);
  true_transform.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitY()));
  const auto fixture = MakeOutlierFixture(true_transform);

  RansacParams params;
  params.inlier_threshold_m = 0.05;

  std::mt19937_64 rng_a(2024);
  std::mt19937_64 rng_b(2024);
  const auto fit_a = FitRigidTransformRansac(fixture.a, fixture.b, params, rng_a);
  const auto fit_b = FitRigidTransformRansac(fixture.a, fixture.b, params, rng_b);
  ASSERT_TRUE(fit_a.has_value());
  ASSERT_TRUE(fit_b.has_value());
  EXPECT_EQ(fit_a->translation, fit_b->translation);
  EXPECT_TRUE(fit_a->rotation.coeffs() == fit_b->rotation.coeffs());
}
