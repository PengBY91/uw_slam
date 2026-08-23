#include "frontends/rigid_transform_fit.hpp"

#include <random>
#include <vector>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

using uw::frontends::CovarianceEstimationParams;
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
  EXPECT_NEAR((fit->pose.translation - true_transform.translation).norm(), 0.0, 1e-6);
  EXPECT_NEAR(std::abs(fit->pose.rotation.dot(true_transform.rotation)), 1.0, 1e-6);
  EXPECT_EQ(fit->correspondence_count, fixture.a.size());
  EXPECT_EQ(fit->inlier_indices.size(), 8u);
  EXPECT_NEAR(fit->inlier_ratio, 8.0 / 11.0, 1e-9);
  EXPECT_NEAR(fit->inlier_rmse_m, 0.0, 1e-6);
  EXPECT_TRUE(fit->covariance.allFinite());
  EXPECT_TRUE(fit->covariance.isApprox(fit->covariance.transpose(), 1e-9));
  EXPECT_GT(fit->normal_matrix_condition_number, 0.0);
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
  EXPECT_EQ(fit_a->pose.translation, fit_b->pose.translation);
  EXPECT_TRUE(fit_a->pose.rotation.coeffs() == fit_b->pose.rotation.coeffs());
  EXPECT_EQ(fit_a->inlier_indices, fit_b->inlier_indices);
  EXPECT_EQ(fit_a->covariance, fit_b->covariance);
  EXPECT_EQ(fit_a->normal_matrix_condition_number, fit_b->normal_matrix_condition_number);
}

TEST(RigidTransformFitRansac, NoisyNonDegeneratePointsGiveFiniteSymmetricPositiveDefiniteCovariance) {
  Pose3 true_transform;
  true_transform.translation = Eigen::Vector3d(0.2, -0.4, 0.1);
  true_transform.rotation =
      Eigen::Quaterniond(Eigen::AngleAxisd(0.15, Eigen::Vector3d(0.3, 0.6, 0.1).normalized()));
  true_transform.rotation.normalize();

  const std::vector<Eigen::Vector3d> points_a = {
      {1.0, 0.0, 5.0},  {0.0, 1.0, 4.0},  {-1.0, -1.0, 6.0}, {2.0, -0.5, 3.0},
      {0.5, 2.0, 5.5},  {-2.0, 0.5, 4.5}, {1.5, -1.5, 6.5},  {-0.5, -2.0, 3.5},
  };
  std::mt19937_64 noise_rng(123);
  std::normal_distribution<double> noise(0.0, 0.01);
  std::vector<Eigen::Vector3d> points_b;
  points_b.reserve(points_a.size());
  for (const auto& p : points_a) {
    points_b.push_back(true_transform.Apply(p) +
                       Eigen::Vector3d(noise(noise_rng), noise(noise_rng), noise(noise_rng)));
  }

  RansacParams params;
  params.inlier_threshold_m = 0.2;
  std::mt19937_64 rng(7);
  const auto fit = FitRigidTransformRansac(points_a, points_b, params, rng);
  ASSERT_TRUE(fit.has_value());
  EXPECT_EQ(fit->correspondence_count, points_a.size());
  EXPECT_GT(fit->inlier_ratio, 0.9);
  EXPECT_GT(fit->inlier_rmse_m, 0.0);
  EXPECT_LT(fit->inlier_rmse_m, 0.05);
  ASSERT_TRUE(fit->covariance.allFinite());
  EXPECT_TRUE(fit->covariance.isApprox(fit->covariance.transpose(), 1e-9));
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigensolver(fit->covariance);
  ASSERT_EQ(eigensolver.info(), Eigen::Success);
  EXPECT_GT(eigensolver.eigenvalues().minCoeff(), 0.0);
}

TEST(RigidTransformFitRansac, RejectsFitWhoseInlierRmseExceedsConfiguredCap) {
  // Same well-conditioned, non-degenerate correspondences as
  // NoisyNonDegeneratePointsGiveFiniteSymmetricPositiveDefiniteCovariance
  // (inlier_rmse_m < 0.05 there) -- conditioning alone has nothing to
  // reject here. max_inlier_rmse_m is a SEPARATE gate: it catches a fit
  // whose inlier set doesn't actually agree tightly, even when that set is
  // well-conditioned and clears RansacParams::min_inliers -- a small,
  // coincidentally self-consistent set of false correspondences can still
  // pass both of those (see CovarianceEstimationParams::max_inlier_rmse_m's
  // doc comment for the real-data motivation).
  Pose3 true_transform;
  true_transform.translation = Eigen::Vector3d(0.2, -0.4, 0.1);
  true_transform.rotation =
      Eigen::Quaterniond(Eigen::AngleAxisd(0.15, Eigen::Vector3d(0.3, 0.6, 0.1).normalized()));
  true_transform.rotation.normalize();

  const std::vector<Eigen::Vector3d> points_a = {
      {1.0, 0.0, 5.0},  {0.0, 1.0, 4.0},  {-1.0, -1.0, 6.0}, {2.0, -0.5, 3.0},
      {0.5, 2.0, 5.5},  {-2.0, 0.5, 4.5}, {1.5, -1.5, 6.5},  {-0.5, -2.0, 3.5},
  };
  std::mt19937_64 noise_rng(123);
  std::normal_distribution<double> noise(0.0, 0.01);
  std::vector<Eigen::Vector3d> points_b;
  points_b.reserve(points_a.size());
  for (const auto& p : points_a) {
    points_b.push_back(true_transform.Apply(p) +
                       Eigen::Vector3d(noise(noise_rng), noise(noise_rng), noise(noise_rng)));
  }

  RansacParams params;
  params.inlier_threshold_m = 0.2;
  CovarianceEstimationParams covariance_params;
  covariance_params.max_inlier_rmse_m = 0.001;  // well below the fit's actual ~0.01-0.02m rmse
  std::mt19937_64 rng(7);
  EXPECT_FALSE(FitRigidTransformRansac(points_a, points_b, params, rng, covariance_params).has_value());

  // The same correspondences accepted once the cap is generous enough --
  // confirms the rejection above is the new gate, not some other change.
  CovarianceEstimationParams generous_covariance_params;
  generous_covariance_params.max_inlier_rmse_m = 0.05;
  std::mt19937_64 rng2(7);
  EXPECT_TRUE(
      FitRigidTransformRansac(points_a, points_b, params, rng2, generous_covariance_params).has_value());
}

TEST(RigidTransformFitRansac, RejectsCollinearInlierGeometryOnConditioning) {
  Pose3 true_transform;
  true_transform.translation = Eigen::Vector3d(0.1, 0.1, 0.1);
  true_transform.rotation = Eigen::Quaterniond::Identity();

  // All points on the line x=y=0: rotation about that axis is completely
  // unobservable from these correspondences, so the normal matrix is
  // singular/ill-conditioned regardless of how many points are added.
  std::vector<Eigen::Vector3d> points_a;
  for (int i = 0; i < 10; ++i) points_a.emplace_back(0.0, 0.0, 3.0 + 0.5 * i);
  std::vector<Eigen::Vector3d> points_b;
  points_b.reserve(points_a.size());
  for (const auto& p : points_a) points_b.push_back(true_transform.Apply(p));

  RansacParams params;
  params.inlier_threshold_m = 0.05;
  std::mt19937_64 rng(3);
  EXPECT_FALSE(FitRigidTransformRansac(points_a, points_b, params, rng).has_value());
}
