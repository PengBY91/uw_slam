#include "frontends/rigid_transform_fit.hpp"

#include <cmath>

#include <Eigen/SVD>

namespace uw::frontends {

namespace {

// Left-perturbs `pose` by xi = [dt(3); dtheta(3)]: rotation gets
// Exp(dtheta) composed on the LEFT, translation gets dt added directly
// (a simple decoupled "boxplus", not the fully coupled SE(3) exponential
// — sufficient for a LOCAL numerical Jacobian around an already-good
// pose estimate, and it's the convention RigidTransformFitResult's own
// doc comment promises).
uw::sensor_models::Pose3 Perturb(const uw::sensor_models::Pose3& pose,
                                 const Eigen::Matrix<double, 6, 1>& xi) {
  const Eigen::Vector3d dtheta = xi.tail<3>();
  const double angle = dtheta.norm();
  Eigen::Quaterniond dq;
  if (angle < 1e-9) {
    dq = Eigen::Quaterniond(1.0, 0.5 * dtheta.x(), 0.5 * dtheta.y(), 0.5 * dtheta.z());
  } else {
    dq = Eigen::Quaterniond(Eigen::AngleAxisd(angle, dtheta / angle));
  }
  uw::sensor_models::Pose3 result;
  result.rotation = (dq * pose.rotation).normalized();
  result.translation = pose.translation + xi.head<3>();
  return result;
}

// Estimates RigidTransformFitResult's remaining fields (everything but
// `pose` and `inlier_indices`, already known by the caller) via a
// numerical Jacobian of the inlier residuals around `pose`. Returns
// std::nullopt if the resulting normal matrix (J^T J) fails its rank or
// condition-number check -- degenerate/near-collinear inlier geometry.
std::optional<RigidTransformFitResult> EstimateFitQuality(
    const uw::sensor_models::Pose3& pose, const std::vector<Eigen::Vector3d>& a,
    const std::vector<Eigen::Vector3d>& b, const std::vector<std::size_t>& inliers,
    std::size_t correspondence_count, const CovarianceEstimationParams& covariance_params) {
  const std::size_t m = inliers.size();
  Eigen::MatrixXd jacobian(3 * m, 6);
  Eigen::VectorXd residual0(3 * m);

  constexpr double kStep = 1e-6;
  for (std::size_t row = 0; row < m; ++row) {
    const std::size_t i = inliers[row];
    residual0.segment<3>(3 * row) = b[i] - pose.Apply(a[i]);
    for (int k = 0; k < 6; ++k) {
      Eigen::Matrix<double, 6, 1> xi_plus = Eigen::Matrix<double, 6, 1>::Zero();
      Eigen::Matrix<double, 6, 1> xi_minus = Eigen::Matrix<double, 6, 1>::Zero();
      xi_plus(k) = kStep;
      xi_minus(k) = -kStep;
      const Eigen::Vector3d r_plus = b[i] - Perturb(pose, xi_plus).Apply(a[i]);
      const Eigen::Vector3d r_minus = b[i] - Perturb(pose, xi_minus).Apply(a[i]);
      jacobian.block<3, 1>(3 * row, k) = (r_plus - r_minus) / (2.0 * kStep);
    }
  }

  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd singular_values = svd.singularValues();
  if (singular_values.size() < 6) return std::nullopt;
  const double s_max = singular_values(0);
  const double s_min = singular_values(5);
  if (!std::isfinite(s_max) || !std::isfinite(s_min) || s_max <= 0.0) return std::nullopt;
  if (s_min <= covariance_params.singular_value_tolerance * s_max) return std::nullopt;

  const double normal_matrix_condition_number = (s_max / s_min) * (s_max / s_min);
  if (normal_matrix_condition_number > covariance_params.max_condition_number) return std::nullopt;

  const double squared_error = residual0.squaredNorm();
  if (std::sqrt(squared_error / static_cast<double>(m)) > covariance_params.max_inlier_rmse_m) {
    return std::nullopt;
  }
  const std::ptrdiff_t dof = static_cast<std::ptrdiff_t>(3 * m) - 6;
  const double sigma2 =
      std::max(covariance_params.residual_variance_floor_m2, squared_error / std::max<std::ptrdiff_t>(1, dof));

  Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
  const Eigen::MatrixXd v = svd.matrixV();
  for (int k = 0; k < 6; ++k) {
    covariance += (sigma2 / (singular_values(k) * singular_values(k))) * (v.col(k) * v.col(k).transpose());
  }
  covariance = 0.5 * (covariance + covariance.transpose());
  if (!covariance.allFinite()) return std::nullopt;

  RigidTransformFitResult result;
  result.pose = pose;
  result.correspondence_count = correspondence_count;
  result.inlier_indices = inliers;
  result.inlier_ratio = correspondence_count > 0
                            ? static_cast<double>(m) / static_cast<double>(correspondence_count)
                            : 0.0;
  result.inlier_rmse_m = std::sqrt(squared_error / static_cast<double>(m));
  result.normal_matrix_condition_number = normal_matrix_condition_number;
  result.covariance = covariance;
  return result;
}

}  // namespace

std::optional<uw::sensor_models::Pose3> FitRigidTransform(const std::vector<Eigen::Vector3d>& a,
                                                            const std::vector<Eigen::Vector3d>& b) {
  if (a.size() != b.size() || a.size() < 3) return std::nullopt;

  const std::size_t n = a.size();
  Eigen::Vector3d centroid_a = Eigen::Vector3d::Zero();
  Eigen::Vector3d centroid_b = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i < n; ++i) {
    centroid_a += a[i];
    centroid_b += b[i];
  }
  centroid_a /= static_cast<double>(n);
  centroid_b /= static_cast<double>(n);

  // H = sum_i (a[i] - centroid_a) * (b[i] - centroid_b)^T. SVD(H) = U*S*V^T
  // gives the rotation R = V*U^T minimizing sum ||b_centered - R*a_centered||^2
  // (Kabsch 1976 / Umeyama 1991), with a determinant-sign correction below
  // so R is always a proper rotation, never a reflection.
  Eigen::Matrix3d h = Eigen::Matrix3d::Zero();
  for (std::size_t i = 0; i < n; ++i) {
    h += (a[i] - centroid_a) * (b[i] - centroid_b).transpose();
  }

  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(h, Eigen::ComputeFullU | Eigen::ComputeFullV);
  if (svd.info() != Eigen::Success) return std::nullopt;

  Eigen::Matrix3d rotation = svd.matrixV() * svd.matrixU().transpose();
  if (rotation.determinant() < 0.0) {
    Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
    correction(2, 2) = -1.0;
    rotation = svd.matrixV() * correction * svd.matrixU().transpose();
  }

  if (!rotation.allFinite()) return std::nullopt;

  uw::sensor_models::Pose3 result;
  result.rotation = Eigen::Quaterniond(rotation);
  result.rotation.normalize();
  result.translation = centroid_b - rotation * centroid_a;
  return result;
}

std::optional<RigidTransformFitResult> FitRigidTransformRansac(
    const std::vector<Eigen::Vector3d>& a, const std::vector<Eigen::Vector3d>& b,
    const RansacParams& params, std::mt19937_64& rng,
    const CovarianceEstimationParams& covariance_params) {
  if (a.size() != b.size() || a.size() < 3) return std::nullopt;
  const std::size_t n = a.size();

  std::vector<std::size_t> best_inliers;
  if (n == 3) {
    best_inliers = {0, 1, 2};  // nothing to robustify against
  } else {
    std::uniform_int_distribution<std::size_t> index_dist(0, n - 1);
    for (int iter = 0; iter < params.max_iterations; ++iter) {
      const std::size_t i0 = index_dist(rng);
      std::size_t i1 = index_dist(rng);
      while (i1 == i0) i1 = index_dist(rng);
      std::size_t i2 = index_dist(rng);
      while (i2 == i0 || i2 == i1) i2 = index_dist(rng);

      const std::vector<Eigen::Vector3d> sample_a = {a[i0], a[i1], a[i2]};
      const std::vector<Eigen::Vector3d> sample_b = {b[i0], b[i1], b[i2]};
      const auto candidate = FitRigidTransform(sample_a, sample_b);
      if (!candidate.has_value()) continue;

      std::vector<std::size_t> inliers;
      inliers.reserve(n);
      for (std::size_t i = 0; i < n; ++i) {
        if ((b[i] - candidate->Apply(a[i])).norm() <= params.inlier_threshold_m) inliers.push_back(i);
      }
      if (inliers.size() > best_inliers.size()) best_inliers = std::move(inliers);
    }
  }

  if (static_cast<int>(best_inliers.size()) < params.min_inliers) return std::nullopt;

  std::vector<Eigen::Vector3d> inlier_a, inlier_b;
  inlier_a.reserve(best_inliers.size());
  inlier_b.reserve(best_inliers.size());
  for (const auto i : best_inliers) {
    inlier_a.push_back(a[i]);
    inlier_b.push_back(b[i]);
  }
  const auto refit = FitRigidTransform(inlier_a, inlier_b);
  if (!refit.has_value()) return std::nullopt;

  return EstimateFitQuality(*refit, a, b, best_inliers, n, covariance_params);
}

}  // namespace uw::frontends
