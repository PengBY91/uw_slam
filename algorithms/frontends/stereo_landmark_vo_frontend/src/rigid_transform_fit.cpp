#include "uw/frontends/rigid_transform_fit.hpp"

#include <Eigen/SVD>

namespace uw::frontends {

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

std::optional<uw::sensor_models::Pose3> FitRigidTransformRansac(const std::vector<Eigen::Vector3d>& a,
                                                                  const std::vector<Eigen::Vector3d>& b,
                                                                  const RansacParams& params,
                                                                  std::mt19937_64& rng) {
  if (a.size() != b.size() || a.size() < 3) return std::nullopt;
  const std::size_t n = a.size();
  if (n == 3) return FitRigidTransform(a, b);  // nothing to robustify against

  std::uniform_int_distribution<std::size_t> index_dist(0, n - 1);
  std::vector<std::size_t> best_inliers;

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

  if (static_cast<int>(best_inliers.size()) < params.min_inliers) return std::nullopt;

  std::vector<Eigen::Vector3d> inlier_a, inlier_b;
  inlier_a.reserve(best_inliers.size());
  inlier_b.reserve(best_inliers.size());
  for (const auto i : best_inliers) {
    inlier_a.push_back(a[i]);
    inlier_b.push_back(b[i]);
  }
  return FitRigidTransform(inlier_a, inlier_b);
}

}  // namespace uw::frontends
