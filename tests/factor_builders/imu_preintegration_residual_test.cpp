#include "factor_builders/imu_preintegration_residual.hpp"

#include <array>
#include <cmath>
#include <random>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "sensor_models/geometry.hpp"
#include "sensor_models/so3.hpp"

using uw::factor_builders::ImuPreintegrationResidual;
using uw::sensor_models::Pose3;
using uw::sensor_models::PreintegratedImuDelta;
namespace so3 = uw::sensor_models::so3;

namespace {

using Matrix15d = Eigen::Matrix<double, 15, 15>;
using Vector15d = Eigen::Matrix<double, 15, 1>;

struct States {
  std::array<double, 7> pose_i{};
  std::array<double, 9> inertial_i{};
  std::array<double, 7> pose_j{};
  std::array<double, 9> inertial_j{};
};

std::array<double, 7> PoseParams(const Eigen::Vector3d& t, const Eigen::Vector3d& rotvec) {
  Pose3 pose;
  pose.translation = t;
  pose.rotation = so3::ExpQuaternion(rotvec);
  return pose.ToParameterBlock();
}

std::array<double, 9> InertialParams(const Eigen::Vector3d& v, const Eigen::Vector3d& bg,
                                      const Eigen::Vector3d& ba) {
  return {v.x(), v.y(), v.z(), bg.x(), bg.y(), bg.z(), ba.x(), ba.y(), ba.z()};
}

std::vector<const double*> Pointers(const States& s) {
  return {s.pose_i.data(), s.inertial_i.data(), s.pose_j.data(), s.inertial_j.data()};
}

Vector15d Residual(const ImuPreintegrationResidual& residual, const States& s) {
  Vector15d out = Vector15d::Zero();
  auto params = Pointers(s);
  EXPECT_TRUE(residual.Evaluate(params, out.data(), nullptr));
  return out;
}

// A delta whose bias Jacobians and covariance are dense but well
// conditioned, so the finite-difference check exercises every derived
// block rather than accidentally zeroing some of them out.
PreintegratedImuDelta MakeDelta() {
  PreintegratedImuDelta delta;
  delta.delta_time_s = 0.4;
  delta.sample_count = 80;
  delta.gravity_mps2 = 9.80665;
  delta.delta_rotation = so3::Exp(Eigen::Vector3d(0.05, -0.03, 0.12));
  delta.delta_velocity = Eigen::Vector3d(0.31, -0.12, 0.07);
  delta.delta_position = Eigen::Vector3d(0.062, -0.021, 0.014);
  delta.bias_gyro = Eigen::Vector3d(0.002, -0.001, 0.0005);
  delta.bias_accel = Eigen::Vector3d(0.01, 0.02, -0.015);

  Eigen::Matrix3d jac;
  jac << 0.40, 0.03, -0.02, 0.01, 0.38, 0.04, -0.03, 0.02, 0.41;
  delta.d_rotation_d_bias_gyro = -jac;
  delta.d_velocity_d_bias_gyro = 0.12 * jac;
  delta.d_velocity_d_bias_accel = -0.40 * Eigen::Matrix3d::Identity() - 0.02 * jac;
  delta.d_position_d_bias_gyro = 0.03 * jac;
  delta.d_position_d_bias_accel = -0.08 * Eigen::Matrix3d::Identity() - 0.005 * jac;

  Matrix15d root = Matrix15d::Zero();
  std::mt19937_64 rng(20260903u);
  std::uniform_real_distribution<double> dist(-0.3, 0.3);
  for (int r = 0; r < 15; ++r) {
    for (int c = 0; c <= r; ++c) root(r, c) = dist(rng);
    root(r, r) = 0.4 + 0.05 * r;
  }
  delta.covariance = root * root.transpose();
  return delta;
}

// sqrt_information = L^-1 of covariance = L L^T, the same construction
// ImuPreintegrationFactorBuilder performs.
Matrix15d SqrtInformationOf(const PreintegratedImuDelta& delta) {
  const Eigen::LLT<Matrix15d> llt(delta.covariance);
  EXPECT_EQ(llt.info(), Eigen::Success);
  return llt.matrixL().solve(Matrix15d::Identity());
}

States MakeStates() {
  States s;
  s.pose_i = PoseParams(Eigen::Vector3d(1.0, -0.5, -2.0), Eigen::Vector3d(0.10, -0.07, 0.25));
  s.inertial_i = InertialParams(Eigen::Vector3d(0.8, 0.15, -0.05), Eigen::Vector3d(0.004, -0.002, 0.001),
                                 Eigen::Vector3d(0.02, -0.01, 0.03));
  s.pose_j = PoseParams(Eigen::Vector3d(1.4, -0.42, -1.94), Eigen::Vector3d(0.14, -0.02, 0.31));
  s.inertial_j = InertialParams(Eigen::Vector3d(0.9, 0.2, 0.02), Eigen::Vector3d(0.0035, -0.0018, 0.0013),
                                 Eigen::Vector3d(0.018, -0.008, 0.026));
  return s;
}

double* BlockData(States& s, int block) {
  switch (block) {
    case 0: return s.pose_i.data();
    case 1: return s.inertial_i.data();
    case 2: return s.pose_j.data();
    default: return s.inertial_j.data();
  }
}

}  // namespace

TEST(ImuPreintegrationResidual, ZeroResidualOnStatesConsistentWithTheDeltas) {
  const auto delta = MakeDelta();
  // Build state j EXACTLY from the delta definitions at the linearisation
  // bias, so every one of the 15 rows must come out zero.
  States s;
  const Eigen::Vector3d t_i(0.3, -1.2, -4.0);
  const Eigen::Vector3d phi_i(-0.2, 0.11, 0.4);
  const Eigen::Vector3d v_i(0.5, -0.3, 0.12);
  s.pose_i = PoseParams(t_i, phi_i);
  s.inertial_i = InertialParams(v_i, delta.bias_gyro, delta.bias_accel);

  const Eigen::Matrix3d R_i = so3::Exp(phi_i);
  const double dt = delta.delta_time_s;
  const Eigen::Vector3d gravity(0.0, 0.0, -delta.gravity_mps2);
  const Eigen::Matrix3d R_j = R_i * delta.delta_rotation;
  const Eigen::Vector3d v_j = v_i + gravity * dt + R_i * delta.delta_velocity;
  const Eigen::Vector3d t_j =
      t_i + v_i * dt + 0.5 * gravity * dt * dt + R_i * delta.delta_position;
  s.pose_j = PoseParams(t_j, so3::Log(R_j));
  s.inertial_j = InertialParams(v_j, delta.bias_gyro, delta.bias_accel);

  ImuPreintegrationResidual residual(delta, Matrix15d::Identity());
  const Vector15d r = Residual(residual, s);
  EXPECT_LT(r.norm(), 1e-9) << r.transpose();
}

TEST(ImuPreintegrationResidual, BiasBlockRowsAreTheBiasDifference) {
  const auto delta = MakeDelta();
  ImuPreintegrationResidual residual(delta, Matrix15d::Identity());
  const States s = MakeStates();
  const Vector15d r = Residual(residual, s);

  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(r(9 + i), s.inertial_j[3 + i] - s.inertial_i[3 + i], 1e-12);
    EXPECT_NEAR(r(12 + i), s.inertial_j[6 + i] - s.inertial_i[6 + i], 1e-12);
  }
}

TEST(ImuPreintegrationResidual, WhiteningIsAppliedAsSqrtInformationTimesRawResidual) {
  const auto delta = MakeDelta();
  const Matrix15d sqrt_information = SqrtInformationOf(delta);
  const States s = MakeStates();

  ImuPreintegrationResidual raw_residual(delta, Matrix15d::Identity());
  ImuPreintegrationResidual whitened_residual(delta, sqrt_information);
  const Vector15d raw = Residual(raw_residual, s);
  const Vector15d whitened = Residual(whitened_residual, s);
  EXPECT_LT((whitened - sqrt_information * raw).norm(), 1e-10);
}

// The reason this file exists: every Jacobian block is hand-derived (see
// the header), so it is cross-checked against central finite differences
// over the RAW parameters the solvers actually step -- including the
// 4-parameter quaternion columns, where the chain rule through
// Pose3::FromParameterBlock's normalisation is what makes the analytic
// form exact rather than approximate.
TEST(ImuPreintegrationResidual, AnalyticJacobiansMatchCentralFiniteDifferences) {
  const auto delta = MakeDelta();
  ImuPreintegrationResidual residual(delta, SqrtInformationOf(delta));
  States s = MakeStates();

  const std::array<int, 4> sizes{7, 9, 7, 9};
  std::array<std::vector<double>, 4> storage;
  std::vector<double*> jac_ptrs(4, nullptr);
  for (int b = 0; b < 4; ++b) {
    storage[b].assign(15 * static_cast<std::size_t>(sizes[b]), 0.0);
    jac_ptrs[b] = storage[b].data();
  }
  Vector15d analytic_residual = Vector15d::Zero();
  auto params = Pointers(s);
  ASSERT_TRUE(residual.Evaluate(params, analytic_residual.data(), &jac_ptrs));

  constexpr double kEps = 1e-7;
  for (int b = 0; b < 4; ++b) {
    double* block = BlockData(s, b);
    for (int p = 0; p < sizes[b]; ++p) {
      const double original = block[p];
      block[p] = original + kEps;
      const Vector15d plus = Residual(residual, s);
      block[p] = original - kEps;
      const Vector15d minus = Residual(residual, s);
      block[p] = original;

      const Vector15d numeric = (plus - minus) / (2.0 * kEps);
      for (int row = 0; row < 15; ++row) {
        const double analytic = storage[b][row * sizes[b] + p];
        EXPECT_NEAR(analytic, numeric(row), 1e-4)
            << "block " << b << " param " << p << " row " << row;
      }
    }
  }
}

// A large bias offset from the linearisation point exercises the
// re-linearisation path (Corrected*, the Jr(J_R dbg) term in the rotation
// row) rather than the dbg = 0 shortcut.
TEST(ImuPreintegrationResidual, JacobiansStayExactFarFromTheLinearisationBias) {
  const auto delta = MakeDelta();
  ImuPreintegrationResidual residual(delta, Matrix15d::Identity());
  States s = MakeStates();
  s.inertial_i[3] = delta.bias_gyro.x() + 0.08;
  s.inertial_i[4] = delta.bias_gyro.y() - 0.05;
  s.inertial_i[5] = delta.bias_gyro.z() + 0.11;
  s.inertial_i[6] = delta.bias_accel.x() + 0.30;
  s.inertial_i[7] = delta.bias_accel.y() - 0.22;
  s.inertial_i[8] = delta.bias_accel.z() + 0.17;

  const std::array<int, 4> sizes{7, 9, 7, 9};
  std::array<std::vector<double>, 4> storage;
  std::vector<double*> jac_ptrs(4, nullptr);
  for (int b = 0; b < 4; ++b) {
    storage[b].assign(15 * static_cast<std::size_t>(sizes[b]), 0.0);
    jac_ptrs[b] = storage[b].data();
  }
  Vector15d out = Vector15d::Zero();
  auto params = Pointers(s);
  ASSERT_TRUE(residual.Evaluate(params, out.data(), &jac_ptrs));

  constexpr double kEps = 1e-7;
  for (int b = 0; b < 4; ++b) {
    double* block = BlockData(s, b);
    for (int p = 0; p < sizes[b]; ++p) {
      const double original = block[p];
      block[p] = original + kEps;
      const Vector15d plus = Residual(residual, s);
      block[p] = original - kEps;
      const Vector15d minus = Residual(residual, s);
      block[p] = original;
      const Vector15d numeric = (plus - minus) / (2.0 * kEps);
      for (int row = 0; row < 15; ++row) {
        EXPECT_NEAR(storage[b][row * sizes[b] + p], numeric(row), 1e-4)
            << "block " << b << " param " << p << " row " << row;
      }
    }
  }
}

// Under Ceres the ambient 15x4 quaternion Jacobian gets post-multiplied by
// the manifold's dq/ddphi = 0.5 * Q; that product must reproduce the
// minimal 15x3 rotation Jacobian exactly, otherwise the two solver
// backends would see different curvature for the same factor.
TEST(ImuPreintegrationResidual, QuaternionColumnsProjectBackToTheMinimalJacobian) {
  const auto delta = MakeDelta();
  ImuPreintegrationResidual residual(delta, Matrix15d::Identity());
  States s = MakeStates();

  const std::array<int, 4> sizes{7, 9, 7, 9};
  std::array<std::vector<double>, 4> storage;
  std::vector<double*> jac_ptrs(4, nullptr);
  for (int b = 0; b < 4; ++b) {
    storage[b].assign(15 * static_cast<std::size_t>(sizes[b]), 0.0);
    jac_ptrs[b] = storage[b].data();
  }
  Vector15d out = Vector15d::Zero();
  auto params = Pointers(s);
  ASSERT_TRUE(residual.Evaluate(params, out.data(), &jac_ptrs));

  for (int b : {0, 2}) {
    const Pose3 pose = Pose3::FromParameterBlock(BlockData(s, b));
    Eigen::Matrix<double, 4, 3> tangent_map;
    tangent_map.topRows<3>() =
        pose.rotation.w() * Eigen::Matrix3d::Identity() + so3::Hat(pose.rotation.vec());
    tangent_map.bottomRows<1>() = -pose.rotation.vec().transpose();

    Eigen::Matrix<double, 15, 4> ambient;
    for (int row = 0; row < 15; ++row) {
      for (int col = 0; col < 4; ++col) ambient(row, col) = storage[b][row * 7 + 3 + col];
    }
    const Eigen::Matrix<double, 15, 3> projected = ambient * (0.5 * tangent_map);

    // Recover the minimal Jacobian numerically: perturb the pose on the
    // right, R <- R Exp(dphi), and difference the residual.
    Eigen::Matrix<double, 15, 3> numeric;
    constexpr double kEps = 1e-7;
    for (int axis = 0; axis < 3; ++axis) {
      Eigen::Vector3d dphi = Eigen::Vector3d::Zero();
      States perturbed = s;
      dphi(axis) = kEps;
      Pose3 plus_pose = pose;
      plus_pose.rotation = (pose.rotation * so3::ExpQuaternion(dphi)).normalized();
      const auto plus_params = plus_pose.ToParameterBlock();
      std::copy(plus_params.begin(), plus_params.end(), BlockData(perturbed, b));
      const Vector15d plus = Residual(residual, perturbed);

      dphi(axis) = -kEps;
      Pose3 minus_pose = pose;
      minus_pose.rotation = (pose.rotation * so3::ExpQuaternion(dphi)).normalized();
      const auto minus_params = minus_pose.ToParameterBlock();
      perturbed = s;
      std::copy(minus_params.begin(), minus_params.end(), BlockData(perturbed, b));
      const Vector15d minus = Residual(residual, perturbed);

      numeric.col(axis) = (plus - minus) / (2.0 * kEps);
    }
    EXPECT_LT((projected - numeric).norm(), 1e-5) << "block " << b;
  }
}

TEST(ImuPreintegrationResidual, RejectsShortParameterList) {
  const auto delta = MakeDelta();
  ImuPreintegrationResidual residual(delta, Matrix15d::Identity());
  States s = MakeStates();
  std::vector<const double*> short_params{s.pose_i.data(), s.inertial_i.data()};
  Vector15d out = Vector15d::Zero();
  EXPECT_FALSE(residual.Evaluate(short_params, out.data(), nullptr));
}
