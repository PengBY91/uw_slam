#include "factor_builders/relative_pose_residual.hpp"

#include <array>

#include <gtest/gtest.h>

using uw::factor_builders::RelativePoseResidual;
using uw::sensor_models::Pose3;

TEST(RelativePoseResidual, ZeroWhenPosesMatchMeasurement) {
  Pose3 pose_i;
  pose_i.translation = Eigen::Vector3d(1.0, 2.0, 3.0);
  pose_i.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()));

  Pose3 measured_relative;
  measured_relative.translation = Eigen::Vector3d(0.5, -0.1, 0.2);
  measured_relative.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitY()));

  const Pose3 pose_j = pose_i * measured_relative;

  RelativePoseResidual residual(measured_relative, Eigen::Matrix<double, 6, 6>::Identity());

  const auto params_i = pose_i.ToParameterBlock();
  const auto params_j = pose_j.ToParameterBlock();
  std::vector<const double*> parameters{params_i.data(), params_j.data()};

  std::array<double, 6> out{};
  ASSERT_TRUE(residual.Evaluate(parameters, out.data(), nullptr));
  for (double v : out) EXPECT_NEAR(v, 0.0, 1e-9);
}

TEST(RelativePoseResidual, JacobianIsFiniteAndNonZeroAwayFromMatch) {
  Pose3 pose_i;
  Pose3 pose_j;
  pose_j.translation = Eigen::Vector3d(1.0, 0.0, 0.0);

  Pose3 measured_relative;
  measured_relative.translation = Eigen::Vector3d(0.9, 0.05, 0.0);  // slightly off

  RelativePoseResidual residual(measured_relative, Eigen::Matrix<double, 6, 6>::Identity());

  const auto params_i = pose_i.ToParameterBlock();
  const auto params_j = pose_j.ToParameterBlock();
  std::vector<const double*> parameters{params_i.data(), params_j.data()};

  std::array<double, 6> out{};
  std::array<double, 42> jac_i{};  // 6x7
  std::array<double, 42> jac_j{};
  std::vector<double*> jacobians{jac_i.data(), jac_j.data()};

  ASSERT_TRUE(residual.Evaluate(parameters, out.data(), &jacobians));
  EXPECT_GT(std::abs(out[0]), 1e-3);  // translation-x should show the 0.1m offset

  bool any_nonzero = false;
  for (double v : jac_j) {
    ASSERT_TRUE(std::isfinite(v));
    if (v != 0.0) any_nonzero = true;
  }
  EXPECT_TRUE(any_nonzero);
}

TEST(RelativePoseResidual, ZeroRawResidualStaysZeroRegardlessOfSqrtInformation) {
  Pose3 pose_i;
  Pose3 measured_relative;
  measured_relative.translation = Eigen::Vector3d(0.2, 0.1, -0.1);
  const Pose3 pose_j = pose_i * measured_relative;

  Eigen::Matrix<double, 6, 6> sqrt_information = Eigen::Matrix<double, 6, 6>::Zero();
  sqrt_information(0, 1) = 5.0;  // deliberately non-diagonal/coupled
  sqrt_information(3, 4) = -2.0;
  sqrt_information.diagonal().setConstant(3.0);

  RelativePoseResidual residual(measured_relative, sqrt_information);
  const auto params_i = pose_i.ToParameterBlock();
  const auto params_j = pose_j.ToParameterBlock();
  std::vector<const double*> parameters{params_i.data(), params_j.data()};

  std::array<double, 6> out{};
  ASSERT_TRUE(residual.Evaluate(parameters, out.data(), nullptr));
  for (double v : out) EXPECT_NEAR(v, 0.0, 1e-9);
}

TEST(RelativePoseResidual, NonDiagonalSqrtInformationCouplesResidualComponents) {
  Pose3 pose_i;
  Pose3 measured_relative;  // identity: predicted == measured when pose_j == pose_i
  const Pose3 pose_j_offset = [] {
    Pose3 p;
    p.translation = Eigen::Vector3d(0.05, 0.0, 0.0);  // raw translation-x residual = 0.05, others 0
    return p;
  }();
  const Pose3 pose_j = pose_i * pose_j_offset;

  Eigen::Matrix<double, 6, 6> diagonal_only = Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix<double, 6, 6> coupled = Eigen::Matrix<double, 6, 6>::Identity();
  coupled(1, 0) = 4.0;  // translation-y whitened residual should now pick up translation-x's raw error

  const auto params_i = pose_i.ToParameterBlock();
  const auto params_j = pose_j.ToParameterBlock();
  std::vector<const double*> parameters{params_i.data(), params_j.data()};

  RelativePoseResidual diagonal_residual(measured_relative, diagonal_only);
  std::array<double, 6> diagonal_out{};
  ASSERT_TRUE(diagonal_residual.Evaluate(parameters, diagonal_out.data(), nullptr));
  EXPECT_NEAR(diagonal_out[1], 0.0, 1e-9);  // no coupling: y-component untouched by x's error

  RelativePoseResidual coupled_residual(measured_relative, coupled);
  std::array<double, 6> coupled_out{};
  ASSERT_TRUE(coupled_residual.Evaluate(parameters, coupled_out.data(), nullptr));
  EXPECT_NEAR(coupled_out[1], 4.0 * 0.05, 1e-9);  // coupling term pulls in the x residual
}
