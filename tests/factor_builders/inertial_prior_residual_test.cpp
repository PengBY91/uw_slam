#include "factor_builders/inertial_prior_residual.hpp"

#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

using uw::factor_builders::InertialPriorResidual;

namespace {

Eigen::Matrix<double, 9, 1> MakeTarget() {
  Eigen::Matrix<double, 9, 1> target;
  target << 0.0, 0.0, 0.0, 1.0e-4, -2.0e-4, 3.0e-4, 0.01, -0.02, 0.03;
  return target;
}

Eigen::Matrix<double, 9, 1> MakeSigma() {
  Eigen::Matrix<double, 9, 1> sigma;
  sigma << 0.05, 0.05, 0.05, 2.0e-4, 2.0e-4, 2.0e-4, 4.0e-3, 4.0e-3, 4.0e-3;
  return sigma;
}

Eigen::Matrix<double, 9, 1> MakeState() {
  Eigen::Matrix<double, 9, 1> state;
  state << 0.02, -0.03, 0.01, 3.0e-4, 1.0e-4, -1.0e-4, 0.02, -0.015, 0.05;
  return state;
}

}  // namespace

TEST(InertialPriorResidual, DeclaresOneNineDimensionalInertialBlock) {
  const auto residual = InertialPriorResidual::Create(MakeTarget(), MakeSigma());
  ASSERT_NE(residual, nullptr);
  EXPECT_EQ(residual->ResidualDim(), 9);
  ASSERT_EQ(residual->ParameterBlockSizes().size(), 1u);
  EXPECT_EQ(residual->ParameterBlockSizes()[0], 9);
}

TEST(InertialPriorResidual, IsZeroAtTheTarget) {
  const auto target = MakeTarget();
  const auto residual = InertialPriorResidual::Create(target, MakeSigma());
  ASSERT_NE(residual, nullptr);
  Eigen::Matrix<double, 9, 1> values;
  ASSERT_TRUE(residual->Evaluate({target.data()}, values.data(), nullptr));
  EXPECT_NEAR(values.norm(), 0.0, 1e-15);
}

TEST(InertialPriorResidual, ScalesEachAxisByItsOwnSqrtInformation) {
  const auto target = MakeTarget();
  const auto sigma = MakeSigma();
  const auto state = MakeState();
  const auto residual = InertialPriorResidual::Create(target, sigma);
  ASSERT_NE(residual, nullptr);
  Eigen::Matrix<double, 9, 1> values;
  ASSERT_TRUE(residual->Evaluate({state.data()}, values.data(), nullptr));
  for (int i = 0; i < 9; ++i) {
    EXPECT_NEAR(values(i), (state(i) - target(i)) / sigma(i), 1e-12) << "row " << i;
  }
}

TEST(InertialPriorResidual, JacobianIsTheSqrtInformationDiagonalAndMatchesFiniteDifferences) {
  const auto target = MakeTarget();
  const auto sigma = MakeSigma();
  auto state = MakeState();
  const auto residual = InertialPriorResidual::Create(target, sigma);
  ASSERT_NE(residual, nullptr);

  Eigen::Matrix<double, 9, 1> values;
  Eigen::Matrix<double, 9, 9, Eigen::RowMajor> jacobian;
  std::vector<double*> jacobians{jacobian.data()};
  ASSERT_TRUE(residual->Evaluate({state.data()}, values.data(), &jacobians));

  Eigen::Matrix<double, 9, 9> expected = Eigen::Matrix<double, 9, 9>::Zero();
  for (int i = 0; i < 9; ++i) expected(i, i) = 1.0 / sigma(i);
  EXPECT_NEAR((jacobian - expected).cwiseAbs().maxCoeff(), 0.0, 1e-15);

  // Same central-difference cross-check the other residuals in this
  // directory carry, so the "obviously diagonal" claim is verified rather
  // than assumed.
  for (int column = 0; column < 9; ++column) {
    const double step = 1e-6 * std::max(1.0, std::abs(state(column)));
    Eigen::Matrix<double, 9, 1> plus;
    Eigen::Matrix<double, 9, 1> minus;
    auto perturbed = state;
    perturbed(column) = state(column) + step;
    ASSERT_TRUE(residual->Evaluate({perturbed.data()}, plus.data(), nullptr));
    perturbed(column) = state(column) - step;
    ASSERT_TRUE(residual->Evaluate({perturbed.data()}, minus.data(), nullptr));
    const Eigen::Matrix<double, 9, 1> numeric = (plus - minus) / (2.0 * step);
    for (int row = 0; row < 9; ++row) {
      EXPECT_NEAR(jacobian(row, column), numeric(row), 1e-6 / sigma(row))
          << "row " << row << " column " << column;
    }
  }
}

TEST(InertialPriorResidual, RejectsSigmasThatCannotDefineAPrior) {
  const auto target = MakeTarget();
  auto sigma = MakeSigma();
  EXPECT_NE(InertialPriorResidual::Create(target, sigma), nullptr);

  for (int i = 0; i < 9; ++i) {
    auto zero = MakeSigma();
    zero(i) = 0.0;
    EXPECT_EQ(InertialPriorResidual::Create(target, zero), nullptr) << "zero sigma on axis " << i;
    auto negative = MakeSigma();
    negative(i) = -1.0;
    EXPECT_EQ(InertialPriorResidual::Create(target, negative), nullptr)
        << "negative sigma on axis " << i;
    auto not_a_number = MakeSigma();
    not_a_number(i) = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(InertialPriorResidual::Create(target, not_a_number), nullptr)
        << "NaN sigma on axis " << i;
    auto infinite = MakeSigma();
    infinite(i) = std::numeric_limits<double>::infinity();
    EXPECT_EQ(InertialPriorResidual::Create(target, infinite), nullptr)
        << "infinite sigma on axis " << i;
  }
}

TEST(InertialPriorResidual, RejectsANonFiniteTarget) {
  auto target = MakeTarget();
  target(4) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(InertialPriorResidual::Create(target, MakeSigma()), nullptr);
}

TEST(InertialPriorResidual, RejectsAnEmptyParameterList) {
  const auto residual = InertialPriorResidual::Create(MakeTarget(), MakeSigma());
  ASSERT_NE(residual, nullptr);
  Eigen::Matrix<double, 9, 1> values;
  EXPECT_FALSE(residual->Evaluate({}, values.data(), nullptr));
}
