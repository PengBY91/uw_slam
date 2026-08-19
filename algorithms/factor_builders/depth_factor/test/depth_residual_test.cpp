#include "uw/factor_builders/depth_residual.hpp"

#include <array>

#include <gtest/gtest.h>

using uw::factor_builders::DepthResidual;

TEST(DepthResidual, ResidualAndJacobian) {
  DepthResidual residual(/*measured_depth_m=*/12.5, /*sqrt_information=*/3.0);
  std::array<double, 7> pose{0.0, 0.0, -12.0, 0.0, 0.0, 0.0, 1.0};
  const double* params = pose.data();
  std::vector<const double*> parameters{params};

  double out = 0.0;
  std::array<double, 7> jacobian{};
  double* jac_ptr = jacobian.data();
  std::vector<double*> jacobians{jac_ptr};
  ASSERT_TRUE(residual.Evaluate(parameters, &out, &jacobians));

  EXPECT_NEAR(out, 3.0 * (12.5 + (-12.0)), 1e-9);
  EXPECT_DOUBLE_EQ(jacobian[2], 3.0);
  for (int i : {0, 1, 3, 4, 5, 6}) {
    EXPECT_DOUBLE_EQ(jacobian[i], 0.0);
  }
}
