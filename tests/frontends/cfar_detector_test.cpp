#include "frontends/cfar_detector.hpp"

#include <cmath>

#include <gtest/gtest.h>

using uw::frontends::CfarDetector;
using uw::frontends::CfarParams;
using uw::frontends::CfarVariant;

TEST(CfarDetector, FlagsAnIsolatedSpikeAboveFlatBackground) {
  CfarParams params;
  params.num_training_cells = 16;
  params.num_guard_cells = 4;
  params.probability_false_alarm = 1e-2;

  CfarDetector detector(params);

  const int rows = 40, cols = 4;
  Eigen::MatrixXf intensity = Eigen::MatrixXf::Constant(rows, cols, 5.0f);
  intensity(20, 1) = 200.0f;

  const auto mask_ca = detector.Detect(intensity, CfarVariant::kCA);
  const auto mask_soca = detector.Detect(intensity, CfarVariant::kSOCA);

  EXPECT_EQ(mask_ca(20, 1), 1);
  EXPECT_EQ(mask_soca(20, 1), 1);
  // Flat background elsewhere should not trigger.
  EXPECT_EQ(mask_ca(20, 0), 0);
  EXPECT_EQ(mask_soca(19, 1), 0);
}
