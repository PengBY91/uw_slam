// Ported from sonar_camera_reconstruction (MIT, Copyright (c) 2025 Ivana
// Collado) — see /NOTICE.
//   sonar_camera_reconstruction_pkg/scripts/CFAR.py (threshold-factor
//     derivation for CA/SOCA/GOCA/OS)
//   sonar_camera_reconstruction_pkg/src/.../cpp/cfar.cpp (the actual
//     per-pixel windowed test — already pure Eigen/C++, no pybind11 code
//     ported here, just the four `ca`/`soca`/`goca`/`os` loop bodies)
//
// Deliberately NOT ported: cfar.cpp operated on a full 2D image because
// upstream needed CFAR results in the same pixel grid it later remaps to
// Cartesian for its own point cloud. This frontend produces range-bearing
// evidence directly from polar space (see sonar_cfar_frontend.hpp) and
// never performs that remap, so only the detection test itself is ported.
#pragma once

#include <Eigen/Core>

namespace uw::frontends {

enum class CfarVariant { kCA, kSOCA, kGOCA, kOS };

struct CfarParams {
  int num_training_cells = 20;    // Ntc, must be even
  int num_guard_cells = 4;        // Ngc, must be even
  double probability_false_alarm = 1e-3;  // Pfa
  int rank = -1;                  // OS-CFAR rank; < 0 defaults to Ntc/2
};

class CfarDetector {
 public:
  explicit CfarDetector(CfarParams params);

  // `intensity` is [num_ranges x num_beams] (rows = range bin, cols =
  // beam), matching uw::domain::SonarFrame's layout. Returns a same-shaped
  // detection mask (1 = target, 0 = background); the border
  // (num_training_cells/2 + num_guard_cells/2) rows at each end of the
  // range axis are always 0 (no full window available there).
  Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> Detect(const Eigen::MatrixXf& intensity,
                                                                 CfarVariant variant) const;

 private:
  double ThresholdFactor(CfarVariant variant) const;

  CfarParams params_;
  double threshold_factor_ca_;
  double threshold_factor_soca_;
  double threshold_factor_goca_;
  double threshold_factor_os_;
};

}  // namespace uw::frontends
