#pragma once

#include "sensor_models/camera_model.hpp"
#include "sensor_models/geometry.hpp"

namespace uw::frontends {

struct PosteriorDepthOptimizerParams {
  int iterations = 30;
  double search_radius_sigma = 3.0;
};

struct PosteriorDepthResult {
  bool valid = false;
  double depth_m = 0.0;
  double variance_m2 = 0.0;
  double range_residual_m = 0.0;
  double bearing_residual_rad = 0.0;
};

// Optimizes a scalar depth `d` at a fixed pixel:
//   min_d (d-d_o)^2/sigma_d^2 + (range(d)-rho)^2/sigma_rho^2 + (bearing(d)-theta)^2/sigma_theta^2
// where range(d)/bearing(d) come from UnprojectPixelToSonarRangeBearing at the fixed pixel.
// v1 uses plain squared residuals (Gaussian loss — Huber/Cauchy is a documented future
// enhancement) and a deterministic, bounded golden-section search over
// d in [d_o - k*sigma_d, d_o + k*sigma_d] (k = search_radius_sigma) — never unconstrained
// Gauss-Newton, so it cannot diverge, at the cost of assuming near-unimodality within that
// window. Posterior variance is a Laplace approximation (2/f''(d*), f'' via central finite
// difference). Returns valid=false if any input sigma/prior variance is <= 0 or the optimum
// is non-finite — callers must fall back to the optical prior in that case.
PosteriorDepthResult OptimizePosteriorDepth(double pixel_u, double pixel_v, double prior_depth_m,
                                            double prior_variance_m2, double sonar_range_m,
                                            double sonar_range_sigma_m, double sonar_bearing_rad,
                                            double sonar_bearing_sigma_rad,
                                            const uw::sensor_models::Pose3& camera_T_sonar,
                                            const uw::sensor_models::PinholeCamera& camera,
                                            const PosteriorDepthOptimizerParams& params);

}  // namespace uw::frontends
