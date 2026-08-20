#include "uw/frontends/posterior_depth_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

#include "uw/sensor_models/sonar_arc_projector.hpp"

namespace uw::frontends {

namespace {

double GoldenSectionMinimize(const std::function<double(double)>& f, double lo, double hi,
                             int iterations) {
  const double phi = (std::sqrt(5.0) - 1.0) / 2.0;
  double a = lo, b = hi;
  double c = b - phi * (b - a);
  double d = a + phi * (b - a);
  for (int i = 0; i < iterations; ++i) {
    if (f(c) < f(d)) {
      b = d;
    } else {
      a = c;
    }
    c = b - phi * (b - a);
    d = a + phi * (b - a);
  }
  return (a + b) / 2.0;
}

}  // namespace

PosteriorDepthResult OptimizePosteriorDepth(double pixel_u, double pixel_v, double prior_depth_m,
                                            double prior_variance_m2, double sonar_range_m,
                                            double sonar_range_sigma_m, double sonar_bearing_rad,
                                            double sonar_bearing_sigma_rad,
                                            const uw::sensor_models::Pose3& camera_T_sonar,
                                            const uw::sensor_models::PinholeCamera& camera,
                                            const PosteriorDepthOptimizerParams& params) {
  PosteriorDepthResult result;
  if (prior_variance_m2 <= 0.0 || sonar_range_sigma_m <= 0.0 || sonar_bearing_sigma_rad <= 0.0) {
    return result;
  }

  const double sigma_d = std::sqrt(prior_variance_m2);
  auto cost = [&](double d) {
    const auto observed = uw::sensor_models::UnprojectPixelToSonarRangeBearing(
        pixel_u, pixel_v, d, camera_T_sonar, camera);
    const double range_residual = observed.range_m - sonar_range_m;
    const double bearing_residual = observed.bearing_rad - sonar_bearing_rad;
    const double prior_residual = d - prior_depth_m;
    return (prior_residual * prior_residual) / (sigma_d * sigma_d) +
           (range_residual * range_residual) / (sonar_range_sigma_m * sonar_range_sigma_m) +
           (bearing_residual * bearing_residual) / (sonar_bearing_sigma_rad * sonar_bearing_sigma_rad);
  };

  const double radius = params.search_radius_sigma * sigma_d;
  const double lo = std::max(1e-3, prior_depth_m - radius);
  const double hi = prior_depth_m + radius;
  if (!(hi > lo)) return result;

  const double d_star = GoldenSectionMinimize(cost, lo, hi, params.iterations);
  const double f_star = cost(d_star);
  if (!std::isfinite(d_star) || !std::isfinite(f_star)) return result;

  const double h = std::max(1e-6, 1e-4 * std::max(1.0, std::abs(d_star)));
  const double second_derivative = (cost(d_star + h) - 2.0 * f_star + cost(d_star - h)) / (h * h);
  if (!(second_derivative > 0.0) || !std::isfinite(second_derivative)) return result;

  const auto observed_at_star = uw::sensor_models::UnprojectPixelToSonarRangeBearing(
      pixel_u, pixel_v, d_star, camera_T_sonar, camera);

  result.valid = true;
  result.depth_m = d_star;
  result.variance_m2 = 2.0 / second_derivative;
  result.range_residual_m = observed_at_star.range_m - sonar_range_m;
  result.bearing_residual_rad = observed_at_star.bearing_rad - sonar_bearing_rad;
  return result;
}

}  // namespace uw::frontends
