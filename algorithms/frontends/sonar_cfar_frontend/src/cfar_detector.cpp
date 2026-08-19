#include "uw/frontends/cfar_detector.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

namespace uw::frontends {

namespace {

// Ported from CFAR.py's calc_WGN_pfa_GOSOCA_core.
double GosocaCore(double x, int ntc) {
  double temp = 0.0;
  const double half_ntc = ntc / 2.0;
  for (int k = 0; k < ntc / 2; ++k) {
    const double l1 = std::lgamma(half_ntc + k);
    const double l2 = std::lgamma(static_cast<double>(k + 1));
    const double l3 = std::lgamma(half_ntc);
    temp += std::exp(l1 - l2 - l3) * std::pow(2.0 + x / half_ntc, -static_cast<double>(k));
  }
  return temp * std::pow(2.0 + x / half_ntc, -half_ntc);
}

// Finds x in (0, hi] with f(x) == 0, assuming f is monotonically decreasing
// (true for the Pfa-vs-threshold-factor equations below, matching the
// physical intuition that a higher threshold factor implies a lower false
// alarm probability). Expands the bracket until a sign change is found,
// then bisects — a robust equivalent of CFAR.py's multi-start
// scipy.optimize.root over np.logspace(-2, 2, 10).
double SolveDecreasingRoot(const std::function<double(double)>& f, double lo = 1e-6,
                           double hi = 10.0) {
  double f_lo = f(lo);
  double f_hi = f(hi);
  int expand_iterations = 0;
  while (f_lo * f_hi > 0.0 && expand_iterations < 60) {
    hi *= 2.0;
    f_hi = f(hi);
    ++expand_iterations;
  }
  if (f_lo * f_hi > 0.0) return lo;  // could not bracket a root; caller should treat cautiously

  for (int i = 0; i < 100; ++i) {
    const double mid = 0.5 * (lo + hi);
    const double f_mid = f(mid);
    if (std::abs(f_mid) < 1e-12) return mid;
    if (f_lo * f_mid <= 0.0) {
      hi = mid;
      f_hi = f_mid;
    } else {
      lo = mid;
      f_lo = f_mid;
    }
  }
  return 0.5 * (lo + hi);
}

}  // namespace

CfarDetector::CfarDetector(CfarParams params) : params_(params) {
  if (params_.rank < 0) params_.rank = params_.num_training_cells / 2;

  const int ntc = params_.num_training_cells;
  const double pfa = params_.probability_false_alarm;

  threshold_factor_ca_ = ntc * (std::pow(pfa, -1.0 / ntc) - 1.0);

  threshold_factor_soca_ = SolveDecreasingRoot(
      [&](double x) { return GosocaCore(x, ntc) - pfa / 2.0; }, 1e-6, threshold_factor_ca_ * 4.0 + 1.0);

  threshold_factor_goca_ = SolveDecreasingRoot(
      [&](double x) {
        return std::pow(1.0 + x / (ntc / 2.0), -ntc / 2.0) - GosocaCore(x, ntc) - pfa / 2.0;
      },
      1e-6, threshold_factor_ca_ * 4.0 + 1.0);

  {
    const double l1 = std::lgamma(static_cast<double>(ntc + 1));
    const double l2 = std::lgamma(static_cast<double>(ntc - params_.rank + 1));
    threshold_factor_os_ = SolveDecreasingRoot(
        [&](double x) {
          const double l4 = std::lgamma(x + ntc - params_.rank + 1);
          const double l6 = std::lgamma(x + ntc + 1);
          return std::exp(l1 - l2 + l4 - l6) - pfa;
        },
        1e-6, threshold_factor_ca_ * 4.0 + 1.0);
  }
}

double CfarDetector::ThresholdFactor(CfarVariant variant) const {
  switch (variant) {
    case CfarVariant::kCA:
      return threshold_factor_ca_;
    case CfarVariant::kSOCA:
      return threshold_factor_soca_;
    case CfarVariant::kGOCA:
      return threshold_factor_goca_;
    case CfarVariant::kOS:
      return threshold_factor_os_;
  }
  return threshold_factor_ca_;
}

Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> CfarDetector::Detect(
    const Eigen::MatrixXf& intensity, CfarVariant variant) const {
  const int train_hs = params_.num_training_cells / 2;
  const int guard_hs = params_.num_guard_cells / 2;
  const double tau = ThresholdFactor(variant);
  const int rows = static_cast<int>(intensity.rows());
  const int cols = static_cast<int>(intensity.cols());

  Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> mask =
      Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic>::Zero(rows, cols);

  for (int col = 0; col < cols; ++col) {
    for (int row = train_hs + guard_hs; row < rows - train_hs - guard_hs; ++row) {
      switch (variant) {
        case CfarVariant::kCA: {
          float sum_train = 0.0f;
          for (int i = row - train_hs - guard_hs; i < row + train_hs + guard_hs + 1; ++i) {
            if (std::abs(i - row) > guard_hs) sum_train += intensity(i, col);
          }
          mask(row, col) = intensity(row, col) > tau * sum_train / (2.0 * train_hs);
          break;
        }
        case CfarVariant::kSOCA:
        case CfarVariant::kGOCA: {
          float leading_sum = 0.0f, lagging_sum = 0.0f;
          for (int i = row - train_hs - guard_hs; i < row + train_hs + guard_hs + 1; ++i) {
            if ((i - row) > guard_hs)
              lagging_sum += intensity(i, col);
            else if ((i - row) < -guard_hs)
              leading_sum += intensity(i, col);
          }
          const float sum_train = (variant == CfarVariant::kSOCA)
                                       ? std::min(leading_sum, lagging_sum)
                                       : std::max(leading_sum, lagging_sum);
          mask(row, col) = intensity(row, col) > tau * sum_train / train_hs;
          break;
        }
        case CfarVariant::kOS: {
          std::vector<float> train;
          train.reserve(static_cast<std::size_t>(2 * train_hs));
          for (int i = row - train_hs - guard_hs; i < row + train_hs + guard_hs + 1; ++i) {
            if (std::abs(i - row) > guard_hs) train.push_back(intensity(i, col));
          }
          std::nth_element(train.begin(), train.begin() + params_.rank, train.end());
          mask(row, col) = intensity(row, col) > tau * train[static_cast<std::size_t>(params_.rank)];
          break;
        }
      }
    }
  }
  return mask;
}

}  // namespace uw::frontends
