#include "frontends/sonar_cfar_frontend.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "frontends/dbscan.hpp"

namespace uw::frontends {

namespace {

Eigen::MatrixXf IntensityTensorToMatrix(const uw::domain::SonarFrame& frame) {
  const int rows = static_cast<int>(frame.num_ranges());
  const int cols = static_cast<int>(frame.num_beams());
  Eigen::MatrixXf mat(rows, cols);
  const auto& bytes = frame.intensity_tensor();
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      const std::size_t idx = static_cast<std::size_t>(r) * cols + c;
      mat(r, c) = (idx < bytes.size()) ? static_cast<float>(static_cast<uint8_t>(bytes[idx])) : 0.0f;
    }
  }
  return mat;
}

double RangeAtBin(const uw::domain::SonarFrame& frame, int row) {
  if (frame.range_bins_size() > row) return frame.range_bins(row);
  return frame.min_range() + row * frame.range_resolution();
}

class StableFnv1a64 {
 public:
  void AddBytes(const char* bytes, std::size_t size) {
    for (std::size_t index = 0; index < size; ++index) {
      hash_ ^= static_cast<unsigned char>(bytes[index]);
      hash_ *= 1099511628211ULL;
    }
  }

  void AddUint32(uint32_t value) {
    for (int shift = 0; shift < 32; shift += 8) AddByte(value >> shift);
  }

  void AddInt32(int32_t value) { AddUint32(static_cast<uint32_t>(value)); }

  void AddUint64(uint64_t value) {
    for (int shift = 0; shift < 64; shift += 8) AddByte(value >> shift);
  }

  void AddDouble(double value) {
    // +0 and -0 have identical behavior and therefore one canonical hash.
    if (value == 0.0) value = 0.0;
    uint64_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value) && std::numeric_limits<double>::is_iec559,
                  "config hashing requires an IEEE-754 64-bit double");
    std::memcpy(&bits, &value, sizeof(bits));
    AddUint64(bits);
  }

  uint64_t value() const { return hash_; }

 private:
  void AddByte(uint64_t value) {
    hash_ ^= static_cast<uint8_t>(value & 0xffU);
    hash_ *= 1099511628211ULL;
  }

  uint64_t hash_ = 14695981039346656037ULL;
};

std::string HashActiveConfig(const SonarCfarFrontendParams& params) {
  StableFnv1a64 hash;
  constexpr char kFormatVersion[] = "sonar_cfar_frontend_params_v1";
  // sizeof intentionally includes the terminating NUL as a version delimiter.
  hash.AddBytes(kFormatVersion, sizeof(kFormatVersion));
  hash.AddInt32(params.cfar.num_training_cells);
  hash.AddInt32(params.cfar.num_guard_cells);
  hash.AddDouble(params.cfar.probability_false_alarm);
  hash.AddInt32(params.cfar.rank);
  hash.AddInt32(static_cast<int32_t>(params.cfar_variant));
  hash.AddUint32(params.detector_threshold);
  hash.AddDouble(params.dbscan_eps_m);
  hash.AddInt32(params.dbscan_min_samples);
  hash.AddDouble(params.default_range_sigma_m);
  hash.AddDouble(params.default_bearing_sigma_rad);
  hash.AddDouble(params.health.max_background_noise_mean);
  hash.AddDouble(params.health.max_false_alarm_density);
  hash.AddUint64(params.health.min_valid_measurements);
  hash.AddDouble(params.health.max_processing_latency_ms);
  // expected_config_hash is the deployment-side value compared against this
  // active hash; including it here would make the comparison self-referential.

  constexpr char kHexDigits[] = "0123456789abcdef";
  std::string encoded(16, '0');
  const uint64_t value = hash.value();
  for (std::size_t nibble = 0; nibble < encoded.size(); ++nibble) {
    encoded[encoded.size() - 1 - nibble] = kHexDigits[(value >> (4 * nibble)) & 0x0fU];
  }
  return encoded;
}

double Percentile(const std::deque<double>& samples, double quantile) {
  std::vector<double> values(samples.begin(), samples.end());
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const auto index = static_cast<std::size_t>(
      std::ceil(quantile * static_cast<double>(values.size())) - 1.0);
  return values[std::min(index, values.size() - 1)];
}

}  // namespace

SonarCfarFrontend::SonarCfarFrontend(SonarCfarFrontendParams params, SteadyClockNow now)
    : params_(std::move(params)),
      detector_(params_.cfar),
      now_(std::move(now)),
      active_config_hash_(HashActiveConfig(params_)) {}

void SonarCfarFrontend::FinishProcessing(std::chrono::steady_clock::time_point start) {
  const auto elapsed = std::chrono::duration<double, std::milli>(now_() - start).count();
  processing_latencies_ms_.push_back(std::max(0.0, elapsed));
  if (processing_latencies_ms_.size() > kProcessingLatencyWindowSize) {
    processing_latencies_ms_.pop_front();
  }
}

uw::domain::HypothesisSet SonarCfarFrontend::ProcessSonarFrame(const uw::domain::SonarFrame& frame) {
  const auto processing_start = now_();
  uw::domain::HypothesisSet hypothesis_set;
  ++frames_processed_;
  last_background_noise_mean_ = 0.0;
  last_false_alarm_density_ = 0.0;
  last_valid_measurement_count_ = 0;

  if (!uw::domain::IsAzimuthAscending(frame)) {
    ++frames_rejected_;
    hypothesis_set.set_ambiguity_reason(
        "azimuth_angles not strictly ascending; rejected at the boundary instead of "
        "silently remapping (see SonarFrame.azimuth_angles contract)");
    hypothesis_set.set_out_of_distribution(true);
    FinishProcessing(processing_start);
    return hypothesis_set;
  }

  const Eigen::MatrixXf intensity = IntensityTensorToMatrix(frame);
  const auto mask = detector_.Detect(intensity, params_.cfar_variant);

  double background_sum = 0.0;
  uint64_t background_count = 0;
  for (int row = 0; row < mask.rows(); ++row) {
    for (int col = 0; col < mask.cols(); ++col) {
      if (mask(row, col) == 0) {
        background_sum += intensity(row, col);
        ++background_count;
      }
    }
  }
  if (background_count > 0) {
    last_background_noise_mean_ = background_sum / static_cast<double>(background_count);
  }

  // First-contact extraction per bearing column: nearest range bin (lowest
  // row index, per this repo's range_bins-ascending convention) with a
  // detection above both the CFAR mask and the additional intensity floor.
  // Semantically equivalent to imaging_sonar.py's extract_line_scan, direct
  // rather than via the rot90 index trick upstream uses (see header
  // comment).
  struct Detection {
    double range_m;
    double bearing_rad;
    double intensity;
  };
  std::vector<Detection> detections;
  for (int col = 0; col < mask.cols(); ++col) {
    for (int row = 0; row < mask.rows(); ++row) {
      if (mask(row, col) != 0 && intensity(row, col) > static_cast<float>(params_.detector_threshold)) {
        detections.push_back({RangeAtBin(frame, row), frame.azimuth_angles(col), intensity(row, col)});
        break;
      }
    }
  }

  if (detections.empty()) {
    ++frames_with_no_detections_;
    FinishProcessing(processing_start);
    return hypothesis_set;
  }

  std::vector<Eigen::Vector2d> plane_points;
  plane_points.reserve(detections.size());
  for (const auto& d : detections) {
    plane_points.emplace_back(d.range_m * std::cos(d.bearing_rad), d.range_m * std::sin(d.bearing_rad));
  }
  const auto labels = Dbscan(plane_points, params_.dbscan_eps_m, params_.dbscan_min_samples);

  std::map<int, std::vector<int>> clusters;  // cluster_id -> detection indices
  std::vector<int> noise_indices;
  for (int i = 0; i < static_cast<int>(labels.size()); ++i) {
    if (labels[i] == -1) {
      noise_indices.push_back(i);
    } else {
      clusters[labels[i]].push_back(i);
    }
  }
  last_false_alarm_density_ =
      static_cast<double>(noise_indices.size()) / static_cast<double>(detections.size());
  last_valid_measurement_count_ = clusters.size();
  if (clusters.empty()) ++frames_with_no_detections_;

  std::vector<std::pair<uw::domain::MeasurementEvidence, double>> ranked_candidates;
  for (auto& [cluster_id, indices] : clusters) {
    (void)cluster_id;
    double sum_range = 0.0, sum_bearing = 0.0, sum_intensity = 0.0;
    double min_range = std::numeric_limits<double>::infinity();
    double max_range = -std::numeric_limits<double>::infinity();
    double min_bearing = std::numeric_limits<double>::infinity();
    double max_bearing = -std::numeric_limits<double>::infinity();
    for (int idx : indices) {
      sum_range += detections[idx].range_m;
      sum_bearing += detections[idx].bearing_rad;
      sum_intensity += detections[idx].intensity;
      min_range = std::min(min_range, detections[idx].range_m);
      max_range = std::max(max_range, detections[idx].range_m);
      min_bearing = std::min(min_bearing, detections[idx].bearing_rad);
      max_bearing = std::max(max_bearing, detections[idx].bearing_rad);
    }
    const auto n = static_cast<double>(indices.size());

    uw::domain::SonarRangeBearing measurement;
    measurement.set_range_m(sum_range / n);
    measurement.set_bearing_rad(sum_bearing / n);
    measurement.set_range_sigma_m(params_.default_range_sigma_m);
    measurement.set_bearing_sigma_rad(params_.default_bearing_sigma_rad);
    *measurement.mutable_sonar_frame() = frame.header().sensor_frame();

    uw::domain::EvidenceId evidence_id;
    evidence_id.set_value("sonar_cfar_" + std::to_string(next_evidence_id_++));

    std::vector<uw::domain::ObservationId> sources{frame.header().observation_id()};
    // Larger clusters are ranked as more likely candidates — more
    // independent CFAR detections agreeing is stronger evidence.
    const double likelihood = n;
    auto evidence = uw::domain::MakeEvidence<uw::domain::SonarRangeBearing>(
        evidence_id, sources, measurement, /*noise_scale=*/1.0 / n, "sonar_cfar_frontend_v1");
    (*evidence.mutable_quality_features())["angular_extent_rad"] = max_bearing - min_bearing;
    (*evidence.mutable_quality_features())["range_extent_m"] = max_range - min_range;
    (*evidence.mutable_quality_features())["intensity_score"] = sum_intensity / n;
    (*evidence.mutable_quality_features())["cfar_score"] = likelihood;
    (*evidence.mutable_quality_features())["cluster_size"] = n;
    ranked_candidates.emplace_back(std::move(evidence), likelihood);
  }

  std::sort(ranked_candidates.begin(), ranked_candidates.end(),
            [](const auto& a, const auto& b) { return a.second > b.second; });

  for (auto& [evidence, likelihood] : ranked_candidates) {
    *hypothesis_set.add_candidates() = evidence;
    hypothesis_set.add_calibrated_likelihoods(likelihood);
  }
  if (ranked_candidates.size() > 1) {
    hypothesis_set.set_ambiguity_reason("multiple DBSCAN clusters detected in this ping");
  }

  for (int idx : noise_indices) {
    uw::domain::SonarRangeBearing measurement;
    measurement.set_range_m(detections[idx].range_m);
    measurement.set_bearing_rad(detections[idx].bearing_rad);
    uw::domain::EvidenceId evidence_id;
    evidence_id.set_value("sonar_cfar_noise_" + std::to_string(next_evidence_id_++));
    std::vector<uw::domain::ObservationId> sources{frame.header().observation_id()};
    *hypothesis_set.add_rejected_candidates() = uw::domain::MakeEvidence<uw::domain::SonarRangeBearing>(
        evidence_id, sources, measurement, 0.0, "sonar_cfar_frontend_v1");
  }

  FinishProcessing(processing_start);
  return hypothesis_set;
}

uw::domain::HealthReport SonarCfarFrontend::Health() const {
  uw::domain::HealthReport report;
  report.set_component_id("sonar_cfar_frontend");
  report.set_background_noise_mean(last_background_noise_mean_);
  report.set_false_alarm_density(last_false_alarm_density_);
  report.set_valid_measurement_count(last_valid_measurement_count_);
  const bool config_hash_consistent = params_.expected_config_hash.empty() ||
                                      params_.expected_config_hash == active_config_hash_;
  report.set_config_hash_consistent(config_hash_consistent);
  report.set_latency_p50_ms(Percentile(processing_latencies_ms_, 0.50));
  report.set_latency_p95_ms(Percentile(processing_latencies_ms_, 0.95));
  report.set_latency_p99_ms(Percentile(processing_latencies_ms_, 0.99));

  if (frames_processed_ == 0) {
    report.set_status(uw::domain::HealthReport::STATUS_UNSPECIFIED);
  } else {
    report.set_status(uw::domain::HealthReport::STATUS_HEALTHY);
    if (!config_hash_consistent) {
      report.set_status(uw::domain::HealthReport::STATUS_SUSPECT);
      report.set_reason_code("sonar_config_hash_mismatch");
    } else if (last_background_noise_mean_ > params_.health.max_background_noise_mean) {
      report.set_status(uw::domain::HealthReport::STATUS_SUSPECT);
      report.set_reason_code("sonar_background_noise_high");
    } else if (last_false_alarm_density_ > params_.health.max_false_alarm_density) {
      report.set_status(uw::domain::HealthReport::STATUS_SUSPECT);
      report.set_reason_code("sonar_false_alarm_density_high");
    } else if (last_valid_measurement_count_ < params_.health.min_valid_measurements) {
      report.set_status(uw::domain::HealthReport::STATUS_SUSPECT);
      report.set_reason_code("sonar_no_valid_measurements");
    } else if (report.latency_p95_ms() > params_.health.max_processing_latency_ms) {
      report.set_status(uw::domain::HealthReport::STATUS_SUSPECT);
      report.set_reason_code("sonar_processing_latency_high");
    }
  }
  if (frames_processed_ > 0) {
    report.set_input_valid_rate(1.0 - static_cast<double>(frames_rejected_) /
                                          static_cast<double>(frames_processed_));
    report.set_valid_domain_rate(1.0 - static_cast<double>(frames_with_no_detections_) /
                                            static_cast<double>(frames_processed_));
  }
  return report;
}

}  // namespace uw::frontends
