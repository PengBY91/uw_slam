#include "frontends/sonar_cfar_frontend.hpp"

#include <algorithm>
#include <cmath>
#include <map>
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

}  // namespace

SonarCfarFrontend::SonarCfarFrontend(SonarCfarFrontendParams params)
    : params_(params), detector_(params.cfar) {}

uw::domain::HypothesisSet SonarCfarFrontend::ProcessSonarFrame(const uw::domain::SonarFrame& frame) {
  uw::domain::HypothesisSet hypothesis_set;
  ++frames_processed_;

  if (!uw::domain::IsAzimuthAscending(frame)) {
    ++frames_rejected_;
    hypothesis_set.set_ambiguity_reason(
        "azimuth_angles not strictly ascending; rejected at the boundary instead of "
        "silently remapping (see SonarFrame.azimuth_angles contract)");
    hypothesis_set.set_out_of_distribution(true);
    return hypothesis_set;
  }

  const Eigen::MatrixXf intensity = IntensityTensorToMatrix(frame);
  const auto mask = detector_.Detect(intensity, params_.cfar_variant);

  // First-contact extraction per bearing column: nearest range bin (lowest
  // row index, per this repo's range_bins-ascending convention) with a
  // detection above both the CFAR mask and the additional intensity floor.
  // Semantically equivalent to imaging_sonar.py's extract_line_scan, direct
  // rather than via the rot90 index trick upstream uses (see header
  // comment).
  struct Detection {
    double range_m;
    double bearing_rad;
  };
  std::vector<Detection> detections;
  for (int col = 0; col < mask.cols(); ++col) {
    for (int row = 0; row < mask.rows(); ++row) {
      if (mask(row, col) != 0 && intensity(row, col) > static_cast<float>(params_.detector_threshold)) {
        detections.push_back({RangeAtBin(frame, row), frame.azimuth_angles(col)});
        break;
      }
    }
  }

  if (detections.empty()) {
    ++frames_with_no_detections_;
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

  std::vector<std::pair<uw::domain::MeasurementEvidence, double>> ranked_candidates;
  for (auto& [cluster_id, indices] : clusters) {
    (void)cluster_id;
    double sum_range = 0.0, sum_bearing = 0.0;
    for (int idx : indices) {
      sum_range += detections[idx].range_m;
      sum_bearing += detections[idx].bearing_rad;
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

  return hypothesis_set;
}

uw::domain::HealthReport SonarCfarFrontend::Health() const {
  uw::domain::HealthReport report;
  report.set_component_id("sonar_cfar_frontend");
  report.set_status(frames_processed_ == 0 ? uw::domain::HealthReport::STATUS_UNSPECIFIED
                                            : uw::domain::HealthReport::STATUS_HEALTHY);
  if (frames_processed_ > 0) {
    report.set_input_valid_rate(1.0 - static_cast<double>(frames_rejected_) /
                                          static_cast<double>(frames_processed_));
    report.set_valid_domain_rate(1.0 - static_cast<double>(frames_with_no_detections_) /
                                            static_cast<double>(frames_processed_));
  }
  return report;
}

}  // namespace uw::frontends
