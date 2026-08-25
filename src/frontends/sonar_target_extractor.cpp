#include "frontends/sonar_target_extractor.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl_lite.h>

namespace uw::frontends {

namespace {

double QualityMetricOr(const uw::domain::MeasurementEvidence& evidence,
                       const std::string& name, double fallback) {
  const auto it = evidence.quality_features().find(name);
  return it == evidence.quality_features().end() ? fallback : it->second;
}

bool EndsWith(const std::string& value, const std::string& suffix) {
  return value.size() >= suffix.size() &&
         value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

bool RequiresNonNegativeValue(const std::string& name) {
  return name == "score" || EndsWith(name, "_score") || name == "extent" ||
         name.find("_extent_") != std::string::npos || EndsWith(name, "_extent") ||
         name == "count" || EndsWith(name, "_count") || name == "size" ||
         EndsWith(name, "_size");
}

bool HasValidQualityMetrics(const uw::domain::MeasurementEvidence& evidence) {
  return std::all_of(evidence.quality_features().begin(), evidence.quality_features().end(),
                     [](const auto& entry) {
                       return std::isfinite(entry.second) &&
                              (!RequiresNonNegativeValue(entry.first) || entry.second >= 0.0);
                     });
}

std::string CanonicalEvidenceBytes(const uw::domain::MeasurementEvidence& evidence) {
  std::string bytes;
  bool serialized = false;
  {
    google::protobuf::io::StringOutputStream stream(&bytes);
    google::protobuf::io::CodedOutputStream coded_stream(&stream);
    coded_stream.SetSerializationDeterministic(true);
    serialized = evidence.SerializeToCodedStream(&coded_stream);
  }
  if (!serialized) return {};
  return bytes;
}

struct ExtractedDetection {
  uw::domain::TargetDetection detection;
  std::string canonical_evidence;
};

}  // namespace

std::vector<uw::domain::TargetDetection> SonarTargetExtractor::Extract(
    const uw::domain::HypothesisSet& hypotheses,
    const uw::domain::SonarFrame& source_frame) const {
  std::vector<ExtractedDetection> extracted;
  extracted.reserve(static_cast<std::size_t>(hypotheses.candidates_size()));

  const auto& source_header = source_frame.header();
  constexpr int32_t kNanosPerSecond = 1'000'000'000;
  if (source_header.observation_id().value().empty() || !source_header.has_capture_time() ||
      source_header.capture_time().nanos() < 0 ||
      source_header.capture_time().nanos() >= kNanosPerSecond) {
    return {};
  }

  for (int index = 0; index < hypotheses.candidates_size(); ++index) {
    const auto& evidence = hypotheses.candidates(index);
    if (!uw::domain::HasPayload<uw::domain::SonarRangeBearing>(evidence)) continue;
    if (evidence.source_observations_size() != 1 ||
        evidence.source_observations(0).value() != source_header.observation_id().value()) {
      continue;
    }
    const auto& measurement =
        uw::domain::GetPayload<uw::domain::SonarRangeBearing>(evidence);
    if (!std::isfinite(measurement.bearing_rad()) || !std::isfinite(measurement.range_m()) ||
        !std::isfinite(measurement.bearing_sigma_rad()) ||
        !std::isfinite(measurement.range_sigma_m()) || measurement.range_m() < 0.0 ||
        measurement.bearing_sigma_rad() <= 0.0 || measurement.range_sigma_m() <= 0.0 ||
        !HasValidQualityMetrics(evidence)) {
      continue;
    }
    const double bearing_variance =
        measurement.bearing_sigma_rad() * measurement.bearing_sigma_rad();
    const double range_variance = measurement.range_sigma_m() * measurement.range_sigma_m();
    if (!std::isfinite(bearing_variance) || !std::isfinite(range_variance)) continue;

    uw::domain::TargetDetection detection;
    *detection.mutable_source_observation() = evidence.source_observations(0);
    *detection.mutable_capture_time() = source_header.capture_time();
    detection.set_class_label("sonar_target");
    const double cfar_score = QualityMetricOr(evidence, "cfar_score", 0.0);
    detection.set_confidence(cfar_score > 0.0 ? cfar_score / (cfar_score + 1.0) : 0.0);
    detection.set_bearing_rad(measurement.bearing_rad());
    detection.set_has_range(true);
    detection.set_range_m(measurement.range_m());
    detection.add_covariance_2x2_row_major(bearing_variance);
    detection.add_covariance_2x2_row_major(0.0);
    detection.add_covariance_2x2_row_major(0.0);
    detection.add_covariance_2x2_row_major(range_variance);
    detection.set_source(uw::domain::ASSIST_SOURCE_SONAR);
    for (const auto& [name, value] : evidence.quality_features()) {
      (*detection.mutable_quality_metrics())[name] = value;
    }
    detection.set_angular_extent_rad(QualityMetricOr(evidence, "angular_extent_rad", 0.0));
    detection.set_range_extent_m(QualityMetricOr(evidence, "range_extent_m", 0.0));
    detection.set_intensity_score(QualityMetricOr(evidence, "intensity_score", 0.0));
    extracted.push_back({std::move(detection), CanonicalEvidenceBytes(evidence)});
  }

  std::sort(extracted.begin(), extracted.end(), [](const auto& lhs, const auto& rhs) {
    if (lhs.detection.bearing_rad() != rhs.detection.bearing_rad()) {
      return lhs.detection.bearing_rad() < rhs.detection.bearing_rad();
    }
    if (lhs.detection.range_m() != rhs.detection.range_m()) {
      return lhs.detection.range_m() < rhs.detection.range_m();
    }
    if (lhs.detection.source_observation().value() !=
        rhs.detection.source_observation().value()) {
      return lhs.detection.source_observation().value() <
             rhs.detection.source_observation().value();
    }
    return lhs.canonical_evidence < rhs.canonical_evidence;
  });
  std::vector<uw::domain::TargetDetection> detections;
  detections.reserve(extracted.size());
  for (auto& item : extracted) detections.push_back(std::move(item.detection));
  return detections;
}

}  // namespace uw::frontends
