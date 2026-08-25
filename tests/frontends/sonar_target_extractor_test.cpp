#include "frontends/sonar_target_extractor.hpp"

#include <limits>

#include <gtest/gtest.h>

#include "frontends/sonar_cfar_frontend.hpp"

namespace {

uw::frontends::SonarCfarFrontendParams TestCfarParams() {
  uw::frontends::SonarCfarFrontendParams params;
  params.cfar.num_training_cells = 16;
  params.cfar.num_guard_cells = 4;
  params.cfar.probability_false_alarm = 1e-2;
  params.detector_threshold = 50;
  params.dbscan_eps_m = 0.15;
  params.dbscan_min_samples = 2;
  return params;
}

uw::domain::SonarFrame MakeTwoClusterSonarFrame() {
  constexpr int kNumRanges = 96;
  constexpr int kNumBeams = 64;
  uw::domain::SonarFrame frame;
  frame.mutable_header()->mutable_observation_id()->set_value("sonar_two_clusters");
  frame.mutable_header()->mutable_sensor_frame()->set_value("sonar_link");
  frame.mutable_header()->mutable_capture_time()->set_seconds(7);
  frame.set_num_ranges(kNumRanges);
  frame.set_num_beams(kNumBeams);
  frame.set_range_resolution(0.1f);
  frame.set_min_range(0.0f);
  frame.set_max_range(9.6f);
  frame.set_encoding(uw::domain::SonarFrame::ENCODING_UINT8_GRAY);
  for (int row = 0; row <= kNumRanges; ++row) frame.add_range_bins(0.1f * row);
  for (int col = 0; col < kNumBeams; ++col) {
    frame.add_azimuth_angles(-0.5f + static_cast<float>(col) / (kNumBeams - 1));
  }

  std::string bytes(kNumRanges * kNumBeams, static_cast<char>(5));
  // The higher-bearing cluster has more points, so SonarCfarFrontend ranks
  // it first by likelihood. The extractor must still reorder the emitted
  // detections by bearing rather than inheriting candidate rank order.
  for (const int col : {16, 17}) {
    bytes[static_cast<std::size_t>(30) * kNumBeams + col] = static_cast<char>(220);
  }
  for (const int col : {45, 46, 47}) {
    bytes[static_cast<std::size_t>(60) * kNumBeams + col] = static_cast<char>(220);
  }
  frame.set_intensity_tensor(bytes);
  return frame;
}

uw::domain::MeasurementEvidence MakeSonarCandidate(const std::string& observation_id,
                                                    const std::string& evidence_id,
                                                    double bearing_rad, double range_m,
                                                    double bearing_sigma_rad = 0.01,
                                                    double range_sigma_m = 0.05) {
  uw::domain::SonarRangeBearing measurement;
  measurement.set_bearing_rad(bearing_rad);
  measurement.set_range_m(range_m);
  measurement.set_bearing_sigma_rad(bearing_sigma_rad);
  measurement.set_range_sigma_m(range_sigma_m);
  uw::domain::ObservationId source;
  source.set_value(observation_id);
  uw::domain::EvidenceId id;
  id.set_value(evidence_id);
  auto evidence =
      uw::domain::MakeEvidence(id, {source}, measurement, 1.0, "sonar_cfar_frontend_v1");
  (*evidence.mutable_quality_features())["cfar_score"] = 2.0;
  return evidence;
}

}  // namespace

TEST(SonarTargetExtractor, ConvertsEveryAcceptedCluster) {
  const auto frame = MakeTwoClusterSonarFrame();
  uw::frontends::SonarCfarFrontend frontend(TestCfarParams());
  const auto hypotheses = frontend.ProcessSonarFrame(frame);
  const auto detections = uw::frontends::SonarTargetExtractor().Extract(hypotheses, frame);

  ASSERT_EQ(hypotheses.candidates_size(), 2);
  ASSERT_EQ(detections.size(), 2u);
  EXPECT_LT(detections[0].bearing_rad(), detections[1].bearing_rad());
  EXPECT_TRUE(detections[0].has_range());
  EXPECT_EQ(detections[0].source(), uw::domain::ASSIST_SOURCE_SONAR);
  EXPECT_EQ(detections[0].source_observation().value(), "sonar_two_clusters");
  EXPECT_EQ(detections[0].capture_time().seconds(), 7);
  ASSERT_EQ(detections[0].covariance_2x2_row_major_size(), 4);
  EXPECT_DOUBLE_EQ(detections[0].covariance_2x2_row_major(0), 0.0001);
  EXPECT_DOUBLE_EQ(detections[0].covariance_2x2_row_major(3), 0.0025);
  EXPECT_GT(detections[0].angular_extent_rad(), 0.0);
  EXPECT_GT(detections[0].intensity_score(), 0.0);
  EXPECT_NE(detections[0].quality_metrics().find("cfar_score"),
            detections[0].quality_metrics().end());
}

TEST(SonarTargetExtractor, SkipsCandidatesWithoutSonarRangeBearingPayload) {
  uw::domain::HypothesisSet hypotheses;
  hypotheses.add_candidates()->mutable_pressure_depth()->set_depth_m(3.0);
  const auto detections =
      uw::frontends::SonarTargetExtractor().Extract(hypotheses, MakeTwoClusterSonarFrame());
  EXPECT_TRUE(detections.empty());
}

TEST(SonarTargetExtractor, EqualBearingAndRangeUseCanonicalSourceTieBreak) {
  uw::domain::HypothesisSet hypotheses;
  *hypotheses.add_candidates() = MakeSonarCandidate("source_b", "evidence_equal", 0.25, 4.0);
  *hypotheses.add_candidates() = MakeSonarCandidate("source_a", "evidence_equal", 0.25, 4.0);

  const auto detections =
      uw::frontends::SonarTargetExtractor().Extract(hypotheses, MakeTwoClusterSonarFrame());

  ASSERT_EQ(detections.size(), 2u);
  EXPECT_EQ(detections[0].source_observation().value(), "source_a");
  EXPECT_EQ(detections[1].source_observation().value(), "source_b");
}

TEST(SonarTargetExtractor, SkipsEveryCandidateThatCouldEmitNonFiniteValues) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  uw::domain::HypothesisSet hypotheses;
  *hypotheses.add_candidates() = MakeSonarCandidate("valid", "valid", 0.1, 2.0);
  *hypotheses.add_candidates() = MakeSonarCandidate("nan_bearing", "nan_bearing", nan, 2.0);
  *hypotheses.add_candidates() = MakeSonarCandidate("inf_range", "inf_range", 0.1, inf);
  *hypotheses.add_candidates() = MakeSonarCandidate("nan_bearing_sigma", "nan_bearing_sigma",
                                                     0.1, 2.0, nan, 0.05);
  *hypotheses.add_candidates() =
      MakeSonarCandidate("inf_range_sigma", "inf_range_sigma", 0.1, 2.0, 0.01, inf);
  auto non_finite_quality = MakeSonarCandidate("nan_quality", "nan_quality", 0.1, 2.0);
  (*non_finite_quality.mutable_quality_features())["intensity_score"] = nan;
  *hypotheses.add_candidates() = std::move(non_finite_quality);

  const auto detections =
      uw::frontends::SonarTargetExtractor().Extract(hypotheses, MakeTwoClusterSonarFrame());

  ASSERT_EQ(detections.size(), 1u);
  EXPECT_EQ(detections[0].source_observation().value(), "valid");
}
