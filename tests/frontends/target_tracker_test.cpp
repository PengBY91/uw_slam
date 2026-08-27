#include "frontends/target_tracker.hpp"

#include <cmath>
#include <algorithm>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <Eigen/Eigenvalues>

namespace {

uw::frontends::TargetMeasurement Measurement(
    std::string id, double time_s, double bearing_rad, std::optional<double> range_m,
    std::optional<uw::domain::AssistSource> source = std::nullopt,
    std::string class_label = "target") {
  uw::frontends::TargetMeasurement measurement;
  measurement.corrected_time_s = time_s;
  measurement.class_label = std::move(class_label);
  measurement.confidence = 0.9;
  measurement.bearing_rad = bearing_rad;
  measurement.range_m = range_m;
  measurement.covariance.setZero();
  measurement.covariance(0, 0) = 0.01 * 0.01;
  measurement.covariance(1, 1) = range_m ? 0.1 * 0.1 : 1000.0;
  measurement.sources = {
      source.value_or(range_m ? uw::domain::ASSIST_SOURCE_SONAR
                              : uw::domain::ASSIST_SOURCE_VISUAL)};
  uw::domain::ObservationId observation;
  observation.set_value(std::move(id));
  measurement.observation_ids = {observation};
  return measurement;
}

uw::frontends::TargetMeasurement FusedMeasurement(
    std::string visual_id, std::string sonar_id, double time_s,
    double bearing_rad, double range_m, std::string class_label = "target") {
  auto measurement = Measurement(std::move(visual_id), time_s, bearing_rad,
                                 range_m, uw::domain::ASSIST_SOURCE_SONAR,
                                 std::move(class_label));
  measurement.sources = {uw::domain::ASSIST_SOURCE_VISUAL,
                         uw::domain::ASSIST_SOURCE_SONAR};
  uw::domain::ObservationId sonar_observation;
  sonar_observation.set_value(std::move(sonar_id));
  measurement.observation_ids.push_back(std::move(sonar_observation));
  return measurement;
}

uw::frontends::TargetTrackerParams Params() {
  uw::frontends::TargetTrackerParams params;
  params.association_mahalanobis_sq = 25.0;
  params.confirm_hits = 2;
  params.degraded_misses = 3;
  params.stale_after_s = 0.5;
  params.max_prediction_dt_s = 0.5;
  params.bearing_acceleration_noise = 0.05;
  params.range_acceleration_noise = 0.5;
  params.merge_bearing_threshold_rad = 0.03;
  params.merge_range_threshold_m = 0.30;
  return params;
}

}  // namespace

TEST(TargetTracker, ConfirmsOnSecondHitAndStalesOnlyAfterExactFiveHundredMilliseconds) {
  uw::frontends::TargetTracker tracker(Params());
  ASSERT_TRUE(tracker.Update({Measurement("v-1", 10.0, 0.1, std::nullopt)}, 10.0));
  auto tracks = tracker.Tracks(10.0);
  ASSERT_EQ(tracks.size(), 1u);
  EXPECT_EQ(tracks[0].status, uw::domain::TARGET_TRACK_STATUS_TENTATIVE);

  ASSERT_TRUE(tracker.Update({Measurement("s-2", 10.1, 0.1, 5.0,
                                         uw::domain::ASSIST_SOURCE_SONAR)}, 10.1));
  tracks = tracker.Tracks(10.1);
  ASSERT_EQ(tracks.size(), 1u);
  EXPECT_EQ(tracks[0].status, uw::domain::TARGET_TRACK_STATUS_CONFIRMED);
  EXPECT_TRUE(tracks[0].range_observable);
  EXPECT_EQ(tracks[0].sources, (std::vector<uw::domain::AssistSource>{
                                   uw::domain::ASSIST_SOURCE_VISUAL,
                                   uw::domain::ASSIST_SOURCE_SONAR}));
  ASSERT_EQ(tracks[0].observation_ids.size(), 2u);
  EXPECT_EQ(tracks[0].observation_ids[0].value(), "s-2");
  EXPECT_EQ(tracks[0].observation_ids[1].value(), "v-1");

  EXPECT_EQ(tracker.Tracks(10.6)[0].status,
            uw::domain::TARGET_TRACK_STATUS_CONFIRMED);
  EXPECT_EQ(tracker.Tracks(10.601)[0].status,
            uw::domain::TARGET_TRACK_STATUS_STALE);
}

TEST(TargetTracker, UsesCorrectedCaptureWatermarkWhenProcessingHasFixedLatency) {
  uw::frontends::TargetTracker tracker(Params());
  ASSERT_TRUE(tracker.Update({Measurement("delayed-1", 9.8, 0.0, 5.0)}, 10.0));
  ASSERT_TRUE(tracker.Update({Measurement("delayed-2", 9.9, 0.01, 5.0)}, 10.1));
  const auto tracks = tracker.Tracks(10.1);
  ASSERT_EQ(tracks.size(), 1u);
  EXPECT_EQ(tracks[0].status, uw::domain::TARGET_TRACK_STATUS_CONFIRMED);
  EXPECT_DOUBLE_EQ(tracks[0].last_capture_time_s, 9.9);
}

TEST(TargetTracker, BearingOnlyTrackNeverFabricatesRange) {
  uw::frontends::TargetTracker tracker(Params());
  ASSERT_TRUE(tracker.Update({Measurement("v", 1.0, -0.2, std::nullopt)}, 1.0));
  const auto tracks = tracker.Tracks(1.2);
  ASSERT_EQ(tracks.size(), 1u);
  EXPECT_FALSE(tracks[0].range_observable);
  EXPECT_TRUE(std::isfinite(tracks[0].state[0]));
  EXPECT_TRUE(tracks[0].covariance.allFinite());
  const auto proto = tracks[0].ToProto(1.2);
  ASSERT_TRUE(proto.has_value());
  EXPECT_FALSE(proto->has_range_m());
  const auto set = tracker.ToProtoSet(1.2);
  ASSERT_TRUE(set.has_value());
  ASSERT_EQ(set->tracks_size(), 1);
  EXPECT_FALSE(set->tracks(0).has_range_m());
}

TEST(TargetTracker, ProtoExportFailsClosedForUnsafeTimeAndTrackInvariants) {
  uw::frontends::TargetTracker tracker(Params());
  ASSERT_TRUE(tracker.Update({Measurement("wire", 1.0, 0.1, 4.0,
                                         uw::domain::ASSIST_SOURCE_SONAR)}, 1.0));
  const auto valid = tracker.Tracks(1.1)[0];
  const auto valid_proto = valid.ToProto(1.1);
  ASSERT_TRUE(valid_proto.has_value());
  EXPECT_TRUE(valid_proto->has_range_m());

  auto invalid = valid;
  invalid.first_capture_time_s = 1.0e300;
  EXPECT_FALSE(invalid.ToProto(1.1).has_value());
  invalid = valid;
  invalid.covariance(0, 1) = 1.0;
  EXPECT_FALSE(invalid.ToProto(1.1).has_value());
  invalid = valid;
  invalid.track_id.clear();
  EXPECT_FALSE(invalid.ToProto(1.1).has_value());
  invalid = valid;
  invalid.sources = {uw::domain::ASSIST_SOURCE_ACOUSTIC_OPTIC};
  EXPECT_FALSE(invalid.ToProto(1.1).has_value());
  invalid = valid;
  invalid.sources = {uw::domain::ASSIST_SOURCE_VISUAL};
  EXPECT_FALSE(invalid.ToProto(1.1).has_value());
  EXPECT_FALSE(valid.ToProto(0.9).has_value());

  uw::frontends::TargetTracker huge_time_tracker(Params());
  EXPECT_FALSE(huge_time_tracker.Update(
      {Measurement("huge-time", 1.0e300, 0.0, 4.0)}, 1.0e300));
}

TEST(TargetTracker, SonarOnlyBirthHasObservableRangeAndJosephPsdCovariance) {
  uw::frontends::TargetTracker tracker(Params());
  ASSERT_TRUE(tracker.Update({Measurement("s", 2.0, 0.2, 7.0,
                                         uw::domain::ASSIST_SOURCE_SONAR)}, 2.0));
  ASSERT_TRUE(tracker.Update(
      {FusedMeasurement("v-f", "s-f", 2.1, 0.21, 7.1)}, 2.1));
  const auto track = tracker.Tracks(2.1)[0];
  EXPECT_TRUE(track.range_observable);
  EXPECT_TRUE(track.covariance.isApprox(track.covariance.transpose(), 1e-12));
  EXPECT_GE(Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d>(track.covariance)
                .eigenvalues().minCoeff(),
            -1e-10);
}

TEST(TargetTracker, FirstCorrelatedRangeUsesFullTwoDimensionalJosephUpdate) {
  uw::frontends::TargetTracker tracker(Params());
  const auto visual = Measurement("visual-birth", 2.0, 0.20, std::nullopt);
  ASSERT_TRUE(tracker.Update({visual}, 2.0));

  auto fused = FusedMeasurement("visual-range", "sonar-range", 2.0,
                                0.25, 7.0);
  fused.covariance << 0.04, 0.018,
                      0.018, 0.09;

  Eigen::Vector2d prior_state(visual.bearing_rad, *fused.range_m);
  Eigen::Matrix2d prior_covariance = Eigen::Matrix2d::Zero();
  prior_covariance(0, 0) = visual.covariance(0, 0);
  prior_covariance(1, 1) = 1.0e6;
  const Eigen::Matrix2d prior_information =
      prior_covariance.ldlt().solve(Eigen::Matrix2d::Identity());
  const Eigen::Matrix2d measurement_information =
      fused.covariance.ldlt().solve(Eigen::Matrix2d::Identity());
  const Eigen::Matrix2d expected_position_covariance =
      (prior_information + measurement_information)
          .ldlt()
          .solve(Eigen::Matrix2d::Identity());
  const Eigen::Vector2d expected_position = expected_position_covariance *
      (prior_information * prior_state +
       measurement_information *
           Eigen::Vector2d(fused.bearing_rad, *fused.range_m));

  ASSERT_TRUE(tracker.Update({fused}, 2.0));
  const auto tracks = tracker.Tracks(2.0);
  ASSERT_EQ(tracks.size(), 1u);
  const auto& track = tracks[0];
  EXPECT_TRUE(track.range_observable);
  EXPECT_TRUE(track.state.head<2>().isApprox(expected_position, 1e-10));
  EXPECT_TRUE((track.covariance.topLeftCorner<2, 2>().isApprox(
      expected_position_covariance, 1e-10)));
  EXPECT_NEAR(track.covariance(0, 1),
              expected_position_covariance(0, 1), 1e-12);
  EXPECT_GT(std::abs(track.covariance(0, 1)), 1e-8);
  EXPECT_TRUE(track.covariance.isApprox(track.covariance.transpose(), 1e-12));
  EXPECT_GE(Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d>(track.covariance)
                .eigenvalues().minCoeff(),
            -1e-10);
}

TEST(TargetTracker, RejectsInvalidSourceRangeAndProvenanceBatchAtomically) {
  uw::frontends::TargetTracker tracker(Params());
  ASSERT_TRUE(tracker.Update(
      {Measurement("seed", 1.0, 0.0, std::nullopt)}, 1.0));
  const auto before = tracker.Tracks(1.0);

  auto visual_with_range = Measurement(
      "visual-range", 1.1, 0.0, 4.0, uw::domain::ASSIST_SOURCE_VISUAL);
  auto sonar_without_range = Measurement(
      "sonar-bearing", 1.1, 0.0, std::nullopt,
      uw::domain::ASSIST_SOURCE_SONAR);
  auto fused_one_observation = FusedMeasurement(
      "fused-v", "fused-s", 1.1, 0.0, 4.0);
  fused_one_observation.observation_ids.pop_back();
  auto aggregate_source = Measurement(
      "aggregate", 1.1, 0.0, 4.0,
      uw::domain::ASSIST_SOURCE_ACOUSTIC_OPTIC);

  EXPECT_FALSE(tracker.Update(
      {visual_with_range, sonar_without_range, fused_one_observation,
       aggregate_source},
      1.1));
  const auto unchanged = tracker.Tracks(1.0);
  ASSERT_EQ(unchanged.size(), before.size());
  EXPECT_TRUE(unchanged[0].state.isApprox(before[0].state));
  EXPECT_TRUE(unchanged[0].covariance.isApprox(before[0].covariance));

  // The rejected processing time must not advance either watermark.
  EXPECT_TRUE(tracker.Update(
      {Measurement("valid-after-reject", 1.05, 0.01, std::nullopt)}, 1.05));
}

TEST(TargetTracker, EmptyBatchCommitsPredictionAndCovarianceWithoutDoublePrediction) {
  uw::frontends::TargetTracker committed(Params());
  uw::frontends::TargetTracker uncommitted(Params());
  const auto seed = Measurement("seed", 3.0, 0.0, 5.0);
  ASSERT_TRUE(committed.Update({seed}, 3.0));
  ASSERT_TRUE(uncommitted.Update({seed}, 3.0));

  ASSERT_TRUE(committed.Update({}, 3.2));
  const auto committed_view = committed.Tracks(3.4);
  const auto single_step_view = uncommitted.Tracks(3.4);
  ASSERT_EQ(committed_view.size(), 1u);
  ASSERT_EQ(single_step_view.size(), 1u);
  EXPECT_TRUE(committed_view[0].state.isApprox(
      single_step_view[0].state, 1e-12));
  EXPECT_TRUE(committed_view[0].covariance.isApprox(
      single_step_view[0].covariance, 1e-12));

  const auto at_commit = committed.Tracks(3.2)[0];
  ASSERT_TRUE(committed.Update({}, 3.2));
  const auto repeated_same_time = committed.Tracks(3.2)[0];
  EXPECT_TRUE(repeated_same_time.state.isApprox(at_commit.state, 1e-12));
  EXPECT_TRUE(repeated_same_time.covariance.isApprox(at_commit.covariance, 1e-12));
}

TEST(TargetTracker, EmptyBatchPredictionPreservesCorrectedCaptureTimeDomain) {
  uw::frontends::TargetTracker with_disappearance(Params());
  uw::frontends::TargetTracker direct_measurement(Params());
  for (auto* tracker : {&with_disappearance, &direct_measurement}) {
    ASSERT_TRUE(tracker->Update(
        {Measurement("motion-1", 9.7, 0.0, 5.0)}, 9.9));
    ASSERT_TRUE(tracker->Update(
        {Measurement("motion-2", 9.8, 0.05, 5.0)}, 10.0));
  }

  ASSERT_TRUE(with_disappearance.Update({}, 10.1));
  ASSERT_TRUE(with_disappearance.Update(
      {Measurement("motion-3", 9.9, 0.10, 5.0)}, 10.1));
  ASSERT_TRUE(direct_measurement.Update(
      {Measurement("motion-3", 9.9, 0.10, 5.0)}, 10.1));

  const auto disappeared = with_disappearance.Tracks(10.1);
  const auto direct = direct_measurement.Tracks(10.1);
  ASSERT_EQ(disappeared.size(), 1u);
  ASSERT_EQ(direct.size(), 1u);
  EXPECT_TRUE(disappeared[0].state.isApprox(direct[0].state, 1e-10));
  EXPECT_TRUE(
      disappeared[0].covariance.isApprox(direct[0].covariance, 1e-10));
}

TEST(TargetTracker, RejectsCaptureBehindCommittedStateHorizonAtomicallyAndRecovers) {
  uw::frontends::TargetTracker tracker(Params());
  ASSERT_TRUE(tracker.Update(
      {Measurement("seed", 9.8, 0.0, 5.0)}, 10.0));
  ASSERT_TRUE(tracker.Update({}, 10.1));
  const auto before = tracker.Tracks(10.1);
  ASSERT_EQ(before.size(), 1u);

  const auto late = Measurement("reusable", 9.85, 0.01, 5.0);
  EXPECT_FALSE(tracker.Update({late}, 10.2));
  const auto unchanged = tracker.Tracks(10.1);
  ASSERT_EQ(unchanged.size(), 1u);
  EXPECT_TRUE(unchanged[0].state.isApprox(before[0].state, 1e-12));
  EXPECT_TRUE(unchanged[0].covariance.isApprox(before[0].covariance, 1e-12));
  EXPECT_EQ(unchanged[0].status, before[0].status);
  ASSERT_EQ(unchanged[0].observation_ids.size(),
            before[0].observation_ids.size());
  EXPECT_EQ(unchanged[0].observation_ids[0].value(),
            before[0].observation_ids[0].value());

  // Reusing the rejected observation ID and the earlier processing time proves
  // neither provenance nor the processing watermark advanced on rejection.
  EXPECT_TRUE(tracker.Update(
      {Measurement("reusable", 9.9, 0.01, 5.0)}, 10.1));
  const auto recovered = tracker.Tracks(10.1);
  ASSERT_EQ(recovered.size(), 1u);
  EXPECT_EQ(recovered[0].status,
            uw::domain::TARGET_TRACK_STATUS_CONFIRMED);
  EXPECT_DOUBLE_EQ(recovered[0].last_capture_time_s, 9.9);
}

TEST(TargetTracker, ThreeMissesDegradeAndLaterQueryStalesWithoutAnotherUpdate) {
  uw::frontends::TargetTracker tracker(Params());
  ASSERT_TRUE(tracker.Update({Measurement("a", 4.0, 0.0, 4.0)}, 4.0));
  ASSERT_TRUE(tracker.Update({Measurement("b", 4.1, 0.0, 4.0)}, 4.1));
  ASSERT_TRUE(tracker.Update({}, 4.2));
  ASSERT_TRUE(tracker.Update({}, 4.3));
  EXPECT_EQ(tracker.Tracks(4.3)[0].status,
            uw::domain::TARGET_TRACK_STATUS_CONFIRMED);
  ASSERT_TRUE(tracker.Update({}, 4.4));
  EXPECT_EQ(tracker.Tracks(4.4)[0].status,
            uw::domain::TARGET_TRACK_STATUS_DEGRADED);
  EXPECT_EQ(tracker.Tracks(4.601)[0].status,
            uw::domain::TARGET_TRACK_STATUS_STALE);
}

// docs/rov-realtime-closed-loop-code-review-2026-08-27.md finding C2:
// TargetTracker never evicted stale tracks -- a track that goes STALE
// (marked via Tracks()) stayed in tracks_ forever, growing memory and
// per-publish Predict() cost without bound over a long run. retention_
// after_s (default 5.0s, must exceed stale_after_s) now actually erases it.
TEST(TargetTracker, TrackIsEvictedAfterRetentionWindowElapsesWithNoHit) {
  auto params = Params();
  params.stale_after_s = 0.5;
  params.retention_after_s = 2.0;
  uw::frontends::TargetTracker tracker(params);

  ASSERT_TRUE(tracker.Update({Measurement("a", 0.0, 0.0, 4.0)}, 0.0));
  ASSERT_TRUE(tracker.Update({Measurement("b", 0.1, 0.0, 4.0)}, 0.1));
  ASSERT_EQ(tracker.Tracks(0.1).size(), 1u);

  // Well past stale_after_s (0.5s) but not yet past retention_after_s
  // (2.0s) -- STALE, but still present.
  ASSERT_TRUE(tracker.Update({}, 1.5));
  const auto still_stale = tracker.Tracks(1.5);
  ASSERT_EQ(still_stale.size(), 1u);
  EXPECT_EQ(still_stale[0].status, uw::domain::TARGET_TRACK_STATUS_STALE);

  // Past retention_after_s (2.0s since the last hit at 0.1s) -- gone.
  ASSERT_TRUE(tracker.Update({}, 2.6));
  EXPECT_TRUE(tracker.Tracks(2.6).empty());
}

TEST(TargetTracker, EvictionDoesNotAffectTracksStillWithinRetentionWindow) {
  auto params = Params();
  params.stale_after_s = 0.5;
  params.retention_after_s = 2.0;
  uw::frontends::TargetTracker tracker(params);

  ASSERT_TRUE(tracker.Update({Measurement("a", 0.0, 0.0, 4.0)}, 0.0));
  ASSERT_TRUE(tracker.Update({Measurement("b", 0.1, 0.0, 4.0)}, 0.1));
  ASSERT_TRUE(tracker.Update({}, 1.9));  // 1.8s since last hit -- inside the 2.0s window
  EXPECT_EQ(tracker.Tracks(1.9).size(), 1u);
}

TEST(TargetTracker, EvictionAlsoAppliesOnBatchesWithFreshDetections) {
  // Eviction is exercised via the non-empty-detections branch too, not just
  // the empty-batch/miss path -- a second, unrelated track's own detections
  // must not keep an unrelated, long-idle track alive.
  auto params = Params();
  params.stale_after_s = 0.5;
  params.retention_after_s = 2.0;
  uw::frontends::TargetTracker tracker(params);

  ASSERT_TRUE(tracker.Update({Measurement("a", 0.0, 0.0, 4.0)}, 0.0));
  ASSERT_TRUE(tracker.Update({Measurement("b", 0.1, 0.0, 4.0)}, 0.1));
  ASSERT_EQ(tracker.Tracks(0.1).size(), 1u);

  // A fresh detection far from the first track's bearing births a second,
  // unrelated track at t=3.0 -- well past the first track's retention
  // window (0.1 + 2.0 = 2.1).
  ASSERT_TRUE(tracker.Update({Measurement("c", 3.0, 2.5, 8.0)}, 3.0));
  const auto tracks = tracker.Tracks(3.0);
  ASSERT_EQ(tracks.size(), 1u);
  EXPECT_EQ(tracks[0].track_id, "track_2");
}

TEST(TargetTracker, RejectsRetentionAfterSNotExceedingStaleAfterS) {
  auto params = Params();
  params.stale_after_s = 1.0;
  params.retention_after_s = 1.0;  // must be strictly greater, not equal
  EXPECT_THROW(uw::frontends::TargetTracker tracker(params), std::invalid_argument);

  params.retention_after_s = 0.5;  // less than stale_after_s
  EXPECT_THROW(uw::frontends::TargetTracker tracker2(params), std::invalid_argument);
}

TEST(TargetTracker, UnmatchedDetectionsBirthMonotonicIdsAndCrossingTracksStayDistinct) {
  uw::frontends::TargetTracker tracker(Params());
  ASSERT_TRUE(tracker.Update(
      {Measurement("l1", 5.0, -0.2, 4.0), Measurement("r1", 5.0, 0.2, 6.0)}, 5.0));
  ASSERT_TRUE(tracker.Update(
      {Measurement("r2", 5.1, 0.05, 6.0), Measurement("l2", 5.1, -0.05, 4.0)}, 5.1));
  const auto tracks = tracker.Tracks(5.1);
  ASSERT_EQ(tracks.size(), 2u);
  EXPECT_EQ(tracks[0].track_id, "track_1");
  EXPECT_EQ(tracks[1].track_id, "track_2");
  EXPECT_NEAR(tracks[0].state[1], 4.0, 0.2);
  EXPECT_NEAR(tracks[1].state[1], 6.0, 0.2);
}

TEST(TargetTracker, SplitCreatesNewTentativeIdAndMergeKeepsOlderId) {
  uw::frontends::TargetTracker tracker(Params());
  ASSERT_TRUE(tracker.Update({Measurement("seed", 6.0, 0.0, 5.0)}, 6.0));
  ASSERT_TRUE(tracker.Update(
      {Measurement("continuation", 6.1, 0.0, 5.0),
       Measurement("split", 6.1, 0.01, 5.05)}, 6.1));
  auto tracks = tracker.Tracks(6.1);
  ASSERT_EQ(tracks.size(), 2u);
  EXPECT_EQ(tracks[0].track_id, "track_1");
  EXPECT_EQ(tracks[0].status, uw::domain::TARGET_TRACK_STATUS_CONFIRMED);
  EXPECT_EQ(tracks[1].track_id, "track_2");
  EXPECT_EQ(tracks[1].status, uw::domain::TARGET_TRACK_STATUS_TENTATIVE);

  ASSERT_TRUE(tracker.Update({Measurement("merged", 6.2, 0.005, 5.02)}, 6.2));
  tracks = tracker.Tracks(6.2);
  ASSERT_EQ(tracks.size(), 1u);
  EXPECT_EQ(tracks[0].track_id, "track_1");
  EXPECT_NE(std::find_if(tracks[0].observation_ids.begin(), tracks[0].observation_ids.end(),
                         [](const auto& id) { return id.value() == "split"; }),
            tracks[0].observation_ids.end());
}

TEST(TargetTracker, MixedObservabilityMergePromotesOlderVisualTrackRangeSafely) {
  uw::frontends::TargetTracker tracker(Params());
  ASSERT_TRUE(tracker.Update(
      {Measurement("visual-old", 7.0, -0.01, std::nullopt,
                   uw::domain::ASSIST_SOURCE_VISUAL),
       Measurement("sonar-new", 7.0, 0.01, 5.0,
                   uw::domain::ASSIST_SOURCE_SONAR)},
      7.0));
  auto tracks = tracker.Tracks(7.0);
  ASSERT_EQ(tracks.size(), 2u);
  EXPECT_EQ(tracks[0].track_id, "track_1");
  EXPECT_FALSE(tracks[0].range_observable);
  EXPECT_EQ(tracks[1].track_id, "track_2");
  EXPECT_TRUE(tracks[1].range_observable);

  ASSERT_TRUE(tracker.Update(
      {Measurement("bearing-merge", 7.1, 0.0, std::nullopt,
                   uw::domain::ASSIST_SOURCE_VISUAL)},
      7.1));
  tracks = tracker.Tracks(7.1);
  ASSERT_EQ(tracks.size(), 1u);
  const auto& merged = tracks[0];
  EXPECT_EQ(merged.track_id, "track_1");
  EXPECT_TRUE(merged.range_observable);
  EXPECT_NEAR(merged.state[1], 5.0, 0.2);
  EXPECT_TRUE(merged.state.allFinite());
  EXPECT_TRUE(merged.covariance.allFinite());
  EXPECT_TRUE(merged.covariance.isApprox(
      merged.covariance.transpose(), 1e-12));
  EXPECT_GE(Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d>(merged.covariance)
                .eigenvalues().minCoeff(),
            -1e-10);
  EXPECT_EQ(merged.sources,
            (std::vector<uw::domain::AssistSource>{
                uw::domain::ASSIST_SOURCE_VISUAL,
                uw::domain::ASSIST_SOURCE_SONAR}));
  EXPECT_TRUE(merged.ToProto(7.1).has_value());
  const auto set = tracker.ToProtoSet(7.1);
  ASSERT_TRUE(set.has_value());
  ASSERT_EQ(set->tracks_size(), 1);
  EXPECT_TRUE(set->tracks(0).has_range_m());
}

TEST(TargetTracker, LongPredictionUsesWholeElapsedTimeIndependentOfEmptyFrequency) {
  auto params = Params();
  params.max_prediction_dt_s = 0.5;
  params.degraded_misses = 100;
  params.stale_after_s = 10.0;
  params.retention_after_s = 20.0;  // must exceed stale_after_s; this test predicts out to t=5.1s
  uw::frontends::TargetTracker direct(params);
  uw::frontends::TargetTracker stepped(params);
  for (auto* tracker : {&direct, &stepped}) {
    ASSERT_TRUE(tracker->Update(
        {Measurement("motion-1", 0.0, 0.0, 5.0)}, 0.0));
    ASSERT_TRUE(tracker->Update(
        {Measurement("motion-2", 0.1, 0.02, 5.05)}, 0.1));
  }
  const auto start = direct.Tracks(0.1)[0];

  ASSERT_TRUE(direct.Update({}, 5.1));
  for (int step = 1; step <= 10; ++step) {
    ASSERT_TRUE(stepped.Update({}, 0.1 + 0.5 * step));
  }
  const auto direct_track = direct.Tracks(5.1)[0];
  const auto stepped_track = stepped.Tracks(5.1)[0];
  const double pi = std::acos(-1.0);
  double expected_bearing =
      std::remainder(start.state[0] + 5.0 * start.state[2], 2.0 * pi);
  if (expected_bearing <= -pi) expected_bearing += 2.0 * pi;
  EXPECT_NEAR(direct_track.state[0], expected_bearing, 1e-10);
  EXPECT_NEAR(direct_track.state[1],
              start.state[1] + 5.0 * start.state[3], 1e-10);
  EXPECT_TRUE(direct_track.state.isApprox(stepped_track.state, 1e-10));
  EXPECT_TRUE(direct_track.covariance.isApprox(
      stepped_track.covariance, 1e-10));

  EXPECT_FALSE(direct.Update(
      {Measurement("behind-horizon", 2.5, direct_track.state[0],
                   direct_track.state[1])},
      5.2));
  EXPECT_TRUE(direct.Update(
      {Measurement("at-horizon", 5.1, direct_track.state[0],
                   direct_track.state[1])},
      5.1));
}

TEST(TargetTracker, TwoCloseTracksWithSeparateDetectionsDoNotFalseMerge) {
  uw::frontends::TargetTracker tracker(Params());
  ASSERT_TRUE(tracker.Update(
      {Measurement("left-1", 7.0, 0.0, 5.0),
       Measurement("right-1", 7.0, 0.02, 5.1)}, 7.0));
  ASSERT_TRUE(tracker.Update(
      {Measurement("left-2", 7.1, 0.0, 5.0),
       Measurement("right-2", 7.1, 0.02, 5.1)}, 7.1));

  const auto tracks = tracker.Tracks(7.1);
  ASSERT_EQ(tracks.size(), 2u);
  EXPECT_EQ(tracks[0].track_id, "track_1");
  EXPECT_EQ(tracks[1].track_id, "track_2");
}

TEST(TargetTracker, MergedTrackCannotBeReusedByLaterDetectionInSameBatch) {
  uw::frontends::TargetTracker tracker(Params());
  ASSERT_TRUE(tracker.Update(
      {Measurement("t1", 7.0, 0.00, 5.00),
       Measurement("t2", 7.0, 0.01, 5.05),
       Measurement("t3", 7.0, 0.02, 5.10)}, 7.0));
  ASSERT_TRUE(tracker.Update(
      {Measurement("merge-12", 7.1, 0.009, 5.04),
       Measurement("continue-3", 7.1, 0.020, 5.10)}, 7.1));

  const auto tracks = tracker.Tracks(7.1);
  ASSERT_EQ(tracks.size(), 2u);
  EXPECT_EQ(tracks[0].track_id, "track_1");
  EXPECT_EQ(tracks[1].track_id, "track_3");
}

TEST(TargetTracker, RejectsBatchAndHistoricalObservationReplayAtomically) {
  uw::frontends::TargetTracker batch_tracker(Params());
  EXPECT_FALSE(batch_tracker.Update(
      {Measurement("duplicate", 7.5, -0.1, 4.0),
       Measurement("duplicate", 7.5, 0.1, 6.0)}, 7.5));
  EXPECT_TRUE(batch_tracker.Tracks(7.5).empty());

  uw::frontends::TargetTracker replay_tracker(Params());
  ASSERT_TRUE(replay_tracker.Update(
      {Measurement("once", 7.5, 0.0, 5.0)}, 7.5));
  EXPECT_FALSE(replay_tracker.Update(
      {Measurement("once", 7.6, 0.0, 5.0)}, 7.6));
  const auto tracks = replay_tracker.Tracks(7.6);
  ASSERT_EQ(tracks.size(), 1u);
  EXPECT_EQ(tracks[0].status, uw::domain::TARGET_TRACK_STATUS_TENTATIVE);
  ASSERT_EQ(tracks[0].observation_ids.size(), 1u);
}

TEST(TargetTracker, AssociationUsesFullCorrelatedCovarianceMahalanobis) {
  auto params = Params();
  params.association_mahalanobis_sq = 10.0;
  uw::frontends::TargetTracker tracker(params);
  auto first = Measurement("correlated-1", 7.7, 0.0, 5.0);
  first.covariance << 1.0, 0.99, 0.99, 1.0;
  auto second = Measurement("correlated-2", 7.8, 0.5, 4.5);
  second.covariance << 1.0, 0.99, 0.99, 1.0;
  ASSERT_TRUE(tracker.Update({first}, 7.7));
  ASSERT_TRUE(tracker.Update({second}, 7.8));
  EXPECT_EQ(tracker.Tracks(7.8).size(), 2u);
}

TEST(TargetTracker, RejectsInvalidOrOutOfOrderBatchWithoutPollutingState) {
  uw::frontends::TargetTracker tracker(Params());
  ASSERT_TRUE(tracker.Update({Measurement("valid", 8.0, 0.0, 3.0)}, 8.0));
  const auto before = tracker.Tracks(8.0);
  auto invalid = Measurement("nan", 8.1, 0.0, 3.0);
  invalid.covariance(0, 0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(tracker.Update({invalid}, 8.1));
  invalid = Measurement("huge-cov", 8.1, 0.0, 3.0);
  invalid.covariance(1, 1) = 1.0e20;
  EXPECT_FALSE(tracker.Update({invalid}, 8.1));
  EXPECT_FALSE(tracker.Update({Measurement("old", 7.9, 0.0, 3.0)}, 7.9));
  const auto after = tracker.Tracks(8.0);
  ASSERT_EQ(after.size(), 1u);
  ASSERT_EQ(after[0].observation_ids.size(), before[0].observation_ids.size());
  EXPECT_EQ(after[0].observation_ids[0].value(),
            before[0].observation_ids[0].value());
  EXPECT_TRUE(after[0].state.isApprox(before[0].state));
}

TEST(TargetTracker, WrapsBearingInnovationsAcrossPi) {
  uw::frontends::TargetTracker tracker(Params());
  ASSERT_TRUE(tracker.Update({Measurement("a", 9.0, 3.13, 5.0)}, 9.0));
  ASSERT_TRUE(tracker.Update({Measurement("b", 9.1, -3.13, 5.0)}, 9.1));
  const auto bearing = tracker.Tracks(9.1)[0].state[0];
  EXPECT_GT(std::abs(bearing), 3.0);
  EXPECT_LE(bearing, 3.14159265358979323846);
  EXPECT_GT(bearing, -3.14159265358979323846);
}
