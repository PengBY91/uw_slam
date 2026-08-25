#include "frontends/target_tracker.hpp"

#include <cmath>
#include <algorithm>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <Eigen/Eigenvalues>

namespace {

uw::frontends::TargetMeasurement Measurement(
    std::string id, double time_s, double bearing_rad, std::optional<double> range_m,
    uw::domain::AssistSource source = uw::domain::ASSIST_SOURCE_VISUAL,
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
  measurement.sources = {source};
  uw::domain::ObservationId observation;
  observation.set_value(std::move(id));
  measurement.observation_ids = {observation};
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
  EXPECT_FALSE(tracks[0].ToProto(1.2).has_value());
}

TEST(TargetTracker, ProtoExportFailsClosedForUnsafeTimeAndTrackInvariants) {
  uw::frontends::TargetTracker tracker(Params());
  ASSERT_TRUE(tracker.Update({Measurement("wire", 1.0, 0.1, 4.0,
                                         uw::domain::ASSIST_SOURCE_SONAR)}, 1.0));
  const auto valid = tracker.Tracks(1.1)[0];
  EXPECT_TRUE(valid.ToProto(1.1).has_value());

  auto invalid = valid;
  invalid.first_capture_time_s = 1.0e300;
  EXPECT_FALSE(invalid.ToProto(1.1).has_value());
  invalid = valid;
  invalid.covariance(0, 1) = 1.0;
  EXPECT_FALSE(invalid.ToProto(1.1).has_value());
  invalid = valid;
  invalid.track_id.clear();
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
  ASSERT_TRUE(tracker.Update({Measurement("f", 2.1, 0.21, 7.1,
                                         uw::domain::ASSIST_SOURCE_ACOUSTIC_OPTIC)}, 2.1));
  const auto track = tracker.Tracks(2.1)[0];
  EXPECT_TRUE(track.range_observable);
  EXPECT_TRUE(track.covariance.isApprox(track.covariance.transpose(), 1e-12));
  EXPECT_GE(Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d>(track.covariance)
                .eigenvalues().minCoeff(),
            -1e-10);
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
