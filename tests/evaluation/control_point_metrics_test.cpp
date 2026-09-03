#include "evaluation/control_point_metrics.hpp"

#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

using uw::evaluation::ComputeControlPointError;
using uw::evaluation::ControlPoint;
using uw::evaluation::ControlPointObservation;
using uw::evaluation::TrajectoryPose;
using uw::sensor_models::Pose3;

namespace {

TrajectoryPose MakePose(double t, const Eigen::Vector3d& translation,
                        const Eigen::Quaterniond& rotation = Eigen::Quaterniond::Identity()) {
  TrajectoryPose pose;
  pose.timestamp_s = t;
  pose.pose_WB.translation = translation;
  pose.pose_WB.rotation = rotation.normalized();
  return pose;
}

std::vector<ControlPoint> PoolMarkers() {
  // Three 30 cm plates on the floor of a 3 m pool, Z-up like the rest of
  // the repo (PressureDepthMeasurement's positive-down depth becomes a
  // negative z here).
  return {
      ControlPoint{"cp_a", Eigen::Vector3d(1.0, 1.0, -3.0), 0.3},
      ControlPoint{"cp_b", Eigen::Vector3d(4.0, 1.0, -3.0), 0.3},
      ControlPoint{"cp_c", Eigen::Vector3d(4.0, 4.0, -3.0), 0.3},
  };
}

}  // namespace

TEST(ControlPointMetrics, PerfectEstimateScoresZero) {
  const auto markers = PoolMarkers();
  const std::vector<TrajectoryPose> estimated{MakePose(0.0, Eigen::Vector3d(1.0, 1.0, -1.0))};
  // Observed from directly 2 m above cp_a, identity attitude.
  const std::vector<ControlPointObservation> observations{
      {"cp_a", 0.0, Eigen::Vector3d(0.0, 0.0, -2.0)}};

  const auto result = ComputeControlPointError(estimated, markers, observations);
  EXPECT_EQ(result.num_scored_observations, 1u);
  EXPECT_EQ(result.num_covered_control_points, 1u);
  EXPECT_NEAR(result.rmse_m, 0.0, 1e-12);
  EXPECT_NEAR(result.p95_m, 0.0, 1e-12);
  ASSERT_EQ(result.per_observation.size(), 1u);
  EXPECT_EQ(result.per_observation[0].tag, "cp_a");
  EXPECT_NEAR(result.per_observation[0].matched_pose_time_s, 0.0, 1e-12);
}

// The whole point of the metric: an estimator that thinks it is somewhere
// it is not mispredicts the marker by exactly its own position error.
TEST(ControlPointMetrics, ErrorEqualsThePoseErrorForATranslationOnlyMistake) {
  const auto markers = PoolMarkers();
  const Eigen::Vector3d drift(0.12, -0.05, 0.03);
  const std::vector<TrajectoryPose> estimated{
      MakePose(0.0, Eigen::Vector3d(1.0, 1.0, -1.0) + drift)};
  const std::vector<ControlPointObservation> observations{
      {"cp_a", 0.0, Eigen::Vector3d(0.0, 0.0, -2.0)}};

  const auto result = ComputeControlPointError(estimated, markers, observations);
  EXPECT_NEAR(result.rmse_m, drift.norm(), 1e-12);
  EXPECT_NEAR(result.max_m, drift.norm(), 1e-12);
}

TEST(ControlPointMetrics, AttitudeErrorShowsUpScaledByRange) {
  const auto markers = PoolMarkers();
  // 10 m away, 1 degree of yaw error: the marker is mispredicted by
  // roughly 10 m * 1 deg in radians.
  const double yaw_error_rad = 1.0 * M_PI / 180.0;
  const std::vector<TrajectoryPose> estimated{
      MakePose(0.0, Eigen::Vector3d(1.0, -9.0, -3.0),
               Eigen::Quaterniond(Eigen::AngleAxisd(yaw_error_rad, Eigen::Vector3d::UnitZ())))};
  const std::vector<ControlPointObservation> observations{
      {"cp_a", 0.0, Eigen::Vector3d(0.0, 10.0, 0.0)}};

  const auto result = ComputeControlPointError(estimated, markers, observations);
  EXPECT_NEAR(result.rmse_m, 2.0 * 10.0 * std::sin(yaw_error_rad / 2.0), 1e-9);
}

// A marker seen from several poses contributes several errors — an
// estimator that drifts is penalised once per drifted sighting.
TEST(ControlPointMetrics, RepeatedSightingsOfOneMarkerEachScore) {
  const auto markers = PoolMarkers();
  const std::vector<TrajectoryPose> estimated{
      MakePose(0.0, Eigen::Vector3d(1.0, 1.0, -1.0)),
      MakePose(1.0, Eigen::Vector3d(1.0 + 0.2, 1.0, -1.0)),
  };
  const std::vector<ControlPointObservation> observations{
      {"cp_a", 0.0, Eigen::Vector3d(0.0, 0.0, -2.0)},
      {"cp_a", 1.0, Eigen::Vector3d(0.0, 0.0, -2.0)},
  };
  const auto result = ComputeControlPointError(estimated, markers, observations);
  EXPECT_EQ(result.num_scored_observations, 2u);
  EXPECT_EQ(result.num_covered_control_points, 1u);
  EXPECT_NEAR(result.mean_m, 0.1, 1e-12);
  EXPECT_NEAR(result.rmse_m, std::sqrt((0.0 + 0.04) / 2.0), 1e-12);
  EXPECT_NEAR(result.max_m, 0.2, 1e-12);
}

TEST(ControlPointMetrics, ObservationWithNoNearbyPoseIsReportedUnmatchedNotScored) {
  const auto markers = PoolMarkers();
  const std::vector<TrajectoryPose> estimated{MakePose(0.0, Eigen::Vector3d(1.0, 1.0, -1.0))};
  const std::vector<ControlPointObservation> observations{
      {"cp_a", 0.0, Eigen::Vector3d(0.0, 0.0, -2.0)},
      {"cp_a", 10.0, Eigen::Vector3d(0.0, 0.0, -2.0)},  // no pose within 50 ms
  };
  const auto result = ComputeControlPointError(estimated, markers, observations);
  EXPECT_EQ(result.num_scored_observations, 1u);
  EXPECT_EQ(result.num_unmatched_observations, 1u);
  EXPECT_EQ(result.num_unknown_tags, 0u);
}

TEST(ControlPointMetrics, UnknownTagIsCountedSeparatelyInsteadOfSilentlyDropped) {
  const auto markers = PoolMarkers();
  const std::vector<TrajectoryPose> estimated{MakePose(0.0, Eigen::Vector3d(1.0, 1.0, -1.0))};
  const std::vector<ControlPointObservation> observations{
      {"cp_a", 0.0, Eigen::Vector3d(0.0, 0.0, -2.0)},
      {"cp_typo", 0.0, Eigen::Vector3d(0.0, 0.0, -2.0)},
  };
  const auto result = ComputeControlPointError(estimated, markers, observations);
  EXPECT_EQ(result.num_scored_observations, 1u);
  EXPECT_EQ(result.num_unknown_tags, 1u);
  EXPECT_EQ(result.num_unmatched_observations, 1u);
}

TEST(ControlPointMetrics, P95IsNearestRankOverTheObservations) {
  const auto markers = PoolMarkers();
  std::vector<TrajectoryPose> estimated;
  std::vector<ControlPointObservation> observations;
  // 20 sightings with errors 0.00, 0.01, ..., 0.19 m along +x.
  for (int i = 0; i < 20; ++i) {
    const double t = 0.1 * i;
    estimated.push_back(MakePose(t, Eigen::Vector3d(1.0 + 0.01 * i, 1.0, -1.0)));
    observations.push_back({"cp_a", t, Eigen::Vector3d(0.0, 0.0, -2.0)});
  }
  const auto result = ComputeControlPointError(estimated, markers, observations);
  EXPECT_EQ(result.num_scored_observations, 20u);
  // ceil(0.95 * 20) = 19 -> the 19th smallest, i.e. 0.18 m.
  EXPECT_NEAR(result.p95_m, 0.18, 1e-12);
  EXPECT_NEAR(result.max_m, 0.19, 1e-12);
}

// Unaligned by default: a whole-frame offset must show up as error, not be
// fitted away, because control points are surveyed in the scenario's own
// world frame.
TEST(ControlPointMetrics, WholeFrameOffsetIsNotFittedAwayByDefault) {
  const auto markers = PoolMarkers();
  const Eigen::Vector3d offset(0.5, 0.0, 0.0);
  std::vector<TrajectoryPose> estimated;
  std::vector<ControlPointObservation> observations;
  for (std::size_t i = 0; i < markers.size(); ++i) {
    const double t = static_cast<double>(i);
    estimated.push_back(MakePose(t, markers[i].position_W + Eigen::Vector3d(0, 0, 2.0) + offset));
    observations.push_back({markers[i].tag, t, Eigen::Vector3d(0.0, 0.0, -2.0)});
  }

  const auto unaligned = ComputeControlPointError(estimated, markers, observations);
  EXPECT_NEAR(unaligned.rmse_m, offset.norm(), 1e-12);

  const auto aligned = ComputeControlPointError(estimated, markers, observations,
                                                 /*max_time_diff_s=*/0.05,
                                                 /*align_before_scoring=*/true);
  EXPECT_NEAR(aligned.rmse_m, 0.0, 1e-9);
}

// Three sightings of ONE marker pin no rigid transform, so alignment must
// decline rather than fit a meaningless one.
TEST(ControlPointMetrics, AlignmentNeedsThreeDistinctMarkersNotThreeSightings) {
  const auto markers = PoolMarkers();
  const Eigen::Vector3d offset(0.5, 0.0, 0.0);
  std::vector<TrajectoryPose> estimated;
  std::vector<ControlPointObservation> observations;
  for (int i = 0; i < 3; ++i) {
    const double t = static_cast<double>(i);
    estimated.push_back(MakePose(t, markers[0].position_W + Eigen::Vector3d(0, 0, 2.0) + offset));
    observations.push_back({"cp_a", t, Eigen::Vector3d(0.0, 0.0, -2.0)});
  }
  const auto result = ComputeControlPointError(estimated, markers, observations, 0.05, true);
  EXPECT_NEAR(result.rmse_m, offset.norm(), 1e-12);
}

TEST(ControlPointMetrics, EmptyInputsScoreNothingRatherThanCrash) {
  const auto markers = PoolMarkers();
  EXPECT_EQ(ComputeControlPointError({}, markers, {{"cp_a", 0.0, {}}}).num_scored_observations, 0u);
  EXPECT_EQ(ComputeControlPointError({MakePose(0.0, {})}, {}, {{"cp_a", 0.0, {}}}).num_scored_observations, 0u);
  EXPECT_EQ(ComputeControlPointError({MakePose(0.0, {})}, markers, {}).num_scored_observations, 0u);
}
