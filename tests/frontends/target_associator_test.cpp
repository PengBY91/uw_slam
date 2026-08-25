#include "frontends/target_associator.hpp"
#include "frontends/target_tracker.hpp"

#include <cmath>
#include <algorithm>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <Eigen/Eigenvalues>

#include "sensor_models/geometry.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

uw::domain::Stamp Stamp(double seconds) {
  uw::domain::Stamp stamp;
  const auto whole = static_cast<int64_t>(std::floor(seconds));
  stamp.set_seconds(whole);
  stamp.set_nanos(static_cast<int32_t>(std::llround((seconds - whole) * 1.0e9)));
  return stamp;
}

void AddEdge(uw::domain::RigCalibrationSnapshot* rig, const std::string& parent,
             const std::string& child,
             const uw::sensor_models::Pose3& parent_from_child) {
  auto* edge = rig->add_frame_tree();
  edge->mutable_parent_frame()->set_value(parent);
  edge->mutable_child_frame()->set_value(child);
  *edge->mutable_transform() = parent_from_child.ToProto();
}

uw::domain::RigCalibrationSnapshot Rig(double camera_offset_s = 0.0,
                                       double sonar_offset_s = 0.0,
                                       double sonar_y_m = 0.0) {
  uw::domain::RigCalibrationSnapshot rig;
  rig.mutable_calibration_version()->set_value("rig-v7");
  AddEdge(&rig, "base_link", "payload_mount", uw::sensor_models::Pose3::Identity());
  AddEdge(&rig, "payload_mount", "camera_left_link",
          uw::sensor_models::Pose3::Identity());
  auto sonar_pose = uw::sensor_models::Pose3::Identity();
  sonar_pose.translation.y() = sonar_y_m;
  AddEdge(&rig, "payload_mount", "sonar_link", sonar_pose);
  auto* camera = rig.add_cameras();
  camera->mutable_sensor_id()->set_value("camera_left");
  camera->set_width(640);
  camera->set_height(480);
  for (double value : {420.0, 0.0, 320.0, 0.0, 420.0, 240.0, 0.0, 0.0, 1.0}) {
    camera->add_k_matrix_row_major(value);
  }
  auto* sonar = rig.add_sonar_beam_models();
  sonar->mutable_sensor_id()->set_value("sonar0");
  sonar->set_sonar_enabled(true);
  (*rig.mutable_time_offset_seconds())["camera_left"] = camera_offset_s;
  (*rig.mutable_time_offset_seconds())["sonar0"] = sonar_offset_s;
  (*rig.mutable_time_offset_provenance())["camera_left"] = "measured:pps";
  (*rig.mutable_time_offset_provenance())["sonar0"] = "measured:pps";
  return rig;
}

uw::frontends::SensorTargetDetection Detection(
    std::string id, uw::domain::AssistSource source, double capture_time_s,
    double bearing_rad, std::optional<double> range_m, std::string class_label = "target") {
  uw::frontends::SensorTargetDetection input;
  input.sensor_id = source == uw::domain::ASSIST_SOURCE_VISUAL ? "camera_left" : "sonar0";
  input.sensor_frame = source == uw::domain::ASSIST_SOURCE_VISUAL ? "camera_left_link" : "sonar_link";
  input.calibration_version = "rig-v7";
  input.detection.mutable_source_observation()->set_value(std::move(id));
  *input.detection.mutable_capture_time() = Stamp(capture_time_s);
  input.detection.set_class_label(std::move(class_label));
  input.detection.set_confidence(0.9);
  input.detection.set_bearing_rad(bearing_rad);
  input.detection.set_has_range(range_m.has_value());
  input.detection.set_range_m(range_m.value_or(0.0));
  input.detection.add_covariance_2x2_row_major(0.01 * 0.01);
  input.detection.add_covariance_2x2_row_major(0.0);
  input.detection.add_covariance_2x2_row_major(0.0);
  input.detection.add_covariance_2x2_row_major(range_m ? 0.04 : 1000.0);
  input.detection.set_source(source);
  return input;
}

uw::frontends::TargetAssociatorParams Params() {
  uw::frontends::TargetAssociatorParams params;
  params.max_corrected_time_delta_s = 0.06;
  params.max_bearing_mahalanobis_sq = 16.0;
  params.max_range_mahalanobis_sq = 16.0;
  params.max_motion_bearing_delta_rad = 0.30;
  params.max_motion_rate_rad_s = 2.0;
  params.max_bearing_variance_rad2 = 0.20;
  params.max_range_variance_m2 = 4.0;
  return params;
}

}  // namespace

TEST(TargetAssociator, ProjectsVersionedRigAndPreservesDeterministicProvenance) {
  const double base_bearing = std::atan2(1.0, 4.0);
  const auto visual = Detection("visual-7", uw::domain::ASSIST_SOURCE_VISUAL, 10.0,
                                -base_bearing, std::nullopt, "crate");
  const auto sonar = Detection("sonar-9", uw::domain::ASSIST_SOURCE_SONAR, 10.0, 0.0, 4.0,
                               "sonar_target");

  const auto result = uw::frontends::TargetAssociator(Params()).Associate({visual}, {sonar}, Rig(0, 0, 1));

  ASSERT_EQ(result.measurements.size(), 1u);
  const auto& fused = result.measurements[0];
  EXPECT_NEAR(fused.bearing_rad, base_bearing, 2e-3);
  ASSERT_TRUE(fused.range_m.has_value());
  EXPECT_NEAR(*fused.range_m, std::sqrt(17.0), 1e-6);
  EXPECT_EQ(fused.class_label, "crate");
  EXPECT_EQ(fused.sources, (std::vector<uw::domain::AssistSource>{
                               uw::domain::ASSIST_SOURCE_VISUAL,
                               uw::domain::ASSIST_SOURCE_SONAR}));
  ASSERT_EQ(fused.observation_ids.size(), 2u);
  EXPECT_EQ(fused.observation_ids[0].value(), "sonar-9");
  EXPECT_EQ(fused.observation_ids[1].value(), "visual-7");
  ASSERT_EQ(result.diagnostics.size(), 1u);
  EXPECT_TRUE(result.diagnostics[0].accepted);
  EXPECT_EQ(result.diagnostics[0].reason, uw::frontends::AssociationReason::kAccepted);
  EXPECT_EQ(result.diagnostics[0].metric,
            uw::frontends::AssociationMetric::kPairCost);
  EXPECT_DOUBLE_EQ(result.diagnostics[0].threshold,
                   Params().max_bearing_mahalanobis_sq + 2.0);
}

TEST(TargetAssociator, GreedyMinimumCostKeepsIdentityWhenTwoBearingsCross) {
  const auto result = uw::frontends::TargetAssociator(Params()).Associate(
      {Detection("v-left", uw::domain::ASSIST_SOURCE_VISUAL, 3.0, 0.20, std::nullopt),
       Detection("v-right", uw::domain::ASSIST_SOURCE_VISUAL, 3.0, -0.20, std::nullopt)},
      {Detection("s-left", uw::domain::ASSIST_SOURCE_SONAR, 3.0, -0.19, 5.0),
       Detection("s-right", uw::domain::ASSIST_SOURCE_SONAR, 3.0, 0.19, 5.0)},
      Rig());

  ASSERT_EQ(result.measurements.size(), 2u);
  std::vector<std::vector<std::string>> accepted_pairs;
  for (const auto& diagnostic : result.diagnostics) {
    if (diagnostic.accepted) {
      accepted_pairs.push_back({diagnostic.visual_observation_id,
                                diagnostic.sonar_observation_id});
    }
  }
  EXPECT_EQ(accepted_pairs, (std::vector<std::vector<std::string>>{
                                {"v-left", "s-left"}, {"v-right", "s-right"}}));
}

TEST(TargetAssociator, RejectsIncompatibleClassesWithExactGateThreshold) {
  auto params = Params();
  const auto result = uw::frontends::TargetAssociator(params).Associate(
      {Detection("v", uw::domain::ASSIST_SOURCE_VISUAL, 1.0, 0.0, std::nullopt, "pipe")},
      {Detection("s", uw::domain::ASSIST_SOURCE_SONAR, 1.0, 0.0, 4.0, "fish")}, Rig());

  ASSERT_EQ(result.measurements.size(), 2u);
  const auto rejected = std::find_if(
      result.diagnostics.begin(), result.diagnostics.end(),
      [](const auto& diagnostic) {
        return !diagnostic.visual_observation_id.empty() &&
               !diagnostic.sonar_observation_id.empty();
      });
  ASSERT_NE(rejected, result.diagnostics.end());
  EXPECT_FALSE(rejected->accepted);
  EXPECT_EQ(rejected->reason,
            uw::frontends::AssociationReason::kClassIncompatible);
  EXPECT_EQ(rejected->metric, uw::frontends::AssociationMetric::kCompatibility);
  EXPECT_DOUBLE_EQ(rejected->value, 0.0);
  EXPECT_DOUBLE_EQ(rejected->threshold, 1.0);
}

TEST(TargetAssociator, AppliesRigClockOffsetsAndKeepsUnmatchedSingleSensorMeasurements) {
  const auto result = uw::frontends::TargetAssociator(Params()).Associate(
      {Detection("visual", uw::domain::ASSIST_SOURCE_VISUAL, 10.0, 0.0, std::nullopt),
       Detection("visual-only", uw::domain::ASSIST_SOURCE_VISUAL, 20.0, -0.3, std::nullopt)},
      {Detection("sonar", uw::domain::ASSIST_SOURCE_SONAR, 10.1, 0.0, 6.0),
       Detection("sonar-only", uw::domain::ASSIST_SOURCE_SONAR, 30.0, 0.4, 7.0)},
      Rig(0.1, 0.0));

  ASSERT_EQ(result.measurements.size(), 3u);
  EXPECT_TRUE(result.measurements[0].range_m.has_value());
  const auto visual_only = std::find_if(result.measurements.begin(), result.measurements.end(),
                                        [](const auto& m) { return m.observation_ids[0].value() == "visual-only"; });
  ASSERT_NE(visual_only, result.measurements.end());
  EXPECT_FALSE(visual_only->range_m.has_value());
  const auto sonar_only = std::find_if(result.measurements.begin(), result.measurements.end(),
                                       [](const auto& m) { return m.observation_ids[0].value() == "sonar-only"; });
  ASSERT_NE(sonar_only, result.measurements.end());
  ASSERT_TRUE(sonar_only->range_m.has_value());
  EXPECT_DOUBLE_EQ(*sonar_only->range_m, 7.0);
  for (const auto& diagnostic : result.diagnostics) {
    EXPECT_TRUE(std::isfinite(diagnostic.value));
    EXPECT_TRUE(std::isfinite(diagnostic.threshold));
    EXPECT_NE(diagnostic.metric,
              uw::frontends::AssociationMetric::kUnspecified);
  }
}

TEST(TargetAssociator, UnmatchedVisualDepthBecomesBearingOnlyBeforeTrackerBoundary) {
  const auto result = uw::frontends::TargetAssociator(Params()).Associate(
      {Detection("visual-depth", uw::domain::ASSIST_SOURCE_VISUAL, 2.0,
                 0.1, 6.0, "crate")},
      {}, Rig());

  ASSERT_EQ(result.measurements.size(), 1u);
  const auto& measurement = result.measurements[0];
  EXPECT_FALSE(measurement.range_m.has_value());
  EXPECT_GT(measurement.covariance(0, 0), 0.0);
  EXPECT_DOUBLE_EQ(measurement.covariance(0, 1), 0.0);
  EXPECT_DOUBLE_EQ(measurement.covariance(1, 0), 0.0);
  EXPECT_DOUBLE_EQ(measurement.covariance(1, 1), 0.0);
  EXPECT_EQ(measurement.sources,
            (std::vector<uw::domain::AssistSource>{
                uw::domain::ASSIST_SOURCE_VISUAL}));
  ASSERT_EQ(measurement.observation_ids.size(), 1u);
  EXPECT_EQ(measurement.observation_ids[0].value(), "visual-depth");

  ASSERT_EQ(result.diagnostics.size(), 1u);
  EXPECT_TRUE(result.diagnostics[0].accepted);
  EXPECT_TRUE(std::isfinite(result.diagnostics[0].value));
  EXPECT_TRUE(std::isfinite(result.diagnostics[0].threshold));

  uw::frontends::TargetTracker tracker;
  ASSERT_TRUE(tracker.Update(result.measurements, 2.0));
  const auto tracks = tracker.Tracks(2.0);
  ASSERT_EQ(tracks.size(), 1u);
  EXPECT_FALSE(tracks[0].range_observable);
  EXPECT_DOUBLE_EQ(tracks[0].state[1], 0.0);
}

TEST(TargetAssociator, FailsClosedForInvalidCovarianceCalibrationAndFrame) {
  auto invalid_covariance = Detection("bad-cov", uw::domain::ASSIST_SOURCE_VISUAL, 1.0, 0.0,
                                      std::nullopt);
  invalid_covariance.detection.set_covariance_2x2_row_major(
      0, std::numeric_limits<double>::quiet_NaN());
  auto wrong_version = Detection("wrong-version", uw::domain::ASSIST_SOURCE_SONAR, 1.0, 0.0, 4.0);
  wrong_version.calibration_version = "rig-v6";
  auto bad_frame = Detection("bad-frame", uw::domain::ASSIST_SOURCE_SONAR, 1.0, 0.0, 4.0);
  bad_frame.sensor_frame = "missing_link";

  const auto result = uw::frontends::TargetAssociator(Params()).Associate(
      {invalid_covariance}, {wrong_version, bad_frame}, Rig());

  EXPECT_TRUE(result.measurements.empty());
  ASSERT_EQ(result.diagnostics.size(), 3u);
  EXPECT_EQ(result.diagnostics[0].reason, uw::frontends::AssociationReason::kInvalidInput);
  EXPECT_EQ(result.diagnostics[1].reason,
            uw::frontends::AssociationReason::kFrameUnresolved);
  EXPECT_EQ(result.diagnostics[2].reason,
            uw::frontends::AssociationReason::kCalibrationMismatch);
}

TEST(TargetAssociator, RejectsFiniteButPhysicallyUnreasonableBearingAndCovariance) {
  auto huge_bearing = Detection("huge-bearing", uw::domain::ASSIST_SOURCE_VISUAL,
                                1.0, 100.0, std::nullopt);
  auto huge_covariance = Detection("huge-cov", uw::domain::ASSIST_SOURCE_SONAR,
                                   1.0, 0.0, 4.0);
  huge_covariance.detection.set_covariance_2x2_row_major(3, 1.0e20);

  const auto result = uw::frontends::TargetAssociator(Params()).Associate(
      {huge_bearing}, {huge_covariance}, Rig());

  EXPECT_TRUE(result.measurements.empty());
  ASSERT_EQ(result.diagnostics.size(), 2u);
  EXPECT_EQ(result.diagnostics[0].reason,
            uw::frontends::AssociationReason::kInvalidInput);
  EXPECT_EQ(result.diagnostics[1].reason,
            uw::frontends::AssociationReason::kInvalidInput);
}

TEST(TargetAssociator, RejectsSensorRoleOrFrameImpersonationWithinOtherwiseValidRig) {
  auto sonar_as_visual = Detection("sonar-as-visual",
                                   uw::domain::ASSIST_SOURCE_VISUAL, 1.0, 0.0,
                                   std::nullopt);
  sonar_as_visual.sensor_id = "sonar0";
  auto camera_on_sonar_frame = Detection("camera-wrong-frame",
                                         uw::domain::ASSIST_SOURCE_VISUAL,
                                         1.0, 0.0, std::nullopt);
  camera_on_sonar_frame.sensor_frame = "sonar_link";

  const auto result = uw::frontends::TargetAssociator(Params()).Associate(
      {sonar_as_visual, camera_on_sonar_frame}, {}, Rig());

  EXPECT_TRUE(result.measurements.empty());
  ASSERT_EQ(result.diagnostics.size(), 2u);
  for (const auto& diagnostic : result.diagnostics) {
    EXPECT_EQ(diagnostic.reason, uw::frontends::AssociationReason::kInvalidInput);
  }
}

TEST(TargetAssociator, AcceptsExactNumberedCameraFrameButRejectsDerivedNameSpoof) {
  auto numbered_rig = Rig();
  numbered_rig.mutable_cameras(0)->mutable_sensor_id()->set_value("camera2");
  numbered_rig.mutable_frame_tree(1)->mutable_child_frame()->set_value("camera2_link");
  numbered_rig.mutable_time_offset_seconds()->erase("camera_left");
  numbered_rig.mutable_time_offset_provenance()->erase("camera_left");
  (*numbered_rig.mutable_time_offset_seconds())["camera2"] = 0.0;
  (*numbered_rig.mutable_time_offset_provenance())["camera2"] = "measured:pps";
  auto numbered = Detection("camera2-observation", uw::domain::ASSIST_SOURCE_VISUAL,
                            1.0, 0.0, std::nullopt);
  numbered.sensor_id = "camera2";
  numbered.sensor_frame = "camera2_link";
  const auto accepted = uw::frontends::TargetAssociator(Params()).Associate(
      {numbered}, {}, numbered_rig);
  ASSERT_EQ(accepted.measurements.size(), 1u);

  auto spoof_rig = Rig();
  AddEdge(&spoof_rig, "camera_left_link", "camera_left_link_spoof",
          uw::sensor_models::Pose3::Identity());
  auto spoof = Detection("spoof", uw::domain::ASSIST_SOURCE_VISUAL, 1.0, 0.0,
                         std::nullopt);
  spoof.sensor_frame = "camera_left_link_spoof";
  const auto rejected = uw::frontends::TargetAssociator(Params()).Associate(
      {spoof}, {}, spoof_rig);
  EXPECT_TRUE(rejected.measurements.empty());
}

TEST(TargetAssociator, RejectsAmbiguousSensorRoleAndDuplicateObservationIdsAtomically) {
  auto ambiguous_rig = Rig();
  auto* ambiguous_sonar = ambiguous_rig.add_sonar_beam_models();
  ambiguous_sonar->mutable_sensor_id()->set_value("camera_left");
  ambiguous_sonar->set_sonar_enabled(true);
  const auto ambiguous = uw::frontends::TargetAssociator(Params()).Associate(
      {Detection("ambiguous", uw::domain::ASSIST_SOURCE_VISUAL, 1.0, 0.0,
                 std::nullopt)}, {}, ambiguous_rig);
  EXPECT_TRUE(ambiguous.measurements.empty());

  const auto duplicate_within_modality =
      uw::frontends::TargetAssociator(Params()).Associate(
          {Detection("duplicate", uw::domain::ASSIST_SOURCE_VISUAL, 1.0, 0.0,
                     std::nullopt),
           Detection("duplicate", uw::domain::ASSIST_SOURCE_VISUAL, 1.0, 0.1,
                     std::nullopt)},
          {}, Rig());
  EXPECT_TRUE(duplicate_within_modality.measurements.empty());

  const auto duplicate_across_modalities =
      uw::frontends::TargetAssociator(Params()).Associate(
          {Detection("shared", uw::domain::ASSIST_SOURCE_VISUAL, 1.0, 0.0,
                     std::nullopt)},
          {Detection("shared", uw::domain::ASSIST_SOURCE_SONAR, 1.0, 0.0, 4.0)},
          Rig());
  EXPECT_TRUE(duplicate_across_modalities.measurements.empty());
}

TEST(TargetAssociator, RejectsCyclicOrMultiParentRigInsteadOfChoosingAPath) {
  auto cyclic_rig = Rig();
  AddEdge(&cyclic_rig, "camera_left_link", "payload_mount",
          uw::sensor_models::Pose3::Identity());
  const auto cyclic = uw::frontends::TargetAssociator(Params()).Associate(
      {Detection("v-cycle", uw::domain::ASSIST_SOURCE_VISUAL, 1.0, 0.0,
                 std::nullopt)}, {}, cyclic_rig);
  EXPECT_TRUE(cyclic.measurements.empty());

  auto multi_parent_rig = Rig();
  AddEdge(&multi_parent_rig, "base_link", "camera_left_link",
          uw::sensor_models::Pose3::Identity());
  const auto multi_parent = uw::frontends::TargetAssociator(Params()).Associate(
      {Detection("v-multi", uw::domain::ASSIST_SOURCE_VISUAL, 1.0, 0.0,
                 std::nullopt)}, {}, multi_parent_rig);
  EXPECT_TRUE(multi_parent.measurements.empty());
}

TEST(TargetAssociator, AcceptedAndConflictDiagnosticsDoNotInventTotalCostThreshold) {
  auto params = Params();
  params.max_bearing_mahalanobis_sq = 100.0;
  const auto result = uw::frontends::TargetAssociator(params).Associate(
      {Detection("v", uw::domain::ASSIST_SOURCE_VISUAL, 1.0, -0.02,
                 std::nullopt)},
      {Detection("s-best", uw::domain::ASSIST_SOURCE_SONAR, 1.0, 0.0, 4.0),
       Detection("s-conflict", uw::domain::ASSIST_SOURCE_SONAR, 1.0, 0.01, 4.0)}, Rig());
  ASSERT_GE(result.diagnostics.size(), 2u);
  for (const auto& diagnostic : result.diagnostics) {
    EXPECT_TRUE(std::isfinite(diagnostic.value));
    EXPECT_TRUE(std::isfinite(diagnostic.threshold));
    EXPECT_NE(diagnostic.metric,
              uw::frontends::AssociationMetric::kUnspecified);
    if (diagnostic.reason == uw::frontends::AssociationReason::kPairConflict) {
      EXPECT_EQ(diagnostic.metric,
                uw::frontends::AssociationMetric::kWinningPairCost);
      EXPECT_GE(diagnostic.value, diagnostic.threshold);
    }
  }
}

TEST(TargetAssociator, RejectsBearingWrapAwareMotionAndMahalanobisGatesDeterministically) {
  auto params = Params();
  params.max_motion_bearing_delta_rad = 0.05;
  params.max_motion_rate_rad_s = 0.0;
  params.max_bearing_mahalanobis_sq = 1.0e6;
  auto visual = Detection("v", uw::domain::ASSIST_SOURCE_VISUAL, 2.0, -(kPi - 0.01), std::nullopt);
  auto sonar = Detection("s", uw::domain::ASSIST_SOURCE_SONAR, 2.0, -kPi + 0.02, 4.0);
  const auto wrapped = uw::frontends::TargetAssociator(params).Associate({visual}, {sonar}, Rig());
  ASSERT_EQ(wrapped.measurements.size(), 1u);

  sonar.detection.set_bearing_rad(-kPi + 0.20);
  const auto rejected = uw::frontends::TargetAssociator(params).Associate({visual}, {sonar}, Rig());
  const auto motion_rejection = std::find_if(
      rejected.diagnostics.begin(), rejected.diagnostics.end(),
      [](const auto& diagnostic) {
        return diagnostic.reason ==
               uw::frontends::AssociationReason::kMotionContinuity;
      });
  ASSERT_NE(motion_rejection, rejected.diagnostics.end());
  EXPECT_EQ(motion_rejection->reason,
            uw::frontends::AssociationReason::kMotionContinuity);
  EXPECT_DOUBLE_EQ(motion_rejection->threshold, 0.05);
}

TEST(TargetAssociator, FullCorrelatedCovarianceRejectsJointlyInconsistentPair) {
  auto params = Params();
  params.max_bearing_mahalanobis_sq = 4.0;
  params.max_range_mahalanobis_sq = 4.0;
  params.max_motion_bearing_delta_rad = 1.0;
  params.max_bearing_variance_rad2 = 2.0;
  params.max_range_variance_m2 = 2.0;
  auto visual = Detection("v-correlated", uw::domain::ASSIST_SOURCE_VISUAL,
                          2.0, 0.0, 5.0);
  auto sonar = Detection("s-correlated", uw::domain::ASSIST_SOURCE_SONAR,
                         2.0, 0.5, 4.5);
  // The camera bearing Jacobian reverses sign, so this projects to the same
  // positive cross-covariance as the sonar measurement.
  visual.detection.set_covariance_2x2_row_major(0, 1.0);
  visual.detection.set_covariance_2x2_row_major(1, -0.99);
  visual.detection.set_covariance_2x2_row_major(2, -0.99);
  visual.detection.set_covariance_2x2_row_major(3, 1.0);
  sonar.detection.set_covariance_2x2_row_major(0, 1.0);
  sonar.detection.set_covariance_2x2_row_major(1, 0.99);
  sonar.detection.set_covariance_2x2_row_major(2, 0.99);
  sonar.detection.set_covariance_2x2_row_major(3, 1.0);

  const auto result = uw::frontends::TargetAssociator(params).Associate(
      {visual}, {sonar}, Rig());
  EXPECT_EQ(result.measurements.size(), 2u);
  const auto joint = std::find_if(
      result.diagnostics.begin(), result.diagnostics.end(),
      [](const auto& diagnostic) {
        return diagnostic.reason ==
               uw::frontends::AssociationReason::kJointMahalanobis;
      });
  ASSERT_NE(joint, result.diagnostics.end());
  EXPECT_EQ(joint->metric,
            uw::frontends::AssociationMetric::kJointMahalanobisSquared);
  EXPECT_GT(joint->value, joint->threshold);
  EXPECT_DOUBLE_EQ(joint->threshold, 8.0);
}

TEST(TargetAssociator, FusedMeasurementPropagatesCorrelatedCovariance) {
  auto params = Params();
  params.max_bearing_variance_rad2 = 1.0;
  auto visual = Detection("v-cross", uw::domain::ASSIST_SOURCE_VISUAL,
                          2.0, 0.0, 5.0);
  auto sonar = Detection("s-cross", uw::domain::ASSIST_SOURCE_SONAR,
                         2.0, 0.0, 5.0);
  visual.detection.set_covariance_2x2_row_major(0, 0.04);
  visual.detection.set_covariance_2x2_row_major(1, -0.01);
  visual.detection.set_covariance_2x2_row_major(2, -0.01);
  visual.detection.set_covariance_2x2_row_major(3, 0.09);
  sonar.detection.set_covariance_2x2_row_major(0, 0.04);
  sonar.detection.set_covariance_2x2_row_major(1, 0.01);
  sonar.detection.set_covariance_2x2_row_major(2, 0.01);
  sonar.detection.set_covariance_2x2_row_major(3, 0.09);

  const auto result = uw::frontends::TargetAssociator(params).Associate(
      {visual}, {sonar}, Rig());
  ASSERT_EQ(result.measurements.size(), 1u);
  const auto& covariance = result.measurements[0].covariance;
  EXPECT_GT(std::abs(covariance(0, 1)), 1e-6);
  EXPECT_TRUE(covariance.isApprox(covariance.transpose(), 1e-12));
  EXPECT_GT(Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(covariance)
                .eigenvalues().minCoeff(),
            0.0);
}

TEST(TargetAssociator, SingleSourceAcceptanceHasAuditableFiniteDecision) {
  const auto result = uw::frontends::TargetAssociator(Params()).Associate(
      {Detection("visual-only", uw::domain::ASSIST_SOURCE_VISUAL, 2.0,
                 0.0, std::nullopt)},
      {}, Rig());
  ASSERT_EQ(result.measurements.size(), 1u);
  ASSERT_EQ(result.diagnostics.size(), 1u);
  const auto& diagnostic = result.diagnostics[0];
  EXPECT_TRUE(diagnostic.accepted);
  EXPECT_EQ(diagnostic.reason,
            uw::frontends::AssociationReason::kSingleSourceAccepted);
  EXPECT_EQ(diagnostic.metric,
            uw::frontends::AssociationMetric::kInputValidity);
  EXPECT_DOUBLE_EQ(diagnostic.value, 1.0);
  EXPECT_DOUBLE_EQ(diagnostic.threshold, 1.0);
}

TEST(TargetAssociator, RejectsNegativeAndUnsafeCorrectedCaptureTime) {
  auto negative = Detection("negative", uw::domain::ASSIST_SOURCE_VISUAL,
                            0.0, 0.0, std::nullopt);
  negative.detection.mutable_capture_time()->set_seconds(-1);
  auto huge = Detection("huge", uw::domain::ASSIST_SOURCE_VISUAL,
                        0.0, 0.0, std::nullopt);
  huge.detection.mutable_capture_time()->set_seconds(
      std::numeric_limits<int64_t>::max());
  const auto invalid_stamps = uw::frontends::TargetAssociator(Params()).Associate(
      {negative, huge}, {}, Rig());
  EXPECT_TRUE(invalid_stamps.measurements.empty());
  ASSERT_EQ(invalid_stamps.diagnostics.size(), 2u);

  auto negative_offset_rig = Rig(-0.1, 0.0);
  const auto negative_corrected =
      uw::frontends::TargetAssociator(Params()).Associate(
          {Detection("offset-negative", uw::domain::ASSIST_SOURCE_VISUAL,
                     0.0, 0.0, std::nullopt)},
          {}, negative_offset_rig);
  EXPECT_TRUE(negative_corrected.measurements.empty());
  for (const auto& diagnostic : invalid_stamps.diagnostics) {
    EXPECT_TRUE(std::isfinite(diagnostic.value));
    EXPECT_TRUE(std::isfinite(diagnostic.threshold));
  }
}
