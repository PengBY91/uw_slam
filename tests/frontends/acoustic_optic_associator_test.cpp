#include <gtest/gtest.h>

#include "frontends/acoustic_optic_associator.hpp"

namespace {

uw::domain::RigCalibrationSnapshot MakeCoLocatedRig() {
  uw::domain::RigCalibrationSnapshot rig;
  auto* camera = rig.add_cameras();
  camera->mutable_sensor_id()->set_value("camera_left");
  camera->set_width(20);
  camera->set_height(10);
  for (double v : {100.0, 0.0, 10.0, 0.0, 100.0, 5.0, 0.0, 0.0, 1.0}) camera->add_k_matrix_row_major(v);

  auto add_identity_edge = [&](const std::string& child) {
    auto* edge = rig.add_frame_tree();
    edge->mutable_parent_frame()->set_value("base_link");
    edge->mutable_child_frame()->set_value(child);
    for (double v : {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0}) {
      edge->mutable_transform()->add_matrix_row_major(v);
    }
  };
  add_identity_edge("camera_left_link");
  add_identity_edge("sonar_link");

  auto* beam_model = rig.add_sonar_beam_models();
  beam_model->mutable_sensor_id()->set_value("sonar0");
  beam_model->set_elevation_aperture_rad(0.0);
  return rig;
}

uw::domain::HypothesisSet MakeSonarHypothesis(double range_m, double bearing_rad) {
  uw::domain::SonarRangeBearing measurement;
  measurement.set_range_m(range_m);
  measurement.set_bearing_rad(bearing_rad);
  measurement.set_range_sigma_m(0.1);
  measurement.set_bearing_sigma_rad(0.05);
  uw::domain::EvidenceId id;
  id.set_value("sonar_hyp_1");
  auto evidence = uw::domain::MakeEvidence(id, {}, measurement, 1.0, "sonar_cfar_frontend_v1");
  uw::domain::HypothesisSet hypotheses;
  *hypotheses.add_candidates() = evidence;
  hypotheses.add_calibrated_likelihoods(1.0);
  return hypotheses;
}

uw::domain::MeasurementEvidence MakeOpticalEvidence(int width, int height, int valid_index,
                                                     float depth_m,
                                                     uw::domain::OpticalDepthScaleStatus scale) {
  uw::domain::OpticalDepthPriorMeasurement prior;
  prior.set_width(width);
  prior.set_height(height);
  const int pixels = width * height;
  std::string valid_mask(pixels, '\0');
  for (int i = 0; i < pixels; ++i) {
    prior.add_depth_m(i == valid_index ? depth_m : 0.0f);
    prior.add_variance_m2(i == valid_index ? 0.01f : 0.0f);
  }
  valid_mask[valid_index] = 1;
  prior.set_valid_mask(valid_mask);
  prior.set_scale_status(scale);
  prior.set_producer_type("stereo");
  uw::domain::EvidenceId id;
  id.set_value("optical_1");
  return uw::domain::MakeEvidence(id, {}, prior, 1.0, "stereo_depth_frontend_v1");
}

// Like MakeCoLocatedRig, but with a configurable focal length (in pixels)
// and sonar beam elevation aperture, so a test can control how far apart
// (in image rows) the sonar arc's elevation samples land.
uw::domain::RigCalibrationSnapshot MakeCoLocatedRigWithApertures(double focal_px,
                                                                  double elevation_aperture_rad) {
  uw::domain::RigCalibrationSnapshot rig;
  auto* camera = rig.add_cameras();
  camera->mutable_sensor_id()->set_value("camera_left");
  camera->set_width(20);
  camera->set_height(10);
  for (double v : {focal_px, 0.0, 10.0, 0.0, focal_px, 5.0, 0.0, 0.0, 1.0}) camera->add_k_matrix_row_major(v);

  auto add_identity_edge = [&](const std::string& child) {
    auto* edge = rig.add_frame_tree();
    edge->mutable_parent_frame()->set_value("base_link");
    edge->mutable_child_frame()->set_value(child);
    for (double v : {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0}) {
      edge->mutable_transform()->add_matrix_row_major(v);
    }
  };
  add_identity_edge("camera_left_link");
  add_identity_edge("sonar_link");

  auto* beam_model = rig.add_sonar_beam_models();
  beam_model->mutable_sensor_id()->set_value("sonar0");
  beam_model->set_elevation_aperture_rad(elevation_aperture_rad);
  return rig;
}

uw::domain::MeasurementEvidence MakeMultiPixelOpticalEvidence(
    int width, int height, const std::vector<std::pair<int, float>>& index_to_depth_m) {
  uw::domain::OpticalDepthPriorMeasurement prior;
  prior.set_width(width);
  prior.set_height(height);
  const int pixels = width * height;
  std::string valid_mask(pixels, '\0');
  for (int i = 0; i < pixels; ++i) {
    prior.add_depth_m(0.0f);
    prior.add_variance_m2(0.0f);
  }
  for (const auto& [index, depth_m] : index_to_depth_m) {
    prior.set_depth_m(index, depth_m);
    prior.set_variance_m2(index, 0.01f);
    valid_mask[static_cast<std::size_t>(index)] = 1;
  }
  prior.set_valid_mask(valid_mask);
  prior.set_scale_status(uw::domain::OPTICAL_DEPTH_SCALE_STATUS_METRIC);
  prior.set_producer_type("stereo");
  uw::domain::EvidenceId id;
  id.set_value("optical_1");
  return uw::domain::MakeEvidence(id, {}, prior, 1.0, "stereo_depth_frontend_v1");
}

}  // namespace

TEST(AcousticOpticAssociator, AcceptsConsistentBoresightDetection) {
  const auto rig = MakeCoLocatedRig();
  const auto sonar_hypotheses = MakeSonarHypothesis(/*range_m=*/5.0, /*bearing_rad=*/0.0);
  // Pixel (10,5) = index 5*20+10 = 110, matching boresight math (range=5 -> pixel center of a
  // 20x10 image with cx=10, cy=5, camera co-located and co-oriented with the sonar).
  const auto optical_evidence =
      MakeOpticalEvidence(20, 10, /*valid_index=*/110, /*depth_m=*/5.0,
                          uw::domain::OPTICAL_DEPTH_SCALE_STATUS_METRIC);

  uw::frontends::AcousticOpticAssociatorParams params;
  uw::frontends::AcousticOpticAssociator associator(params);
  const auto result = associator.Associate(sonar_hypotheses, optical_evidence, rig, /*time_delta_seconds=*/0.01);

  ASSERT_EQ(result.records.size(), 1u);
  const auto& record = result.records[0];
  EXPECT_EQ(record.status(), uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_ACCEPTED);
  EXPECT_EQ(record.reason(), uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_NONE);
  EXPECT_TRUE(record.has_selected_pixel());
  EXPECT_EQ(record.selected_pixel_index(), 110u);
  EXPECT_NEAR(record.prior_depth_m(), 5.0, 1e-6);
  EXPECT_NEAR(record.best_score(), 0.0, 1e-6);
  EXPECT_NEAR(record.time_delta_seconds(), 0.01, 1e-9);
  // Association must leave posterior fields for the fusion stage.
  EXPECT_EQ(record.posterior_depth_m(), 0.0);
  EXPECT_EQ(record.posterior_variance_m2(), 0.0);
}

TEST(AcousticOpticAssociator, RejectsWhenNoOpticalPixelSurvivesTheRangeGate) {
  const auto rig = MakeCoLocatedRig();
  const auto sonar_hypotheses = MakeSonarHypothesis(5.0, 0.0);
  // Optical depth at the boresight pixel is 9.0m, far outside a tight range gate for a
  // 5.0m sonar detection.
  const auto optical_evidence =
      MakeOpticalEvidence(20, 10, 110, /*depth_m=*/9.0, uw::domain::OPTICAL_DEPTH_SCALE_STATUS_METRIC);

  uw::frontends::AcousticOpticAssociatorParams params;
  params.range_gate_m = 0.5;
  uw::frontends::AcousticOpticAssociator associator(params);
  const auto result = associator.Associate(sonar_hypotheses, optical_evidence, rig, 0.0);

  ASSERT_EQ(result.records.size(), 1u);
  EXPECT_EQ(result.records[0].status(), uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_REJECTED);
  EXPECT_EQ(result.records[0].reason(), uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_NO_CANDIDATE);
}

TEST(AcousticOpticAssociator, RejectsRelativeScaleOpticalPrior) {
  const auto rig = MakeCoLocatedRig();
  const auto sonar_hypotheses = MakeSonarHypothesis(5.0, 0.0);
  const auto optical_evidence = MakeOpticalEvidence(
      20, 10, 110, 5.0, uw::domain::OPTICAL_DEPTH_SCALE_STATUS_RELATIVE_SCALE);

  uw::frontends::AcousticOpticAssociatorParams params;
  uw::frontends::AcousticOpticAssociator associator(params);
  const auto result = associator.Associate(sonar_hypotheses, optical_evidence, rig, 0.0);

  ASSERT_EQ(result.records.size(), 1u);
  EXPECT_EQ(result.records[0].status(), uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_REJECTED);
  EXPECT_EQ(result.records[0].reason(), uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_SCALE);
}

TEST(AcousticOpticAssociator, ReturnsNoRecordsWhenHypothesisSetIsEmpty) {
  const auto rig = MakeCoLocatedRig();
  uw::domain::HypothesisSet empty_hypotheses;
  const auto optical_evidence =
      MakeOpticalEvidence(20, 10, 110, 5.0, uw::domain::OPTICAL_DEPTH_SCALE_STATUS_METRIC);

  uw::frontends::AcousticOpticAssociatorParams params;
  uw::frontends::AcousticOpticAssociator associator(params);
  const auto result = associator.Associate(empty_hypotheses, optical_evidence, rig, 0.0);
  EXPECT_TRUE(result.records.empty());
}

// Regression test for the associator scoring bug found via
// docs/uw-slam-production-readiness-and-roadmap-2026-08-21.md's P0
// investigation: near boresight, bearing is exactly independent of
// elevation and range only picks up a tiny sec(phi) correction, so two arc
// samples a few rows apart can tie almost exactly on geometric score even
// when they agree (to well within sensor noise) on the depth they'd imply
// — not a real competing hypothesis. Two valid pixels are placed at
// elevation phi=0 (row 5, depth chosen for an exact-zero-residual match)
// and phi=atan(0.1) (row 4, likewise exact-zero-residual for THAT pixel's
// own geometry) — geometric scores are both ~0 (tied), but the two
// pixels' depth_m values differ by only ~0.025m, well inside their
// combined ~0.14m (sqrt(0.01+0.01)) sigma, so they should be accepted as
// consistent rather than rejected as ambiguous.
TEST(AcousticOpticAssociator, AcceptsTiedScoreCandidatesWhoseDepthsAgree) {
  const auto rig = MakeCoLocatedRigWithApertures(/*focal_px=*/10.0, /*elevation_aperture_rad=*/0.199339);
  const auto sonar_hypotheses = MakeSonarHypothesis(/*range_m=*/5.0, /*bearing_rad=*/0.0);
  const auto optical_evidence = MakeMultiPixelOpticalEvidence(
      20, 10,
      {
          {110, 5.0f},        // row 5 (phi=0): depth = range * cos(0) = 5.0, exact-zero residual.
          {90, 4.975184f},    // row 4 (phi=atan(0.1)): depth = range * cos(atan(0.1)), also exact-zero.
      });

  uw::frontends::AcousticOpticAssociatorParams params;
  params.arc_samples = 3;
  uw::frontends::AcousticOpticAssociator associator(params);
  const auto result = associator.Associate(sonar_hypotheses, optical_evidence, rig, 0.0);

  ASSERT_EQ(result.records.size(), 1u);
  const auto& record = result.records[0];
  EXPECT_EQ(record.status(), uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_ACCEPTED);
  EXPECT_TRUE(record.has_selected_pixel());
  EXPECT_NEAR(record.best_score(), 0.0, 1e-6);
  EXPECT_NEAR(record.second_best_score(), 0.0, 1e-6);
}

// Companion to the AcceptsTiedScoreCandidatesWhoseDepthsAgree case above:
// same tied-geometric-score setup, but the two candidate pixels' depths
// (5.0m at row 5 vs 4.472136m at row 0, from phi=atan(0.5)) disagree by
// ~0.53m — far outside their combined ~0.14m sigma — so this IS a genuine
// competing-hypothesis case and must still be rejected as ambiguous.
TEST(AcousticOpticAssociator, RejectsTiedScoreCandidatesWhoseDepthsDisagree) {
  const auto rig = MakeCoLocatedRigWithApertures(/*focal_px=*/10.0, /*elevation_aperture_rad=*/0.927295);
  const auto sonar_hypotheses = MakeSonarHypothesis(/*range_m=*/5.0, /*bearing_rad=*/0.0);
  const auto optical_evidence = MakeMultiPixelOpticalEvidence(
      20, 10,
      {
          {110, 5.0f},        // row 5 (phi=0): depth = range * cos(0) = 5.0, exact-zero residual.
          {10, 4.472136f},    // row 0 (phi=atan(0.5)): depth = range * cos(atan(0.5)), also exact-zero.
      });

  uw::frontends::AcousticOpticAssociatorParams params;
  params.arc_samples = 3;
  uw::frontends::AcousticOpticAssociator associator(params);
  const auto result = associator.Associate(sonar_hypotheses, optical_evidence, rig, 0.0);

  ASSERT_EQ(result.records.size(), 1u);
  const auto& record = result.records[0];
  EXPECT_EQ(record.status(), uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_AMBIGUOUS);
  EXPECT_EQ(record.reason(), uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_AMBIGUOUS_MARGIN);
}
