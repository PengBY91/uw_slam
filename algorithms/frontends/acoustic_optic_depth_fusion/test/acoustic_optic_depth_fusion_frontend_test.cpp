#include <cmath>

#include <gtest/gtest.h>

#include "uw/frontends/acoustic_optic_depth_fusion_frontend.hpp"

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

uw::domain::HypothesisSet MakeSonarHypothesis(double range_m, double bearing_rad,
                                               double range_sigma_m, double bearing_sigma_rad) {
  uw::domain::SonarRangeBearing measurement;
  measurement.set_range_m(range_m);
  measurement.set_bearing_rad(bearing_rad);
  measurement.set_range_sigma_m(range_sigma_m);
  measurement.set_bearing_sigma_rad(bearing_sigma_rad);
  uw::domain::EvidenceId id;
  id.set_value("sonar_hyp_1");
  auto evidence = uw::domain::MakeEvidence(id, {}, measurement, 1.0, "sonar_cfar_frontend_v1");
  uw::domain::HypothesisSet hypotheses;
  *hypotheses.add_candidates() = evidence;
  return hypotheses;
}

uw::domain::MeasurementEvidence MakeOpticalEvidence(int width, int height, int valid_index,
                                                     float depth_m, float variance_m2) {
  uw::domain::OpticalDepthPriorMeasurement prior;
  prior.set_width(width);
  prior.set_height(height);
  const int pixels = width * height;
  std::string valid_mask(pixels, '\0');
  for (int i = 0; i < pixels; ++i) {
    prior.add_depth_m(i == valid_index ? depth_m : 1.0f);
    prior.add_variance_m2(i == valid_index ? variance_m2 : 0.01f);
  }
  valid_mask[valid_index] = 1;
  valid_mask[5] = 1;  // a second always-valid pixel, distinct from valid_index, to test passthrough
  prior.set_valid_mask(valid_mask);
  prior.set_scale_status(uw::domain::OPTICAL_DEPTH_SCALE_STATUS_METRIC);
  prior.set_producer_type("stereo");
  uw::domain::EvidenceId id;
  id.set_value("optical_1");
  return uw::domain::MakeEvidence(id, {}, prior, 1.0, "stereo_depth_frontend_v1");
}

}  // namespace

TEST(AcousticOpticDepthFusionFrontend, AcceptsAndCorrectsTheSelectedPixel) {
  const auto rig = MakeCoLocatedRig();
  const auto sonar_hypotheses =
      MakeSonarHypothesis(/*range_m=*/5.0, 0.0, /*range_sigma_m=*/0.05, /*bearing_sigma_rad=*/0.02);
  const auto optical_evidence =
      MakeOpticalEvidence(20, 10, /*valid_index=*/110, /*depth_m=*/5.2, /*variance_m2=*/0.09);

  uw::frontends::AcousticOpticDepthFusionParams params;
  params.optimizer.iterations = 60;
  uw::frontends::AcousticOpticDepthFusionFrontend fusion(params);
  const auto result = fusion.Fuse(sonar_hypotheses, optical_evidence, rig, /*time_delta_seconds=*/0.0);

  ASSERT_TRUE(result.has_value());
  const auto& fused = uw::domain::GetPayload<uw::domain::FusedDepthMeasurement>(result->fused_evidence);
  ASSERT_EQ(uw::domain::ValidateFusedDepth(fused).code, uw::domain::ValidationCode::kOk);

  const double w_prior = 1.0 / 0.09, w_sonar = 1.0 / (0.05 * 0.05);
  const double expected_depth = (5.2 * w_prior + 5.0 * w_sonar) / (w_prior + w_sonar);
  EXPECT_NEAR(fused.depth_m(110), expected_depth, 1e-3);
  EXPECT_LT(fused.variance_m2(110), 0.09);
  EXPECT_EQ(static_cast<unsigned char>(fused.contribution_mask()[110]),
            uw::domain::DEPTH_CONTRIBUTION_ACOUSTIC_OPTIC);
  // An untouched but optical-valid pixel stays a pure passthrough.
  EXPECT_NEAR(fused.depth_m(5), 1.0, 1e-6);
  EXPECT_EQ(static_cast<unsigned char>(fused.contribution_mask()[5]),
            uw::domain::DEPTH_CONTRIBUTION_OPTICAL_ONLY);
  // Pixel 0 was never marked valid in the fixture.
  EXPECT_EQ(static_cast<unsigned char>(fused.contribution_mask()[0]),
            uw::domain::DEPTH_CONTRIBUTION_INVALID);

  ASSERT_EQ(fused.associations_size(), 1);
  EXPECT_EQ(fused.associations(0).status(), uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_ACCEPTED);
  EXPECT_NEAR(fused.associations(0).posterior_depth_m(), expected_depth, 1e-3);
}

TEST(AcousticOpticDepthFusionFrontend, FallsBackToOpticalOnlyOnSonarDropout) {
  const auto rig = MakeCoLocatedRig();
  uw::domain::HypothesisSet empty_hypotheses;
  const auto optical_evidence = MakeOpticalEvidence(20, 10, 110, 5.2, 0.09);

  uw::frontends::AcousticOpticDepthFusionParams params;
  uw::frontends::AcousticOpticDepthFusionFrontend fusion(params);
  const auto result = fusion.Fuse(empty_hypotheses, optical_evidence, rig, 0.0);

  ASSERT_TRUE(result.has_value());
  const auto& fused = uw::domain::GetPayload<uw::domain::FusedDepthMeasurement>(result->fused_evidence);
  EXPECT_EQ(fused.associations_size(), 0);
  EXPECT_EQ(static_cast<unsigned char>(fused.contribution_mask()[110]),
            uw::domain::DEPTH_CONTRIBUTION_OPTICAL_ONLY);
  EXPECT_NEAR(fused.depth_m(110), 5.2, 1e-6);  // unchanged
}

TEST(AcousticOpticDepthFusionFrontend, FallsBackWhenInnovationGateRejectsThePosterior) {
  const auto rig = MakeCoLocatedRig();
  const auto sonar_hypotheses = MakeSonarHypothesis(5.0, 0.0, 0.05, 0.02);
  const auto optical_evidence = MakeOpticalEvidence(20, 10, 110, 5.2, 0.09);

  uw::frontends::AcousticOpticDepthFusionParams params;
  params.optimizer.iterations = 60;
  params.innovation_gate_sigma = 0.05;  // unrealistically tight — the ~0.005m residual will fail it
  uw::frontends::AcousticOpticDepthFusionFrontend fusion(params);
  const auto result = fusion.Fuse(sonar_hypotheses, optical_evidence, rig, 0.0);

  ASSERT_TRUE(result.has_value());
  const auto& fused = uw::domain::GetPayload<uw::domain::FusedDepthMeasurement>(result->fused_evidence);
  ASSERT_EQ(fused.associations_size(), 1);
  EXPECT_EQ(fused.associations(0).status(), uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_CONFLICT);
  EXPECT_EQ(fused.associations(0).reason(), uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_CROSS_MODAL_CONFLICT);
  EXPECT_EQ(static_cast<unsigned char>(fused.contribution_mask()[110]),
            uw::domain::DEPTH_CONTRIBUTION_OPTICAL_ONLY);
  EXPECT_NEAR(fused.depth_m(110), 5.2, 1e-6);  // fallback: unchanged
}

TEST(AcousticOpticDepthFusionFrontend, FallsBackWhenVarianceIsNotSufficientlyImproved) {
  const auto rig = MakeCoLocatedRig();
  const auto sonar_hypotheses = MakeSonarHypothesis(5.0, 0.0, 0.05, 0.02);
  const auto optical_evidence = MakeOpticalEvidence(20, 10, 110, 5.2, 0.09);

  uw::frontends::AcousticOpticDepthFusionParams params;
  params.optimizer.iterations = 60;
  params.min_variance_improvement_fraction = 0.999;  // demand a 99.9% reduction — unattainable here
  uw::frontends::AcousticOpticDepthFusionFrontend fusion(params);
  const auto result = fusion.Fuse(sonar_hypotheses, optical_evidence, rig, 0.0);

  ASSERT_TRUE(result.has_value());
  const auto& fused = uw::domain::GetPayload<uw::domain::FusedDepthMeasurement>(result->fused_evidence);
  ASSERT_EQ(fused.associations_size(), 1);
  EXPECT_EQ(fused.associations(0).status(), uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_STATUS_REJECTED);
  EXPECT_EQ(fused.associations(0).reason(), uw::domain::ACOUSTIC_OPTIC_ASSOCIATION_REASON_VARIANCE_NOT_IMPROVED);
}

TEST(AcousticOpticDepthFusionFrontend, ReturnsNulloptWhenOpticalEvidenceHasNoPrior) {
  const auto rig = MakeCoLocatedRig();
  const auto sonar_hypotheses = MakeSonarHypothesis(5.0, 0.0, 0.05, 0.02);
  uw::domain::PressureDepthMeasurement wrong_payload;
  uw::domain::EvidenceId id;
  id.set_value("not_optical");
  const auto not_optical_evidence =
      uw::domain::MakeEvidence(id, {}, wrong_payload, 1.0, "irrelevant_v1");

  uw::frontends::AcousticOpticDepthFusionParams params;
  uw::frontends::AcousticOpticDepthFusionFrontend fusion(params);
  EXPECT_FALSE(fusion.Fuse(sonar_hypotheses, not_optical_evidence, rig, 0.0).has_value());
}
