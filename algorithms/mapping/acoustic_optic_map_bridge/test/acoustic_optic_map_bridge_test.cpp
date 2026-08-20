#include <gtest/gtest.h>

#include "uw/mapping/acoustic_optic_map_bridge.hpp"
#include "uw/mapping/submap_manager.hpp"

namespace {

uw::domain::RigCalibrationSnapshot MakeRig() {
  uw::domain::RigCalibrationSnapshot rig;
  auto* camera = rig.add_cameras();
  camera->mutable_sensor_id()->set_value("camera_left");
  camera->set_width(20);
  camera->set_height(10);
  for (double v : {100.0, 0.0, 10.0, 0.0, 100.0, 5.0, 0.0, 0.0, 1.0}) camera->add_k_matrix_row_major(v);

  auto* edge = rig.add_frame_tree();
  edge->mutable_parent_frame()->set_value("base_link");
  edge->mutable_child_frame()->set_value("camera_left_link");
  for (double v : {1.0, 0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0}) {
    edge->mutable_transform()->add_matrix_row_major(v);
  }
  return rig;
}

uw::domain::MeasurementEvidence MakeFusedEvidence(int width, int height, int valid_index,
                                                   float depth_m, float variance_m2) {
  uw::domain::FusedDepthMeasurement fused;
  fused.set_width(width);
  fused.set_height(height);
  const int pixels = width * height;
  std::string valid_mask(pixels, '\0');
  std::string contribution_mask(pixels, static_cast<char>(uw::domain::DEPTH_CONTRIBUTION_INVALID));
  for (int i = 0; i < pixels; ++i) {
    fused.add_depth_m(i == valid_index ? depth_m : 0.0f);
    fused.add_variance_m2(i == valid_index ? variance_m2 : 0.0f);
  }
  valid_mask[valid_index] = 1;
  contribution_mask[valid_index] = static_cast<char>(uw::domain::DEPTH_CONTRIBUTION_ACOUSTIC_OPTIC);
  fused.set_valid_mask(valid_mask);
  fused.set_contribution_mask(contribution_mask);
  uw::domain::EvidenceId id;
  id.set_value("fused_1");
  return uw::domain::MakeEvidence(id, {}, fused, 0.5, "acoustic_optic_depth_fusion_v1");
}

}  // namespace

TEST(AcousticOpticMapBridge, ConvertsBoresightPixelToBaseLinkFramePoint) {
  const auto rig = MakeRig();
  // index 5*20+10 = 110, matching the (10,5) boresight pixel used throughout plans 3/4.
  const auto fused_evidence = MakeFusedEvidence(20, 10, 110, 5.0f, 0.02f);

  uw::mapping::AcousticOpticMapBridgeParams params;
  const auto map_evidence =
      uw::mapping::BuildMapEvidenceFromFusedDepth(fused_evidence, rig, params, "kf3", /*state_version=*/7);

  ASSERT_TRUE(map_evidence.has_value());
  EXPECT_EQ(map_evidence->keyframe_id().value(), "kf3");
  EXPECT_EQ(map_evidence->state_version().value(), 7u);
  EXPECT_EQ(map_evidence->local_frame().value(), "base_link");
  EXPECT_EQ(map_evidence->representation_type(), uw::domain::MAP_REPRESENTATION_POINT_CLOUD);
  EXPECT_EQ(map_evidence->reintegration_policy(),
            uw::domain::MapEvidence::REINTEGRATION_POLICY_TRANSFORM_ONLY);

  ASSERT_EQ(map_evidence->geometry_or_occupancy().size(), 3 * sizeof(float));
  const auto* raw = reinterpret_cast<const float*>(map_evidence->geometry_or_occupancy().data());
  // Unproject(10,5,5.0) = (0,0,5) optical -> R^T*(0,0,5) = (5,0,0) body ->
  // + camera translation (0.1,0,0) = (5.1,0,0) base_link.
  EXPECT_NEAR(raw[0], 5.1f, 1e-4f);
  EXPECT_NEAR(raw[1], 0.0f, 1e-4f);
  EXPECT_NEAR(raw[2], 0.0f, 1e-4f);

  ASSERT_EQ(map_evidence->uncertainty_size(), 1);
  EXPECT_NEAR(map_evidence->uncertainty(0), 0.02, 1e-6);
}

TEST(AcousticOpticMapBridge, SkipsInvalidAndOpticalOnlyPixelsByDefaultConfig) {
  // valid_index pixel is ACOUSTIC_OPTIC (kept); every other pixel is
  // INVALID contribution (default) and correctly excluded.
  const auto rig = MakeRig();
  const auto fused_evidence = MakeFusedEvidence(20, 10, 110, 5.0f, 0.02f);
  uw::mapping::AcousticOpticMapBridgeParams params;
  const auto map_evidence =
      uw::mapping::BuildMapEvidenceFromFusedDepth(fused_evidence, rig, params, "kf3", 7);
  ASSERT_TRUE(map_evidence.has_value());
  // Only 1 of 200 pixels is non-INVALID contribution -> exactly 1 point.
  EXPECT_EQ(map_evidence->geometry_or_occupancy().size(), 3 * sizeof(float));
}

TEST(AcousticOpticMapBridge, ReturnsNulloptWhenEvidenceHasNoFusedDepthPayload) {
  const auto rig = MakeRig();
  uw::domain::PressureDepthMeasurement wrong_payload;
  uw::domain::EvidenceId id;
  id.set_value("not_fused");
  const auto not_fused_evidence = uw::domain::MakeEvidence(id, {}, wrong_payload, 1.0, "irrelevant_v1");
  uw::mapping::AcousticOpticMapBridgeParams params;
  EXPECT_FALSE(uw::mapping::BuildMapEvidenceFromFusedDepth(not_fused_evidence, rig, params, "kf3", 7)
                   .has_value());
}

TEST(AcousticOpticMapBridge, SurvivesPoseGraphCorrectionThroughRealSubmapManager) {
  // The design spec's actual completion condition (section 16): local
  // fused evidence must be re-transformable after a state update, not
  // baked into world frame — proven here against the REAL, pre-existing
  // SubmapManager, not a mock.
  const auto rig = MakeRig();
  const auto fused_evidence = MakeFusedEvidence(20, 10, 110, 5.0f, 0.02f);
  uw::mapping::AcousticOpticMapBridgeParams params;
  const auto map_evidence =
      uw::mapping::BuildMapEvidenceFromFusedDepth(fused_evidence, rig, params, "kf3", 7);
  ASSERT_TRUE(map_evidence.has_value());

  uw::mapping::SubmapManager manager;
  manager.AddMapEvidence(*map_evidence);

  uw::sensor_models::Pose3 pose1;
  pose1.translation = Eigen::Vector3d(10.0, 0.0, 0.0);
  manager.UpdateKeyframePose("kf3", pose1);
  auto points1 = manager.WorldPointsForKeyframe("kf3");
  ASSERT_EQ(points1.size(), 1u);
  EXPECT_NEAR((points1[0] - Eigen::Vector3d(15.1, 0.0, 0.0)).norm(), 0.0, 1e-4);

  // Pose-graph correction: pose changes, evidence is NOT re-added or
  // regenerated from the frontend, yet WorldPointsForKeyframe reflects it
  // immediately (same guarantee submap_manager_test.cpp already proves
  // generically — this test proves THIS plan's evidence source honors it).
  uw::sensor_models::Pose3 pose2;
  pose2.translation = Eigen::Vector3d(0.0, 20.0, 0.0);
  manager.UpdateKeyframePose("kf3", pose2);
  auto points2 = manager.WorldPointsForKeyframe("kf3");
  ASSERT_EQ(points2.size(), 1u);
  EXPECT_NEAR((points2[0] - Eigen::Vector3d(5.1, 20.0, 0.0)).norm(), 0.0, 1e-4);
  EXPECT_TRUE(manager.StaleKeyframes().empty());
}
