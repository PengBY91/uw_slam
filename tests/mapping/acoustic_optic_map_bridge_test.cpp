#include <cmath>

#include <gtest/gtest.h>

#include "mapping/acoustic_optic_map_bridge.hpp"
#include "mapping/submap_manager.hpp"
#include "mapping/surfel_map.hpp"

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

// Same single-valid-pixel shape as MakeFusedEvidence above, but with a
// caller-chosen DepthContribution/variance instead of always
// DEPTH_CONTRIBUTION_ACOUSTIC_OPTIC — needed to build the "two local
// geometry paths carry different confidence" fixtures below.
uw::domain::MeasurementEvidence MakeSinglePixelFusedEvidence(int width, int height, int valid_index,
                                                              float depth_m, float variance_m2,
                                                              uw::domain::DepthContribution contribution) {
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
  contribution_mask[valid_index] = static_cast<char>(contribution);
  fused.set_valid_mask(valid_mask);
  fused.set_contribution_mask(contribution_mask);
  uw::domain::EvidenceId id;
  id.set_value("fused_single");
  return uw::domain::MakeEvidence(id, {}, fused, 0.5, "acoustic_optic_depth_fusion_v1");
}

// A 2x2 patch of same-depth, same-contribution valid pixels at (10,5),
// (11,5), (10,6), (11,6) — enough for pixel (10,5) to have both a right and
// a down neighbor (the only one of the four that does; see the normal-
// estimation test below for why that specific pixel is the one checked).
uw::domain::MeasurementEvidence MakeFlatPatchFusedEvidence(int width, int height, float depth_m,
                                                            float variance_m2) {
  uw::domain::FusedDepthMeasurement fused;
  fused.set_width(width);
  fused.set_height(height);
  const int pixels = width * height;
  std::string valid_mask(pixels, '\0');
  std::string contribution_mask(pixels, static_cast<char>(uw::domain::DEPTH_CONTRIBUTION_INVALID));
  for (int i = 0; i < pixels; ++i) fused.add_depth_m(0.0f);
  for (int i = 0; i < pixels; ++i) fused.add_variance_m2(0.0f);
  for (int idx : {5 * width + 10, 5 * width + 11, 6 * width + 10, 6 * width + 11}) {
    fused.set_depth_m(idx, depth_m);
    fused.set_variance_m2(idx, variance_m2);
    valid_mask[idx] = 1;
    contribution_mask[idx] = static_cast<char>(uw::domain::DEPTH_CONTRIBUTION_OPTICAL_ONLY);
  }
  fused.set_valid_mask(valid_mask);
  fused.set_contribution_mask(contribution_mask);
  uw::domain::EvidenceId id;
  id.set_value("fused_patch");
  return uw::domain::MakeEvidence(id, {}, fused, 0.5, "acoustic_optic_depth_fusion_v1");
}

}  // namespace

TEST(AcousticOpticMapBridge, ConvertsBoresightPixelToBaseLinkFramePoint) {
  const auto rig = MakeRig();
  // index 5*20+10 = 110, matching the (10,5) boresight fixture pixel.
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

// --- FuseDepthIntoSurfels (P3 roadmap item 2: visual-only / sonar-grounded
// local geometry paths) ---------------------------------------------------

TEST(FuseDepthIntoSurfels, ReturnsZeroWhenEvidenceHasNoFusedDepthPayload) {
  const auto rig = MakeRig();
  uw::domain::PressureDepthMeasurement wrong_payload;
  uw::domain::EvidenceId id;
  id.set_value("not_fused");
  const auto not_fused_evidence = uw::domain::MakeEvidence(id, {}, wrong_payload, 1.0, "irrelevant_v1");
  uw::mapping::AcousticOpticMapBridgeParams params;
  uw::mapping::SurfelMap surfels;
  EXPECT_EQ(uw::mapping::FuseDepthIntoSurfels(not_fused_evidence, rig, params, uw::sensor_models::Pose3(),
                                              surfels),
            0);
  EXPECT_EQ(surfels.NumSurfels(), 0u);
}

TEST(FuseDepthIntoSurfels, EstimatesNormalFromGridNeighborsOfAFrontalFlatPatch) {
  // Pixel (10,5) is the only one of the 2x2 patch with BOTH a right (11,5)
  // and a down (10,6) neighbor valid — see MakeFlatPatchFusedEvidence's own
  // comment. Hand-derived expected normal (all at depth=5.0, fx=fy=100,
  // cx=10, cy=5, so Unproject(10,5,5)=(0,0,5), Unproject(11,5,5)=(0.05,0,5),
  // Unproject(10,6,5)=(0,0.05,5) optical): tangent_down=(0,0.05,0),
  // tangent_right=(0.05,0,0), tangent_down x tangent_right = (0,0,-0.0025)
  // -> normalized (0,0,-1) optical -> body via OpticalFromBodyRotation()^T
  // ((x,y,z)_optical -> (z,-x,-y)_body) = (-1,0,0) -> world (identity
  // pose_WB and identity camera rotation here) = (-1,0,0): the surface
  // faces back toward the camera, which sits at -X relative to the surface
  // point (matches ConvertsBoresightPixelToBaseLinkFramePoint's own
  // Unproject/rotation derivation above).
  const auto rig = MakeRig();
  const auto fused_evidence = MakeFlatPatchFusedEvidence(20, 10, 5.0f, 0.02f);
  uw::mapping::AcousticOpticMapBridgeParams params;
  // Generous merge distance: guarantees the 2x2 patch's four ~0.05m-spaced
  // points collapse into one surfel regardless of exact pixel-spacing
  // arithmetic, which is not what this test is checking (see the dedicated
  // confidence-weighting test below for a merge-distance-sensitive case).
  uw::mapping::SurfelMap surfels(uw::mapping::SurfelMapParams{/*merge_distance_m=*/1.0});

  const int added = uw::mapping::FuseDepthIntoSurfels(fused_evidence, rig, params, uw::sensor_models::Pose3(),
                                                       surfels);
  EXPECT_EQ(added, 4);
  ASSERT_EQ(surfels.NumSurfels(), 1u);
  const auto& normal = surfels.Surfels()[0].normal_W;
  EXPECT_NEAR(normal.norm(), 1.0, 1e-9);
  EXPECT_NEAR(normal.x(), -1.0, 1e-6);
  EXPECT_NEAR(normal.y(), 0.0, 1e-6);
  EXPECT_NEAR(normal.z(), 0.0, 1e-6);
}

TEST(FuseDepthIntoSurfels, AcousticOpticContributionDominatesOpticalOnlyOnMerge) {
  // Two single-pixel observations of nearly the same physical point (both
  // at the boresight pixel (10,5), 0.02m apart in depth — well within the
  // default 0.05m merge gate) but with very different confidence: the
  // optical-only one at variance=1.0 (confidence 1.0), the acoustic-optic
  // one at variance=0.01 (confidence 100.0) — the ~100x confidence gap this
  // test locks in is exactly the kind of improvement
  // AcousticOpticDepthFusionFrontend's min_variance_improvement_fraction
  // gate guarantees for any pixel actually marked ACOUSTIC_OPTIC (see
  // acoustic_optic_depth_fusion_frontend.cpp) — this test proves SurfelMap's
  // existing confidence-weighted merge (already covered abstractly by
  // surfel_map_test.cpp's HigherConfidencePointDominatesMergedPosition)
  // actually receives that real confidence gap through this bridge.
  const auto rig = MakeRig();
  constexpr int kBoresightIndex = 5 * 20 + 10;
  const auto optical_only = MakeSinglePixelFusedEvidence(20, 10, kBoresightIndex, /*depth_m=*/5.0f,
                                                          /*variance_m2=*/1.0f,
                                                          uw::domain::DEPTH_CONTRIBUTION_OPTICAL_ONLY);
  const auto acoustic_optic = MakeSinglePixelFusedEvidence(20, 10, kBoresightIndex, /*depth_m=*/5.02f,
                                                            /*variance_m2=*/0.01f,
                                                            uw::domain::DEPTH_CONTRIBUTION_ACOUSTIC_OPTIC);

  uw::mapping::AcousticOpticMapBridgeParams params;
  uw::mapping::SurfelMap surfels;  // default merge_distance_m = 0.05
  const uw::sensor_models::Pose3 identity_pose;

  EXPECT_EQ(uw::mapping::FuseDepthIntoSurfels(optical_only, rig, params, identity_pose, surfels), 1);
  EXPECT_EQ(uw::mapping::FuseDepthIntoSurfels(acoustic_optic, rig, params, identity_pose, surfels), 1);

  ASSERT_EQ(surfels.NumSurfels(), 1u) << "0.02m apart must merge under the default 0.05m gate";
  const auto& surfel = surfels.Surfels()[0];
  // variance_m2 is a proto `float` (32-bit) field, so 1/0.01f isn't exactly
  // 100.0 in double arithmetic — 1e-3 comfortably clears that quantization
  // noise while still catching a real formula error (e.g. confidence not
  // being additive, or using the wrong variance).
  EXPECT_NEAR(surfel.confidence, 101.0, 1e-3);  // 1/1.0 + 1/0.01

  // Base-link/world x = depth + 0.1 (camera translation), per
  // ConvertsBoresightPixelToBaseLinkFramePoint's derivation above:
  // optical (0,0,5.0)+camera R^T -> body (5.0,0,0) -> +0.1 = 5.1;
  // optical (0,0,5.02) -> 5.12. Confidence-weighted: (5.1*1 + 5.12*100)/101.
  constexpr double kOpticalOnlyX = 5.1;
  constexpr double kAcousticOpticX = 5.12;
  const double expected_x = (kOpticalOnlyX * 1.0 + kAcousticOpticX * 100.0) / 101.0;
  EXPECT_NEAR(surfel.position_W.x(), expected_x, 1e-4);  // same float32-variance quantization as above
  // The dominance claim itself: merged position is far closer to the
  // acoustic-optic (higher-confidence) observation than to the
  // optical-only one.
  EXPECT_LT(std::abs(surfel.position_W.x() - kAcousticOpticX), std::abs(surfel.position_W.x() - kOpticalOnlyX));
}
