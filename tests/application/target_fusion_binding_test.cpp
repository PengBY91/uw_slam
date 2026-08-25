#include "frontends/target_fusion_components.hpp"

#include <algorithm>
#include <optional>
#include <string>

#include <gtest/gtest.h>

#include "runtime/config.hpp"
#include "runtime/run_manifest.hpp"
#include "sensor_models/geometry.hpp"

namespace {

uw::domain::Stamp Stamp(double seconds) {
  uw::domain::Stamp stamp;
  stamp.set_seconds(static_cast<int64_t>(seconds));
  stamp.set_nanos(static_cast<int32_t>(
      (seconds - static_cast<double>(stamp.seconds())) * 1.0e9));
  return stamp;
}

void AddIdentityEdge(uw::domain::RigCalibrationSnapshot* rig,
                     const std::string& child) {
  auto* edge = rig->add_frame_tree();
  edge->mutable_parent_frame()->set_value("base_link");
  edge->mutable_child_frame()->set_value(child);
  *edge->mutable_transform() = uw::sensor_models::Pose3::Identity().ToProto();
}

uw::domain::RigCalibrationSnapshot Rig() {
  uw::domain::RigCalibrationSnapshot rig;
  rig.mutable_calibration_version()->set_value("binding-rig");
  AddIdentityEdge(&rig, "camera_left_link");
  AddIdentityEdge(&rig, "sonar_link");
  rig.add_cameras()->mutable_sensor_id()->set_value("camera_left");
  auto* sonar = rig.add_sonar_beam_models();
  sonar->mutable_sensor_id()->set_value("sonar0");
  sonar->set_sonar_enabled(true);
  (*rig.mutable_time_offset_seconds())["camera_left"] = 0.0;
  (*rig.mutable_time_offset_seconds())["sonar0"] = 0.0;
  (*rig.mutable_time_offset_provenance())["camera_left"] = "measured:test";
  (*rig.mutable_time_offset_provenance())["sonar0"] = "measured:test";
  return rig;
}

uw::frontends::SensorTargetDetection Detection(
    const std::string& id, uw::domain::AssistSource source, double time_s) {
  uw::frontends::SensorTargetDetection input;
  input.sensor_id = source == uw::domain::ASSIST_SOURCE_VISUAL
                        ? "camera_left"
                        : "sonar0";
  input.sensor_frame = source == uw::domain::ASSIST_SOURCE_VISUAL
                           ? "camera_left_link"
                           : "sonar_link";
  input.calibration_version = "binding-rig";
  input.detection.mutable_source_observation()->set_value(id);
  *input.detection.mutable_capture_time() = Stamp(time_s);
  input.detection.set_class_label("target");
  input.detection.set_confidence(0.9);
  input.detection.set_bearing_rad(0.0);
  input.detection.set_has_range(source == uw::domain::ASSIST_SOURCE_SONAR);
  input.detection.set_range_m(
      source == uw::domain::ASSIST_SOURCE_SONAR ? 4.0 : 0.0);
  input.detection.add_covariance_2x2_row_major(1.0e-4);
  input.detection.add_covariance_2x2_row_major(0.0);
  input.detection.add_covariance_2x2_row_major(0.0);
  input.detection.add_covariance_2x2_row_major(
      source == uw::domain::ASSIST_SOURCE_SONAR ? 0.04 : 1000.0);
  input.detection.set_source(source);
  return input;
}

uw::frontends::TargetMeasurement VisualMeasurement() {
  uw::frontends::TargetMeasurement measurement;
  measurement.corrected_time_s = 2.0;
  measurement.class_label = "target";
  measurement.confidence = 0.9;
  measurement.bearing_rad = 0.0;
  measurement.covariance.setZero();
  measurement.covariance(0, 0) = 1.0e-4;
  measurement.covariance(1, 1) = 1000.0;
  measurement.sources = {uw::domain::ASSIST_SOURCE_VISUAL};
  measurement.observation_ids.emplace_back();
  measurement.observation_ids.back().set_value("tracker-visual");
  return measurement;
}

}  // namespace

TEST(TargetFusionBinding, OneResolvedConfigDrivesComponentsAndManifestFragment) {
  auto config = uw::runtime::LoadPlatformDefaultsConfig(
      std::string(UW_REPO_ROOT) + "/configs/defaults/platform.yaml");
  config.target_association.max_corrected_time_delta_s = 0.001;
  config.target_tracker.confirm_hits = 1;

  uw::frontends::TargetFusionComponents components(
      config.target_association, config.target_tracker);
  const auto association = components.associator().Associate(
      {Detection("visual", uw::domain::ASSIST_SOURCE_VISUAL, 1.0)},
      {Detection("sonar", uw::domain::ASSIST_SOURCE_SONAR, 1.01)}, Rig());
  ASSERT_EQ(association.measurements.size(), 2u);
  const auto pair = std::find_if(
      association.diagnostics.begin(), association.diagnostics.end(),
      [](const auto& diagnostic) {
        return diagnostic.reason ==
               uw::frontends::AssociationReason::kCorrectedTimeDelta;
      });
  ASSERT_NE(pair, association.diagnostics.end());
  EXPECT_DOUBLE_EQ(pair->threshold, 0.001);

  ASSERT_TRUE(components.tracker().Update({VisualMeasurement()}, 2.0));
  ASSERT_EQ(components.tracker().Tracks(2.0).size(), 1u);
  EXPECT_EQ(components.tracker().Tracks(2.0)[0].status,
            uw::domain::TARGET_TRACK_STATUS_CONFIRMED);

  const std::string& fragment = components.manifest_parameters();
  EXPECT_NE(fragment.find("max_corrected_time_delta_s=0.001"),
            std::string::npos);
  EXPECT_NE(fragment.find("confirm_hits=1"), std::string::npos);
  uw::runtime::RunManifest manifest;
  manifest.target_fusion_parameters = fragment;
  EXPECT_NE(manifest.ToJson().find(fragment), std::string::npos);
}
