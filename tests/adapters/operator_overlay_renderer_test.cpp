#include "adapters/operator_overlay_renderer.hpp"

#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace {

using uw::opencv_adapters::OperatorOverlayRenderer;

uw::domain::ImageFrame MakeRgbFrame() {
  uw::domain::ImageFrame image;
  image.mutable_header()->mutable_observation_id()->set_value("pilot_obs");
  image.mutable_header()->mutable_sensor_id()->set_value("pilot_camera");
  image.mutable_header()->mutable_sensor_frame()->set_value("pilot_camera_link");
  *image.mutable_header()->mutable_capture_time() = uw::domain::FromSeconds(10.0);
  image.set_width(64);
  image.set_height(48);
  image.set_row_stride_bytes(64 * 3);
  image.set_encoding(uw::domain::ImageFrame::IMAGE_ENCODING_RGB8);
  image.set_pixel_data(std::string(static_cast<std::size_t>(64) * 48 * 3, '\x20'));
  return image;
}

uw::domain::SonarFrame MakeSonarViewFrame() {
  constexpr int kNumRanges = 40;
  constexpr int kNumBeams = 32;
  uw::domain::SonarFrame frame;
  frame.mutable_header()->mutable_observation_id()->set_value("sonar_overlay_obs");
  frame.mutable_header()->mutable_sensor_id()->set_value("sonar0");
  frame.mutable_header()->mutable_sensor_frame()->set_value("sonar_link");
  *frame.mutable_header()->mutable_capture_time() = uw::domain::FromSeconds(9.9);
  frame.set_num_ranges(kNumRanges);
  frame.set_num_beams(kNumBeams);
  frame.set_range_resolution(0.1f);
  frame.set_min_range(0.0f);
  frame.set_max_range(4.0f);
  frame.set_encoding(uw::domain::SonarFrame::ENCODING_UINT8_GRAY);
  for (int col = 0; col < kNumBeams; ++col) {
    frame.add_azimuth_angles(-0.8f + 1.6f * static_cast<float>(col) / (kNumBeams - 1));
  }
  frame.set_intensity_tensor(std::string(static_cast<std::size_t>(kNumRanges) * kNumBeams,
                                        static_cast<char>(30)));
  return frame;
}

// Single SONAR-only track, publish_time exactly 120ms after last_capture_time,
// and a SUSPECT/visual_unavailable system health -- matches the exact label
// strings from docs/archive/superpowers/plans/2026-08-24-acoustic-optic-online-
// tracking.md Task 7's own example test.
uw::domain::OperatorAssistState MakeDegradedAssistState() {
  uw::domain::OperatorAssistState state;
  auto* track = state.mutable_target_tracks()->add_tracks();
  track->mutable_track_id()->set_value("track_1");
  track->set_class_label("sonar_target");  // generic -- omitted from the label
  track->set_class_confidence(0.82);
  track->set_bearing_rad(0.1);
  track->set_range_m(4.0);
  track->add_covariance_2x2_row_major(0.001);
  track->add_covariance_2x2_row_major(0.0);
  track->add_covariance_2x2_row_major(0.0);
  track->add_covariance_2x2_row_major(0.04);
  *track->mutable_first_capture_time() = uw::domain::FromSeconds(9.88);
  *track->mutable_last_capture_time() = uw::domain::FromSeconds(9.88);
  *track->mutable_publish_time() = uw::domain::FromSeconds(10.0);  // 120ms after last_capture_time
  track->add_sources(uw::domain::ASSIST_SOURCE_SONAR);
  track->set_status(uw::domain::TARGET_TRACK_STATUS_CONFIRMED);

  auto* health = state.mutable_system_health();
  health->set_component_id("online_assist_pipeline");
  health->set_status(uw::domain::HealthReport::STATUS_SUSPECT);
  health->set_reason_code("visual_unavailable");
  state.set_guidance_valid(true);
  state.set_degradation_reason("visual_unavailable");
  return state;
}

}  // namespace

TEST(OperatorOverlayRenderer, MarksSourceAgeAndDegradedState) {
  OperatorOverlayRenderer renderer;
  const auto rendered = renderer.Render(MakeRgbFrame(), MakeSonarViewFrame(), MakeDegradedAssistState());
  ASSERT_TRUE(rendered.has_value());
  EXPECT_EQ(rendered->encoding(), uw::domain::ImageFrame::IMAGE_ENCODING_RGB8);
  EXPECT_NE(rendered->pixel_data(), MakeRgbFrame().pixel_data());
  EXPECT_EQ(renderer.LastLabelsForTest(),
           (std::vector<std::string>{"track_1 SONAR 4.0m c0.82 120ms", "DEGRADED visual_unavailable"}));
}

TEST(OperatorOverlayRenderer, OmitsHealthBannerWhenHealthy) {
  OperatorOverlayRenderer renderer;
  auto state = MakeDegradedAssistState();
  state.mutable_system_health()->set_status(uw::domain::HealthReport::STATUS_HEALTHY);
  state.mutable_system_health()->set_reason_code("");

  const auto rendered = renderer.Render(MakeRgbFrame(), MakeSonarViewFrame(), state);
  ASSERT_TRUE(rendered.has_value());
  ASSERT_EQ(renderer.LastLabelsForTest().size(), 1u);
  EXPECT_EQ(renderer.LastLabelsForTest()[0], "track_1 SONAR 4.0m c0.82 120ms");
}

TEST(OperatorOverlayRenderer, RendersWithoutSonarFrameWhenNoneProvided) {
  OperatorOverlayRenderer renderer;
  const auto pilot = MakeRgbFrame();
  const auto rendered = renderer.Render(pilot, std::nullopt, MakeDegradedAssistState());
  ASSERT_TRUE(rendered.has_value());
  EXPECT_EQ(rendered->encoding(), uw::domain::ImageFrame::IMAGE_ENCODING_RGB8);
  // No sonar side panel appended -- output stays the pilot frame's own width.
  EXPECT_EQ(rendered->width(), pilot.width());
  EXPECT_EQ(rendered->height(), pilot.height());
  EXPECT_EQ(renderer.LastLabelsForTest().size(), 2u);
}

TEST(OperatorOverlayRenderer, DegradesGracefullyOnMalformedSonarFrame) {
  OperatorOverlayRenderer renderer;
  const auto pilot = MakeRgbFrame();
  auto sonar = MakeSonarViewFrame();
  sonar.set_num_beams(0);  // now inconsistent with intensity_tensor/azimuth_angles size

  const auto rendered = renderer.Render(pilot, sonar, MakeDegradedAssistState());
  ASSERT_TRUE(rendered.has_value());
  EXPECT_EQ(rendered->width(), pilot.width());  // panel silently omitted, not appended
  EXPECT_EQ(rendered->height(), pilot.height());
}

TEST(OperatorOverlayRenderer, RejectsUnsupportedPilotEncoding) {
  OperatorOverlayRenderer renderer;
  auto pilot = MakeRgbFrame();
  pilot.set_encoding(uw::domain::ImageFrame::IMAGE_ENCODING_MONO8);
  pilot.set_row_stride_bytes(pilot.width());
  pilot.set_pixel_data(std::string(pilot.width() * pilot.height(), '\x10'));

  EXPECT_FALSE(renderer.Render(pilot, std::nullopt, MakeDegradedAssistState()).has_value());
}
