#include <optional>
#include <utility>

#include <gtest/gtest.h>

#include "application/assist_output_sink.hpp"
#include "measurement_api/frontend.hpp"
#include "measurement_api/providers.hpp"

namespace {

class FakeCameraFrameProvider final : public uw::measurement_api::CameraFrameProvider {
 public:
  explicit FakeCameraFrameProvider(uw::domain::ImageFrame frame) : frame_(std::move(frame)) {}

  std::optional<uw::domain::ImageFrame> PollImageFrame() override {
    if (!frame_.has_value()) return std::nullopt;
    auto result = std::move(frame_);
    frame_.reset();
    return result;
  }

  uw::domain::HealthReport Health() const override { return {}; }

 private:
  std::optional<uw::domain::ImageFrame> frame_;
};

class FakeMetricOpticalFrontend final : public uw::measurement_api::OpticalDepthFrontend {
 public:
  std::optional<uw::domain::MeasurementEvidence> Process(
      const uw::measurement_api::CameraFrameBundle& bundle,
      const uw::domain::RigCalibrationSnapshot&) override {
    uw::domain::OpticalDepthPriorMeasurement prior;
    *prior.mutable_reference_camera_frame() = bundle.primary.header().sensor_frame();
    prior.set_width(1);
    prior.set_height(1);
    prior.add_depth_m(bundle.secondary.has_value() ? 2.0f : 3.0f);
    prior.add_variance_m2(0.01f);
    prior.set_valid_mask(std::string{"\x01", 1});
    prior.set_scale_status(uw::domain::OPTICAL_DEPTH_SCALE_STATUS_METRIC);
    prior.set_producer_type(bundle.secondary.has_value() ? "fake_stereo" : "fake_monocular_metric");
    uw::domain::EvidenceId id;
    id.set_value("fake_optical_depth");
    return uw::domain::MakeEvidence(id, {}, prior, 1.0, "fake_metric_v1");
  }

  uw::domain::HealthReport Health() const override { return {}; }
};

class FakeAssistOutputSink final : public uw::application::AssistOutputSink {
 public:
  void Publish(const uw::domain::OperatorAssistState& state) override { published_ = state; }

  const std::optional<uw::domain::OperatorAssistState>& Published() const { return published_; }

 private:
  std::optional<uw::domain::OperatorAssistState> published_;
};

}  // namespace

TEST(MeasurementApiContract, CameraProviderPollsCanonicalImageFrame) {
  uw::domain::ImageFrame frame;
  frame.mutable_header()->mutable_observation_id()->set_value("camera_1");
  FakeCameraFrameProvider provider(frame);
  ASSERT_TRUE(provider.PollImageFrame().has_value());
  EXPECT_FALSE(provider.PollImageFrame().has_value());
}

TEST(MeasurementApiContract, OpticalFrontendDoesNotRequireStereoAtInterfaceBoundary) {
  uw::measurement_api::CameraFrameBundle bundle;
  bundle.primary.mutable_header()->mutable_sensor_frame()->set_value("camera_left_link");
  FakeMetricOpticalFrontend frontend;
  const auto evidence = frontend.Process(bundle, {});
  ASSERT_TRUE(evidence.has_value());
  const auto& prior =
      uw::domain::GetPayload<uw::domain::OpticalDepthPriorMeasurement>(*evidence);
  EXPECT_EQ(prior.producer_type(), "fake_monocular_metric");
  EXPECT_EQ(prior.scale_status(), uw::domain::OPTICAL_DEPTH_SCALE_STATUS_METRIC);
}

TEST(MeasurementApiContract, AssistOutputSinkPublishesCompleteCanonicalState) {
  uw::domain::OperatorAssistState state;
  auto* track = state.mutable_target_tracks()->add_tracks();
  track->mutable_track_id()->set_value("track_9");
  track->set_class_label("aquaculture_zone");
  track->set_class_confidence(0.9);
  track->set_bearing_rad(0.2);
  track->set_range_m(4.0);
  track->set_status(uw::domain::TARGET_TRACK_STATUS_CONFIRMED);
  state.mutable_target_tracks()->mutable_publish_time()->set_seconds(12);
  state.set_has_path_lateral_offset(true);
  state.set_path_lateral_offset_m(0.3);
  state.set_path_offset_sigma_m(0.05);
  state.mutable_system_health()->set_status(uw::domain::HealthReport::STATUS_HEALTHY);
  state.set_data_age_ms(8.0);
  state.set_guidance_valid(true);
  state.set_degradation_reason("");
  state.add_sensor_health()->set_component_id("camera_left");

  FakeAssistOutputSink sink;
  sink.Publish(state);

  ASSERT_TRUE(sink.Published().has_value());
  ASSERT_EQ(sink.Published()->target_tracks().tracks_size(), 1);
  EXPECT_EQ(sink.Published()->target_tracks().tracks(0).track_id().value(), "track_9");
  EXPECT_TRUE(sink.Published()->has_path_lateral_offset());
  EXPECT_DOUBLE_EQ(sink.Published()->path_lateral_offset_m(), 0.3);
  EXPECT_EQ(sink.Published()->system_health().status(), uw::domain::HealthReport::STATUS_HEALTHY);
  EXPECT_DOUBLE_EQ(sink.Published()->data_age_ms(), 8.0);
  EXPECT_TRUE(sink.Published()->guidance_valid());
  EXPECT_EQ(sink.Published()->sensor_health_size(), 1);
}
