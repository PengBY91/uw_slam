#include "uw/frontends/stereo_optical_depth_frontend.hpp"

#include <cmath>

namespace uw::frontends {

StereoOpticalDepthFrontend::StereoOpticalDepthFrontend(StereoOpticalDepthFrontendParams params)
    : params_(params), matcher_(params.matcher) {}

std::optional<uw::domain::MeasurementEvidence> StereoOpticalDepthFrontend::Process(
    const uw::measurement_api::CameraFrameBundle& bundle,
    const uw::domain::RigCalibrationSnapshot& rig) {
  ++frames_processed_;
  if (!bundle.secondary.has_value()) {
    ++frames_rejected_;
    return std::nullopt;
  }

  const auto geometry = uw::sensor_models::StereoGeometry::Resolve(
      rig, params_.left_sensor_id, params_.left_frame, params_.right_sensor_id, params_.right_frame);
  if (!geometry.valid) {
    ++frames_rejected_;
    return std::nullopt;
  }

  const auto& left_image = bundle.primary;
  const auto& right_image = *bundle.secondary;
  if (left_image.encoding() != uw::domain::ImageFrame::IMAGE_ENCODING_MONO8 ||
      right_image.encoding() != uw::domain::ImageFrame::IMAGE_ENCODING_MONO8 ||
      left_image.width() != right_image.width() || left_image.height() != right_image.height()) {
    ++frames_rejected_;
    return std::nullopt;
  }

  const auto disparity = matcher_.Compute(
      reinterpret_cast<const uint8_t*>(left_image.pixel_data().data()),
      reinterpret_cast<const uint8_t*>(right_image.pixel_data().data()), left_image.width(),
      left_image.height(), left_image.row_stride_bytes());

  uw::domain::OpticalDepthPriorMeasurement prior;
  *prior.mutable_reference_camera_frame() = left_image.header().sensor_frame();
  prior.set_width(left_image.width());
  prior.set_height(left_image.height());
  prior.set_scale_status(uw::domain::OPTICAL_DEPTH_SCALE_STATUS_METRIC);
  prior.set_producer_type("stereo");

  const std::size_t pixels = static_cast<std::size_t>(left_image.width()) * left_image.height();
  std::string valid_mask(pixels, '\0');
  for (std::size_t i = 0; i < pixels; ++i) {
    if (disparity.valid[i] == 0) {
      prior.add_depth_m(0.0f);
      prior.add_variance_m2(0.0f);
      continue;
    }
    const double depth_m = geometry.left.fx * geometry.baseline_m / disparity.disparity_px[i];
    const double variance_m2 =
        std::pow(depth_m * depth_m / (geometry.left.fx * geometry.baseline_m) * params_.disparity_sigma_px, 2);
    prior.add_depth_m(static_cast<float>(depth_m));
    prior.add_variance_m2(static_cast<float>(variance_m2));
    valid_mask[i] = 1;
  }
  prior.set_valid_mask(valid_mask);

  uw::domain::EvidenceId evidence_id;
  evidence_id.set_value("stereo_depth_" + std::to_string(next_evidence_id_++));
  std::vector<uw::domain::ObservationId> sources;
  if (left_image.header().has_observation_id()) sources.push_back(left_image.header().observation_id());
  if (right_image.header().has_observation_id()) sources.push_back(right_image.header().observation_id());

  return uw::domain::MakeEvidence(evidence_id, sources, prior, /*noise_scale=*/1.0,
                                  "stereo_depth_frontend_v1");
}

uw::domain::HealthReport StereoOpticalDepthFrontend::Health() const {
  uw::domain::HealthReport report;
  report.set_component_id("stereo_optical_depth_frontend");
  report.set_status(frames_processed_ > 0 && frames_rejected_ == frames_processed_
                        ? uw::domain::HealthReport::STATUS_SUSPECT
                        : uw::domain::HealthReport::STATUS_HEALTHY);
  return report;
}

}  // namespace uw::frontends
