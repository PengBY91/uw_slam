#include "uw/adapters/holoocean_ros_bridge_sonar_frame_provider.hpp"

#include <algorithm>
#include <cmath>

namespace uw::adapters {

HoloOceanRosBridgeSonarFrameProvider::HoloOceanRosBridgeSonarFrameProvider(
    HoloOceanImagingSonarParams params)
    : params_(std::move(params)) {}

void HoloOceanRosBridgeSonarFrameProvider::PushImagingSonar(int64_t timestamp_ns,
                                                              uint32_t num_ranges,
                                                              uint32_t num_beams,
                                                              const std::vector<float>& image_range,
                                                              std::string observation_id) {
  if (num_ranges == 0 || num_beams == 0 ||
      image_range.size() != static_cast<std::size_t>(num_ranges) * num_beams) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++total_dropped_;
    return;
  }

  uw::domain::SonarFrame frame;
  auto& header = *frame.mutable_header();
  header.mutable_observation_id()->set_value(std::move(observation_id));
  header.mutable_sensor_id()->set_value(params_.sensor_id);
  header.mutable_sensor_frame()->set_value(params_.sensor_frame);
  header.mutable_capture_time()->set_seconds(timestamp_ns / 1'000'000'000);
  header.mutable_capture_time()->set_nanos(static_cast<int32_t>(timestamp_ns % 1'000'000'000));
  header.set_clock_domain(uw::domain::CLOCK_DOMAIN_SIMULATION);
  header.set_validity(uw::domain::ObservationHeader::VALIDITY_OK);
  header.set_provenance("adapters/third_party/holoocean_ros_bridge");

  frame.set_num_ranges(num_ranges);
  frame.set_num_beams(num_beams);
  frame.set_min_range(params_.min_range_m);
  frame.set_max_range(params_.max_range_m);
  const float range_resolution =
      (params_.max_range_m - params_.min_range_m) / static_cast<float>(num_ranges);
  frame.set_range_resolution(range_resolution);
  frame.set_horizontal_fov(params_.horizontal_fov_rad);

  for (uint32_t r = 0; r <= num_ranges; ++r) {
    frame.add_range_bins(params_.min_range_m + static_cast<float>(r) * range_resolution);
  }
  // Ascending bearing, -fov/2 .. +fov/2 — see the header comment for why the
  // intensity row below is mirrored to line up with this ordering.
  const float half_fov = params_.horizontal_fov_rad / 2.0f;
  for (uint32_t c = 0; c < num_beams; ++c) {
    const float t = num_beams > 1 ? static_cast<float>(c) / static_cast<float>(num_beams - 1) : 0.0f;
    frame.add_azimuth_angles(-half_fov + params_.horizontal_fov_rad * t);
  }

  std::string bytes(static_cast<std::size_t>(num_ranges) * num_beams, '\0');
  for (uint32_t r = 0; r < num_ranges; ++r) {
    const float* row_in = image_range.data() + static_cast<std::size_t>(r) * num_beams;
    char* row_out = bytes.data() + static_cast<std::size_t>(r) * num_beams;
    for (uint32_t c = 0; c < num_beams; ++c) {
      const float clamped = std::clamp(row_in[num_beams - 1 - c], 0.0f, 1.0f);
      row_out[c] = static_cast<char>(static_cast<unsigned char>(std::lround(clamped * 255.0f)));
    }
  }
  frame.set_intensity_tensor(std::move(bytes));
  frame.set_encoding(uw::domain::SonarFrame::ENCODING_UINT8_GRAY);

  std::lock_guard<std::mutex> lock(mutex_);
  pending_.push_back(std::move(frame));
  ++total_pushed_;
}

std::optional<uw::domain::SonarFrame> HoloOceanRosBridgeSonarFrameProvider::PollSonarFrame() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (pending_.empty()) return std::nullopt;
  auto frame = std::move(pending_.front());
  pending_.pop_front();
  ++total_polled_;
  return frame;
}

uw::domain::HealthReport HoloOceanRosBridgeSonarFrameProvider::Health() const {
  std::lock_guard<std::mutex> lock(mutex_);
  uw::domain::HealthReport report;
  report.set_component_id("holoocean_ros_bridge_sonar_frame_provider");
  report.set_status(total_pushed_ == 0 ? uw::domain::HealthReport::STATUS_UNSPECIFIED
                                        : uw::domain::HealthReport::STATUS_HEALTHY);
  report.set_queue_depth(static_cast<uint32_t>(pending_.size()));
  report.set_dropped_frame_count(total_dropped_);
  return report;
}

}  // namespace uw::adapters
