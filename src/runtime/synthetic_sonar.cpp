#include "runtime/synthetic_sonar.hpp"

#include <cmath>
#include <cstddef>
#include <utility>

namespace uw::runtime {

SyntheticSonarFrameResult RenderSyntheticSonarFrame(
    const SyntheticSonarFrameSpec& spec, double target_range_m,
    double target_bearing_rad) {
  SyntheticSonarFrameResult result;
  auto& frame = result.frame;
  auto& header = *frame.mutable_header();
  header.mutable_observation_id()->set_value(spec.observation_id);
  header.mutable_sensor_id()->set_value(spec.sensor_id);
  header.mutable_sensor_frame()->set_value(spec.sensor_frame);
  header.mutable_capture_time()->set_seconds(
      static_cast<int64_t>(spec.timestamp_ns / 1'000'000'000ULL));
  header.mutable_capture_time()->set_nanos(
      static_cast<int32_t>(spec.timestamp_ns % 1'000'000'000ULL));
  header.set_clock_domain(uw::domain::CLOCK_DOMAIN_SIMULATION);
  header.set_validity(uw::domain::ObservationHeader::VALIDITY_OK);
  header.set_provenance(spec.provenance);

  frame.set_num_ranges(spec.num_ranges);
  frame.set_num_beams(spec.num_beams);
  frame.set_min_range(static_cast<float>(spec.min_range_m));
  frame.set_max_range(static_cast<float>(spec.max_range_m));
  const double range_resolution =
      (spec.max_range_m - spec.min_range_m) / spec.num_ranges;
  frame.set_range_resolution(static_cast<float>(range_resolution));
  frame.set_horizontal_fov(static_cast<float>(spec.horizontal_fov_rad));

  for (uint32_t row = 0; row <= spec.num_ranges; ++row) {
    frame.add_range_bins(
        static_cast<float>(spec.min_range_m + row * range_resolution));
  }
  const double half_fov = spec.horizontal_fov_rad / 2.0;
  for (uint32_t beam = 0; beam < spec.num_beams; ++beam) {
    const double t = spec.num_beams > 1
                         ? static_cast<double>(beam) / (spec.num_beams - 1)
                         : 0.0;
    frame.add_azimuth_angles(
        static_cast<float>(-half_fov + spec.horizontal_fov_rad * t));
  }

  std::string intensities(
      static_cast<std::size_t>(spec.num_ranges) * spec.num_beams,
      static_cast<char>(spec.background_intensity));
  const int target_row = static_cast<int>(
      std::lround((target_range_m - spec.min_range_m) / range_resolution));
  const int target_beam = static_cast<int>(std::lround(
      (target_bearing_rad + half_fov) / spec.horizontal_fov_rad *
      (spec.num_beams - 1)));
  if (target_row >= 0 && target_row < static_cast<int>(spec.num_ranges) &&
      target_beam >= 0 && target_beam < static_cast<int>(spec.num_beams)) {
    for (int offset = -spec.target_half_width_beams;
         offset <= spec.target_half_width_beams; ++offset) {
      const int beam = target_beam + offset;
      if (beam < 0 || beam >= static_cast<int>(spec.num_beams)) continue;
      intensities[static_cast<std::size_t>(target_row) * spec.num_beams +
                  static_cast<std::size_t>(beam)] =
          static_cast<char>(spec.target_intensity);
    }
    result.target_rendered = true;
  }
  frame.set_intensity_tensor(std::move(intensities));
  frame.set_encoding(uw::domain::SonarFrame::ENCODING_UINT8_GRAY);
  return result;
}

}  // namespace uw::runtime
