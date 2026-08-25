#include "adapters/holoocean_live_conversion.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace uw::adapters {
namespace {

void PopulateHeader(uw::domain::ObservationHeader* header, const std::string& sensor_id,
                     const std::string& sensor_frame, uint64_t sequence_id,
                     const std::string& calibration_version, int64_t capture_ns,
                     int64_t receive_monotonic_ns) {
  header->mutable_observation_id()->set_value(sensor_id + "-" + std::to_string(sequence_id));
  header->mutable_sensor_id()->set_value(sensor_id);
  header->mutable_sensor_frame()->set_value(sensor_frame);
  header->mutable_sequence_id()->set_value(sequence_id);
  header->mutable_capture_time()->set_seconds(capture_ns / 1'000'000'000);
  header->mutable_capture_time()->set_nanos(static_cast<int32_t>(capture_ns % 1'000'000'000));
  header->mutable_receive_time()->set_seconds(receive_monotonic_ns / 1'000'000'000);
  header->mutable_receive_time()->set_nanos(
      static_cast<int32_t>(receive_monotonic_ns % 1'000'000'000));
  // capture_time is HoloOcean simulation time (see RawHoloImage/RawHoloSonar/
  // RawHoloVehicleState doc comments), matching
  // HoloOceanRosBridgeSonarFrameProvider's own established choice -- NOT
  // CLOCK_DOMAIN_SYSTEM_REALTIME, which apps/online_assist_smoke.cpp uses
  // only because its synthetic fixtures are genuinely wall-clock-timed.
  header->set_clock_domain(uw::domain::CLOCK_DOMAIN_SIMULATION);
  header->set_receive_clock_domain(uw::domain::CLOCK_DOMAIN_SYSTEM_MONOTONIC);
  header->mutable_calibration_version()->set_value(calibration_version);
  header->set_validity(uw::domain::ObservationHeader::VALIDITY_OK);
  header->set_provenance("holoocean_realtime_node");
}

}  // namespace

uw::domain::ImageFrame ConvertHoloImage(const RawHoloImage& raw, const std::string& sensor_id,
                                         const std::string& sensor_frame, uint64_t sequence_id,
                                         const std::string& calibration_version,
                                         int64_t receive_monotonic_ns) {
  uw::domain::ImageFrame frame;
  PopulateHeader(frame.mutable_header(), sensor_id, sensor_frame, sequence_id,
                 calibration_version, raw.capture_ns, receive_monotonic_ns);
  frame.set_width(raw.width);
  frame.set_height(raw.height);
  frame.set_row_stride_bytes(raw.width * 3);
  frame.set_encoding(uw::domain::ImageFrame::IMAGE_ENCODING_RGB8);
  frame.set_pixel_data(reinterpret_cast<const char*>(raw.rgb_pixel_data.data()),
                       raw.rgb_pixel_data.size());
  frame.set_is_rectified(false);
  return frame;
}

uw::domain::SonarFrame ConvertHoloSonar(const RawHoloSonar& raw,
                                         const HoloOceanSonarCalibration& calibration,
                                         uint64_t sequence_id, int64_t receive_monotonic_ns) {
  uw::domain::SonarFrame frame;
  PopulateHeader(frame.mutable_header(), calibration.sensor_id, calibration.sensor_frame,
                 sequence_id, calibration.calibration_version, raw.capture_ns,
                 receive_monotonic_ns);

  const uint32_t num_ranges = raw.num_ranges;
  const uint32_t num_beams = raw.num_beams;
  frame.set_num_ranges(num_ranges);
  frame.set_num_beams(num_beams);
  frame.set_min_range(calibration.min_range_m);
  frame.set_max_range(calibration.max_range_m);
  const float range_resolution = num_ranges > 0
                                      ? (calibration.max_range_m - calibration.min_range_m) /
                                            static_cast<float>(num_ranges)
                                      : 0.0f;
  frame.set_range_resolution(range_resolution);
  frame.set_horizontal_fov(calibration.horizontal_fov_rad);
  frame.set_elevation_aperture(calibration.elevation_aperture_rad);
  frame.set_operating_frequency_hz(calibration.operating_frequency_hz);
  frame.mutable_gain_metadata()->set_gain(calibration.gain_db);
  frame.mutable_gain_metadata()->set_mode(calibration.gain_mode);
  frame.mutable_sound_speed_assumption()->set_speed_of_sound_mps(calibration.sound_speed_mps);
  frame.mutable_sound_speed_assumption()->set_salinity_ppt(calibration.salinity_ppt);
  frame.mutable_sound_speed_assumption()->set_is_measured(calibration.sound_speed_is_measured);

  for (uint32_t r = 0; r <= num_ranges; ++r) {
    frame.add_range_bins(calibration.min_range_m + static_cast<float>(r) * range_resolution);
  }
  // Ascending bearing, -fov/2 .. +fov/2 -- see the header comment for why
  // the intensity row below is mirrored to line up with this ordering (same
  // correction as HoloOceanRosBridgeSonarFrameProvider::PushImagingSonar).
  const float half_fov = calibration.horizontal_fov_rad / 2.0f;
  for (uint32_t c = 0; c < num_beams; ++c) {
    const float t = num_beams > 1 ? static_cast<float>(c) / static_cast<float>(num_beams - 1) : 0.0f;
    frame.add_azimuth_angles(-half_fov + calibration.horizontal_fov_rad * t);
  }

  std::string bytes(static_cast<std::size_t>(num_ranges) * num_beams, '\0');
  if (raw.image_range.size() == static_cast<std::size_t>(num_ranges) * num_beams) {
    for (uint32_t r = 0; r < num_ranges; ++r) {
      const float* row_in = raw.image_range.data() + static_cast<std::size_t>(r) * num_beams;
      char* row_out = bytes.data() + static_cast<std::size_t>(r) * num_beams;
      for (uint32_t c = 0; c < num_beams; ++c) {
        const float clamped = std::clamp(row_in[num_beams - 1 - c], 0.0f, 1.0f);
        row_out[c] = static_cast<char>(static_cast<unsigned char>(std::lround(clamped * 255.0f)));
      }
    }
  }
  frame.set_intensity_tensor(std::move(bytes));
  frame.set_encoding(uw::domain::SonarFrame::ENCODING_UINT8_GRAY);
  return frame;
}

uw::domain::VehicleState ConvertHoloVehicleState(const RawHoloVehicleState& raw,
                                                  const std::string& sensor_id,
                                                  const std::string& sensor_frame,
                                                  uint64_t sequence_id,
                                                  const std::string& calibration_version,
                                                  int64_t receive_monotonic_ns) {
  uw::domain::VehicleState state;
  PopulateHeader(state.mutable_header(), sensor_id, sensor_frame, sequence_id,
                 calibration_version, raw.capture_ns, receive_monotonic_ns);
  for (double v : raw.orientation_xyzw) state.add_orientation_xyzw(v);
  for (double v : raw.angular_velocity_radps) state.add_angular_velocity_radps(v);
  // Positive-down convention (see RawHoloVehicleState::raw_position_z_m doc
  // comment) -- negate HoloOcean's raw world-frame z.
  state.set_depth_m(-raw.raw_position_z_m);
  state.set_attitude_valid(true);
  state.set_depth_valid(true);
  state.set_device_health_valid(false);
  return state;
}

}  // namespace uw::adapters
